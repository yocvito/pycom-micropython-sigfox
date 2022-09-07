/*
 * Copyright (c) 2021, Pycom Limited.
 *
 * This software is licensed under the GNU GPL version 3 or any
 * later version, with permitted additional terms. For more information
 * see the Pycom Licence v1.0 document supplied with this file, or
 * available at https://www.pycom.io/opensource/licensing
 */

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "py/mpconfig.h"
#include "py/mpstate.h"
#include "py/obj.h"
#include "py/nlr.h"
#include "py/runtime.h"
#include "py/mphal.h"
#include "py/stream.h"
#include "py/mperrno.h"
#include "bufhelper.h"
#include "mpexception.h"
#include "radio.h"
#include "modnetwork.h"
#include "py/stream.h"
#include "modusocket.h"
#include "pycom_config.h"
#include "mpirq.h"
#include "modlora.h"

#include "esp_heap_caps.h"
#include "sdkconfig.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "lwip/sockets.h"       // for the socket error codes

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "lora/mac/LoRaMacTest.h"
#include "lora/mac/region/Region.h"
#include "lora/mac/region/RegionAS923.h"
#include "lora/mac/region/RegionAU915.h"
#include "lora/mac/region/RegionUS915.h"
#include "lora/mac/region/RegionUS915-Hybrid.h"
#include "lora/mac/region/RegionEU868.h"
#include "lora/mac/region/RegionCN470.h"
#include "lora/mac/region/RegionIN865.h"
#include "lora/mac/region/RegionEU433.h"

// openThread includes
#ifdef LORA_OPENTHREAD_ENABLED
#include "lora/ot-log.h"
#include "modmesh.h"
#endif  // #ifdef LORA_OPENTHREAD_ENABLED

/*!
 *  PHYSEC includes
 */
#ifdef PHYSEC
#include <math.h>

#include "extmod/vfs_posix.h"
#include <fcntl.h>

#include "extmod/crypto-algorithms/sha256.c"
#include "lora/system/crypto/cmac.h"
#include "extmod/uzlib/uzlib.h"

#include "machwdt.h"
#include "str_utils.h"

#include "modlora_physec_rec_mat.h"
#endif

#include "random.h"
/******************************************************************************
 DEFINE PRIVATE CONSTANTS
 ******************************************************************************/
#define TX_OUTPUT_POWER_MAX                         20          // dBm
#define TX_OUTPUT_POWER_MIN                         2           // dBm

#define LORA_FIX_LENGTH_PAYLOAD_ON                  (true)
#define LORA_FIX_LENGTH_PAYLOAD_OFF                 (false)
#define LORA_TX_TIMEOUT_MAX                         (9000)      // 9 seconds
#define LORA_RX_TIMEOUT                             (0)         // No timeout

// [SF6..SF12]
#define LORA_SPREADING_FACTOR_MIN                   (6)
#define LORA_SPREADING_FACTOR_MAX                   (12)

#define LORA_CHECK_SOCKET(s)                        if (s->sock_base.u.sd < 0) {  \
                                                        *_errno = MP_EBADF;     \
                                                        return -1;              \
                                                    }

#define OVER_THE_AIR_ACTIVATION_DUTYCYCLE           10000  // 10 [s] value in ms

#define DEF_LORAWAN_NETWORK_ID                      0
#define DEF_LORAWAN_APP_PORT                        2

#define LORA_JOIN_WAIT_MS                           (50)

#define LORAWAN_SOCKET_GET_FD(sd)                   (sd & 0xFF)

#define LORAWAN_SOCKET_IS_CONFIRMED(sd)             ((sd & 0x40000000) == 0x40000000)
#define LORAWAN_SOCKET_SET_CONFIRMED(sd)            (sd |= 0x40000000)
#define LORAWAN_SOCKET_CLR_CONFIRMED(sd)            (sd &= ~0x40000000)

#define LORAWAN_SOCKET_SET_PORT(sd, port)           (sd &= 0xFFFF00FF); \
                                                    (sd |= (port << 8))

#define LORAWAN_SOCKET_GET_PORT(sd)                 ((sd >> 8) & 0xFF)


#define LORAWAN_SOCKET_SET_DR(sd, dr)               (sd &= 0xFF00FFFF); \
                                                    (sd |= (dr << 16))

#define LORAWAN_SOCKET_GET_DR(sd)                   ((sd >> 16) & 0xFF)


// callback events
#define MODLORA_RX_EVENT                            (0x01)
#define MODLORA_TX_EVENT                            (0x02)
#define MODLORA_TX_FAILED_EVENT                     (0x04)

#define MODLORA_NVS_NAMESPACE                       "LORA_NVM"

#define MESH_CLI_OUTPUT_SIZE                            (1024)


// PHYSEC

/******************************************************************************
 DEFINE PRIVATE TYPES
 ******************************************************************************/
typedef enum {
    E_LORA_STACK_MODE_LORA = 0,
    E_LORA_STACK_MODE_LORAWAN
    #ifdef PHYSEC
    ,
    E_LORA_STACK_MODE_LORAPHYSEC
    #endif
} lora_stack_mode_t;

typedef enum {
    E_LORA_STATE_NOINIT = 0,
    E_LORA_STATE_IDLE,
    E_LORA_STATE_JOIN,
    E_LORA_STATE_LINK_CHECK,
    E_LORA_STATE_RX,
    E_LORA_STATE_RX_DONE,
    E_LORA_STATE_RX_TIMEOUT,
    E_LORA_STATE_RX_ERROR,
    E_LORA_STATE_TX,
    E_LORA_STATE_TX_DONE,
    E_LORA_STATE_TX_TIMEOUT,
    E_LORA_STATE_SLEEP,
    E_LORA_STATE_RESET
} lora_state_t;

typedef enum {
    E_LORA_MODE_ALWAYS_ON = 0,
    E_LORA_MODE_TX_ONLY,
    E_LORA_MODE_SLEEP
} lora_mode_t;

typedef enum {
    E_LORA_BW_125_KHZ = 0,
    E_LORA_BW_250_KHZ = 1,
    E_LORA_BW_500_KHZ = 2
} lora_bandwidth_t;

typedef enum {
    E_LORA_CODING_4_5 = 1,
    E_LORA_CODING_4_6 = 2,
    E_LORA_CODING_4_7 = 3,
    E_LORA_CODING_4_8 = 4
} lora_coding_rate_t;

typedef enum {
    E_LORA_ACTIVATION_OTAA = 0,
    E_LORA_ACTIVATION_ABP
} lora_activation_t;

#ifndef PHYSEC_DEBUG
#define PHYSEC_DEBUG    0
#endif

#define PHYSEC
#ifdef PHYSEC
/*!
 * PHYSEC DEFINITIONS BEGIN
 */

// PHYSEC defines


// -- DEBUG 
// not so good, don't put declaration in here
#define physec_verb_statement(verb, code) do { if (PHYSEC_DEBUG >= verb) { code; } } while(0)
// physec logging maccros (takes a verbosity level and a message to displays)
#define physec_log(verb, fmt)      do { if (PHYSEC_DEBUG >= verb) printf(fmt); } while(0)
#define physec_log2(verb, fmt, ...)      do { if (PHYSEC_DEBUG >= verb) printf(fmt, __VA_ARGS__);  } while(0)
#define physec_hexdump(verb, buf, siz)      do { if (PHYSEC_DEBUG >= verb) hexdump((uint8_t*)buf, siz);  } while(0)

//#define LOGFILE         "/sd/PHYSEC.log"


// Device ID length for KEY GENERATION
#define PHYSEC_KEYGEN_FREQUENCY     867900000
#define PHYSEC_PROBE_TIMEOUT        500         // 500 ms
#define PHYSEC_SYNC_WORD            0x67
#define PHYSEC_COHERENCE_TIME       25        // 0.423 / ((v/c) * f)

// -- Packets defines
// -- -- Headers
#define PHYSEC_KG_PKT_IDENTIFIER    (uint16_t) 0xe4b9
#define PHYSEC_DT_PKT_IDENTIFIER    (uint16_t) 0xe5b9

// -- -- sizes and limits
// -- -- -- Measures
#define PHYSEC_N_MAX_MEASURE                65536
#define PHYSEC_N_REQUIRED_MEASURE           100
#define PHYSEC_N_NEW_REQUIRED_MEASURE       100

// 128 bits = 16 bytes = a 16-char-table.
// For now, if this value is changed, the code will not going to adapt.
#define PHYSEC_KEY_SIZE             128
// future update: need to handle bits key size which are not byte aligned
#define PHYSEC_KEY_SIZE_BYTES       (int16_t) (PHYSEC_KEY_SIZE / 8)

// Channel Sampling
#define PHYSEC_PROBE_FREQUENCY  4.0 // probe per second
#define PHYSEC_PROBE_PERIOD    1.0/PHYSEC_PROBE_FREQUENCY * 1000.0 // ms

// Quantization
#define PHYSEC_DATA_TO_BAND_RATIO   0.1

// PHYSEC_Packet defines
#define PHYSEC_DEV_ID_LEN           4
#define PHYSEC_CS_COMPRESSED_SIZE   35
#define PHYSEC_CS_MAC_SIZE          16

//#define PHYSEC_KEYGEN_PAYLOAD_SIZE  (PHYSEC_CS_COMPRESSED_SIZE+PHYSEC_CS_MAC_SIZE) // uncomment, and comment line below to activate real reconcil (when ready)
#define PHYSEC_KEYGEN_PAYLOAD_SIZE  (PHYSEC_KEY_SIZE_BYTES+PHYSEC_CS_MAC_SIZE)
#define PHYSEC_KG_PKT_HEADER_SIZE      (2 + 1 + PHYSEC_DEV_ID_LEN)

#define PHYSEC_MAX_PAYLOAD_SIZE     PHYSEC_KEYGEN_PAYLOAD_SIZE
// should be equal to sizeof(PHYSEC_packet)
#define PHYSEC_MAX_PKT_SIZE         (3+PHYSEC_MAX_PAYLOAD_SIZE)

// PHYSEC_DataPkt defines
#define PHYSEC_DATA_HDR_SIZE        6
#define PHYSEC_DATA_PKT_MAX_SIZE    (LORA_PAYLOAD_SIZE_MAX-PHYSEC_DATA_HDR_SIZE)

/*!
 *  PHYSEC data structures
 */

// Key structure
typedef struct _PHYSEC_Key {
    uint8_t key[PHYSEC_KEY_SIZE_BYTES];
} PHYSEC_Key;

// -- Measurements structures

// Measurement used for KeyGen
typedef enum _PHYSEC_MeasureType {
    PHYSEC_MT_NONE = 0,
    PHYSEC_MT_RSSI,
    PHYSEC_MT_SNR,
} PHYSEC_MeasureType;

// the more general structure (need to be used instead of PHYSEC_RssiMsrmts)
typedef struct _PHYSEC_Measures {
    uint16_t nb_val;
    int8_t *values;
    float delay;
} PHYSEC_Measures;

// -- Packet structures

// -- -- Key Generation protocol packet structures

// Synchronization structure, containing the IDs of both
// Alice and Bob. This id is used to find lora frames sent
// by the other device
typedef struct _PHYSEC_Sync {
    uint8_t dev_id[PHYSEC_DEV_ID_LEN];
    uint8_t rmt_dev_id[PHYSEC_DEV_ID_LEN];
} PHYSEC_Sync;

// Packet type enum
typedef enum _PHYSEC_packet_type {
    // type for non PHYSEC frames or to handle errors
    PHYSEC_PT_NONE = 0,
    // type for reconciliation packets
    PHYSEC_PT_KEYGEN = 1,
    // type for measurements (probing) packets
    PHYSEC_PT_PROBE = 2,
    // type for reset packet (leave out current measures)
    PHYSEC_PT_RESET = 4,
    PHYSEC_PT_MAX = 255
} PHYSEC_PacketType;

// Packet structure, this is the most low-level frame (PHY) encapsulating
// all the other ones
typedef struct _PHYSEC_packet {
    // allows to recognize PHYSEC packets
    uint16_t identifier;
    // specify packet type
    uint8_t type;
    // dest device id
    uint8_t dev_id[PHYSEC_DEV_ID_LEN];
    // packet data
    uint8_t payload[PHYSEC_MAX_PAYLOAD_SIZE];
} __attribute__((__packed__)) PHYSEC_Packet;

// Probe frame
typedef struct _PHYSEC_probe {
    uint16_t cnt;
} __attribute__((packed)) PHYSEC_Probe;

// Reconciliation frame
typedef struct _PHYSEC_keygen {
    // reconciliation vector (here a compressed vector of Bob key)
    //uint8_t cs_vec[PHYSEC_CS_COMPRESSED_SIZE];
    PHYSEC_Key K; // for using the protocol without real reconciliation
    
    // the checksum of the packet encrypted with the Bob key
    // (actually AES_CMAC, could be reduced like lorawan MIC)
    uint8_t MAC[PHYSEC_CS_MAC_SIZE];
} __attribute__((__packed__)) PHYSEC_KeyGen;

// -- -- Data ciphering packet structures
// type of ciphering
typedef enum _PHYSEC_DataCipher {
    PHYSEC_DT_NONE = -1,
    // the whole payload is ciphered
    PHYSEC_DT_FULL = 0,     // 00
    // only a part of the payload is ciphered (specified by (boff, eoff) )
    PHYSEC_DT_PARTIAL = 1   // 01
} PHYSEC_DataCipher;


// Data control structure
typedef struct _PHYSEC_DataCtrl {
    // type of encryption (full or partial)
    uint16_t type:2;
    // begin offset of encryption
    uint16_t boff:8;
    // end offset of encryption
    uint16_t eoff:8;
    // the size of the packet
    uint16_t size:8;
    uint16_t padd:6;
} __attribute__((__packed__)) PHYSEC_DataCtrl;

// Data packet structure
typedef struct _PHYSEC_DataPkt {
    uint16_t identifier;
    PHYSEC_DataCtrl ctrl;
    uint8_t data[PHYSEC_DATA_PKT_MAX_SIZE];
} __attribute__((__packed__)) PHYSEC_DataPkt;


// -- Various other structs

typedef struct _PHYSEC_KeyGenStats {
    float total;            // total duration
    float measure;          // measure duration
    float signal_process;   // signal processing duration
    float quantization;     // quantization duration
    float reconciliation;   // reconciliation duration
    float privacy_amp;      // privacy amplification duration
    float mismatch;         // mismatch percentage
    float entropy_pk;       // key entropy before privacy amplification
    float entropy_k;        // key entropy after PA
    uint16_t n_measures;      // nb measures
    uint8_t n_retries;        // nb reset by Bob
    uint32_t avg_delay;
} PHYSEC_KeyGenStats;

struct density {
    int8_t q_0;
    uint16_t bin_nbr;
    int8_t *bins;
    double *values;
};

// List of generated keys
struct peer_key{
    uint32_t peer_id;
    uint8_t key[16];
    struct peer_key *next;
};
typedef struct peer_key** peer_key_list_t;

static inline uint16_t toa(uint8_t sf);

static void reschdl_last_rx_pkt();

// PHYSEC DEFINITIONS END

#endif

typedef struct {
    mp_obj_base_t     base;
    mp_obj_t          handler;
    mp_obj_t          handler_arg;
    LoRaMacRegion_t   region;
    lora_stack_mode_t stack_mode;
    DeviceClass_t     device_class;
    lora_state_t      state;
    uint32_t          frequency;
    uint32_t          rx_timestamp;
    uint32_t          net_id;
    uint32_t          tx_time_on_air;
    uint32_t          tx_counter;
    uint32_t          tx_frequency;
    int16_t           rssi;
    int8_t            snr;
    uint8_t           sfrx;
    uint8_t           sftx;
    uint8_t           preamble;
    uint8_t           bandwidth;
    uint8_t           coding_rate;
    uint8_t           sf;
    int8_t            tx_power;
    uint8_t           pwr_mode;

    struct {
        bool Enabled;
        bool Running;
        uint8_t State;
        bool IsTxConfirmed;
        uint16_t DownLinkCounter;
        bool LinkCheck;
        uint8_t DemodMargin;
        uint8_t NbGateways;
    } ComplianceTest;

    uint8_t           activation;
    uint8_t           tx_retries;
    uint8_t           otaa_dr;

    union {
        struct {
            // for OTAA
            uint8_t           DevEui[8];
            uint8_t           AppEui[8];
            uint8_t           AppKey[16];
        } otaa;

        struct {
            // for ABP
            uint32_t          DevAddr;
            uint8_t           NwkSKey[16];
            uint8_t           AppSKey[16];
        } abp;
    } u;

    bool              txiq;
    bool              rxiq;
    bool              adr;
    bool              public;
    bool              joined;
    bool              reset;
    uint8_t           events;
    uint8_t           trigger;
    uint8_t           tx_trials;

    #ifdef PHYSEC

    uint32_t        physec_device_id;
    uint32_t        physec_remote_device_id;
    uint16_t        physec_timeout;
    bool            physec_initiator;

    struct peer_key *peer_key_list;

    #endif

} lora_obj_t;

typedef struct {
    uint32_t    index;
    uint32_t    size;
    uint8_t     data[LORA_PAYLOAD_SIZE_MAX];
    uint8_t     port;
} lora_partial_rx_packet_t;

/******************************************************************************
 DECLARE PRIVATE DATA
 ******************************************************************************/
static QueueHandle_t xCmdQueue;
static QueueHandle_t xRxQueue;
static QueueHandle_t xCbQueue;
static EventGroupHandle_t LoRaEvents;

static RadioEvents_t RadioEvents;
static lora_cmd_data_t task_cmd_data;
static LoRaMacPrimitives_t LoRaMacPrimitives;
static LoRaMacCallback_t LoRaMacCallbacks;

static lora_obj_t lora_obj;
static lora_partial_rx_packet_t lora_partial_rx_packet;
static lora_rx_data_t rx_data_isr;
#ifdef PHYSEC
//static bool bRxBuffered;
static lora_rx_data_t last_rx_pkt;
#endif
static TimerEvent_t TxNextActReqTimer;

static nvs_handle modlora_nvs_handle;
static const char *modlora_nvs_data_key[E_LORA_NVS_NUM_KEYS] = { "JOINED", "UPLNK", "DWLNK", "DEVADDR",
                                                                 "NWSKEY", "APPSKEY", "NETID", "ADRACK",
                                                                 "MACPARAMS", "CHANNELS", "SRVACK", "MACNXTTX",
                                                                 "MACBUFIDX", "MACRPTIDX", "MACBUF", "MACRPTBUF",
                                                                 "REGION", "CHANMASK", "CHANMASKREM" };
/******************************************************************************
 DECLARE PUBLIC DATA
 ******************************************************************************/
extern TaskHandle_t xLoRaTaskHndl;

/******************************************************************************
 DECLARE PRIVATE FUNCTIONS
 ******************************************************************************/
static void TASK_LoRa (void *pvParameters);
static void TASK_LoRa_Timer (void *pvParameters);
static void OnTxDone (void);
static void OnRxDone (uint8_t *payload, uint32_t timestamp, uint16_t size, int16_t rssi, int8_t snr, uint8_t sf);
static void OnTxTimeout (void);
static void OnRxTimeout (void);
static void OnRxError (void);
static void lora_radio_setup (lora_init_cmd_data_t *init_data);
static void lora_validate_mode (uint32_t mode);
static void lora_validate_frequency (uint32_t frequency);
static void lora_validate_power (uint8_t tx_power);
static void lora_validate_bandwidth (uint8_t bandwidth);
static void lora_validate_sf (uint8_t sf);
static void lora_validate_coding_rate (uint8_t coding_rate);
static void lora_set_config (lora_cmd_data_t *cmd_data);
static void lora_get_config (lora_cmd_data_t *cmd_data);
static void lora_send_cmd (lora_cmd_data_t *cmd_data);
static int32_t lora_send (const byte *buf, uint32_t len, uint32_t timeout_ms);
static int32_t lora_recv (byte *buf, uint32_t len, int32_t timeout_ms, uint32_t *port);
static bool lora_rx_any (void);
static bool lora_tx_space (void);
static void lora_callback_handler (void *arg);
static bool lorawan_nvs_open (void);

static int lora_socket_socket (mod_network_socket_obj_t *s, int *_errno);
static void lora_socket_close (mod_network_socket_obj_t *s);
static int lora_socket_send (mod_network_socket_obj_t *s, const byte *buf, mp_uint_t len, int *_errno);
static int lora_socket_recvfrom (mod_network_socket_obj_t *s, byte *buf, mp_uint_t len, byte *ip, mp_uint_t *port, int *_errno);
static int lora_socket_recv (mod_network_socket_obj_t *s, byte *buf, mp_uint_t len, int *_errno);
static int lora_socket_settimeout (mod_network_socket_obj_t *s, mp_int_t timeout_ms, int *_errno);
static int lora_socket_bind (mod_network_socket_obj_t *s, byte *ip, mp_uint_t port, int *_errno);
static int lora_socket_setsockopt (mod_network_socket_obj_t *s, mp_uint_t level, mp_uint_t opt, const void *optval, mp_uint_t optlen, int *_errno);
static int lora_socket_ioctl (mod_network_socket_obj_t *s, mp_uint_t request, mp_uint_t arg, int *_errno);
static int lora_socket_sendto (struct _mod_network_socket_obj_t *s, const byte *buf, mp_uint_t len, byte *ip, mp_uint_t port, int *_errno);

static bool lora_lbt_is_free(void);
STATIC mp_obj_t lora_nvram_erase (mp_obj_t self_in);

#ifdef PHYSEC
// List of generated keys
void peer_key_list_init(peer_key_list_t pkl);
void peer_key_list_free(peer_key_list_t pkl);
void peer_key_push(peer_key_list_t pkl, uint32_t peer_id, const uint8_t *key);
/*
return value:
    0  : peer_id not found, key_out = NULL.
    1   : peer_id founded and the key is copied in the key_out.
*/
char peer_key_list_get_key_by_peer_id(peer_key_list_t pkl, uint32_t peer_id, uint8_t *key_out);
void peer_key_delete_by_peer_id(peer_key_list_t pkl, uint32_t peer_id);
#endif

/******************************************************************************
 DECLARE PUBLIC DATA
 ******************************************************************************/
#if defined(FIPY) || defined(LOPY4)
SemaphoreHandle_t xLoRaSigfoxSem;
#endif

/******************************************************************************
 DEFINE PUBLIC FUNCTIONS
 ******************************************************************************/
void modlora_init0(void) {
    xCmdQueue = xQueueCreate(LORA_CMD_QUEUE_SIZE_MAX, sizeof(lora_cmd_data_t));
    xRxQueue = xQueueCreate(LORA_DATA_QUEUE_SIZE_MAX, sizeof(lora_rx_data_t));
    xCbQueue = xQueueCreate(LORA_CB_QUEUE_SIZE_MAX, sizeof(modlora_timerCallback));
    LoRaEvents = xEventGroupCreate();
#if defined(FIPY) || defined(LOPY4)
    xLoRaSigfoxSem = xSemaphoreCreateMutex();
#endif

    if (!lorawan_nvs_open()) {
        mp_printf(&mp_plat_print, "Error opening LoRa NVS namespace!\n");
    }

    // target board initialisation
    BoardInitMcu();
    BoardInitPeriph();

    xTaskCreatePinnedToCore(TASK_LoRa, "LoRa", LORA_STACK_SIZE / sizeof(StackType_t), NULL, LORA_TASK_PRIORITY, &xLoRaTaskHndl, 1);
    xTaskCreatePinnedToCore(TASK_LoRa_Timer, "LoRa_Timer_callback", LORA_TIMER_STACK_SIZE / sizeof(StackType_t), NULL, LORA_TIMER_TASK_PRIORITY, &xLoRaTimerTaskHndl, 1);
}

bool modlora_nvs_set_uint(uint32_t key_idx, uint32_t value) {
    if (ESP_OK == nvs_set_u32(modlora_nvs_handle, modlora_nvs_data_key[key_idx], value)) {
        return true;
    }
    return false;
}

bool modlora_nvs_set_blob(uint32_t key_idx, const void *value, uint32_t length) {
    if (ESP_OK == nvs_set_blob(modlora_nvs_handle, modlora_nvs_data_key[key_idx], value, length)) {
        return true;
    }
    return false;
}

bool modlora_nvs_get_uint(uint32_t key_idx, uint32_t *value) {
    esp_err_t err;
    if (ESP_OK == (err = nvs_get_u32(modlora_nvs_handle, modlora_nvs_data_key[key_idx], value))) {
        return true;
    }
    return false;
}

bool modlora_nvs_get_blob(uint32_t key_idx, void *value, uint32_t *length) {
    esp_err_t err;
    if (ESP_OK == (err = nvs_get_blob(modlora_nvs_handle, modlora_nvs_data_key[key_idx], value, length))) {
        return true;
    }
    return false;
}

void modlora_sleep_module(void)
{
    lora_cmd_data_t cmd_data;
    /* Set Modem mode to LORA in order to got to Sleep Mode */
    Radio.SetModem(MODEM_LORA);
    cmd_data.cmd = E_LORA_CMD_SLEEP;
    /* Send Sleep Command to Lora Task */
    lora_send_cmd (&cmd_data);
}

bool modlora_is_module_sleep(void)
{
    if (lora_obj.state == E_LORA_STATE_SLEEP)
    {
        return true;
    }
    else
    {
        return false;
    }
}

IRAM_ATTR void modlora_set_timer_callback(modlora_timerCallback cb)
{
    if(cb != NULL)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        xQueueSendFromISR(xCbQueue, &cb, &xHigherPriorityTaskWoken);

        if( xHigherPriorityTaskWoken)
        {
            portYIELD_FROM_ISR();
        }
    }
}

#ifdef LORA_OPENTHREAD_ENABLED
int lora_ot_recv(uint8_t *buf, int8_t *rssi) {

    // put Lora into RX mode
    int len = lora_recv (buf, OT_RADIO_FRAME_MAX_SIZE, 0, NULL);

    if (len > 0) {

        // put rssi on signed 8bit, saturate at -128dB
        if (lora_obj.rssi < INT8_MIN)
            *rssi = INT8_MIN;
        else
            *rssi = lora_obj.rssi;

        otPlatLog(OT_LOG_LEVEL_INFO, 0, "radio rcv: %d, %d", len, *rssi);
    }
    return len;
}

void lora_ot_send(const uint8_t *buf, uint16_t len) {

    // send max 255 bytes
    len = LORA_PAYLOAD_SIZE_MAX < len ? LORA_PAYLOAD_SIZE_MAX : len;

    lora_send(buf, len, 0);

    //otPlatLog(OT_LOG_LEVEL_INFO, 0, "radio TX: %d", len);
}
#endif  // #ifdef LORA_OPENTHREAD_ENABLED

/******************************************************************************
 DEFINE PRIVATE FUNCTIONS
 ******************************************************************************/

static bool lorawan_nvs_open (void) {
    if (nvs_open(MODLORA_NVS_NAMESPACE, NVS_READWRITE, &modlora_nvs_handle) != ESP_OK) {
        return false;
    }

    uint32_t data;
    if (ESP_ERR_NVS_NOT_FOUND == nvs_get_u32(modlora_nvs_handle, "JOINED", &data)) {
        // initialize the value to 0
        nvs_set_u32(modlora_nvs_handle, "JOINED", false);
        if (ESP_OK != nvs_commit(modlora_nvs_handle)) {
            return false;
        }
    }
    return true;
}

static int32_t lorawan_send (const byte *buf, uint32_t len, uint32_t timeout_ms, bool confirmed, uint32_t dr, uint32_t port) {
    lora_cmd_data_t cmd_data;

    cmd_data.cmd = E_LORA_CMD_LORAWAN_TX;
    memcpy (cmd_data.info.tx.data, buf, len);
    cmd_data.info.tx.len = len;
    cmd_data.info.tx.dr = dr;
    if (lora_obj.ComplianceTest.Enabled && lora_obj.ComplianceTest.Running) {
        cmd_data.info.tx.port = 224;  // MAC commands port
        if (lora_obj.ComplianceTest.IsTxConfirmed) {
            cmd_data.info.tx.confirmed = true;
        } else {
            cmd_data.info.tx.confirmed = false;
        }
    } else {
        cmd_data.info.tx.confirmed = confirmed;
        cmd_data.info.tx.port = port;    // data port
    }

    if (timeout_ms < 0) {
        // blocking mode
        timeout_ms = portMAX_DELAY;
    }

    xEventGroupClearBits(LoRaEvents, LORA_STATUS_COMPLETED | LORA_STATUS_ERROR | LORA_STATUS_MSG_SIZE);

    // just pass to the LoRa queue
    if (!xQueueSend(xCmdQueue, (void *)&cmd_data, (TickType_t)(timeout_ms / portTICK_PERIOD_MS))) {
        return 0;
    }

    // validate the message size with the requested data rate
    if (false == ValidatePayloadLength(len, dr, 0)) {
        // message too long
        return -1;
    }

    if (timeout_ms != 0) {
        uint32_t result = xEventGroupWaitBits(LoRaEvents,
                                              LORA_STATUS_COMPLETED | LORA_STATUS_ERROR | LORA_STATUS_MSG_SIZE,
                                              pdTRUE,   // clear on exit
                                              pdFALSE,  // do not wait for all bits
                                              (TickType_t)portMAX_DELAY);

        if (result & LORA_STATUS_MSG_SIZE) {
            return -1;
        } else if (result & LORA_STATUS_ERROR) {
            return 0;
        }
    }
    // return the number of bytes sent
    return len;
}

static void McpsConfirm (McpsConfirm_t *McpsConfirm) {
    uint32_t status = LORA_STATUS_COMPLETED;
    if (McpsConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK) {
        // save the values before calling the event handler
        lora_obj.sftx = McpsConfirm->Datarate;
        lora_obj.tx_trials = McpsConfirm->NbRetries;
        lora_obj.tx_time_on_air = McpsConfirm->TxTimeOnAir;
        lora_obj.tx_power = McpsConfirm->TxPower;
        lora_obj.tx_frequency = McpsConfirm->UpLinkFrequency;
        lora_obj.tx_counter = McpsConfirm->UpLinkCounter;

        switch (McpsConfirm->McpsRequest) {
            case MCPS_UNCONFIRMED: {
                lora_obj.events |= MODLORA_TX_EVENT;
                if (lora_obj.trigger & MODLORA_TX_EVENT) {
                    mp_irq_queue_interrupt(lora_callback_handler, (void *)&lora_obj);
                }
                lora_obj.state = E_LORA_STATE_IDLE;
                xEventGroupSetBits(LoRaEvents, status);
                break;
            }
            case MCPS_CONFIRMED:
                lora_obj.tx_trials = McpsConfirm->NbRetries;
                if (McpsConfirm->AckReceived) {
                    lora_obj.events |= MODLORA_TX_EVENT;
                    if (lora_obj.trigger & MODLORA_TX_EVENT) {
                        mp_irq_queue_interrupt(lora_callback_handler, (void *)&lora_obj);
                    }
                    lora_obj.state = E_LORA_STATE_IDLE;
                    xEventGroupSetBits(LoRaEvents, status);
                } else {
                    // the ack wasn't received, so the stack will re-transmit
                }
                break;
            case MCPS_PROPRIETARY:
                break;
            default:
                break;
        }
    } else {
        lora_obj.events |= MODLORA_TX_FAILED_EVENT;
        if (lora_obj.trigger & MODLORA_TX_FAILED_EVENT) {
            mp_irq_queue_interrupt(lora_callback_handler, (void *)&lora_obj);
        }
        lora_obj.state = E_LORA_STATE_IDLE;
        status |= LORA_STATUS_ERROR;
        xEventGroupSetBits(LoRaEvents, status);
    }
#if defined(FIPY) || defined(LOPY4)
    xSemaphoreGive(xLoRaSigfoxSem);
#endif
}

static void McpsIndication (McpsIndication_t *mcpsIndication) {
    bool bDoEcho = true;

    if (mcpsIndication->Status != LORAMAC_EVENT_INFO_STATUS_OK) {
        return;
    }

    switch (mcpsIndication->McpsIndication) {
        case MCPS_UNCONFIRMED:
            break;
        case MCPS_CONFIRMED:
            break;
        case MCPS_PROPRIETARY:
            break;
        case MCPS_MULTICAST:
            break;
        default:
            break;
    }

    // Check Multicast
    // Check Port
    // Check Datarate
    // Check FramePending
    // Check Buffer
    // Check BufferSize
    // Check Rssi
    // Check Snr
    // Check RxSlot

    lora_obj.rx_timestamp = mcpsIndication->TimeStamp;
    lora_obj.rssi = mcpsIndication->Rssi;
    lora_obj.snr = mcpsIndication->Snr;
    lora_obj.sfrx = mcpsIndication->RxDatarate;

    if ((mcpsIndication->Port == 224) && (lora_obj.ComplianceTest.Enabled == true)
        && (lora_obj.ComplianceTest.Running == true)) {
       MibRequestConfirm_t mibReq;
        mibReq.Type = MIB_CHANNELS_DATARATE;
        LoRaMacMibGetRequestConfirm( &mibReq );
        if (mcpsIndication->Buffer[0] == 4)     { // echo service
            if (ValidatePayloadLength(mcpsIndication->BufferSize, mibReq.Param.ChannelsDatarate, 0)) {
                lora_obj.ComplianceTest.DownLinkCounter++;
            } else {
                // do not increment the downlink counter and don't send the echo either
                bDoEcho = false;
            }
        } else {
            // increament the downlink counter anyhow
            lora_obj.ComplianceTest.DownLinkCounter++;
        }
    }

    // printf("MCPS indication!=%d :%d\n", mcpsIndication->BufferSize, mcpsIndication->Port);

    if (mcpsIndication->RxData && mcpsIndication->BufferSize > 0) {
        if (mcpsIndication->Port > 0 && mcpsIndication->Port < 224) {
            if (mcpsIndication->BufferSize <= LORA_PAYLOAD_SIZE_MAX) {
                memcpy((void *)rx_data_isr.data, mcpsIndication->Buffer, mcpsIndication->BufferSize);
                rx_data_isr.len = mcpsIndication->BufferSize;
                rx_data_isr.port = mcpsIndication->Port;
                xQueueSend(xRxQueue, (void *)&rx_data_isr, 0);
                lora_obj.events |= MODLORA_RX_EVENT;
                if (lora_obj.trigger & MODLORA_RX_EVENT) {
                    mp_irq_queue_interrupt(lora_callback_handler, (void *)&lora_obj);
                }
            }
            // printf("Data on port 1 or 2 received\n");
        } else if (mcpsIndication->Port == 224) {
            if (lora_obj.ComplianceTest.Enabled == true) {
                if (lora_obj.ComplianceTest.Running == false) {
                    // printf("Checking start test msg\n");
                    // Check compliance test enable command (i)
                    if( ( mcpsIndication->BufferSize == 4 ) &&
                        ( mcpsIndication->Buffer[0] == 0x01 ) &&
                        ( mcpsIndication->Buffer[1] == 0x01 ) &&
                        ( mcpsIndication->Buffer[2] == 0x01 ) &&
                        ( mcpsIndication->Buffer[3] == 0x01 ) )
                    {
                        lora_obj.ComplianceTest.IsTxConfirmed = false;
                        lora_obj.ComplianceTest.DownLinkCounter = 0;
                        lora_obj.ComplianceTest.LinkCheck = false;
                        lora_obj.ComplianceTest.DemodMargin = 0;
                        lora_obj.ComplianceTest.NbGateways = 0;
                        lora_obj.ComplianceTest.Running = true;
                        lora_obj.ComplianceTest.State = 1;

                        // flush the rx queue
                        xQueueReset(xRxQueue);

                        // enable ADR during test mode
                        MibRequestConfirm_t mibReq;
                        mibReq.Type = MIB_ADR;
                        mibReq.Param.AdrEnable = true;
                        LoRaMacMibSetRequestConfirm( &mibReq );

                        // always disable duty cycle limitation during test mode
                        LoRaMacTestSetDutyCycleOn(false);

                        // printf("Compliance enabled\n");
                    }
                } else {
                    lora_obj.ComplianceTest.State = mcpsIndication->Buffer[0];
                    switch (lora_obj.ComplianceTest.State) {
                    case 0: // Check compliance test disable command (ii)
                        lora_obj.ComplianceTest.IsTxConfirmed = false;
                        lora_obj.ComplianceTest.DownLinkCounter = 0;
                        lora_obj.ComplianceTest.Running = false;

                        // set adr back to its original value
                        MibRequestConfirm_t mibReq;
                        mibReq.Type = MIB_ADR;
                        mibReq.Param.AdrEnable = lora_obj.adr;
                        LoRaMacMibSetRequestConfirm(&mibReq);
                        LoRaMacTestSetDutyCycleOn(true);
                        // printf("Compliance disabled\n");
                        break;
                    case 1: // (iii, iv)
                        lora_obj.ComplianceTest.Running = true;
                        // printf("Compliance running\n");
                        break;
                    case 2: // Enable confirmed messages (v)
                        lora_obj.ComplianceTest.IsTxConfirmed = true;
                        lora_obj.ComplianceTest.State = 1;
                        // printf("Confirmed messages enabled\n");
                        break;
                    case 3:  // Disable confirmed messages (vi)
                        lora_obj.ComplianceTest.IsTxConfirmed = false;
                        lora_obj.ComplianceTest.State = 1;
                        // printf("Confirmed messages disabled\n");
                        break;
                    case 4: // (vii)
                        // return the payload
                        if (bDoEcho) {
                            if (mcpsIndication->BufferSize <= LORA_PAYLOAD_SIZE_MAX) {
                                memcpy((void *)rx_data_isr.data, mcpsIndication->Buffer, mcpsIndication->BufferSize);
                                rx_data_isr.len = mcpsIndication->BufferSize;
                                xQueueSend(xRxQueue, (void *)&rx_data_isr, 0);
                            }
                        } else {
                            // set the state back to 1
                            lora_obj.ComplianceTest.State = 1;
                        }
                        // printf("Crypto message received\n");
                        break;
                    case 5: // (viii)
                        // trigger a link check
                        lora_obj.state = E_LORA_STATE_LINK_CHECK;
                        // printf("Link check\n");
                        break;
                    case 6: // (ix)
                        // trigger a join request
                        lora_obj.joined = false;
                        lora_obj.ComplianceTest.State = 6;
                        // printf("Trigger join\n");
                        break;
                    default:
                        break;
                    }
                }
            }
        }
    }
}

static void MlmeConfirm (MlmeConfirm_t *MlmeConfirm) {
    if (MlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK) {
        switch (MlmeConfirm->MlmeRequest) {
            case MLME_JOIN:
                TimerStop(&TxNextActReqTimer);
                lora_obj.joined = true;
                lora_obj.ComplianceTest.State = 1;
                lora_obj.ComplianceTest.Running = false;
                lora_obj.ComplianceTest.DownLinkCounter = 0;
                break;
            case MLME_LINK_CHECK:
                // Check DemodMargin
                // Check NbGateways
                if (lora_obj.ComplianceTest.Running == true) {
                    lora_obj.ComplianceTest.LinkCheck = true;
                    lora_obj.ComplianceTest.DemodMargin = MlmeConfirm->DemodMargin;
                    lora_obj.ComplianceTest.NbGateways = MlmeConfirm->NbGateways;
                }
                // printf("Link check confirm\n");
                break;
            default:
                break;
        }
    }
#if defined(FIPY) || defined(LOPY4)
    xSemaphoreGive(xLoRaSigfoxSem);
#endif
}

static void OnTxNextActReqTimerEvent(void) {
    MibRequestConfirm_t mibReq;
    LoRaMacStatus_t status;

    mibReq.Type = MIB_NETWORK_JOINED;
    status = LoRaMacMibGetRequestConfirm(&mibReq);

    if (status == LORAMAC_STATUS_OK) {
        if (mibReq.Param.IsNetworkJoined == true) {
            lora_obj.joined = true;
            lora_obj.ComplianceTest.State = 1;
            lora_obj.ComplianceTest.Running = false;
            lora_obj.ComplianceTest.DownLinkCounter = 0;
        } else {
            lora_obj.state = E_LORA_STATE_JOIN;
        }
    }
}

static void MlmeIndication( MlmeIndication_t *mlmeIndication )
{
    switch( mlmeIndication->MlmeIndication )
    {
        case MLME_SCHEDULE_UPLINK:
        {// The MAC signals that we shall provide an uplink as soon as possible
            printf("Trying to send uplink\n");
            OnTxNextActReqTimerEvent( );
            break;
        }
        default:
            break;
    }
}

static void TASK_LoRa (void *pvParameters) {
    MibRequestConfirm_t mibReq;
    MlmeReq_t mlmeReq;
    McpsReq_t mcpsReq;
    bool isReset;

    lora_obj.state = E_LORA_STATE_NOINIT;
    lora_obj.pwr_mode = E_LORA_MODE_ALWAYS_ON;

    for ( ; ; ) {
        vTaskDelay (2 / portTICK_PERIOD_MS);

        if(lora_obj.reset)
        {
            Radio.Reset();
            lora_obj.state = E_LORA_STATE_RESET;
            lora_obj.reset = false;
        }
        switch (lora_obj.state) {
        case E_LORA_STATE_NOINIT:
        case E_LORA_STATE_IDLE:
        case E_LORA_STATE_RX:
        case E_LORA_STATE_SLEEP:
        case E_LORA_STATE_RESET:
            // receive from the command queue and act accordingly
            if (xQueueReceive(xCmdQueue, &task_cmd_data, 0)) {
                switch (task_cmd_data.cmd) {
                case E_LORA_CMD_INIT:
                    isReset = lora_obj.state == E_LORA_STATE_RESET? true:false;
                    // save the new configuration first
                    lora_set_config(&task_cmd_data);
                    if (task_cmd_data.info.init.stack_mode == E_LORA_STACK_MODE_LORAWAN) {
                        LoRaMacPrimitives.MacMcpsConfirm = McpsConfirm;
                        LoRaMacPrimitives.MacMcpsIndication = McpsIndication;
                        LoRaMacPrimitives.MacMlmeConfirm = MlmeConfirm;
                        LoRaMacPrimitives.MacMlmeIndication = MlmeIndication;
                        LoRaMacCallbacks.GetBatteryLevel = BoardGetBatteryLevel;
                        LoRaMacInitialization(&LoRaMacPrimitives, &LoRaMacCallbacks, task_cmd_data.info.init.region);

                        TimerStop(&TxNextActReqTimer);
                        TimerInit(&TxNextActReqTimer, OnTxNextActReqTimerEvent);
                        TimerSetValue(&TxNextActReqTimer, OVER_THE_AIR_ACTIVATION_DUTYCYCLE);

                        mibReq.Type = MIB_ADR;
                        mibReq.Param.AdrEnable = task_cmd_data.info.init.adr;
                        LoRaMacMibSetRequestConfirm(&mibReq);

                        mibReq.Type = MIB_PUBLIC_NETWORK;
                        mibReq.Param.EnablePublicNetwork = task_cmd_data.info.init.public;
                        LoRaMacMibSetRequestConfirm(&mibReq);

                        mibReq.Type = MIB_DEVICE_CLASS;
                        mibReq.Param.Class = task_cmd_data.info.init.device_class;
                        LoRaMacMibSetRequestConfirm(&mibReq);

                        LoRaMacTestSetDutyCycleOn(false);

                        // check if we have already joined the network
                        if (lora_obj.joined) {
                            uint32_t length;
                            bool result = true;
                            result &= modlora_nvs_get_uint(E_LORA_NVS_ELE_NET_ID, (uint32_t *)&lora_obj.net_id);
                            result &= modlora_nvs_get_uint(E_LORA_NVS_ELE_DEVADDR, (uint32_t *)&lora_obj.u.abp.DevAddr);
                            length = 16;
                            result &= modlora_nvs_get_blob(E_LORA_NVS_ELE_NWSKEY, (void *)lora_obj.u.abp.NwkSKey, &length);
                            length = 16;
                            result &= modlora_nvs_get_blob(E_LORA_NVS_ELE_APPSKEY, (void *)lora_obj.u.abp.AppSKey, &length);

                            uint32_t uplinks, downlinks;
                            result &= modlora_nvs_get_uint(E_LORA_NVS_ELE_UPLINK, &uplinks);
                            result &= modlora_nvs_get_uint(E_LORA_NVS_ELE_DWLINK, &downlinks);
                            result &= modlora_nvs_get_uint(E_LORA_NVS_ELE_ADR_ACKS, LoRaMacGetAdrAckCounter());

                            if (result) {
                                mibReq.Type = MIB_UPLINK_COUNTER;
                                mibReq.Param.UpLinkCounter = uplinks;
                                LoRaMacMibSetRequestConfirm( &mibReq );

                                mibReq.Type = MIB_DOWNLINK_COUNTER;
                                mibReq.Param.DownLinkCounter = downlinks;
                                LoRaMacMibSetRequestConfirm( &mibReq );

                                // write the MAC params directly from the NVRAM
                                length = sizeof(LoRaMacParams_t);
                                modlora_nvs_get_blob(E_LORA_NVS_ELE_MAC_PARAMS, (void *)LoRaMacGetMacParams(), &length);

                                // write the channel list directly from the NVRAM
                                ChannelParams_t *channels;
                                LoRaMacGetChannelList(&channels, &length);
                                modlora_nvs_get_blob(E_LORA_NVS_ELE_CHANNELS, channels, &length);

                                // write the channel mask directly from the NVRAM
                                uint16_t *channelmask;
                                if (LoRaMacGetChannelsMask(&channelmask, &length)) {
                                    modlora_nvs_get_blob(E_LORA_NVS_ELE_CHANNELMASK, channelmask, &length);
                                }

                                // write the channel mask remaining directly from the NVRAM
                                if (LoRaMacGetChannelsMaskRemaining(&channelmask, &length)) {
                                    modlora_nvs_get_blob(E_LORA_NVS_ELE_CHANNELMASK_REMAINING, channelmask, &length);
                                }

                                uint32_t srv_ack_req;
                                modlora_nvs_get_uint(E_LORA_NVS_ELE_ACK_REQ, (uint32_t *)&srv_ack_req);
                                bool *ack_req = LoRaMacGetSrvAckRequested();
                                if (srv_ack_req) {
                                    *ack_req = true;
                                } else {
                                    *ack_req = false;
                                }

                                uint32_t mac_cmd_next_tx;
                                modlora_nvs_get_uint(E_LORA_NVS_MAC_NXT_TX, (uint32_t *)&mac_cmd_next_tx);
                                bool *next_tx = LoRaMacGetMacCmdNextTx();
                                if (mac_cmd_next_tx) {
                                    *next_tx = true;
                                } else {
                                    *next_tx = false;
                                }

                                uint32_t mac_cmd_buffer_idx;
                                modlora_nvs_get_uint(E_LORA_NVS_MAC_CMD_BUF_IDX, (uint32_t *)&mac_cmd_buffer_idx);
                                uint8_t *buffer_idx = LoRaMacGetMacCmdBufferIndex();
                                *buffer_idx = mac_cmd_buffer_idx;

                                modlora_nvs_get_uint(E_LORA_NVS_MAC_CMD_BUF_RPT_IDX, (uint32_t *)&mac_cmd_buffer_idx);
                                buffer_idx = LoRaMacGetMacCmdBufferRepeatIndex();
                                *buffer_idx = mac_cmd_buffer_idx;

                                // write the buffered MAC commads directly from NVRAM
                                length = 128;
                                modlora_nvs_get_blob(E_LORA_NVS_ELE_MAC_BUF, (void *)LoRaMacGetMacCmdBuffer(), &length);

                                // write the buffered MAC commads to repeat directly from NVRAM
                                length = 128;
                                modlora_nvs_get_blob(E_LORA_NVS_ELE_MAC_RPT_BUF, (void *)LoRaMacGetMacCmdBufferRepeat(), &length);

                                lora_obj.activation = E_LORA_ACTIVATION_ABP;
                                lora_obj.state = E_LORA_STATE_JOIN;
                                // clear the joined flag until the nvram_save method is called again
                                modlora_nvs_set_uint(E_LORA_NVS_ELE_JOINED, (uint32_t)false);
                            } else {
                                lora_obj.state = E_LORA_STATE_IDLE;
                            }
                        } else {
                            lora_obj.state = E_LORA_STATE_IDLE;
                        }
                    } else {
                        // radio initialization
                        RadioEvents.TxDone = OnTxDone;
                        RadioEvents.RxDone = OnRxDone;
                        RadioEvents.TxTimeout = OnTxTimeout;
                        RadioEvents.RxTimeout = OnRxTimeout;
                        RadioEvents.RxError = OnRxError;
                        Radio.Init(&RadioEvents);

                        // radio configuration
                        lora_radio_setup(&task_cmd_data.info.init);
                        lora_obj.state = E_LORA_STATE_IDLE;
                    }
                    lora_obj.joined = false;
                    if (lora_obj.state == E_LORA_STATE_IDLE) {
                        xEventGroupSetBits(LoRaEvents, LORA_STATUS_COMPLETED);
                    }
                    if (isReset) {
                        xEventGroupSetBits(LoRaEvents, LORA_STATUS_RESET_DONE);
                    }
                    break;
                case E_LORA_CMD_JOIN:
                    lora_obj.joined = false;
                    lora_obj.activation = task_cmd_data.info.join.activation;
                    if (lora_obj.activation == E_LORA_ACTIVATION_OTAA) {
                        memcpy((void *)lora_obj.u.otaa.DevEui, task_cmd_data.info.join.u.otaa.DevEui, sizeof(lora_obj.u.otaa.DevEui));
                        memcpy((void *)lora_obj.u.otaa.AppEui, task_cmd_data.info.join.u.otaa.AppEui, sizeof(lora_obj.u.otaa.AppEui));
                        memcpy((void *)lora_obj.u.otaa.AppKey, task_cmd_data.info.join.u.otaa.AppKey, sizeof(lora_obj.u.otaa.AppKey));
                        lora_obj.otaa_dr = task_cmd_data.info.join.otaa_dr;
                    } else {
                        lora_obj.net_id = DEF_LORAWAN_NETWORK_ID;
                        lora_obj.u.abp.DevAddr = task_cmd_data.info.join.u.abp.DevAddr;
                        memcpy((void *)lora_obj.u.abp.AppSKey, task_cmd_data.info.join.u.abp.AppSKey, sizeof(lora_obj.u.abp.AppSKey));
                        memcpy((void *)lora_obj.u.abp.NwkSKey, task_cmd_data.info.join.u.abp.NwkSKey, sizeof(lora_obj.u.abp.NwkSKey));
                    }
                    lora_obj.state = E_LORA_STATE_JOIN;
                    break;
                case E_LORA_CMD_TX:
                    // implement Listen-before-Talk LBT, only for LoRa RAW (not LoRaWAN)
                    if (lora_lbt_is_free()) {
                        // no activity detected on Lora, so send the pack now

                        // taking sigfox semaphore blocks ?!?!?
                        // maybe, in the end of TX sempahore has to be released sooner
//                        #if defined(FIPY) || defined(LOPY4)
//                            xSemaphoreTake(xLoRaSigfoxSem, portMAX_DELAY);
//                        #endif
                        Radio.Send(task_cmd_data.info.tx.data, task_cmd_data.info.tx.len);
                        lora_obj.state = E_LORA_STATE_TX;
                    } else {
                        // activity detected on Lora, so put the TX command back on queue on Front
                        // to be executed on next Lora task
                        xQueueSendToFront(xCmdQueue, (void *)&task_cmd_data, (TickType_t)portMAX_DELAY);
                    }
                    break;
                case E_LORA_CMD_CONFIG_CHANNEL:
                    if (task_cmd_data.info.channel.add) {
                        ChannelParams_t channel =
                        { task_cmd_data.info.channel.frequency, 0, {((task_cmd_data.info.channel.dr_max << 4) | task_cmd_data.info.channel.dr_min)}, 0};
                        ChannelAddParams_t channelAdd = { &channel, task_cmd_data.info.channel.index };
                        RegionChannelManualAdd(lora_obj.region, &channelAdd);
                    } else {
                        ChannelRemoveParams_t channelRemove = { task_cmd_data.info.channel.index };
                        RegionChannelsManualRemove(lora_obj.region, &channelRemove);
                    }
                    xEventGroupSetBits(LoRaEvents, LORA_STATUS_COMPLETED);
                    break;
                case E_LORA_CMD_LORAWAN_TX: {
                        LoRaMacTxInfo_t txInfo;
                        EventBits_t status = 0;
                        bool empty_frame = false;
                        int8_t mac_datarate = 0;

                        // set the new data rate before checking if Tx is possible, but store the current one
                        if (!lora_obj.adr) {
                            mibReq.Type = MIB_CHANNELS_DATARATE;
                            LoRaMacMibGetRequestConfirm( &mibReq );
                            mac_datarate = mibReq.Param.ChannelsDatarate;
                            mibReq.Param.ChannelsDatarate = task_cmd_data.info.tx.dr;
                            LoRaMacMibSetRequestConfirm( &mibReq );
                        }

                        if (LoRaMacQueryTxPossible (task_cmd_data.info.tx.len, &txInfo) != LORAMAC_STATUS_OK) {
                            // send an empty frame in order to flush MAC commands
                            mcpsReq.Type = MCPS_UNCONFIRMED;
                            mcpsReq.Req.Unconfirmed.fBuffer = NULL;
                            mcpsReq.Req.Unconfirmed.fBufferSize = 0;
                            mcpsReq.Req.Unconfirmed.Datarate = task_cmd_data.info.tx.dr;
                            empty_frame = true;
                            status |= LORA_STATUS_MSG_SIZE;
                        } else {
                            if (task_cmd_data.info.tx.confirmed) {
                                mcpsReq.Type = MCPS_CONFIRMED;
                                mcpsReq.Req.Confirmed.fPort = task_cmd_data.info.tx.port;
                                mcpsReq.Req.Confirmed.fBuffer = task_cmd_data.info.tx.data;
                                mcpsReq.Req.Confirmed.fBufferSize = task_cmd_data.info.tx.len;
                                mcpsReq.Req.Confirmed.NbTrials = lora_obj.tx_retries + 1;
                                mcpsReq.Req.Confirmed.Datarate = task_cmd_data.info.tx.dr;
                            } else {
                                mcpsReq.Type = MCPS_UNCONFIRMED;
                                mcpsReq.Req.Unconfirmed.fPort = task_cmd_data.info.tx.port;
                                mcpsReq.Req.Unconfirmed.fBuffer = task_cmd_data.info.tx.data;
                                mcpsReq.Req.Unconfirmed.fBufferSize = task_cmd_data.info.tx.len;
                                mcpsReq.Req.Unconfirmed.Datarate = task_cmd_data.info.tx.dr;
                            }
                        }
                    #if defined(FIPY) || defined(LOPY4)
                        xSemaphoreTake(xLoRaSigfoxSem, portMAX_DELAY);
                    #endif

                        // set back the original datarate
                        if (!lora_obj.adr) {
                            mibReq.Param.ChannelsDatarate = mac_datarate;
                            LoRaMacMibSetRequestConfirm( &mibReq );
                        }

                        if (LoRaMacMcpsRequest(&mcpsReq) != LORAMAC_STATUS_OK || empty_frame) {
                            // the command has failed, send the response now
                            lora_obj.state = E_LORA_STATE_IDLE;
                            status |= LORA_STATUS_ERROR;
                            xEventGroupSetBits(LoRaEvents, status);
                        #if defined(FIPY) || defined(LOPY4)
                            xSemaphoreGive(xLoRaSigfoxSem);
                        #endif
                        } else {
                            lora_obj.state = E_LORA_STATE_TX;
                        }
                    }
                    break;
                case E_LORA_CMD_SLEEP:
                    Radio.Sleep();
                    lora_obj.state = E_LORA_STATE_SLEEP;
                    xEventGroupSetBits(LoRaEvents, LORA_STATUS_COMPLETED);
                #if defined(FIPY) || defined(LOPY4)
                    xSemaphoreGive(xLoRaSigfoxSem);
                #endif
                    break;
                case E_LORA_CMD_WAKE_UP:
                    // just enable the receiver again
                    Radio.Rx(LORA_RX_TIMEOUT);
                    lora_obj.state = E_LORA_STATE_RX;
                    xEventGroupSetBits(LoRaEvents, LORA_STATUS_COMPLETED);
                #if defined(FIPY) || defined(LOPY4)
                    xSemaphoreGive(xLoRaSigfoxSem);
                #endif
                    break;
                default:
                    break;
                }
//            } else if (lora_obj.state == E_LORA_STATE_IDLE && lora_obj.stack_mode == E_LORA_STACK_MODE_LORA) {
//                Radio.Rx(LORA_RX_TIMEOUT);
//                lora_obj.state = E_LORA_STATE_RX;
            }
            break;
        case E_LORA_STATE_JOIN:
            TimerStop( &TxNextActReqTimer );
            if (!lora_obj.joined) {
                if (lora_obj.activation == E_LORA_ACTIVATION_OTAA) {
                #if defined(FIPY) || defined(LOPY4)
                    xSemaphoreTake(xLoRaSigfoxSem, portMAX_DELAY);
                #endif
                    mibReq.Type = MIB_NETWORK_ACTIVATION;
                    mibReq.Param.NetworkActivation = ACTIVATION_TYPE_OTAA;
                    LoRaMacMibSetRequestConfirm( &mibReq );

                    TimerStart( &TxNextActReqTimer );
                    mlmeReq.Type = MLME_JOIN;
                    mlmeReq.Req.Join.DevEui = (uint8_t *)lora_obj.u.otaa.DevEui;
                    mlmeReq.Req.Join.AppEui = (uint8_t *)lora_obj.u.otaa.AppEui;
                    mlmeReq.Req.Join.AppKey = (uint8_t *)lora_obj.u.otaa.AppKey;
                    mlmeReq.Req.Join.NbTrials = 1;
                    mlmeReq.Req.Join.DR = (uint8_t) lora_obj.otaa_dr;
                    LoRaMacMlmeRequest( &mlmeReq );
                } else {
                    mibReq.Type = MIB_NETWORK_ACTIVATION;
                    mibReq.Param.NetworkActivation = ACTIVATION_TYPE_ABP;
                    LoRaMacMibSetRequestConfirm( &mibReq );

                    mibReq.Type = MIB_NET_ID;
                    mibReq.Param.NetID = lora_obj.net_id;
                    LoRaMacMibSetRequestConfirm( &mibReq );

                    mibReq.Type = MIB_DEV_ADDR;
                    mibReq.Param.DevAddr = (uint32_t)lora_obj.u.abp.DevAddr;
                    LoRaMacMibSetRequestConfirm( &mibReq );

                    mibReq.Type = MIB_NWK_SKEY;
                    mibReq.Param.NwkSKey = (uint8_t *)lora_obj.u.abp.NwkSKey;
                    LoRaMacMibSetRequestConfirm( &mibReq );

                    mibReq.Type = MIB_APP_SKEY;
                    mibReq.Param.AppSKey = (uint8_t *)lora_obj.u.abp.AppSKey;
                    LoRaMacMibSetRequestConfirm( &mibReq );

                    mibReq.Type = MIB_NETWORK_JOINED;
                    mibReq.Param.IsNetworkJoined = true;
                    LoRaMacMibSetRequestConfirm( &mibReq );
                    lora_obj.joined = true;
                    lora_obj.ComplianceTest.State = 1;
                }
            }
            xEventGroupSetBits(LoRaEvents, LORA_STATUS_COMPLETED);
            lora_obj.state = E_LORA_STATE_IDLE;
            break;
        case E_LORA_STATE_LINK_CHECK:
            mlmeReq.Type = MLME_LINK_CHECK;
            LoRaMacMlmeRequest(&mlmeReq);
            lora_obj.state = E_LORA_STATE_IDLE;
            break;
        case E_LORA_STATE_RX_DONE:
        case E_LORA_STATE_RX_TIMEOUT:
        case E_LORA_STATE_RX_ERROR:
            // we need to perform a mode transition in order to clear the TxRx FIFO
            Radio.Sleep();
            //lora_obj.state = E_LORA_STATE_IDLE;
            lora_obj.state = E_LORA_STATE_RX;
            Radio.Rx(LORA_RX_TIMEOUT);
            break;
        case E_LORA_STATE_TX:
            break;
        case E_LORA_STATE_TX_DONE:
            // we need to perform a mode transition in order to clear the TxRx FIFO
            Radio.Sleep();
            xEventGroupSetBits(LoRaEvents, LORA_STATUS_COMPLETED);
            //lora_obj.state = E_LORA_STATE_IDLE;
            lora_obj.state = E_LORA_STATE_RX;
            Radio.Rx(LORA_RX_TIMEOUT);
        #if defined(FIPY) || defined(LOPY4)
            xSemaphoreGive(xLoRaSigfoxSem);
        #endif
            break;
        case E_LORA_STATE_TX_TIMEOUT:
            // we need to perform a mode transition in order to clear the TxRx FIFO
            Radio.Sleep();
            xEventGroupSetBits(LoRaEvents, LORA_STATUS_ERROR);
            //lora_obj.state = E_LORA_STATE_IDLE;
            lora_obj.state = E_LORA_STATE_RX;
            Radio.Rx(LORA_RX_TIMEOUT);
        #if defined(FIPY) || defined(LOPY4)
            xSemaphoreGive(xLoRaSigfoxSem);
        #endif
            break;
        default:
            break;
        }

        TimerLowPowerHandler();
    }
}

static void TASK_LoRa_Timer (void *pvParameters) {

    for(;;)
    {
        modlora_timerCallback cb;
        while (pdTRUE == xQueueReceive(xCbQueue, &cb, portMAX_DELAY))
        {
            if(cb != NULL)
            {
                cb();
            }
        }
    }
}

/*! lora_lbt_is_free checks if there is any Lora traffic on the current channel
 * note: if Modem is not in RX, returns true (free spectrum)
 * additionally rssi threshold and timeout_us can be added as function parameters
 * returns true if spectrum is free
 * returns false if spectrum is busy
 */

static bool lora_lbt_is_free(void)
{
    int16_t rssi_th = -85;
    int timeout_us = 1000; //[microsec]
    bool is_free = true;

    int rssi = -1000;
    int rssi_max = -1000;

    uint64_t now = mp_hal_ticks_us_non_blocking();

    do {
        if (RF_RX_RUNNING == Radio.GetStatus()) {
            rssi = Radio.Rssi(MODEM_LORA);
        }
        rssi_max = (rssi > rssi_max) ? rssi : rssi_max;

        if ( rssi_max > rssi_th ) {
            break;
        }
    } while (mp_hal_ticks_us_non_blocking() - now < timeout_us);

    is_free = (rssi_max < rssi_th);

    return is_free;
}

static void lora_callback_handler(void *arg) {
    lora_obj_t *self = arg;

    if (self->handler && self->handler != mp_const_none) {
        mp_call_function_1(self->handler, self->handler_arg);
    }
}

static IRAM_ATTR void OnTxDone (void) {
    lora_obj.events |= MODLORA_TX_EVENT;
    if (lora_obj.trigger & MODLORA_TX_EVENT) {
        mp_irq_queue_interrupt(lora_callback_handler, (void *)&lora_obj);
    }
    lora_obj.state = E_LORA_STATE_TX_DONE;
}

static IRAM_ATTR void OnRxDone (uint8_t *payload, uint32_t timestamp, uint16_t size, int16_t rssi, int8_t snr, uint8_t sf) {
    lora_obj.rx_timestamp = timestamp;
    lora_obj.rssi = rssi;
    lora_obj.snr = snr;
    lora_obj.sfrx = sf;
    if (size <= LORA_PAYLOAD_SIZE_MAX) {
        memcpy((void *)rx_data_isr.data, payload, size);
        rx_data_isr.len = size;
        xQueueSendFromISR(xRxQueue, (void *)&rx_data_isr, NULL);
    }

    lora_obj.events |= MODLORA_RX_EVENT;
    if (lora_obj.trigger & MODLORA_RX_EVENT) {
        mp_irq_queue_interrupt(lora_callback_handler, (void *)&lora_obj);
    }

    lora_obj.state = E_LORA_STATE_RX_DONE;
}

static IRAM_ATTR void OnTxTimeout (void) {
    lora_obj.state = E_LORA_STATE_TX_TIMEOUT;
}

static IRAM_ATTR void OnRxTimeout (void) {
    lora_obj.state = E_LORA_STATE_RX_TIMEOUT;
}

static IRAM_ATTR void OnRxError (void) {
    lora_obj.state = E_LORA_STATE_RX_ERROR;
}

static void lora_radio_setup (lora_init_cmd_data_t *init_data) {
    uint16_t symbol_to = 8;

    Radio.SetModem(MODEM_LORA);

    if (init_data->public) {
        Radio.Write(REG_LR_SYNCWORD, LORA_MAC_PUBLIC_SYNCWORD);
    } else {
        Radio.Write(REG_LR_SYNCWORD, LORA_MAC_PRIVATE_SYNCWORD);
    }

    Radio.SetChannel(init_data->frequency);

    Radio.SetTxConfig(MODEM_LORA, init_data->tx_power, 0, init_data->bandwidth,
                                  init_data->sf, init_data->coding_rate,
                                  init_data->preamble, LORA_FIX_LENGTH_PAYLOAD_OFF,
                                  true, 0, 0, init_data->txiq, LORA_TX_TIMEOUT_MAX);

    Radio.SetRxConfig(MODEM_LORA, init_data->bandwidth, init_data->sf,
                                  init_data->coding_rate, 0, init_data->preamble,
                                  symbol_to, LORA_FIX_LENGTH_PAYLOAD_OFF,
                                  0, true, 0, 0, init_data->rxiq, true);

    Radio.SetMaxPayloadLength(MODEM_LORA, LORA_PAYLOAD_SIZE_MAX);

    if (init_data->power_mode == E_LORA_MODE_ALWAYS_ON) {
        // start listening
        Radio.Rx(LORA_RX_TIMEOUT);
        lora_obj.state = E_LORA_STATE_RX;
    } else {
        Radio.Sleep();
        lora_obj.state = E_LORA_STATE_SLEEP;
    }
}

static void lora_validate_mode (uint32_t mode) {
    #ifdef PHYSEC
    if (mode > E_LORA_STACK_MODE_LORAPHYSEC) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "invalid mode %d", mode));
    }else{
        if(mode == E_LORA_STACK_MODE_LORAPHYSEC){
            printf("lora mode : LORAPHYSEC\n");
        }
    }
    #else
    if (mode > E_LORA_STACK_MODE_LORAWAN) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "invalid mode %d", mode));
    }
    #endif
}

static void lora_validate_frequency (uint32_t frequency) {
    switch (lora_obj.region) {
        case LORAMAC_REGION_AS923:
            if (frequency < 915000000 || frequency > 928000000) {
                goto freq_error;
            }
            break;
        case LORAMAC_REGION_AU915:
            if (frequency < 915000000 || frequency > 928000000) {
                goto freq_error;
            }
            break;
        case LORAMAC_REGION_US915:
            if (frequency < 902000000 || frequency > 928000000) {
                goto freq_error;
            }
            break;
        case LORAMAC_REGION_US915_HYBRID:
            if (frequency < 902000000 || frequency > 928000000) {
                goto freq_error;
            }
            break;
        case LORAMAC_REGION_CN470:
        #if defined(LOPY4)
            if (frequency < 470000000 || frequency > 510000000) {
                goto freq_error;
            }
        #else
            goto freq_error;
        #endif
            break;
        case LORAMAC_REGION_IN865:
            if (frequency < 865000000 || frequency > 867000000) {
                goto freq_error;
            }
            break;
        case LORAMAC_REGION_EU433:
        #if defined(LOPY4)
            if (frequency < 433000000 || frequency > 435000000) { // LoRa 433 - 434
                goto freq_error;
            }
        #else
            goto freq_error;
        #endif
            break;
        case LORAMAC_REGION_EU868:
            if (frequency < 863000000 || frequency > 870000000) {
                goto freq_error;
            }
            break;
        default:
            break;
    }
    return;

freq_error:
    nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "frequency %d out of range", frequency));
}

static void lora_validate_channel (uint32_t index) {
    switch (lora_obj.region) {
        case LORAMAC_REGION_AS923:
            if (index >= AS923_MAX_NB_CHANNELS) {
                goto channel_error;
            }
            break;
        case LORAMAC_REGION_AU915:
            if (index >= AU915_MAX_NB_CHANNELS) {
                goto channel_error;
            }
            break;
        case LORAMAC_REGION_US915:
            if (index >= US915_MAX_NB_CHANNELS) {
                goto channel_error;
            }
            break;
        case LORAMAC_REGION_US915_HYBRID:
            if (index >= US915_HYBRID_MAX_NB_CHANNELS) {
                goto channel_error;
            }
            break;
        case LORAMAC_REGION_EU868:
            if (index >= EU868_MAX_NB_CHANNELS) {
                goto channel_error;
            }
            break;
        case LORAMAC_REGION_CN470:
            if (index >= CN470_MAX_NB_CHANNELS) {
                goto channel_error;
            }
            break;
        case LORAMAC_REGION_IN865:
            if (index >= IN865_MAX_NB_CHANNELS) {
                goto channel_error;
            }
            break;
        default:
            break;
    }
    return;

channel_error:
    nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "channel %d out of range", index));
}

static void lora_validate_power (uint8_t tx_power) {
    if (tx_power < TX_OUTPUT_POWER_MIN || tx_power > TX_OUTPUT_POWER_MAX) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "Tx power %d out of range", tx_power));
    }
}

static bool lora_validate_data_rate (uint32_t data_rate) {

    switch (lora_obj.region) {
    case LORAMAC_REGION_AS923:
    case LORAMAC_REGION_EU868:
    case LORAMAC_REGION_AU915:
    case LORAMAC_REGION_EU433:
    case LORAMAC_REGION_CN470:
    case LORAMAC_REGION_IN865:
        if (data_rate > DR_6) {
            return false;
        }
        break;
    case LORAMAC_REGION_US915:
    case LORAMAC_REGION_US915_HYBRID:
        if (data_rate > DR_4) {
            return false;
        }
        break;
    default:
        break;
    }
    return true;
}

static void lora_validate_bandwidth (uint8_t bandwidth) {
    if (bandwidth > E_LORA_BW_500_KHZ) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "bandwidth %d not supported", bandwidth));
    }
}

static void lora_validate_sf (uint8_t sf) {
    if (sf < LORA_SPREADING_FACTOR_MIN || sf > LORA_SPREADING_FACTOR_MAX) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "spreading factor %d out of range", sf));
    }
}

static void lora_validate_coding_rate (uint8_t coding_rate) {
    if (coding_rate < E_LORA_CODING_4_5 || coding_rate > E_LORA_CODING_4_8) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "coding rate %d out of range", coding_rate));
    }
}

static void lora_validate_power_mode (uint8_t power_mode) {
    if (power_mode < E_LORA_MODE_ALWAYS_ON || power_mode > E_LORA_MODE_SLEEP) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "invalid power mode %d", power_mode));
    }
}

static void lora_validate_device_class (DeviceClass_t device_class) {
    // CLASS_B is not implemented
    if (device_class != CLASS_A && device_class != CLASS_C) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "invalid device_class %d", device_class));
    }
}

static void lora_validate_region (LoRaMacRegion_t region) {
    if (region != LORAMAC_REGION_AS923 && region != LORAMAC_REGION_AU915
        && region != LORAMAC_REGION_EU868 && region != LORAMAC_REGION_US915
        && region != LORAMAC_REGION_IN865
#if defined(LOPY4)
        && region != LORAMAC_REGION_EU433 && region != LORAMAC_REGION_CN470
#endif
        ) {
            nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "invalid region %d", region));
    }
}


static void lora_set_config (lora_cmd_data_t *cmd_data) {
    lora_obj.stack_mode = cmd_data->info.init.stack_mode;
    lora_obj.bandwidth = cmd_data->info.init.bandwidth;
    lora_obj.coding_rate = cmd_data->info.init.coding_rate;
    lora_obj.frequency = cmd_data->info.init.frequency;
    lora_obj.preamble = cmd_data->info.init.preamble;
    lora_obj.rxiq = cmd_data->info.init.rxiq;
    lora_obj.txiq = cmd_data->info.init.txiq;
    lora_obj.sf = cmd_data->info.init.sf;
    lora_obj.tx_power = cmd_data->info.init.tx_power;
    lora_obj.pwr_mode = cmd_data->info.init.power_mode;
    lora_obj.adr = cmd_data->info.init.adr;
    lora_obj.public = cmd_data->info.init.public;
    lora_obj.tx_retries = cmd_data->info.init.tx_retries;
    lora_obj.device_class = cmd_data->info.init.device_class;
    lora_obj.region = cmd_data->info.init.region;
}

static void lora_get_config (lora_cmd_data_t *cmd_data) {
    cmd_data->info.init.stack_mode = lora_obj.stack_mode;
    cmd_data->info.init.bandwidth = lora_obj.bandwidth;
    cmd_data->info.init.coding_rate = lora_obj.coding_rate;
    cmd_data->info.init.frequency = lora_obj.frequency;
    cmd_data->info.init.preamble = lora_obj.preamble;
    cmd_data->info.init.rxiq = lora_obj.rxiq;
    cmd_data->info.init.txiq = lora_obj.txiq;
    cmd_data->info.init.sf = lora_obj.sf;
    cmd_data->info.init.tx_power = lora_obj.tx_power;
    cmd_data->info.init.power_mode = lora_obj.pwr_mode;
    cmd_data->info.init.public = lora_obj.public;
    cmd_data->info.init.adr = lora_obj.adr;
    cmd_data->info.init.tx_retries = lora_obj.tx_retries;
    cmd_data->info.init.device_class = lora_obj.device_class;
    cmd_data->info.init.region = lora_obj.region;
}

static void lora_send_cmd (lora_cmd_data_t *cmd_data) {
    xEventGroupClearBits(LoRaEvents, LORA_STATUS_COMPLETED | LORA_STATUS_ERROR | LORA_STATUS_MSG_SIZE);

    xQueueSend(xCmdQueue, (void *)cmd_data, (TickType_t)portMAX_DELAY);

    uint32_t result = xEventGroupWaitBits(LoRaEvents,
                                          LORA_STATUS_COMPLETED | LORA_STATUS_ERROR,
                                          pdTRUE,   // clear on exit
                                          pdFALSE,  // do not wait for all bits
                                          (TickType_t)portMAX_DELAY);

    if (result & LORA_STATUS_ERROR) {
        nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, mpexception_os_operation_failed));
    }
}

static int32_t lora_send (const byte *buf, uint32_t len, uint32_t timeout_ms) {
    lora_cmd_data_t cmd_data;

/*
#if defined(FIPY) || defined(LOPY4)
    xSemaphoreTake(xLoRaSigfoxSem, portMAX_DELAY);
    lora_get_config (&cmd_data);
    cmd_data.cmd = E_LORA_CMD_INIT;
    lora_send_cmd (&cmd_data);
    xSemaphoreGive(xLoRaSigfoxSem);
#endif
*/
    cmd_data.cmd = E_LORA_CMD_TX;
    memcpy (cmd_data.info.tx.data, buf, len);
    cmd_data.info.tx.len = len;

    if (timeout_ms < 0) {
        // blocking mode
        timeout_ms = portMAX_DELAY;
    }

    xEventGroupClearBits(LoRaEvents, LORA_STATUS_COMPLETED | LORA_STATUS_ERROR | LORA_STATUS_MSG_SIZE);

    // just pass to the LoRa queue
    if (!xQueueSend(xCmdQueue, (void *)&cmd_data, (TickType_t)(timeout_ms / portTICK_PERIOD_MS))) {
        //printf("Q full\n");
        return 0;
    }

    lora_obj.sftx = lora_obj.sf;

    if (timeout_ms != 0) {
        //printf("timeout_ms %d\n", timeout_ms);

        xEventGroupWaitBits(LoRaEvents,
                            LORA_STATUS_COMPLETED | LORA_STATUS_ERROR | LORA_STATUS_MSG_SIZE,
                            pdTRUE,   // clear on exit
                            pdFALSE,  // do not wait for all bits
                            (TickType_t)portMAX_DELAY);
    }

    // calculate the time on air
    lora_obj.tx_time_on_air = Radio.TimeOnAir(MODEM_LORA, len);
    lora_obj.tx_counter += 1;
    lora_obj.tx_frequency = lora_obj.frequency;

    // return the number of bytes sent
    return len;
}

static int32_t lora_recv (byte *buf, uint32_t len, int32_t timeout_ms, uint32_t *port) {
    lora_rx_data_t rx_data = { .port = 1 };

    if (timeout_ms < 0) {
        // blocking mode
        timeout_ms = portMAX_DELAY;
    }

    // if there's a partial packet pending
    if (lora_partial_rx_packet.size > 0) {
        // adjust the len
        uint32_t available_len = lora_partial_rx_packet.size - lora_partial_rx_packet.index;
        if (available_len < len) {
            len = available_len;
        }

        // get the available data
        memcpy(buf, (void *)&lora_partial_rx_packet.data[lora_partial_rx_packet.index], len);
        if (port != NULL) {
            *port = rx_data.port;
        }

        // update the index and size values
        lora_partial_rx_packet.index += len;
        if (lora_partial_rx_packet.index == lora_partial_rx_packet.size) {
            // there's no more data left
            lora_partial_rx_packet.size = 0;
        }
        // return the number of bytes received
        return len;
    } else if (xQueueReceive(xRxQueue, &rx_data, (TickType_t)(timeout_ms / portTICK_PERIOD_MS))) {
#ifdef PHYSEC
        // save the last received packet to restore it on need
        memcpy(&last_rx_pkt, &rx_data, sizeof(lora_rx_data_t));
#endif
        // adjust the len
        if (rx_data.len < len) {
            len = rx_data.len;
        }

        // get the available data
        memcpy(buf, rx_data.data, len);
        if (port != NULL) {
            *port = rx_data.port;
        }

        // copy the remainder to the partial data buffer
        int32_t r_len = rx_data.len - len;
        if (r_len > 0) {
            memcpy((void *)lora_partial_rx_packet.data, &rx_data.data[len], r_len);
            lora_partial_rx_packet.size = r_len;
            lora_partial_rx_packet.index = 0;
            lora_partial_rx_packet.port = rx_data.port;
        }
        // return the number of bytes received
        return len;
    }
    // non-blocking sockects do not thrown timeout errors
    if (timeout_ms == 0) {
        return 0;
    }
    // there's no data available
    return -1;
}

static bool lora_rx_any (void) {
    lora_rx_data_t rx_data;
    if (lora_partial_rx_packet.size > 0) {
        return true;
    } else if (xQueuePeek(xRxQueue, &rx_data, (TickType_t)0)) {
        return true;
    }
    return false;
}

static bool lora_tx_space (void) {
    if (uxQueueSpacesAvailable(xCmdQueue) > 0) {
        return true;
    }
    return false;
}

/******************************************************************************/
// Micro Python bindings; LoRa class

/// \class LoRa - Semtech SX1272 radio driver
static mp_obj_t lora_init_helper(lora_obj_t *self, const mp_arg_val_t *args) {
    lora_cmd_data_t cmd_data;

    cmd_data.info.init.stack_mode = args[0].u_int;
    lora_validate_mode (cmd_data.info.init.stack_mode);

    // we need to know the region first
    if (args[14].u_obj == MP_OBJ_NULL) {
        cmd_data.info.init.region = config_get_lora_region();
        if (cmd_data.info.init.region == 0xff) {
            nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "no region specified and no default found in config block"));
        }
    } else {
        cmd_data.info.init.region = mp_obj_get_int(args[14].u_obj);
    }
    lora_validate_region(cmd_data.info.init.region);
    // we need to do it here in advance for the rest of the validation to work
    lora_obj.region = cmd_data.info.init.region;

    if (args[1].u_obj == MP_OBJ_NULL) {
        switch (cmd_data.info.init.region) {
        case LORAMAC_REGION_AS923:
            cmd_data.info.init.frequency = 923000000;
            break;
        case LORAMAC_REGION_AU915:
        case LORAMAC_REGION_US915:
        case LORAMAC_REGION_US915_HYBRID:
            cmd_data.info.init.frequency = 915000000;
            break;
        case LORAMAC_REGION_EU868:
            cmd_data.info.init.frequency = 868000000;
            break;
        case LORAMAC_REGION_EU433:
            cmd_data.info.init.frequency = 433175000;
            break;
        case LORAMAC_REGION_CN470:
            cmd_data.info.init.frequency = 470000000;
        case LORAMAC_REGION_IN865:
            cmd_data.info.init.frequency = 865000000;
        default:
            break;
        }
    } else {
        cmd_data.info.init.frequency = mp_obj_get_int(args[1].u_obj);
        lora_validate_frequency (cmd_data.info.init.frequency);
    }

    if (args[2].u_obj == MP_OBJ_NULL) {
        switch (cmd_data.info.init.region) {
        case LORAMAC_REGION_AS923:
        case LORAMAC_REGION_AU915:
        case LORAMAC_REGION_US915:
        case LORAMAC_REGION_IN865:
        case LORAMAC_REGION_US915_HYBRID:
            cmd_data.info.init.tx_power = 20;
            break;
        case LORAMAC_REGION_CN470:
        case LORAMAC_REGION_EU868:
            cmd_data.info.init.tx_power = 14;
            break;
        case LORAMAC_REGION_EU433:
            cmd_data.info.init.tx_power = 12;
            break;
        default:
            break;
        }
    } else {
        cmd_data.info.init.tx_power = mp_obj_get_int(args[2].u_obj);
        lora_validate_power (cmd_data.info.init.tx_power);
    }

    cmd_data.info.init.bandwidth = args[3].u_int;
    lora_validate_bandwidth (cmd_data.info.init.bandwidth);

    cmd_data.info.init.sf = args[4].u_int;
    lora_validate_sf(cmd_data.info.init.sf);

    cmd_data.info.init.preamble = args[5].u_int;

    cmd_data.info.init.coding_rate = args[6].u_int;
    lora_validate_coding_rate (cmd_data.info.init.coding_rate);

    cmd_data.info.init.power_mode = args[7].u_int;
    lora_validate_power_mode (cmd_data.info.init.power_mode);

    cmd_data.info.init.txiq = args[8].u_bool;
    cmd_data.info.init.rxiq = args[9].u_bool;

    cmd_data.info.init.adr = args[10].u_bool;
    cmd_data.info.init.public = args[11].u_bool;
    cmd_data.info.init.tx_retries = args[12].u_int;

    cmd_data.info.init.device_class = args[13].u_int;
    lora_validate_device_class(cmd_data.info.init.device_class);

    // send message to the lora task
    cmd_data.cmd = E_LORA_CMD_INIT;
    lora_send_cmd(&cmd_data);

    return mp_const_none;
}

STATIC const mp_arg_t lora_init_args[] = {
    { MP_QSTR_id,                             MP_ARG_INT,   {.u_int  = 0} },
    { MP_QSTR_mode,         MP_ARG_KW_ONLY  | MP_ARG_INT,   {.u_int  = E_LORA_STACK_MODE_LORA} },
    { MP_QSTR_frequency,    MP_ARG_KW_ONLY  | MP_ARG_OBJ,   {.u_obj  = MP_OBJ_NULL} },
    { MP_QSTR_tx_power,     MP_ARG_KW_ONLY  | MP_ARG_OBJ,   {.u_obj  = MP_OBJ_NULL} },
    { MP_QSTR_bandwidth,    MP_ARG_KW_ONLY  | MP_ARG_INT,   {.u_int  = E_LORA_BW_125_KHZ} },
    { MP_QSTR_sf,           MP_ARG_KW_ONLY  | MP_ARG_INT,   {.u_int  = 7} },
    { MP_QSTR_preamble,     MP_ARG_KW_ONLY  | MP_ARG_INT,   {.u_int  = 8} },
    { MP_QSTR_coding_rate,  MP_ARG_KW_ONLY  | MP_ARG_INT,   {.u_int  = E_LORA_CODING_4_5} },
    { MP_QSTR_power_mode,   MP_ARG_KW_ONLY  | MP_ARG_INT,   {.u_int  = E_LORA_MODE_ALWAYS_ON} },
    { MP_QSTR_tx_iq,        MP_ARG_KW_ONLY  | MP_ARG_BOOL,  {.u_bool = false} },
    { MP_QSTR_rx_iq,        MP_ARG_KW_ONLY  | MP_ARG_BOOL,  {.u_bool = false} },
    { MP_QSTR_adr,          MP_ARG_KW_ONLY  | MP_ARG_BOOL,  {.u_bool = false} },
    { MP_QSTR_public,       MP_ARG_KW_ONLY  | MP_ARG_BOOL,  {.u_bool = true} },
    { MP_QSTR_tx_retries,   MP_ARG_KW_ONLY  | MP_ARG_INT,   {.u_int = 2} },
    { MP_QSTR_device_class, MP_ARG_KW_ONLY  | MP_ARG_INT,   {.u_int = CLASS_A} },
    { MP_QSTR_region,       MP_ARG_KW_ONLY  | MP_ARG_OBJ,   {.u_obj = MP_OBJ_NULL} },
    #ifdef PHYSEC
    { MP_QSTR_physec_device_id,              MP_ARG_KW_ONLY  | MP_ARG_INT,   {.u_int = 0x11111111} },
    { MP_QSTR_physec_remote_device_id,       MP_ARG_KW_ONLY  | MP_ARG_INT,   {.u_int = 0x00000000} },
    { MP_QSTR_physec_timeout,      MP_ARG_KW_ONLY  | MP_ARG_INT,   {.u_int = PHYSEC_PROBE_TIMEOUT } },
    { MP_QSTR_physec_initiator,                       MP_ARG_KW_ONLY  | MP_ARG_BOOL,   {.u_bool = false } },
    #endif
};
STATIC mp_obj_t lora_make_new(const mp_obj_type_t *type, mp_uint_t n_args, mp_uint_t n_kw, const mp_obj_t *all_args) {
    // parse args
    mp_map_t kw_args;
    mp_map_init_fixed_table(&kw_args, n_kw, all_args + n_args);
    // parse args
    mp_arg_val_t args[MP_ARRAY_SIZE(lora_init_args)];
    mp_arg_parse_all(n_args, all_args, &kw_args, MP_ARRAY_SIZE(args), lora_init_args, args);

    // setup the object
    lora_obj_t *self = (lora_obj_t *)&lora_obj;
    self->base.type = (mp_obj_t)&mod_network_nic_type_lora;

    // check the peripheral id
    if (args[0].u_int != 0) {
        nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, mpexception_os_resource_not_avaliable));
    }

    // run the constructor if the peripehral is not initialized or extra parameters are given
    if (n_kw > 0 || self->state == E_LORA_STATE_NOINIT) {
        // start the peripheral
        lora_init_helper(self, &args[1]);
        // register it as a network card
        mod_network_register_nic(self);
    }

    #ifdef PHYSEC
    // Set device id
    self->physec_device_id = (uint32_t) args[16].u_int;
    self->physec_remote_device_id = (uint32_t) args[17].u_int;
    self->physec_timeout = (uint32_t) args[18].u_int + toa(self->sf);
    self->physec_initiator = args[19].u_bool;

    // init key per peer list
    peer_key_list_init(&(self->peer_key_list));

    #endif

    return (mp_obj_t)self;
}

STATIC mp_obj_t lora_init(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    // parse args
    mp_arg_val_t args[MP_ARRAY_SIZE(lora_init_args) - 1];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(args), &lora_init_args[1], args);
    return lora_init_helper(pos_args[0], args);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(lora_init_obj, 1, lora_init);

STATIC mp_obj_t lora_join(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    STATIC const mp_arg_t allowed_args[] = {
        { MP_QSTR_activation,     MP_ARG_REQUIRED | MP_ARG_INT, },
        { MP_QSTR_auth,           MP_ARG_REQUIRED | MP_ARG_KW_ONLY | MP_ARG_OBJ, },
        { MP_QSTR_dr,             MP_ARG_KW_ONLY  | MP_ARG_OBJ,                         {.u_obj = mp_const_none}},
        { MP_QSTR_timeout,        MP_ARG_KW_ONLY  | MP_ARG_OBJ,                         {.u_obj = mp_const_none} },
    };
    lora_cmd_data_t cmd_data;

    // check for the correct lora radio mode
    if (lora_obj.stack_mode != E_LORA_STACK_MODE_LORAWAN) {
        nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, mpexception_os_request_not_possible));
    }

    // parse args
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    // get the activation type
    uint32_t activation = args[0].u_int;
    if (activation > E_LORA_ACTIVATION_ABP) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "invalid activation type %d", activation));
    }

    // get the auth details
    mp_obj_t *auth;
    mp_buffer_info_t bufinfo_0, bufinfo_1, bufinfo_2;
    if (activation == E_LORA_ACTIVATION_OTAA) {
        uint32_t auth_len;
        mp_obj_get_array(args[1].u_obj, &auth_len, &auth);
        if (auth_len == 2) {
            mp_get_buffer_raise(auth[0], &bufinfo_1, MP_BUFFER_READ);
            mp_get_buffer_raise(auth[1], &bufinfo_2, MP_BUFFER_READ);
            config_get_lpwan_mac(cmd_data.info.join.u.otaa.DevEui);
        } else {
            mp_get_buffer_raise(auth[0], &bufinfo_0, MP_BUFFER_READ);
            memcpy(cmd_data.info.join.u.otaa.DevEui, bufinfo_0.buf, sizeof(cmd_data.info.join.u.otaa.DevEui));
            mp_get_buffer_raise(auth[1], &bufinfo_1, MP_BUFFER_READ);
            mp_get_buffer_raise(auth[2], &bufinfo_2, MP_BUFFER_READ);
        }
        memcpy(cmd_data.info.join.u.otaa.AppEui, bufinfo_1.buf, sizeof(cmd_data.info.join.u.otaa.AppEui));
        memcpy(cmd_data.info.join.u.otaa.AppKey, bufinfo_2.buf, sizeof(cmd_data.info.join.u.otaa.AppKey));
    } else {
        mp_obj_get_array_fixed_n(args[1].u_obj, 3, &auth);
        mp_get_buffer_raise(auth[1], &bufinfo_0, MP_BUFFER_READ);
        mp_get_buffer_raise(auth[2], &bufinfo_1, MP_BUFFER_READ);
        cmd_data.info.join.u.abp.DevAddr = mp_obj_int_get_truncated(auth[0]);
        memcpy(cmd_data.info.join.u.abp.NwkSKey, bufinfo_0.buf, sizeof(cmd_data.info.join.u.abp.NwkSKey));
        memcpy(cmd_data.info.join.u.abp.AppSKey, bufinfo_1.buf, sizeof(cmd_data.info.join.u.abp.AppSKey));
    }

    // need a way to indicate an invalid data rate so the default approach is used
    uint32_t dr = DR_0;
    switch (lora_obj.region) {
    case LORAMAC_REGION_AS923:
        dr = DR_2;
        break;
    case LORAMAC_REGION_AU915:
        dr = DR_6;
        break;
    case LORAMAC_REGION_US915:
    case LORAMAC_REGION_US915_HYBRID:
        dr = DR_4;
        break;
    case LORAMAC_REGION_CN470:
    case LORAMAC_REGION_EU868:
    case LORAMAC_REGION_EU433:
    case LORAMAC_REGION_IN865:
        dr = DR_5;
        break;
    default:
        break;
    }

    // get the data rate
    if (args[2].u_obj != mp_const_none) {
        dr = mp_obj_get_int(args[2].u_obj);
        switch (lora_obj.region) {
        case LORAMAC_REGION_AS923:
            if (dr != DR_2) {
                goto dr_error;
            }
            break;
        case LORAMAC_REGION_AU915:
            if (dr != DR_0 && dr != DR_6) {
                goto dr_error;
            }
            break;
        case LORAMAC_REGION_US915:
            if (dr != DR_0 && dr != DR_4) {
                goto dr_error;
            }
            break;
        case LORAMAC_REGION_US915_HYBRID:
            if (dr != DR_0 && dr != DR_4) {
                goto dr_error;
            }
            break;
        case LORAMAC_REGION_EU433:
        case LORAMAC_REGION_CN470:
        case LORAMAC_REGION_EU868:
            if (dr > DR_5) {
                goto dr_error;
            }
            break;
        default:
            break;
        }
    }
    cmd_data.info.join.otaa_dr = dr;

    // get the timeout
    int32_t timeout = INT32_MAX;
    if (args[3].u_obj != mp_const_none) {
        timeout = mp_obj_get_int(args[3].u_obj);
    }

    // send a join request message
    cmd_data.info.join.activation = activation;
    cmd_data.cmd = E_LORA_CMD_JOIN;
    lora_send_cmd(&cmd_data);

    if (timeout > 0) {
        while (!lora_obj.joined && timeout >= 0) {
            mp_hal_delay_ms(LORA_JOIN_WAIT_MS);
            timeout -= LORA_JOIN_WAIT_MS;
        }
        if (timeout <= 0) {
            nlr_raise(mp_obj_new_exception_msg(&mp_type_TimeoutError, "timed out"));
        }
    }
    return mp_const_none;

dr_error:
    nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "invalid join data rate %d", dr));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(lora_join_obj, 1, lora_join);

STATIC mp_obj_t lora_join_multicast_group (mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_mcAddress,    MP_ARG_REQUIRED | MP_ARG_INT, {.u_int  = -1}         },
        { MP_QSTR_mcNwkKey,     MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_mcAppKey,     MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
    };

    // parse args
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(args), allowed_args, args);

    mp_buffer_info_t bufinfo_0, bufinfo_1;
    mp_get_buffer_raise(args[1].u_obj, &bufinfo_0, MP_BUFFER_READ);
    mp_get_buffer_raise(args[2].u_obj, &bufinfo_1, MP_BUFFER_READ);

    MulticastParams_t *channelParam = m_new_obj(MulticastParams_t);
    channelParam->Next = NULL;
    channelParam->DownLinkCounter = 0;
    channelParam->Address = args[0].u_int;
    memcpy(channelParam->NwkSKey, bufinfo_0.buf, sizeof(channelParam->NwkSKey));
    memcpy(channelParam->AppSKey, bufinfo_1.buf, sizeof(channelParam->AppSKey));

    if (LoRaMacMulticastChannelLink(channelParam) == LORAMAC_STATUS_OK) {
        return mp_const_true;
    }

    return mp_const_false;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(lora_join_multicast_group_obj, 0, lora_join_multicast_group);

STATIC mp_obj_t lora_leave_multicast_group (mp_obj_t self_in, mp_obj_t multicast_addr_obj) {
    uint32_t mcAddr = mp_obj_get_int(multicast_addr_obj);
    MulticastParams_t *channelParam = LoRaMacMulticastGetChannel(mcAddr);
    if (LoRaMacMulticastChannelUnlink(channelParam) == LORAMAC_STATUS_OK) {
        m_del_obj(MulticastParams_t, channelParam);
        return mp_const_true;
    }
    return mp_const_false;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(lora_leave_multicast_group_obj, lora_leave_multicast_group);

STATIC mp_obj_t lora_compliance_test(mp_uint_t n_args, const mp_obj_t *args) {
    // get
    if (n_args == 1) {
        static const qstr lora_compliance_info_fields[] = {
            MP_QSTR_enabled, MP_QSTR_running, MP_QSTR_state, MP_QSTR_tx_confirmed,
            MP_QSTR_downlink_counter, MP_QSTR_link_check, MP_QSTR_demod_margin,
            MP_QSTR_nbr_gateways
        };

        mp_obj_t compliance_tuple[8];
        compliance_tuple[0] = mp_obj_new_bool(lora_obj.ComplianceTest.Enabled);
        compliance_tuple[1] = mp_obj_new_bool(lora_obj.ComplianceTest.Running);
        compliance_tuple[2] = mp_obj_new_int(lora_obj.ComplianceTest.State);
        compliance_tuple[3] = mp_obj_new_bool(lora_obj.ComplianceTest.IsTxConfirmed);
        compliance_tuple[4] = mp_obj_new_int(lora_obj.ComplianceTest.DownLinkCounter);
        compliance_tuple[5] = mp_obj_new_bool(lora_obj.ComplianceTest.LinkCheck);
        compliance_tuple[6] = mp_obj_new_int(lora_obj.ComplianceTest.DemodMargin);
        compliance_tuple[7] = mp_obj_new_int(lora_obj.ComplianceTest.NbGateways);

        return mp_obj_new_attrtuple(lora_compliance_info_fields, 8, compliance_tuple);
    } else {    // set
        if (mp_obj_is_true(args[1])) {  // enable or disable
            lora_obj.ComplianceTest.Enabled = true;
        } else {
            lora_obj.ComplianceTest.Enabled = false;
            lora_obj.ComplianceTest.IsTxConfirmed = false;
            lora_obj.ComplianceTest.DownLinkCounter = 0;
            lora_obj.ComplianceTest.Running = false;

            // set adr back to its original value
            MibRequestConfirm_t mibReq;
            mibReq.Type = MIB_ADR;
            mibReq.Param.AdrEnable = lora_obj.adr;
            LoRaMacMibSetRequestConfirm(&mibReq);
        }

        if (n_args > 2) {
            // state
            lora_obj.ComplianceTest.State = mp_obj_get_int(args[2]);

            if (n_args > 3) {
                // link check
                if (mp_obj_is_true(args[3])) {
                    lora_obj.ComplianceTest.LinkCheck = true;
                } else {
                    lora_obj.ComplianceTest.LinkCheck = false;
                }
            }
        }

        return mp_const_none;
    }
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(lora_compliance_test_obj, 1, 4, lora_compliance_test);

STATIC mp_obj_t lora_tx_power (mp_uint_t n_args, const mp_obj_t *args) {
    lora_obj_t *self = args[0];
    if (n_args == 1) {
        return mp_obj_new_int(self->tx_power);
    } else {
        lora_cmd_data_t cmd_data;
        uint8_t power = mp_obj_get_int(args[1]);
        lora_validate_power(power);
        lora_get_config (&cmd_data);
        cmd_data.info.init.tx_power = power;
        cmd_data.cmd = E_LORA_CMD_INIT;
        lora_send_cmd (&cmd_data);
        return mp_const_none;
    }
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(lora_tx_power_obj, 1, 2, lora_tx_power);

STATIC mp_obj_t lora_coding_rate (mp_uint_t n_args, const mp_obj_t *args) {
    lora_obj_t *self = args[0];
    if (n_args == 1) {
        return mp_obj_new_int(self->coding_rate);
    } else {
        lora_cmd_data_t cmd_data;
        uint8_t coding_rate = mp_obj_get_int(args[1]);
        lora_validate_coding_rate(coding_rate);
        lora_get_config (&cmd_data);
        cmd_data.info.init.coding_rate = coding_rate;
        cmd_data.cmd = E_LORA_CMD_INIT;
        lora_send_cmd (&cmd_data);
        return mp_const_none;
    }
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(lora_coding_rate_obj, 1, 2, lora_coding_rate);

STATIC mp_obj_t lora_preamble (mp_uint_t n_args, const mp_obj_t *args) {
    lora_obj_t *self = args[0];
    if (n_args == 1) {
        return mp_obj_new_int(self->preamble);
    } else {
        lora_cmd_data_t cmd_data;
        uint8_t preamble = mp_obj_get_int(args[1]);
        lora_get_config (&cmd_data);
        cmd_data.info.init.preamble = preamble;
        cmd_data.cmd = E_LORA_CMD_INIT;
        lora_send_cmd (&cmd_data);
        return mp_const_none;
    }
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(lora_preamble_obj, 1, 2, lora_preamble);

STATIC mp_obj_t lora_bandwidth (mp_uint_t n_args, const mp_obj_t *args) {
    lora_obj_t *self = args[0];
    if (n_args == 1) {
        return mp_obj_new_int(self->bandwidth);
    } else {
        lora_cmd_data_t cmd_data;
        uint8_t bandwidth = mp_obj_get_int(args[1]);
        lora_validate_bandwidth(bandwidth);
        lora_get_config (&cmd_data);
        cmd_data.info.init.bandwidth = bandwidth;
        cmd_data.cmd = E_LORA_CMD_INIT;
        lora_send_cmd (&cmd_data);
        return mp_const_none;
    }
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(lora_bandwidth_obj, 1, 2, lora_bandwidth);

STATIC mp_obj_t lora_frequency (mp_uint_t n_args, const mp_obj_t *args) {
    lora_obj_t *self = args[0];
    if (n_args == 1) {
        return mp_obj_new_int(self->frequency);
    } else {
        lora_cmd_data_t cmd_data;
        uint32_t frequency = mp_obj_get_int(args[1]);
        lora_validate_frequency(frequency);
        lora_get_config (&cmd_data);
        cmd_data.info.init.frequency = frequency;
        cmd_data.cmd = E_LORA_CMD_INIT;
        lora_send_cmd (&cmd_data);
        return mp_const_none;
    }
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(lora_frequency_obj, 1, 2, lora_frequency);

STATIC mp_obj_t lora_sf (mp_uint_t n_args, const mp_obj_t *args) {
    lora_obj_t *self = args[0];
    if (n_args == 1) {
        return mp_obj_new_int(self->sf);
    } else {
        lora_cmd_data_t cmd_data;
        uint8_t sf = mp_obj_get_int(args[1]);
        lora_validate_sf(sf);
#ifdef PHYSEC
        self->physec_timeout = PHYSEC_PROBE_TIMEOUT + toa(sf);
#endif
        lora_get_config (&cmd_data);
        cmd_data.info.init.sf = sf;
        cmd_data.cmd = E_LORA_CMD_INIT;
        lora_send_cmd (&cmd_data);
        return mp_const_none;
    }
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(lora_sf_obj, 1, 2, lora_sf);

STATIC mp_obj_t lora_physec_device_id (mp_uint_t n_args, const mp_obj_t *args) {
    lora_obj_t *self = args[0];
    if (n_args == 1) {
        return mp_obj_new_int(self->physec_device_id);
    } else {
        self->physec_device_id = (uint32_t) mp_obj_get_int(args[1]);
        return mp_const_none;
    }
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(lora_physec_device_id_obj, 1, 2, lora_physec_device_id);

STATIC mp_obj_t lora_physec_remote_device_id (mp_uint_t n_args, const mp_obj_t *args) {
    lora_obj_t *self = args[0];
    if (n_args == 1) {
        return mp_obj_new_int(self->physec_remote_device_id);
    } else {
        self->physec_remote_device_id = (uint32_t) mp_obj_get_int(args[1]);
        return mp_const_none;
    }
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(lora_physec_remote_device_id_obj, 1, 2, lora_physec_remote_device_id);

STATIC mp_obj_t lora_power_mode(mp_uint_t n_args, const mp_obj_t *args) {
    lora_obj_t *self = args[0];
    lora_cmd_data_t cmd_data;

    if (n_args == 1) {
        return mp_obj_new_int(self->pwr_mode);
    } else {
        uint8_t pwr_mode = mp_obj_get_int(args[1]);
        lora_validate_power_mode(pwr_mode);
        if (pwr_mode == E_LORA_MODE_ALWAYS_ON) {
            cmd_data.cmd = E_LORA_CMD_WAKE_UP;
        } else {
            cmd_data.cmd = E_LORA_CMD_SLEEP;
        }
        self->pwr_mode = pwr_mode;
        lora_send_cmd (&cmd_data);
        return mp_const_none;
    }
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(lora_power_mode_obj, 1, 2, lora_power_mode);

STATIC mp_obj_t lora_stats(mp_obj_t self_in) {
    lora_obj_t *self = self_in;
    float snr;

    static const qstr lora_stats_info_fields[] = {
        MP_QSTR_rx_timestamp, MP_QSTR_rssi, MP_QSTR_snr, MP_QSTR_sfrx, MP_QSTR_sftx,
        MP_QSTR_tx_trials, MP_QSTR_tx_power, MP_QSTR_tx_time_on_air, MP_QSTR_tx_counter,
        MP_QSTR_tx_frequency
    };

    if (self->snr & 0x80)  { // the SNR sign bit is 1
        // invert and divide by 4
        snr = ((~self->snr + 1 ) & 0xFF) / 4;
        snr = -snr;
    } else {
        // divide by 4
        snr = (self->snr & 0xFF) / 4;
    }

    mp_obj_t stats_tuple[10];
    stats_tuple[0] = mp_obj_new_int_from_uint(self->rx_timestamp);
    stats_tuple[1] = mp_obj_new_int(self->rssi);
    stats_tuple[2] = mp_obj_new_float(snr);
    stats_tuple[3] = mp_obj_new_int(self->sfrx);
    stats_tuple[4] = mp_obj_new_int(self->sftx);
    stats_tuple[5] = mp_obj_new_int(self->tx_trials);
    stats_tuple[6] = mp_obj_new_int(self->tx_power);
    stats_tuple[7] = mp_obj_new_int(self->tx_time_on_air);
    stats_tuple[8] = mp_obj_new_int(self->tx_counter);
    stats_tuple[9] = mp_obj_new_int(self->tx_frequency);

    return mp_obj_new_attrtuple(lora_stats_info_fields, sizeof(stats_tuple) / sizeof(stats_tuple[0]), stats_tuple);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(lora_stats_obj, lora_stats);

STATIC mp_obj_t lora_has_joined(mp_obj_t self_in) {
    lora_obj_t *self = self_in;
    return self->joined ? mp_const_true : mp_const_false;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(lora_has_joined_obj, lora_has_joined);

STATIC mp_obj_t lora_add_channel (mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_index,        MP_ARG_REQUIRED | MP_ARG_INT },
        { MP_QSTR_frequency,    MP_ARG_REQUIRED | MP_ARG_KW_ONLY  | MP_ARG_INT },
        { MP_QSTR_dr_min,       MP_ARG_REQUIRED | MP_ARG_KW_ONLY  | MP_ARG_INT },
        { MP_QSTR_dr_max,       MP_ARG_REQUIRED | MP_ARG_KW_ONLY  | MP_ARG_INT },
    };
    lora_cmd_data_t cmd_data;

    // parse args
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(args), allowed_args, args);

    uint32_t index = args[0].u_int;
    lora_validate_channel(index);

    uint32_t frequency = args[1].u_int;
    uint32_t dr_min = args[2].u_int;
    uint32_t dr_max = args[3].u_int;
    if (dr_min > dr_max || !lora_validate_data_rate(dr_min) || !lora_validate_data_rate(dr_max)) {
        goto error;
    }

    cmd_data.info.channel.index = index;
    cmd_data.info.channel.frequency = frequency;
    cmd_data.info.channel.dr_min = dr_min;
    cmd_data.info.channel.dr_max = dr_max;
    cmd_data.info.channel.add = true;
    cmd_data.cmd = E_LORA_CMD_CONFIG_CHANNEL;
    lora_send_cmd (&cmd_data);

    return mp_const_none;

error:
    nlr_raise(mp_obj_new_exception_msg(&mp_type_ValueError, mpexception_value_invalid_arguments));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(lora_add_channel_obj, 1, lora_add_channel);

STATIC mp_obj_t lora_remove_channel (mp_obj_t self_in, mp_obj_t idx) {
    lora_cmd_data_t cmd_data;

    uint32_t index = mp_obj_get_int(idx);
    lora_validate_channel(index);

    cmd_data.info.channel.index = index;
    cmd_data.info.channel.add = false;
    cmd_data.cmd = E_LORA_CMD_CONFIG_CHANNEL;
    lora_send_cmd (&cmd_data);

    // return the number of bytes written
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(lora_remove_channel_obj, lora_remove_channel);

STATIC mp_obj_t lora_mac(mp_obj_t self_in) {
    uint8_t mac[8];
    config_get_lpwan_mac(mac);
    return mp_obj_new_bytes((const byte *)mac, sizeof(mac));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(lora_mac_obj, lora_mac);

/// \method callback(trigger, handler, arg)
STATIC mp_obj_t lora_callback(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    STATIC const mp_arg_t allowed_args[] = {
        { MP_QSTR_trigger,      MP_ARG_REQUIRED | MP_ARG_OBJ,   },
        { MP_QSTR_handler,      MP_ARG_OBJ,                     {.u_obj = mp_const_none} },
        { MP_QSTR_arg,          MP_ARG_OBJ,                     {.u_obj = mp_const_none} },
    };

    // parse arguments
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(args), allowed_args, args);
    lora_obj_t *self = pos_args[0];

    // enable the callback
    if (args[0].u_obj != mp_const_none && args[1].u_obj != mp_const_none) {
        self->trigger = mp_obj_get_int(args[0].u_obj);
        self->handler = args[1].u_obj;
        mp_irq_add(self, args[1].u_obj);
        if (args[2].u_obj == mp_const_none) {
            self->handler_arg = self;
        } else {
            self->handler_arg = args[2].u_obj;
        }
    } else {
        self->trigger = 0;
        mp_irq_remove(self);
        INTERRUPT_OBJ_CLEAN(self);
    }

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(lora_callback_obj, 1, lora_callback);

STATIC mp_obj_t lora_events(mp_obj_t self_in) {
    lora_obj_t *self = self_in;

    int32_t events = self->events;
    self->events = 0;
    return mp_obj_new_int(events);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(lora_events_obj, lora_events);

STATIC mp_obj_t lora_ischannel_free(mp_obj_t self_in, mp_obj_t rssi, mp_obj_t time_ms) {
    lora_obj_t *self = self_in;

    // probably we could listen for 2 symbols (bits) for the current Lora settings (freq, bw, sf)
    if (Radio.IsChannelFree(MODEM_LORA, self->frequency, mp_obj_get_int(rssi), mp_obj_get_int(time_ms))) {
        return mp_const_true;
    }
    return mp_const_false;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(lora_ischannel_free_obj, lora_ischannel_free);

STATIC mp_obj_t lora_set_battery_level(mp_obj_t self_in, mp_obj_t battery) {
    BoardSetBatteryLevel(mp_obj_get_int(battery));
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(lora_set_battery_level_obj, lora_set_battery_level);

STATIC mp_obj_t lora_nvram_save (mp_obj_t self_in) {
    LoRaMacRegion_t region = 0xFF;
    modlora_nvs_get_uint(E_LORA_NVS_ELE_REGION, &region);
    // if the region doesn't match, erase the previous stored data
    if (region != lora_obj.region) {
        lora_nvram_erase(NULL);
    }
    LoRaMacNvsSave();
    modlora_nvs_set_uint(E_LORA_NVS_ELE_REGION, (uint32_t)lora_obj.region);
    modlora_nvs_set_uint(E_LORA_NVS_ELE_JOINED, (uint32_t)lora_obj.joined);
    if (ESP_OK != nvs_commit(modlora_nvs_handle)) {
        nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, mpexception_os_operation_failed));
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(lora_nvram_save_obj, lora_nvram_save);

STATIC mp_obj_t lora_nvram_restore (mp_obj_t self_in) {
    uint32_t joined = 0;
    LoRaMacRegion_t region;
    lora_cmd_data_t cmd_data;

    if (modlora_nvs_get_uint(E_LORA_NVS_ELE_JOINED, &joined)) {
        lora_obj.joined = joined;
        if (joined) {
            if (modlora_nvs_get_uint(E_LORA_NVS_ELE_REGION, &region)) {
                // only restore from NVRAM if the region matches
                if (region == lora_obj.region) {
                    lora_get_config (&cmd_data);
                    cmd_data.cmd = E_LORA_CMD_INIT;
                    lora_send_cmd (&cmd_data);
                } else {
                    // erase the previous NVRAM data
                    lora_nvram_erase(NULL);
                    lora_obj.joined = false;
                }
            }
        }
    } else {
        lora_obj.joined = false;
    }

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(lora_nvram_restore_obj, lora_nvram_restore);

STATIC mp_obj_t lora_nvram_erase (mp_obj_t self_in) {
    if (ESP_OK != nvs_erase_all(modlora_nvs_handle)) {
        nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, mpexception_os_operation_failed));
    }
    nvs_commit(modlora_nvs_handle);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(lora_nvram_erase_obj, lora_nvram_erase);

// return time-on-air (milisec) for the current Lora settings, specifying pack_len
STATIC mp_obj_t lora_airtime (mp_obj_t self_in, mp_obj_t pack_len_obj) {
    int len = mp_obj_get_int(pack_len_obj);
    return mp_obj_new_int(Radio.TimeOnAir(MODEM_LORA, len));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(lora_airtime_obj, lora_airtime);

STATIC mp_obj_t lora_reset (mp_obj_t self_in) {

    lora_obj_t* self = (lora_obj_t*)self_in;
    lora_cmd_data_t cmd_data;

    MP_THREAD_GIL_EXIT();
    //Reset Command Queue
    while(!xQueueReset(xCmdQueue))
    {
        // Try again
        vTaskDelay (100 / portTICK_PERIOD_MS);
    }

    self->reset = true;

    lora_get_config (&cmd_data);
    cmd_data.cmd = E_LORA_CMD_INIT;
    lora_send_cmd (&cmd_data);

    xEventGroupWaitBits(LoRaEvents,
                                  LORA_STATUS_RESET_DONE,
                                  pdTRUE,   // clear on exit
                                  pdTRUE,  // do not wait for all bits
                                  (TickType_t)portMAX_DELAY);

    MP_THREAD_GIL_ENTER();

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(lora_reset_obj, lora_reset);

#ifdef PHYSEC

/****************************
 *  PHYSEC utils functions  *
 ****************************/

// -- Logging 
/*
static void
logging(const uint8_t *buf, const size_t size)
{
    mp_obj_vfs_posix_file_t *file = mp_vfs_posix_file_open(&mp_type_vfs_posix_textio, mp_obj_new_str(LOGFILE, strlen(LOGFILE)), mp_obj_new_str("a+", 3));

    int err;
    mp_type_vfs_posix_fileio.protocol.write(file, buf, size, &err);


}
*/

/**
 * @brief Calculate bit mismatch between K1 and K2 and fill KM bits with mismatches bits (1 for a mismatch, 0 otherwise)
 * 
 * @return uint32_t     the mismatch number
 */
static uint32_t
calculate_mismatch(PHYSEC_Key *K1, PHYSEC_Key *K2, PHYSEC_Key *KM)
{
    uint32_t nmismatch = 0;
    for (int i=0; i<PHYSEC_KEY_SIZE; i++)
    {
        uint8_t cur_k1_bit = (K1->key[i/8] >> (7-(i%8))) & 0x1;
        uint8_t cur_k2_bit = (K2->key[i/8] >> (7-(i%8))) & 0x1;
        if ( cur_k1_bit != cur_k2_bit)
        {
            KM->key[i/8] = KM->key[i/8] | (1 << (7-(i%8)));
            nmismatch++;
        }
    }

    return nmismatch;
}

static void
display_key_bits(const PHYSEC_Key *K)
{
    for (int i=0; i<PHYSEC_KEY_SIZE_BYTES; i++)
    {
        for (int j=7; j>=0; j--)
        {
            if ((K->key[i] >> j) & 0x1)
                printf("1");
            else
                printf("0");
        }
    }
    printf("\n");
}

static void
display_m_vals(const int8_t *m_vals, uint16_t len)
{
    printf("[ \n");
    for (uint16_t i=0; i<len; i++)
    {
        printf("\t%d\n", m_vals[i]);
    }
    printf("]\n");
}

// -- Packet management

/**
 * @brief Returns an approximate toa (for a 6 bytes packet) for a specified SF
 *
 * @param sf    spreading factor used
 * @return uint16_t
 */
static inline uint16_t
toa(uint8_t sf)
{
    switch(sf) {
        case 7:
            return 36;
        case 8:
            return 62;
        case 9:
            return 124;
        case 10:
            return 248;
        case 11:
            return 496;
        case 12:
            return 991;
    }
    return 0;
}

/**
 * @brief Reschedule the last received lora packet. Used to ignore unwanted packet (for example non-PHYSEC ones)
 * 
 */
static void
reschdl_last_rx_pkt(void)
{
    lora_rx_data_t d = { 0 };
    if (memcmp(&last_rx_pkt, &d, sizeof(d)) != 0)
    {
        printf("RESCHEDULED\n");
        xQueueSend(xRxQueue, &last_rx_pkt, 0);
        memset(&last_rx_pkt, 0, sizeof(lora_rx_data_t));
    }
}

/**
 * \brief listen on lora interface for incoming packets and returns the first PHYSEC_Packet it catch
 *
 * \param retbuffer the buffer to be filled with the payload embedded in the physec packet
 * \param len       the size of the retbuffer (it should be equal to the max payload size)
 * \param timeout   time before aborting the reception
 * \param start_time pointer to int, will contains the start time just after packet reception
 * \param rssi      pointer to uint8_t, will contains the rssi of the physec packet
 * \return PHYSEC_PacketType
 */
static PHYSEC_PacketType
PHYSEC_wait_keygen_packet(const PHYSEC_Sync *sync, uint8_t *retbuffer, size_t len, int32_t timeout, uint32_t *start_time, PHYSEC_MeasureType mt, int8_t *m_val)
{
    if (len < PHYSEC_MAX_PAYLOAD_SIZE)
        return PHYSEC_PT_NONE;

    uint32_t start = mp_hal_ticks_ms();
    uint32_t wtime = 0;
    PHYSEC_Packet pkt = { 0 };
    // wait packet
    while (timeout == -1 || (wtime = mp_hal_ticks_ms()-start) < timeout)
    {
        int n = 0;
        memset((uint8_t*)&pkt, 0, sizeof(pkt));
        if ((n = lora_recv((uint8_t*)&pkt, sizeof(pkt), (timeout == -1) ? timeout : timeout-wtime, NULL)) > 0)
        {
            physec_log(3, "<<< LORA PACKET INCOMMING\n");
            physec_verb_statement(3, hexdump((uint8_t*) &pkt, n));
            physec_log(3, "\n");

            if (start_time != NULL)
                *start_time = mp_hal_ticks_ms();
            if (pkt.identifier != PHYSEC_KG_PKT_IDENTIFIER)
                //goto end;
                continue;

            if (memcmp(pkt.dev_id, sync->dev_id, PHYSEC_DEV_ID_LEN) != 0)
                //goto end;
                continue;


            switch (pkt.type)
            {
                case PHYSEC_PT_PROBE:
                {
                    memcpy(retbuffer, &(pkt.payload), sizeof(PHYSEC_Probe));
                    if (m_val != NULL)
                    {
                        switch (mt)
                        {
                            case PHYSEC_MT_RSSI:
                            {
                                if (lora_obj.rssi < INT8_MIN)
                                    *m_val = INT8_MAX;
                                else
                                    *m_val = abs(lora_obj.rssi);
                                break;
                            }
                            case PHYSEC_MT_SNR:
                            {
                                if (lora_obj.snr < INT8_MIN)
                                    *m_val = INT8_MAX;
                                else
                                    *m_val = abs(lora_obj.snr);

                                break;
                            }
                            default:
                            {
                                *m_val = 0;
                                break;
                            }
                        }
                    }
                    return PHYSEC_PT_PROBE;
                }
                case PHYSEC_PT_KEYGEN:
                {
                    memcpy(retbuffer, &(pkt.payload), sizeof(PHYSEC_KeyGen));
                    return PHYSEC_PT_KEYGEN;
                }
                case PHYSEC_PT_RESET:
                {
                    memcpy(retbuffer, &(pkt.payload), PHYSEC_DEV_ID_LEN);
                    return PHYSEC_PT_RESET;
                }
                default:
                    break;
            }
//end:
            //reschdl_last_rx_pkt();
        }
    }

    return PHYSEC_PT_NONE;
}
 
/**
 * \brief wait for the reception of a PHYSEC probe destinated to id
 *
 * \param id        the id of the device calling the function
 * \param rssi      the rssi value of the received probe
 * \param duration  time waiting for the probe
 * \param timeout   the time before exiting the listenning mode
 * \return the probe number of the last received probe
 */
static uint16_t
wait_probe(const PHYSEC_Sync *sync, PHYSEC_MeasureType mt, int8_t *rssi, uint32_t *duration, int32_t timeout)
{
    uint32_t start = mp_hal_ticks_ms();
    if (duration != NULL)
        *duration = timeout;
    uint8_t buf[PHYSEC_MAX_PAYLOAD_SIZE] = { 0 };
    uint8_t pt = PHYSEC_PT_NONE;
    int32_t wtime = 0;
    uint16_t cnt = 0;
    int8_t rss = 0;
    while ( true )
    {
        wtime = mp_hal_ticks_ms()-start;
        if (wtime >= timeout)
        {
            if (duration != NULL)
                *duration = timeout;
            break;
        }
        if ((pt = PHYSEC_wait_keygen_packet(sync, buf, sizeof(buf), timeout-wtime, NULL, mt, &rss)) == PHYSEC_PT_PROBE)
        {
            PHYSEC_Probe *p = (PHYSEC_Probe *) buf;
            if (duration != NULL)
                *duration = mp_hal_ticks_ms()-start;
            if (rssi != NULL)
                *rssi = rss;
            cnt = p->cnt;
            break;
        }

    }

    return cnt;
}

#define AES_BLOCK_SIZE PHYSEC_KEY_SIZE
/**
 * @brief Encrypt a message using PHYSEC generated key
 *
 * @param K       the PHYSEC Key
 * @param buf     the message to encrypt
 * @param size    the size of the data to encrypt
 * @param diggest the buffer to fill with the encrypted data (must be padded to a block size and greater or equal to buf size)
 * @param diggest_len the diggest new length (must be set to diggest max size and superior to buffer size)
 */
static int32_t PHYSEC_encrypt(PHYSEC_Key *K, const uint8_t *buf, const uint32_t size, uint8_t *diggest, uint32_t *diggest_len)
{
    if (*diggest_len < size)
        return -1;
    for (uint32_t i=0; i<size; i++)
    {
        diggest[i] = buf[i] ^ K->key[i%PHYSEC_KEY_SIZE_BYTES];      // for now just XOR for testing
    }
    *diggest_len = size;
    return size;
}

/**
 * @brief Decrypt a message using PHYSEC generated key
 *
 * @param K       the PHYSEC Key
 * @param buf     the encrypted message
 * @param size    the size of the data to encrypt (must be block size aligned)
 * @param cleartext the buffer to fill with the decrypted data (size must be block size aligned and greater or equal to buf size)
 * @param cleartext_len the cleartext new length (must be set to cleartext max size and superior to buffer size)
 */
static int32_t PHYSEC_decrypt(PHYSEC_Key *K, const uint8_t *buf, const uint32_t size, uint8_t *cleartext, uint32_t *cleartext_len)
{
    return PHYSEC_encrypt(K, buf, size, cleartext, cleartext_len);
}

/**
 * @brief Wait for a PHYSEC data packet and returns the offset (in the data field) for the encrypted part
 * 
 * @param retbuffer a pointer to a buffer to hold the packet
 * @param len       a pointer to the buffer size (must be equal to buffer max size), will be modified with the new len
 * @param boff      a begin offset pointer 
 * @param eoff      an end offset pointer
 * @param timeout   the timeout before aborting
 * @return PHYSEC_DataCipher    the type of ciphering
 */
static PHYSEC_DataCipher
PHYSEC_wait_data_packet(uint8_t *retbuffer, size_t *len, uint8_t *boff, uint8_t *eoff, uint32_t timeout)
{
if (*len < PHYSEC_DATA_PKT_MAX_SIZE)
        return PHYSEC_DT_NONE;

    uint32_t start = mp_hal_ticks_ms();
    uint32_t wtime = 0;
    uint8_t buf[sizeof(PHYSEC_DataPkt)];
    // wait packet
    while (timeout == -1 || (wtime = mp_hal_ticks_ms()-start) < timeout)
    {
        memset(buf, 0, sizeof(PHYSEC_DataPkt));
        if (lora_recv(buf, sizeof(buf), (timeout == -1) ? timeout : timeout-wtime, NULL) > 0)
        {
            PHYSEC_DataPkt *pkt = (PHYSEC_DataPkt*) buf;
            if (pkt->identifier != PHYSEC_DT_PKT_IDENTIFIER)
            {
                continue;
            }

            if (pkt->ctrl.size < *len)
                *len = pkt->ctrl.size;      // extra bytes are lost for now, could add a cache buffer for
                                            // remaining data or achieve to let data in the transceiver FIFO
            memcpy(retbuffer, pkt->data, *len);

            switch (pkt->ctrl.type)
            {
                case PHYSEC_DT_FULL:
                {
                    *boff = 0;
                    *eoff = pkt->ctrl.size;
                    break;
                }
                case PHYSEC_DT_PARTIAL:
                {
                    *boff = pkt->ctrl.boff;
                    *eoff = pkt->ctrl.eoff;
                    break;
                }
                default:
                    return PHYSEC_DT_NONE;

            }

            return pkt->ctrl.type;
        }
    }

    return PHYSEC_DT_NONE;
}

/**
 * @brief Encrypt a payload using a PHYSEC Key and send it using LoRa
 * 
 * @param K         the Key generated with PHYSEC algorithm
 * @param msg       the payload to encrypt
 * @param len       the max size of the payload
 * @param timeout   timeout before aborting
 * @return int32_t  size of message sent (not the size of the PHYSEC data packet)
 */
static int32_t
PHYSEC_send(PHYSEC_Key *K, const uint8_t *msg, const uint32_t len, uint32_t timeout)
{
    if (len > 0)
    {
        PHYSEC_DataPkt pkt = {
            .identifier = PHYSEC_DT_PKT_IDENTIFIER,
            .ctrl = {
                .type = PHYSEC_DT_FULL,
                .boff = 0,
                .eoff = len,
                .size = len
            },
            .data = { 0 }
        };
        uint32_t buf_len = PHYSEC_DATA_PKT_MAX_SIZE;
        PHYSEC_encrypt(K, msg, len, (uint8_t*) &(pkt.data), &buf_len);
        pkt.ctrl.eoff = buf_len;
        pkt.ctrl.size = buf_len;
        uint32_t sent_len = sizeof(pkt)-(PHYSEC_DATA_PKT_MAX_SIZE-buf_len);

        int32_t ret = lora_send((const uint8_t*)&pkt, sent_len, timeout);
        if (ret == sent_len)
            return buf_len;

        return ret;
    }
    return 0;
}

/**
 * @brief Receive some data from lora interface, and decrypt it if it's a PHYSEC data packet
 * 
 * @param K         the Key generated with PHYSEC algorithm
 * @param msg       a buffer to store the received payload
 * @param len       the max size of the buffer
 * @param timeout   time before aborting
 * @return int32_t  size of message received (not the size of the PHYSEC data packet)
 */
static int32_t
PHYSEC_recv(PHYSEC_Key *K, uint8_t *msg, const uint32_t len, uint32_t timeout)
{
    if (len > 0)
    {
        uint32_t newlen = sizeof(PHYSEC_DataPkt);
        uint8_t buf[newlen];
        memset(buf, 0, newlen);

        uint8_t boff;
        uint8_t eoff;
        switch (PHYSEC_wait_data_packet(buf, &newlen, &boff, &eoff, timeout))
        {
            case PHYSEC_DT_FULL:
            {
                physec_log(1, "### PHYSEC FULL CIPHERED PAYLOAD\n");
                break;
            }
            case PHYSEC_DT_PARTIAL:
            {
                physec_log(1, "### PHYSEC PARTIAL CIPHERED PAYLOAD\n");
                break;
            }
            default:
                return 0;
        }
        uint32_t clear_len = newlen;
        PHYSEC_decrypt(K, &(buf[boff]), newlen, msg, &clear_len);

        return clear_len;
    }
    return 0;
}


/**
 * @brief Fill MAC field of a PHYSEC_KeyGen struct regarding the content of the KeyGen payload (ID + CS_VEC)
 *
 * @param K  the key to use to generate the MAC
 * @param kg the PHYSEC_KeyGen structure to work on
 */
static void
PHYSEC_compute_CS_MAC(const PHYSEC_Key *K, PHYSEC_KeyGen *kg)
{
    AES_CMAC_CTX ctx;
    AES_CMAC_Init(&ctx);
    AES_CMAC_SetKey(&ctx, K->key);
    AES_CMAC_Update(&ctx, (const uint8_t*) kg, sizeof(PHYSEC_KeyGen)-PHYSEC_CS_MAC_SIZE);
    AES_CMAC_Final(kg->MAC, &ctx);
}

/**
 *  @brief Verify the reduduncy check field of a KeyGen reconciliation packet using a specific key. 
 *         Basicaly, just compute the CMAC with the submitted Key and check if it's match with the one
 *         in the packet.
 *  @param  K   The Key retrieved from reconciliation (used for encrypting the CMAC)
 *  @param  kg  The reconciliation packet, with an already computed CMAC, that we want to verify
 *  @return boolean, true if the submitted Key was used to generate the CMAC, otherwise false
 */
static bool
PHYSEC_verify_CS_MAC(const PHYSEC_Key *K, const PHYSEC_KeyGen *kg)
{
    PHYSEC_KeyGen tmp = { 0 };
    memcpy(&tmp, kg, sizeof(PHYSEC_KeyGen)-PHYSEC_CS_MAC_SIZE);
    PHYSEC_compute_CS_MAC(K, &tmp);

    return (memcmp(tmp.MAC, kg->MAC, PHYSEC_CS_MAC_SIZE) == 0);
}

/*************************
 * PHYSEC core functions *
 *************************/

// Reciprocity enhancement

// -- Filtering

static
void PHYSEC_golay_filter(PHYSEC_Measures *rssi_msermts){

    int8_t coef[] = {-3, 12, 17, 12, -3};
    float normalization;

    int rssi_tmp; // it is not normalisze so int8_t is not a valid type.

    int8_t filtred_values[rssi_msermts->nb_val];

    for(int i_rssi = 0; i_rssi < rssi_msermts->nb_val; i_rssi++){

        normalization = 0;
        rssi_tmp = 0;

        for(int i_coef = -2; i_coef < 3; i_coef++){
            if(i_rssi+i_coef >= 0 && i_rssi+i_coef < rssi_msermts->nb_val){
                rssi_tmp += coef[i_coef+2] * rssi_msermts->values[i_rssi+i_coef];
                normalization += coef[i_coef+2];
            }
        }

        filtred_values[i_rssi] = (int8_t) ((float)(rssi_tmp)/normalization);

    }

    memcpy(rssi_msermts->values, filtred_values, rssi_msermts->nb_val * sizeof(int8_t));

}

// --- Interpolation

static
void  PHYSEC_interpolation(PHYSEC_Measures *rssi_msermts){

    int8_t interpolated_values[rssi_msermts->nb_val];

    int8_t delta_rssi;
    float rssi_err;

    if(rssi_msermts->nb_val > 0 && rssi_msermts->delay > 0.0){

        interpolated_values[0] = rssi_msermts->values[0];

        for(int i = 1; i < rssi_msermts->nb_val; i++){
            delta_rssi = rssi_msermts->values[i] - rssi_msermts->values[i-1];
            rssi_err = (rssi_msermts->delay * (float)(delta_rssi));
            interpolated_values[i] = rssi_msermts->values[i] - round(rssi_err);
        }

        rssi_msermts->delay = 0;
        memcpy(rssi_msermts->values, interpolated_values, rssi_msermts->nb_val*sizeof(int8_t));

    }

}

// Key generation

// -- Quantization

#define PHYSEC_QUNTIFICATION_WINDOW_LEN 30

static
void PHYSEC_quantization_sort_rssi_window(int8_t *rssi_window, int8_t rssi_window_size){

    if(rssi_window_size <= 1){
        return;
    }

    int8_t *tab1 = rssi_window;
    int8_t tab1_size = (int8_t) (((float)(rssi_window_size))/2.0);
    int8_t *tab2 = rssi_window+tab1_size;
    int8_t tab2_size = rssi_window_size - tab1_size;

    int8_t tmp_tab[rssi_window_size];

    PHYSEC_quantization_sort_rssi_window(tab1, tab1_size);
    PHYSEC_quantization_sort_rssi_window(tab2, tab2_size);

    int i1=0, i2=0, i=0;

    while(i<rssi_window_size){
        if(i2 >= tab2_size){
            tmp_tab[i++] =  tab1[i1++];
            continue;
        }
        if(i1 >= tab1_size){
            tmp_tab[i++] =  tab2[i2++];
            continue;
        }
        if(tab1[i1]<tab2[i2]){
            tmp_tab[i++] =  tab1[i1++];
        }else{
            tmp_tab[i++] =  tab2[i2++];
        }
    }

    memcpy(rssi_window, tmp_tab, rssi_window_size*sizeof(int8_t));
}

// ---> Density function estimation

static
struct density PHYSEC_quantization_get_density(const int8_t *rssi_window){

    struct density d;

    int8_t last_ele;

    // sorting
    int8_t sorted_rssi_window[PHYSEC_QUNTIFICATION_WINDOW_LEN];
    memcpy(sorted_rssi_window, rssi_window, PHYSEC_QUNTIFICATION_WINDOW_LEN*sizeof(int8_t));
    PHYSEC_quantization_sort_rssi_window(sorted_rssi_window, PHYSEC_QUNTIFICATION_WINDOW_LEN);

    // q_0
    d.q_0 = sorted_rssi_window[0];

    // bins number
    last_ele = sorted_rssi_window[0];
    d.bin_nbr = 0;
    for(int i = 1; i < PHYSEC_QUNTIFICATION_WINDOW_LEN; i++){
        if(sorted_rssi_window[i] != last_ele){
            d.bin_nbr++;
            last_ele = sorted_rssi_window[i];
        }
    }

    // bins & values
    d.bins = malloc(d.bin_nbr * sizeof(int8_t));
    d.values = malloc(d.bin_nbr * sizeof(double));

    last_ele = sorted_rssi_window[0];
    char rep_nbr = 1;
    int j = 0;
    for(int i = 1; i < PHYSEC_QUNTIFICATION_WINDOW_LEN; i++){
        if(sorted_rssi_window[i] != last_ele){

            d.bins[j] = sorted_rssi_window[i] - last_ele;
            d.values[j] = 1.0/((double)(d.bins[j]*PHYSEC_QUNTIFICATION_WINDOW_LEN))*((double)rep_nbr);
            j++;

            last_ele = sorted_rssi_window[i];
            rep_nbr = 1;
        }else{
            rep_nbr++;
        }
    }

    return d;

}

static
void PHYSEC_quantization_free_density(struct density *d){
    free(d->bins);
    free(d->values);
}

/*
    CDF : cumulative distribution function
*/
static
double PHYSEC_quantization_inverse_cdf(double cdf, const struct density *d){

    if(cdf < 0.0){
        cdf = 0.0;
    }

    if(cdf > 1.0){
        cdf = 1.0;
    }

    double q = (double) d->q_0, q_rest;
    double integ = 0, current_integ;

    for(int i = 0; i < d->bin_nbr; i++){
        current_integ = d->bins[i]*d->values[i];
        if(cdf < integ+current_integ){
            q_rest = (cdf - integ)/d->values[i];
            return q + q_rest;
        }else{
            q += d->bins[i];
            integ += current_integ;
        }
    }

    return q;
}

// <--- Density function estimation

static
int8_t PHYSEC_quantization_compute_level_nbr(const struct density *d){

    double negatif_entropy = 0;
    double proba;

    for(int i = 0; i < d->bin_nbr; i++){
        proba = d->values[i] * d->bins[i];
        if(proba>0){
            negatif_entropy +=  proba*log2(proba);
        }
    }

    return (int8_t) pow(2.0, floor(-negatif_entropy));
}

/*
    return value:
        level index strating from 1.
        0 : in case of error
*/
static
uint8_t PHYSEC_quantization_get_level(
    int8_t rssi,
    const double *threshold_starts,
    const double *threshold_ends,
    int8_t qunatification_level_nbr
){
    uint8_t level = 1;
    for(int i = 0; i<qunatification_level_nbr; i++){
        if(rssi<threshold_starts[i])
            return 0;
        if(rssi<=threshold_ends[i])
            return level;
        level++;
    }
    return 0;
}

// Useful bitewise operation

/**
 * Shift a number of bits to the right
 *
 * @param   array       The array to shift
 * @param   len         The length of the array
 * @param   shift       The number of consecutive bits to shift. To the right if shift is positif.
 *
*/
static void shift_bits_right(uint8_t *array, int len, int shift) {

    if(shift < 0)
        fprintf(stderr, "Error\n");

    uint8_t macro_shift = shift / 8;
    shift = shift % 8;

    uint8_t array_out[len];
    memset(array_out, 0, len);

    for(int i = 0; i < len; i++) {
        if(i+macro_shift < len)
            array_out[i+macro_shift] += array[i]>>shift;
        if(i+macro_shift+1 < len)
            array_out[i+macro_shift+1] += array[i]<<(8-shift);
    }

    memcpy(array, array_out, len);
}

static void Measures_copy(PHYSEC_Measures *M, const PHYSEC_Measures *M_org){

    M->delay = M_org->delay;
    M->nb_val = M_org->nb_val;

    M->values = malloc(M->nb_val * sizeof(int8_t));
    memcpy(M->values, M_org->values, M->nb_val * sizeof(int8_t));

}

static void Measures_free(PHYSEC_Measures *M){
    free(M->values);
}

/*
    key1 will conatin the concatenation of both keys.
    return value:
        concatinated key size.
*/
int PHYSEC_key_concatenation(uint8_t *key1, int key1_size, const uint8_t *key2_org, int key2_size){

    uint8_t key2[16];
    memcpy(key2, key2_org, 16);

    int key2_kept_part_size = key2_size;
    int key2_max_size = 128-key1_size;
    if(key2_max_size>0){
        if(key2_kept_part_size > key2_max_size){
            key2_kept_part_size = key2_max_size;
        }

        shift_bits_right(key2, 16, key1_size);

        for(int i = 0; i < 16; i++){
            key1[i]+=key2[i];
        }

        return key1_size+key2_kept_part_size;
    }
    return 128;
}

// For debug
#if PHYSEC_DEBUG >= 4
    char first_pack = 1;
    char first_quantization_window = 1;
    float delay_ratio; // Neded for interpolation plot
    static void vec_key2vec(const PHYSEC_Key *k, int *vec_out);
#endif

/*
    Return value :
        number of generated bit (from left)
        key_output = 128 bits = 16 bytes = a 16-uint8_t-table.
*/
static
int PHYSEC_quantization(
    const PHYSEC_Measures *msermts_org,
    double data_to_band_ration,
    uint8_t *key_output,
    PHYSEC_KeyGenStats *kgs
){

    uint8_t nbr_of_generated_bits = 0;
    uint8_t nbr_of_processed_windows = 0;
    uint16_t rssi_window_align_index = 0;
    int8_t *rssi_window;
    int8_t qunatification_level_nbr;
    struct density density;

    // creating a copy of rssi msr :
    PHYSEC_Measures rssi_msermts;
    Measures_copy(&rssi_msermts, msermts_org);

    // filtering

    uint32_t cur_start = 0;
    if (kgs)
        cur_start = mp_hal_ticks_ms();
    
    PHYSEC_golay_filter(&rssi_msermts);

    #if PHYSEC_DEBUG >= 4

        if(first_pack){

            // first_pack = 0; // Done in the interpolation phase

            // Filtred rssi
            printf("(plot)\n");
            printf("filtred_measurment_pack_0\n"); //id
            printf("Filtred RSSI Measurments : Pack 0\n"); //title
            printf("Time unit\n"); // xlabel
            printf("Filtred RSSI\n"); // ylabel
            for(int i = 0; i < rssi_msermts.nb_val; i++){
                printf("%d, %d\n", i, rssi_msermts.values[i]);
            }
            printf("(plot end)\n");

            // Probe frequency
            printf("(value)\n");
            printf("probe_frequency_ms\n"); //id
            printf("Probe frequency in ms\n"); //Name
            printf("%f probes/s\n", PHYSEC_PROBE_FREQUENCY); //Vlaue

            // Measurement delay
            printf("(value)\n");
            printf("measurement_delay_ms\n"); //id
            printf("1st measurement pack average delay in ms\n"); //Name
            printf("%f ms\n", rssi_msermts.delay * PHYSEC_PROBE_PERIOD); //Vlaue

            // Measurement delay
            printf("(value)\n");
            printf("data_to_band_ratio\n"); //id
            printf("Data to band ratio\n"); //Name
            printf("%f\n", PHYSEC_DATA_TO_BAND_RATIO); //Vlaue

            // Neded for interpolation plot
            delay_ratio = rssi_msermts.delay;
        }

    #endif

    // same time measure estimation
    PHYSEC_interpolation(&rssi_msermts);

    #if PHYSEC_DEBUG >= 4

        if(first_pack){

            first_pack = 0;

            // Filtred rssi
            printf("(plot)\n");
            printf("time_shifted_measurment_pack_0\n"); //id
            printf("Time Shifted RSSI Measurments : Pack 0\n"); //title
            printf("Time unit\n"); // xlabel
            printf("Shifted RSSI\n"); // ylabel
            for(int i = 0; i < rssi_msermts.nb_val; i++){
                printf("%f, %d\n", (float)i - delay_ratio, rssi_msermts.values[i]);
            }
            printf("(plot end)\n");

        }

    #endif
    
    if (kgs)
        kgs->signal_process =  ((float) mp_hal_ticks_ms()-(float)cur_start) / 1000.0;

    if (kgs)
        cur_start = mp_hal_ticks_ms();
    
    // preaparing for key generation
    memset(key_output, 0, 16*sizeof(uint8_t));
    uint8_t level;
    uint8_t gen_bits;

    uint8_t key_tmp[16];
    memset(key_tmp, 0, 16*sizeof(uint8_t));

    while(rssi_msermts.nb_val - rssi_window_align_index  >= PHYSEC_QUNTIFICATION_WINDOW_LEN){

        // rssi window
        rssi_window = rssi_msermts.values+rssi_window_align_index;

        // computing density
        density = PHYSEC_quantization_get_density(rssi_window);

        // computing level number
        qunatification_level_nbr = PHYSEC_quantization_compute_level_nbr(&density);

        // computing thresholds
        double threshold_starts[qunatification_level_nbr];
        double threshold_ends[qunatification_level_nbr];
        double cdf = 0;
        for(int i = 0; i<qunatification_level_nbr; i++){
            threshold_starts[i] = PHYSEC_quantization_inverse_cdf(cdf, &density);
            cdf+=(1-data_to_band_ration)/qunatification_level_nbr;
            threshold_ends[i] = PHYSEC_quantization_inverse_cdf(cdf, &density);
            cdf+=data_to_band_ration/(qunatification_level_nbr-1);
        }

        // quantification
        gen_bits = (uint8_t) log2(qunatification_level_nbr);
        for(int i = 0; i < PHYSEC_QUNTIFICATION_WINDOW_LEN; i++){
            level = PHYSEC_quantization_get_level(
                rssi_window[i],
                threshold_starts,
                threshold_ends,
                qunatification_level_nbr
            );
            if(level > 0){

                level--;
                key_tmp[0] = level << (8 - gen_bits);

                nbr_of_generated_bits = PHYSEC_key_concatenation(key_output, nbr_of_generated_bits, key_tmp, gen_bits);

                if(nbr_of_generated_bits >= 128){
                    Measures_free(&rssi_msermts);
                    return nbr_of_generated_bits;
                }
            }
        }

        // For report
        #if PHYSEC_DEBUG >= 4

            if(first_quantization_window){

                first_quantization_window = 0;

                // Filtred rssi
                printf("(plot)\n");
                printf("quantization_window\n"); //id
                printf("Quantization Window\n"); //title
                printf("Time unit\n"); // xlabel
                printf("RSSI\n"); // ylabel
                for(int i = 0; i < PHYSEC_QUNTIFICATION_WINDOW_LEN; i++){
                    printf("%d, %d\n", i, rssi_window[i]);
                }
                printf("(plot end)\n");

                // Quantization thresholds
                printf("(plot)\n");
                printf("quantization_thresholds\n"); //id
                printf("Quantization Thresholds\n"); //title
                printf("Time unit\n"); // xlabel
                printf("RSSI\n"); // ylabel
                for(int i = 0; i < qunatification_level_nbr; i++){
                    printf("%d, %lf\n", i, threshold_starts[i]);
                }
                for(int i = 0; i < qunatification_level_nbr; i++){
                    printf("%d, %lf\n", i, threshold_ends[i]);
                }
                printf("(plot end)\n");

                // generated key from window
                printf("qunatification_level_nbr = %d\n", qunatification_level_nbr);
                printf("(value)\n");
                
                printf("generated_key_from_quantization_window\n"); //id
                printf("Generated key from quantization window\n"); //Name
                
                // vlaue
                {
                    int key_vec[PHYSEC_KEY_SIZE];
                    vec_key2vec(key_output, key_vec);
                    for(int i = 0; i < nbr_of_generated_bits; i++){
                        printf("%d", key_vec[i]);
                    }
                    printf("\n");
                }

            }

        #endif


        nbr_of_processed_windows++;
        rssi_window_align_index = (uint16_t)(nbr_of_processed_windows*PHYSEC_QUNTIFICATION_WINDOW_LEN);
    }

    // free
    Measures_free(&rssi_msermts);

    if (kgs)
        kgs->quantization = ((float)mp_hal_ticks_ms()-(float)cur_start) / 1000.0;

    return nbr_of_generated_bits;

}

#if PHYSEC_DEBUG >= 3

void PHYSEC_signal_processing_test(){

    int8_t rssi_tmp[] = {81, 66, 50, 40, 84, 92, 79, 95, 102, 86};

    PHYSEC_Measures M;
    M.nb_val = 10;
    M.values = rssi_tmp;
    M.delay = 12;

    printf("rssi original :");
    for(int i = 0; i < M.nb_val; i++){
        printf(" %d", M.values[i]);

    }
    printf("\n");

    PHYSEC_golay_filter(&M);
    printf("rssi filtered :");
    for(int i = 0; i < M.nb_val; i++){
        printf(" %d", M.values[i]);
    }
    printf("\n");

    PHYSEC_interpolation(&M);
    printf("rssi estimated :");
    for(int i = 0; i < M.nb_val; i++){
        printf(" %d", M.values[i]);
    }
    printf("\n");

    printf("density estimation :\n");
    struct density density = PHYSEC_quantization_get_density(M.values);
    printf("\tq_0 : %d\n", density.q_0);
    printf("\tbin_nbr : %d\n", density.bin_nbr);
    printf("\tbins = [");
    for(int i = 0; i < density.bin_nbr; i++){
        printf(" %d", density.bins[i]);
    }
    printf("]\n");
    printf("\tvalues = [");
    for(int i = 0; i < density.bin_nbr; i++){
        printf(" %lf", density.values[i]);
    }
    printf("]\n");

    int8_t qunatification_level_nbr = PHYSEC_quantization_compute_level_nbr(&density);
    printf("Quantification level number : %d\n", qunatification_level_nbr);

    // quantification test
    int8_t rssi_tmp2[] = {81, 66, 50, 40, 84, 92, 79, 95, 102, 86, 96, 47, 58, 74, 87, 92, 66, 84, 53, 61, 72, 83, 81, 64, 55, 47, 85, 95, 77, 98, 102, 85, 97, 45, 57, 78, 85, 93, 47, 58, 74, 87, 92, 66, 84, 53, 64, 85, 52, 64, 76, 88, 81, 66, 50, 40, 84, 92, 79, 95, 102, 86};

    PHYSEC_Measures M2;
    M2.nb_val = 62;
    M2.values = rssi_tmp2;
    M2.delay = 12;

    uint8_t generated_key[16];
    int generated_key_len = PHYSEC_quantization(&M2, 0.1, generated_key, NULL);

    printf("Qunatification :\n");
    printf("\tkey len : %d bits\n", generated_key_len);
    printf("\tkey = [");
    for(int i = 0; i < 16; i++){
        printf("%d ", generated_key[i]);
    }
    printf("]\n");

    PHYSEC_quantization_free_density(&density);

}

#endif

// --- END : Quantization

// END : Key generation

// Reconciliation

#define PHYSEC_RECONCILIATION_EPS 1.0

// --- Elementery matrix operations

typedef struct _matrix {
    uint8_t nrows;
    uint8_t ncols;
    int **content; // A row is a ncols array
} matrix;

// void matrix_extract_row(const matrix *A, int index, int *vec_out){
//     // memcpy(vec_out, A->content[index], A->ncols * sizeof(int));
// }

void matrix_extract_col(const matrix *A, int index, int *vec_out){
    for(int i = 0; i < A->nrows; i++){
        vec_out[i] = A->content[i][index];
    }
}

void vec_sum(const int *vec1, const int *vec2, int *vec_out, unsigned char size){
    for(int i = 0; i < size; i++){
        vec_out[i] = vec1[i] + vec2[i];
    }
}

void vec_diff(const int *vec1, const int *vec2, int *vec_out, unsigned char size){
    for(int i = 0; i < size; i++){
        vec_out[i] = vec1[i] - vec2[i];
    }
}

double vec_norm2(const int *vec, unsigned char size){

    double norm = 0;
    for(int i = 0; i < size; i++){
        norm += (double) (vec[i] * vec[i]);
    }
    return sqrt(norm);
}

int vec_dot_product(const int *vec1, const int *vec2, unsigned char size){
    
    int res = 0;
    
    for(int i = 0; i < size; i++){
        res += (vec1[i]*vec2[i]);
    }

    return res;
}

static void vec_key2vec(const PHYSEC_Key *k, int *vec_out){
    
    int byte_index = -1;
    int in_byte_index = 0;
    
    for (int i=0; i<PHYSEC_KEY_SIZE; i++){
        
        // vec_out[i] = (k->key[i/8] >> (7-(i%8))) & 0x1;

        if(i%8 == 0){
            byte_index++;
            in_byte_index = 0;
        }
        
        vec_out[i] = (k->key[byte_index] >> (7-in_byte_index)) & 0x1;
        in_byte_index++;
    }
}

static void vec_vec2key(int *vec, PHYSEC_Key *k_out){
    
    int byte_index = -1;
    int in_byte_index;

    memset(k_out->key, 0, PHYSEC_KEY_SIZE_BYTES*sizeof(uint8_t));

    for(int i = 0; i < PHYSEC_KEY_SIZE; i++){
        if((i%8) == 0){
            byte_index++;
            in_byte_index = 0;
        }

        k_out->key[byte_index] +=  ((vec[i] << (7-in_byte_index)) & (0x1) << (7-in_byte_index));
        in_byte_index++;
    }

}

// defining compression matrix
matrix compression_matrix_def(){

    int A_1D_content[] = A_1D_CONTENT;

    matrix A;
    A.nrows = PHYSEC_CS_COMPRESSED_SIZE;
    A.ncols = PHYSEC_KEY_SIZE;
    A.content = malloc(PHYSEC_CS_COMPRESSED_SIZE * sizeof(int*));
    
    for (int i=0; i<PHYSEC_CS_COMPRESSED_SIZE; i++){

        A.content[i] = malloc(sizeof(int) * A.ncols);

        for(int j = 0; j < PHYSEC_KEY_SIZE; j++){
            A.content[i][j] = A_1D_content[i*PHYSEC_KEY_SIZE+j];
        }
    }

    return A;
};

void compression_matrix_free(matrix *A){
    
    for (int i=0; i<PHYSEC_CS_COMPRESSED_SIZE; i++){
        free(A->content[i]);
    }

    free(A->content);
};

static void
mul_matrix_H(const matrix *A, const int *vec, size_t size, int *ret)
{
    memset(ret, 0, sizeof(int) * A->nrows);
    for (int i=0; i < A->nrows; i++)
    {
        for (int j=0; j < A->ncols; j++)
        {
            ret[i] += A->content[i][j] * vec[j];
        }
    }
}

// --- atom array

struct atom{
    int index;
    int atom[PHYSEC_CS_COMPRESSED_SIZE];
};

struct atom_array{
    int not_used_atoms_nbr;
    struct atom atoms[PHYSEC_KEY_SIZE];
};

static struct atom_array atom_array_init(const matrix *A){
    
    struct atom_array atoms;

    atoms.not_used_atoms_nbr = PHYSEC_KEY_SIZE;

    for(int i = 0; i<PHYSEC_KEY_SIZE; i++){
        atoms.atoms[i].index = i;
        matrix_extract_col(A, i, atoms.atoms[i].atom);
    }

    return atoms;
}

static int atom_get_in_tab_index_by_atom_index(struct atom_array *atoms, int index){
    for(int i = 0; i < atoms->not_used_atoms_nbr; i++){
        if(atoms->atoms[i].index == index){
            return i;
        }
    }
    fprintf(stderr, "Atom index not found.\n");
    return -1;
}

static void atom_array_mark_as_used(struct atom_array *atoms, int index){

    int in_tab_index = atom_get_in_tab_index_by_atom_index(atoms, index);

    memcpy(
        atoms->atoms+in_tab_index,
        atoms->atoms+(atoms->not_used_atoms_nbr -1),
        sizeof(struct atom)
    );

    atoms->not_used_atoms_nbr--;
}

static int atom_array_get_best_matching_atom_index(struct atom_array *atoms, int *residual, int *dot_product_sign){

    char max_not_define = 1;
    int max_index = -1;
    double max = -1.0;
    *dot_product_sign = 1;

    double metric, metric_tmp;

    int dot_product_sign_tmp;
    int tmp_vec[PHYSEC_CS_COMPRESSED_SIZE];

    for(int i = 0; i < atoms->not_used_atoms_nbr; i++){
        
        // metric : dot product
        // metric = vec_dot_product(atoms->atoms[i].atom, residual, PHYSEC_CS_COMPRESSED_SIZE);

        // metric : norm2
        vec_diff(residual, atoms->atoms[i].atom, tmp_vec, PHYSEC_CS_COMPRESSED_SIZE);
        metric = vec_norm2(tmp_vec, PHYSEC_CS_COMPRESSED_SIZE);
        dot_product_sign_tmp = 1;

        vec_sum(residual, atoms->atoms[i].atom, tmp_vec, PHYSEC_CS_COMPRESSED_SIZE);
        metric_tmp = vec_norm2(tmp_vec, PHYSEC_CS_COMPRESSED_SIZE);
        
        if(metric_tmp > metric){
            metric = metric_tmp;
            dot_product_sign_tmp = -1;
        }
        
        if(max_not_define || metric > max){
            max_not_define = 0;
            max = metric;
            max_index = atoms->atoms[i].index;
            *dot_product_sign = dot_product_sign_tmp;
        }
    
    }

    return max_index;
}

static void
PHYSEC_craft_reconciliate_vector(int *cs_vec, const PHYSEC_Key *k, matrix *A)
{
    int k_vec[PHYSEC_KEY_SIZE];
    vec_key2vec(k, k_vec);

    mul_matrix_H(A, k_vec, PHYSEC_KEY_SIZE, cs_vec);
}

static void PHYSEC_compute_key_error(const matrix *A, const int *y, int *x_out){

    // Init key error
    int x_tmp[PHYSEC_KEY_SIZE];
    memset(x_tmp, 0, PHYSEC_KEY_SIZE*sizeof(int));

    // Init atom array
    struct atom_array atoms = atom_array_init(A);
    int best_matching_atom_index, dot_product_sig;

    // Init residual
    int residual[PHYSEC_CS_COMPRESSED_SIZE];
    memcpy(residual, y, PHYSEC_CS_COMPRESSED_SIZE * sizeof(int));

    double residual_norm2;
    double min_residual_norm2 = vec_norm2(residual, PHYSEC_CS_COMPRESSED_SIZE);
    memcpy(x_out, x_tmp, sizeof(int)*PHYSEC_KEY_SIZE);

    int nbr_of_ignored_max = 0;

    
    while(atoms.not_used_atoms_nbr > 0){

        best_matching_atom_index = atom_array_get_best_matching_atom_index(&atoms, residual, &dot_product_sig);

        x_tmp[best_matching_atom_index] = dot_product_sig;
        atom_array_mark_as_used(&atoms, best_matching_atom_index);

        if(dot_product_sig == -1){
            vec_diff(
                residual,
                atoms.atoms[best_matching_atom_index].atom,
                residual,
                PHYSEC_CS_COMPRESSED_SIZE
            );
        }else{
            vec_sum(
                residual,
                atoms.atoms[best_matching_atom_index].atom,
                residual,
                PHYSEC_CS_COMPRESSED_SIZE
            );
        }

        residual_norm2 = vec_norm2(residual, PHYSEC_CS_COMPRESSED_SIZE);

        if(residual_norm2 < min_residual_norm2){

            min_residual_norm2 = residual_norm2;
            memcpy(x_out, x_tmp, sizeof(int)*PHYSEC_KEY_SIZE);

            if(residual_norm2 == 0){
                break;
            }
        
        }

        if(residual_norm2 >= min_residual_norm2){
            nbr_of_ignored_max++;
            if(nbr_of_ignored_max == 1){
                break;
            }
        }

    }

}

// --- The hight level function

// --- --- For Bob
static void PHYSEC_reconciliation_compute_compressed_key(const PHYSEC_Key *k, int *y_out){
    
    matrix A = compression_matrix_def();

    PHYSEC_craft_reconciliate_vector(y_out, k, &A);

    compression_matrix_free(&A);
}

// --- --- For Alice
static PHYSEC_Key PHYSEC_reconciliate_key(
    const PHYSEC_Key *Alice_key,
    const int *Bob_compressed_key
){
    matrix A = compression_matrix_def();

    int Alice_key_vec[PHYSEC_KEY_SIZE];
    int Alice_compressed_key[PHYSEC_CS_COMPRESSED_SIZE];
    int compressed_key_diff[PHYSEC_CS_COMPRESSED_SIZE];
    int key_error[PHYSEC_KEY_SIZE];
    PHYSEC_Key reconciliated_key;
    int reconciliated_key_vec[PHYSEC_KEY_SIZE];

    PHYSEC_craft_reconciliate_vector(Alice_compressed_key, Alice_key, &A);
    vec_diff(Alice_compressed_key, Bob_compressed_key, compressed_key_diff, PHYSEC_CS_COMPRESSED_SIZE);
    PHYSEC_compute_key_error(&A, compressed_key_diff, key_error);

    // reconciliation
    vec_key2vec(Alice_key, Alice_key_vec);
    for(int i = 0; i<PHYSEC_KEY_SIZE; i++){
        reconciliated_key_vec[i] = (Alice_key_vec[i] + key_error[i]) % 2;
    }
    vec_vec2key(reconciliated_key_vec, &reconciliated_key);

    compression_matrix_free(&A);

    return reconciliated_key;
}

// END : Reconciliation

static void
PHYSEC_privacy_amplification(PHYSEC_Key *K)
{
    CRYAL_SHA256_CTX ctx =  { 0 };
    sha256_init(&ctx);
    sha256_update(&ctx, (BYTE*) K->key, PHYSEC_KEY_SIZE_BYTES);
    uint8_t hash[32] = { 0 };
    sha256_final(&ctx, hash);
    memcpy(K->key, hash, PHYSEC_KEY_SIZE_BYTES); // truncate hash
}

static float
entropy(uint8_t *bits, uint32_t nbits)
{
    uint32_t c1 = 0;

    for (uint32_t i=0; i < nbits; i++)
    {
        if ( (bits[i/8] >> (7-(i%8))) & 0x1 )
            c1 ++;
    }

    float p1 = (float)c1 / (float)nbits;
    float p0 = 1.0 - p1;

    return p0 * log2( 1 / p0 ) + p1 * log2(1 / p1);
}

// -- Key management
void peer_key_list_init(peer_key_list_t pkl){
    *pkl = NULL;
}

void peer_key_free(struct peer_key *pk){
    if(pk != NULL){
        free(pk);
    }
}

void peer_key_recursive_free(struct peer_key *pk){
    if(pk != NULL){
        peer_key_recursive_free(pk->next);
        peer_key_free(pk);
    }
}

void peer_key_list_free(peer_key_list_t pkl){
    peer_key_recursive_free(*pkl);
    *pkl = NULL;
}

void peer_key_push(peer_key_list_t pkl, uint32_t peer_id, const uint8_t *key){

    struct peer_key *pk = malloc(sizeof(struct peer_key));
    pk->peer_id = peer_id;
    memcpy(pk->key, key, 16*sizeof(uint8_t));
    pk->next = *pkl;

    *pkl = pk;

}

/*
return value:
    0  : peer_id not found, key_out = NULL.
    1   : peer_id founded and the key is copied in the key_out.
*/
char peer_key_list_get_key_by_peer_id(peer_key_list_t pkl, uint32_t peer_id, uint8_t *key_out){

    struct peer_key *pk_curr = *pkl;

    while(pk_curr != NULL){
        if(pk_curr->peer_id == peer_id){
            memcpy(key_out, pk_curr->key, 16*sizeof(uint8_t));
            return 1;
        }
    }

    return 0;

}

void peer_key_delete_by_peer_id(peer_key_list_t pkl, uint32_t peer_id){

    struct peer_key *pk_prev = NULL;
    struct peer_key *pk_curr = *pkl;

    while(pk_curr != NULL){
        if(pk_curr->peer_id == peer_id){
            if(pk_prev == NULL){
                (*pkl)->next = pk_curr->next;
                peer_key_free(pk_curr);
                pk_curr = (*pkl)->next;
            }else{
                pk_prev->next = pk_curr->next;
                peer_key_free(pk_curr);
                pk_curr = pk_prev->next;
            }
        }else{
            pk_prev = pk_curr;
            pk_curr = pk_curr->next;
        }
    }

}

// -- Key Generation main functions 

static void PHYSEC_sampling(const uint16_t n_required,
                            const uint16_t cnt,
                            const PHYSEC_Sync *sync, 
                            const PHYSEC_MeasureType mt, 
                            PHYSEC_Measures *m)
{
    uint16_t cur_cnt = 0;
    uint32_t probe_start = 0;
    uint32_t last_delay = 0;
    uint32_t delay_sum = 0;

    uint32_t tick;


    for (int i=0; i<n_required; )
    {

        tick = mp_hal_ticks_ms();
        if( i>0 && ((tick - probe_start) < PHYSEC_PROBE_PERIOD)){
            usleep((tick - probe_start)*1000);
        }

        cur_cnt = cnt + i;

        // send probe
        PHYSEC_Packet pkt = {
            .identifier = PHYSEC_KG_PKT_IDENTIFIER,
            .type = PHYSEC_PT_PROBE,
            .dev_id = { 0 },
            .payload = { 0 }
        };
        memcpy(pkt.dev_id, sync->rmt_dev_id, PHYSEC_DEV_ID_LEN);

        PHYSEC_Probe *probe = (PHYSEC_Probe*) &(pkt.payload);
        probe->cnt = cur_cnt;

        lora_send((uint8_t*) &pkt, (sizeof(PHYSEC_Probe) + PHYSEC_KG_PKT_HEADER_SIZE), -1);

        probe_start = mp_hal_ticks_ms();

        physec_log(3, ">>> PROBE SENT\n");
        physec_hexdump(3, (uint8_t*)probe, sizeof(PHYSEC_Probe));
        physec_log2(3, "probe->cnt=%d, cnt=%d\n", probe->cnt, cur_cnt);
        physec_log(3, "\n");

        // wait probe response, verify and increment
        int8_t m_val = 0;
        uint32_t wtime = 0;
        if ( wait_probe(sync, PHYSEC_MT_SNR, &m_val, &wtime, lora_obj.physec_timeout) == cur_cnt && wtime < lora_obj.physec_timeout)
        {
            // last_delay = (mp_hal_ticks_ms()-probe_start) - lora_obj.tx_time_on_air; // minus the toa of the Alice probe
            last_delay = (mp_hal_ticks_ms()-probe_start) / 2 ;
            
            delay_sum += last_delay;

            physec_log(3, "<<< PROBE ANSWER RECEIVED\n");
            physec_log(3, "\n");

            // store characteristic of last probe
            m->values[i] = m_val;
            i++;

            //mp_hal_delay_ms(PHYSEC_COHERENCE_TIME);
        }
        //else if (cur_cnt > 0)
        // else
        //     delay_sum -= last_delay;

    }
    m->nb_val = n_required;
    m->delay = (float) delay_sum / ((float) (n_required) * PHYSEC_PROBE_PERIOD);

}

/**
 * @brief Initiate the key generation with the waiter (Bob), Takes a pair of identifiers to detect 
 *        the packets transmitted between the two involved devices
 *        -- Alice generation function
 * 
 * @param k     the data structure to hold the generated key
 * @param sync  the synchronization structure
 * @param kgs   the structure for holding various measurements of performance 
 */
static void
PHYSEC_initiate_key_agg(PHYSEC_Key *k, const PHYSEC_Sync *sync, PHYSEC_KeyGenStats *kgs)
{
    bool generated = false;
    PHYSEC_Key key = { 0 };
    int key_len = 0;
    int n_mismatch = 0;

    uint32_t cur_stage_start = 0;
    if (kgs)
        memset(kgs, 0, sizeof(PHYSEC_KeyGenStats));

    PHYSEC_Key P;
    int32_t nbits;

#if PHYSEC_DEBUG == 4
    int8_t *rssis = calloc(PHYSEC_N_MAX_MEASURE, sizeof(int8_t));
    int8_t *raw_rssis = calloc(PHYSEC_N_MAX_MEASURE, sizeof(int8_t));
#endif

    uint16_t n_required = PHYSEC_N_REQUIRED_MEASURE;

    // Generation loop
    uint16_t cnt = 0;
    uint16_t prev_cnt = 0;
    int8_t measure_frame[PHYSEC_N_REQUIRED_MEASURE] = { 0 };
    PHYSEC_Measures mtmp = { 0 };
    mtmp.values = measure_frame;

    #if PHYSEC_DEBUG >= 4
        int measurment_pack_index = 0;
    #endif

    while ( !generated )
    {
        if (kgs)
            cur_stage_start = mp_hal_ticks_ms();
        
        // Phase 0 : Probing and measurements
        PHYSEC_sampling(n_required, cnt, sync, PHYSEC_MT_SNR, &mtmp);
        prev_cnt = cnt;
        cnt += n_required;
        if (kgs)
        {
            kgs->measure += (float)(mp_hal_ticks_ms() - cur_stage_start) / 1000.0;
            // for now erase last delay (must be better to calculate the delay over all measures)
            kgs->avg_delay = mtmp.delay;
        }

        #if PHYSEC_DEBUG == 4
            memcpy(raw_rssis+prev_cnt, mtmp.values, n_required * sizeof(int8_t));
            printf("### RSSI BEFORE QUANTIFICATION\n");
            display_m_vals(raw_rssis, cnt);
        #endif

        // data report
        #if PHYSEC_DEBUG >= 4
            printf("(plot)\n");
            printf("measurment_pack_%d\n", measurment_pack_index); //id
            printf("RSSI Measurments : Pack %d\n", measurment_pack_index); //title
            printf("Time unit\n"); // xlabel
            printf("RSSI\n"); // ylabel
            for(int i = 0; i < mtmp.nb_val; i++){
                printf("%d, %d\n", i, mtmp.values[i]);
            }
            printf("(plot end)\n");
            measurment_pack_index++;
        #endif


        if (key_len <= PHYSEC_KEY_SIZE)
        {
            physec_log(1, "### QUANTIFICATION\n\n");

            nbits = PHYSEC_quantization(&mtmp, PHYSEC_DATA_TO_BAND_RATIO, P.key, kgs);
            key_len = PHYSEC_key_concatenation(key.key, key_len, P.key, nbits);

            #if PHYSEC_DEBUG == 4
                memcpy(rssis+prev_cnt, mtmp.values, n_required * sizeof(int8_t));
            #endif

            physec_log2(2, "generated bits %d/128 with %d measures\n", key_len, cnt);
            //physec_hexdump(2,(uint8_t*) key.key, PHYSEC_KEY_SIZE_BYTES);
            physec_log(2, "P = ");
            physec_verb_statement(2, display_key_bits(&key));
            physec_log(1, "\n");

            // check if bit key len >= PHYSEC_KEY_SIZE, else increase n_required
            if (key_len < PHYSEC_KEY_SIZE)
            {

                n_required = PHYSEC_N_NEW_REQUIRED_MEASURE;
                continue;
            }
            
        }

        // data report
            #if PHYSEC_DEBUG >= 4
            
                printf("(value)\n");
                
                printf("generated_key_at_quantization\n"); //id
                printf("Generated key at quantization\n"); //Name
                
                // vlaue
                {
                    int key_vec[PHYSEC_KEY_SIZE];
                    vec_key2vec(&key, key_vec);
                    for(int i = 0; i < PHYSEC_KEY_SIZE; i++){
                        printf("%d", key_vec[i]);
                    }
                    printf("\n");
                }

            #endif


        #if PHYSEC_DEBUG == 4
            printf("\n### RSSI AFTER QUANTIFICATION\n");
            display_m_vals(raw_rssis, cnt);
        #endif

        if (kgs)
            cur_stage_start = mp_hal_ticks_ms();
        // send KeyGen begin packet
        PHYSEC_Packet pkt = {
            .identifier = PHYSEC_KG_PKT_IDENTIFIER,
            .type = PHYSEC_PT_KEYGEN,
            .dev_id = { 0 },
            .payload = { 0 }
        };

        memcpy(pkt.dev_id, sync->rmt_dev_id, PHYSEC_DEV_ID_LEN);
        lora_send((uint8_t*) &pkt, (sizeof(PHYSEC_KeyGen) + PHYSEC_KG_PKT_HEADER_SIZE), -1);

        physec_log(3, ">>> KGS START PKT SENT\n");
        physec_hexdump(3, (uint8_t*) pkt.payload, sizeof(PHYSEC_KeyGen));
        physec_log(3, "\n");

        // wait for Bob KeyGen response (Reconciliation or Reset packet)
        uint8_t buf[PHYSEC_MAX_PAYLOAD_SIZE] = { 0 };
        PHYSEC_PacketType pkt_type = PHYSEC_PT_NONE;
        uint32_t kgstart = mp_hal_ticks_ms();
        uint32_t kgtime = 5000;
        while ( (kgtime > (mp_hal_ticks_ms()-kgstart)) )
        {
            pkt_type = PHYSEC_wait_keygen_packet(sync, buf, sizeof(buf), kgtime, NULL, PHYSEC_MT_NONE, NULL);
            if (pkt_type & (PHYSEC_PT_KEYGEN|PHYSEC_PT_RESET))
                break;
        }
        // if timeout happened, jump to KeyGen begin packet sending

        if (pkt_type == PHYSEC_PT_KEYGEN)
        {
            // Phase 3 : Reconciliation
            PHYSEC_Key Kb = { 0 };
            PHYSEC_KeyGen *kg_r = (PHYSEC_KeyGen*) buf;

            physec_log(3, "<<< KEYGEN PKT RECEIVED\n");
            physec_hexdump(3, (uint8_t*) kg_r, sizeof(PHYSEC_KeyGen));
            physec_log(3, "\n");

            physec_log(1, "### RECONCILIATION\n\n");

            if (kgs)
                cur_stage_start = mp_hal_ticks_ms();

            //PHYSEC_reconciliate_key(&key, kg_r->cs_vec, &Kb); // uncomment and comment line below when reconciliation is fixed
            memcpy(&Kb, &(kg_r->K), sizeof(PHYSEC_Key));

            // if reconciliated key doesn't allow to verify packet MAC, 
            // discard reconcil packet and send back KeyGen begin packet 
            if ( !PHYSEC_verify_CS_MAC(&Kb, kg_r) )
            {
                physec_log(2, "Unvalid reconciliated key\n");
                continue;
            }

            if (kgs)
                kgs->reconciliation = (float) (mp_hal_ticks_ms()-cur_stage_start) / 1000.0;

            physec_log(1, "### KEYS RECONCILIATED\n");
            PHYSEC_Key KM = { 0 };
            n_mismatch = calculate_mismatch(&Kb, &key, &KM);
            physec_log2(2, "Nb mismatch : %d/%d - %f %%\n", n_mismatch, PHYSEC_KEY_SIZE, (float) n_mismatch / (float) PHYSEC_KEY_SIZE);
            physec_log(2, "Mismatches : ");
            physec_verb_statement(2, display_key_bits(&KM));
            physec_log(2, "Alice Key  : ");
            physec_verb_statement(2, display_key_bits(&key));
            physec_log(2, "Bob Key    : ");
            physec_verb_statement(2, display_key_bits(&Kb));
            physec_log(1, "\n");

            physec_log2(2, "Entropy before PA: %f\n", entropy(Kb.key, key_len));

            if (kgs)
                kgs->entropy_pk = entropy(Kb.key, key_len);


            // Phase 4 : Privacy Amplification
            physec_log(1, "### PRIVACY AMPLIFICATION\n\n");
            cur_stage_start = mp_hal_ticks_ms();

            PHYSEC_privacy_amplification(&Kb);

            if (kgs)
            {
                kgs->mismatch = (float) n_mismatch / (float) PHYSEC_KEY_SIZE;
                kgs->privacy_amp = (float) (mp_hal_ticks_ms()-cur_stage_start) / 1000.0;
                kgs->entropy_k = entropy(Kb.key, key_len);
            }

            memcpy(k, &Kb, sizeof(PHYSEC_Key));

            physec_log(1, "### KEY GENERATED\n");
            physec_log(1, "K = ");
            physec_verb_statement(1, display_key_bits(k));
            physec_hexdump(3, (uint8_t*) k, PHYSEC_KEY_SIZE_BYTES);
            physec_log2(2, "Entropy after PA: %f\n", entropy(key.key, key_len));
            physec_log(1, "\n");

            // data report
            #if PHYSEC_DEBUG >= 4
            
                printf("(value)\n");
                
                printf("generated_key\n"); //id
                printf("Generated key\n"); //Name
                
                // vlaue
                {
                    int key_vec[PHYSEC_KEY_SIZE];
                    vec_key2vec(k, key_vec);
                    for(int i = 0; i < PHYSEC_KEY_SIZE; i++){
                        printf("%d", key_vec[i]);
                    }
                    printf("\n");
                }

            #endif

#if PHYSEC_DEBUG == 4
            free(raw_rssis);
            free(rssis);
#endif
            if (kgs)
            {
                kgs->n_measures = cnt;
                kgs->entropy_k = entropy(k->key, PHYSEC_KEY_SIZE_BYTES);
                kgs->total = kgs->measure + kgs->signal_process + kgs->quantization + kgs->reconciliation + kgs->privacy_amp;
            }

            generated = true;
        }
        // quantification failed to gen 128 bits on the other side, thus reset
        else if (pkt_type == PHYSEC_PT_RESET)
        {
            physec_log(3, "<<< KEYGEN PKT RECEIVED - MEASURE RESET\n");
            physec_log(3, "\n");
            if (kgs)
                kgs->n_retries++;
            cnt = 0;
            memset(&key, 0, sizeof(PHYSEC_Key));
            key_len = 0;
            n_required = PHYSEC_N_REQUIRED_MEASURE;
        }
    }
}

#define TEST_WDT_TIMER_MS              10*(60*1000)    // 10 minutes
/**
 * @brief Initiate the key generation with the waiter (Bob), Takes a pair of identifiers to detect 
 *        the packets transmitted between the two involved devices
 *        -- Alice generation function
 * 
 * @param k     the data structure to hold the generated key
 * @param sync  the synchronization structure
 * @param kgs   the structure for holding various measurements of performance 
 */
static void
PHYSEC_wait_key_agg(PHYSEC_Key *k, const PHYSEC_Sync *sync, PHYSEC_KeyGenStats *kgs)
{
    bool generated = false;

    PHYSEC_Measures m = {
        .nb_val = 0,
        .values = malloc(sizeof(int8_t) * PHYSEC_N_MAX_MEASURE), // alloc failure is already caught by micro python
        .delay = 0
    };

    uint16_t last_cnt = 255;
    uint16_t cnt = 0;
    uint32_t cur_stage_start;
    if (kgs)
        memset(kgs, 0, sizeof(PHYSEC_KeyGenStats));

    // Generation loop
    uint32_t wdt_start = mp_hal_ticks_ms();
    while ( !generated )
    {

        if (mp_hal_ticks_ms()-wdt_start > TEST_WDT_TIMER_MS)
        {
            printf("Too much time for PHYSEC KEYGEN ! ABORT...\n\n\n");
            free(m.values);
            return;
            //machine_wdt_start(1);
            //while( 1 );     // reset waiting loop
        }

        uint8_t buf[PHYSEC_MAX_PAYLOAD_SIZE] = { 0 };
        uint32_t start = 0;
        int8_t rssi = 0;
        // Catch PHYSEC packets and react to them accordingly to lora-key
        switch ( PHYSEC_wait_keygen_packet(sync, buf, sizeof(buf), lora_obj.physec_timeout, &start, PHYSEC_MT_SNR, &rssi) )
        {
            case PHYSEC_PT_PROBE:
            {
                PHYSEC_Probe *probe = (PHYSEC_Probe*) buf;
                physec_log(3, "<<< PROBE REQUEST RECEIVED\n");
                physec_hexdump(3, (uint8_t*) probe, sizeof(PHYSEC_Probe));
                physec_log2(3, "probe->cnt=%d, cnt=%d\n", probe->cnt, cnt);
                physec_log(3, "\n");

                // verify probe
                if (kgs && cnt == 0)
                    cur_stage_start = mp_hal_ticks_ms();

                // alice didn't get our previous response
                if (probe->cnt == last_cnt)
                    cnt--;                  // discard last measure

                if (probe->cnt == cnt)
                {
                    // probe accepted, store measure and respond to probe
                    m.values[cnt] = rssi;

                    PHYSEC_Packet pkt = {
                        .identifier = PHYSEC_KG_PKT_IDENTIFIER,
                        .type = PHYSEC_PT_PROBE,
                        .dev_id = { 0 },
                        .payload = { 0 }
                    };
                    memcpy(pkt.dev_id, sync->rmt_dev_id, PHYSEC_DEV_ID_LEN);
                    PHYSEC_Probe *response = (PHYSEC_Probe*) pkt.payload;
                    response->cnt = cnt;
                    lora_send((uint8_t*) &pkt, sizeof(PHYSEC_Probe) + PHYSEC_KG_PKT_HEADER_SIZE, -1);

                    physec_log(3, ">>> PROBE ANS SENT\n");
                    physec_hexdump(3, (uint8_t*)pkt.payload, sizeof(PHYSEC_Probe));
                    physec_log(3, "\n");

                    last_cnt = cnt;
                    cnt ++;
                }
                break;
            }
            case PHYSEC_PT_KEYGEN:
            {
             
                physec_log(3, "<<< KEYGEN PKT RECEIVED\n");
                physec_hexdump(3, (uint8_t*)buf, sizeof(PHYSEC_KeyGen));
                physec_log(3, "\n");

                // didn't finish the first measure phase, cannot be a Alice packet
                if (cnt < PHYSEC_N_REQUIRED_MEASURE)
                    continue;

                m.delay = 0;
                m.nb_val = cnt;

                if (kgs)
                    kgs->measure = (float) (mp_hal_ticks_ms()-cur_stage_start) / 1000.0;

                physec_log(4, "### RSSI BEFORE QUANTIFICATION\n");
                physec_verb_statement(4, display_m_vals(m.values, cnt));
                physec_log(4, "\n");

                // PHYSEC_N_REQUIRED_MEASURE
                // PHYSEC_N_NEW_REQUIRED_MEASURE

                // generate key
                
                physec_log(1, "### QUANTIFICATION\n\n");

                if((m.nb_val - PHYSEC_N_REQUIRED_MEASURE) < 0 || (m.nb_val - PHYSEC_N_REQUIRED_MEASURE) % PHYSEC_N_NEW_REQUIRED_MEASURE != 0){
                    fprintf(stderr, "m.nb_val does not recpect this condition : m.nb_val = PHYSEC_N_REQUIRED_MEASURE + k * PHYSEC_N_NEW_REQUIRED_MEASURE\n");
                }

                PHYSEC_Key P;
                int32_t P_len = 0;
                memset(&P, 0, sizeof(PHYSEC_Key));

                PHYSEC_Key P_tmp;
                int32_t P_tmp_len = 0;

                PHYSEC_Measures m_tmp;
                m_tmp.delay = 0;
                m_tmp.values = malloc(sizeof(uint8_t) * PHYSEC_N_REQUIRED_MEASURE);

                #if PHYSEC_DEBUG >= 4
                    int measurment_pack_index = 0;
                #endif

                int32_t measures_index = 0;
                while(measures_index < m.nb_val && P_len < PHYSEC_KEY_SIZE){

                    // construct measurement part

                    if(measures_index == 0){
                        memcpy(m_tmp.values, m.values, PHYSEC_N_REQUIRED_MEASURE * sizeof(uint8_t));
                        m_tmp.nb_val = PHYSEC_N_REQUIRED_MEASURE;
                        measures_index += PHYSEC_N_REQUIRED_MEASURE;
                    }else{
                        memcpy(m_tmp.values, m.values + measures_index, PHYSEC_N_NEW_REQUIRED_MEASURE * sizeof(uint8_t));
                        m_tmp.nb_val = PHYSEC_N_NEW_REQUIRED_MEASURE;
                        measures_index += PHYSEC_N_NEW_REQUIRED_MEASURE;
                    }

                    // data report
                    #if PHYSEC_DEBUG >= 4
                        printf("(plot)\n");
                        printf("measurment_pack_%d\n", measurment_pack_index); //id
                        for(int i = 0; i < m_tmp.nb_val; i++){
                            printf("%d, %d\n", i, m_tmp.values[i]);
                        }
                        printf("(plot end)\n");
                        measurment_pack_index++;
                    #endif

                    // measurement part quantization
                    P_tmp_len = PHYSEC_quantization(&m_tmp, PHYSEC_DATA_TO_BAND_RATIO, P_tmp.key, kgs);

                    // Key concatenation
                    P_len = PHYSEC_key_concatenation(P.key, P_len, P_tmp.key, P_tmp_len);

                }
                free(m_tmp.values);

                physec_log(2, "P = ");
                physec_verb_statement(2, display_key_bits(&P));

                // send reconciliation response and finish keygen, or send reset measure
                PHYSEC_Packet pkt = {
                    .identifier = PHYSEC_KG_PKT_IDENTIFIER,
                    .type = PHYSEC_PT_KEYGEN,
                    .dev_id = { 0 },
                    .payload = { 0 }
                };
                memcpy(pkt.dev_id, sync->rmt_dev_id, PHYSEC_DEV_ID_LEN);
                PHYSEC_KeyGen *kg_s = (PHYSEC_KeyGen*) &(pkt.payload);
                if ( 1 || P_len >= PHYSEC_KEY_SIZE)
                {
                    

                    physec_log(4, "### RSSI AFTER QUANTIFICATION\n");
                    physec_verb_statement(4, display_m_vals(m.values, cnt));


                    physec_log(1, "### RECONCILIATION\n\n");

                    //PHYSEC_reconciliation_compute_compressed_key((const PHYSEC_Key*) &P, kg_s->cs_vec);
                    memcpy(&(kg_s->K), &P, sizeof(PHYSEC_Key));

                    PHYSEC_compute_CS_MAC(&P, kg_s);

                    lora_send((uint8_t*)&pkt, PHYSEC_KG_PKT_HEADER_SIZE + sizeof(PHYSEC_KeyGen), -1);

                    physec_log2(2, "Key entropy before PA: %f\n\n", entropy(P.key, PHYSEC_KEY_SIZE));
                    physec_log(1, "### PRIVACY AMPLIFICATION\n\n");
                    if (kgs)
                    {
                        cur_stage_start = mp_hal_ticks_ms();
                        kgs->entropy_pk = entropy(P.key, PHYSEC_KEY_SIZE_BYTES);
                    }

                    // data report
                    #if PHYSEC_DEBUG >= 4
                    
                        printf("(value)\n");
                        
                        printf("generated_key_at_quantization\n"); //id
                        printf("Generated key at quantization\n"); //Name
                        
                        // vlaue
                        {
                            int key_vec[PHYSEC_KEY_SIZE];
                            vec_key2vec(&P, key_vec);
                            for(int i = 0; i < PHYSEC_KEY_SIZE; i++){
                                printf("%d", key_vec[i]);
                            }
                            printf("\n");
                        }

                    #endif

                    PHYSEC_privacy_amplification(&P);
                    if (kgs)
                    {
                        kgs->privacy_amp = (float) (mp_hal_ticks_ms()-cur_stage_start) / 1000.0;
                        kgs->entropy_k = entropy(P.key, PHYSEC_KEY_SIZE_BYTES);
                    }

                    memcpy(k, &P, sizeof(PHYSEC_Key));
                    physec_log(1, "### KEY GENERATED\n");
                    physec_log(1, "K = ");
                    physec_verb_statement(1, display_key_bits(k));
                    physec_hexdump(2, (uint8_t*) k, PHYSEC_KEY_SIZE_BYTES);
                    physec_log2(2, "Key entropy after PA: %f\n", entropy(k->key, PHYSEC_KEY_SIZE));
                    physec_log(1, "\n");

                    if (kgs)
                    {
                        kgs->n_measures = cnt;
                        kgs->entropy_k = entropy(P.key, PHYSEC_KEY_SIZE_BYTES);
                        kgs->total = kgs->measure + kgs->signal_process + kgs->quantization + kgs->reconciliation + kgs->privacy_amp;
                    }

                    // data report
                    #if PHYSEC_DEBUG >= 4
                    
                        printf("(value)\n");
                        
                        printf("generated_key\n"); //id
                        printf("Generated key\n"); //Name
                        
                        // vlaue
                        {
                            int key_vec[PHYSEC_KEY_SIZE];
                            vec_key2vec(k, key_vec);
                            for(int i = 0; i < PHYSEC_KEY_SIZE; i++){
                                printf("%d", key_vec[i]);
                            }
                            printf("\n");
                        }

                    #endif

                    generated = true;
                }
                else
                { // if we did not get enough bits, we reset measurements
                    physec_log(1, "### QUANTIFICATION FAILED - RESET\n");
                    physec_log(1, "\n");


                    cnt = 0;
                    pkt.type = PHYSEC_PT_RESET;

                    if (kgs)
                        kgs->n_retries++;

                    lora_send((uint8_t*)&pkt, PHYSEC_KG_PKT_HEADER_SIZE, -1);
                }

                continue;
            }
            default:
                break;
        }

    }
    free(m.values);
}

/**
 * @brief Generate a PHYSEC Key with the wanted ID
 * 
 * @param sync          the synchronization structure
 * @param initiator     if the device is the initiator of waiter during the generation
 * @return PHYSEC_Key*  a pointer to the generated key
 */
static PHYSEC_Key*
PHYSEC_key_agg(PHYSEC_Sync *sync, bool initiator)
{

    // For debug
    #if PHYSEC_DEBUG >= 4
        first_pack = 1;
        first_quantization_window = 1;
    #endif

    PHYSEC_KeyGenStats *kgs = NULL;
    assert (sizeof(sync->rmt_dev_id) >= 4);
    physec_log2(1, "Starting Keygen with 0x%4x\n", *(uint32_t*) sync->rmt_dev_id);
    PHYSEC_KeyGenStats stats = { 0 };
    physec_verb_statement(1, kgs = &stats);

    PHYSEC_Key *k = malloc(sizeof(PHYSEC_Key));
    memset(k, 0, sizeof(PHYSEC_Key));
    if (initiator)
        PHYSEC_initiate_key_agg(k, sync, kgs);
    else
        PHYSEC_wait_key_agg(k, sync, kgs);

    physec_log(1, "\n### KEY GENERATION STATISTICS\n\n");
    physec_log2(1, "[+] Total KeyGen duration  : %f s\n", kgs->total );
    physec_log2(1, "[+] Measure duration : %f s\n", kgs->measure );
    physec_log2(1, "[+] Signal Processing duration : %f s\n", kgs->signal_process );
    physec_log2(1, "[+] Quantization duration : %f s\n", kgs->quantization );
    physec_log2(1, "[+] Reconciliation duration : %f s\n", kgs->reconciliation );
    physec_log2(1, "[+] Privacy Amplification duration : %f s\n", kgs->privacy_amp );
    physec_log2(1, "[+] Mismatch : %f\n", kgs->mismatch);
    physec_log2(1, "[+] NB measures : %d\n", kgs->n_measures );
    physec_log2(1, "[+] NB reset : %d\n", kgs->n_retries );
    physec_log2(1, "[+] Average measure delay : %d ms\n", kgs->avg_delay);
    physec_log2(1, "[+] Key entropy before Privacy amplification : %f\n", kgs->entropy_pk);
    physec_log2(1, "[+] Key entropy after Privacy amplification : %f\n", kgs->entropy_k);
    physec_log(1, "\n");

    return k;
}

/*************************
 * PHYSEC python bidings *
 *************************/

/**
 * @brief Just a sandbox function ready to be use from python, build your tests here
 * 
 * @param self 
 * @return STATIC 
 */
STATIC mp_obj_t
lora_physec_sandbox(mp_obj_t self){
    printf("---------- > PHYSEC sandbox > -----------\n");

    printf("---------- < PHYSEC sandbox < -----------\n");

    return mp_obj_new_int(0);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(lora_physec_sandbox_obj, lora_physec_sandbox);

/**
 * @brief Python biding for lora PHYSEC key aggreement
 * 
 * @param self              lora object
 * @param sync_obj          tuple containing synchronization id's
 * @param initiator_obj     boolean variable to determine wether we initiate the measure or not
 * @return mp_obj_t         the generated key 
 */
STATIC mp_obj_t
lora_physec_key_agg(mp_obj_t self, mp_obj_t sync_obj, mp_obj_t initiator_obj)
{
    size_t len = 0;
    mp_obj_t *tuple;
    
    // fetch args
    mp_obj_tuple_get(sync_obj, &len, &tuple);
    if (len != 2)
        nlr_raise(mp_obj_new_exception_msg(&mp_type_Exception, "sync_obj should be a byte tuple of length 2"));

    if (mp_obj_get_int(mp_obj_len(tuple[0])) != PHYSEC_DEV_ID_LEN)
        nlr_raise(mp_obj_new_exception_msg(&mp_type_Exception, "dev_id in sync_obj[0] should match PHYSEC_DEV_ID_LEN"));
    if (mp_obj_get_int(mp_obj_len(tuple[1])) != PHYSEC_DEV_ID_LEN)
        nlr_raise(mp_obj_new_exception_msg(&mp_type_Exception, "rmt_dev_id in sync_obj[1] should match PHYSEC_DEV_ID_LEN"));

    PHYSEC_Sync sync = { 0 };
    memcpy(&(sync.dev_id), mp_obj_str_get_str(tuple[0]), PHYSEC_DEV_ID_LEN);
    memcpy(&(sync.rmt_dev_id), mp_obj_str_get_str(tuple[1]), PHYSEC_DEV_ID_LEN);

    bool initiator = (bool) mp_obj_get_int(initiator_obj);

    // generate key
    PHYSEC_Key *k = PHYSEC_key_agg(&sync, initiator);

    return mp_obj_new_bytes(k->key, PHYSEC_KEY_SIZE_BYTES);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(lora_physec_key_agg_obj, lora_physec_key_agg);


/**
 * @brief Add a key (associated with an id) to the LORAPHYSEC mode key list 
 *        (allowing to test PHYSEC ciphering without the keygen process) 
 * 
 * @param self  lora object
 * @param id    the id to associate with the key
 * @param key   the key to add (16 bytes)
 * @return true 
 */
/*
STATIC mp_obj_t
lora_physec_add_key(mp_obj_t self, mp_obj_t id, mp_obj_t key)
{
    if (mp_obj_get_int(mp_obj_len(key)) != PHYSEC_KEY_SIZE_BYTES)
        nlr_raise(mp_obj_new_exception_msg(&mp_type_Exception, "key should match PHYSEC_KEY_SIZE_BYTES (16 bytes)"));

    int32_t rmt_id = mp_obj_get_int(id);
    peer_key_delete_by_peer_id(&(lora_obj.peer_key_list), rmt_id);

    peer_key_push(&(lora_obj.peer_key_list), rmt_id, (const uint8_t*) mp_obj_str_get_str(key));

    return mp_obj_new_bool(true);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(lora_physec_add_key_obj, lora_physec_add_key);
*/
#endif

STATIC const mp_map_elem_t lora_locals_dict_table[] = {
    // instance methods
    { MP_OBJ_NEW_QSTR(MP_QSTR_init),                  (mp_obj_t)&lora_init_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_join),                  (mp_obj_t)&lora_join_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_join_multicast_group),  (mp_obj_t)&lora_join_multicast_group_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_leave_multicast_group), (mp_obj_t)&lora_leave_multicast_group_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_tx_power),              (mp_obj_t)&lora_tx_power_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_bandwidth),             (mp_obj_t)&lora_bandwidth_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_frequency),             (mp_obj_t)&lora_frequency_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_coding_rate),           (mp_obj_t)&lora_coding_rate_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_preamble),              (mp_obj_t)&lora_preamble_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_sf),                    (mp_obj_t)&lora_sf_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_power_mode),            (mp_obj_t)&lora_power_mode_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_stats),                 (mp_obj_t)&lora_stats_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_has_joined),            (mp_obj_t)&lora_has_joined_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_add_channel),           (mp_obj_t)&lora_add_channel_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_remove_channel),        (mp_obj_t)&lora_remove_channel_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_mac),                   (mp_obj_t)&lora_mac_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_compliance_test),       (mp_obj_t)&lora_compliance_test_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_callback),              (mp_obj_t)&lora_callback_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_events),                (mp_obj_t)&lora_events_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_ischannel_free),        (mp_obj_t)&lora_ischannel_free_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_set_battery_level),     (mp_obj_t)&lora_set_battery_level_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_nvram_save),            (mp_obj_t)&lora_nvram_save_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_nvram_restore),         (mp_obj_t)&lora_nvram_restore_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_nvram_erase),           (mp_obj_t)&lora_nvram_erase_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_airtime),               (mp_obj_t)&lora_airtime_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_reset),                 (mp_obj_t)&lora_reset_obj },
#ifdef PHYSEC
    { MP_OBJ_NEW_QSTR(MP_QSTR_physec_sandbox),     (mp_obj_t)&lora_physec_sandbox_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_physec_key_agg),     (mp_obj_t)&lora_physec_key_agg_obj },
    //{ MP_OBJ_NEW_QSTR(MP_QSTR_physec_add_key),     (mp_obj_t)&lora_physec_add_key_obj },
#endif

#ifdef LORA_OPENTHREAD_ENABLED
    { MP_OBJ_NEW_QSTR(MP_QSTR_Mesh),                (mp_obj_t)&lora_mesh_type },
#endif  // #ifdef LORA_OPENTHREAD_ENABLED

    // exceptions
    { MP_OBJ_NEW_QSTR(MP_QSTR_timeout),             (mp_obj_t)&mp_type_TimeoutError },

    // class constants
    { MP_OBJ_NEW_QSTR(MP_QSTR_LORA),                MP_OBJ_NEW_SMALL_INT(E_LORA_STACK_MODE_LORA) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_LORAWAN),             MP_OBJ_NEW_SMALL_INT(E_LORA_STACK_MODE_LORAWAN) },
    #ifdef PHYSEC
    { MP_OBJ_NEW_QSTR(MP_QSTR_LORAPHYSEC),          MP_OBJ_NEW_SMALL_INT(E_LORA_STACK_MODE_LORAPHYSEC) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_PHYSEC_PROBE_TIMEOUT),    MP_OBJ_NEW_SMALL_INT(PHYSEC_PROBE_TIMEOUT) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_PHYSEC_KEYGEN_FREQUENCY), MP_OBJ_NEW_SMALL_INT(PHYSEC_KEYGEN_FREQUENCY) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_PHYSEC_DEV_ID_LEN),       MP_OBJ_NEW_SMALL_INT(PHYSEC_DEV_ID_LEN) },
    #endif

    { MP_OBJ_NEW_QSTR(MP_QSTR_OTAA),                MP_OBJ_NEW_SMALL_INT(E_LORA_ACTIVATION_OTAA) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_ABP),                 MP_OBJ_NEW_SMALL_INT(E_LORA_ACTIVATION_ABP) },

    { MP_OBJ_NEW_QSTR(MP_QSTR_ALWAYS_ON),           MP_OBJ_NEW_SMALL_INT(E_LORA_MODE_ALWAYS_ON) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_TX_ONLY),             MP_OBJ_NEW_SMALL_INT(E_LORA_MODE_TX_ONLY) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_SLEEP),               MP_OBJ_NEW_SMALL_INT(E_LORA_MODE_SLEEP) },

    { MP_OBJ_NEW_QSTR(MP_QSTR_BW_125KHZ),           MP_OBJ_NEW_SMALL_INT(E_LORA_BW_125_KHZ) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_BW_250KHZ),           MP_OBJ_NEW_SMALL_INT(E_LORA_BW_250_KHZ) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_BW_500KHZ),           MP_OBJ_NEW_SMALL_INT(E_LORA_BW_500_KHZ) },

    { MP_OBJ_NEW_QSTR(MP_QSTR_CODING_4_5),          MP_OBJ_NEW_SMALL_INT(E_LORA_CODING_4_5) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_CODING_4_6),          MP_OBJ_NEW_SMALL_INT(E_LORA_CODING_4_6) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_CODING_4_7),          MP_OBJ_NEW_SMALL_INT(E_LORA_CODING_4_7) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_CODING_4_8),          MP_OBJ_NEW_SMALL_INT(E_LORA_CODING_4_8) },

    { MP_OBJ_NEW_QSTR(MP_QSTR_RX_PACKET_EVENT),     MP_OBJ_NEW_SMALL_INT(MODLORA_RX_EVENT) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_TX_PACKET_EVENT),     MP_OBJ_NEW_SMALL_INT(MODLORA_TX_EVENT) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_TX_FAILED_EVENT),     MP_OBJ_NEW_SMALL_INT(MODLORA_TX_FAILED_EVENT) },

    { MP_OBJ_NEW_QSTR(MP_QSTR_CLASS_A),             MP_OBJ_NEW_SMALL_INT(CLASS_A) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_CLASS_C),             MP_OBJ_NEW_SMALL_INT(CLASS_C) },

    { MP_OBJ_NEW_QSTR(MP_QSTR_AS923),               MP_OBJ_NEW_SMALL_INT(LORAMAC_REGION_AS923) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_AU915),               MP_OBJ_NEW_SMALL_INT(LORAMAC_REGION_AU915) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_EU868),               MP_OBJ_NEW_SMALL_INT(LORAMAC_REGION_EU868) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_US915),               MP_OBJ_NEW_SMALL_INT(LORAMAC_REGION_US915) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_CN470),               MP_OBJ_NEW_SMALL_INT(LORAMAC_REGION_CN470) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_IN865),               MP_OBJ_NEW_SMALL_INT(LORAMAC_REGION_IN865) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_EU433),               MP_OBJ_NEW_SMALL_INT(LORAMAC_REGION_EU433) },

};

STATIC MP_DEFINE_CONST_DICT(lora_locals_dict, lora_locals_dict_table);

const mod_network_nic_type_t mod_network_nic_type_lora = {
    .base = {
        { &mp_type_type },
        .name = MP_QSTR_LoRa,
        .make_new = lora_make_new,
        .locals_dict = (mp_obj_t)&lora_locals_dict,
     },

    .n_socket = lora_socket_socket,
    .n_close = lora_socket_close,
    .n_send = lora_socket_send,
    .n_sendto = lora_socket_sendto,
    .n_recv = lora_socket_recv,
    .n_recvfrom = lora_socket_recvfrom,
    .n_settimeout = lora_socket_settimeout,
    .n_setsockopt = lora_socket_setsockopt,
    .n_bind = lora_socket_bind,
    .n_ioctl = lora_socket_ioctl,
};

///******************************************************************************/
//// Micro Python bindings; LoRa socket

static int lora_socket_socket (mod_network_socket_obj_t *s, int *_errno) {
    if (lora_obj.state == E_LORA_STATE_NOINIT) {
        *_errno = MP_ENETDOWN;
        return -1;
    }
    s->sock_base.u.sd = 1;

#ifdef LORA_OPENTHREAD_ENABLED
    // if mesh is enabled, assume socket is for mesh, not for LoraWAN
    if (lora_mesh_ready()) {
        return mesh_socket_open(s, _errno);
    }
#endif  // #ifdef LORA_OPENTHREAD_ENABLED

    uint32_t dr = DR_0;
    switch (lora_obj.region) {
    case LORAMAC_REGION_AS923:
    case LORAMAC_REGION_EU868:
    case LORAMAC_REGION_EU433:
    case LORAMAC_REGION_CN470:
        dr = DR_5;
        break;
    case LORAMAC_REGION_AU915:
    case LORAMAC_REGION_US915:
    case LORAMAC_REGION_US915_HYBRID:
        dr = DR_4;
        break;
    default:
        break;
    }
    LORAWAN_SOCKET_SET_DR(s->sock_base.u.sd, dr);

    // port number 2 is the default one
    LORAWAN_SOCKET_SET_PORT(s->sock_base.u.sd, 2);
    return 0;
}

static void lora_socket_close (mod_network_socket_obj_t *s) {
    s->sock_base.u.sd = -1;
#ifdef LORA_OPENTHREAD_ENABLED
    mesh_socket_close(s);
#endif  // #ifdef LORA_OPENTHREAD_ENABLED
}

static int lora_socket_send (mod_network_socket_obj_t *s, const byte *buf, mp_uint_t len, int *_errno) {
    mp_int_t n_bytes = -1;

    LORA_CHECK_SOCKET(s);

#ifdef LORA_OPENTHREAD_ENABLED
    if (lora_mesh_ready()) {
            *_errno = MP_EOPNOTSUPP;
            return -1;
    }
#endif  // #ifdef LORA_OPENTHREAD_ENABLED

    // is the radio able to transmit
    if (lora_obj.pwr_mode == E_LORA_MODE_SLEEP) {
        *_errno = MP_ENETDOWN;
    } else if (len > LORA_PAYLOAD_SIZE_MAX) {
        *_errno = MP_EMSGSIZE;
    } else if (len > 0) {

        switch(lora_obj.stack_mode){

            case E_LORA_STACK_MODE_LORA:
                n_bytes = lora_send (buf, len, s->sock_base.timeout);
                break;

            case E_LORA_STACK_MODE_LORAWAN:
                if (lora_obj.joined) {
                    n_bytes = lorawan_send (buf, len, s->sock_base.timeout,
                                            LORAWAN_SOCKET_IS_CONFIRMED(s->sock_base.u.sd),
                                            LORAWAN_SOCKET_GET_DR(s->sock_base.u.sd),
                                            LORAWAN_SOCKET_GET_PORT(s->sock_base.u.sd));
                } else {
                    *_errno = MP_ENETDOWN;
                    return -1;
                }
                break;

            #ifdef PHYSEC
            // LORA PHYSEC MODE send routine
            case E_LORA_STACK_MODE_LORAPHYSEC:
                {
                    PHYSEC_Key key = { 0 };
                    //uint8_t key[16];
                    physec_log(2, "--- lora_socket_send using LORAPHYSEC protocol ---\n");
                    physec_log2(2, "\t 0x%04x -> 0x%04X\n", lora_obj.physec_device_id, lora_obj.physec_remote_device_id);
                    if(peer_key_list_get_key_by_peer_id(&(lora_obj.peer_key_list), lora_obj.physec_remote_device_id, key.key)){
                        physec_log(3, "\tkey : [");
                        for(int i = 0; i<16; i++){
                            physec_log2(3, " 0x%02x", key.key[i]);
                        }
                        physec_log(3, "]\n");
                        // encrypt msg
                        return PHYSEC_send(&key, buf, len, s->sock_base.timeout);
                    }else{
                        physec_log(2, "\t key : Not found. Generating new key using PHYSEC...\n");
                        // gen key
                        PHYSEC_Sync sync = { 0 };
                        memcpy(&(sync.dev_id), &(lora_obj.physec_device_id), sizeof(PHYSEC_DEV_ID_LEN));
                        memcpy(&(sync.rmt_dev_id), &(lora_obj.physec_remote_device_id), sizeof(PHYSEC_DEV_ID_LEN));
                        PHYSEC_Key *newkey = PHYSEC_key_agg(&sync, lora_obj.physec_initiator);

                        physec_log(2, "PHYSEC generation done !\n");

                        // register key
                        peer_key_push(&(lora_obj.peer_key_list), lora_obj.physec_remote_device_id, (const uint8_t*) newkey->key);

                        free(newkey);

                        // encrypt msg
                        return PHYSEC_send(newkey, buf, len, s->sock_base.timeout);
                    }
                }
                // n_bytes = lora_send (buf, len, s->sock_base.timeout);
                break;
            #endif

            default :
                nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, " lora_socket_send : invalid mode %d", lora_obj.stack_mode));
                break;

        }

        if (n_bytes == 0) {
            *_errno = MP_EAGAIN;
            n_bytes = -1;
        } else if (n_bytes < 0) {
            *_errno = MP_EMSGSIZE;
        }
    } else {
        n_bytes = 0;
    }
    return n_bytes;
}

static int lora_socket_recv (mod_network_socket_obj_t *s, byte *buf, mp_uint_t len, int *_errno) {
    LORA_CHECK_SOCKET(s);

#ifdef LORA_OPENTHREAD_ENABLED
    if (lora_mesh_ready()) {
        *_errno = MP_EOPNOTSUPP;
        return -1;
    }
#endif  // #ifdef LORA_OPENTHREAD_ENABLED

    if (lora_obj.stack_mode == E_LORA_STACK_MODE_LORAPHYSEC)
    {

        PHYSEC_Key key = { 0 };
        //uint8_t key[16];
        physec_log(2, "--- lora_socket_recv using LORAPHYSEC protocol ---\n");
        physec_log2(2, "\t 0x%04X -> 0x%04X\n", lora_obj.physec_device_id, lora_obj.physec_remote_device_id);
        if(peer_key_list_get_key_by_peer_id(&(lora_obj.peer_key_list), lora_obj.physec_remote_device_id, key.key)){
            physec_log(3, "\tkey : [");
            for(int i = 0; i<16; i++){
                physec_log2(3, " 0x%02x", key.key[i]);
            }
            physec_log(3, "]\n");

            // recv & decrypt msg
            return PHYSEC_recv(&key, buf, len, s->sock_base.timeout);
        }else{
            physec_log(2, "\t key : Not found. Generating new key using PHYSEC...\n");
            // gen key
            PHYSEC_Sync sync = { 0 };
            memcpy(&(sync.dev_id), &(lora_obj.physec_device_id), sizeof(PHYSEC_DEV_ID_LEN));
            memcpy(&(sync.rmt_dev_id), &(lora_obj.physec_remote_device_id), sizeof(PHYSEC_DEV_ID_LEN));
            PHYSEC_Key *newkey = PHYSEC_key_agg(&sync, lora_obj.physec_initiator);

            physec_log(2, "PHYSEC generation done !\n");

            // register key
            peer_key_push(&(lora_obj.peer_key_list), lora_obj.physec_remote_device_id, (const uint8_t*) newkey->key);

            // recv & decrypt msg
            int ret = PHYSEC_recv(newkey, buf, len, s->sock_base.timeout);

            free(newkey);

            return ret;
        }
    }
    else
    {
        int ret = lora_recv (buf, len, s->sock_base.timeout, NULL);
        if (ret < 0) {
            *_errno = MP_EAGAIN;
            return -1;
        }
        return ret;

    }
}

static int lora_socket_recvfrom (mod_network_socket_obj_t *s, byte *buf, mp_uint_t len, byte *ip, mp_uint_t *port, int *_errno) {
    LORA_CHECK_SOCKET(s);

#ifdef LORA_OPENTHREAD_ENABLED
    if (lora_mesh_ready()) {
        return mesh_socket_recvfrom(s, buf, len, ip, port, _errno);
    }
#endif  // #ifdef LORA_OPENTHREAD_ENABLED

    *port = 0;      // in case there's no data received
    int ret = lora_recv (buf, len, s->sock_base.timeout, (uint32_t *)port);
    if (ret < 0) {
        *_errno = MP_EAGAIN;
        return -1;
    }
    return ret;
}

static int lora_socket_setsockopt(mod_network_socket_obj_t *s, mp_uint_t level, mp_uint_t opt, const void *optval, mp_uint_t optlen, int *_errno) {
    LORA_CHECK_SOCKET(s);

#ifdef LORA_OPENTHREAD_ENABLED
    if (lora_mesh_ready()) {
        *_errno = MP_EOPNOTSUPP;
        return -1;
    }
#endif  // #ifdef LORA_OPENTHREAD_ENABLED

    if (level != SOL_LORA) {
        *_errno = MP_EOPNOTSUPP;
        return -1;
    }

    if (opt == SO_LORAWAN_CONFIRMED) {
        if (*(uint8_t *)optval) {
            LORAWAN_SOCKET_SET_CONFIRMED(s->sock_base.u.sd);
        } else {
            LORAWAN_SOCKET_CLR_CONFIRMED(s->sock_base.u.sd);
        }
    } else if (opt == SO_LORAWAN_DR) {
        if (!lora_validate_data_rate(*(uint32_t *)optval)) {
            *_errno = MP_EOPNOTSUPP;
            return -1;
        }
        LORAWAN_SOCKET_SET_DR(s->sock_base.u.sd, *(uint8_t *)optval);
    } else {
        *_errno = MP_EOPNOTSUPP;
        return -1;
    }
    return 0;
}

static int lora_socket_settimeout (mod_network_socket_obj_t *s, mp_int_t timeout_ms, int *_errno) {
    LORA_CHECK_SOCKET(s);

#ifdef LORA_OPENTHREAD_ENABLED
    if (lora_mesh_ready()) {
        *_errno = MP_EOPNOTSUPP;
        return -1;
    }
#endif  // #ifdef LORA_OPENTHREAD_ENABLED

    s->sock_base.timeout = timeout_ms;
    return 0;
}

static int lora_socket_bind(mod_network_socket_obj_t *s, byte *ip, mp_uint_t port, int *_errno) {
    LORA_CHECK_SOCKET(s);

#ifdef LORA_OPENTHREAD_ENABLED
    if (lora_mesh_ready()) {
        return mesh_socket_bind(s, ip, port, _errno);
    }
#endif  // #ifdef LORA_OPENTHREAD_ENABLED

    if (port > 224) {
        *_errno = MP_EOPNOTSUPP;
        return -1;
    }
    LORAWAN_SOCKET_SET_PORT(s->sock_base.u.sd, port);
    return 0;
}

static int lora_socket_ioctl (mod_network_socket_obj_t *s, mp_uint_t request, mp_uint_t arg, int *_errno) {
    mp_int_t ret = 0;

    LORA_CHECK_SOCKET(s);

#ifdef LORA_OPENTHREAD_ENABLED
    if (lora_mesh_ready()) {
        *_errno = MP_EOPNOTSUPP;
        return -1;
    }
#endif  // #ifdef LORA_OPENTHREAD_ENABLED

    if (request == MP_STREAM_POLL) {
        mp_uint_t flags = arg;
        if ((flags & MP_STREAM_POLL_RD) && lora_rx_any()) {
            ret |= MP_STREAM_POLL_RD;
        }
        if ((flags & MP_STREAM_POLL_WR) && lora_tx_space()) {
            ret |= MP_STREAM_POLL_WR;
        }
    } else {
        *_errno = MP_EINVAL;
        ret = MP_STREAM_ERROR;
    }
    return ret;
}

static int lora_socket_sendto(struct _mod_network_socket_obj_t *s, const byte *buf, mp_uint_t len, byte *ip, mp_uint_t port, int *_errno) {
    LORA_CHECK_SOCKET(s);

#ifdef LORA_OPENTHREAD_ENABLED
    if (lora_mesh_ready()) {
        return mesh_socket_sendto(s, buf, len, ip, port, _errno);
    }
#endif  // #ifdef LORA_OPENTHREAD_ENABLED

    // not implemented for LoraWAN
    *_errno = MP_EOPNOTSUPP;
    return -1;
}
