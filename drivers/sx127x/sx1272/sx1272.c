/*
 * This file is derived from the MicroPython project, http://micropython.org/
 *
 * Copyright (c) 2021, Pycom Limited and its licensors.
 *
 * This software is licensed under the GNU GPL version 3 or any later version,
 * with permitted additional terms. For more information see the Pycom Licence
 * v1.0 document supplied with this file, or available at:
 * https://www.pycom.io/opensource/licensing
 *
 * This file contains code under the following copyright and licensing notices.
 * The code has been changed but otherwise retained.
 */

/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: Generic SX1272 driver implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
#include <math.h>
#include <string.h>
#include "board.h"
#include "radio.h"
#include "sx1272.h"
#include "sx1272-board.h"
#include "esp_attr.h"
#include "esp32_mphal.h"
#include <time.h>

/*
 * Local types definition
 */

/*!
 * Radio registers definition
 */
typedef struct
{
    RadioModems_t Modem;
    uint8_t       Addr;
    uint8_t       Value;
}RadioRegisters_t;


/*
 * Private functions prototypes
 */


/*!
 * \brief Sets the SX1272 in transmission mode for the given time
 * \param [IN] timeout Transmission timeout [ms] [0: continuous, others timeout]
 */
void SX1272SetTx( uint32_t timeout );

/*!
 * \brief Writes the buffer contents to the SX1272 FIFO
 *
 * \param [IN] buffer Buffer containing data to be put on the FIFO.
 * \param [IN] size Number of bytes to be written to the FIFO
 */
void SX1272WriteFifo( uint8_t *buffer, uint8_t size );

/*!
 * \brief Reads the contents of the SX1272 FIFO
 *
 * \param [OUT] buffer Buffer where to copy the FIFO read data.
 * \param [IN] size Number of bytes to be read from the FIFO
 */
void SX1272ReadFifo( uint8_t *buffer, uint8_t size );

/*
 * SX1272 DIO IRQ callback functions prototype
 */

/*!
 * \brief Common DIO IRQ callback
 */
static void SX1272OnDioIrq (void);

/*!
 * \brief DIO 0 IRQ callback
 */
void SX1272OnDio0Irq( void );

/*!
 * \brief DIO 1 IRQ callback
 */
void SX1272OnDio1Irq( void );

/*!
 * \brief DIO 2 IRQ callback
 */
void SX1272OnDio2Irq( void );

/*!
 * \brief DIO 3 IRQ callback
 */
void SX1272OnDio3Irq( void );

/*!
 * \brief DIO 4 IRQ callback
 */
void SX1272OnDio4Irq( void );

/*!
 * \brief DIO 5 IRQ callback
 */
void SX1272OnDio5Irq( void );

/*!
 * \brief Tx & Rx timeout timer callback
 */
void SX1272OnTimeoutIrq( void );

/*!
 * \brief General radio flags check callback
 */
void SX1272RadioFlagsIrq (void);

/*
 * Private global constants
 */

/*!
 * Radio hardware registers initialization
 *
 * \remark RADIO_INIT_REGISTERS_VALUE is defined in sx1272-board.h file
 */
const RadioRegisters_t RadioRegsInit[] = RADIO_INIT_REGISTERS_VALUE;

/*!
 * Constant values need to compute the RSSI value
 */
#define RSSI_OFFSET                                 -139

/*
 * Private global variables
 */

/*!
 * Radio callbacks variable
 */
static RadioEvents_t *RadioEvents;

/*!
 * Reception buffer
 */
static uint8_t RxTxBuffer[RX_BUFFER_SIZE];

/*
 * Public global variables
 */

/*!
 * Radio hardware and global parameters
 */
SX1272_t SX1272;

/*!
 * Hardware DIO IRQ callback initialization
 */
DioIrqHandler *DioIrq[] = { SX1272OnDioIrq };

/*!
 * Tx and Rx timers
 */
static TimerEvent_t TxTimeoutTimer;
static TimerEvent_t RxTimeoutTimer;
static TimerEvent_t RadioIrqFlagsTimer;

/*
 * Radio driver functions implementation
 */

void SX1272Init( RadioEvents_t *events )
{
    uint8_t i;

    RadioEvents = events;

    // Initialize driver timeout timers
    TimerInit( &TxTimeoutTimer, SX1272OnTimeoutIrq );
    TimerInit( &RxTimeoutTimer, SX1272OnTimeoutIrq );
    TimerInit( &RadioIrqFlagsTimer, SX1272RadioFlagsIrq );
    TimerSetValue( &RadioIrqFlagsTimer, 1 );

    SX1272Reset( );

    SX1272SetOpMode( RF_OPMODE_SLEEP );

    SX1272IoIrqInit( DioIrq );

    for( i = 0; i < sizeof( RadioRegsInit ) / sizeof( RadioRegisters_t ); i++ )
    {
        SX1272SetModem( RadioRegsInit[i].Modem );
        SX1272Write( RadioRegsInit[i].Addr, RadioRegsInit[i].Value );
    }

    SX1272SetModem( MODEM_FSK );

    SX1272.Settings.State = RF_IDLE;
    SX1272.irqFlags = 0;
}

IRAM_ATTR RadioState_t SX1272GetStatus( void )
{
    return SX1272.Settings.State;
}

IRAM_ATTR void SX1272SetChannel( uint32_t freq )
{
    SX1272.Settings.Channel = freq;
    freq = ( uint32_t )( ( double )freq / ( double )FREQ_STEP );
    SX1272Write( REG_FRFMSB, ( uint8_t )( ( freq >> 16 ) & 0xFF ) );
    SX1272Write( REG_FRFMID, ( uint8_t )( ( freq >> 8 ) & 0xFF ) );
    SX1272Write( REG_FRFLSB, ( uint8_t )( freq & 0xFF ) );
}

IRAM_ATTR uint32_t SX1272GetChannel( void )
{
    return SX1272.Settings.Channel;
}

bool SX1272IsChannelFree( RadioModems_t modem, uint32_t freq, int16_t rssiThresh, uint32_t maxCarrierSenseTime )
{
    bool status = true;
    int16_t rssi = 0;

    SX1272SetModem( modem );

    SX1272SetChannel( freq );

    SX1272SetOpMode( RF_OPMODE_RECEIVER );

    DelayMs( 1 );

    // Perform carrier sense for maxCarrierSenseTime
    do {
        rssi = SX1272ReadRssi( modem );

        if( rssi > rssiThresh ) {
            status = false;
            break;
        }
        DelayMs( 1 );
        maxCarrierSenseTime -= 1;
    } while( maxCarrierSenseTime > 0 );
    SX1272SetSleep( );
    return status;
}

uint32_t SX1272Random( void )
{
    uint8_t i;
    uint32_t rnd = 0;

    /*
     * Radio setup for random number generation
     */
    // Set LoRa modem ON
    SX1272SetModem( MODEM_LORA );

    // Disable LoRa modem interrupts
    SX1272Write( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                  RFLR_IRQFLAGS_RXDONE |
                  RFLR_IRQFLAGS_PAYLOADCRCERROR |
                  RFLR_IRQFLAGS_VALIDHEADER |
                  RFLR_IRQFLAGS_TXDONE |
                  RFLR_IRQFLAGS_CADDONE |
                  RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                  RFLR_IRQFLAGS_CADDETECTED );

    // Set radio in continuous reception
    SX1272SetOpMode( RF_OPMODE_RECEIVER );

    for( i = 0; i < 32; i++ )
    {
        DelayMs( 1 );
        // Unfiltered RSSI value reading. Only takes the LSB value
        rnd |= ( ( uint32_t )SX1272Read( REG_LR_RSSIWIDEBAND ) & 0x01 ) << i;
    }

    SX1272SetSleep( );

    return rnd;
}

IRAM_ATTR void SX1272SetRxConfig( RadioModems_t modem, uint32_t bandwidth,
                         uint32_t datarate, uint8_t coderate,
                         uint32_t bandwidthAfc, uint16_t preambleLen,
                         uint16_t symbTimeout, bool fixLen,
                         uint8_t payloadLen,
                         bool crcOn, bool freqHopOn, uint8_t hopPeriod,
                         bool iqInverted, bool rxContinuous )
{
    SX1272SetModem( modem );

    switch( modem )
    {
    case MODEM_FSK:
        break;
    case MODEM_LORA:
        {
            SX1272.Settings.LoRa.Bandwidth = bandwidth;
            SX1272.Settings.LoRa.Datarate = datarate;
            SX1272.Settings.LoRa.Coderate = coderate;
            SX1272.Settings.LoRa.PreambleLen = preambleLen;
            SX1272.Settings.LoRa.FixLen = fixLen;
            SX1272.Settings.LoRa.PayloadLen = payloadLen;
            SX1272.Settings.LoRa.CrcOn = crcOn;
            SX1272.Settings.LoRa.FreqHopOn = freqHopOn;
            SX1272.Settings.LoRa.HopPeriod = hopPeriod;
            SX1272.Settings.LoRa.RxIqInverted = iqInverted;
            SX1272.Settings.LoRa.RxContinuous = rxContinuous;

            if( datarate > 12 )
            {
                datarate = 12;
            }
            else if( datarate < 6 )
            {
                datarate = 6;
            }

            if( ( ( bandwidth == 0 ) && ( ( datarate == 11 ) || ( datarate == 12 ) ) ) ||
                ( ( bandwidth == 1 ) && ( datarate == 12 ) ) )
            {
                SX1272.Settings.LoRa.LowDatarateOptimize = 0x01;
            }
            else
            {
                SX1272.Settings.LoRa.LowDatarateOptimize = 0x00;
            }

            SX1272Write( REG_LR_MODEMCONFIG1,
                         ( SX1272Read( REG_LR_MODEMCONFIG1 ) &
                           RFLR_MODEMCONFIG1_BW_MASK &
                           RFLR_MODEMCONFIG1_CODINGRATE_MASK &
                           RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK &
                           RFLR_MODEMCONFIG1_RXPAYLOADCRC_MASK &
                           RFLR_MODEMCONFIG1_LOWDATARATEOPTIMIZE_MASK ) |
                           ( bandwidth << 6 ) | ( coderate << 3 ) |
                           ( fixLen << 2 ) | ( crcOn << 1 ) |
                           SX1272.Settings.LoRa.LowDatarateOptimize );

            SX1272Write( REG_LR_MODEMCONFIG2,
                         ( SX1272Read( REG_LR_MODEMCONFIG2 ) &
                           RFLR_MODEMCONFIG2_SF_MASK &
                           RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK ) |
                           ( datarate << 4 ) |
                           ( ( symbTimeout >> 8 ) & ~RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK ) );

            SX1272Write( REG_LR_SYMBTIMEOUTLSB, ( uint8_t )( symbTimeout & 0xFF ) );

            SX1272Write( REG_LR_PREAMBLEMSB, ( uint8_t )( ( preambleLen >> 8 ) & 0xFF ) );
            SX1272Write( REG_LR_PREAMBLELSB, ( uint8_t )( preambleLen & 0xFF ) );

            if( fixLen == 1 )
            {
                SX1272Write( REG_LR_PAYLOADLENGTH, payloadLen );
            }

            if( SX1272.Settings.LoRa.FreqHopOn == true )
            {
                SX1272Write( REG_LR_PLLHOP, ( SX1272Read( REG_LR_PLLHOP ) & RFLR_PLLHOP_FASTHOP_MASK ) | RFLR_PLLHOP_FASTHOP_ON );
                SX1272Write( REG_LR_HOPPERIOD, SX1272.Settings.LoRa.HopPeriod );
            }

            if( datarate == 6 )
            {
                SX1272Write( REG_LR_DETECTOPTIMIZE,
                             ( SX1272Read( REG_LR_DETECTOPTIMIZE ) &
                               RFLR_DETECTIONOPTIMIZE_MASK ) |
                               RFLR_DETECTIONOPTIMIZE_SF6 );
                SX1272Write( REG_LR_DETECTIONTHRESHOLD,
                             RFLR_DETECTIONTHRESH_SF6 );
            }
            else
            {
                SX1272Write( REG_LR_DETECTOPTIMIZE,
                             ( SX1272Read( REG_LR_DETECTOPTIMIZE ) &
                             RFLR_DETECTIONOPTIMIZE_MASK ) |
                             RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12 );
                SX1272Write( REG_LR_DETECTIONTHRESHOLD,
                             RFLR_DETECTIONTHRESH_SF7_TO_SF12 );
            }
        }
        break;
    }
}

void SX1272SetTxConfig( RadioModems_t modem, int8_t power, uint32_t fdev,
                        uint32_t bandwidth, uint32_t datarate,
                        uint8_t coderate, uint16_t preambleLen,
                        bool fixLen, bool crcOn, bool freqHopOn,
                        uint8_t hopPeriod, bool iqInverted, uint32_t timeout )
{
    SX1272SetModem( modem );

    SX1272SetRfTxPower( power );


    switch( modem )
    {
    case MODEM_FSK:
        break;
    case MODEM_LORA:
        {
            SX1272.Settings.LoRa.Power = power;
            SX1272.Settings.LoRa.Bandwidth = bandwidth;
            SX1272.Settings.LoRa.Datarate = datarate;
            SX1272.Settings.LoRa.Coderate = coderate;
            SX1272.Settings.LoRa.PreambleLen = preambleLen;
            SX1272.Settings.LoRa.FixLen = fixLen;
            SX1272.Settings.LoRa.FreqHopOn = freqHopOn;
            SX1272.Settings.LoRa.HopPeriod = hopPeriod;
            SX1272.Settings.LoRa.CrcOn = crcOn;
            SX1272.Settings.LoRa.TxIqInverted = iqInverted;
            SX1272.Settings.LoRa.TxTimeout = timeout;

            if( datarate > 12 )
            {
                datarate = 12;
            }
            else if( datarate < 6 )
            {
                datarate = 6;
            }
            if( ( ( bandwidth == 0 ) && ( ( datarate == 11 ) || ( datarate == 12 ) ) ) ||
                ( ( bandwidth == 1 ) && ( datarate == 12 ) ) )
            {
                SX1272.Settings.LoRa.LowDatarateOptimize = 0x01;
            }
            else
            {
                SX1272.Settings.LoRa.LowDatarateOptimize = 0x00;
            }

            if( SX1272.Settings.LoRa.FreqHopOn == true )
            {
                SX1272Write( REG_LR_PLLHOP, ( SX1272Read( REG_LR_PLLHOP ) & RFLR_PLLHOP_FASTHOP_MASK ) | RFLR_PLLHOP_FASTHOP_ON );
                SX1272Write( REG_LR_HOPPERIOD, SX1272.Settings.LoRa.HopPeriod );
            }

            SX1272Write( REG_LR_MODEMCONFIG1,
                         ( SX1272Read( REG_LR_MODEMCONFIG1 ) &
                           RFLR_MODEMCONFIG1_BW_MASK &
                           RFLR_MODEMCONFIG1_CODINGRATE_MASK &
                           RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK &
                           RFLR_MODEMCONFIG1_RXPAYLOADCRC_MASK &
                           RFLR_MODEMCONFIG1_LOWDATARATEOPTIMIZE_MASK ) |
                           ( bandwidth << 6 ) | ( coderate << 3 ) |
                           ( fixLen << 2 ) | ( crcOn << 1 ) |
                           SX1272.Settings.LoRa.LowDatarateOptimize );

            SX1272Write( REG_LR_MODEMCONFIG2,
                        ( SX1272Read( REG_LR_MODEMCONFIG2 ) &
                          RFLR_MODEMCONFIG2_SF_MASK ) |
                          ( datarate << 4 ) );


            SX1272Write( REG_LR_PREAMBLEMSB, ( preambleLen >> 8 ) & 0x00FF );
            SX1272Write( REG_LR_PREAMBLELSB, preambleLen & 0xFF );

            if( datarate == 6 )
            {
                SX1272Write( REG_LR_DETECTOPTIMIZE,
                             ( SX1272Read( REG_LR_DETECTOPTIMIZE ) &
                               RFLR_DETECTIONOPTIMIZE_MASK ) |
                               RFLR_DETECTIONOPTIMIZE_SF6 );
                SX1272Write( REG_LR_DETECTIONTHRESHOLD,
                             RFLR_DETECTIONTHRESH_SF6 );
            }
            else
            {
                SX1272Write( REG_LR_DETECTOPTIMIZE,
                             ( SX1272Read( REG_LR_DETECTOPTIMIZE ) &
                             RFLR_DETECTIONOPTIMIZE_MASK ) |
                             RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12 );
                SX1272Write( REG_LR_DETECTIONTHRESHOLD,
                             RFLR_DETECTIONTHRESH_SF7_TO_SF12 );
            }
        }
        break;
    }
}

uint32_t SX1272GetTimeOnAir( RadioModems_t modem, uint8_t pktLen )
{
    uint32_t airTime = 0;

    switch( modem )
    {
    case MODEM_FSK:
        break;
    case MODEM_LORA:
        {
            double bw = 0.0;
            switch( SX1272.Settings.LoRa.Bandwidth )
            {
            case 0: // 125 kHz
                bw = 125000;
                break;
            case 1: // 250 kHz
                bw = 250000;
                break;
            case 2: // 500 kHz
                bw = 500000;
                break;
            }

            // Symbol rate : time for one symbol (secs)
            double rs = bw / ( 1 << SX1272.Settings.LoRa.Datarate );
            double ts = 1 / rs;
            // time of preamble
            double tPreamble = ( SX1272.Settings.LoRa.PreambleLen + 4.25 ) * ts;
            // Symbol length of payload and time
            double tmp = ceil( ( 8 * pktLen - 4 * SX1272.Settings.LoRa.Datarate +
                                 28 + 16 * SX1272.Settings.LoRa.CrcOn -
                                 ( SX1272.Settings.LoRa.FixLen ? 20 : 0 ) ) /
                                 ( double )( 4 * ( SX1272.Settings.LoRa.Datarate -
                                 ( ( SX1272.Settings.LoRa.LowDatarateOptimize > 0 ) ? 2 : 0 ) ) ) ) *
                                 ( SX1272.Settings.LoRa.Coderate + 4 );
            double nPayload = 8 + ( ( tmp > 0 ) ? tmp : 0 );
            double tPayload = nPayload * ts;
            // Time on air
            double tOnAir = tPreamble + tPayload;
            // return ms secs
            airTime = floor( tOnAir * 1000 + 0.999 );
        }
        break;
    }
    return airTime;
}

void SX1272Send( uint8_t *buffer, uint8_t size )
{
    uint32_t txTimeout = 0;

    switch( SX1272.Settings.Modem )
    {
    case MODEM_FSK:
        break;
    case MODEM_LORA:
        {
            if( SX1272.Settings.LoRa.TxIqInverted == true )
            {
                SX1272Write( REG_LR_INVERTIQ, ( ( SX1272Read( REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_ON ) );
                SX1272Write( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON );
            }
            else
            {
                SX1272Write( REG_LR_INVERTIQ, ( ( SX1272Read( REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_OFF ) );
                SX1272Write( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_OFF );
            }

            SX1272.Settings.LoRaPacketHandler.Size = size;

            // Initializes the payload size
            SX1272Write( REG_LR_PAYLOADLENGTH, size );

            // Full buffer used for Tx
            SX1272Write( REG_LR_FIFOTXBASEADDR, 0 );
            SX1272Write( REG_LR_FIFOADDRPTR, 0 );

            // FIFO operations can not take place in Sleep mode
            if( ( SX1272Read( REG_OPMODE ) & ~RF_OPMODE_MASK ) == RF_OPMODE_SLEEP )
            {
                SX1272SetStby( );
                DelayMs( 1 );
            }
            // Write payload buffer
            SX1272WriteFifo( buffer, size );
            txTimeout = SX1272.Settings.LoRa.TxTimeout;
        }
        break;
    }

    SX1272SetTx( txTimeout );
}

IRAM_ATTR void SX1272SetSleep( void )
{
    TimerStop( &RxTimeoutTimer );
    TimerStop( &TxTimeoutTimer );

    SX1272SetOpMode( RF_OPMODE_SLEEP );
    SX1272.Settings.State = RF_IDLE;
}

void SX1272SetStby( void )
{
    TimerStop( &RxTimeoutTimer );
    TimerStop( &TxTimeoutTimer );

    SX1272SetOpMode( RF_OPMODE_STANDBY );
    SX1272.Settings.State = RF_IDLE;
}

IRAM_ATTR void SX1272SetRx( uint32_t timeout )
{
    bool rxContinuous = false;

    switch( SX1272.Settings.Modem )
    {
    case MODEM_FSK:
        break;
    case MODEM_LORA:
        {
            if( SX1272.Settings.LoRa.RxIqInverted == true )
            {
                SX1272Write( REG_LR_INVERTIQ, ( ( SX1272Read( REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_ON | RFLR_INVERTIQ_TX_OFF ) );
                SX1272Write( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON );
            }
            else
            {
                SX1272Write( REG_LR_INVERTIQ, ( ( SX1272Read( REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_OFF ) );
                SX1272Write( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_OFF );
            }

            rxContinuous = SX1272.Settings.LoRa.RxContinuous;

            if( SX1272.Settings.LoRa.FreqHopOn == true )
            {
                SX1272Write( REG_LR_IRQFLAGSMASK, //RFLR_IRQFLAGS_RXTIMEOUT |
                                                  //RFLR_IRQFLAGS_RXDONE |
                                                  //RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                                  RFLR_IRQFLAGS_VALIDHEADER |
                                                  RFLR_IRQFLAGS_TXDONE |
                                                  RFLR_IRQFLAGS_CADDONE |
                                                  //RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                                  RFLR_IRQFLAGS_CADDETECTED );

                // DIO0=RxDone, DIO2=FhssChangeChannel
                SX1272Write( REG_DIOMAPPING1, ( SX1272Read( REG_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO0_MASK & RFLR_DIOMAPPING1_DIO2_MASK  ) | RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO2_00 );
            }
            else
            {
                SX1272Write( REG_LR_IRQFLAGSMASK, //RFLR_IRQFLAGS_RXTIMEOUT |
                                                  //RFLR_IRQFLAGS_RXDONE |
                                                  //RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                                  RFLR_IRQFLAGS_VALIDHEADER |
                                                  RFLR_IRQFLAGS_TXDONE |
                                                  RFLR_IRQFLAGS_CADDONE |
                                                  RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                                  RFLR_IRQFLAGS_CADDETECTED );

                // DIO0=RxDone
                SX1272Write( REG_DIOMAPPING1, ( SX1272Read( REG_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO0_MASK ) | RFLR_DIOMAPPING1_DIO0_00 );
            }
            SX1272Write( REG_LR_FIFORXBASEADDR, 0 );
            SX1272Write( REG_LR_FIFOADDRPTR, 0 );
        }
        break;
    }

    memset( RxTxBuffer, 0, ( size_t )RX_BUFFER_SIZE );

    SX1272.Settings.State = RF_RX_RUNNING;
    if( timeout != 0 )
    {
        TimerSetValue( &RxTimeoutTimer, timeout );
        TimerStart( &RxTimeoutTimer );
    }

    if( SX1272.Settings.Modem == MODEM_LORA )
    {
        if( rxContinuous == true )
        {
            SX1272SetOpMode( RFLR_OPMODE_RECEIVER );
        }
        else
        {
            SX1272SetOpMode( RFLR_OPMODE_RECEIVER_SINGLE );
        }
    }
}

void SX1272SetTx( uint32_t timeout )
{
    TimerSetValue( &TxTimeoutTimer, timeout );

    switch( SX1272.Settings.Modem )
    {
    case MODEM_FSK:
        break;
    case MODEM_LORA:
        {
            if( SX1272.Settings.LoRa.FreqHopOn == true )
            {
                SX1272Write( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                                                  RFLR_IRQFLAGS_RXDONE |
                                                  RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                                  RFLR_IRQFLAGS_VALIDHEADER |
                                                  //RFLR_IRQFLAGS_TXDONE |
                                                  RFLR_IRQFLAGS_CADDONE |
                                                  //RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                                  RFLR_IRQFLAGS_CADDETECTED );

                // DIO0=TxDone, DIO2=FhssChangeChannel
                SX1272Write( REG_DIOMAPPING1, ( SX1272Read( REG_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO0_MASK & RFLR_DIOMAPPING1_DIO2_MASK ) | RFLR_DIOMAPPING1_DIO0_01 | RFLR_DIOMAPPING1_DIO2_00 );
            }
            else
            {
                SX1272Write( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                                                  RFLR_IRQFLAGS_RXDONE |
                                                  RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                                  RFLR_IRQFLAGS_VALIDHEADER |
                                                  //RFLR_IRQFLAGS_TXDONE |
                                                  RFLR_IRQFLAGS_CADDONE |
                                                  RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                                  RFLR_IRQFLAGS_CADDETECTED );

                // DIO0=TxDone
                SX1272Write( REG_DIOMAPPING1, ( SX1272Read( REG_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO0_MASK ) | RFLR_DIOMAPPING1_DIO0_01 );
            }
        }
        break;
    }

    SX1272.Settings.State = RF_TX_RUNNING;
    TimerStart( &TxTimeoutTimer );
    SX1272SetOpMode( RF_OPMODE_TRANSMITTER );
}

void SX1272StartCad( void )
{
    switch( SX1272.Settings.Modem )
    {
    case MODEM_FSK:
        break;
    case MODEM_LORA:
        {
            SX1272Write( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                                        RFLR_IRQFLAGS_RXDONE |
                                        RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                        RFLR_IRQFLAGS_VALIDHEADER |
                                        RFLR_IRQFLAGS_TXDONE |
                                        //RFLR_IRQFLAGS_CADDONE |
                                        RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL // |
                                        //RFLR_IRQFLAGS_CADDETECTED
                                        );

            // DIO3=CADDone
            SX1272Write( REG_DIOMAPPING1, ( SX1272Read( REG_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO3_MASK ) | RFLR_DIOMAPPING1_DIO3_00 );

            SX1272.Settings.State = RF_CAD;
            SX1272SetOpMode( RFLR_OPMODE_CAD );
        }
        break;
    default:
        break;
    }
}

void SX1272SetTxContinuousWave( uint32_t freq, int8_t power, uint16_t time )
{
    uint32_t timeout = ( uint32_t )( time * 1000 );

    SX1272SetChannel( freq );

    SX1272SetTxConfig( MODEM_FSK, power, 0, 0, 4800, 0, 5, false, false, 0, 0, 0, timeout );

    SX1272Write( REG_PACKETCONFIG2, ( SX1272Read( REG_PACKETCONFIG2 ) & RF_PACKETCONFIG2_DATAMODE_MASK ) );
    // Disable radio interrupts
    SX1272Write( REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_11 | RF_DIOMAPPING1_DIO1_11 );
    SX1272Write( REG_DIOMAPPING2, RF_DIOMAPPING2_DIO4_10 | RF_DIOMAPPING2_DIO5_10 );

    TimerSetValue( &TxTimeoutTimer, timeout );

    SX1272.Settings.State = RF_TX_RUNNING;
    TimerStart( &TxTimeoutTimer );
    SX1272SetOpMode( RF_OPMODE_TRANSMITTER );
}

int16_t SX1272ReadRssi( RadioModems_t modem )
{
    int16_t rssi = -1;

    switch( modem )
    {
    case MODEM_FSK:
        break;
    case MODEM_LORA:
        rssi = RSSI_OFFSET + SX1272Read( REG_LR_RSSIVALUE );
        break;
    default:
        break;
    }
    return rssi;
}

void SX1272Reset( void )
{
    if (micropy_lpwan_use_reset_pin) {
        // Set RESET pin to 1
        GpioInit( &SX1272.Reset, RADIO_RESET, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1 );

        // Wait 2 ms
        DelayMs( 2 );

        // Configure RESET as input
        GpioInit( &SX1272.Reset, RADIO_RESET, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1 );

        // Wait 6 ms
        DelayMs( 6 );
    } else {
        DelayMs( 2 );
    }
}

IRAM_ATTR void SX1272SetOpMode( uint8_t opMode )
{
    SX1272Write( REG_OPMODE, ( SX1272Read( REG_OPMODE ) & RF_OPMODE_MASK ) | opMode );
}

IRAM_ATTR void SX1272SetModem( RadioModems_t modem )
{
    if( ( SX1272Read( REG_OPMODE ) & RFLR_OPMODE_LONGRANGEMODE_ON ) != 0 )
    {
        SX1272.Settings.Modem = MODEM_LORA;
    }
    else
    {
        SX1272.Settings.Modem = MODEM_FSK;
    }

    if( SX1272.Settings.Modem == modem )
    {
        return;
    }

    SX1272.Settings.Modem = modem;
    switch( SX1272.Settings.Modem )
    {
    default:
    case MODEM_FSK:
        SX1272SetSleep( );
        SX1272Write( REG_OPMODE, ( SX1272Read( REG_OPMODE ) & RFLR_OPMODE_LONGRANGEMODE_MASK ) | RFLR_OPMODE_LONGRANGEMODE_OFF );

        SX1272Write( REG_DIOMAPPING1, 0x00 );
        SX1272Write( REG_DIOMAPPING2, 0x30 ); // DIO5=ModeReady
        break;
    case MODEM_LORA:
        SX1272SetSleep( );
        SX1272Write( REG_OPMODE, ( SX1272Read( REG_OPMODE ) & RFLR_OPMODE_LONGRANGEMODE_MASK ) | RFLR_OPMODE_LONGRANGEMODE_ON );

        SX1272Write( REG_DIOMAPPING1, 0x00 );
        SX1272Write( REG_DIOMAPPING2, 0x00 );
        break;
    }
}

IRAM_ATTR void SX1272Write( uint8_t addr, uint8_t data )
{
    SX1272WriteBuffer( addr, &data, 1 );
}

IRAM_ATTR uint8_t SX1272Read( uint8_t addr )
{
    uint8_t data;
    SX1272ReadBuffer( addr, &data, 1 );
    return data;
}

IRAM_ATTR void SX1272WriteBuffer( uint8_t addr, uint8_t *buffer, uint8_t size )
{
    uint8_t i;

    //NSS = 0;
    GpioWrite( &SX1272.Spi.Nss, 0 );

    SpiInOut( &SX1272.Spi, addr | 0x80 );
    for( i = 0; i < size; i++ )
    {
        SpiInOut( &SX1272.Spi, buffer[i] );
    }

    //NSS = 1;
    GpioWrite( &SX1272.Spi.Nss, 1 );
}

IRAM_ATTR void SX1272ReadBuffer( uint8_t addr, uint8_t *buffer, uint8_t size )
{
    uint8_t i;

    //NSS = 0;
    GpioWrite( &SX1272.Spi.Nss, 0 );

    SpiInOut( &SX1272.Spi, addr & 0x7F );

    for( i = 0; i < size; i++ )
    {
        buffer[i] = SpiInOut( &SX1272.Spi, 0 );
    }

    //NSS = 1;
    GpioWrite( &SX1272.Spi.Nss, 1 );
}

IRAM_ATTR void SX1272WriteFifo( uint8_t *buffer, uint8_t size )
{
    SX1272WriteBuffer( 0, buffer, size );
}

IRAM_ATTR void SX1272ReadFifo( uint8_t *buffer, uint8_t size )
{
    SX1272ReadBuffer( 0, buffer, size );
}

IRAM_ATTR void SX1272SetMaxPayloadLength( RadioModems_t modem, uint8_t max )
{
    SX1272SetModem( modem );

    switch( modem )
    {
    case MODEM_FSK:
        break;
    case MODEM_LORA:
        SX1272Write( REG_LR_PAYLOADMAXLENGTH, max );
        break;
    }
}

void SX1272SetPublicNetwork( bool enable )
{
    SX1272SetModem( MODEM_LORA );
    SX1272.Settings.LoRa.PublicNetwork = enable;
    if( enable == true )
    {
        // Change LoRa modem SyncWord
        SX1272Write( REG_LR_SYNCWORD, LORA_MAC_PUBLIC_SYNCWORD );
    }
    else
    {
        // Change LoRa modem SyncWord
        SX1272Write( REG_LR_SYNCWORD, LORA_MAC_PRIVATE_SYNCWORD );
    }
}

void SX1272OnTimeoutIrq( void )
{
    switch( SX1272.Settings.State )
    {
    case RF_RX_RUNNING:
        if( SX1272.Settings.Modem == MODEM_LORA )
        {
            // Clear Irq
            SX1272Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXTIMEOUT );
            SX1272.Settings.State = RF_IDLE;
        }
        if( ( RadioEvents != NULL ) && ( RadioEvents->RxTimeout != NULL ) )
        {
            RadioEvents->RxTimeout( );
        }
        break;
    case RF_TX_RUNNING:
        // Tx timeout shouldn't happen.
        // But it has been observed that when it happens it is a result of a corrupted SPI transfer
        // it depends on the platform design.
        //
        // The workaround is to put the radio in a known state. Thus, we re-initialize it.

        // BEGIN WORKAROUND

        // Reset the radio
        SX1272Reset( );

        // Initialize radio default values
        SX1272SetOpMode( RF_OPMODE_SLEEP );

        for( uint8_t i = 0; i < sizeof( RadioRegsInit ) / sizeof( RadioRegisters_t ); i++ )
        {
            SX1272SetModem( RadioRegsInit[i].Modem );
            SX1272Write( RadioRegsInit[i].Addr, RadioRegsInit[i].Value );
        }
        SX1272SetModem( MODEM_FSK );

        // Restore previous network type setting.
        SX1272SetPublicNetwork( SX1272.Settings.LoRa.PublicNetwork );
        // END WORKAROUND

        SX1272.Settings.State = RF_IDLE;
        if( ( RadioEvents != NULL ) && ( RadioEvents->TxTimeout != NULL ) )
        {
            RadioEvents->TxTimeout( );
        }
        break;
    default:
        break;
    }
}

IRAM_ATTR void SX1272RadioFlagsIrq (void) {
    if (SX1272.irqFlags & RADIO_IRQ_FLAG_RX_TIMEOUT) {
        SX1272.irqFlags &= ~RADIO_IRQ_FLAG_RX_TIMEOUT;
        if( ( RadioEvents != NULL ) && ( RadioEvents->RxTimeout != NULL ) )
        {
            RadioEvents->RxTimeout( );
        }
    }
    if (SX1272.irqFlags & RADIO_IRQ_FLAG_RX_DONE) {
        SX1272.irqFlags &= ~RADIO_IRQ_FLAG_RX_DONE;
        if( ( RadioEvents != NULL ) && ( RadioEvents->RxDone != NULL ) )
        {
            RadioEvents->RxDone( RxTxBuffer, SX1272.Settings.LoRaPacketHandler.TimeStamp, SX1272.Settings.LoRaPacketHandler.Size,
                                 SX1272.Settings.LoRaPacketHandler.RssiValue, SX1272.Settings.LoRaPacketHandler.SnrValue,
                                 SX1272.Settings.LoRa.Datarate );
        }
    }
    if (SX1272.irqFlags & RADIO_IRQ_FLAG_RX_ERROR) {
        SX1272.irqFlags &= ~RADIO_IRQ_FLAG_RX_ERROR;
        if( ( RadioEvents != NULL ) && ( RadioEvents->RxError != NULL ) )
        {
            RadioEvents->RxError( );
        }
    }
}

static IRAM_ATTR void SX1272OnDioIrq (void) {
    if (SX1272.Settings.State > RF_IDLE) {
        // read the the irq flags registers
        uint8_t volatile irqflags1;
        switch (SX1272.Settings.Modem) {
        case MODEM_FSK:
            break;
        case MODEM_LORA:
            irqflags1 = SX1272Read(REG_LR_IRQFLAGS);
            if ((irqflags1 & RFLR_IRQFLAGS_RXDONE) || (irqflags1 & RFLR_IRQFLAGS_TXDONE)) {
                SX1272OnDio0Irq();
            }
            if (irqflags1 & RFLR_IRQFLAGS_RXTIMEOUT) {
                SX1272OnDio1Irq();
            }
            if (SX1272.Settings.LoRa.FreqHopOn == true && (irqflags1 & RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL)) {
                SX1272OnDio2Irq();
            }
            if ((irqflags1 & RFLR_IRQFLAGS_CADDETECTED) || (irqflags1 & RFLR_IRQFLAGS_CADDONE)) {
                SX1272OnDio3Irq();
            }
            break;
        default:
            break;
        }
    }
}

IRAM_ATTR void SX1272OnDio0Irq( void )
{
    volatile uint8_t irqFlags = 0;

    switch( SX1272.Settings.State )
    {
        case RF_RX_RUNNING:
            // RxDone interrupt
            switch( SX1272.Settings.Modem )
            {
            case MODEM_FSK:
                break;
            case MODEM_LORA:
                {
                    int8_t snr = 0;

                    // Store the packet timestamp
                    SX1272.Settings.LoRaPacketHandler.TimeStamp = mp_hal_ticks_us_non_blocking();

                    // Clear Irq
                    SX1272Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXDONE );

                    irqFlags = SX1272Read( REG_LR_IRQFLAGS );
                    if( ( irqFlags & RFLR_IRQFLAGS_PAYLOADCRCERROR_MASK ) == RFLR_IRQFLAGS_PAYLOADCRCERROR )
                    {
                        // Clear Irq
                        SX1272Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_PAYLOADCRCERROR );

                        if( SX1272.Settings.LoRa.RxContinuous == false )
                        {
                            SX1272.Settings.State = RF_IDLE;
                        }
                        TimerStop( &RxTimeoutTimer );

                        // set the flag and trigger the timer to call the handler as soon as possible
                        SX1272.irqFlags |= RADIO_IRQ_FLAG_RX_ERROR;
                        TimerStart(&RadioIrqFlagsTimer);
                        break;
                    }

                    SX1272.Settings.LoRaPacketHandler.SnrValue = SX1272Read( REG_LR_PKTSNRVALUE );
                    if( SX1272.Settings.LoRaPacketHandler.SnrValue & 0x80 ) // The SNR sign bit is 1
                    {
                        // Invert and divide by 4
                        // neg A = ~A + 1
                        snr = ( ( ~SX1272.Settings.LoRaPacketHandler.SnrValue + 1 ) & 0xFF ) >> 2;
                        snr = -snr;
                    }
                    else
                    {
                        // Divide by 4
                        snr = ( SX1272.Settings.LoRaPacketHandler.SnrValue & 0xFF ) >> 2;
                    }

                    int16_t rssi = SX1272Read( REG_LR_PKTRSSIVALUE );
                    if( snr < 0 )
                    {
                        SX1272.Settings.LoRaPacketHandler.RssiValue = RSSI_OFFSET + rssi + ( rssi >> 4 ) +
                                                                      snr;
                    }
                    else
                    {
                        SX1272.Settings.LoRaPacketHandler.RssiValue = RSSI_OFFSET + rssi + ( rssi >> 4 );
                    }

                    SX1272.Settings.LoRaPacketHandler.Size = SX1272Read( REG_LR_RXNBBYTES );
                    SX1272Write( REG_LR_FIFOADDRPTR, SX1272Read( REG_LR_FIFORXCURRENTADDR ) );
                    SX1272ReadFifo( RxTxBuffer, SX1272.Settings.LoRaPacketHandler.Size );

                    if( SX1272.Settings.LoRa.RxContinuous == false )
                    {
                        SX1272.Settings.State = RF_IDLE;
                    }
                    TimerStop( &RxTimeoutTimer );

                    // set the flag and trigger the timer to call the handler as soon as possible
                    SX1272.irqFlags |= RADIO_IRQ_FLAG_RX_DONE;
                    TimerStart(&RadioIrqFlagsTimer);
                }
                break;
            default:
                break;
            }
            break;
        case RF_TX_RUNNING:
            TimerStop( &TxTimeoutTimer );
            // TxDone interrupt
            switch( SX1272.Settings.Modem )
            {
            case MODEM_LORA:
                // Clear Irq
                SX1272Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_TXDONE );
                // Intentional fall through
            case MODEM_FSK:
            default:
                SX1272.Settings.State = RF_IDLE;
                if( ( RadioEvents != NULL ) && ( RadioEvents->TxDone != NULL ) )
                {
                    RadioEvents->TxDone( );
                }
                break;
            }
            break;
        default:
            break;
    }
}

IRAM_ATTR void SX1272OnDio1Irq( void )
{
    switch( SX1272.Settings.State )
    {
        case RF_RX_RUNNING:
            switch( SX1272.Settings.Modem )
            {
            case MODEM_FSK:
                break;
            case MODEM_LORA:
                // Sync time out
                TimerStop( &RxTimeoutTimer );
                // Clear Irq
                SX1272Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXTIMEOUT );

                SX1272.Settings.State = RF_IDLE;

                // set the flag and trigger the timer to call the handler as soon as possible
                SX1272.irqFlags |= RADIO_IRQ_FLAG_RX_TIMEOUT;
                TimerStart(&RadioIrqFlagsTimer);
                break;
            default:
                break;
            }
            break;
        case RF_TX_RUNNING:
            switch( SX1272.Settings.Modem )
            {
            case MODEM_FSK:
                break;
            case MODEM_LORA:
                break;
            default:
                break;
            }
            break;
        default:
            break;
    }
}

IRAM_ATTR void SX1272OnDio2Irq( void )
{
    switch( SX1272.Settings.State )
    {
        case RF_RX_RUNNING:
            switch( SX1272.Settings.Modem )
            {
            case MODEM_FSK:
                break;
            case MODEM_LORA:
                if( SX1272.Settings.LoRa.FreqHopOn == true )
                {
                    // Clear Irq
                    SX1272Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL );

                    if( ( RadioEvents != NULL ) && ( RadioEvents->FhssChangeChannel != NULL ) )
                    {
                        RadioEvents->FhssChangeChannel( ( SX1272Read( REG_LR_HOPCHANNEL ) & RFLR_HOPCHANNEL_CHANNEL_MASK ) );
                    }
                }
                break;
            default:
                break;
            }
            break;
        case RF_TX_RUNNING:
            switch( SX1272.Settings.Modem )
            {
            case MODEM_FSK:
                break;
            case MODEM_LORA:
                if( SX1272.Settings.LoRa.FreqHopOn == true )
                {
                    // Clear Irq
                    SX1272Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL );

                    if( ( RadioEvents != NULL ) && ( RadioEvents->FhssChangeChannel != NULL ) )
                    {
                        RadioEvents->FhssChangeChannel( ( SX1272Read( REG_LR_HOPCHANNEL ) & RFLR_HOPCHANNEL_CHANNEL_MASK ) );
                    }
                }
                break;
            default:
                break;
            }
            break;
        default:
            break;
    }
}

IRAM_ATTR void SX1272OnDio3Irq( void )
{
    switch( SX1272.Settings.Modem )
    {
    case MODEM_FSK:
        break;
    case MODEM_LORA:
        if( ( SX1272Read( REG_LR_IRQFLAGS ) & RFLR_IRQFLAGS_CADDETECTED ) == RFLR_IRQFLAGS_CADDETECTED )
        {
            // Clear Irq
            SX1272Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_CADDETECTED | RFLR_IRQFLAGS_CADDONE );
            if( ( RadioEvents != NULL ) && ( RadioEvents->CadDone != NULL ) )
            {
                RadioEvents->CadDone( true );
            }
        }
        else
        {
            // Clear Irq
            SX1272Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_CADDONE );
            if( ( RadioEvents != NULL ) && ( RadioEvents->CadDone != NULL ) )
            {
                RadioEvents->CadDone( false );
            }
        }
        break;
    default:
        break;
    }
}

IRAM_ATTR void SX1272OnDio4Irq( void )
{
    switch( SX1272.Settings.Modem )
    {
    case MODEM_FSK:
        break;
    case MODEM_LORA:
        break;
    default:
        break;
    }
}

// not used
void SX1272OnDio5Irq( void )
{
    switch( SX1272.Settings.Modem )
    {
    case MODEM_FSK:
        break;
    case MODEM_LORA:
        break;
    default:
        break;
    }
}

#define PHYSEC
#ifdef PHYSEC


/*!
 * \brief Optimized packet exchange and rssi extracting
 *
 * Transceiver is never sent to sleep while performing measurements
 * to avoid transceiver wake up overload
 *
 * \returns structure containing all the rssi measurements, or NULL if allocation failed,
 *          or (void*) -1 if something went wrong during measure
 */
PHYSEC_RssiMsrmts *
IRAM_ATTR SX1272InitiateRssiMeasure(PHYSEC_Sync *sync, const uint16_t nb_measures)
{
/*
    //int16_t error_margin = 10 + 20; // coherence time tolerance + hardware tolerance (completely random)

#ifndef PHYSEC_OPTIMIZED
    // check if LoRa mode
    uint8_t mode = SX1272Read(REG_LR_OPMODE);
    // if highest bit (7th) of RegOpMode is 1 then we are running lora mode on transceiver, otherwise its FSK
    if ( !(mode & 0x80) ) {
        printf("Not in LoRa mode. Cannot performs RSSI measurements\n");
        return (void*) -1;
    }

    printf("mode = %d\n", mode & 0x07);

    // activate RX & TX IRQ masks
    uint8_t masks = SX1272Read(REG_LR_IRQFLAGSMASK);
            printf("IRQ masks = %u %u %u %u %u %u %u %u\n",
            (unsigned int)(masks >> 7) & 0x1,
            (unsigned int)(masks >> 6) & 0x1,
            (unsigned int)(masks >> 5) & 0x1,
            (unsigned int)(masks >> 4) & 0x1,
            (unsigned int)(masks >> 3) & 0x1,
            (unsigned int)(masks >> 2) & 0x1,
            (unsigned int)(masks >> 1) & 0x1,
            (unsigned int)(masks) & 0x1
            );
    SX1272Write(REG_LR_IRQFLAGSMASK, (masks & (~(0x08|0x40))));
     masks = SX1272Read(REG_LR_IRQFLAGSMASK);
            printf("IRQ masks = %u %u %u %u %u %u %u %u\n",
            (unsigned int)(masks >> 7) & 0x1,
            (unsigned int)(masks >> 6) & 0x1,
            (unsigned int)(masks >> 5) & 0x1,
            (unsigned int)(masks >> 4) & 0x1,
            (unsigned int)(masks >> 3) & 0x1,
            (unsigned int)(masks >> 2) & 0x1,
            (unsigned int)(masks >> 1) & 0x1,
            (unsigned int)(masks) & 0x1
            );
#endif
*/

    PHYSEC_RssiMsrmts *m = malloc(sizeof(PHYSEC_RssiMsrmts));
    if (!m)
        return NULL;

    m->nb_msrmts = nb_measures;
    m->rssi_msrmts = malloc(sizeof(int8_t) * nb_measures);
    if (!(m->rssi_msrmts))
    {
        free(m);
        return NULL;
    }
/*
    SX1272SetChannel(PHYSEC_KEYGEN_FREQUENCY);
    printf("Starting KeyGen on freq %d\n", SX1272GetChannel());

    uint8_t buf[PHYSEC_DEV_ID_LEN+1];
    for (uint16_t i=0; i<nb_measures; i++)
    {
        printf("Craft and send Probe Request\n");
        uint8_t irq_flags = 0;
        // prepare packet
        memcpy(buf, sync->rmt_dev_id, PHYSEC_DEV_ID_LEN);
        *(int8_t*) (buf + PHYSEC_DEV_ID_LEN) = sync->cnt;

        SX1272SetOpMode( RFLR_OPMODE_TRANSMITTER );
        // schedule packet
        SX1272Send(buf, sizeof(buf));

        printf("Probe sent ! Waiting for response...\n");

        SX1272SetOpMode( RFLR_OPMODE_RECEIVER );
        SX1272SetRx(PHYSEC_PROBE_TIMEOUT);
        uint8_t on_rx_wait = 1;
        int8_t rssi = 0;
        uint32_t start = time(NULL);
        // wait for our probe response (timeout after 1 packet received)
        while ( on_rx_wait && PHYSEC_PROBE_TIMEOUT > time(NULL)-start )
        {
            while ( (irq_flags = SX1272Read(REG_LR_IRQFLAGS)) & 0x10 );
                usleep(5000);

            printf("Header received ! Retrieving rssi...\n");
            rssi = SX1272Read( REG_LR_RSSIVALUE );

            while ( !(( irq_flags = SX1272Read(REG_LR_IRQFLAGS)) & 0x40) )
                usleep(5000);  // 0.005 seconds

            SX1272Write(REG_LR_IRQFLAGS, irq_flags & (~0x40) );
            printf("Packet received. Is Probe Response ?"); 

            // check if packet match
            uint8_t pkt_addr = SX1272Read(REG_LR_FIFORXCURRENTADDR);
            uint8_t pkt_size = SX1272Read(REG_LR_RXNBBYTES);
            uint8_t pkt[pkt_size];
            
            memset(pkt, 0, sizeof(pkt));
            SX1272ReadBuffer(pkt_addr, pkt, pkt_size);

            if (memcmp(pkt, sync->dev_id, PHYSEC_DEV_ID_LEN) == 0 && pkt[PHYSEC_DEV_ID_LEN] == (sync->cnt+1))
            {
                printf("yes\n");
                on_rx_wait = 0;
            }
            else 
                printf("no\n");

        }




        uint8_t snr = SX1272Read( REG_LR_PKTSNRVALUE );
        if( snr & 0x80 ) // The SNR sign bit is 1
        {
            snr = ( ( ~snr + 1 ) & 0xFF ) >> 2;
            snr = -snr;
        }
        else
        {
            // Divide by 4
            snr = ( snr & 0xFF ) >> 2;
        }
        if( snr < 0 )
        {
            m->rssi_msrmts[i] = RSSI_OFFSET + rssi + ( rssi >> 4 ) +
                                                            snr;
        }
        else
        {
            m->rssi_msrmts[i] = RSSI_OFFSET + rssi + ( rssi >> 4 );
        }
        printf("Rssi measured from Probe Response : %d\n", m-> rssi_msrmts[i]);

        sync->cnt++;
    }
*/
    return m;
}
PHYSEC_RssiMsrmts *
IRAM_ATTR SX1272WaitRssiMeasure(PHYSEC_Sync *sync, const uint16_t nb_measures )
{
#ifndef PHYSEC_OPTIMIZED
    // check if LoRa mode
    uint8_t mode = SX1272Read(REG_LR_OPMODE);
    // if highest bit (7th) of RegOpMode is 1 then we are running lora mode on transceiver, otherwise its FSK
    if ( (mode & 0x80) ^ 0x1 ) {
        printf("Not in LoRa mode. Cannot performs RSSI measurements\n");
        return (void*) -1;
    }
#endif

    //int16_t error_margin = 10 + 20; // coherence time tolerance + hardware tolerance (completely random)

    PHYSEC_RssiMsrmts *m = malloc(sizeof(PHYSEC_RssiMsrmts));
    if (!m)
        return NULL;

    m->nb_msrmts = nb_measures;
    m->rssi_msrmts = malloc(sizeof(int8_t) * nb_measures);
    if (!(m->rssi_msrmts))
    {
        free(m);
        return NULL;
    }

    for (uint16_t i=0; i<nb_measures; i++)
    {
        // wait for sync packet

        // extract rssi

        // send answer packet

        // wait for sending

    }

    return m;
}

// Reciprocity enhancement

// --- Filtering

PHYSEC_RssiMsrmts PHYSEC_golay_filter(PHYSEC_RssiMsrmts rssi_msermts){

    int8_t coef[] = {-3, 12, 17, 12, -3};
    float normalization;

    int rssi_tmp; // it is not normalisze so int8_t is not a valid type.

    PHYSEC_RssiMsrmts rssi_msermts_filterd;
    rssi_msermts_filterd.nb_msrmts = rssi_msermts.nb_msrmts;
    rssi_msermts_filterd.rssi_msrmts = malloc(rssi_msermts.nb_msrmts * sizeof(int8_t));

    for(int i_rssi = 0; i_rssi < rssi_msermts.nb_msrmts; i_rssi++){

        normalization = 0;
        rssi_tmp = 0;

        for(int i_coef = -2; i_coef < 3; i_coef++){
            if(i_rssi+i_coef >= 0 && i_rssi+i_coef < rssi_msermts.nb_msrmts){
                rssi_tmp += coef[i_coef+2] * rssi_msermts.rssi_msrmts[i_rssi+i_coef];
                normalization += coef[i_coef+2];
            }
        }

        rssi_msermts_filterd.rssi_msrmts[i_rssi] = (int8_t) ((float)(rssi_tmp)/normalization);

    }

    return rssi_msermts_filterd;

}

// --- Interpolation

PHYSEC_RssiMsrmts PHYSEC_interpolation(PHYSEC_RssiMsrmts rssi_msermts){

    PHYSEC_RssiMsrmts rssi_msermts_estimation;
    rssi_msermts_estimation.nb_msrmts = rssi_msermts.nb_msrmts;
    rssi_msermts_estimation.rssi_msrmts = malloc(rssi_msermts.nb_msrmts * sizeof(int8_t));
    rssi_msermts_estimation.rssi_msrmts_delay = 0;

    int8_t delta_rssi;
    float rssi_err;

    if(rssi_msermts.nb_msrmts > 0 && rssi_msermts.rssi_msrmts_delay > 0){

        rssi_msermts_estimation.rssi_msrmts[0] = rssi_msermts.rssi_msrmts[0];

        for(int i = 1; i < rssi_msermts.nb_msrmts; i++){
            delta_rssi = rssi_msermts.rssi_msrmts[i] - rssi_msermts.rssi_msrmts[i-1];
            rssi_err = (rssi_msermts.rssi_msrmts_delay * (float)(delta_rssi));
            rssi_msermts_estimation.rssi_msrmts[i] = rssi_msermts.rssi_msrmts[i] - rssi_err;
        }

    }

    return rssi_msermts_estimation;

}

// Key generation

/*
    128 bits = 16 bytes = a 16-chat-table.
    For now, if this value is changed, the code will not going to adapt.
*/
#define PHYSEC_KEY_LEN 128 // bits.


// -- Quntification

#define PHYSEC_QUNTIFICATION_WINDOW_LEN 10

void PHYSEC_quntification_sort_rssi_window(int8_t *rssi_window, int8_t rssi_window_size){
    
    if(rssi_window_size <= 1){
        return;
    }

    int8_t *tab1 = rssi_window;
    int8_t tab1_size = (int8_t) (((float)(rssi_window_size))/2.0);
    int8_t *tab2 = rssi_window+tab1_size;
    int8_t tab2_size = rssi_window - tab1_size;

    int8_t tmp_tab[rssi_window_size];

    PHYSEC_quntification_sort_rssi_window(rssi_window, half_size);
    PHYSEC_quntification_sort_rssi_window(rssi_window+half_size, rssi_window_size-half_size);

    int i1=0, i2=0, i=0;
    while(i1 < tab1_size){
        while(i2 < tab2_size){
            if(tab1[i1]<tab2[i2]){
               tmp_tab[i++] =  tab1[i1++];
            }else{
                tmp_tab[i++] =  tab2[i2++];
            }
        }
    }

    memcpy(rssi_window, tmp_tab);
}

void PHYSEC_quntification_compute_bin(int8_t *sorted_rssi_window, int8_t *bin_len, int8_t *q_0, int8_t *q_m){
    
    int8_t bin_tmp;

    *bin_len = abs(sorted_rssi_window[1] - sorted_rssi_window[0])
    for(int i = 2 ; i < PHYSEC_QUNTIFICATION_WINDOW_LEN; i++){
        bin_tmp = abs(sorted_rssi_window[i] - sorted_rssi_window[i-1]);
        if(bin_tmp < *bin_len){
            *bin_len = bin_tmp;
        }
    }

    *q_0 = sorted_rssi_window[0];
    *q_m = sorted_rssi_window[PHYSEC_QUNTIFICATION_WINDOW_LEN-1];
}

char *PHYSEC_quntification_compute_hist(
    int8_t *rssi_window,
    int8_t *bin_len, int8_t *q_0, int8_t *q_m,
    uint16_t *hist_size
){

    int8_t *rssi_window_sorted[PHYSEC_QUNTIFICATION_WINDOW_LEN];
    memcpy(rssi_window_sorted, rssi_window);

    PHYSEC_quntification_sort_rssi_window(rssi_window_sorted, PHYSEC_QUNTIFICATION_WINDOW_LEN);
    
    PHYSEC_quntification_compute_bin(sorted_rssi_window, bin_len, q_0, q_m);

    *hist_size = (uint16_t)(((float)(q_m -q_0))/((float)(bin_len))) +1;
    char *hist = malloc(*hist_size * sizeof(char));

    int rssi_i = 0;
    for(int i = 0; i < *hist_size; i++){
        if(sorted_rssi_window[rssi_i] >= *q_0 +i*(*bin_len)){
            hist[i] = 1;
            rssi_i++;
        }else{
            hist[i] = 0;
        }
    }

    return hist;

}



int PHYSEC_quntification_compute_level_nbr(int8_t *rssi_window){

    double entropy = 0;

    for(int i = 0; i < PHYSEC_QUNTIFICATION_WINDOW_LEN; i++){
        entropy += rssi_window[i] * log2(rssi_window[i]); 
    }
}

/*
    Return value :
        0   : Success.
        -1  : The number of rssi measurement is not enough to generate the key.
*/
int PHYSEC_quntification(PHYSEC_RssiMsrmts rssi_msermts, char *key_output){

    uint8_t nbr_of_bit_generated = 0;
    uint8_t nbr_of_processed_windows = 0;
    int8_t *rssi_window;

    while(nbr_of_bit_generated < PHYSEC_KEY_LEN){

        if(rssi_msermts.nb_msrmts - nbr_of_processed_windows*PHYSEC_QUNTIFICATION_WINDOW_LEN < PHYSEC_QUNTIFICATION_WINDOW_LEN){
            return -1;
        }

        rssi_window = rssi_msermts.rssi_msrmts+nbr_of_processed_windows;



    }

}

#endif // PHYSEC
