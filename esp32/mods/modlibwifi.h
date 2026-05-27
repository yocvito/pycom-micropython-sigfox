/*
 * Copyright (c) 2024, Pycom community.
 *
 * This software is licensed under the GNU GPL version 3 or any
 * later version, with permitted additional terms. For more information
 * see the Pycom Licence v1.0 document supplied with this file, or
 * available at https://www.pycom.io/opensource/licensing
 *
 * modlibwifi: MicroPython bindings around libwifi's 802.11 parsing helpers.
 *
 * These functions take a raw 802.11 MAC frame (no radiotap header — ESP32's
 * legacy esp_wifi promiscuous callback delivers the bare MAC frame) and turn
 * it into Python attrtuples. They are called from modwlan.c and exposed as
 * WLAN.parse_packet() / WLAN.parse_frame(buf).
 *
 * Every libwifi struct that owns heap memory (tags, body, radiotap_info) is
 * copied into MicroPython-owned objects and freed before returning, so a
 * caller exception can never leak.
 */

#ifndef MODLIBWIFI_H_
#define MODLIBWIFI_H_

#ifdef MOD_LIBWIFI_ENABLED

#include "py/obj.h"
#include <stdint.h>
#include <stddef.h>

/*
 * Parse a raw 802.11 frame and return an attrtuple describing it.
 *
 * The returned object always has at least these fields:
 *   type, subtype, flags (ds/retry/protect/...), duration,
 *   addr1, addr2, addr3 (as bytes, len 6),
 *   seq (sequence number), frag (fragment number),
 *   body (raw payload after the header, bytes)
 *
 * For management subtypes with a richer libwifi parser (beacon, probe req/
 * resp, assoc req/resp, deauth, disassoc), the attrtuple is extended with
 * subtype-specific fields. See modlibwifi.c for the exact schema.
 *
 * Returns mp_const_none if the frame is unparseable (too short / unknown
 * type). Never raises (caller's responsibility to handle None).
 *
 * data must point to a raw MAC frame; data_len is its length in bytes.
 */
mp_obj_t mod_libwifi_parse_frame(const uint8_t *data, size_t data_len);

#endif /* MOD_LIBWIFI_ENABLED */

#endif /* MODLIBWIFI_H_ */
