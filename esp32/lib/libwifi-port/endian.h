/*
 * endian.h - minimal shim for xtensa-esp32-elf newlib + libwifi.
 *
 * Newlib does not ship a full <endian.h>. libwifi's core/radiotap/platform.h
 * does `#include "endian.h"` when ESP_PLATFORM is defined and expects the
 * usual le16toh / le32toh / htole16 / htole32 names. Provide them here.
 *
 * ESP32 (Xtensa LX6) is little-endian, so host<->LE conversions are no-ops
 * and BE<->host uses __builtin_bswap.
 */
#ifndef LIBWIFI_PORT_ENDIAN_H
#define LIBWIFI_PORT_ENDIAN_H

#include <stdint.h>

#ifndef __BYTE_ORDER__
#define __ORDER_LITTLE_ENDIAN__ 1234
#define __ORDER_BIG_ENDIAN__    4321
#define __BYTE_ORDER__          __ORDER_LITTLE_ENDIAN__
#endif

#ifndef __LITTLE_ENDIAN
#define __LITTLE_ENDIAN __ORDER_LITTLE_ENDIAN__
#endif
#ifndef __BIG_ENDIAN
#define __BIG_ENDIAN    __ORDER_BIG_ENDIAN__
#endif
#ifndef __BYTE_ORDER
#define __BYTE_ORDER    __BYTE_ORDER__
#endif

/* Little-endian host (ESP32): LE conversions are identity. */
#ifndef le16toh
#define le16toh(x) ((uint16_t)(x))
#endif
#ifndef le32toh
#define le32toh(x) ((uint32_t)(x))
#endif
#ifndef le64toh
#define le64toh(x) ((uint64_t)(x))
#endif
#ifndef htole16
#define htole16(x) ((uint16_t)(x))
#endif
#ifndef htole32
#define htole32(x) ((uint32_t)(x))
#endif
#ifndef htole64
#define htole64(x) ((uint64_t)(x))
#endif

/* Big-endian conversions use compiler bswap intrinsics. */
#ifndef be16toh
#define be16toh(x) __builtin_bswap16((uint16_t)(x))
#endif
#ifndef be32toh
#define be32toh(x) __builtin_bswap32((uint32_t)(x))
#endif
#ifndef be64toh
#define be64toh(x) __builtin_bswap64((uint64_t)(x))
#endif
#ifndef htobe16
#define htobe16(x) __builtin_bswap16((uint16_t)(x))
#endif
#ifndef htobe32
#define htobe32(x) __builtin_bswap32((uint32_t)(x))
#endif
#ifndef htobe64
#define htobe64(x) __builtin_bswap64((uint64_t)(x))
#endif

#endif /* LIBWIFI_PORT_ENDIAN_H */
