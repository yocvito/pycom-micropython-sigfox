/*
 * byteswap.h - minimal shim for xtensa-esp32-elf newlib + libwifi.
 *
 * libwifi/core/misc/byteswap.h does `#include <byteswap.h>` (we patched it
 * from the upstream quoted form, which infinite-included libwifi's own
 * header) and expects __bswap_16 / __bswap_32 to be available. Newlib does
 * not ship <byteswap.h>, so we provide them.
 *
 * Implementation note: we use `static inline` functions rather than macros.
 * On the host (glibc) a real <byteswap.h> exists and declares __bswap_*
 * differently; defining them as macros here would corrupt those declarations
 * when both headers happen to be visible (e.g. via lwIP's <arpa/inet.h>).
 * static-inline functions don't clash. The `__bswap_*` symbols are reserved
 * for the implementation, so collisions across translation units are not a
 * concern.
 */
#ifndef LIBWIFI_PORT_BYTESWAP_H
#define LIBWIFI_PORT_BYTESWAP_H

/* If glibc's <bits/byteswap.h> is already in scope (host builds, e.g. unit
 * tests run on Linux), don't redefine the inline functions — glibc's win. */
#if !defined(_BITS_BYTESWAP_H) && !defined(_BYTESWAP_H)

#include <stdint.h>

#ifndef __bswap_16
static inline uint16_t __bswap_16(uint16_t v) { return __builtin_bswap16(v); }
#endif
#ifndef __bswap_32
static inline uint32_t __bswap_32(uint32_t v) { return __builtin_bswap32(v); }
#endif
#ifndef __bswap_64
static inline uint64_t __bswap_64(uint64_t v) { return __builtin_bswap64(v); }
#endif

#endif /* !_BITS_BYTESWAP_H && !_BYTESWAP_H */

#endif /* LIBWIFI_PORT_BYTESWAP_H */
