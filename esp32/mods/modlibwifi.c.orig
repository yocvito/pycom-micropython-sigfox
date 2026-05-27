/*
 * Copyright (c) 2024, Pycom community.
 *
 * This software is licensed under the GNU GPL version 3 or any
 * later version, with permitted additional terms. For more information
 * see the Pycom Licence v1.0 document supplied with this file, or
 * available at https://www.pycom.io/opensource/licensing
 *
 * modlibwifi: see modlibwifi.h for design notes.
 */

#ifdef MOD_LIBWIFI_ENABLED

#include "modlibwifi.h"

#include "py/mpconfig.h"
#include "py/obj.h"
#include "py/objstr.h"
#include "py/objtuple.h"
#include "py/runtime.h"

#include <stdint.h>
#include <string.h>

#include "libwifi.h"

/* ------------------------------------------------------------------------- */
/* Small helpers.                                                            */
/* ------------------------------------------------------------------------- */

/* Build a bytes object from a 6-byte MAC address. */
static mp_obj_t mac_bytes(const unsigned char mac[6]) {
    return mp_obj_new_bytes((const byte *)mac, 6);
}

/* Build a list of (tag_num, bytes) tuples from libwifi_tagged_parameters. */
static mp_obj_t tags_to_list(const struct libwifi_tagged_parameters *tags) {
    mp_obj_t list = mp_obj_new_list(0, NULL);
    if (tags == NULL || tags->parameters == NULL || tags->length == 0) {
        return list;
    }
    struct libwifi_tag_iterator it = {0};
    if (libwifi_tag_iterator_init(&it, tags->parameters, tags->length) != 0) {
        return list;
    }
    do {
        if (it.tag_header == NULL) {
            break;
        }
        mp_obj_t entry[2] = {
            mp_obj_new_int(it.tag_header->tag_num),
            mp_obj_new_bytes((const byte *)it.tag_data, it.tag_header->tag_len),
        };
        mp_obj_list_append(list, mp_obj_new_tuple(2, entry));
    } while (libwifi_tag_iterator_next(&it) != -1);
    return list;
}

/* Extract a flags dict from a frame_ctrl. Returned as a small attrtuple so
 * Python users get attribute access (frame.flags.retry, etc.) without the
 * overhead of a real dict. */
static mp_obj_t flags_attrtuple(const struct libwifi_frame_ctrl_flags *f) {
    STATIC const qstr fields[] = {
        MP_QSTR_to_ds, MP_QSTR_from_ds, MP_QSTR_more_frags, MP_QSTR_retry,
        MP_QSTR_power_mgmt, MP_QSTR_more_data, MP_QSTR_protect, MP_QSTR_ordered,
    };
    mp_obj_t t[8] = {
        mp_obj_new_bool(f->to_ds),
        mp_obj_new_bool(f->from_ds),
        mp_obj_new_bool(f->more_frags),
        mp_obj_new_bool(f->retry),
        mp_obj_new_bool(f->power_mgmt),
        mp_obj_new_bool(f->more_data),
        mp_obj_new_bool(f->protect),
        mp_obj_new_bool(f->ordered),
    };
    return mp_obj_new_attrtuple(fields, 8, t);
}

/* ------------------------------------------------------------------------- */
/* Pull the per-frame-type header fields out of a libwifi_frame. Returns the */
/* address/seq triple as a base attrtuple. Each path reads only its own union*/
/* member, picked from the frame_control type/subtype and flags.             */
/* ------------------------------------------------------------------------- */

struct base_header_info {
    uint16_t duration;
    const unsigned char *addr1;
    const unsigned char *addr2;
    const unsigned char *addr3;
    unsigned int seq_num;
    unsigned int frag_num;
    int have_addrs; /* 0 if control frame (only addr1 is meaningful, addr2/3 NULL) */
};

static void fill_header_info(const struct libwifi_frame *fi, struct base_header_info *bh) {
    memset(bh, 0, sizeof(*bh));
    switch (fi->frame_control.type) {
    case TYPE_MANAGEMENT:
        if (fi->flags & LIBWIFI_FLAGS_IS_ORDERED) {
            const struct libwifi_mgmt_ordered_frame_header *h = &fi->header.mgmt_ordered;
            bh->duration = h->duration;
            bh->addr1 = h->addr1; bh->addr2 = h->addr2; bh->addr3 = h->addr3;
            bh->seq_num = h->seq_control.sequence_number;
            bh->frag_num = h->seq_control.fragment_number;
        } else {
            const struct libwifi_mgmt_unordered_frame_header *h = &fi->header.mgmt_unordered;
            bh->duration = h->duration;
            bh->addr1 = h->addr1; bh->addr2 = h->addr2; bh->addr3 = h->addr3;
            bh->seq_num = h->seq_control.sequence_number;
            bh->frag_num = h->seq_control.fragment_number;
        }
        bh->have_addrs = 1;
        break;
    case TYPE_DATA:
        if (fi->flags & LIBWIFI_FLAGS_IS_QOS) {
            const struct libwifi_data_qos_frame_header *h = &fi->header.data_qos;
            bh->duration = h->duration;
            bh->addr1 = h->addr1; bh->addr2 = h->addr2; bh->addr3 = h->addr3;
            bh->seq_num = h->seq_control.sequence_number;
            bh->frag_num = h->seq_control.fragment_number;
        } else {
            const struct libwifi_data_frame_header *h = &fi->header.data;
            bh->duration = h->duration;
            bh->addr1 = h->addr1; bh->addr2 = h->addr2; bh->addr3 = h->addr3;
            bh->seq_num = h->seq_control.sequence_number;
            bh->frag_num = h->seq_control.fragment_number;
        }
        bh->have_addrs = 1;
        break;
    case TYPE_CONTROL:
        bh->duration = fi->header.ctrl.duration;
        bh->have_addrs = 0; /* control frame headers only carry the duration */
        break;
    default:
        break;
    }
}

/* ------------------------------------------------------------------------- */
/* Build the generic base attrtuple every parsed frame returns.              */
/* Fields: type, subtype, flags, duration, addr1, addr2, addr3, seq, frag,   */
/*         body, detail                                                      */
/*                                                                           */
/* `detail` is mp_const_none for unsupported subtypes, or a subtype-specific */
/* attrtuple (beacon, probe_req, etc.) when a richer parser succeeded.       */
/* ------------------------------------------------------------------------- */

static const qstr base_fields[] = {
    MP_QSTR_type,
    MP_QSTR_subtype,
    MP_QSTR_flags,
    MP_QSTR_duration,
    MP_QSTR_addr1,
    MP_QSTR_addr2,
    MP_QSTR_addr3,
    MP_QSTR_seq,
    MP_QSTR_frag,
    MP_QSTR_body,
    MP_QSTR_detail,
};
#define BASE_NFIELDS 11

static mp_obj_t build_base(const struct libwifi_frame *fi, mp_obj_t detail) {
    struct base_header_info bh;
    fill_header_info(fi, &bh);

    mp_obj_t t[BASE_NFIELDS];
    t[0] = mp_obj_new_int(fi->frame_control.type);
    t[1] = mp_obj_new_int(fi->frame_control.subtype);
    t[2] = flags_attrtuple(&fi->frame_control.flags);
    t[3] = mp_obj_new_int(bh.duration);
    t[4] = bh.have_addrs && bh.addr1 ? mac_bytes(bh.addr1) : mp_const_none;
    t[5] = bh.have_addrs && bh.addr2 ? mac_bytes(bh.addr2) : mp_const_none;
    t[6] = bh.have_addrs && bh.addr3 ? mac_bytes(bh.addr3) : mp_const_none;
    t[7] = mp_obj_new_int(bh.seq_num);
    t[8] = mp_obj_new_int(bh.frag_num);
    if (fi->body != NULL && fi->len > fi->header_len) {
        t[9] = mp_obj_new_bytes((const byte *)fi->body, fi->len - fi->header_len);
    } else {
        t[9] = mp_const_none;
    }
    t[10] = detail;

    return mp_obj_new_attrtuple(base_fields, BASE_NFIELDS, t);
}

/* ------------------------------------------------------------------------- */
/* Detail builders (one per supported subtype).                              */
/* ------------------------------------------------------------------------- */

/* Build a BSS detail attrtuple (used by beacon / probe_resp / (re)assoc_resp).
 * The security strings are written into a single LIBWIFI_SECURITY_BUF_LEN
 * buffer, which we reuse across libwifi_get_*_ciphers calls. */
static mp_obj_t bss_detail(struct libwifi_bss *bss) {
    STATIC const qstr fields[] = {
        MP_QSTR_ssid, MP_QSTR_hidden, MP_QSTR_bssid, MP_QSTR_transmitter,
        MP_QSTR_receiver, MP_QSTR_channel, MP_QSTR_wps, MP_QSTR_encryption_info,
        MP_QSTR_security, MP_QSTR_group_ciphers, MP_QSTR_pairwise_ciphers,
        MP_QSTR_akm_suites, MP_QSTR_tags,
    };
    char sec[LIBWIFI_SECURITY_BUF_LEN];

    mp_obj_t t[13];
    /* libwifi guarantees ssid is NUL-terminated within 33 bytes. */
    t[0] = bss->hidden ? mp_const_none : mp_obj_new_str(bss->ssid, strlen(bss->ssid));
    t[1] = mp_obj_new_bool(bss->hidden);
    t[2] = mac_bytes(bss->bssid);
    t[3] = mac_bytes(bss->transmitter);
    t[4] = mac_bytes(bss->receiver);
    t[5] = mp_obj_new_int(bss->channel);
    t[6] = mp_obj_new_bool(bss->wps);
    t[7] = mp_obj_new_int_from_uint(bss->encryption_info);

    sec[0] = '\0';
    libwifi_get_security_type(bss, sec);
    t[8] = mp_obj_new_str(sec, strlen(sec));

    sec[0] = '\0';
    libwifi_get_group_ciphers(bss, sec);
    t[9] = mp_obj_new_str(sec, strlen(sec));

    sec[0] = '\0';
    libwifi_get_pairwise_ciphers(bss, sec);
    t[10] = mp_obj_new_str(sec, strlen(sec));

    sec[0] = '\0';
    libwifi_get_auth_key_suites(bss, sec);
    t[11] = mp_obj_new_str(sec, strlen(sec));

    t[12] = tags_to_list(&bss->tags);
    return mp_obj_new_attrtuple(fields, 13, t);
}

/* STA detail (probe_req / (re)assoc_req). */
static mp_obj_t sta_detail(struct libwifi_sta *sta) {
    STATIC const qstr fields[] = {
        MP_QSTR_ssid, MP_QSTR_broadcast_ssid, MP_QSTR_bssid,
        MP_QSTR_transmitter, MP_QSTR_receiver, MP_QSTR_channel,
        MP_QSTR_randomized, MP_QSTR_tags,
    };
    mp_obj_t t[8];
    t[0] = (sta->broadcast_ssid || sta->ssid[0] == '\0')
                ? mp_const_none
                : mp_obj_new_str(sta->ssid, strlen(sta->ssid));
    t[1] = mp_obj_new_bool(sta->broadcast_ssid);
    t[2] = mac_bytes(sta->bssid);
    t[3] = mac_bytes(sta->transmitter);
    t[4] = mac_bytes(sta->receiver);
    t[5] = mp_obj_new_int(sta->channel);
    t[6] = mp_obj_new_bool(sta->randomized);
    t[7] = tags_to_list(&sta->tags);
    return mp_obj_new_attrtuple(fields, 8, t);
}

/* Deauth / disassoc share a tiny detail. */
static mp_obj_t deauth_detail(struct libwifi_parsed_deauth *d) {
    STATIC const qstr fields[] = { MP_QSTR_reason_code, MP_QSTR_tags };
    mp_obj_t t[2] = {
        mp_obj_new_int(d->fixed_parameters.reason_code),
        tags_to_list(&d->tags),
    };
    return mp_obj_new_attrtuple(fields, 2, t);
}
static mp_obj_t disassoc_detail(struct libwifi_parsed_disassoc *d) {
    STATIC const qstr fields[] = { MP_QSTR_reason_code, MP_QSTR_tags };
    mp_obj_t t[2] = {
        mp_obj_new_int(d->fixed_parameters.reason_code),
        tags_to_list(&d->tags),
    };
    return mp_obj_new_attrtuple(fields, 2, t);
}

/* ------------------------------------------------------------------------- */
/* Top-level entry. Dispatches on type/subtype, calls the right libwifi      */
/* parser, builds the result, and frees libwifi-owned heap before returning. */
/* ------------------------------------------------------------------------- */

mp_obj_t mod_libwifi_parse_frame(const uint8_t *data, size_t data_len) {
    if (data == NULL || data_len < 2) {
        return mp_const_none;
    }

    struct libwifi_frame frame = {0};
    /* radiotap=0: ESP32's promiscuous callback delivers the bare MAC frame. */
    if (libwifi_get_wifi_frame(&frame, data, data_len, 0) != 0) {
        return mp_const_none;
    }

    mp_obj_t detail = mp_const_none;

    /* Only management frames have rich parsers in libwifi today. */
    if (frame.frame_control.type == TYPE_MANAGEMENT) {
        switch (frame.frame_control.subtype) {
        case SUBTYPE_BEACON: {
            struct libwifi_bss bss = {0};
            if (libwifi_parse_beacon(&bss, &frame) == 0) {
                detail = bss_detail(&bss);
            }
            libwifi_free_bss(&bss);
            break;
        }
        case SUBTYPE_PROBE_RESP: {
            struct libwifi_bss bss = {0};
            if (libwifi_parse_probe_resp(&bss, &frame) == 0) {
                detail = bss_detail(&bss);
            }
            libwifi_free_bss(&bss);
            break;
        }
        case SUBTYPE_ASSOC_RESP: {
            struct libwifi_bss bss = {0};
            if (libwifi_parse_assoc_resp(&bss, &frame) == 0) {
                detail = bss_detail(&bss);
            }
            libwifi_free_bss(&bss);
            break;
        }
        case SUBTYPE_REASSOC_RESP: {
            struct libwifi_bss bss = {0};
            if (libwifi_parse_reassoc_resp(&bss, &frame) == 0) {
                detail = bss_detail(&bss);
            }
            libwifi_free_bss(&bss);
            break;
        }
        case SUBTYPE_PROBE_REQ: {
            struct libwifi_sta sta = {0};
            if (libwifi_parse_probe_req(&sta, &frame) == 0) {
                detail = sta_detail(&sta);
            }
            libwifi_free_sta(&sta);
            break;
        }
        case SUBTYPE_ASSOC_REQ: {
            struct libwifi_sta sta = {0};
            if (libwifi_parse_assoc_req(&sta, &frame) == 0) {
                detail = sta_detail(&sta);
            }
            libwifi_free_sta(&sta);
            break;
        }
        case SUBTYPE_REASSOC_REQ: {
            struct libwifi_sta sta = {0};
            if (libwifi_parse_reassoc_req(&sta, &frame) == 0) {
                detail = sta_detail(&sta);
            }
            libwifi_free_sta(&sta);
            break;
        }
        case SUBTYPE_DEAUTH: {
            struct libwifi_parsed_deauth d = {0};
            if (libwifi_parse_deauth(&d, &frame) == 0) {
                detail = deauth_detail(&d);
            }
            /* libwifi_parse_deauth allocates tags.parameters internally. */
            free(d.tags.parameters);
            break;
        }
        case SUBTYPE_DISASSOC: {
            struct libwifi_parsed_disassoc d = {0};
            if (libwifi_parse_disassoc(&d, &frame) == 0) {
                detail = disassoc_detail(&d);
            }
            free(d.tags.parameters);
            break;
        }
        default:
            break;
        }
    }

    mp_obj_t result = build_base(&frame, detail);

    /* build_base() copied everything it needs into MP-owned objects. */
    libwifi_free_wifi_frame(&frame);

    return result;
}

#endif /* MOD_LIBWIFI_ENABLED */
