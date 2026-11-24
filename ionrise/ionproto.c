/*
 * ionproto.c - IONRISE protocol codec (framing, CRC, handshake proof)
 * Shared by display and controllers. Small on purpose: ~300 bytes Thumb-2 at -Os.
 * See protocol/IONRISE_PROTOCOL.md.
 */
#include "ionproto.h"

/* CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no final xor.
 * Check value: "123456789" -> 0x29B1. Bitwise: slow but tiny (frames are short). */
uint16_t ion_crc16(uint16_t crc, const uint8_t *p, uint16_t n)
{
    while (n--) {
        crc ^= (uint16_t)(*p++) << 8;
        for (uint8_t i = 0; i < 8; i++)
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
    }
    return crc;
}

uint8_t ion_build(uint8_t *out, uint8_t cmd, uint8_t seq, const uint8_t *payload, uint8_t len)
{
    if (len > ION_MAX_PAYLOAD) len = ION_MAX_PAYLOAD;
    out[0] = ION_SOF;
    out[1] = (uint8_t)(len + 2u);
    out[2] = cmd;
    out[3] = seq;
    for (uint8_t i = 0; i < len; i++) out[4 + i] = payload[i];
    uint16_t c = ion_crc16(0xFFFFu, out + 1, (uint16_t)(len + 3u));
    out[4 + len] = (uint8_t)c;
    out[5 + len] = (uint8_t)(c >> 8);
    return (uint8_t)(len + 6u);
}

/* Receiver: hunts for SOF, checks LEN, collects the frame, verifies the CRC.
 * On a bad frame it rescans the bytes after the false SOF, so a 0xA5 inside a
 * payload cannot lock the receiver out of the next real frame. */
/* drop buf[0] and everything up to the next SOF */
static void ion_rx_shift(ion_rx_t *rx)
{
    uint8_t i = 1, j = 0;
    while (i < rx->pos && rx->buf[i] != ION_SOF) i++;
    while (i < rx->pos) rx->buf[j++] = rx->buf[i++];
    rx->pos = j;
}

uint8_t ion_rx_byte(ion_rx_t *rx, uint8_t b)
{
    if (rx->pos == 0 && b != ION_SOF) return 0;
    if (rx->pos >= ION_MAX_FRAME) rx->pos = 0;            /* cannot happen, keeps the index safe */
    rx->buf[rx->pos++] = b;

    while (rx->pos >= 2u) {                              /* loop only re-checks after a shift */
        uint8_t n = rx->buf[1];
        if (n < 2u || n > ION_MAX_PAYLOAD + 2u) { ion_rx_shift(rx); continue; }
        if (rx->pos < (uint8_t)(n + 4u)) return 0;       /* frame not complete yet */
        uint16_t c = ion_crc16(0xFFFFu, rx->buf + 1, (uint16_t)(n + 1u));
        if ((uint8_t)c == rx->buf[n + 2] && (uint8_t)(c >> 8) == rx->buf[n + 3]) {
            rx->pos = 0;
            return 1;
        }
        ion_rx_shift(rx);                                /* bad CRC: resync on a later SOF */
    }
    return 0;
}

uint16_t ion_proof(uint32_t nonce, uint8_t id)
{
    uint8_t b[5];
    ion_put32(b, nonce);
    b[4] = id;
    uint16_t c = ion_crc16(0xFFFFu, b, 5);
    return ion_crc16(c, (const uint8_t *)ION_KEY, ION_KEY_LEN);
}
