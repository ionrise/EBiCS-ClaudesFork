/*
 * ionctrl.c - IONRISE protocol, controller side. See ionctrl.h and IONRISE_PROTOCOL.md.
 * Flash budget: this file + ionproto.c must fit in the space freed by the KM5S
 * driver on the 32 KB Lishui (~1.7 KB). Keep it lean.
 */
#include "ionctrl.h"

static ion_rx_t rx;
static volatile uint8_t frame_ready;      /* set by ionc_rx_byte, frame stays in rx.buf until served */
static uint8_t  tx[ION_MAX_FRAME];
static uint8_t  linked;
static uint32_t nonce_c;
static uint16_t last_ok_ms;
static uint8_t  stale;            /* telemetry timed out: assist forced off, session kept */

void ionc_init(void)
{
    rx.pos = 0;
    frame_ready = 0;
    linked = 0;
    stale = 0;
    nonce_c = 0;
    last_ok_ms = 0;
}

void ionc_rx_byte(uint8_t b)
{
    if (frame_ready) return;              /* previous frame not served yet: drop (display will retry) */
    if (ion_rx_byte(&rx, b)) frame_ready = 1;
}

uint8_t ionc_linked(void) { return linked && !stale; }

static void reply(uint8_t cmd, uint8_t seq, const uint8_t *p, uint8_t n)
{
    ionc_port_send(tx, ion_build(tx, (uint8_t)(cmd | ION_RESP), seq, p, n));
}

static void handle(uint16_t now_ms)
{
    const uint8_t *in = ION_RX_PAYLOAD(&rx);
    uint8_t len = ION_RX_LEN(&rx), cmd = ION_RX_CMD(&rx), seq = ION_RX_SEQ(&rx);
    uint8_t out[ION_MAX_PAYLOAD];
    uint8_t n = 1;
    const ionc_info_t *I = &ionc_info;
    out[0] = ION_OK;

    if (cmd == ION_CMD_HELLO) {
        if (len < 8) { out[0] = ION_E_LENGTH; goto send; }
        linked = 0;
        nonce_c = ionc_port_random() ^ ((uint32_t)now_ms << 16) ^ ion_get32(in + 4);
        out[0] = ION_PROTO_MAJOR;
        out[1] = ION_PROTO_MINOR;
        out[2] = I->fw_family;
        out[3] = I->hw;
        ion_put16(out + 4, I->fw_version);
        out[6] = I->layout_id;
        out[7] = I->layout_ver;
        ion_put16(out + 8, I->settings_size);
        out[10] = ionc_port_source();
        out[11] = 0;
        ion_put16(out + 12, I->caps);
        ion_put16(out + 14, ion_proof(ion_get32(in + 4), I->fw_family));
        ion_put32(out + 16, nonce_c);
        n = 20;
        goto send;
    }
    if (cmd == ION_CMD_AUTH) {
        if (len < 3) { out[0] = ION_E_LENGTH; goto send; }
        if (ion_get16(in) != ion_proof(nonce_c, in[2])) { out[0] = ION_E_AUTH; goto send; }
        linked = 1;
        stale = 0;
        last_ok_ms = now_ms;
        for (uint16_t i = 0; i < I->settings_size; i++) I->staging[i] = I->active[i];
        goto send;
    }
    if (!linked) { out[0] = ION_E_NOT_LINKED; goto send; }

    switch (cmd) {
    case ION_CMD_TELEMETRY:
        if (len < ION_TQ_LEN) { out[0] = ION_E_LENGTH; break; }
        last_ok_ms = now_ms;
        stale = 0;
        ionc_port_control(in);
        ionc_port_telemetry(out);          /* telemetry has no status byte */
        n = ION_TR_LEN;
        break;

    case ION_CMD_SET_READ: {               /* space u8, offset u16, count u8 */
        if (len < 4) { out[0] = ION_E_LENGTH; break; }
        uint16_t off = ion_get16(in + 1);
        uint8_t cnt = in[3];
        if (cnt == 0 || cnt > ION_MAX_PAYLOAD - 3 || off + cnt > I->settings_size || in[0] > 1) {
            out[0] = ION_E_RANGE; break;
        }
        const uint8_t *src = (in[0] ? I->staging : I->active) + off;
        ion_put16(out + 1, off);
        for (uint8_t i = 0; i < cnt; i++) out[3 + i] = src[i];
        n = (uint8_t)(3 + cnt);
        break;
    }

    case ION_CMD_SET_WRITE: {              /* offset u16, data[] */
        if (len < 3) { out[0] = ION_E_LENGTH; break; }
        uint16_t off = ion_get16(in);
        uint8_t cnt = (uint8_t)(len - 2);
        if (off + cnt > I->settings_size) { out[0] = ION_E_RANGE; break; }
        for (uint8_t i = 0; i < cnt; i++) I->staging[off + i] = in[2 + i];
        break;
    }

    case ION_CMD_SET_APPLY:                /* mode u8 */
        if (len < 1) { out[0] = ION_E_LENGTH; break; }
        if ((in[0] & ION_APPLY_REBOOT) && !ionc_port_motor_stopped()) { out[0] = ION_E_BUSY; break; }
        out[0] = ionc_port_apply(in[0]);
        break;

    case ION_CMD_ACTION:                   /* action u8, arg[3] */
        if (len < 4) { out[0] = ION_E_LENGTH; break; }
        if (in[0] == ION_ACT_REVERT) {
            for (uint16_t i = 0; i < I->settings_size; i++) I->staging[i] = I->active[i];
            break;
        }
        out[0] = ionc_port_action(in[0], in + 1);
        break;

    default:
        out[0] = ION_E_UNSUPPORTED;
    }
send:
    reply(cmd, seq, out, n);
}

void ionc_service(uint16_t now_ms)
{
    if (frame_ready) {
        handle(now_ms);
        frame_ready = 0;
    }
    /* no valid TELEMETRY for IONC_LINK_TIMEOUT_MS: assist off at once (safety), but keep the
     * session so that control resumes with the next good TELEMETRY; a new HELLO / a reboot ends it */
    if (linked && !stale && (uint16_t)(now_ms - last_ok_ms) > IONC_LINK_TIMEOUT_MS) {
        stale = 1;
        ionc_port_link_lost();
    }
}
