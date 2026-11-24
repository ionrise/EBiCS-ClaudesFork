/*
 * ionctrl.h - IONRISE protocol, controller side (EBiCS Lishui / Kunteng)
 *
 * The protocol logic is here; everything firmware specific goes through the
 * port hooks below, implemented in the firmware (see IONRISE_PROTOCOL.md,
 * "Integration in EBiCS"). The controller only answers requests.
 */
#ifndef IONCTRL_H
#define IONCTRL_H

#include <stdint.h>
#include "ionproto.h"

/* no valid TELEMETRY for this long -> assist 0 at once; the session stays, control resumes with
 * the next good TELEMETRY (only HELLO or a reboot ends the session) */
#define IONC_LINK_TIMEOUT_MS   500u

typedef struct {
    uint8_t  fw_family;       /* ION_FW_* */
    uint8_t  hw;              /* hardware variant, firmware defined (0 = unknown) */
    uint16_t fw_version;      /* major << 8 | minor */
    uint8_t  layout_id;       /* ION_LAYOUT_* */
    uint8_t  layout_ver;
    uint16_t settings_size;   /* bytes, <= 65535 */
    uint16_t caps;            /* ION_CAP_* */
    uint8_t *active;          /* settings block in use (RAM) */
    uint8_t *staging;         /* staging copy, settings_size bytes (RAM) */
} ionc_info_t;

/* ---- provided by the firmware (port) ---- */
/* const by default; a firmware that fills in hw / caps at start-up builds with -DIONC_INFO_CONST= */
#ifndef IONC_INFO_CONST
#define IONC_INFO_CONST const
#endif
extern IONC_INFO_CONST ionc_info_t ionc_info;
void     ionc_port_send(const uint8_t *frame, uint8_t len);   /* start sending, may return at once (DMA) */
uint32_t ionc_port_random(void);                               /* any changing value: timer count, ADC noise */
uint8_t  ionc_port_source(void);                               /* ION_SRC_* */
uint8_t  ionc_port_motor_stopped(void);                        /* 1 = wheel stopped and PWM off */
void     ionc_port_control(const uint8_t *req);                /* ION_TQ_* payload, ION_TQ_LEN bytes */
void     ionc_port_telemetry(uint8_t *resp);                   /* fill ION_TR_* payload, ION_TR_LEN bytes */
uint8_t  ionc_port_apply(uint8_t mode);                        /* validate staging, do ION_APPLY_*; ION_OK / ION_E_* */
uint8_t  ionc_port_action(uint8_t action, const uint8_t *arg); /* ION_ACT_*; ION_OK / ION_E_* */
void     ionc_port_link_lost(void);                            /* assist 0, walk off */

/* ---- called by the firmware ---- */
void    ionc_init(void);                  /* at start-up (clears the protocol state) */
void    ionc_rx_byte(uint8_t b);          /* from the UART RX interrupt or a DMA ring reader */
void    ionc_service(uint16_t now_ms);    /* main loop, >= every 10 ms; now_ms free-running */
uint8_t ionc_linked(void);                /* 1 while an authenticated display sends valid telemetry */

#endif
