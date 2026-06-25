/*
 * uart_link.h
 *
 *  Created on: Jun 12, 2026
 *      Author: admin
 */

#ifndef INC_UART_LINK_H_
#define INC_UART_LINK_H_

#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

/* =========================================================================
 * Size constants
 * ========================================================================= */

#define UART_RX_DMA_SIZE        512u     /* DMA circular buffer (bytes)       */
#define UART_RX_RING_SIZE       1024u    /* Software ring buffer (power-of-2) */

#define PAYLOAD_MAX_SIZE        128u     /* Max payload bytes per frame        */
#define RX_QUEUE_DEPTH          32u      /* RX frame queue depth               */

#define TX_FRAME_MAX_SIZE       (PAYLOAD_MAX_SIZE + 4u)     /* Max TX frame bytes (header + len + crc16_lo + crc16_hi)*/
#define TX_QUEUE_DEPTH          32u      /* TX frame queue depth               */

/* =========================================================================
 * Protocol constants
 *   Frame layout: [0xAA][LEN_OF_PAYLOAD][PAYLOAD][CRC16_OF_LEN+PAYLOAD]
 * ========================================================================= */

#define PROTO_HEADER            0xFDu //same Mavlink v2 Start byte

/* Command IDs carried in payload[0] */
#define CMD_SET_PID             0x01u
#define CMD_GET_STATUS          0x02u
#define CMD_HEARTBEAT           0x03u
#define CMD_ECHO_FRAME          0x04u

#define ACK_OK                   0x00u
#define ACK_ERROR                0xFFu

/* =========================================================================
 * Ring buffer  (lock-free, SPSC, writer = ISR, reader = task)
 * ========================================================================= */

typedef struct
{
    uint8_t  buf[UART_RX_RING_SIZE];
    volatile uint16_t head;     /* written by ISR  */
    volatile uint16_t tail;     /* read  by task   */
} RingBuffer_t;

bool RingBuffer_Write(RingBuffer_t *rb, uint8_t byte);

bool RingBuffer_Read(RingBuffer_t *rb, uint8_t *byte);


/* =========================================================================
 * RX Frame (decoded, CRC-verified)
 * ========================================================================= */

typedef struct
{
    uint8_t len;
    uint8_t data[PAYLOAD_MAX_SIZE];
} Frame_t;

/* =========================================================================
 * TX Frame (fully encoded: header + len + payload + crc)
 * ========================================================================= */

typedef struct
{
    uint16_t len;
    uint8_t  data[TX_FRAME_MAX_SIZE];
} TxFrame_t;

/* =========================================================================
 * Generic circular queue macro – instantiated for Frame_t and TxFrame_t
 * ========================================================================= */

typedef struct
{
    Frame_t  buf[RX_QUEUE_DEPTH];
    volatile uint8_t head;
    volatile uint8_t tail;
} RxQueue_t;

typedef struct
{
    TxFrame_t buf[TX_QUEUE_DEPTH];
    volatile uint8_t head;
    volatile uint8_t tail;
} TxQueue_t;

bool RxQueue_Push(RxQueue_t *q, const Frame_t *frame);
bool RxQueue_Pop(RxQueue_t *q, Frame_t *frame);

bool TxQueue_Push(TxQueue_t *q, const TxFrame_t *frame);
bool TxQueue_Pop(TxQueue_t *q, TxFrame_t *frame);

/* =========================================================================
 * Protocol state machine
 * ========================================================================= */

typedef enum
{
    PROTO_WAIT_HEADER = 0,
    PROTO_WAIT_LENGTH,
    PROTO_WAIT_PAYLOAD,
    PROTO_WAIT_CRC_LO,
    PROTO_WAIT_CRC_HI
} ProtocolState_t;

typedef struct
{
    ProtocolState_t state;

    uint8_t  length;
    uint8_t  index;
    uint8_t  payload[PAYLOAD_MAX_SIZE];
    uint16_t crc_accum;
    uint16_t crc_rx;
} Protocol_t;
/* =========================================================================
 * Main UART Link handle
 *   All state is self-contained – no global variables.
 * ========================================================================= */

typedef struct
{
    /* ── HAL handle ───────────────────────────────────────────────────── */
    UART_HandleTypeDef *huart;

    /* ── DMA circular reader ──────────────────────────────────────────── */
    uint8_t  dma_rx_buf[UART_RX_DMA_SIZE];
    uint16_t dma_old_pos;           /* last known DMA write position      */

    /* ── Software ring buffer (ISR → task) ───────────────────────────── */
    RingBuffer_t rb_rx;

    /* ── Protocol decoder (persistent across task calls) ─────────────── */
    Protocol_t proto;

    /* ── RX frame queue (protocol → command handler) ─────────────────── */
    RxQueue_t rx_queue;

    /* ── TX frame queue (command handler → DMA driver) ───────────────── */
    TxQueue_t tx_queue;

    /* ── TX state ─────────────────────────────────────────────────────── */
    uint8_t          tx_dma_buf[TX_FRAME_MAX_SIZE]; /* active DMA buffer  */
    volatile bool    tx_busy;

} Uart_Link_t;

/* =========================================================================
 * Public API
 * ========================================================================= */
void Protocol_ProcessByte(Protocol_t *proto, uint8_t byte, RxQueue_t *fq);
void Protocol_Task(Uart_Link_t *ul);
void Command_Process(Uart_Link_t *ul, const Frame_t *frame);
void Command_Task(Uart_Link_t *ul);
void TX_Task(Uart_Link_t *ul);
void TX_SendFrame(Uart_Link_t *ul);



/**
 * @brief  Initialise the UART link handle and start circular DMA reception.
 * @param  ul     Pointer to Uart_Link_t handle (caller-allocated).
 * @param  huart  HAL UART handle with DMA already configured.
 */
void Uart_Link_Init(Uart_Link_t *ul, UART_HandleTypeDef *huart);

/**
 * @brief  Call from HAL_UARTEx_RxEventCallback().
 *         Copies new bytes from the DMA circular buffer into the ring buffer.
 */
void Uart_Link_RxEventCallback(Uart_Link_t *ul,
                               UART_HandleTypeDef *huart,
                               uint16_t size);

/**
 * @brief  Call from HAL_UART_TxCpltCallback().
 *         Clears tx_busy and immediately starts the next queued TX if any.
 */
void Uart_Link_TxCpltCallback(Uart_Link_t *ul,
                              UART_HandleTypeDef *huart);

/**
 * @brief  Main task – call repeatedly from the superloop (or a task).
 *         Runs protocol decoding, command processing, and TX dispatch.
 */
void Uart_Link_Task(Uart_Link_t *ul);

/**
 * @brief  Build and enqueue a TX frame from raw payload bytes.
 * @return true if the frame was enqueued, false if TX queue is full.
 */
bool Uart_Link_Send(Uart_Link_t *ul,
                    const uint8_t *payload,
                    uint8_t len);

#endif /* INC_UART_LINK_H_ */