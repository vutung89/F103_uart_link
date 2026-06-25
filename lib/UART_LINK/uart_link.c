/*
 * uart_link.c
 *
 *  Created on: Jun 12, 2026
 *      Author: admin
 *
 *  Data-flow:
 *
 *   UART RX DMA Circular
 *          │
 *          ▼
 *   HAL_UARTEx_RxEventCallback
 *          │  Uart_Link_RxEventCallback()
 *          ▼
 *   DMA Reader (old_pos / new_pos)
 *          │   copy data into ring buffer
 *          ▼
 *   RX Ring Buffer  (ul->rb_rx)        ← SPSC, ISR writes / task reads
 *          │   
 *          ▼
 *   Protocol Task  (Protocol_Task)
 *          │   processes bytes into frames, verifies CRC, pushes to frame queue
 *          ▼
 *   Protocol State Machine             [HEADER → LENGTH → PAYLOAD → CRC]
 *          │  CRC-8 verified
 *          ▼
 *   Frame Queue  (ul->rx_queue)
 *          │   push frames from protocol task, pop frames in command task
 *          ▼
 *   Command Handler  (Command_Task)
 *          │  builds response
 *          ▼
 *   TX Queue  (ul->tx_queue)
 *          │   push frames from command task, pop frames in TX task
 *          ▼
 *   UART TX DMA Driver  (TX_Task)
 *          │  HAL_UART_Transmit_DMA()
 *          ▼
 *   HAL_UART_TxCpltCallback
 *          │  Uart_Link_TxCpltCallback()
 *          ▼
 *   Clear tx_busy flag to allow next frame
 */

 /**
  * 
  * Example usage:
  * HEARTBEAT 
  * send (hex): FD 01 03 40 21
  * response: FD 01 03 40 21
  * SET_PID
  * send (hex): FD 07 01 00 40 00 0D 00 00 14
  * response: FD 02 01 00 8B
*/

#include "uart_link.h"
#include "crc16.h"


/* =========================================================================
 * Ring Bufer Circle
 * ========================================================================= */
bool RingBuffer_Write(RingBuffer_t *rb, uint8_t byte)
{
    uint16_t next_head = (rb->head + 1u) & (UART_RX_RING_SIZE - 1u);
    if (next_head == rb->tail)
        return false;           /* overflow – drop byte */
    rb->buf[rb->head] = byte;
    rb->head = next_head;
    return true;
}

bool RingBuffer_Read(RingBuffer_t *rb, uint8_t *byte)
{
    if (rb->tail == rb->head)
        return false;
    *byte    = rb->buf[rb->tail];
    uint16_t next_tail = (rb->tail + 1u) & (UART_RX_RING_SIZE - 1u);
    rb->tail = next_tail;
    return true;
}

/* =========================================================================
 * Internal queue helpers
 * ========================================================================= */

bool RxQueue_Push(RxQueue_t *q, const Frame_t *frame)
{
    uint8_t next = (uint8_t)((q->head + 1u) % RX_QUEUE_DEPTH);
    if (next == q->tail)
        return false;   /* queue full */
    q->buf[q->head] = *frame;
    q->head = next;
    return true;
}

bool RxQueue_Pop(RxQueue_t *q, Frame_t *frame)
{
    if (q->tail == q->head)
        return false;   /* queue empty */
    *frame  = q->buf[q->tail];
    q->tail = (uint8_t)((q->tail + 1u) % RX_QUEUE_DEPTH);
    return true;
}

bool TxQueue_Push(TxQueue_t *q, const TxFrame_t *frame)
{
    uint8_t next = (uint8_t)((q->head + 1u) % TX_QUEUE_DEPTH);
    if (next == q->tail)
        return false;
    q->buf[q->head] = *frame;
    q->head = next;
    return true;
}

bool TxQueue_Pop(TxQueue_t *q, TxFrame_t *frame)
{
    if (q->tail == q->head)
        return false;
    *frame  = q->buf[q->tail];
    q->tail = (uint8_t)((q->tail + 1u) % TX_QUEUE_DEPTH);
    return true;
}


/* =========================================================================
 * TX helpers – build encoded frame and kick DMA
 * =========================================================================
 *  Wire format:  [HEAD] [LEN] [PAYLOAD × LEN] [CRC8]
 *  CRC8 covers:  LEN + all PAYLOAD bytes
 * ========================================================================= */

void TX_SendFrame(Uart_Link_t *ul)
{
    TxFrame_t frame;

    if (ul->tx_busy)
        return;

    if (!TxQueue_Pop(&ul->tx_queue, &frame))
        return;

    /* Copy into the persistent DMA buffer (must stay valid until TxCplt) */
    memcpy(ul->tx_dma_buf, frame.data, frame.len);
    ul->tx_busy = true;

    HAL_UART_Transmit_DMA(ul->huart, ul->tx_dma_buf, frame.len);
}

/* =========================================================================
 * Public API – Init
 * ========================================================================= */

void Uart_Link_Init(Uart_Link_t *ul, UART_HandleTypeDef *huart)
{
    if (ul == NULL || huart == NULL || huart->Instance == NULL)
        return;

    memset(ul, 0, sizeof(*ul));

    ul->huart    = huart;
    ul->tx_busy  = false;

    /* Protocol decoder starts in WAIT_HEADER */
    ul->proto.state = PROTO_WAIT_HEADER;

    /* Start DMA circular reception */
    HAL_UARTEx_ReceiveToIdle_DMA(
        ul->huart,
        ul->dma_rx_buf,
        UART_RX_DMA_SIZE);

    /* Disable half-transfer interrupt – we only need idle/TC events */
    __HAL_DMA_DISABLE_IT(ul->huart->hdmarx, DMA_IT_HT);
}

/* =========================================================================
 * Public API – RX event callback  (called from ISR context)
 * ========================================================================= */

void Uart_Link_RxEventCallback(
    Uart_Link_t       *ul,
    UART_HandleTypeDef *huart,
    uint16_t            size)
{

    if (huart->Instance != ul->huart->Instance)
        return;
    
    if (size > UART_RX_DMA_SIZE)
        return;

    /* 'size' từ HAL = số bytes DMA đã ghi vào buffer (vị trí write hiện tại) */
    uint16_t dma_new_pos = size;

    if (dma_new_pos == ul->dma_old_pos)
        return;

    if (dma_new_pos > ul->dma_old_pos)
    {
        /* Trường hợp thường: chưa wrap */
        for (uint16_t i = ul->dma_old_pos; i < dma_new_pos; i++)
        {
            RingBuffer_Write(&ul->rb_rx, ul->dma_rx_buf[i]);
        }
    }
    else
    {
        /* Wrap-around: đọc từ old_pos đến cuối buffer, rồi từ 0 đến new_pos */
        for (uint16_t i = ul->dma_old_pos; i < UART_RX_DMA_SIZE; i++)
        {
            RingBuffer_Write(&ul->rb_rx, ul->dma_rx_buf[i]);
        }
        for (uint16_t i = 0; i < dma_new_pos; i++)
        {
            RingBuffer_Write(&ul->rb_rx, ul->dma_rx_buf[i]);
        }
    }

    ul->dma_old_pos = dma_new_pos;

}

/* =========================================================================
 * Public API – TX complete callback  (called from ISR context)
 * ========================================================================= */

void Uart_Link_TxCpltCallback(
    Uart_Link_t       *ul,
    UART_HandleTypeDef *huart)
{
    if (huart->Instance != ul->huart->Instance)
        return;

    ul->tx_busy = false;
}

/* =========================================================================
 * Protocol state machine  (task context)
 * ========================================================================= */
void Protocol_ProcessByte(
    Protocol_t   *proto,
    uint8_t       byte,
    RxQueue_t *rq)
{
    switch (proto->state)
    {
        case PROTO_WAIT_HEADER:
        {
            if (byte == PROTO_HEADER)
            {
                proto->state = PROTO_WAIT_LENGTH;
            }
            break;
        }

        case PROTO_WAIT_LENGTH:
        {
            if ((byte == 0u) || (byte > PAYLOAD_MAX_SIZE))
            {
                proto->state = PROTO_WAIT_HEADER;
                break;
            }
            proto->crc_accum = CRC16_Init();

            proto->crc_accum =CRC16_Update(proto->crc_accum, byte);

            proto->length = byte;
            proto->index  = 0u;

            proto->state = PROTO_WAIT_PAYLOAD;
            break;
        }

        case PROTO_WAIT_PAYLOAD:
        {
            proto->payload[proto->index++] = byte;

            proto->crc_accum = CRC16_Update(proto->crc_accum, byte);

            if (proto->index >= proto->length)
            {
                proto->crc_rx = 0u;
                proto->state  = PROTO_WAIT_CRC_LO;
            }

            break;
        }

        case PROTO_WAIT_CRC_LO:
        {
            proto->crc_rx = byte;
            proto->state  = PROTO_WAIT_CRC_HI;
            break;
        }

        case PROTO_WAIT_CRC_HI:
        {
            proto->crc_rx |= ((uint16_t)byte << 8);

            if (proto->crc_accum == proto->crc_rx)
            {
                Frame_t frame;

                frame.len = proto->length;

                memcpy(frame.data,
                    proto->payload,
                    proto->length);

                RxQueue_Push(rq, &frame);
            }

            proto->state = PROTO_WAIT_HEADER;
            break;
        }

        default:
        {
            proto->state = PROTO_WAIT_HEADER;
            break;
        }
    }
}

void Protocol_Task(Uart_Link_t *ul)
{
    uint8_t byte;

    while (RingBuffer_Read(&ul->rb_rx, &byte))
    {
        Protocol_ProcessByte(
            &ul->proto,
            byte,
            &ul->rx_queue);
    }
}

/* =========================================================================
 * Command handler  (task context)
 *
 *  Add application logic inside the CMD_* cases.
 *  Call Uart_Link_Send() to enqueue a response frame.
 * ========================================================================= */

void Command_Process(Uart_Link_t *ul, const Frame_t *frame)
{
    if (frame->len == 0u)
        return;
        
    switch (frame->data[0])
    {
        case CMD_SET_PID:
        {
            /*
             * Expected payload: [CMD_SET_PID][kp_hi][kp_lo][ki_hi][ki_lo]…
             * TODO: parse parameters and apply to PID controller.
             * Range: 0.00 to 655.35, scaled by 100 (e.g. 0x007B → 123 → 1.23)
             *
             */

            // uint8_t kp = ((frame->data[1] << 8) | frame->data[2] )/100;
            // uint8_t ki = ((frame->data[3] << 8) | frame->data[4] )/100;
            // uint8_t kd = ((frame->data[5] << 8) | frame->data[6] )/100;

            // Respond with ACK
            uint8_t ack[] = { CMD_SET_PID, ACK_OK};
            Uart_Link_Send(ul, ack, sizeof(ack));
            break;
        }

        case CMD_GET_STATUS:
        {
            /*
             * TODO: fill status_buf with actual system state.
             *
             * Example:
             *   uint8_t status_buf[5];
             *   status_buf[0] = CMD_GET_STATUS;
             *   // … populate remaining bytes …
             *   Uart_Link_Send(ul, status_buf, sizeof(status_buf));
             */
            break;
        }

        case CMD_HEARTBEAT:
        {
            /* Echo heartbeat back to host */
            uint8_t ack = CMD_HEARTBEAT;
            Uart_Link_Send(ul, &ack, 1u);
            break;
        }

        case CMD_ECHO_FRAME:
        {
            /* Echo received frame back to host (for testing) */
            Uart_Link_Send(ul, frame->data, frame->len);
            break;
        }

        default:
            /* Unknown command – silently ignore */
            break;
    }
}

void Command_Task(Uart_Link_t *ul)
{
    Frame_t frame;

    while (RxQueue_Pop(&ul->rx_queue, &frame))
        Command_Process(ul, &frame);
}

/* =========================================================================
 * TX dispatch task  (task context)
 * ========================================================================= */

void TX_Task(Uart_Link_t *ul)
{
    TX_SendFrame(ul);
}

/* =========================================================================
 * Public API – main task entry point
 * ========================================================================= */

void Uart_Link_Task(Uart_Link_t *ul)
{
    Protocol_Task(ul);  /* ring buffer → frame queue */
    Command_Task(ul);   /* frame queue → tx queue    */
    TX_Task(ul);        /* tx queue    → DMA         */
}

/* =========================================================================
 * Public API – enqueue an outgoing frame
 *
 *  Encodes:  [HEAD][LEN][PAYLOAD…][CRC]
 *  Returns false if TX queue is full or payload is too large.
 * ========================================================================= */
bool Uart_Link_Send(Uart_Link_t *ul,
                    const uint8_t *payload,
                    uint8_t len)
{
    if (ul == NULL || payload == NULL)
        return false;

    if (len == 0u || len > PAYLOAD_MAX_SIZE)
        return false;

    /* header(1) + len(1) + crc16_lo + crc16_hi */
    if ((uint16_t)(len + 4u) > TX_FRAME_MAX_SIZE)
        return false;

    TxFrame_t tx;
    uint16_t  idx = 0u;

    tx.data[idx++] = PROTO_HEADER;
    tx.data[idx++] = len;

    memcpy(&tx.data[idx], payload, len);
    idx += len;

    uint16_t crc = CRC16_Calculate(&tx.data[1], (uint16_t)(len + 1u)); //LEN + PAYLOAD

    tx.data[idx++] = (uint8_t)(crc & 0xFF);         // CRC_LO
    tx.data[idx++] = (uint8_t)((crc >> 8) & 0xFF); // CRC_HI

    tx.len = idx;

    if (!TxQueue_Push(&ul->tx_queue, &tx))
        return false;

    return true;
}
