#include "bsp_can.h"
#include "string.h"

/* select can */
#define DEV_CAN0_USED
// #define DEV_CAN1_USED

#ifdef DEV_CAN0_USED
#define CANX CAN0
#else
#define CANX CAN1
#endif

#define TX_SIZE 512
#define RX_SIZE 1024
#define FILTER_NUM sizeof(can_rx) / sizeof(ST_CAN_RX)

#define RX_FIFO_MB_IDX (5)
#define TX_MB_IDX (9)

static struct
{
    uint8_t buf[TX_SIZE];
    uint16_t wr;
    uint16_t rd;
} can_tx;

typedef struct
{
    uint16_t can_id;
    uint16_t wr;
    uint16_t rd;
    uint8_t buf[RX_SIZE];
} ST_CAN_RX;

ST_CAN_RX can_rx[3] =
    {
        {0x301, 0, 0, {0}},
        {0x350, 0, 0, {0}},
        {0x351, 0, 0, {0}},
};

FlagStatus receive_flag;
uint8_t transmit_number = 0x0;
can_receive_message_struct g_receive_message;
can_trasnmit_message_struct g_transmit_message;
uint32_t can_send_error_cnt = 0;

/*!
    \brief      initialize CAN and filter
    \param[in]  can_parameter
      \arg        can_parameter_struct
    \param[in]  can_filter
      \arg        can_filter_parameter_struct
    \param[out] none
    \retval     none
*/
void bsp_can_networking_init(void)
{
    can_parameter_struct can_parameter;
    can_filter_parameter_struct can_filter;

    can_struct_para_init(CAN_INIT_STRUCT, &can_parameter);
    can_struct_para_init(CAN_FILTER_STRUCT, &can_filter);

    /* initialize CAN register */
    can_deinit(CANX);

    /* initialize CAN */
    can_parameter.time_triggered = DISABLE;
    can_parameter.auto_bus_off_recovery = ENABLE;
    can_parameter.auto_wake_up = DISABLE;
    can_parameter.auto_retrans = ENABLE;
    can_parameter.rec_fifo_overwrite = DISABLE;
    can_parameter.trans_fifo_order = DISABLE;
    can_parameter.working_mode = CAN_NORMAL_MODE;
    can_parameter.resync_jump_width = CAN_BT_SJW_1TQ;
    can_parameter.time_segment_1 = CAN_BT_BS1_7TQ;
    can_parameter.time_segment_2 = CAN_BT_BS2_2TQ;
    /* baudrate 1Mbps */
    can_parameter.prescaler = 5;
    can_init(CANX, &can_parameter);

    /* initialize filter */
#ifdef DEV_CAN0_USED
    /* CAN0 filter number */
    can_filter.filter_number = 0;
#else
    /* CAN1 filter number */
    can_filter.filter_number = 15;
#endif
    /* initialize filter */
    can_filter.filter_mode = CAN_FILTERMODE_MASK;
    can_filter.filter_bits = CAN_FILTERBITS_32BIT;
    can_filter.filter_list_high = 0x0000;
    can_filter.filter_list_low = 0x0000;
    can_filter.filter_mask_high = 0x0000;
    can_filter.filter_mask_low = 0x0000;
    can_filter.filter_fifo_number = CAN_FIFO1;
    can_filter.filter_enable = ENABLE;
    can_filter_init(&can_filter);
}

/*!
    \brief      configure the nested vectored interrupt controller
    \param[in]  none
    \param[out] none
    \retval     none
*/
void bsp_can_nvic_config(void)
{
#ifdef DEV_CAN0_USED
    /* configure CAN0 NVIC */
    nvic_irq_enable(CAN0_RX1_IRQn, 0, 0);
#else
    /* configure CAN1 NVIC */
    nvic_irq_enable(CAN1_RX1_IRQn, 0, 0);
#endif
}

/*!
    \brief      configure GPIO
    \param[in]  none
    \param[out] none
    \retval     none
*/
void bsp_can_gpio_config(void)
{
    /* enable can clock */
    rcu_periph_clock_enable(RCU_CAN0);
    rcu_periph_clock_enable(RCU_CAN1);
    rcu_periph_clock_enable(RCU_GPIOB);

    /* configure CAN0 GPIO */
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_8);
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_8);
    gpio_af_set(GPIOB, GPIO_AF_9, GPIO_PIN_8);

    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_9);
    gpio_af_set(GPIOB, GPIO_AF_9, GPIO_PIN_9);

    /* configure CAN1 GPIO */
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_5);
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_5);
    gpio_af_set(GPIOB, GPIO_AF_9, GPIO_PIN_5);

    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_6);
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_6);
    gpio_af_set(GPIOB, GPIO_AF_9, GPIO_PIN_6);
}

void bsp_can_init(void)
{
    bsp_can_gpio_config();
    bsp_can_networking_init();
    bsp_can_nvic_config();

    /* enable CAN receive FIFO1 not empty interrupt */
    can_interrupt_enable(CANX, CAN_INT_RFNE1);

    /* initialize transmit message */
    can_struct_para_init(CAN_TX_MESSAGE_STRUCT, &g_transmit_message);
    g_transmit_message.tx_sfid = 0x00;
    g_transmit_message.tx_efid = 0x00;
    g_transmit_message.tx_ft = CAN_FT_DATA;
    g_transmit_message.tx_ff = CAN_FF_EXTENDED;
    g_transmit_message.tx_dlen = 8;

    /* initialize receive message */
    can_struct_para_init(CAN_RX_MESSAGE_STRUCT, &g_receive_message);
}

void can0_send_msg(uint32_t id, uint8_t *msg)
{
    g_transmit_message.tx_sfid = id;
    memcpy(&g_transmit_message.tx_data[0], &msg[0], 8);
    if (CAN_NOMAILBOX == can_message_transmit(CANX, &g_transmit_message))
    {
        can_send_error_cnt++;
    }
}

void CAN0_RX1_IRQHandler(void)
{
    can_receive_message_struct rx_frame;
    /* check the receive message */
    can_message_receive(CANX, CAN_FIFO1, &rx_frame);

    for (uint8_t n = 0; n < (sizeof(can_rx) / sizeof(can_rx[0])); n++)
    {
        if (rx_frame.rx_sfid == can_rx[n].can_id)
        {
            for (uint8_t i = 0; i < rx_frame.rx_dlen; i++)
            {
                can_rx[n].buf[can_rx[n].wr++] = rx_frame.rx_data[i];
                can_rx[n].wr &= (RX_SIZE - 1);
            }
            break;
        }
    }
}

static __inline uint16_t can_receive_index(uint8_t n, uint8_t *buf, uint16_t buf_size)
{
    uint16_t i, len;

    len = (can_rx[n].wr - can_rx[n].rd) & (RX_SIZE - 1);
    len = (len > buf_size) ? buf_size : len;

    for (i = 0; i < len; i++)
    {
        buf[i] = can_rx[n].buf[can_rx[n].rd++];
        can_rx[n].rd &= (RX_SIZE - 1);
    }

    return len;
}

uint16_t can_receive(uint8_t *buf, uint16_t buf_size)
{
    return can_receive_index(0, buf, buf_size);
}

uint16_t can_tx_id = 0x310;

void can_send(uint8_t *buf, uint16_t len)
{
    can_trasnmit_message_struct tx_frame;
    /* initialize transmit message */
    can_struct_para_init(CAN_TX_MESSAGE_STRUCT, &tx_frame);
    tx_frame.tx_sfid = can_tx_id;
    tx_frame.tx_efid = 0x00;
    tx_frame.tx_ft = CAN_FT_DATA;
    tx_frame.tx_ff = CAN_FF_EXTENDED;
    tx_frame.tx_dlen = len;
    memcpy(&tx_frame.tx_data[0], &buf[0], len);

    if (CAN_NOMAILBOX == can_message_transmit(CANX, &tx_frame))
    {
        can_send_error_cnt++;
    }
}
