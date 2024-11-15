#include <stm32h747xx.h>
#include "comm_mpsv.h"
#include "motion_controller.h"
#include "crc8.h"

#define USART_ICR_ALL ( USART_ICR_PECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_ORECF | USART_ICR_IDLECF\
                        | USART_ICR_TXFECF | USART_ICR_TCCF | USART_ICR_TCBGTCF | USART_ICR_LBDCF | USART_ICR_CTSCF \
                        | USART_ICR_RTOCF | USART_ICR_EOBCF | USART_ICR_UDRCF | USART_ICR_CMCF | USART_ICR_WUCF )


#define UART_RX_BUF_SIZE        (4096)

SV_Comm_t sv_data[COMM_SV_NUM];

/***********************
  プライベート変数
***********************/
static uint8_t uart_rx_buf[UART_RX_BUF_SIZE] = {0};
Queue comm_receive_queue = {0};
static USART_error_count usart_error_count = {0};
static uint8_t _mp_sv_current_write = 0;
static uint8_t _mp_sv_current_read = 1;
static COM_MP_TO_SV _mp_sv_mode;
static COM_MP_TO_SV _mp_sv_cmd;
static COM_SV_TO_MP _sv_mp;
static uint8_t _com_st_current_write = 0;
static uint8_t _com_st_current_read = 1;
static COM_STATUS _com_st[2];   // double buffer
static STATE_MACHINE state_mcn;

/***********************
  プロトタイプ宣言
***********************/
void init_queue( Queue* _q, uint8_t* _buf, uint32_t _size );
//static void init_receive_parser_buff(COM_MP_TO_SV * mp_sv);
static void init_receive_parser_buff_multi(COM_STATUS * com_st);
void parser(COM_MP_TO_SV * mp_sv, COM_SV_TO_MP * sv_mp);
void set_send_status_data(COM_SV_TO_MP * sv_mp, COM_STATUS* com_st);
void send_status( COM_SV_TO_MP * sv_mp );

extern uint32_t SystemD2Clock;

/***********************
  関数
***********************/
void comm_mpsv_Init( void )
{

  // RS-485 initialize
  // Peripheral clock enable
  RCC->APB1LENR |= RCC_APB1LENR_UART4EN;
  // PD0     ------> AF8:UART4_RX
  // PD1     ------> AF8:UART4_TX
  // PB14    ------> AF8:UART4_DE
  GPIOD->AFR[0] = ( ( GPIOD->AFR[0] & ~GPIO_AFRL_AFSEL0_Msk ) | ( 8 << GPIO_AFRL_AFSEL0_Pos ) );
  GPIOD->MODER = ( ( GPIOD->MODER & ~GPIO_MODER_MODE0_Msk ) | ( 2 << GPIO_MODER_MODE0_Pos ) );
  GPIOD->PUPDR = ((uint32_t)(GPIOD->PUPDR) & ~GPIO_PUPDR_PUPD0_Msk) | (1 << GPIO_PUPDR_PUPD0_Pos);
  GPIOD->AFR[0] = ( ( GPIOD->AFR[0] & ~GPIO_AFRL_AFSEL1_Msk ) | ( 8 << GPIO_AFRL_AFSEL1_Pos ) );
  GPIOD->MODER = ( ( GPIOD->MODER & ~GPIO_MODER_MODE1_Msk ) | ( 2 << GPIO_MODER_MODE1_Pos ) );
  GPIOB->AFR[1] = ( ( GPIOB->AFR[1] & ~GPIO_AFRH_AFSEL14_Msk ) | ( 8 << GPIO_AFRH_AFSEL14_Pos ) );
  GPIOB->MODER = ( ( GPIOB->MODER & ~GPIO_MODER_MODE14_Msk ) | ( 2 << GPIO_MODER_MODE14_Pos ) );

  // baudrate : 3000000
  // parity   : none
  // word     : 8bit
  // stop     : 1bit
  // flow ctrl: none
  UART4->CR1 &= ~USART_CR1_UE;
  UART4->CR1 = USART_CR1_FIFOEN | USART_CR1_PEIE | USART_CR1_RXNEIE_RXFNEIE |  USART_CR1_TE | USART_CR1_RE;
  UART4->CR2 = 0;
  UART4->CR3 = USART_CR3_DEM | USART_CR3_EIE;
  UART4->GTPR = 0; // prescaller x1
  uint32_t usart_div = SystemD2Clock / 2 / RS485_BAUDRATE;
  UART4->BRR = (usart_div & (USART_BRR_DIV_FRACTION_Msk|USART_BRR_DIV_MANTISSA_Msk)) << USART_BRR_DIV_FRACTION_Pos; //baudrate設定
  UART4->CR1 |= USART_CR1_UE;

  init_queue( &comm_receive_queue, uart_rx_buf, UART_RX_BUF_SIZE );
  init_receive_parser_buff_multi( &_com_st[0]);
  init_receive_parser_buff_multi( &_com_st[1]);
  _mp_sv_current_write = 0;
  _mp_sv_current_read = 1;
  _com_st_current_write = 0;
  _com_st_current_read = 1;
  state_mcn.status = MP_TO_SV;
  state_mcn.err_info_cnt = 0;
}

void init_queue( Queue* _q, uint8_t* _buf, uint32_t _size )
{
    if ( _q ) {
        _q->buf = _buf;
        _q->size = _size;
        _q->head = 0;
        _q->tail = 0;
    }
}

static void init_receive_parser_buff_multi(COM_STATUS * com_st)
{
    for (int i = 0; i < COMMAND_SIZE_MAX; i++) {
        com_st->buff[i] = 0x00;
    }
    com_st->direction = MP_TO_SV;
    com_st->command = 0x00;
    com_st->tar_id = 0x00;
    com_st->dest = 0x00;
    com_st->packet_len = 0x00;
    com_st->req_status = 0x00;
    for (int i = 0; i < MAX_SERVO_NUM; i++) {
        com_st->param_mode[i] = 0x00;
        com_st->force[i] = 0x0000;
        com_st->pos[i] = 0x0000;
    }

    com_st->status_id = 0x00;
    com_st->sending_sv_id = 0x00;
    com_st->reqed_status = 0x00;
    com_st->source_current = 0x0000;
    com_st->pos_hall = 0x0000;
    com_st->pos_mr = 0x0000;
    com_st->primary_status = 0x0000;
    com_st->reqed_status_value = 0x0000;

    com_st->crc = 0x00;
    com_st->crc_err = 0x00;
    com_st->receive_status = NO_RECEIVING;
    com_st->received_byte_num = 0;
}

inline static uint8_t is_usart_isr_error_detected( uint32_t isr )
{
    return ( ( isr & ( USART_ISR_ORE | USART_ISR_NE | USART_ISR_FE | USART_ISR_PE ) ) != 0 );
}

inline static uint8_t is_usart_isr_rx_empty( uint32_t isr )
{
    uint8_t ret = 0;
    uint32_t temp = isr & USART_ISR_RXNE_RXFNE;
    if (temp == USART_ISR_RXNE_RXFNE) {     //RXNEフラグが立っているのでRXは空ではないのでゼロ
        ret = 0;
    } else {            // RXNEフラグが立ってないのでRX空なので1を返す
        ret = 1;
    }
    return ret;
}

inline static uint8_t is_usart_txFIFO_full( uint32_t isr )
{
    uint8_t ret = 0;
    uint32_t temp = isr & USART_ISR_TXE_TXFNF;  // TXFNF:TXFIFOノットフル. 1ならばフルではないのでTDRに書き込める
    if (temp == USART_ISR_TXE_TXFNF) {     //TXNEフラグが立っているのでTXは空ではないのでゼロ
        ret = 0;
    } else {            // TXNEフラグが立ってないのでRX空なので1を返す
        ret = 1;
    }
    return ret;
}

uint8_t is_transmit_completed( uint32_t isr )
{
    return ((isr & USART_ISR_TC) != 0);
}

uint8_t is_queue_empty( const Queue* _q )
{
    return (_q->head == _q->tail);
}

uint8_t is_queue_full( const Queue* _q )
{
    uint8_t is_full = 0;
    // tailの次がheadだったらfullと判定。ただしtailがキューの最後だったら0に戻す。
    uint16_t temp_next_tail = _q->tail + 1;
    if (temp_next_tail == _q->size) {
        temp_next_tail = 0;
    }
    if (temp_next_tail == _q->head) {
        is_full = 1;
    } else {
        is_full = 0;
    }
    return is_full;
}

uint8_t enqueue_no_overwrite( Queue* _q, uint8_t val )
{
    // キューがいっぱいで無ければキューに格納。いっぱいの時は破棄。
    if ( !is_queue_full(_q) ) {
        _q->buf[_q->tail] = val;
        // tailを更新する。キューサイズを超えたら0も戻す。(リングバッファ)
        _q->tail++;
        if (_q->tail == _q->size) {
            _q->tail = 0;
        }
        return 1;
    }
    return 0;
}

uint8_t dequeue( Queue* _q, uint8_t* val )
{
    if ( val && !is_queue_empty(_q) ) {
        *val = _q->buf[_q->head];
        // headを更新する。キューサイズを超えたら0も戻す。(リングバッファ)
        _q->head++;
        if (_q->head == _q->size) {
            _q->head = 0;
        }
        return 1;
    }
    return 0;
}

static void USARTx_ISR_error_detected( uint32_t error_register )
{
    USART_TypeDef *UARTx = RS485_UART;
    if (error_register & USART_ISR_FE) {
        usart_error_count.framing_error++;
    }
    if (error_register & USART_ISR_PE) {
        usart_error_count.parity_error++;
    }
    if (error_register & USART_ISR_ORE) {
        usart_error_count.overrun_error++;

        // Receive control disable
        NVIC_DisableIRQ(RS485_IRQn);
        UARTx->CR1 &= (~USART_CR1_RE);      // Receive disable

        // Receive FIFO clear ( RFCLR )
        UARTx->RQR |= USART_RQR_RXFRQ;

        // 割り込みフラグをクリアする
        UARTx->ICR |= USART_ICR_ALL;

        // Receive control enable
        UARTx->CR1 |= USART_CR1_RE;    // Receive enable

        uint16_t timeout_count = 0;
        const uint16_t TIMEOUT_COUNT_MAX = 100;
        while (0 == (( UARTx->ISR ) & USART_ISR_REACK)) {  // RE wait
            if ( timeout_count++ >= TIMEOUT_COUNT_MAX ) {
                break;
            }
        }
        NVIC_EnableIRQ(RS485_IRQn);
    }
    usart_error_count.is_error = 1;
}

/********************************************************
  上流通信のUARTのRX FIFOにデータが入るとかかる割り込み処理
********************************************************/
void MP_COMM_IrqHandler(void) {

    uint32_t isr = 0;
    uint8_t is_received = 0;
    uint8_t is_error = 0;
    uint32_t timeout_count = 0;
    const uint32_t TIMEOUT_COUNT_MAX = 1000;

    while ( timeout_count++ < TIMEOUT_COUNT_MAX ) {
        isr = RS485_UART->ISR;

        // 割り込みフラグをクリアする
        RS485_UART->ICR |= USART_ICR_ALL;

        if ( (is_error = is_usart_isr_error_detected(isr)) ) {
            break;
        }

        //RXの受信が無い(RXNEフラグが立っていない)ならば抜ける
        if ( is_usart_isr_rx_empty(isr) ) {
            break;
        }
        //RXに受信しているのでキューに積む
        enqueue_no_overwrite( &comm_receive_queue, (uint8_t)(RS485_UART->RDR & 0xff) );//読めてない？
        RS485_UART->RQR |= USART_RQR_TXFRQ;
        is_received = 1;
    }
    // error detected
    if ( is_error ) {
        USARTx_ISR_error_detected( isr );
    }
    if ( is_received ) {
        // 拡張割り込みコントローラー(EXTI)でソフト割り込みを発生させる
        EXTI->SWIER1 |= EXTI_SWIER1_SWIER0;
    }
}


inline static PACKET_RECEIVE_STATUS next_receive_status_multi( const COM_STATUS* com_st )
{
    if ( com_st->crc_err ) {
        return RECEIVE_FAILED;
    }
    PACKET_RECEIVE_STATUS current_status = com_st->receive_status;
    switch ( current_status ) {
        case NO_RECEIVING:
            if ( com_st->received_byte_num > 0 ) {
                return HEADER_RECEIVING;
            }
            return current_status;
        case HEADER_RECEIVING:
            if ( com_st->received_byte_num > HEADER_SIZE ) {
                return BODY_RECEIVING;
            }
            return current_status;
        case BODY_RECEIVING:
            if ( com_st->received_byte_num >= com_st->packet_len ) {
                return RECEIVE_COMPLETED;
            }
            return current_status;
        case RECEIVE_COMPLETED:
            return current_status;
        case RECEIVE_ABORTED:
            return current_status;
        case RECEIVE_FAILED:
            if ( com_st->received_byte_num > 0 ) {
                // try header receive again
                return HEADER_RECEIVING;
            }
            return current_status;
        default:
            return current_status;
    }
}

inline static PACKET_RECEIVE_STATUS receive_byte_multi( COM_STATUS* com_st )
{
    if ( (com_st->receive_status == RECEIVE_COMPLETED) || (com_st->receive_status == RECEIVE_ABORTED) || (com_st->receive_status == RECEIVE_FAILED) ) {
        // reset
        init_receive_parser_buff_multi( com_st );
    }

    com_st->received_byte_num += dequeue( &comm_receive_queue, &com_st->buff[com_st->received_byte_num] );
    return ( com_st->receive_status = next_receive_status_multi( com_st ) );
}

COM_MP_TO_SV * get_send_mode_handle()
{
    return &_mp_sv_mode;
}

COM_MP_TO_SV * get_send_cmd_handle()
{
    return &_mp_sv_cmd;
}

COM_SV_TO_MP * get_status_handle()
{
    return &_sv_mp;
}

COM_STATUS * get_com_status_handle()
{
    return &_com_st[_com_st_current_read];
}

COM_STATUS * get_com_status_ura_handle()
{
    if (_com_st_current_read == 0) {
        return &_com_st[1];
    } else {
        return &_com_st[0];
    }
}

STATE_MACHINE * get_com_state_machine()
{
    return &state_mcn;
}

inline static void check_command_id( COM_MP_TO_SV* mp_sv )
{
    if (   mp_sv->command != MODE_CHANGE_ALL
        && mp_sv->command != MODE_CHANGE_TID
        && mp_sv->command != FORCE_CNTRL_ALL
        && mp_sv->command != FORCE_CNTRL_TID
        && mp_sv->command != ACC_CNTRL_ALL
        && mp_sv->command != ACC_CNTRL_TID
        && mp_sv->command != POS_CNTRL_ALL
        && mp_sv->command != POS_CNTRL_TID ) {
        // command id が不正なので受信abort
        mp_sv->receive_status = RECEIVE_ABORTED;
    }
}

inline static void parse_command_header( COM_MP_TO_SV* mp_sv )
{
    if ( mp_sv->received_byte_num == HEADER_COMMAND_ID + 1 ) {
        mp_sv->command = mp_sv->buff[HEADER_COMMAND_ID];
        check_command_id( mp_sv );
    }
    else if ( mp_sv->received_byte_num == HEADER_TARGET_ID + 1 ) {
        mp_sv->tar_id = mp_sv->buff[HEADER_TARGET_ID];
        // TODO: この時点で、tar_idが自分と違う場合はreceive_statusを"SKIP"とかに設定して
        //       以後の受信はpacket_len分飛ばす、という特殊処理に分岐できると良い
    }
    else if ( mp_sv->received_byte_num == HEADER_PACKET_LEN + 1 ) {
        mp_sv->packet_len = mp_sv->buff[HEADER_PACKET_LEN];
    }
    else if ( mp_sv->received_byte_num == HEADER_REQ_STATUS + 1 ) {
        mp_sv->req_status = mp_sv->buff[HEADER_REQ_STATUS];
    }
    else {
        // all header should be received and parsed
        return;
    }
}

void check_com_direction( COM_STATUS* com_st )
{
    uint8_t _data = com_st->buff[HEADER_COMMAND_ID];
    if ( _data == SV_STATUS_ID ) {
        com_st->direction = SV_TO_MP;
        com_st->status_id = _data;
    } else if (_data == MODE_CHANGE_ALL
            || _data == MODE_CHANGE_TID
            || _data == FORCE_CNTRL_ALL
            || _data == FORCE_CNTRL_TID
            || _data == ACC_CNTRL_ALL
            || _data == ACC_CNTRL_TID
            || _data == POS_CNTRL_ALL
            || _data == POS_CNTRL_TID ) {
        com_st->direction = MP_TO_SV;
        com_st->command = _data;
    } else {
        // command id が不正なので受信abort
        com_st->receive_status = RECEIVE_ABORTED;
    }
}

inline static void parse_command_header_multi( COM_STATUS* com_st )
{
    if ( com_st->received_byte_num == HEADER_COMMAND_ID + 1 ) {
        check_com_direction(com_st);
    }
    else if ( com_st->received_byte_num == HEADER_TARGET_ID + 1 ) {
        if ( com_st->direction == MP_TO_SV ) {
            com_st->tar_id = com_st->buff[HEADER_TARGET_ID];
            if (com_st->tar_id == BROAD) {
                com_st->dest = BROAD;
            } else {
                com_st->dest = UNI;
            }
        // TODO: この時点で、tar_idが自分と違う場合はreceive_statusを"SKIP"とかに設定して
        //       以後の受信はpacket_len分飛ばす、という特殊処理に分岐できると良い
        } else {
            com_st->sending_sv_id = com_st->buff[HEADER_MY_SV_ID];
        }
    }
    else if ( com_st->received_byte_num == HEADER_PACKET_LEN + 1 ) {
        com_st->packet_len = com_st->buff[HEADER_PACKET_LEN];
    }
    else if ( com_st->received_byte_num == HEADER_REQ_STATUS + 1 ) {
        if ( com_st->direction == MP_TO_SV ) {
            com_st->req_status = com_st->buff[HEADER_REQ_STATUS];
        } else {
            com_st->reqed_status = com_st->buff[HEADER_REQUESTED_STATUS];
        }
    }
    else {
        // all header should be received and parsed
        return;
    }
}

inline static void parse_2byte_param( const uint8_t* raw, int16_t* parsed, uint8_t raw_idx )
{
    if ( ((raw_idx-HEADER_SIZE) & 1) == 0 ) {
        // even number -> should be higher byte, received former
        *parsed = (int16_t)((uint16_t)(raw[raw_idx] << 8));
    }
    else {
        // odd number -> should be lower byte, received later
        *parsed = (int16_t)((uint16_t)*parsed | (uint16_t)raw[raw_idx]);
    }
}

inline static void parse_command_body_multi( COM_STATUS* com_st )
{
    if ( com_st->received_byte_num >= com_st->packet_len ) {
        // all body (except CRC) shoud be received and parsed
        return;
    }

    // TODO: tar_idが自分と違う場合はpacket_len分読み飛ばせるようにしたい

    if ( com_st->received_byte_num <= HEADER_SIZE ) {
        // no body byte comming yet
        return;
    }

    uint8_t param_byte_idx = com_st->received_byte_num-1;

    if ( com_st->direction == MP_TO_SV ) {
        switch ( com_st->command ) {
            case MODE_CHANGE_ALL:
            case MODE_CHANGE_TID:
                com_st->param_mode[(param_byte_idx-HEADER_SIZE)] = com_st->buff[param_byte_idx];
                break;
            case FORCE_CNTRL_ALL:
            case FORCE_CNTRL_TID:
                {   // C言語ではswitch文の中で変数宣言を行うとエラーになる。対策として{}で括る
                    // 2byte毎にidxを進めたい。param_byte_idxが4,5のとき0   6,7のとき1  8,9のとき2
                    uint8_t idx = (param_byte_idx - HEADER_SIZE) >> 1;
                    parse_2byte_param( com_st->buff, com_st->force + idx, param_byte_idx );
                }
                break;
            case ACC_CNTRL_ALL:
            case ACC_CNTRL_TID:
                {   // C言語ではswitch文の中で変数宣言を行うとエラーになる。対策として{}で括る
                    // 2byte毎にidxを進めたい。param_byte_idxが4,5のとき0   6,7のとき1  8,9のとき2
                    uint8_t idx = (param_byte_idx - HEADER_SIZE) >> 1;
                    parse_2byte_param( com_st->buff, com_st->force + idx, param_byte_idx );
                }
                break;
            case POS_CNTRL_ALL:
            case POS_CNTRL_TID:
                {
                    uint8_t idx = (param_byte_idx - HEADER_SIZE) >> 1;
                    parse_2byte_param( com_st->buff, com_st->pos + idx, param_byte_idx );
                }
                break;
            default:
                com_st->receive_status = RECEIVE_FAILED;
                break;
        }
    } else if ( com_st->direction == SV_TO_MP ) {
        switch( param_byte_idx ) {
            case BODY_SOURCE_CURRENT:
                com_st->source_current = com_st->buff[param_byte_idx];
                break;
            case BODY_IQ_CURRENT_L:
            case BODY_IQ_CURRENT_H:
                //parse_2byte_param( com_st->buff, &com_st->iq_current, param_byte_idx);
                com_st->iq_current = (int16_t)(com_st->buff[param_byte_idx] + (uint16_t)(com_st->buff[BODY_IQ_CURRENT_L] <<8));//HL逆?
                break;
            case BODY_POS_HALL_L:
            case BODY_POS_HALL_H:
                //parse_2byte_param( com_st->buff, &com_st->pos_hall, param_byte_idx);
                com_st->pos_hall = (int16_t)(com_st->buff[param_byte_idx] + (uint16_t)(com_st->buff[BODY_POS_HALL_L] << 8));//HL逆?
                break;
            case BODY_POS_MR_L:
            case BODY_POS_MR_H:
                //parse_2byte_param( com_st->buff, &com_st->pos_mr, param_byte_idx);
                com_st->pos_mr = (int16_t)(com_st->buff[param_byte_idx] + (uint16_t)(com_st->buff[BODY_POS_MR_L] << 8));//HL逆?
                break;
            case BODY_PRIMARY_STATUS_L:
            case BODY_PRIMARY_STATUS_H:
                //parse_2byte_param( com_st->buff, &com_st->primary_status, param_byte_idx);
                com_st->primary_status= (int16_t)(com_st->buff[param_byte_idx] + (uint16_t)(com_st->buff[BODY_PRIMARY_STATUS_L] << 8));//HL逆?
                break;
            case BODY_REQED_STATUS_VALUE_L:
            case BODY_REQED_STATUS_VALUE_H:
                //parse_2byte_param( com_st->buff, &com_st->reqed_status_value, param_byte_idx);
                com_st->reqed_status_value = (int16_t)(com_st->buff[param_byte_idx] + (uint16_t)(com_st->buff[BODY_REQED_STATUS_VALUE_L] << 8));//HL逆?
                break;
            default:
                com_st->receive_status = RECEIVE_FAILED;
                break;
        }
    }
}

inline static void parse_command_crc( COM_MP_TO_SV* mp_sv )
{
    mp_sv->crc_err = 0x01;
    if ( mp_sv->received_byte_num != mp_sv->packet_len ) {
        // something is wrong
        return;
    }
    mp_sv->crc = mp_sv->buff[mp_sv->received_byte_num-1];

    if ( mp_sv->crc == calc_crc( mp_sv->buff, mp_sv->packet_len-1 ) ) {
        mp_sv->crc_err = 0x00;
    }
}

inline static void parse_command_crc_multi( COM_STATUS* com_st )
{
    com_st->crc_err = 0x01;
    if ( com_st->received_byte_num != com_st->packet_len ) {
        // something is wrong
        return;
    }
    com_st->crc = com_st->buff[com_st->received_byte_num-1];

    if ( com_st->crc == calc_crc( com_st->buff, com_st->packet_len-1 ) ) {
        com_st->crc_err = 0x00;
    }
}


/**************************************
 バスに接続されたSV通信の受信対応に拡張
***************************************/
void receive_command_parse_multi(void)
{
    while ( !is_queue_empty( &comm_receive_queue ) ) {
        switch ( receive_byte_multi( &_com_st[_com_st_current_write] ) ) {
            case NO_RECEIVING:
                break;
            case HEADER_RECEIVING:
                parse_command_header_multi( &_com_st[_com_st_current_write] );
                break;
            case BODY_RECEIVING:
                parse_command_body_multi( &_com_st[_com_st_current_write] );
                break;
            case RECEIVE_COMPLETED:
                parse_command_crc_multi( &_com_st[_com_st_current_write] );
                break;
            case RECEIVE_ABORTED:
                // 間に合ってないかも
                return;  // 以後の処理を中断する
            case RECEIVE_FAILED:
            default:
                break;
        }

        if ( _com_st[_com_st_current_write].receive_status == RECEIVE_COMPLETED ) {
            // CRCエラーが出ていたら失敗扱い
            if ( _com_st[_com_st_current_write].crc_err ) {
                _com_st[_com_st_current_write].receive_status = RECEIVE_FAILED;
                _com_st[_com_st_current_write].receive_buff_remained = 0x00;
            }
            else {
                _com_st[_com_st_current_write].receive_buff_remained = 0x01;
            }

            // 先のparse結果の引き取りがされていれば面を入れ替え
            if ( !(_com_st[_com_st_current_read].receive_buff_remained) ) {
                _com_st_current_write = 1-_com_st_current_write;
                _com_st_current_read = 1-_com_st_current_read;
            }
            else {
                // 引き取りが間に合ってないので次も同じ面に書く
            }
        }
    }
}


void parser_multi_mp_sv(COM_STATUS * com_st)
{
    //とりあえず0
    com_st->receive_buff_remained = 0;
}

uint16_t dbg_cnt[12] = {0};
void parser_multi_sv_mp(COM_STATUS * com_st)
{
    uint8_t sv_id=0;
    // SVのIDに応じて処理変更する
    switch(com_st->sending_sv_id){
        case 0x01://SV1
            sv_id = SV2_FL;
            dbg_cnt[0]++;
            sv_data[0].cnt++;
            break;
#if (COMM_SV_NUM >= 2)
        case 0x02://SV2
            sv_id = SV1_FR;
            dbg_cnt[1]++;
            sv_data[1].cnt++;
            break;
#endif
#if (COMM_SV_NUM >= 3)
        case 0x03://SV3
            sv_id = SV3_HR;
            dbg_cnt[2]++;
            sv_data[2].cnt++;
            break;
#endif
#if (COMM_SV_NUM >= 4)
        case 0x04://SV4
            sv_id = SV4_HL;
            dbg_cnt[3]++;
            sv_data[3].cnt++;
            break;
#endif
        default:
            break;
    }

    //q軸電流
    //16bit -25A~+25A
    //link[link_id].current = (float)com_st->iq_current * 25.0f / 32767.0f;
    //20240521現在、これはsrc電流が返ってきている。
    sv_data[sv_id].iq = com_st->iq_current;

    //MR角度
    //16bit 360deg
    //link[link_id].q_mr = (float)com_st->pos_mr * 180.0f / 32767.0f;
    sv_data[sv_id].mr = com_st->pos_mr;

    //HALL角度
    //16bit 360deg
    //link[link_id].q_hall = (float)com_st->pos_hall * 180.0f / 32767.0f;
    //20240521現在、これは機械角速度(response)が返ってきている[Hz]。
    sv_data[sv_id].hall = com_st->pos_hall;

    //温度
    //20240521現在、これは機械角速度(command)が返ってきている[Hz]。
    sv_data[sv_id].req = com_st->reqed_status_value;

    //電源電圧(電源電流箇所代用)
    //20240521現在、これは電源電圧が返ってきている[0.1V]
    sv_data[sv_id].vdc = com_st->source_current;

    //primary status
    sv_data[sv_id].current_state = (com_st->primary_status & 3);

    //crcエラー
    sv_data[sv_id].crc_err = com_st->crc_err;

    //0にする
    com_st->receive_buff_remained = 0;
}

void parser_multi(COM_STATUS * com_st)
{
    int16_t max_count_crc_err = 10;
    static int16_t count_continuous_crc_err = 0;

    if (com_st->crc_err != 1) {
        if (com_st->direction == MP_TO_SV) {
            parser_multi_mp_sv(com_st);
        } else {
            parser_multi_sv_mp(com_st);
        }
        count_continuous_crc_err = 0;
    } else {
        // CRCエラー起きたけどバッファを空にする(次の受信をするため)
        com_st->receive_buff_remained = 0;
        count_continuous_crc_err ++;
    }

    state_mcn.err_info_cnt = count_continuous_crc_err;
    //エラーが連続したら落とす
    if(count_continuous_crc_err >= max_count_crc_err){
        //mc_state.state = MC_STATE_EMERGENCY;
    }
}

void pack_2Byte_data(uint8_t * buff, int16_t _data)
{
    if (_data < 0) {
        //_data = SMC_F16 + _data;
        _data = 65536 + _data;
    }
    buff[1] = _data % 256;
    buff[0] = (_data - buff[1]) >> 8;
}

void send_mode_broadcast_data(COM_MP_TO_SV * mp_sv, uint8_t sv_num, uint8_t mode[], uint8_t req_status)
{
    //モードコマンド設定
    mp_sv->command = 0x01;
    mp_sv->tar_id = 0x00;
    mp_sv->packet_len = 0x05 + sv_num;
    mp_sv->req_status = req_status;
    for(int i = 0; i < sv_num; i++){
        mp_sv->param_mode[i] = mode[i];
    }

    //送信バッファに詰める
    mp_sv->buff[HEADER_COMMAND_ID] = mp_sv->command;
    mp_sv->buff[HEADER_TARGET_ID] = mp_sv->tar_id;
    mp_sv->buff[HEADER_PACKET_LEN] = mp_sv->packet_len;
    mp_sv->buff[HEADER_REQ_STATUS] = mp_sv->req_status;
    for(int i = 0; i < sv_num; i++){
        mp_sv->buff[HEADER_REQ_STATUS + 1 + i] = mode[i];
    }

    mp_sv->crc = calc_crc(mp_sv->buff, 4 + sv_num + 1 - 1);
    mp_sv->buff[HEADER_REQ_STATUS + sv_num + 1] = mp_sv->crc;


    //送信
    USART_TypeDef *UARTx = RS485_UART;
    uint32_t timeout_count = 0;
    const uint32_t TIMEOUT_COUNT_MAX = 10000;

    // TE (Transmit Enable)をセットすることでidleフレームを送信
    UARTx->CR1 |= USART_CR1_TE;

    // データ長だけTDRに書き込む。データはFIFOに積まれる。
    for (int i = 0; i < 5 + sv_num; i++) {
        // while (UART_FLAG_TXFNF != ((UARTx->ISR) & UART_FLAG_TXFNF)) {
        while ( is_usart_txFIFO_full(UARTx->ISR) ) {
            timeout_count++;
            if ( timeout_count > TIMEOUT_COUNT_MAX ) {
                timeout_count = 0;
                break;
            }
        }
        UARTx->TDR = mp_sv->buff[i];
    }

    // 全てのデータをTDRに書いたらTC (Transmit Complete)が1になるまで待つ
    timeout_count = 0;
    while (!is_transmit_completed(UARTx->ISR)) {
        timeout_count++;
        if ( timeout_count > TIMEOUT_COUNT_MAX) {
            break;
        }
    }

    // TE をクリアすることで送信を停止
    UARTx->CR1 &= (~USART_CR1_TE);
}

void send_cmd_broadcast_data(COM_MP_TO_SV * mp_sv, uint8_t cmd, uint8_t sv_num, int16_t param[], uint8_t req_status)
{
    //制御コマンド設定
    mp_sv->command = cmd;
    mp_sv->tar_id = 0x00;
    mp_sv->packet_len = 0x05 + sv_num * 2;
    mp_sv->req_status = req_status;
    for(int i = 0; i < sv_num; i++){
        if(cmd == FORCE_CNTRL_ALL){
            mp_sv->force[i] = param[i];
        }else if(cmd == POS_CNTRL_ALL){
            mp_sv->pos[i] = param[i];
        }else if(cmd == VEL_CNTRL_ALL){
            mp_sv->vel[i] = param[i];
        }else if(cmd == ACC_CNTRL_ALL){
            mp_sv->force[i] = param[i];
        }
    }

    //送信バッファに詰める
    mp_sv->buff[HEADER_COMMAND_ID] = mp_sv->command;
    mp_sv->buff[HEADER_TARGET_ID] = mp_sv->tar_id;
    mp_sv->buff[HEADER_PACKET_LEN] = mp_sv->packet_len;
    mp_sv->buff[HEADER_REQ_STATUS] = mp_sv->req_status;
    for(int i = 0; i < sv_num; i++){
        if(cmd == FORCE_CNTRL_ALL){
            pack_2Byte_data(&mp_sv->buff[HEADER_REQ_STATUS + 1 + 2 * i], mp_sv->force[i]);
        }else if(cmd == POS_CNTRL_ALL){
            pack_2Byte_data(&mp_sv->buff[HEADER_REQ_STATUS + 1 + 2 * i], mp_sv->pos[i]);
        }else if(cmd == VEL_CNTRL_ALL){
            pack_2Byte_data(&mp_sv->buff[HEADER_REQ_STATUS + 1 + 2 * i], mp_sv->vel[i]);
        }else if(cmd == ACC_CNTRL_ALL){
            pack_2Byte_data(&mp_sv->buff[HEADER_REQ_STATUS + 1 + 2 * i], mp_sv->force[i]);
        }
    }

    mp_sv->crc = calc_crc(mp_sv->buff, 4 + sv_num * 2 + 1 - 1);
    mp_sv->buff[HEADER_REQ_STATUS + sv_num * 2 + 1] = mp_sv->crc;


    //送信
    USART_TypeDef *UARTx = RS485_UART;
    uint32_t timeout_count = 0;
    const uint32_t TIMEOUT_COUNT_MAX = 10000;

    // TE (Transmit Enable)をセットすることでidleフレームを送信
    UARTx->CR1 |= USART_CR1_TE;

    // データ長だけTDRに書き込む。データはFIFOに積まれる。
    for (int i = 0; i < 5 + sv_num * 2; i++) {
        // while (UART_FLAG_TXFNF != ((UARTx->ISR) & UART_FLAG_TXFNF)) {
        while ( is_usart_txFIFO_full(UARTx->ISR) ) {
            timeout_count++;
            if ( timeout_count > TIMEOUT_COUNT_MAX ) {
                timeout_count = 0;
                break;
            }
        }
        UARTx->TDR = mp_sv->buff[i];
    }

    // 全てのデータをTDRに書いたらTC (Transmit Complete)が1になるまで待つ
    timeout_count = 0;
    while (!is_transmit_completed(UARTx->ISR)) {
        timeout_count++;
        if ( timeout_count > TIMEOUT_COUNT_MAX) {
            break;
        }
    }

    // TE をクリアすることで送信を停止
    UARTx->CR1 &= (~USART_CR1_TE);
}

void send_status( COM_SV_TO_MP * sv_mp )
{
    USART_TypeDef *UARTx = RS485_UART;
    uint32_t timeout_count = 0;
    const uint32_t TIMEOUT_COUNT_MAX = 10000;

    // TE (Transmit Enable)をセットすることでidleフレームを送信
    UARTx->CR1 |= USART_CR1_TE;

    // データ長だけTDRに書き込む。データはFIFOに積まれる。
    for (int i = 0; i < SEND_STATUS_LEN; i++) {
        // while (UART_FLAG_TXFNF != ((UARTx->ISR) & UART_FLAG_TXFNF)) {
        while ( is_usart_txFIFO_full(UARTx->ISR) ) {
            timeout_count++;
            if ( timeout_count > TIMEOUT_COUNT_MAX ) {
                timeout_count = 0;
                break;
            }
        }
        UARTx->TDR = sv_mp->buff[i];
    }

    // 全てのデータをTDRに書いたらTC (Transmit Complete)が1になるまで待つ
    timeout_count = 0;
    while (!is_transmit_completed(UARTx->ISR)) {
        timeout_count++;
        if ( timeout_count > TIMEOUT_COUNT_MAX) {
            break;
        }
    }

    sv_mp->send_buff_remained = 0;

    // TE をクリアすることで送信を停止
    UARTx->CR1 &= (~USART_CR1_TE);
}


/****************************************************************
  上流通信でRXを受信後にパースするための低優先度のソフト割り込み処理
****************************************************************/
COM_STATUS dbg_com[3];
void MP_COMM_SOFT_IrqHandler(void)
{

    // Clear EXTI0 interrupt request
    EXTI->PR1 |= EXTI_PR1_PR0;

    // 低優先度のソフト割り込みの中で受信バッファから値を抽出
    // 受信完了したらreceive_buff_remainedをセット.Readのreceive_buff_remainedがクリアされていれば面を入れ替え
    receive_command_parse_multi();

    COM_STATUS * com_st = get_com_status_handle();          // read
    
    // static int count = 0;
    // for(int j = 0; j < 16; j++){
    //     dbg_com[count].buff[j] = com_st->buff[j];
    // }
    // count++;
    // if(count > 2){
    //     count = 0;
    // }

    // 低優先度のソフト割り込みの中でパース処理
    if (com_st->receive_buff_remained == 1) {
        parser_multi(com_st);
    }
}
