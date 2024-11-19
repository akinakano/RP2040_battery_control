#include <stm32h747xx.h>
#include "comm_mpsv.h"
#include "motion_controller.h"
#include "crc8.h"
#include "HW_type.h"
#include "rs485.h"
#include "gpio.h"

SV_Comm_t sv_data[COMM_SV_NUM];

/***********************
  プライベート変数
***********************/
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
static void init_receive_parser_buff_multi(COM_STATUS * com_st);
static void receive_timeout_callback(void);
static void receive_command_parse_multi(void);
static void parser_multi(COM_STATUS * com_st);

void comm_mpsv_Init( void ) {

  rs485_Init();
  rs485_register_receive_timeout_callback(&receive_timeout_callback);

  init_receive_parser_buff_multi( &_com_st[0]);
  init_receive_parser_buff_multi( &_com_st[1]);
  _com_st_current_write = 0;
  _com_st_current_read = 1;
  state_mcn.status = MP_TO_SV;
  state_mcn.err_info_cnt = 0;
}

static void receive_timeout_callback(void) {

  TEST_PIN(1);
    receive_command_parse_multi();
    COM_STATUS * com_st = get_com_status_handle();          // read
    if(com_st->receive_buff_remained == 1) parser_multi(com_st);
  TEST_PIN(0);
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

    int d = rs485_readByte();
    if(d >= 0) com_st->buff[com_st->received_byte_num++] = d;
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
static void receive_command_parse_multi(void)
{
    while (rs485_receivedLength()) {
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

static void parser_multi(COM_STATUS * com_st)
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


    rs485_send(mp_sv->buff, 5 + sv_num);
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

    rs485_send(mp_sv->buff, 5 + sv_num);
}
