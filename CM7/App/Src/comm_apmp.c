#include    <stdio.h>
#include    <ctype.h>
#include    <stdarg.h>
#include    <stm32h747xx.h>
#include    "comm_apmp.h"
#include    "tim.h"
#include    "util.h"
#include    "gpio.h"
#include    "motion_controller.h"
#include    "HW_type.h"
#include    "comm_battery.h"
extern struct BatteryStatus_st BatteryStatus;

#define     ABS_I(x)    (x < 0 ? (-x) :(x))

// 送受信バッファ（リングバッファ）(デバッグ通信用)
#define RX_BUFF_SIZE 1024
static volatile char  rx_buff[RX_BUFF_SIZE];
static volatile uint32_t rx_tail = 0;  // 受信バッファTailポインタ（格納ポインタ）
static          uint32_t rx_head = 0;  // 受信バッファHeadポインタ（取り出しポインタ）

#define TX_BUFF_SIZE 256
static          char tx_buff[TX_BUFF_SIZE];
static          uint32_t tx_tail = 0;  // 送信バッファTailポインタ（格納ポインタ）
static volatile uint32_t tx_head = 0;  // 送信バッファHeadポインタ（取り出しポインタ）

static uint8_t calc_crc(uint8_t *ptr, uint8_t len);

void Comm_APMP_Rx_IrqHandler(){
    //更新割込み処理終了
    if(TIM2->SR & TIM_SR_UIF){
        TIM2->SR &= ~TIM_SR_UIF;
    }

    comm_apmp_rx_handler();
}


Comm_APMP_Receive apmp_data = {
    .cmd = 0,
    .crc = 0,
    .head_h = 0,
    .head_l = 0,
    .vx_cmd_h = 0x80,
    .vx_cmd_l = 0x00,
    .vy_cmd_h = 0x80,
    .vy_cmd_l = 0x00,
    .wz_cmd_h = 0x80,
    .wz_cmd_l = 0x00,
};
static uint8_t find_head_flag = 0;
static uint16_t byte_count = 0;
uint8_t apmp_buff[COMM_APMP_SIZE];
uint8_t mpap_buff[COMM_MPAP_SIZE];
uint16_t state_mp_to_ap = STATE_MP_TO_AP_NORMAL;
uint16_t dbg_flag_mpap;

void Comm_APMP_Parse_IrqHandler(){
    //PA1_ON();
    //更新割込み処理終了
    if(TIM12->SR & TIM_SR_UIF){
        TIM12->SR &= ~TIM_SR_UIF;
    }

    int32_t c;
    static int32_t buff_z1;
//    static uint16_t error_count=0;

    if(comm_apmp_read(&c) == 0){
        return;
    }

    //ヘッダを探す
    if(buff_z1 == COMM_APMP_HEADER_H && c == COMM_APMP_HEADER_L){
        find_head_flag = 1;
        apmp_buff[0] = buff_z1;
        byte_count = 1;
    }
    buff_z1 = c;

    //ヘッダが見つかっていたらデータを詰める
    if(find_head_flag){
        apmp_buff[byte_count] = c;
        byte_count++;
    }

    //全部受信終了したらもどす　＆　詰めかえる
    if(byte_count == COMM_APMP_SIZE){
        byte_count = 0;
        find_head_flag = 0;
        if(calc_crc(apmp_buff, COMM_APMP_SIZE-1) == apmp_buff[COMM_APMP_SIZE-1]){
            apmp_data.head_h = apmp_buff[0];
            apmp_data.head_l = apmp_buff[1];
            apmp_data.cmd = apmp_buff[2];
            apmp_data.vx_cmd_h = apmp_buff[3];
            apmp_data.vx_cmd_l = apmp_buff[4];
            apmp_data.vy_cmd_h = apmp_buff[5];
            apmp_data.vy_cmd_l = apmp_buff[6];
            apmp_data.wz_cmd_h = apmp_buff[7];
            apmp_data.wz_cmd_l = apmp_buff[8];
#if 0
            apmp_data.wheel_radius_right_h = apmp_buff[9];
            apmp_data.wheel_radius_right_l = apmp_buff[10];
            apmp_data.wheel_radius_left_h = apmp_buff[11];
            apmp_data.wheel_radius_left_l = apmp_buff[12];
            apmp_data.tread_h = apmp_buff[13];
            apmp_data.tread_l = apmp_buff[14];
            apmp_data.imu_gyro_scale_gain_h = apmp_buff[15];
            apmp_data.imu_gyro_scale_gain_l = apmp_buff[16];
#else
            apmp_data.wheel_radius_right = (apmp_buff[9]  << 8) | apmp_buff[10];
            apmp_data.wheel_radius_left  = (apmp_buff[11] << 8) | apmp_buff[12];
            apmp_data.tread = (apmp_buff[13] << 8) | apmp_buff[14];
            apmp_data.imu_gyro_scale_gain = (apmp_buff[15] << 8) | apmp_buff[16];
#endif
            apmp_data.crc = apmp_buff[COMM_APMP_SIZE-1];
        }
    }
    //PA1_OFF();
}


static uint8_t s_send_status = SEND_OFF_MP_TO_AP;
void Comm_APMP_Tx_IrqHandler(){
    //更新割込み処理終了
    if(TIM5->SR & TIM_SR_UIF){
        TIM5->SR &= ~TIM_SR_UIF;
    }

    if(s_send_status == SEND_ON_MP_TO_AP){
        send_status_mp_to_ap();
        s_send_status = SEND_OFF_MP_TO_AP;
    }

    //comm_apmp_tx_handler();
}

inline static uint8_t __is_usart_txFIFO_full( uint32_t isr )
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

uint8_t __is_transmit_completed( uint32_t isr )
{
    return ((isr & USART_ISR_TC) != 0);
}


void send_status_mp_to_ap(void){
    //uint8_t mpap_buff[COMM_MP_AP_SIZE]={0};
    //head
    //dbg_flag_mpap |= 0x01;
    mpap_buff[0] = 0xab;
    mpap_buff[1] = 0xcd;
    //time count
    mpap_buff[2] = (uint8_t)((MC_state.time_count & 0xFF00) >> 8);
    mpap_buff[3] = (uint8_t)(MC_state.time_count & 0x00FF);
    //vel_cmd
    //vx
    //uint16_t vx_cmd = (uint16_t)(COMM_DIR_X * BVC.vel_cmd[X_AXIS] * VXY_MPS_TO_INT16t + 32768);
    //uint16_t vx_cmd = (int16_t)(COMM_DIR_X * BVC.vel_cmd[X_AXIS] * VXY_MPS_TO_INT16t) + 32768;
    uint16_t vx_cmd = (int16_t)(COMM_DIR_X * BVC.vel_ref[X_AXIS] * VXY_MPS_TO_INT16t) + 32768;
    mpap_buff[4] = (uint8_t)((vx_cmd & 0xFF00) >> 8);
    mpap_buff[5] = (uint8_t)(vx_cmd & 0x00FF);
    //vy
    //uint16_t vy_cmd = (uint16_t)(COMM_DIR_Y * BVC.vel_cmd[Y_AXIS] * VXY_MPS_TO_INT16t + 32768);
    //uint16_t vy_cmd = (int16_t)(COMM_DIR_Y * BVC.vel_cmd[Y_AXIS] * VXY_MPS_TO_INT16t) + 32768;
    uint16_t vy_cmd = (int16_t)(COMM_DIR_Y * BVC.vel_ref[Y_AXIS] * VXY_MPS_TO_INT16t) + 32768;
    mpap_buff[6] = (uint8_t)((vy_cmd & 0xFF00) >> 8);
    mpap_buff[7] = (uint8_t)(vy_cmd & 0x00FF);
    //vz
    //uint16_t wz_cmd = (uint16_t)(COMM_DIR_Z * BVC.vel_cmd[Z_AXIS] * WZ_RADPS_TO_INT16t + 32768);
    //uint16_t wz_cmd = (int16_t)(COMM_DIR_Z * BVC.vel_cmd[Z_AXIS] * WZ_RADPS_TO_INT16t) + 32768;
    uint16_t wz_cmd = (int16_t)(COMM_DIR_Z * BVC.vel_ref[Z_AXIS] * WZ_RADPS_TO_INT16t) + 32768;
    mpap_buff[8] = (uint8_t)((wz_cmd & 0xFF00) >> 8);
    mpap_buff[9] = (uint8_t)(wz_cmd & 0x00FF);
    //vel_res
    //vx
    //uint16_t vx_res = (uint16_t)(COMM_DIR_X * BVC.vel_res[X_AXIS] * VXY_MPS_TO_INT16t + 32768);
    uint16_t vx_res = (int16_t)(COMM_DIR_X * BVC.vel_res[X_AXIS] * VXY_MPS_TO_INT16t) + 32768;
    mpap_buff[10] = (uint8_t)((vx_res & 0xFF00) >> 8);
    mpap_buff[11] = (uint8_t)(vx_res & 0x00FF);
    //vy
    //uint16_t vy_res = (uint16_t)(COMM_DIR_Y * BVC.vel_res[Y_AXIS] * VXY_MPS_TO_INT16t + 32768);
    uint16_t vy_res = (int16_t)(COMM_DIR_Y * BVC.vel_res[Y_AXIS] * VXY_MPS_TO_INT16t) + 32768;
    mpap_buff[12] = (uint8_t)((vy_res & 0xFF00) >> 8);
    mpap_buff[13] = (uint8_t)(vy_res & 0x00FF);
    //wz
    //uint16_t wz_res = (uint16_t)(COMM_DIR_Z * BVC.vel_res[Z_AXIS] * WZ_RADPS_TO_INT16t + 32768);
    uint16_t wz_res = (int16_t)(COMM_DIR_Z * BVC.vel_res[Z_AXIS] * WZ_RADPS_TO_INT16t) + 32768;
    mpap_buff[14] = (uint8_t)((wz_res & 0xFF00) >> 8);
    mpap_buff[15] = (uint8_t)(wz_res & 0x00FF);
    //odomet
    //px
    //uint32_t px_res = (uint32_t)(COMM_DIR_X * BVC.pos_res[X_AXIS] * PXY_M_TO_INT16t + 2147483648);
    uint32_t px_res = (int32_t)(COMM_DIR_X * BVC.pos_res[X_AXIS] * PXY_M_TO_INT16t) + 2147483648;
    mpap_buff[16] = (uint8_t)((px_res & 0xFF000000) >> 24);
    mpap_buff[17] = (uint8_t)((px_res & 0x00FF0000) >> 16);
    mpap_buff[18] = (uint8_t)((px_res & 0x0000FF00) >> 8);
    mpap_buff[19] = (uint8_t)((px_res & 0x000000FF));
    //py
    //uint32_t py_res = (uint32_t)(BVC.pos_res[Y_AXIS] * PXY_M_TO_INT16t + 2147483648);//オドメトリ計算で符号反転を合わせているので符号設定不要
    uint32_t py_res = (int32_t)(BVC.pos_res[Y_AXIS] * PXY_M_TO_INT16t) + 2147483648;//オドメトリ計算で符号反転を合わせているので符号設定不要
    mpap_buff[20] = (uint8_t)((py_res & 0xFF000000) >> 24);
    mpap_buff[21] = (uint8_t)((py_res & 0x00FF0000) >> 16);
    mpap_buff[22] = (uint8_t)((py_res & 0x0000FF00) >> 8);
    mpap_buff[23] = (uint8_t)((py_res & 0x000000FF));
    //qz
    uint16_t qz_res = (int16_t)(COMM_DIR_Z * BVC.pos_res[Z_AXIS] * QZ_RAD_TO_INT16t) + 32768;
    mpap_buff[24] = (uint8_t)((qz_res & 0xFF00) >> 8);
    mpap_buff[25] = (uint8_t)((qz_res & 0x00FF));
#ifdef BATTERY_INFO
    mpap_buff[26] = BatteryStatus.Capacity;
#else
    //vm
#if(HW_WHEEL == HW_WHEEL_MECANUM)
    float vm_ave_f = 0.25f * (sv_res[SV1_FR].vdc + sv_res[SV2_FL].vdc + sv_res[SV3_HR].vdc + sv_res[SV4_HL].vdc);
#elif(HW_WHEEL == HW_WHEEL_DIFF)
    float vm_ave_f = 0.5f * (sv_res[SV1_FR].vdc + sv_res[SV2_FL].vdc);
#endif
    uint8_t vm_ave = (uint8_t)(vm_ave_f);
    mpap_buff[26] = vm_ave;
#endif
    //err
    mpap_buff[27] = state_mp_to_ap;
    mpap_buff[27] |= BVC.BVC_debug_flag;
    BVC.BVC_debug_flag = 0;
    state_mp_to_ap = 0; //APに送ったらリセット
    //crc

    //IMU data
    //gyro x float
    uint32_t gyro_x = imu_data.uni_gyro[X_AXIS].i;
    mpap_buff[28] = (uint8_t)((gyro_x & 0xFF000000) >> 24);
    mpap_buff[29] = (uint8_t)((gyro_x & 0x00FF0000) >> 16);
    mpap_buff[30] = (uint8_t)((gyro_x & 0x0000FF00) >> 8);
    mpap_buff[31] = (uint8_t)((gyro_x & 0x000000FF));

    //gyro y float
    uint32_t gyro_y = imu_data.uni_gyro[Y_AXIS].i;
    mpap_buff[32] = (uint8_t)((gyro_y & 0xFF000000) >> 24);
    mpap_buff[33] = (uint8_t)((gyro_y & 0x00FF0000) >> 16);
    mpap_buff[34] = (uint8_t)((gyro_y & 0x0000FF00) >> 8);
    mpap_buff[35] = (uint8_t)((gyro_y & 0x000000FF));   

    //gyro z float
    uint32_t gyro_z = imu_data.uni_gyro[Z_AXIS].i;
    mpap_buff[36] = (uint8_t)((gyro_z & 0xFF000000) >> 24);
    mpap_buff[37] = (uint8_t)((gyro_z & 0x00FF0000) >> 16);
    mpap_buff[38] = (uint8_t)((gyro_z & 0x0000FF00) >> 8);
    mpap_buff[39] = (uint8_t)((gyro_z & 0x000000FF));

    //acc x float
    uint32_t acc_x = imu_data.uni_acc[X_AXIS].i;
    mpap_buff[40] = (uint8_t)((acc_x & 0xFF000000) >> 24);
    mpap_buff[41] = (uint8_t)((acc_x & 0x00FF0000) >> 16);
    mpap_buff[42] = (uint8_t)((acc_x & 0x0000FF00) >> 8);
    mpap_buff[43] = (uint8_t)((acc_x & 0x000000FF));

    //acc y float
    uint32_t acc_y = imu_data.uni_acc[Y_AXIS].i;
    mpap_buff[44] = (uint8_t)((acc_y & 0xFF000000) >> 24);
    mpap_buff[45] = (uint8_t)((acc_y & 0x00FF0000) >> 16);
    mpap_buff[46] = (uint8_t)((acc_y & 0x0000FF00) >> 8);
    mpap_buff[47] = (uint8_t)((acc_y & 0x000000FF));

    //acc z float
    uint32_t acc_z = imu_data.uni_acc[Z_AXIS].i;
    mpap_buff[48] = (uint8_t)((acc_z & 0xFF000000) >> 24);
    mpap_buff[49] = (uint8_t)((acc_z & 0x00FF0000) >> 16);
    mpap_buff[50] = (uint8_t)((acc_z & 0x0000FF00) >> 8);
    mpap_buff[51] = (uint8_t)((acc_z & 0x000000FF));

    //wheel right
    int16_t torque_cmd_r = (int16_t)(BVC.t_ref[SV1_FR]*1000.0f);
    mpap_buff[52] = (uint8_t)((torque_cmd_r & 0xFF00) >> 8);
    mpap_buff[53] = (uint8_t)((torque_cmd_r & 0x00FF));

    int16_t omega_res_r = (int16_t)(sv_res[SV1_FR].vel_res);
    mpap_buff[54] = (uint8_t)((omega_res_r & 0xFF00) >> 8);
    mpap_buff[55] = (uint8_t)(omega_res_r & 0x00FF);

    //wheel left
    int16_t torque_cmd_l = (int16_t)(BVC.t_ref[SV2_FL]*1000.0f);
    mpap_buff[56] = (uint8_t)((torque_cmd_l & 0xFF00) >> 8);
    mpap_buff[57] = (uint8_t)((torque_cmd_l & 0x00FF));

    int16_t omega_res_l = (int16_t)(sv_res[SV2_FL].vel_res);
    mpap_buff[58] = (uint8_t)((omega_res_l & 0xFF00) >> 8);
    mpap_buff[59] = (uint8_t)(omega_res_l & 0x00FF);

    //vpos x
    uint32_t vpx = (int32_t)(COMM_DIR_X * BVC.pos_cmd[X_AXIS] * PXY_M_TO_INT16t) + 2147483648;
    mpap_buff[60] = (uint8_t)((vpx & 0xFF000000) >> 24);
    mpap_buff[61] = (uint8_t)((vpx & 0x00FF0000) >> 16);
    mpap_buff[62] = (uint8_t)((vpx & 0x0000FF00) >> 8);
    mpap_buff[63] = (uint8_t)((vpx & 0x000000FF));
    //vpos y
    //uint32_t py_res = (uint32_t)(BVC.pos_res[Y_AXIS] * PXY_M_TO_INT16t + 2147483648);//オドメトリ計算で符号反転を合わせているので符号設定不要
    uint32_t vpy = (int32_t)(BVC.pos_cmd[Y_AXIS] * PXY_M_TO_INT16t) + 2147483648;//オドメトリ計算で符号反転を合わせているので符号設定不要
    mpap_buff[64] = (uint8_t)((vpy & 0xFF000000) >> 24);
    mpap_buff[65] = (uint8_t)((vpy & 0x00FF0000) >> 16);
    mpap_buff[66] = (uint8_t)((vpy & 0x0000FF00) >> 8);
    mpap_buff[67] = (uint8_t)((vpy & 0x000000FF));
    //vpos qz
    uint16_t vqz = (int16_t)(COMM_DIR_Z * BVC.pos_cmd[Z_AXIS] * QZ_RAD_TO_INT16t) + 32768;
    mpap_buff[68] = (uint8_t)((vqz & 0xFF00) >> 8);
    mpap_buff[69] = (uint8_t)((vqz & 0x00FF));

    //mp version
    mpap_buff[70] = MP_VERSION_YY;
    mpap_buff[71] = MP_VERSION_MM;
    mpap_buff[72] = MP_VERSION_DD;
    mpap_buff[73] = MP_VERSION_NN;

    //qz from gyro
    //uint16_t qz_gyro = (int16_t)(COMM_DIR_Z * imu_data.theta[Z_AXIS] * QZ_RAD_TO_INT16t) + 32768;
    uint16_t qz_gyro = (int16_t)(COMM_DIR_Z * BVC.pos_res[Z_AXIS] * QZ_RAD_TO_INT16t) + 32768;
    mpap_buff[74] = (uint8_t)((qz_gyro & 0xFF00) >> 8);
    mpap_buff[75] = (uint8_t)((qz_gyro & 0x00FF));

    //reserved 76 - 91
    mpap_buff[76] = 0;
    mpap_buff[77] = 0;
    mpap_buff[78] = 0;
    mpap_buff[79] = 0;
    mpap_buff[80] = 0;
    mpap_buff[81] = 0;
    mpap_buff[82] = 0;
    mpap_buff[83] = 0;
    mpap_buff[84] = 0;
    mpap_buff[85] = 0;
    mpap_buff[86] = 0;
    mpap_buff[87] = 0;
    mpap_buff[88] = 0;
    mpap_buff[89] = 0;
    mpap_buff[90] = 0;
    mpap_buff[91] = 0;
    

    mpap_buff[COMM_MPAP_SIZE-1] = calc_crc(mpap_buff, COMM_MPAP_SIZE-1);



    //　送信
    USART_TypeDef *UARTx = COMM_APMP_UART;
    uint32_t timeout_count = 0;
    const uint32_t TIMEOUT_COUNT_MAX = 100000000;

    // TE (Transmit Enable)をセットすることでidleフレームを送信
    UARTx->CR1 |= USART_CR1_TE;

    // データ長だけTDRに書き込む。データはFIFOに積まれる。
    for (int i = 0; i < COMM_MPAP_SIZE; i++) {
        // while (UART_FLAG_TXFNF != ((UARTx->ISR) & UART_FLAG_TXFNF)) {
        while ( __is_usart_txFIFO_full(UARTx->ISR) ) {
            timeout_count++;
            if ( timeout_count > TIMEOUT_COUNT_MAX ) {
                timeout_count = 0;
                break;
                //dbg_flag_mpap |= 0x02;
            }
        }
        timeout_count = 0;
        UARTx->TDR = mpap_buff[i];
    }

    // 全てのデータをTDRに書いたらTC (Transmit Complete)が1になるまで待つ
    timeout_count = 0;
    while (!__is_transmit_completed(UARTx->ISR)) {
        timeout_count++;
        if ( timeout_count > TIMEOUT_COUNT_MAX) {
            //dbg_flag_mpap |= 0x04;
            break;
        }
    }

    // TE をクリアすることで送信を停止
    UARTx->CR1 &= (~USART_CR1_TE);
}

void send_data_on_mp_to_ap(void){
    s_send_status = SEND_ON_MP_TO_AP;
    //dbg_flag_mpap |= 0x08;
}

void comm_apmp_rx_handler(void)
{
    USART_TypeDef *UARTx = COMM_APMP_UART;
    static uint16_t error_count=0;
    static uint8_t cycle_count=0;

    if(!UARTx) {
        return;
    }
    PA1_ON();
    // 受信FIFOのデータをすべて処理する(受信FIFOのデータ格納段数<RLVL>が0になるまで)
    while (UARTx->ISR & USART_ISR_RXNE_RXFNE) {
        PA0_ON();
        // 受信FIFOから受信データ取り出し
        char read_data = (char)(UARTx->RDR & 0x000000FF);
        uint32_t rx_tail_tmp = (rx_tail+1) % RX_BUFF_SIZE;

        // 受信バッファに空きがあれば格納、なければ破棄
        if (rx_tail_tmp != rx_head) {
            rx_buff[rx_tail] = read_data;
            // 受信バッファ格納ポインタ更新
            rx_tail = rx_tail_tmp;
        }
        if(error_count > 0){
            error_count -= 1;
        }
        PA0_OFF();
    }
    cycle_count += 1;
    if(cycle_count%160==0){    //10Hzごとにエラーカウント加算
        error_count += 1;
        cycle_count = 0;
    }
    //エラーカウントが一定の値を超えたら強制停止（機能をオンにした場合）
    if(error_count > APMP_COMM_LOST_COUNT){
        error_count = 0;
        #ifdef APMP_COMM_LOST_EMERGENCY
            apmp_data.cmd = AP_MP_CMD_IDLE;
        #endif
        state_mp_to_ap |= STATE_MP_TO_AP_ERROR_APCOMM;
    }

    dbg_flag_mpap = error_count;
    PA1_OFF();
}

void comm_apmp_tx_handler(void)
{
    USART_TypeDef *UARTx = COMM_APMP_UART;

    // 送信バッファが空になるまで繰り返す
    while (tx_tail != tx_head) {
        if( UARTx->ISR & USART_ISR_TXE_TXFNF ) {
            // 送信FIFOに空きがあれば送信データを書き込む
            UARTx->TDR = tx_buff[tx_head];
            // 送信バッファ取り出しポインタ更新
            tx_head = (tx_head+1) % TX_BUFF_SIZE;
        } else {
            // 送信FIFOがフルなので次の割り込みまで処理を延期する
            return;
        }
    }
}

/**********************************
  １バイト受信
 **********************************/
int32_t comm_apmp_read(int32_t *read_char)
{
    if (rx_tail != rx_head) {
        // 受信バッファにデータがあれば取り出す
        *read_char = rx_buff[rx_head];
        // 受信バッファ取り出しポインタ更新
        rx_head = (rx_head+1) % RX_BUFF_SIZE;
        return 1;
    }

    // 受信バッファが空なら何もしないで終了
    *read_char = 0;
    return 0;
}

/*********************************
  １バイト送信
 *********************************/
int32_t comm_apmp_write(int32_t send_char)
{
    uint32_t txTailTmp = (tx_tail+1) % TX_BUFF_SIZE;
    // 送信バッファに空きができるまで待つ
    while (txTailTmp == tx_head) {}
    // 送信バッファに送信データ格納
    tx_buff[tx_tail] = (char)send_char;
    // 送信バッファ格納ポインタ更新
    tx_tail = txTailTmp;

    return 1;
}



/**********************************
  １文字表示
 **********************************/
int32_t comm_apmp_putchar(int32_t c)
{
    /* 改行はCR+LFに変換して送信 */
    if(c == '\n'){
        if(!comm_apmp_write(0x0D))
            return(0);
        if(!comm_apmp_write(0x0A))
            return 0;
    }

    /* それ以外はそのまま送信 */
    else{
        if(!comm_apmp_write(c))
            return 0;
    }

    return 1;
}

/**********************************
  文字列表示
 **********************************/
static int32_t printStr(const char *str)
{
    const char *s;

    /* 終端文字まで1文字ずつ表示 */
    for(s=str; *s != '\0'; s++)
        if(!comm_apmp_putchar(*s))
            return 0;

    /* 表示した文字数を返す */
    return (s - str);
}

/**********************************
  文字列表示(最後に改行付き)
 **********************************/
int32_t comm_apmp_puts(const char *str)
{
    int32_t len;

    /* 文字列表示 */
    len = printStr(str);

    /* 最後に改行を表示 */
    comm_apmp_putchar('\n');

    return (len + 1);
}

/**********************************
  指定されたフォーマットで表示する
 **********************************/
static void printFmt(char *p, uint16_t order, uint16_t alignLeft, uint16_t fillZero, uint16_t minus)
{
    char *s = p;
    char pad = ' ';
    uint16_t len = 0;
    int32_t i;

    /* 文字数のカウント */
    for(len=0; *s != '\0'; len++, s++);

    /* マイナスなら文字数調整 */
    if(minus)
        len++;

    /* 文字数の調整 */
    if(order){
        if(order > len){
            order -= len;
        }else{
            order = 0;
        }
    }

    /* 右詰め */
    if( ! alignLeft){
        /* 詰め文字の設定 */
        if(fillZero){
            pad = '0';
            
            /* マイナス表示 */
            if(minus){
                comm_apmp_putchar('-');
                minus = 0;
            }
        }

        for(i=0; i<order; i++)
            comm_apmp_putchar(pad);
    }

    /* マイナス表示 */
    if(minus)
        comm_apmp_putchar('-');

    /* データの表示 */
    printStr(p);

    /* 左詰め */
    if(alignLeft)
        for(i=0; i<order; i++)
            comm_apmp_putchar(' ');
}

/**********************************
  フォーマットを解釈する
 **********************************/
static const char *parseFmt(const char *s, void *value)
{
    char buf[12];
    //char buf_f[6];
    char *p = buf;
    uint16_t alignLeft = 0;
    uint16_t fillZero = 0;
    uint16_t order = 0;
    uint16_t minus = 0;

    /* 左詰判定 */
    if(*s == '-'){
        alignLeft = 1;
        s++;
    }

    /* ゼロフィル判定 */
    if(*s == '0'){
        fillZero = 1;
        s++;
    }

    /* 文字数指定判定 */
    if(isDec(*s)){
        for(order = 0; isDec(*s); ){
            order *= 10;
            order += (*s - '0');
            s++;
        }
    }

    /* 種類判定、表示準備 */
    switch (*s){
    case 'd':   /* 符号付10進数 */
        /* 正負の判定 */
        if((int32_t)value >= 0){
            uint2Dec((uint32_t)value, buf);
        }else{
            uint2Dec((uint32_t)(-(int32_t)value), buf);
            minus = 1;
        }
        break;
    case 'u':   /* 符号無し10進数 */
        uint2Dec((uint32_t)value, buf);
        break;
    case 'x':   /* 小文字16進数 */
        uint2Hex((uint32_t)value, 0, buf);
        break;
    case 'X':   /* 大文字16進数 */
        uint2Hex((uint32_t)value, 1, buf);
        break;
    case 's':   /* 文字列 */
        p = (char*)value;
        break;
    case 'c':   /* １文字 */
        buf[0] = (char)((uint32_t)value & 0xFF);
        buf[1] = '\0';
        break;
    // case 'f':   /* float型 */

    //     value_int = (int32_t)value;
    //     vf = (float)(*value);
    //     // value_float = (int32_t)((float)value*1000 - value_int * 1000);

    //     break;
    default:
        buf[0] = '\0';
        break;
    }

    /* 表示 */
    printFmt(p, order, alignLeft, fillZero, minus);

    s++;
    return(s);
}

/**********************************
  いわゆる printf
 **********************************/
int32_t comm_apmp_printf(const char *str, ... )
{
    va_list ap;
    const char *s;

    /* 可変引数の初期化 */
    va_start(ap, str);

    for(s = str; *s != '\0'; ){
        /* 特殊文字判定 */
        if(*s == '%'){
            s++;

            /* "%%"なら'%'を表示 */
            if(*s == '%'){
                comm_apmp_putchar('%');
                s++;
            }
            /* フォーマットに従って表示 */
            else{
                s = parseFmt(s, va_arg(ap, void *));
                if(s == NULL)
                    return -1;
            }
        }

        /* 1文字ずつ普通に表示 */
        else{
            comm_apmp_putchar(*s);
            s++;
        }
    }

    va_end(ap);
    return(s - str);
}

/**********************************
  １文字受信
 **********************************/
int32_t comm_apmp_getchar(void)
{
    int32_t c;

    /* データを受信するまで待つ */
    while(!comm_apmp_read(&c));

    /* 改行文字変換 */
    // if(c == 0x0D)
    //     c = '\n';
    return c;
}

/**********************************
  １文字受信(データを待たない)
 **********************************/
int32_t comm_apmp_getchar2(void)
{
    int32_t c;

    /* データを受信 */
    comm_apmp_read(&c);

    /* 改行文字変換 */
    if(c == 0x0D)
        c = '\n';
    return c;
}


/**********************************
  文字列を一行受信
 **********************************/
char *comm_apmp_gets(char *s)
{
    int32_t i = 0;
    int32_t c;

    /* 改行まで受信 */
    while((c = comm_apmp_getchar()) != '\n'){
        s[i] = c;
        i++;
        if (isprint(c)) {
            // comm_apmp_putchar(c);     /* エコーバック */
            comm_apmp_printf("%c", c);
        } else {
                //comm_apmp_printf("[0x%02X]",c);
                // 何も表示しない
        }
    }

    /* 改行をNULLに変換 */
    s[i] = '\0';

    return s;
}


static const uint8_t CRC8Table[256] = {  // x8 + x5 + x4 + 1
    0x00, 0x31, 0x62, 0x53, 0xC4, 0xF5, 0xA6, 0x97, 0xB9, 0x88, 0xDB, 0xEA, 0x7D, 0x4C, 0x1F, 0x2E,
    0x43, 0x72, 0x21, 0x10, 0x87, 0xB6, 0xE5, 0xD4, 0xFA, 0xCB, 0x98, 0xA9, 0x3E, 0x0F, 0x5C, 0x6D,
    0x86, 0xB7, 0xE4, 0xD5, 0x42, 0x73, 0x20, 0x11, 0x3F, 0x0E, 0x5D, 0x6C, 0xFB, 0xCA, 0x99, 0xA8,
    0xC5, 0xF4, 0xA7, 0x96, 0x01, 0x30, 0x63, 0x52, 0x7C, 0x4D, 0x1E, 0x2F, 0xB8, 0x89, 0xDA, 0xEB,

    0x3D, 0x0C, 0x5F, 0x6E, 0xF9, 0xC8, 0x9B, 0xAA, 0x84, 0xB5, 0xE6, 0xD7, 0x40, 0x71, 0x22, 0x13,
    0x7E, 0x4F, 0x1C, 0x2D, 0xBA, 0x8B, 0xD8, 0xE9, 0xC7, 0xF6, 0xA5, 0x94, 0x03, 0x32, 0x61, 0x50,
    0xBB, 0x8A, 0xD9, 0xE8, 0x7F, 0x4E, 0x1D, 0x2C, 0x02, 0x33, 0x60, 0x51, 0xC6, 0xF7, 0xA4, 0x95,
    0xF8, 0xC9, 0x9A, 0xAB, 0x3C, 0x0D, 0x5E, 0x6F, 0x41, 0x70, 0x23, 0x12, 0x85, 0xB4, 0xE7, 0xD6,

    0x7A, 0x4B, 0x18, 0x29, 0xBE, 0x8F, 0xDC, 0xED, 0xC3, 0xF2, 0xA1, 0x90, 0x07, 0x36, 0x65, 0x54,
    0x39, 0x08, 0x5B, 0x6A, 0xFD, 0xCC, 0x9F, 0xAE, 0x80, 0xB1, 0xE2, 0xD3, 0x44, 0x75, 0x26, 0x17,
    0xFC, 0xCD, 0x9E, 0xAF, 0x38, 0x09, 0x5A, 0x6B, 0x45, 0x74, 0x27, 0x16, 0x81, 0xB0, 0xE3, 0xD2,
    0xBF, 0x8E, 0xDD, 0xEC, 0x7B, 0x4A, 0x19, 0x28, 0x06, 0x37, 0x64, 0x55, 0xC2, 0xF3, 0xA0, 0x91,

    0x47, 0x76, 0x25, 0x14, 0x83, 0xB2, 0xE1, 0xD0, 0xFE, 0xCF, 0x9C, 0xAD, 0x3A, 0x0B, 0x58, 0x69,
    0x04, 0x35, 0x66, 0x57, 0xC0, 0xF1, 0xA2, 0x93, 0xBD, 0x8C, 0xDF, 0xEE, 0x79, 0x48, 0x1B, 0x2A,
    0xC1, 0xF0, 0xA3, 0x92, 0x05, 0x34, 0x67, 0x56, 0x78, 0x49, 0x1A, 0x2B, 0xBC, 0x8D, 0xDE, 0xEF,
    0x82, 0xB3, 0xE0, 0xD1, 0x46, 0x77, 0x24, 0x15, 0x3B, 0x0A, 0x59, 0x68, 0xFF, 0xCE, 0x9D, 0xAC,
};

static uint8_t calc_crc(uint8_t *ptr, uint8_t len)
{
    uint8_t crc = 0xff;
    while (len--) {
        crc = CRC8Table[crc ^ *ptr++];
    }
    return crc;
}
