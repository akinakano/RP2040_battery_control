#include    <stdio.h>
#include    <ctype.h>
#include    <stdarg.h>
#include    <stm32h747xx.h>
#include    "comm_apmp.h"
#include    "motion_controller.h"
#include    "usb.h"
#include    "HW_type.h"
#include    "comm_battery.h"

extern struct BatteryStatus_st BatteryStatus;

static uint8_t tx_buff[MPAP_TX_BUFF_SIZE];
static uint8_t rx_buff[APMP_RX_BUFF_SIZE];
static int rx_pointer = 0;
static int s_send_status = SEND_OFF_MP_TO_AP;

static uint8_t calc_crc(uint8_t *ptr, uint8_t len);
static void Parse_APMPData(uint8_t *buf);
static void mpap_status(uint8_t *buf);

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
uint16_t state_mp_to_ap = STATE_MP_TO_AP_NORMAL;
uint16_t dbg_flag_mpap;
int error_count = 0;

void Comm_APMP_Rx_Handler(uint8_t* buf, uint32_t len) {

  for(int i = 0; i < len; i++) {
    if((rx_pointer == 0) && (buf[i] != COMM_APMP_HEADER_H)) continue;
    if((rx_pointer == 1) && (buf[i] != COMM_APMP_HEADER_L)) {
      rx_pointer = 0;
      continue;
    }
    rx_buff[rx_pointer++] = buf[i];
    if(rx_pointer >= sizeof(Comm_APMP_Receive)) Parse_APMPData(rx_buff);
  }
}

void Comm_APMP_ErrorCheckInterval() { // 10Hz

  error_count++;
  if(error_count > APMP_COMM_LOST_COUNT) {
    error_count = 0;

    #ifdef APMP_COMM_LOST_EMERGENCY
        apmp_data.cmd = AP_MP_CMD_IDLE;
    #endif
    state_mp_to_ap |= STATE_MP_TO_AP_ERROR_APCOMM;
  }
  dbg_flag_mpap = error_count;
}

static void Parse_APMPData(uint8_t *buf) {

  if(calc_crc(rx_buff, sizeof(Comm_APMP_Receive) - 1) != rx_buff[sizeof(Comm_APMP_Receive) - 1]) {
    rx_pointer = 0;
    return;
  }

  apmp_data.head_h = buf[0];
  apmp_data.head_l = buf[1];
  apmp_data.cmd = buf[2];
  apmp_data.vx_cmd_h = buf[3];
  apmp_data.vx_cmd_l = buf[4];
  apmp_data.vy_cmd_h = buf[5];
  apmp_data.vy_cmd_l = buf[6];
  apmp_data.wz_cmd_h = buf[7];
  apmp_data.wz_cmd_l = buf[8];
  apmp_data.wheel_radius_right = (buf[9]  << 8) | buf[10];
  apmp_data.wheel_radius_left  = (buf[11] << 8) | buf[12];
  apmp_data.tread = (buf[13] << 8) | buf[14];
  apmp_data.imu_gyro_scale_gain = (buf[15] << 8) | buf[16];
  apmp_data.crc = buf[sizeof(Comm_APMP_Receive) - 1];
  rx_pointer = 0;
}

void send_data_on_mp_to_ap(void) {

  s_send_status = SEND_ON_MP_TO_AP;
}

void Comm_APMP_Tx_Handler() {

  if(s_send_status == SEND_ON_MP_TO_AP) {
    mpap_status(tx_buff);
    CDC_Transmit_FS(tx_buff, COMM_MPAP_SIZE);
    s_send_status = SEND_OFF_MP_TO_AP;
  }
}

static void mpap_status(uint8_t *buf) {

    //head
    buf[0] = 0xab;
    buf[1] = 0xcd;
    //time count
    buf[2] = (uint8_t)((MC_state.time_count & 0xFF00) >> 8);
    buf[3] = (uint8_t)(MC_state.time_count & 0x00FF);
    //vel_cmd
    //vx
    //uint16_t vx_cmd = (uint16_t)(COMM_DIR_X * BVC.vel_cmd[X_AXIS] * VXY_MPS_TO_INT16t + 32768);
    //uint16_t vx_cmd = (int16_t)(COMM_DIR_X * BVC.vel_cmd[X_AXIS] * VXY_MPS_TO_INT16t) + 32768;
    uint16_t vx_cmd = (int16_t)(COMM_DIR_X * BVC.vel_ref[X_AXIS] * VXY_MPS_TO_INT16t) + 32768;
    buf[4] = (uint8_t)((vx_cmd & 0xFF00) >> 8);
    buf[5] = (uint8_t)(vx_cmd & 0x00FF);
    //vy
    //uint16_t vy_cmd = (uint16_t)(COMM_DIR_Y * BVC.vel_cmd[Y_AXIS] * VXY_MPS_TO_INT16t + 32768);
    //uint16_t vy_cmd = (int16_t)(COMM_DIR_Y * BVC.vel_cmd[Y_AXIS] * VXY_MPS_TO_INT16t) + 32768;
    uint16_t vy_cmd = (int16_t)(COMM_DIR_Y * BVC.vel_ref[Y_AXIS] * VXY_MPS_TO_INT16t) + 32768;
    buf[6] = (uint8_t)((vy_cmd & 0xFF00) >> 8);
    buf[7] = (uint8_t)(vy_cmd & 0x00FF);
    //vz
    //uint16_t wz_cmd = (uint16_t)(COMM_DIR_Z * BVC.vel_cmd[Z_AXIS] * WZ_RADPS_TO_INT16t + 32768);
    //uint16_t wz_cmd = (int16_t)(COMM_DIR_Z * BVC.vel_cmd[Z_AXIS] * WZ_RADPS_TO_INT16t) + 32768;
    uint16_t wz_cmd = (int16_t)(COMM_DIR_Z * BVC.vel_ref[Z_AXIS] * WZ_RADPS_TO_INT16t) + 32768;
    buf[8] = (uint8_t)((wz_cmd & 0xFF00) >> 8);
    buf[9] = (uint8_t)(wz_cmd & 0x00FF);
    //vel_res
    //vx
    //uint16_t vx_res = (uint16_t)(COMM_DIR_X * BVC.vel_res[X_AXIS] * VXY_MPS_TO_INT16t + 32768);
    uint16_t vx_res = (int16_t)(COMM_DIR_X * BVC.vel_res[X_AXIS] * VXY_MPS_TO_INT16t) + 32768;
    buf[10] = (uint8_t)((vx_res & 0xFF00) >> 8);
    buf[11] = (uint8_t)(vx_res & 0x00FF);
    //vy
    //uint16_t vy_res = (uint16_t)(COMM_DIR_Y * BVC.vel_res[Y_AXIS] * VXY_MPS_TO_INT16t + 32768);
    uint16_t vy_res = (int16_t)(COMM_DIR_Y * BVC.vel_res[Y_AXIS] * VXY_MPS_TO_INT16t) + 32768;
    buf[12] = (uint8_t)((vy_res & 0xFF00) >> 8);
    buf[13] = (uint8_t)(vy_res & 0x00FF);
    //wz
    //uint16_t wz_res = (uint16_t)(COMM_DIR_Z * BVC.vel_res[Z_AXIS] * WZ_RADPS_TO_INT16t + 32768);
    uint16_t wz_res = (int16_t)(COMM_DIR_Z * BVC.vel_res[Z_AXIS] * WZ_RADPS_TO_INT16t) + 32768;
    buf[14] = (uint8_t)((wz_res & 0xFF00) >> 8);
    buf[15] = (uint8_t)(wz_res & 0x00FF);
    //odomet
    //px
    //uint32_t px_res = (uint32_t)(COMM_DIR_X * BVC.pos_res[X_AXIS] * PXY_M_TO_INT16t + 2147483648);
    uint32_t px_res = (int32_t)(COMM_DIR_X * BVC.pos_res[X_AXIS] * PXY_M_TO_INT16t) + 2147483648;
    buf[16] = (uint8_t)((px_res & 0xFF000000) >> 24);
    buf[17] = (uint8_t)((px_res & 0x00FF0000) >> 16);
    buf[18] = (uint8_t)((px_res & 0x0000FF00) >> 8);
    buf[19] = (uint8_t)((px_res & 0x000000FF));
    //py
    //uint32_t py_res = (uint32_t)(BVC.pos_res[Y_AXIS] * PXY_M_TO_INT16t + 2147483648);//オドメトリ計算で符号反転を合わせているので符号設定不要
    uint32_t py_res = (int32_t)(BVC.pos_res[Y_AXIS] * PXY_M_TO_INT16t) + 2147483648;//オドメトリ計算で符号反転を合わせているので符号設定不要
    buf[20] = (uint8_t)((py_res & 0xFF000000) >> 24);
    buf[21] = (uint8_t)((py_res & 0x00FF0000) >> 16);
    buf[22] = (uint8_t)((py_res & 0x0000FF00) >> 8);
    buf[23] = (uint8_t)((py_res & 0x000000FF));
    //qz
    uint16_t qz_res = (int16_t)(COMM_DIR_Z * BVC.pos_res[Z_AXIS] * QZ_RAD_TO_INT16t) + 32768;
    buf[24] = (uint8_t)((qz_res & 0xFF00) >> 8);
    buf[25] = (uint8_t)((qz_res & 0x00FF));
#ifdef BATTERY_INFO
    buf[26] = BatteryStatus.Capacity;
#else
    //vm
#if(HW_WHEEL == HW_WHEEL_MECANUM)
    float vm_ave_f = 0.25f * (sv_res[SV1_FR].vdc + sv_res[SV2_FL].vdc + sv_res[SV3_HR].vdc + sv_res[SV4_HL].vdc);
#elif(HW_WHEEL == HW_WHEEL_DIFF)
    float vm_ave_f = 0.5f * (sv_res[SV1_FR].vdc + sv_res[SV2_FL].vdc);
#endif
    uint8_t vm_ave = (uint8_t)(vm_ave_f);
    buf[26] = vm_ave;
#endif
    //err
    buf[27] = state_mp_to_ap;
    buf[27] |= BVC.BVC_debug_flag;
    BVC.BVC_debug_flag = 0;
    state_mp_to_ap = 0; //APに送ったらリセット
    //crc

    //IMU data
    //gyro x float
    uint32_t gyro_x = imu_data.uni_gyro[X_AXIS].i;
    buf[28] = (uint8_t)((gyro_x & 0xFF000000) >> 24);
    buf[29] = (uint8_t)((gyro_x & 0x00FF0000) >> 16);
    buf[30] = (uint8_t)((gyro_x & 0x0000FF00) >> 8);
    buf[31] = (uint8_t)((gyro_x & 0x000000FF));

    //gyro y float
    uint32_t gyro_y = imu_data.uni_gyro[Y_AXIS].i;
    buf[32] = (uint8_t)((gyro_y & 0xFF000000) >> 24);
    buf[33] = (uint8_t)((gyro_y & 0x00FF0000) >> 16);
    buf[34] = (uint8_t)((gyro_y & 0x0000FF00) >> 8);
    buf[35] = (uint8_t)((gyro_y & 0x000000FF));   

    //gyro z float
    uint32_t gyro_z = imu_data.uni_gyro[Z_AXIS].i;
    buf[36] = (uint8_t)((gyro_z & 0xFF000000) >> 24);
    buf[37] = (uint8_t)((gyro_z & 0x00FF0000) >> 16);
    buf[38] = (uint8_t)((gyro_z & 0x0000FF00) >> 8);
    buf[39] = (uint8_t)((gyro_z & 0x000000FF));

    //acc x float
    uint32_t acc_x = imu_data.uni_acc[X_AXIS].i;
    buf[40] = (uint8_t)((acc_x & 0xFF000000) >> 24);
    buf[41] = (uint8_t)((acc_x & 0x00FF0000) >> 16);
    buf[42] = (uint8_t)((acc_x & 0x0000FF00) >> 8);
    buf[43] = (uint8_t)((acc_x & 0x000000FF));

    //acc y float
    uint32_t acc_y = imu_data.uni_acc[Y_AXIS].i;
    buf[44] = (uint8_t)((acc_y & 0xFF000000) >> 24);
    buf[45] = (uint8_t)((acc_y & 0x00FF0000) >> 16);
    buf[46] = (uint8_t)((acc_y & 0x0000FF00) >> 8);
    buf[47] = (uint8_t)((acc_y & 0x000000FF));

    //acc z float
    uint32_t acc_z = imu_data.uni_acc[Z_AXIS].i;
    buf[48] = (uint8_t)((acc_z & 0xFF000000) >> 24);
    buf[49] = (uint8_t)((acc_z & 0x00FF0000) >> 16);
    buf[50] = (uint8_t)((acc_z & 0x0000FF00) >> 8);
    buf[51] = (uint8_t)((acc_z & 0x000000FF));

    //wheel right
    int16_t torque_cmd_r = (int16_t)(BVC.t_ref[SV1_FR]*1000.0f);
    buf[52] = (uint8_t)((torque_cmd_r & 0xFF00) >> 8);
    buf[53] = (uint8_t)((torque_cmd_r & 0x00FF));

    int16_t omega_res_r = (int16_t)(sv_res[SV1_FR].vel_res);
    buf[54] = (uint8_t)((omega_res_r & 0xFF00) >> 8);
    buf[55] = (uint8_t)(omega_res_r & 0x00FF);

    //wheel left
    int16_t torque_cmd_l = (int16_t)(BVC.t_ref[SV2_FL]*1000.0f);
    buf[56] = (uint8_t)((torque_cmd_l & 0xFF00) >> 8);
    buf[57] = (uint8_t)((torque_cmd_l & 0x00FF));

    int16_t omega_res_l = (int16_t)(sv_res[SV2_FL].vel_res);
    buf[58] = (uint8_t)((omega_res_l & 0xFF00) >> 8);
    buf[59] = (uint8_t)(omega_res_l & 0x00FF);

    //vpos x
    uint32_t vpx = (int32_t)(COMM_DIR_X * BVC.pos_cmd[X_AXIS] * PXY_M_TO_INT16t) + 2147483648;
    buf[60] = (uint8_t)((vpx & 0xFF000000) >> 24);
    buf[61] = (uint8_t)((vpx & 0x00FF0000) >> 16);
    buf[62] = (uint8_t)((vpx & 0x0000FF00) >> 8);
    buf[63] = (uint8_t)((vpx & 0x000000FF));
    //vpos y
    //uint32_t py_res = (uint32_t)(BVC.pos_res[Y_AXIS] * PXY_M_TO_INT16t + 2147483648);//オドメトリ計算で符号反転を合わせているので符号設定不要
    uint32_t vpy = (int32_t)(BVC.pos_cmd[Y_AXIS] * PXY_M_TO_INT16t) + 2147483648;//オドメトリ計算で符号反転を合わせているので符号設定不要
    buf[64] = (uint8_t)((vpy & 0xFF000000) >> 24);
    buf[65] = (uint8_t)((vpy & 0x00FF0000) >> 16);
    buf[66] = (uint8_t)((vpy & 0x0000FF00) >> 8);
    buf[67] = (uint8_t)((vpy & 0x000000FF));
    //vpos qz
    uint16_t vqz = (int16_t)(COMM_DIR_Z * BVC.pos_cmd[Z_AXIS] * QZ_RAD_TO_INT16t) + 32768;
    buf[68] = (uint8_t)((vqz & 0xFF00) >> 8);
    buf[69] = (uint8_t)((vqz & 0x00FF));

    //mp version
    buf[70] = MP_VERSION_YY;
    buf[71] = MP_VERSION_MM;
    buf[72] = MP_VERSION_DD;
    buf[73] = MP_VERSION_NN;

    //qz from gyro
    //uint16_t qz_gyro = (int16_t)(COMM_DIR_Z * imu_data.theta[Z_AXIS] * QZ_RAD_TO_INT16t) + 32768;
    uint16_t qz_gyro = (int16_t)(COMM_DIR_Z * BVC.pos_res[Z_AXIS] * QZ_RAD_TO_INT16t) + 32768;
    buf[74] = (uint8_t)((qz_gyro & 0xFF00) >> 8);
    buf[75] = (uint8_t)((qz_gyro & 0x00FF));

    //reserved 76 - 91
    buf[76] = 0;
    buf[77] = 0;
    buf[78] = 0;
    buf[79] = 0;
    buf[80] = 0;
    buf[81] = 0;
    buf[82] = 0;
    buf[83] = 0;
    buf[84] = 0;
    buf[85] = 0;
    buf[86] = 0;
    buf[87] = 0;
    buf[88] = 0;
    buf[89] = 0;
    buf[90] = 0;
    buf[91] = 0;

    // CRC
    buf[COMM_MPAP_SIZE - 1] = calc_crc(buf, COMM_MPAP_SIZE - 1);
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
