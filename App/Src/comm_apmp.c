#include <stdio.h>
#include <ctype.h>
#include <stdarg.h>
#include <stm32h747xx.h>
#include "comm_apmp.h"
#include "motion_controller.h"
#include "usb.h"
#include "HW_type.h"
#include "comm_battery.h"
#include "crc8.h"

static uint8_t tx_buff[MPAP_TX_BUFF_SIZE];
static uint8_t rx_buff[APMP_RX_BUFF_SIZE];
static int rx_pointer = 0;
static int s_send_status = SEND_OFF_MP_TO_AP;

static void Parse_APMPData(uint8_t *buf);
static void mpap_status(uint8_t *buf);

Comm_APMP_Receive apmp_data[2] = {
  {
    .cmd = 0,
    .crc = 0,
    .head = 0,
    .vx_cmd = 0x8000,
    .vy_cmd = 0x8000,
    .wz_cmd = 0x8000,
  },
  {
    .cmd = 0,
    .crc = 0,
    .head = 0,
    .vx_cmd = 0x8000,
    .vy_cmd = 0x8000,
    .wz_cmd = 0x8000,
  },
};
int apmp_data_bank = 0;

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
    if(rx_pointer >= COMM_APMP_SIZE) {
      Parse_APMPData(rx_buff);
      rx_pointer = 0;
    }
  }
}

void Comm_APMP_ErrorCheckInterval() { // 10Hz

  error_count++;
  if(error_count > APMP_COMM_LOST_COUNT) {
    error_count = 0;

    #ifdef APMP_COMM_LOST_EMERGENCY
        apmp_data[apmp_data_bank].cmd = AP_MP_CMD_IDLE;
    #endif
    state_mp_to_ap |= STATE_MP_TO_AP_ERROR_APCOMM;
  }
  dbg_flag_mpap = error_count;
}

static void Parse_APMPData(uint8_t *buf) {

  if(calc_crc(buf, COMM_APMP_SIZE - 1) != buf[COMM_APMP_SIZE - 1]) return;

  int nextBank = apmp_data_bank ^ 1;
  apmp_data[nextBank].head = (buf[0] << 8) | buf[1];
  apmp_data[nextBank].cmd = buf[2];
  apmp_data[nextBank].vx_cmd = (buf[3] << 8) | buf[4];
  apmp_data[nextBank].vy_cmd = (buf[5] << 8) | buf[6];
  apmp_data[nextBank].wz_cmd = (buf[7] << 8) | buf[8];
  apmp_data[nextBank].wheel_radius_right = (buf[9]  << 8) | buf[10];
  apmp_data[nextBank].wheel_radius_left  = (buf[11] << 8) | buf[12];
  apmp_data[nextBank].tread = (buf[13] << 8) | buf[14];
  apmp_data[nextBank].imu_gyro_scale_gain = (buf[15] << 8) | buf[16];
  apmp_data[nextBank].crc = buf[COMM_APMP_SIZE - 1];
  apmp_data_bank = nextBank;
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

    // battery remaining capacity
    buf[26] = BatteryStatus[BatteryStatusBank].Capacity;
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

    // battery Voltage mV(16bit)
    buf[76] = BatteryStatus[BatteryStatusBank].Voltage >> 8;
    buf[77] = BatteryStatus[BatteryStatusBank].Voltage;
    // battery Current mA(32bit)
    buf[78] = BatteryStatus[BatteryStatusBank].Current >> 24;
    buf[79] = BatteryStatus[BatteryStatusBank].Current >> 16;
    buf[80] = BatteryStatus[BatteryStatusBank].Current >> 8;
    buf[81] = BatteryStatus[BatteryStatusBank].Current;
    // battery status (32bit)
    buf[82] = 0;
    buf[83] = 0;
    buf[84] = BatteryStatus[BatteryStatusBank].ManufacturerAccess >> 8;
    buf[85] = BatteryStatus[BatteryStatusBank].ManufacturerAccess;
    // battery temperture 0.1K
    buf[86] = BatteryStatus[BatteryStatusBank].Temperture >> 8;
    buf[87] = BatteryStatus[BatteryStatusBank].Temperture;
    //reserved 88 - 91
    buf[88] = 0;
    buf[89] = 0;
    buf[90] = 0;
    buf[91] = 0;

    // CRC
    buf[COMM_MPAP_SIZE - 1] = calc_crc(buf, COMM_MPAP_SIZE - 1);
}
