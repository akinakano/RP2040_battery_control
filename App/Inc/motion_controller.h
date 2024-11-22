#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>
#include "HW_type.h"

void MotionControl_IrqHandler();
void MotionControl_Init();

#define LIM_MIN_MAX(val,min,max)        ((val)<(min) ? (min) : ((val)>(max) ? (max) : (val)))
#define SMC_PI_F                    (3.14159265358979f)

// ********************
// 運動制御ステータス
// ********************
#define MC_STATE_IDLE       (0)
#define MC_STATE_START      (1)
#define MC_STATE_RUN        (2)
#define MC_STATE_STOP       (3)
#define MC_STATE_INIT       (4)
#define MC_STATE_CALIB      (5)
#define MC_STATE_EMERGENCY  (99)

#define MC_START_CYCLE      (800)

#define MC_UNCONTROLLABLE_EMMERGENCY    //持ち上げたときの暴走検知して止める
#define MC_OVER_MOTOR_OMEGA     (4500)      //45H = 2700RPM > 6mpsなので異常
#define MC_UNCONTROLLABLE_LIMIT (400)       //0.25s

//軸番号
#define X_AXIS (0)
#define Y_AXIS (1)
#define Z_AXIS (2)

//SV
#define SV1_FR      (1) //差動のときは右車輪 //0808 1へ変更
#define SV2_FL      (0) //差動のときは左車輪 //0808 0へ変更
#define SV3_HR      (2)
#define SV4_HL      (3)
//軸方向
#define SV1_DIRECTION (1) //差動のときは右車輪 //0808 1へ変更
#define SV2_DIRECTION (-1) //差動のときは左車輪 //0808 -1へ変更
#define SV3_DIRECTION (-1)
#define SV4_DIRECTION (1)

#define ACC_X_DIRECTION  (1)
#define ACC_Y_DIRECTION  (1)
#define ACC_Z_DIRECTION  (1)
#define GYRO_X_DIRECTION (1)
#define GYRO_Y_DIRECTION (1)
#define GYRO_Z_DIRECTION (-1)

//モデル固有値
#define WHEEL_RADIUS           (0.028995f)    //車輪半径 実測17号機 m
//#define WHEEL_RADIUS           (0.02922f)    //車輪半径 後で調べる 直径58.8mm //m
//#define WHEEL_TREAD            (0.250f)           //車輪間距離 m
//#define WHEEL_TREAD            (0.222f)           //車輪間距離 m
//#define WHEEL_TREAD            (0.2195f)           //車輪間距離 m 厚木体育館補正 -4deg
//#define WHEEL_TREAD            (0.2183f)           //車輪間距離 m 厚木体育館補正 -6deg
#define WHEEL_TREAD            (0.2179f)           //車輪間距離 m 厚木体育館補正 -6.5deg
#define WHEEL_TREAD_HARF       (WHEEL_TREAD/2)  //車輪間距離の半分 m
#define BODY_HALF_LENGTH_X     (0.075f)    //X方向長さ/2    //m
#define BODY_HALF_LENGTH_Y     (0.1f)     //Y方向長さ/2     //m
#define MOTOR_TORQUE_CONSTANT  (0.085f)   //トルク定数
#define MOTOR_INERTIA          (0.0001f)  //慣性モーメント(ギア込み)
#define BODY_WEIGHT            (3.560f)    // (実測)本体の重量 [kg]
#define BODY_Z_INERTIA         (0.05f)    // (仮値)本体のZ軸周りのイナーシャ [kg m^2]
#define IMU_GYRO_SCALE_GAIN  (1.0000)   // 機体ごとのIMUジャイロ値の変換補正値、その場回転のログから個体ごとに算出
//#define IMU_GYRO_SCALE_GAIN        (0.9990f)     // 機体ごとのIMUジャイロ値の変換補正値、その場回転のログから個体ごとに算出

//リミット
#define MOTOR_CURRENT_LIMIT    (2.5f)     //電流リミット
#define BODY_VX_LIMIT          (6.0f)     //速度リミット
#define BODY_VY_LIMIT          (0.5f)
#define DBG_AP_REF_GAIN        (10)    //AP指令値にかける倍率（後で変更した方がいいかも）
#define BODY_WZ_LIMIT          (180.0f / 180.0f * SMC_PI_F)     //角速度リミット
#define MC_V_DELTA_LIMIT       (0.002f)          //加速度リミット（MCの制御周期が1.6kHzなので、左の値*1600が加速度の値[m/s^2]になる）
#define MC_V_DELTA_LIMIT2      (0.005f)         //加速度リミット（減速時のもの）

#define MC_DELTA_LIMIT(val, tgt, delta)    ((tgt) > ((val) + (delta)) ? ((val) + (delta)) : (tgt) < ((val) - (delta)) ? ((val) - (delta)) : (tgt))
#define MC_ABS(x)                          ((x)<(0) ? (-x) : (x))
#define MC_DELTA_LIMIT2(val, tgt, delta1, delta2)    (MC_ABS(tgt) > (MC_ABS(val) + (delta1)) ? (MC_ABS(val) + (delta1)) : MC_ABS(tgt) < (MC_ABS(val) - (delta2)) ? (MC_ABS(val) - (delta2)) : MC_ABS(tgt))
#define MC_SIGN(s,val)                     ((s)<(0) ? (-val) : (val))
#define MC_CLIP_PI(s)                      ((s)>(SMC_PI_F) ? ((s)-2.0f*SMC_PI_F) : ((s)<(-SMC_PI_F) ? ((s)+2.0f*SMC_PI_F) : (s)))

//運動制御ステータス
typedef struct {
    uint8_t  state;
    uint8_t  mode;
    uint16_t time_count;    
}MotionControlStatus_t;

//IMUデータ
typedef union{
    float f;
    uint32_t i;
}imu_union_data;

typedef struct {
    float temp;
    float acc[3];
    float gyro[3];
    imu_union_data uni_acc[3];
    imu_union_data uni_gyro[3];
    float acc_bias[3];
    float gyro_bias[3];
    float acc_min[3];
    float gyro_min[3];
    float acc_max[3];
    float gyro_max[3];
    //float theta[3];
    int32_t calib_count;
    float gyro_scale_gain;
    bool  calib_end;
    float acc_quat[4];
    float gyro_quat[4];
}IMU_Data_t;

//sv受信データ
typedef struct {
    float vel_res;
    float iq_res;
    float th_res;
    float vdc;
}SV_Data_float_t;

typedef struct{
    float omega_wob;
    float acc_res[3];
    float vel_cmd_raw[3];
    float vel_cmd[3];
    float vel_cmd_dbg[3];
    float vel_ref[3];
    float vel_res[3];
    float vel_res_z1[3];
    float pos_cmd[3];
    float pos_res[3];
    float acc_ref[3];
    float acc_lpf[3];
    float acc_res_z1[3];
    float acc_dis[3];
    float acc_dis_lpf[3];
    float dot_acc_dis[3];
    float dot_acc_dis_z1[3];
    float dq_res[4];
    float ddq_ref[4];
    float tau_ref[4];
    float i_ref[4];
    float v_ref[4];
    float t_ref[4];
    float Jaco[3][4];
    float transJaco[4][3];
    float invJaco[4][3];
    float invtransJaco[3][4];
    float kp[3];
    float omega_ref_adj_y;
    float omega_ref_prev;
    uint16_t BVC_debug_flag;
}BODY_VEL_CTRL_t;

// オドメトリ計算用
typedef struct {
    float sv1_delta_len;
    float sv2_delta_len;
    int16_t sv1_mr_prev;
    int16_t sv2_mr_prev;
    double delta_theta;
    double delta_theta_gyro;
    double delta_theta_odom;
    double delta_len;
    float x_error_body;
    float y_error_body;
    float SV1_force;
    float SV2_force;
    float dtheta_ratio;
}ODOM_Data_t;

// メカパラメータ
typedef struct {
    float wheel_radius_r;
    float wheel_radius_l;
    float tread;
}MECHA_PARAM_t;

extern MotionControlStatus_t MC_state;
extern IMU_Data_t imu_data;
extern SV_Data_float_t sv_res[4];
extern BODY_VEL_CTRL_t BVC;
extern ODOM_Data_t ODOM_data;
extern MECHA_PARAM_t MECP;

#endif
