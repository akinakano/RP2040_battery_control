#ifndef _VQF_INSTANCE_H_
#define _VQF_INSTANCE_H_

/*------------------------------------------------------------------------------*/
/*  Include                                                                     */
/*------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------*/
/*  Public functions                                                            */
/*------------------------------------------------------------------------------*/

#ifdef __cplusplus
extern "C"{
#endif

#define FLT_EPSILON (1.19209e-07)

void vqf_Initialize(void);
void vqf_Update(const float gyro[3], const float acc[3]);
void vqf_GetQuat6D(float quat[4]);
float vqf_GetBiasEstimate(float out[3]);
bool vqf_GetRestDetected();
void vqf_QuatToEuler(float out[3], float quat[4]);
float vqf_QuatToEulerZ(float quat[4]);
int vqf_GetDebugCountThGyro();
int vqf_GetDebugCountThAcc();
void vqf_ResetQuat();
void vqf_SetQuat(const float gyrQuat[4], const float accQuat[4]);
void vqf_GetQuat(float gyrQuat[4], float accQuat[4]);
void vqf_ResetState();
void vqf_SetRestBiasEstEnabled(bool enabled);


#ifdef __cplusplus
}
#endif
/*------------------------------------------------------------------------------*/
/*  Global Value                                                                */
/*------------------------------------------------------------------------------*/

#endif      //include Guard
