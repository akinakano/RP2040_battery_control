/*------------------------------------------------------------------------------*/
/*  Include                                                                     */
/*------------------------------------------------------------------------------*/
#include "vqfInstance.h"
//#include "commonType.h"

//#include "basicvqf.hpp"
#include "vqf.hpp"
#include <math.h>
#include "timer.h"

/*------------------------------------------------------------------------------*/
/*  Macro Definition                                                            */
/*------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------*/
/*  Value Definition                                                            */
/*------------------------------------------------------------------------------*/
//BasicVQF vqf(0.001);
//VQF vqf(0.001);
VQF vqf(1.0f / MOTION_CONTROL_FREQ);

/*------------------------------------------------------------------------------*/
/*  Prototype Declaration                                                       */
/*------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------*/
/*  Function Implementation                                                     */
/*------------------------------------------------------------------------------*/

void vqf_Initialize(void)
{
    //vqf.setTauAcc(10.0f);
}

void vqf_Update(const float gyro[3], const float acc[3])
{
    float gyro_vqf[3];

    gyro_vqf[0] = gyro[0];
    gyro_vqf[1] = gyro[1];
    gyro_vqf[2] = gyro[2];

    //Bias追加 1deg/s
    //gyro_vqf[0] += 1.0f * M_PI_FLOAT / 180.0f;

    vqf.update(gyro_vqf, acc);
}

void vqf_GetQuat6D(float quat[4])
{
    vqf.getQuat6D(quat);
}

float vqf_GetBiasEstimate(float bias[3])
{
    return(vqf.getBiasEstimate(bias));
}

bool vqf_GetRestDetected()
{
    return(vqf.getRestDetected());
}

void vqf_QuatToEuler(float out[3], float quat[4])
{
    float qw = quat[0];
    float qx = quat[1];
    float qy = quat[2];
    float qz = quat[3];

    // ZYX
    out[0] = atan2f(2.0f * (qw * qx + qy * qz), 1.0f - 2.0f * (qx * qx + qy * qy));
    out[1] = asinf(2.0f * (qw * qy - qz * qx));
    out[2] = atan2f(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (qy * qy + qz * qz));
}

float vqf_QuatToEulerZ(float quat[4])
{
    float qw = quat[0];
    float qx = quat[1];
    float qy = quat[2];
    float qz = quat[3];

    return (atan2f(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (qy * qy + qz * qz)));
}

int vqf_GetDebugCountThGyro()
{
    return (vqf.getDebugCountThGyro());
}

int vqf_GetDebugCountThAcc()
{
    return (vqf.getDebugCountThAcc());
}

void vqf_ResetQuat()
{
    vqf.resetQuat();
}

void vqf_SetQuat(const float gyrQuat[4], const float accQuat[4])
{
    vqf.setQuat(gyrQuat, accQuat);
}

void vqf_GetQuat(float gyrQuat[4], float accQuat[4])
{
    vqf.getQuat(gyrQuat, accQuat);
}

void vqf_ResetState()
{
    vqf.resetState();
}

void vqf_SetRestBiasEstEnabled(bool enabled)
{
	vqf.setRestBiasEstEnabled(enabled);
}
