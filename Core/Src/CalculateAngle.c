#include "CalculateAngle.h"

Struct_Angle Angle;

void CalculateAccAngle(Struct_Angle* Angle, Struct_MPU6050* MPU6050)
{
	Angle->acc_roll  = atan(-MPU6050->acc_x / sqrt(pow(MPU6050->acc_y,2) + pow(MPU6050->acc_z,2))) * RADIAN_TO_DEGREE;
	Angle->acc_pitch = atan(MPU6050->acc_y / sqrt(pow(MPU6050->acc_x,2) + pow(MPU6050->acc_z,2))) * RADIAN_TO_DEGREE;
//	Angle->acc_yaw = atan(sqrt(pow(MPU6050->acc_x, 2) + pow(MPU6050->acc_y, 2)) / MPU6050->acc_z) * RADIAN_TO_DEGREE;
	//Can't use Angle->acc_yaw there is no reliability. It's based on my personal experimental view.
}

static float dt = 1/200.f; //Sample rate is 200Hz
void CalculateGyroAngle(Struct_Angle* Angle, Struct_MPU6050* MPU6050)
{
	Angle->gyro_roll  += MPU6050->gyro_y * dt;
	Angle->gyro_pitch += MPU6050->gyro_x * dt;
	Angle->gyro_yaw   += MPU6050->gyro_z * dt;
}

void CalculateCompliFilter(Struct_Angle* Angle, Struct_MPU6050* MPU6050)
{
	CalculateAccAngle(Angle, MPU6050);

	static float alpha = 0.96f;
	Angle->ComFilt_roll  = alpha*(MPU6050->gyro_y * dt + Angle->ComFilt_roll) + (1-alpha) * Angle->acc_roll;
	Angle->ComFilt_pitch = alpha*(MPU6050->gyro_x * dt + Angle->ComFilt_pitch) + (1-alpha) * Angle->acc_pitch;
	Angle->ComFilt_yaw   = Angle->ComFilt_yaw + MPU6050->gyro_z * dt;
}


/*ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡKalman Filterㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ*/

Kalman_t KalmanX = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f
};

Kalman_t KalmanY = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f,
};


void CalculateKalmanFilter(Struct_Angle* Angle, Struct_MPU6050* MPU6050)
{
	CalculateAccAngle(Angle, MPU6050);
	Angle->KalmFilt_roll  = Kalman_getAngle(&KalmanX, Angle->acc_roll, Angle->gyro_roll, dt);
	Angle->KalmFilt_pitch = Kalman_getAngle(&KalmanY, Angle->acc_pitch, Angle->gyro_pitch, dt);
}




float Kalman_getAngle(Kalman_t *Kalman, float newAngle, float newRate, float dt)
{
    float rate = newRate - Kalman->bias;
    Kalman->angle += dt * rate;

    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;

    float S = Kalman->P[0][0] + Kalman->R_measure;
    float K[2];
    K[0] = Kalman->P[0][0] / S;
    K[1] = Kalman->P[1][0] / S;

    float y = newAngle - Kalman->angle;
    Kalman->angle += K[0] * y;
    Kalman->bias += K[1] * y;

    float P00_temp = Kalman->P[0][0];
    float P01_temp = Kalman->P[0][1];

    Kalman->P[0][0] -= K[0] * P00_temp;
    Kalman->P[0][1] -= K[0] * P01_temp;
    Kalman->P[1][0] -= K[1] * P00_temp;
    Kalman->P[1][1] -= K[1] * P01_temp;

    return Kalman->angle;
};






