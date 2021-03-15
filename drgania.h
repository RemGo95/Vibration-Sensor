/*
 * mpu6050.h
 *
 *  Created on: Apr 13, 2020
 *
 */

#ifndef INC_GY521_H_
#define INC_GY521_H_

#endif /* INC_GY521_H_ */

#include <stdint.h>
#include "i2c.h"

// MPU6050
typedef struct {

   // int16_t Accel_X;
  //  int16_t Accel_Y;
  //  int16_t Accel_Z;
  //  double Ax;
  //  double Ay;
  //  double Az;

    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;
    double Gx;
    double Gy;
    double Gz;

    float Temp;

} MPU6050_t;
/*
typedef struct {
    double Q_angle;
    double Q_bias;
    double R_measure;
    double angle;
    double bias;
    double P[2][2];
} Kalman_t;
*/


uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx);

void NAPEDY(uint16_t ang, uint16_t speed);

void MPU6050_Read_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

void Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

void Read_Temp(I2C_HandleTypeDef *I2Cx, float T);



