/*
 * drgania.c
 *
 *  Created on: Apr 13, 2020
 *
 *
 *
 *
 *
 */





#include <drgania.h>
#include <math.h>
#include "tim.h"


//Rejestry akcelerometru MPU6050
#define RAD_TO_DEG 57.295779513082320876798154814105

#define WHO_AM_I_REG 0x75
#define PWR_MGMT_1_REG 0x6B
#define SMPLRT_DIV_REG 0x19
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_CONFIG_REG 0x1B
#define GYRO_XOUT_H_REG 0x43

// Setup MPU6050
#define MPU6050_ADDR 0xD0
const uint16_t i2c_timeout = 100;
const double Accel_Z_corrector = 14418.0;

uint32_t timer;



uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx) {
    uint8_t check;
    uint8_t Data;

    // check device ID WHO_AM_I

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, i2c_timeout);

    if (check == 104)  // 0x68 will be returned by the sensor if everything goes well
    {
        // power management register 0X6B we should write all 0's to wake the sensor up
        Data = 0;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, i2c_timeout);

        // Set DATA RATE of 1KHz by writing SMPLRT_DIV register
        Data = 0x07;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, i2c_timeout);

        // Set accelerometer configuration in ACCEL_CONFIG Register
        // XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> � 2g
        Data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, i2c_timeout);

        // Set Gyroscopic configuration in GYRO_CONFIG Register
        // XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> � 250 �/s
        Data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, i2c_timeout);
        return 0;
    }
    return 1;
}


void MPU6050_Read_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct) {
    uint8_t Rec_Data[6];
    double X;             //przyspieszenie w osi X [m/s^2]
    double Y;				//przyspieszenie w osi Y [m/s^2]

    double pi=3.1415;

    double T;           //OKRES drgan
    double time;         //nasz t [s]

    double f;           //czestotliwosc [Hz]

    f=1/T;      //  [Hz]



    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    //int16_t Accel_Z_RAW;

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, i2c_timeout);

    Accel_X_RAW = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
    Accel_Y_RAW = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]);
    //Accel_Z_RAW = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]);



    X = Accel_X_RAW / 16384.0;
   Y = Accel_Y_RAW / 16384.0;
   // DataStruct->Az = Accel_Z_RAW / 1000;

   //a=apeak * sin(2pift)







}




//Transmisja cyfrowa
void Transmisja(unsigned int A){

	int U;
	U=A/20;

	HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, GPIO_PIN_SET);
	for(int i=0; i<U; i++){
											HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, GPIO_PIN_SET);
		  				  		  		  		HAL_Delay(A);
		  				  		  		  HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, GPIO_PIN_RESET);
		  				  		  		  		HAL_Delay(A);

		  				  		  		  	  }
	HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, GPIO_PIN_RESET);

}

void angletime(uint16_t ang, uint16_t time){
	//in 1000-2000
	int a;
	int t;


	//angle 0-90


	__HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, ang);
	HAL_Delay(time);


}

/*
void NAPEDY(uint16_t ang, uint16_t speed){

	uint16_t k;
	uint16_t s;
	uint16_t aimPoit;

	aimPoint = 240+90;

	if(ang > angle1_max){
		ang = angle1_max;
	}

	if(ang > angle1_min){
		ang = angle1_min;
	}

	k=ang/1000;
	s=speed/100;


	for(int i; i<k; i++){

		//__HAL_TIM_SET_COMPARE(&TIM_NO, TIM_CH_NO, s);
		//__HAL_Delay(s);

	}



}
*/


void Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct) {
    uint8_t Rec_Data[6];

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, i2c_timeout);

   // DataStruct->Gyro_X = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
  //  DataStruct->Gyro_Y = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]);
  //  DataStruct->Gyro_Z = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]);



  //  DataStruct->Gx = DataStruct->Gyro_X/ 131.0;
  //  DataStruct->Gy = DataStruct->Gyro_Y/ 131.0;
 //   DataStruct->Gz = DataStruct->Gyro_Z/ 131.0;
}

void Read_Temp(I2C_HandleTypeDef *I2Cx, float T) {
    uint8_t Rec_Data[2];
    int16_t temp;

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, TEMP_OUT_H_REG, 1, Rec_Data, 2, i2c_timeout);

    temp = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
    T = (float) ((int16_t) temp / (float) 340.0 + (float) 36.53);
}





