 #ifndef MPU6050_H_
 #define MPU6050_H_

 #include "stm32f10x_i2c.h"
 #include "delay.h"
 
 #define MPU_PWR_MGMT1_ADDR 0x6B
 #define MPU_PWR_MGMT1_WAKEUP 0x0
 #define GYRO_SCALE_ADDR 0x1B
 #define ACC_SCALE_ADDR 0x1C
 #define MPU_ADDR 0xD0
 #define ACC_X_ADDR 0x3B
 #define ACC_Y_ADDR 0x3D
 #define ACC_Z_ADDR 0x3F
 #define GYRO_X_ADDR 0x43
 #define GYRO_Y_ADDR 0x45
 #define GYRO_Z_ADDR 0x47
 #define MPU_WHO_AM_I 0x75
 
 enum GYRO_SCALE
 {
    GYRO_SCALE_250 = 0,
    GYRO_SCALE_500 = 8,
    GYRO_SCALE_1000 = 16,
    GYRO_SCALE_2000 = 24
 };
 enum ACC_SCALE
 {
    ACC_SCALE_2G = 0,
    ACC_SCALE_4G = 8,
    ACC_SCALE_8G = 16,
    ACC_SCALE_16G = 24
 };
 struct SensorData
 {
    int16_t ax, ay, az, gx, gy, gz;
 };
 
 void MPU_Init();
 void I2C_Transmit(I2C_TypeDef *I2Cx,uint8_t addr,uint8_t *buff,uint8_t size,uint8_t stop);
 void I2C_Receive(I2C_TypeDef *I2Cx,uint8_t addr,uint8_t *buff,uint8_t size,uint8_t stop);
 
 void MPU_ReadBytes(uint8_t regAddr,uint8_t *buff,uint16_t size);
 void MPU_ReadReg(uint8_t regAddr,uint8_t *data);
 void MPU_ReadReg16(uint8_t regAddr,int16_t *data);
 
 int MPU_isReady();
 void MPU_SetScale(enum GYRO_SCALE gyroscale, enum ACC_SCALE accscale);
 void MPU_ReadAx(int16_t * pax);
 void MPU_ReadAy(int16_t * pay);
 void MPU_ReadAz(int16_t * paz);
 void MPU_ReadGx(int16_t * pgx);
 void MPU_ReadGy(int16_t * pgy);
 void MPU_ReadGz(int16_t * pgz);
 void MPU_WakeUp();

 #endif
