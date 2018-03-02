#include "mpu6050.h"

void MPU_Init()
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO,ENABLE);

    I2C_InitTypeDef i2c_str;
    I2C_StructInit(&i2c_str);
    i2c_str.I2C_ClockSpeed=400000;
    i2c_str.I2C_Ack=I2C_Ack_Enable;
    i2c_str.I2C_AcknowledgedAddress=I2C_AcknowledgedAddress_7bit;
    i2c_str.I2C_OwnAddress1=0;
    I2C_Init(I2C1,&i2c_str);
    
    
    GPIO_InitTypeDef pb_init;
    pb_init.GPIO_Mode=GPIO_Mode_AF_OD;
    pb_init.GPIO_Pin=GPIO_Pin_6|GPIO_Pin_7;
    pb_init.GPIO_Speed=GPIO_Speed_50MHz;
    
    GPIO_Init(GPIOB,&pb_init);
    
    I2C_Cmd(I2C1,ENABLE);
}


void I2C_Transmit(I2C_TypeDef *I2Cx,uint8_t addr,uint8_t *buff,uint8_t size,uint8_t stop)
{
    I2C_GenerateSTART(I2Cx,ENABLE);
    
    while(I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT)!=SUCCESS)
        ;
    
    I2C_Send7bitAddress(I2Cx,addr,I2C_Direction_Transmitter);
    while(I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)!=SUCCESS);
    while(size>0)
    {
        I2C_SendData(I2Cx,*buff);
        while(I2C_CheckEvent(I2Cx,I2C_EVENT_MASTER_BYTE_TRANSMITTED)!=SUCCESS)
         ;
        size--;
        buff++;
    }
    if(stop!=0)
        I2C_GenerateSTOP(I2Cx,ENABLE);
}


void I2C_Receive(I2C_TypeDef *I2Cx,uint8_t addr,uint8_t *buff,uint8_t size,uint8_t stop)
{
    I2C_GenerateSTART(I2Cx,ENABLE);
    while(I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT)!=SUCCESS)
        ;
    I2C_Send7bitAddress(I2Cx,addr,I2C_Direction_Receiver);
    while(I2C_CheckEvent(I2Cx,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)!=SUCCESS)
        ;
    while(size-->0)
    {
        if(size==0)
        {
            I2C_AcknowledgeConfig(I2Cx,DISABLE);
            if(stop!=0)
            {
                I2C_GenerateSTOP(I2Cx,ENABLE);
               // I2C_AcknowledgeConfig(I2Cx,ENABLE);
            }
        }
        while(I2C_GetFlagStatus(I2C1,I2C_FLAG_RXNE)!=SET)
            ;
        *buff=I2C_ReceiveData(I2Cx);
        buff++;
    }
    I2C_AcknowledgeConfig(I2Cx,ENABLE);
}

void MPU_WriteReg(uint8_t regAddr,uint8_t data)
{
 //   I2C_ByteWrite(data,regAddr);
    uint8_t buffer[2]={regAddr,data};
    I2C_Transmit(I2C1,MPU_ADDR,buffer,2,1);
}

void MPU_ReadReg(uint8_t regAddr,uint8_t *data)
{
 //   I2C_BufferRead(I2C1,data,regAddr,1);
    I2C_Transmit(I2C1,MPU_ADDR,&regAddr,1,0);
    I2C_Receive(I2C1,MPU_ADDR,data,1,1);
}

void MPU_ReadBytes(uint8_t regAddr,uint8_t *buff,uint16_t size)
{
    I2C_Transmit(I2C1,MPU_ADDR,&regAddr,1,0);
    I2C_Receive(I2C1,MPU_ADDR,buff,size,1);
}

void MPU_ReadReg16(uint8_t regAddr,int16_t *data)
{
    uint8_t *p=(uint8_t *)data;
    MPU_ReadBytes(regAddr,(uint8_t *)data,2);
    uint8_t temp;
    temp=p[0];
    p[0]=p[1];
    p[1]=temp;
}
void MPU_WakeUp()
{
    MPU_WriteReg(MPU_PWR_MGMT1_ADDR,MPU_PWR_MGMT1_WAKEUP);
}

int MPU_isReady()
{
    uint8_t data;
    MPU_ReadReg(MPU_WHO_AM_I,&data);
    return data==MPU_ADDR>>1;
}

void MPU_SetScale(enum GYRO_SCALE gyroscale, enum ACC_SCALE accscale)
{
    MPU_WriteReg(GYRO_SCALE_ADDR,gyroscale);
    MPU_WriteReg(ACC_SCALE_ADDR,accscale);
}
void MPU_ReadAx(int16_t * pax)
{
    MPU_ReadReg16(ACC_X_ADDR,pax);
}
void MPU_ReadAy(int16_t * pay)
{
    MPU_ReadReg16(ACC_Y_ADDR,pay);
}
void MPU_ReadAz(int16_t * paz)
{
    MPU_ReadReg16(ACC_Z_ADDR,paz);
}
void MPU_ReadGx(int16_t * pgx)
{
    MPU_ReadReg16(GYRO_X_ADDR,pgx);
}
void MPU_ReadGy(int16_t * pgy)
{
    MPU_ReadReg16(GYRO_Y_ADDR,pgy);
}
void MPU_ReadGz(int16_t * pgz)
{
    MPU_ReadReg16(GYRO_Z_ADDR,pgz);
}
