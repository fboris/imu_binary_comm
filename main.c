#include "bool.h"
#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include "MPU6050.h"
#include "usart.h"
#include <stdio.h>
#include "comm.h"
void delay(uint32_t delay_count)
{
	while (delay_count) delay_count--;
}
void init_led()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	/* GPIOA Periph clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	/* Configure PA0 and PA1 in output pushpull mode */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}
void gpio_toggle(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIOx->ODR ^= GPIO_Pin;
}
void *memcpy(void *dest, const void *src, size_t n)
{
        void *ret = dest;

        //Cut rear
        uint8_t *dst8 = dest;
        const uint8_t *src8 = src;
        switch (n % 4) {
                case 3 : *dst8++ = *src8++;
                case 2 : *dst8++ = *src8++;
                case 1 : *dst8++ = *src8++;
                case 0 : ;
        }

        //stm32 data bus width
        uint32_t *dst32 = (void *)dst8;
        const uint32_t *src32 = (void *)src8;
        n = n / 4;
        while (n--) {
                *dst32++ = *src32++;
        }

        return ret;
}
void generate_package(comm_package* pack, uint8_t* buff)
{
	memcpy( &(buff[0]), &(pack->acc_x), sizeof(int16_t) );
	memcpy( &(buff[2]), &(pack->acc_y), sizeof(int16_t) );
	memcpy( &(buff[4]), &(pack->acc_z), sizeof(int16_t) );
	memcpy( &(buff[6]), &(pack->gyro_x), sizeof(int16_t) );
	memcpy( &(buff[8]), &(pack->gyro_y), sizeof(int16_t) );
	memcpy( &(buff[10]), &(pack->gyro_z), sizeof(int16_t) );
	memcpy( &(buff[12]), &(pack->header), sizeof(uint8_t));

}
int main(void)
{
	int16_t buff[6];
	uint8_t bin_buff[13];
	comm_package imu_comm;
	imu_comm.header = (uint8_t)'I';
	init_led();
	init_usart1();
	MPU6050_I2C_Init();
	MPU6050_Initialize();
	if( MPU6050_TestConnection() == TRUE)
	{
	   puts("connection success\r\n");
	}else {
	   puts("connection failed\r\n");
	}

	while (1) {
		//puts("running now\r\n");
		MPU6050_GetRawAccelGyro(buff);
		imu_comm.acc_x = buff[0];
		imu_comm.acc_y = buff[1];
		imu_comm.acc_z = buff[2];
		imu_comm.gyro_x = buff[3];
		imu_comm.gyro_y = buff[4];
		imu_comm.gyro_z = buff[5];
		generate_package( &imu_comm, &bin_buff[0]);
		for (int i = 0 ; i<13 ; i++)
			send_byte( bin_buff[i] );
		gpio_toggle(GPIOA, GPIO_Pin_0);
		gpio_toggle(GPIOA, GPIO_Pin_1);
		delay(5000);

	}
}
