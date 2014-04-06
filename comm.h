#ifndef __COMM_H
typedef struct comm_packet{


	int16_t acc_x, acc_y, acc_z;
	int16_t gyro_x, gyro_y, gyro_z;
	uint8_t header;
	uint8_t check_sum;
} comm_packet;
#define COMM_PACKET_SIZE 14
uint8_t TxBuffer[COMM_PACKET_SIZE];


#endif