#ifndef __COMM_H
typedef struct comm_packet{


	int16_t acc_x, acc_y, acc_z;
	int16_t gyro_x, gyro_y, gyro_z;
	uint8_t header;
	uint8_t check_sum;
} comm_packet;
typedef enum {
	BUFF_1 = 0,
	BUFF_2
}buffer_status;
#define COMM_PACKET_SIZE 14
uint8_t TX_BUFFER_1[COMM_PACKET_SIZE];
uint8_t TX_BUFFER_2[COMM_PACKET_SIZE];
buffer_status CURR_DOUBLE_BUFF;
#endif