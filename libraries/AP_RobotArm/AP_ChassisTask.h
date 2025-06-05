#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H
// #include "struct_typedef.h"
#include "AP_CAN_Processor.h"
#include "AP_MIT_Motor.h"
#include "AP_KinematicsLibrary.h"
// #include <math.h>
// #include <stdio.h>
// #include "bsp_delay.h"
// #include "arm_math.h"
// #include "cmsis_os.h"

void usart1_init (uint8_t* rx1_buf, uint8_t* rx2_buf, uint16_t dma_buf_num);
void USART1_Callback (void);
float generate_position (MIT_Motor::MotorInstance* motor);
extern void chassis_task (void const* pvParameters);

#endif
