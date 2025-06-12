#include "CAN_Robot_Tx_Process.h"

#include "cmsis_os.h"
#include "arm_math.h"
#include "main.h"
#include "bsp_rng.h"
#include <math.h>
#include "detect_task.h"
#include "chassis_task.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
#define IS_VALID_CAN_ID(can) ((can) == 1 || (can) == 2)
#define IS_VALID_MOTOR_ID(mot) ((mot) > 0 && (mot) <= 8)

// Tihu Motor
Mit_Velocity_type Mit_receive_Velocity = {0};
Mit_Position_type Mit_receive_Position = {0};
Mit_Current_type Mit_receive_Current = {0};
Mit_Temperture_type Mit_receive_Temperature = {0}; 
Mit_State_type Mit_receive_State = {0};

// Kegu Motor
KeGu_Motor_Status_type KeguMotor_Status = {0};

//--------------const parameter variable--------------
static const uint16_t VALID_ONE_BYTE_CMDS[] = {
    0x02,  // stop motor
    0x03,  // get motor state
    0x04,  // get motor current
    0x05,  // get motor target current
    0x06,  // get motor speed
    0x07,  // get motor target speed
    0x08,  // get motor position
		0x14,  // get bus voltage
		0x17,  // get max accel
		0x18,  // get max velocity
    0x31,  // get motor temperature
    0x32,  // get board temperature
		0x54,  // get encoder offset
		0x65,  // get motor version
		0x66,  // get motor soft version
		0x0A,  // get motor errors
		0x0B,  // eliminate errors
		0x0D,  // get parameters from flash
    0x0E,  // save to flash
	  0x0F,  // restore factory settings
};

static const MotorDataParserConfig parser_configs[] = {
		{0x0A, 1.0f, &Mit_receive_State},
    {0x04, 1.0f / 1000.0f, &Mit_receive_Current},
    {0x06, 0.6f / 101.0f, &Mit_receive_Velocity},
    {0x08, 360.0f / (262144.0f), &Mit_receive_Position},
		{0x32, 1.0f, &Mit_receive_Temperature}
};

static const MotorIdMap motor_id_map[] = {
		// CAN1
    {0x01, CAN_CHANNEL_1, 0}, {0x02, CAN_CHANNEL_1, 1}, {0x03, CAN_CHANNEL_1, 2}, {0x04, CAN_CHANNEL_1, 3},
		{0x05, CAN_CHANNEL_1, 4}, {0x06, CAN_CHANNEL_1, 5}, {0x07, CAN_CHANNEL_1, 6}, {0x08, CAN_CHANNEL_1, 7},
		// CAN2
    {0x01, CAN_CHANNEL_2, 0}, {0x02, CAN_CHANNEL_2, 1}, {0x03, CAN_CHANNEL_2, 2}, {0x04, CAN_CHANNEL_2, 3},
		{0x05, CAN_CHANNEL_2, 4}, {0x06, CAN_CHANNEL_2, 5}, {0x07, CAN_CHANNEL_2, 6}, {0x08, CAN_CHANNEL_2, 7},
};


//--------------static physical layer function--------------
static void CanTransmit_can1 (uint8_t* buf, uint8_t len,uint16_t id)
{
  CAN_TxHeaderTypeDef TxHead;
  uint32_t canTxMailbox;

  if ( (buf != NULL) && (len != 0)) {
    TxHead.StdId    = id;
    TxHead.IDE      = CAN_ID_STD;
    TxHead.RTR      = CAN_RTR_DATA;
    TxHead.DLC      = len;

    while (HAL_CAN_AddTxMessage (&hcan1, &TxHead, buf, (uint32_t*) &canTxMailbox) != HAL_OK) {
      HAL_CAN_AddTxMessage (&hcan1, &TxHead, buf, (uint32_t*) &canTxMailbox);
      osDelay (1);
    }
  }
}

static void CanTransmit_can2 (uint8_t* buf, uint8_t len,uint16_t id)
{
  CAN_TxHeaderTypeDef TxHead;
  uint32_t canTxMailbox;

  if ( (buf != NULL) && (len != 0)) {
    TxHead.StdId    = id;
    TxHead.IDE      = CAN_ID_STD;
    TxHead.RTR      = CAN_RTR_DATA;
    TxHead.DLC      = len;

    while (HAL_CAN_AddTxMessage (&hcan2, &TxHead, buf, (uint32_t*) &canTxMailbox) != HAL_OK) {
      HAL_CAN_AddTxMessage (&hcan2, &TxHead, buf, (uint32_t*) &canTxMailbox);
      osDelay (1);
    }
  }
}

//--------------static tool layer function--------------
static void Tihu_SendCommand(uint8_t can_id, uint8_t motor_id, uint8_t cmd, int32_t value) {
    uint32_t raw_value = (uint32_t)value;
    uint8_t buf[5] = {
        cmd,
        (uint8_t)(raw_value >> 0),
        (uint8_t)(raw_value >> 8),
        (uint8_t)(raw_value >> 16),
        (uint8_t)(raw_value >> 24)
    };
    switch(can_id) {
        case 1:
            CanTransmit_can1(buf, sizeof(buf), motor_id);
            break;
        case 2:
            CanTransmit_can2(buf, sizeof(buf), motor_id);
            break;
        default:
            return;
    }
}

static bool is_valid_ctrl_cmd(uint16_t cmd) {
    for (size_t i = 0; i < sizeof(VALID_ONE_BYTE_CMDS)/sizeof(VALID_ONE_BYTE_CMDS[0]); i++) {
        if (VALID_ONE_BYTE_CMDS[i] == cmd) {
            return true;
        }
    }
    return false;
}

static int32_t Tihu_resolve_int32(const uint8_t *data) {
    return (data[1] << 0)  | (data[2] << 8) | 
           (data[3] << 16) | (data[4] << 24);
}

static float* get_target_field(uint32_t can_id, void *struct_ptr) {
    for (size_t i = 0; i < sizeof(motor_id_map)/sizeof(motor_id_map[0]); i++) {
        const MotorIdMap *map = &motor_id_map[i];
        if (map->can_id == can_id) {
            switch (map->channel) {
                case CAN_CHANNEL_1:
                    return &((Mit_Current_type*)struct_ptr)->can1[map->index];
                case CAN_CHANNEL_2:
                    return &((Mit_Current_type*)struct_ptr)->can2[map->index];
                default:
                    return NULL;
            }
        }
    }
    return NULL;
}

//--------------application layer function--------------
void Tihu_motor_ctrl(uint8_t can_id, uint8_t motor_id, MotorCtrlMode mode, float value) {
    uint8_t cmd = 0x00;
    int32_t converted = 0;

    switch(mode) {
        case SPEED:
            cmd = 0x1D;
            converted = (int32_t)roundf(value * 101.0f * (5.0f / 3.0f));
            break;
        case POSITION:
            cmd = 0x1E;
            converted = (int32_t)roundf(value / 360.0f * (262144.0f));
            break;
        case CURRENT:
            cmd = 0x1C;
            converted = (int32_t)roundf(value * 1000.0f);
            break;
        case SET_ID:
            cmd = 0x2E;
            converted = (int32_t)value;
            break;
        case SET_MAX_SPD:
            cmd = 0x24;
            converted = (int32_t)value;
            break;
        case SET_MIN_SPD:
            cmd = 0x25;
            converted = (int32_t)value;
            break;
				case SET_ZERO:
						cmd = 0x53;
            converted = (int32_t)value;
            break;
				default:
//						log_error("Invalid control mode: %d", mode);
						return;
    }
		
    Tihu_SendCommand(can_id, motor_id, cmd, converted);
}

void Tihu_motor_one_byte_ctrl(uint8_t can_id, uint8_t motor_id, uint8_t ctrl_cmd) {
    if (!is_valid_ctrl_cmd(ctrl_cmd)) {
        return;
    }

    uint8_t buf[1] = { (uint8_t)ctrl_cmd };
    switch(can_id) {
        case 1:
            CanTransmit_can1(buf, sizeof(buf), motor_id);
            break;
        case 2:
            CanTransmit_can2(buf, sizeof(buf), motor_id);
            break;
        default:
//            log_error("Invalid CAN ID: %d", can_id);
            break;
    }
}

int test1 = 0;
void KeGu_Motor_ctrl(uint8_t can_id, uint8_t motor_id, MotorCtrlMode mode, float value) {
    uint32_t arbitration_id = 0;
    uint8_t data[8] = {0};
    int32_t param = 0;
		uint8_t enable_mode = 0;

		// Enable mode
		if (mode == SET_INIT) {
				arbitration_id = 0x000;
				data[0] = data[1] = 0x01;
				data[2] = data[3] = data[4] = data[5] = data[6] = data[7] = 0x00;
        if(can_id == 1) {
            CanTransmit_can1(data, 8, arbitration_id);
        } else {
            CanTransmit_can2(data, 8, arbitration_id);
        }
				osDelay(5);
				// reset frame
				memset(data, 0, sizeof(data));
				arbitration_id = 0;		
		} else if (mode == SET_CUR_ENABLE || mode == SET_SPD_ENABLE) {
				// change mode to enable
				if (mode == SET_SPD_ENABLE) {
						enable_mode = 3;
				} else if (mode == SET_CUR_ENABLE) {
						enable_mode = 10;
						test1 = 200;
				}
        arbitration_id = 0x200 + motor_id;
				data[1] = (uint8_t)(enable_mode & 0xFF);
				uint8_t enable_byte = (enable_mode != 0) ? 0x0F : 0x00;
				uint16_t control_word = (uint16_t)enable_byte;
				memcpy(&data[2], &control_word, sizeof(control_word));
				int32_t velocity_limit = 3000;
				memcpy(&data[4], &velocity_limit, sizeof(velocity_limit));
        if(can_id == 1) {
            CanTransmit_can1(data, 8, arbitration_id);
        } else {
            CanTransmit_can2(data, 8, arbitration_id);
        }
				osDelay(5);
				// reset data frame
				memset(data, 0, sizeof(data));
				arbitration_id = 0;							
		} else {					
				// control mode
				switch(mode) {
						case POSITION: {
								arbitration_id = 0x500 + motor_id;
								int32_t position = (int32_t)value;
								memset(data, 0, 8);
								memcpy(data, &position, 4);
						} break;

						case SPEED: {
								arbitration_id = 0x400 + motor_id;
								int32_t speed = (int32_t)value;
								memset(data, 0, 8);
								memcpy(data, &speed, 4);
						} break;

						case CURRENT: {
//								test1 = 100;
								arbitration_id = 0x400 + motor_id;
								if (value >= 0) {
										value = (value > 500) ? 500 : value;
								} else {
										value = (value < -500) ? -500 : value;
								}

								int16_t current = (int16_t)(value / 10.0f);
								memset(data, 0, 8);
								memcpy(data + 4, &current, 2);
						} break;

						default:
								return;
				}

				if(can_id == 1) {
						CanTransmit_can1(data, 8, arbitration_id);
				} else {
						CanTransmit_can2(data, 8, arbitration_id);
				}				
		}
}

float get_motor_position(uint8_t can_id, uint8_t motor_id, MotorType type) {
    if(type == MOTOR_TYPE_KEGU) {
        return KeguMotor_Status.position[motor_id-1];
    } else {
        return (can_id == 1) ? Mit_receive_Position.can1[motor_id-1] 
                             : Mit_receive_Position.can2[motor_id-1];
    }
}



//--------------call back function--------------
uint8_t debug_data[8] = {0};
void HAL_CAN_RxFifo0MsgPendingCallback (CAN_HandleTypeDef* hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    if (hcan != &hcan1) return;

    uint32_t id = rx_header.StdId;
    uint8_t motor_id = id & 0xFF;
		uint8_t high_nibble = (id >> 8) & 0xF;
	
		if (high_nibble == 0x2) {
        if (rx_header.DLC < 6) {
            return;
        }
        int32_t speed = *((int32_t*)rx_data);
        int16_t current = *((int16_t*)(rx_data + 4));
        current *= 10.0f;

        KeguMotor_Status.speed[motor_id - 1] = speed;
        KeguMotor_Status.current[motor_id - 1] = current;				
		} else if (high_nibble == 0x3) {
        if (rx_header.DLC < 4) {
            return;
        }		
        int32_t position = *((int32_t*)rx_data);
        KeguMotor_Status.position[motor_id - 1] = position;				
		} else {
				for (size_t i = 0; i < sizeof(parser_configs)/sizeof(parser_configs[0]); i++) {
						const MotorDataParserConfig *cfg = &parser_configs[i];
						if (rx_data[0] != cfg->cmd) continue;
						
						for (uint8_t i = 0; i < 8; i++) {
								debug_data[i] = rx_data[i];
						}
						int32_t raw_value = Tihu_resolve_int32(rx_data);
						float value = raw_value * cfg->scale_factor;

						float *target = get_target_field(rx_header.StdId, cfg->data_struct_ptr);
						if (target) {
								*target = value;
						}
						break;
				}				
		}

}

void HAL_CAN_RxFifo1MsgPendingCallback (CAN_HandleTypeDef* hcan)
{
		CAN_RxHeaderTypeDef rx_header;
		uint8_t rx_data[8];

		HAL_CAN_GetRxMessage (hcan, CAN_RX_FIFO1, &rx_header, rx_data);
}

// Singleton instance
CAN_Robot_Tx_Process* CAN_Robot_Tx_Process::_singleton;

// Initialize static instance
void CAN_Robot_Tx_Process::init(void)
{
    if (_singleton == nullptr) {
        _singleton = new CAN_Robot_Tx_Process();
    }
    
    memset(&_mit_status, 0, sizeof(_mit_status));
    memset(&_kegu_status, 0, sizeof(_kegu_status));
}

// Process motor commands and create CAN frames
void CAN_Robot_Tx_Process::process_motor_command(uint8_t can_id, uint8_t motor_id, 
                                               MotorType type, MotorCtrlMode mode, 
                                               float value)
{
    AP_HAL::CANFrame frame;
    
    switch (type) {
        case MotorType::MIT:
            switch (mode) {
                case MotorCtrlMode::POSITION:
                    frame = create_mit_motor_frame(can_id, motor_id, MIT_CMD_POSITION, value);
                    break;
                case MotorCtrlMode::SPEED:
                    frame = create_mit_motor_frame(can_id, motor_id, MIT_CMD_SPEED, value);
                    break;
                case MotorCtrlMode::CURRENT:
                    frame = create_mit_motor_frame(can_id, motor_id, MIT_CMD_CURRENT, value);
                    break;
                default:
                    return;
            }
            break;
            
        case MotorType::KEGU:
            switch (mode) {
                case MotorCtrlMode::SET_INIT:
                    frame = create_kegu_motor_frame(can_id, motor_id, KEGU_CMD_INIT, value);
                    break;
                case MotorCtrlMode::SET_CUR_ENABLE:
                    frame = create_kegu_motor_frame(can_id, motor_id, KEGU_CMD_ENABLE_CUR, value);
                    break;
                case MotorCtrlMode::CURRENT:
                    frame = create_kegu_motor_frame(can_id, motor_id, KEGU_CMD_SET_CUR, value);
                    break;
                default:
                    return;
            }
            break;
    }
    
    // Log the CAN frame creation
    AP::logger().Write_MessageF("CAN_TX: ID=0x%X, Type=%u, Mode=%u, Value=%.2f",
                               (unsigned)can_id, (unsigned)type, (unsigned)mode, (double)value);
}

// Create MIT motor CAN frame
AP_HAL::CANFrame create_mit_motor_frame(uint8_t can_id, uint8_t motor_id, uint8_t cmd, float value)
{
    AP_HAL::CANFrame frame;
    frame.id = motor_id;
    frame.dlc = 8;
    
    // Convert float value to int32 based on command type
    int32_t int_value;
    switch (cmd) {
        case MIT_CMD_POSITION:
            int_value = static_cast<int32_t>(value * 100.0f); // 0.01 degree resolution
            break;
        case MIT_CMD_SPEED:
            int_value = static_cast<int32_t>(value * 100.0f); // 0.01 RPM resolution
            break;
        case MIT_CMD_CURRENT:
            int_value = static_cast<int32_t>(value * 1000.0f); // mA resolution
            break;
        default:
            int_value = static_cast<int32_t>(value);
    }
    
    frame.data[0] = cmd;
    frame.data[1] = 0;
    frame.data[2] = (int_value >> 24) & 0xFF;
    frame.data[3] = (int_value >> 16) & 0xFF;
    frame.data[4] = (int_value >> 8) & 0xFF;
    frame.data[5] = int_value & 0xFF;
    frame.data[6] = 0;
    frame.data[7] = 0;
    
    return frame;
}

// Create KEGU motor CAN frame
AP_HAL::CANFrame create_kegu_motor_frame(uint8_t can_id, uint8_t motor_id, uint8_t cmd, float value)
{
    AP_HAL::CANFrame frame;
    frame.id = motor_id;
    frame.dlc = 8;
    
    int32_t int_value = static_cast<int32_t>(value);
    
    frame.data[0] = cmd;
    frame.data[1] = (int_value >> 24) & 0xFF;
    frame.data[2] = (int_value >> 16) & 0xFF;
    frame.data[3] = (int_value >> 8) & 0xFF;
    frame.data[4] = int_value & 0xFF;
    frame.data[5] = 0;
    frame.data[6] = 0;
    frame.data[7] = 0;
    
    return frame;
}

// Process incoming MIT motor frame
void process_mit_motor_frame(const AP_HAL::CANFrame &frame, Mit_Motor_Status &status)
{
    uint8_t motor_id = frame.id & 0x0F;
    if (motor_id >= 8) return;
    
    uint8_t cmd = frame.data[0];
    int32_t value = (frame.data[1] << 24) | (frame.data[2] << 16) | 
                    (frame.data[3] << 8) | frame.data[4];
                    
    switch (cmd) {
        case 0x06: // Velocity
            status.velocity[motor_id] = value * 0.01f;
            break;
        case 0x08: // Position
            status.position[motor_id] = value * 0.01f;
            break;
        case 0x04: // Current
            status.current[motor_id] = value * 0.001f;
            break;
        case 0x32: // Temperature
            status.temperature[motor_id] = static_cast<float>(value);
            break;
    }
}

// Process incoming KEGU motor frame
void process_kegu_motor_frame(const AP_HAL::CANFrame &frame, KeGu_Motor_Status &status)
{
    uint8_t motor_id = frame.id & 0x0F;
    if (motor_id >= 8) return;
    
    uint8_t cmd = frame.data[0];
    int32_t value = (frame.data[1] << 24) | (frame.data[2] << 16) | 
                    (frame.data[3] << 8) | frame.data[4];
                    
    switch (cmd) {
        case 0x01: // Speed
            status.speed[motor_id] = value;
            break;
        case 0x02: // Current
            status.current[motor_id] = value;
            break;
        case 0x03: // Position
            status.position[motor_id] = value;
            break;
    }
}

// Get motor position based on type
float get_motor_position(uint8_t can_id, uint8_t motor_id, MotorType type)
{
    CAN_Robot_Tx_Process* processor = CAN_Robot_Tx_Process::get_singleton();
    if (!processor) return 0.0f;
    
    if (type == MotorType::MIT) {
        return processor->get_mit_status().position[motor_id - 1];
    } else {
        return processor->get_kegu_status().position[motor_id - 1];
    }
}

