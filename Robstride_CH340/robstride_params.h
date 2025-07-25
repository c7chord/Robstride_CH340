#ifndef ROBSTRIDE_PARAMS_H
#define ROBSTRIDE_PARAMS_H

#define PI                         3.14159265358979323846
#define HOST_ID                    2024
#define SERIAL_NAME                 "/dev/ttyUSB0"
#define BAUDRATE                    921600

#define CMD_POSITION                  12
#define CMD_REQUEST                   16
#define CMD_ENABLE                    24
#define CMD_STOP                      32
#define CMD_SET_MECH_POSITION_TO_ZERO 48
#define CMD_GET_STATUS                120
#define CMD_RAM_READ                  136
#define CMD_RAM_WRITE                 144
#define CMD_SET_CAN_ID                152


#define ADDR_SPEED_KP              0x2014
#define ADDR_SPEED_KI              0x2015
#define ADDR_POSITION_KP           0x2016
#define ADDR_RUN_MODE              0x7005
#define ADDR_I_REF                 0x7006
#define ADDR_SPEED_REF             0x700A
#define ADDR_LIMIT_TORQUE          0x700B
#define ADDR_CURRENT_KP            0x7010
#define ADDR_CURRENT_KI            0x7011
#define ADDR_CURRENT_FILTER_GAIN   0x7014
#define ADDR_POSITION_REF          0x7016
#define ADDR_LIMIT_SPEED           0x7017
#define ADDR_LIMIT_CURRENT         0x7018

#define MODE_MOTION                  0x00
#define MODE_POSITION                0x01
#define MODE_SPEED                   0x02
#define MODE_CURRENT                 0x03

#define POS_MIN                   -12.5f
#define POS_MAX                    12.5f
#define V_MIN                     -30.0f
#define V_MAX                      30.0f
#define KP_MIN                      0.0f
#define KP_MAX                    500.0f
#define KI_MIN                      0.0f
#define KI_MAX                     10.0f
#define KD_MIN                      0.0f
#define KD_MAX                      5.0f
#define T_MIN                     -12.0f
#define T_MAX                      12.0f
#define I_MIN                     -27.0f
#define I_MAX                      27.0f
#define CURRENT_FILTER_GAIN_MIN     0.0f
#define CURRENT_FILTER_GAIN_MAX     1.0f

#define RET_ROBSTRIDE_ERROR          -0x01
#define RET_ROBSTRIDE_OK              0x00
#define RET_ROBSTRIDE_MSG_NOT_AVAIL   0x01
#define RET_ROBSTRIDE_INVALID_CAN_ID  0x02
#define RET_ROBSTRIDE_INVALID_PACKET  0x03


#endif //
