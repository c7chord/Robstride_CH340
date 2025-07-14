#ifndef ROBSTRIDE_DRIVER_H
#define ROBSTRIDE_DRIVER_H

#include "robstride_params.h"
#include <cstdint>
#include <string>
#include <vector>

struct RobstrideStatus {
    float position;
    float speed;
    float torque;
    uint16_t temperature;
};

struct RobstrideMotionCommand {
    float position;
    float speed;
    float torque;
    float kp;
    float kd;
};

class RobstrideDriver {
    public:
        // CONSTRUCTOR & DESTRUCTOR
        RobstrideDriver();
        RobstrideDriver(int cybergear_can_id, uint16_t master_can_id);
        virtual ~RobstrideDriver();

        // SERIAL VARIABLES & FUNCTIONS
        int fd {0};
        int open_serial(int baud_rate);
        void close_serial(int fd);
        
        // MOTOR CONTROL FUNCTIONS
        void init_motor(uint8_t mode);
        void enable_motor();
        void stop_motor();

        // SET FUNCTIONS
        void set_run_mode(uint8_t mode);
        void set_limit_speed(float speed);
        void set_limit_current(float current);
        void set_limit_torque(float torque);
        void set_motor_can_id(uint8_t can_id); // POTENTAILLY NEED TO BE FIXED

        // GET FUNCTIONS
        uint8_t get_run_mode() const;
        uint8_t get_motor_can_id() const; 
        bool request_status(); // NEED TO BE FIXED
        RobstrideStatus get_status() const;

        // MODE_MOTION
        void send_motion_control(RobstrideMotionCommand cmd);

        // MODE_CURRENT
        void set_current_kp(float kp);
        void set_current_ki(float ki);
        void set_current_filter_gain(float gain);
        void set_current_ref(float current);

        // MODE_POSITION
        void set_position_kp(float kp);
        void set_position_ref(float position);

        // MODE_SPEED
        void set_speed_kp(float kp);
        void set_speed_ki(float ki);
        void set_speed_ref(float speed);
        
    private:
        // TYPE CONVERSION FUNCTIONS
        uint16_t _float_to_uint(float x, float x_min, float x_max, int bits);
        float _uint_to_float(uint16_t x, float x_min, float x_max);
        float _bytes_to_float_le(uint8_t *data);
        std::string bytes_to_hex_string(const std::vector<uint8_t>& bytes);
        std::vector<uint8_t> hex_string_to_bytes(const std::string& hex);
        
        // SEND FUNCTIONS
        void _send_can_package(const int motor_id, const int cmd, uint16_t option, const int msg_len, uint8_t *msg);
        void _send_can_float_package(const int motor_id, uint16_t option, uint16_t addr, float value, float min, float max);

        // DATA MANIPULATION FUNCTIONS
        std::string can_id(const int motor_id, const int cmd, uint16_t option);
        std::string packet(std::string can_id, const int msg_len, uint8_t msg[8]);  
        void parse_status_response(const std::vector<uint8_t> &data);

        // VARIABLES
        const char* _serial_name;
        int _cybergear_can_id;
        uint16_t _master_can_id;
        uint8_t _run_mode;
        bool _use_serial_debug;
        RobstrideStatus _status;
};

#endif // ROBSTRIDE_DRIVER_H