// SEARCH : "NEED TO BE FIXED"
// cd /home/poko49/experiments/my_robstride && g++ -o test robstride_driver.cpp && ./test
#include "robstride_driver.h"
#include "robstride_params.h"
#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>
#include <vector>
#include <chrono>
#include <thread>
#include <memory>
#include <cstring>
#include <sys/time.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>

/* CONSTRUCTOR & DESTRUCTOR */
RobstrideDriver::RobstrideDriver() {};
RobstrideDriver::RobstrideDriver(int cybergear_can_id, uint16_t master_can_id) 
    : _cybergear_can_id(cybergear_can_id),
    _master_can_id(master_can_id),
    _run_mode(MODE_MOTION),
    _use_serial_debug(false),
    _serial_name(SERIAL_NAME)
{
    _status.position = 0.0f;
    _status.speed = 0.0f;
    _status.torque = 0.0f;
    _status.temperature = 0;
}

RobstrideDriver::~RobstrideDriver(){}

/* SERIAL FUNCTIONS */
int RobstrideDriver::open_serial(int baud_rate) {
    
    // OPEN SERIAL PORT
    struct termios newtermios;
    fd = open(_serial_name, O_RDWR | O_NOCTTY); 
    if (fd == -1) {
        std::cout << "❌ SERIAL PORT OPEN FAILED" << SERIAL_NAME << std::endl;
        return -1;
    }

    // SET SERIAL PORT
    newtermios.c_cflag= CBAUD | CS8 | CLOCAL | CREAD;
    newtermios.c_iflag=IGNPAR;
    newtermios.c_oflag=0;
    newtermios.c_lflag=0;
    newtermios.c_cc[VMIN]=1;
    newtermios.c_cc[VTIME]=0;
    cfsetospeed(&newtermios,baud_rate);
    cfsetispeed(&newtermios,baud_rate);
    if (tcflush(fd,TCIFLUSH)==-1) return -1;
    if (tcflush(fd,TCOFLUSH)==-1) return -1;
    if (tcsetattr(fd,TCSANOW,&newtermios)==-1) return -1;

    std::cout << "✅ Serial PORT OPEN SUCCESS (fd=" << fd << ")" << std::endl;
    
    // SHOOT AT handshake
    const char* handshake = "AT+AT\r\n";
    fsync(fd);
    
    return fd;
}    
void RobstrideDriver::close_serial(int fd) {
 close(fd);
}
/* MOTOR CONTROL FUNCTIONS */
void RobstrideDriver::init_motor(uint8_t mode){
    // STOP MOTOR
    stop_motor();

    // SET MOTOR
    set_run_mode(mode);
    set_limit_speed(V_MAX);
    set_limit_current(I_MAX);
    set_position_ref(0.0f);
}
void RobstrideDriver::enable_motor(){
    uint8_t data[8] = {0x00};
    _send_can_package(_cybergear_can_id, CMD_ENABLE, _master_can_id, 8, data);
}
void RobstrideDriver::stop_motor(){
    uint8_t data[8] = {0x00};
    _send_can_package(_cybergear_can_id, CMD_STOP, _master_can_id, 8, data);
}

/* SET FUNCTIONS */
void RobstrideDriver::set_motor_can_id(uint8_t can_id){
    uint8_t data[8] = {0x00};
    uint16_t option = can_id << 8 | _master_can_id;
    _send_can_package(_cybergear_can_id, CMD_SET_CAN_ID, option, 8, data);
    _cybergear_can_id = can_id;
}
void RobstrideDriver::set_run_mode(uint8_t mode){
    _run_mode = mode;
    uint8_t data[8] = {0x00};
    data[0] = ADDR_RUN_MODE & 0x00FF;
    data[1] = ADDR_RUN_MODE >> 8;
    data[4] = mode;
    _send_can_package(_cybergear_can_id, CMD_RAM_WRITE, _master_can_id, 8, data);
}
void RobstrideDriver::set_limit_speed(float speed){
    _send_can_float_package(_cybergear_can_id, HOST_ID, ADDR_LIMIT_SPEED, speed, 0.0f, V_MAX);
}
void RobstrideDriver::set_limit_current(float current){
    _send_can_float_package(_cybergear_can_id, HOST_ID, ADDR_LIMIT_CURRENT, current, 0.0f, I_MAX);
}
void RobstrideDriver::set_limit_torque(float torque){
    _send_can_float_package(_cybergear_can_id, HOST_ID, ADDR_LIMIT_TORQUE, torque, 0.0f, T_MAX);
}

/* GET FUNCTIONS */
uint8_t RobstrideDriver::get_run_mode() const {
    return _run_mode;
}
uint8_t RobstrideDriver::get_motor_can_id() const {
    return _cybergear_can_id;
}
// NEED TO BE FIXED
bool RobstrideDriver::request_status() {
    // SEND REQUEST MESSAGE
    uint8_t data[8] = {0x00};
    _send_can_package(_cybergear_can_id, CMD_GET_STATUS, _master_can_id, 8, data);

    // RECEIVE RESPONSE MESSAGE
    uint8_t recv_buf[64] = {0};
    size_t total_read = 0;
    auto start = std::chrono::steady_clock::now();

    // RECEIVE LOOP
    while (total_read < sizeof(recv_buf)) {
        int ret = read(fd, recv_buf + total_read, sizeof(recv_buf) - total_read);
        if (ret > 0) {
            total_read += ret;
        }

        auto now = std::chrono::steady_clock::now();
        int elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
        if (elapsed > 100) break;

        if (ret <= 0) std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    if (total_read == 0) {
        std::cerr << "Error: No response received in request_status()" << std::endl;
        return false;
    }

    // DEBUGGING
    std::cout << "total_read : " << total_read << std::endl;
    std::cout << "read data : " << bytes_to_hex_string(std::vector<uint8_t>(recv_buf, recv_buf + total_read)) << std::endl;
    
    // PARSE RESPONSE MESSAGE
    for (size_t i = 0; i < total_read - 2; ++i) {
        if (recv_buf[i] == 'A' && recv_buf[i+1] == 'T') {
            // COMMAND FORMAT (17BTYES) : AT(2) + CAN ID(4) + LEN(1) + DATA(8) + 0D0A(2)
            if (i + 4 + 1 + 8 + 2 <= total_read) {
                uint8_t* full_message = recv_buf + i;
                uint8_t* payload = full_message + 7;  // AT(2) + CAN ID(4) + LEN(1)

                // DEBUGGING
                std::cout << "=== Received Status Response ===" << std::endl;
                for (int k = 0; k < 8; ++k) {
                    printf("%02X ", payload[k]);
                }
                std::cout << std::endl;

                // PARSE RESPONSE MESSAGE
                parse_status_response(std::vector<uint8_t>(payload, payload + 8));
                return true;
            }
        }
    }
    std::cerr << "Error: Valid AT packet not found in response" << std::endl;
    return true;
}
RobstrideStatus RobstrideDriver::get_status() const {
    return _status;
}
/* MODE_MOTION */
void RobstrideDriver::send_motion_control(RobstrideMotionCommand cmd){
    uint8_t data[8] = {0x00};

    uint16_t position = _float_to_uint(cmd.position, POS_MIN, POS_MAX, 16);
    data[0] = position >> 8;
    data[1] = position & 0x00FF;

    uint16_t speed = _float_to_uint(cmd.speed, V_MIN, V_MAX, 16);
    data[2] = speed >> 8;
    data[3] = speed & 0x00FF;

    uint16_t kp = _float_to_uint(cmd.kp, KP_MIN, KP_MAX, 16);
    data[4] = kp >> 8;
    data[5] = kp & 0x00FF;

    uint16_t kd = _float_to_uint(cmd.kd, KD_MIN, KD_MAX, 16);
    data[6] = kd >> 8;
    data[7] = kd & 0x00FF;

    uint16_t torque = _float_to_uint(cmd.torque, T_MIN, T_MAX, 16);

    _send_can_package(_cybergear_can_id, CMD_POSITION, torque, 8, data);
}

/* MODE_CURRENT */
void RobstrideDriver::set_current_kp(float kp){
    _send_can_float_package(_cybergear_can_id, HOST_ID, ADDR_CURRENT_KP, kp, KP_MIN, KP_MAX);
}
void RobstrideDriver::set_current_ki(float ki){
    _send_can_float_package(_cybergear_can_id, HOST_ID, ADDR_CURRENT_KI, ki, KI_MIN, KI_MAX);
}
void RobstrideDriver::set_current_filter_gain(float gain){
    _send_can_float_package(_cybergear_can_id, HOST_ID, ADDR_CURRENT_FILTER_GAIN, gain, CURRENT_FILTER_GAIN_MIN, CURRENT_FILTER_GAIN_MAX);
}
void RobstrideDriver::set_current_ref(float current){
    _send_can_float_package(_cybergear_can_id, HOST_ID, ADDR_I_REF, current, I_MIN, I_MAX);
}

/* MODE_POSITION */
void RobstrideDriver::set_position_kp(float kp){
    _send_can_float_package(_cybergear_can_id, HOST_ID, ADDR_POSITION_KP, kp, KP_MIN, KP_MAX);
}
void RobstrideDriver::set_position_ref(float position){
    _send_can_float_package(_cybergear_can_id, HOST_ID, ADDR_POSITION_REF, position, POS_MIN, POS_MAX);
}

/* MODE_SPEED */
void RobstrideDriver::set_speed_kp(float kp){
    _send_can_float_package(_cybergear_can_id, HOST_ID, ADDR_SPEED_KP, kp, KP_MIN, KP_MAX);
}
void RobstrideDriver::set_speed_ki(float ki){
    _send_can_float_package(_cybergear_can_id, HOST_ID, ADDR_SPEED_KI, ki, KI_MIN, KI_MAX);
}
void RobstrideDriver::set_speed_ref(float speed){
    _send_can_float_package(_cybergear_can_id, HOST_ID, ADDR_SPEED_REF, speed, V_MIN, V_MAX);
}

/* TYPE CONVERSION FUNCTIONS */
uint16_t RobstrideDriver::_float_to_uint(float x, float x_min, float x_max, int bits){
    if (bits>16) bits=16;
    float span = x_max - x_min;
    float offset = x_min;
    if(x > x_max) x = x_max;
    else if(x < x_min) x = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}
float RobstrideDriver::_uint_to_float(uint16_t x, float x_min, float x_max){
    uint16_t type_max = 0xFFFF;
    float span = x_max - x_min;
    return (float) x / type_max * span + x_min;
}

std::vector<uint8_t> RobstrideDriver::hex_string_to_bytes(const std::string& hex_string) {
    std::vector<uint8_t> bytes;
    
    // CHECK STRING LENGTH
    if (hex_string.length() % 2 != 0) {
        std::cerr << "Error: HEX string length must be even" << std::endl;
        return bytes;
    }
    
    // CONVERT HEX STRING TO BYTES
    for (size_t i = 0; i < hex_string.length(); i += 2) {
        std::string byte_string = hex_string.substr(i, 2);
        uint8_t byte = static_cast<uint8_t>(std::stoi(byte_string, nullptr, 16));
        bytes.push_back(byte);
    }   
    return bytes;
}

std::string RobstrideDriver::bytes_to_hex_string(const std::vector<uint8_t>& bytes) {
    std::stringstream ss;
    for (uint8_t byte : bytes) {
        ss << std::hex << std::setw(2) << std::setfill('0') << (int)byte;
    }
    return ss.str();
}

float RobstrideDriver::_bytes_to_float_le(uint8_t *data) {
    uint8_t reversed[4] = { data[3], data[2], data[1], data[0] };
    float result;
    std::memcpy(&result, reversed, sizeof(float));
    return result;
}

/* DATA MANIPULATION FUNCTIONS */
void RobstrideDriver::parse_status_response(const std::vector<uint8_t>& data) {
    
    // CHECK DATA SIZE
    if (data.size() < 8) {
        std::cerr << "Error: Insufficient data size in parse_status_response" << std::endl;
        return;
    }
    
    // PARSE RESPONSE MESSAGE : BIG ENDIAN
    uint16_t raw_position = data[1] | data[0] << 8;
    uint16_t raw_speed = data[3] | data[2] << 8;
    uint16_t raw_torque = data[5] | data[4] << 8;
    uint16_t raw_temperature = data[7] | data[6] << 8;

    // APPLY DATA TO STATUS
    _status.position = _uint_to_float(raw_position, POS_MIN, POS_MAX);
    _status.speed = _uint_to_float(raw_speed, V_MIN, V_MAX);
    _status.torque = _uint_to_float(raw_torque, T_MIN, T_MAX);
    _status.temperature = raw_temperature;

    // DEBUGGING
    std::cout << "position : " << _status.position
              << " speed : " << _status.speed
              << " torque : " << _status.torque
              << " temperature : " << _status.temperature
              << std::endl;
}

void RobstrideDriver::_send_can_package(const int motor_id, const int cmd, uint16_t option, const int msg_len, uint8_t *msg){
    // CREATE PACKET & POINTER
    std::unique_ptr<std::string> ptr = std::make_unique<std::string>(packet(can_id(motor_id, cmd, option), msg_len, msg));

    // CONVERT PACKET TO HEX STRING
    std::vector<uint8_t> packet_hex = this->hex_string_to_bytes(*ptr); 
    std::cout << "sent message : " << *ptr << std::endl;

    // SEND PACKET
    write(fd, packet_hex.data(), packet_hex.size());
    fsync(fd);

    // WAIT IF NEEDED 
    // std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

void RobstrideDriver::_send_can_float_package(const int motor_id, uint16_t option, uint16_t addr, float value, float min, float max){
    uint8_t data[8] = {0x00};
    data[0] = addr & 0x00FF;
    data[1] = addr >> 8;

    float val = (max < value) ? max : value;
    val = (min > value) ? min : value;
    memcpy(&data[4], &val, 4);
    _send_can_package(motor_id, CMD_RAM_WRITE, option, 8, data);
}

std::string RobstrideDriver::can_id(const int motor_id, const int cmd, uint16_t option) { 
    std::stringstream ss;

    // CREATE CAN ID
    int _motor_id = motor_id << 3;
    _motor_id = _motor_id + 4;
    ss << std::setfill('0') << std::setw(2) << std::hex << cmd;
    ss << std::setfill('0') << std::setw(4) << std::hex << option;
    ss << std::setfill('0') << std::setw(2) << std::hex << _motor_id;

    return ss.str();   
}

std::string RobstrideDriver::packet(std::string can_id, const int msg_len, uint8_t *msg) {
    std::stringstream ss;

    // CREATE PACKET
    // COMMAND FORMAT (17BTYES) : AT(2) + CAN ID(4) + LEN(1) + DATA(8) + 0D0A(2)
    ss << "4154"; // AT
    ss << can_id; // CAN ID
    ss << std::setfill('0') << std::setw(2) << msg_len; // LEN
    for (int i = 0; i < msg_len; i++) {
        ss << std::setfill('0') << std::setw(2) << std::hex << (int)msg[i]; // DATA
    }
    ss << "0d0a"; // 0D0A   
    return ss.str();  
}