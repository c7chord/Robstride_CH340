#include "robstride_driver.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <termios.h> // NEED TO BE ADDED (IDK)

int main() {
    std::cout << "=== Robstride Driver : Set Test ===" << std::endl;
    RobstrideDriver motor(1, HOST_ID);

    // Serial Open
    std::cout << "=== Serial Open ===" << std::endl;
    int fd = motor.open_serial(B921600); 
    
    // get variables : CAN ID, MODE
    std::cout << "=== Get Variables ===" << std::endl;
    std::cout<< "CAN ID : " << static_cast<int>(motor.get_motor_can_id()) << std::endl;
    int mode = static_cast<int>(motor.get_run_mode());
    std::string mode_str;
    if (mode == MODE_MOTION) mode_str = "MOTION";
    else if (mode == MODE_POSITION) mode_str = "POSITION";
    else if (mode == MODE_SPEED) mode_str = "SPEED";
    else mode_str = "Error";
    std::cout<< "MODE : " << mode_str << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(5));        

    // Can ID Test
    std::cout << "=== Can ID Test ===" << std::endl;
    std::cout << "=== Current Can ID : " << static_cast<int>(motor.get_motor_can_id()) << std::endl;
    
    std::cout << "=== Setting Motor Can ID to 2 ===" << std::endl;
    motor.set_motor_can_id(2);
    std::this_thread::sleep_for(std::chrono::seconds(3));
    std::cout << "=== Current Can ID : " << static_cast<int>(motor.get_motor_can_id()) << std::endl;

    std::cout << "=== Setting Motor Can ID back to 1 ===" << std::endl;
    motor.set_motor_can_id(1);
    std::this_thread::sleep_for(std::chrono::seconds(3));
    std::cout << "=== Current Can ID : " << static_cast<int>(motor.get_motor_can_id()) << std::endl;
    

    // POSITION MODE TEST
    std::cout << "=== POSITION MODE TEST ===" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(5));
    std::cout << "=== Motor Setup to Position Mode ===" << std::endl;
    motor.init_motor(MODE_POSITION);
    std::cout << "=== Motor Enable ===" << std::endl;
    motor.enable_motor();
    std::cout << "=== Set Position Reference to 1.0 ===" << std::endl;
    motor.set_position_ref(0.5f);
    std::this_thread::sleep_for(std::chrono::seconds(5));
    std::cout << "=== Set Position Reference to 0.0 ===" << std::endl; // 이거 안돼
    motor.set_position_ref(0.0f);
    std::this_thread::sleep_for(std::chrono::seconds(5));
    std::cout << "=== Motor Stop ===" << std::endl;
    motor.stop_motor();

    //SPEED MODE TEST
    std::cout << "=== SPEED MODE TEST ===" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(5));
    std::cout << "=== Motor Setup to Speed Mode ===" << std::endl;
    motor.init_motor(MODE_SPEED);
    std::cout << "=== Motor Enable ===" << std::endl;
    motor.enable_motor();
    std::cout << "=== Set Speed Reference to 5.0 ===" << std::endl;
    motor.set_speed_ref(5.0f);
    std::this_thread::sleep_for(std::chrono::seconds(5));
    std::cout << "=== Set Speed Reference to -5.0 ===" << std::endl;
    motor.set_speed_ref(-5.0f);
    std::this_thread::sleep_for(std::chrono::seconds(10));
    std::cout << "=== Get Speed ===" << std::endl;
    motor.request_status();
    float result = motor.get_status().speed;
    std::cout << "=== Speed : " << result << std::endl;
    std::cout << "=== Motor Stop ===" << std::endl;
    motor.stop_motor();
    
    // TORQUE MODE TEST
    std::cout << "=== TORQUE MODE TEST ===" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(5));
    std::cout << "=== Motor Setup to Torque Mode ===" << std::endl;
    motor.init_motor(MODE_CURRENT);
    std::cout << "=== Motor Enable ===" << std::endl;
    motor.enable_motor();
    std::cout << "=== Set Torque (Current) Reference to 0.2 ===" << std::endl;
    motor.set_current_ref(0.2f);
    std::this_thread::sleep_for(std::chrono::seconds(5));
    std::cout << "=== Motor Stop ===" << std::endl;
    motor.stop_motor();
    
    // Motion Mode TEST
    std::cout << "=== MOTION MODE TEST ===" << std::endl;  // 이거 안돼
    std::this_thread::sleep_for(std::chrono::seconds(5));
    std::cout << "=== Motor Setup to Motion Mode ===" << std::endl;
    motor.init_motor(MODE_MOTION);
    std::cout << "=== Motor Enable ===" << std::endl;
    motor.enable_motor();
    std::cout << "=== Set Motion Reference to POS 5.0, with VEL 1.0, TQ 0.5 ===" << std::endl;
    RobstrideMotionCommand cmd;
    cmd.position = 5.0f;
    cmd.speed = 4.0f;
    cmd.torque = 0.05f;
    cmd.kp = 0.3f;
    cmd.kd = 0.01f;
    motor.send_motion_control(cmd);
    std::this_thread::sleep_for(std::chrono::seconds(5));
    std::cout << "=== Set Motion Reference to POS 0.0 ===" << std::endl;
    cmd.position = 0.0f;
    motor.send_motion_control(cmd);
    std::this_thread::sleep_for(std::chrono::seconds(5));
    std::cout << "=== Motor Stop ===" << std::endl;
    motor.stop_motor();

    // Close Serial
    std::cout << "=== Serial Close ===" << std::endl;
    motor.close_serial(fd);
    std::cout << "=== Test Complete ===" << std::endl;
    
    return 0;
}
