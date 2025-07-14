# Robstride_CH340
Repository for Robstride &amp; Xiaomi Cybergear, for CH34X communication between UART-USB

# Intro
This Repository is needed when your USB2CAN device is detected as CH34X driver.
Note : CH340, CH341 all works, as long as the device is detected with the format of : CH34X

However, This repository is NOT RECOMMENDED for practical purpose (Robot Actuator, Differential Drive System, ...) since UART communication via CH340 Driver has relatively low communication frequency compared to direct USB2CAN communication.

The following communication Type is Recommended.

# Hardware Setup
MCU : Raspberry Pi4 (Linux 24.04 LTS)
USB2CAN : [link](https://www.aliexpress.us/item/1005004296661528.html?gatewayAdapt=4itemAdapt) (Communication : CH340 UART)
Motor : Robstride or Xiaomi Cybergear (Uses the Same CAN message protocol)

The following is the wiring diagram.

# Software Setup
Make sure Every hardware is well-connected.

Check CH340 device : 
```bash
lsusb
```
Expected Result :
```bash
...
Bus 001 Device 011: ID 1a86:7523 QinHeng Electronics CH340 serial converter
...
```

<br>

Check ttyUSB : 
```bash
ls /dev/ttyUSB*
```
Expected Result : 
```bash
/dev/ttyUSBX (X : 0, 1, 2 ...)
```
Modify robstride_params.h :
```cpp
#define SERIAL_NAME                 "/dev/ttyUSBX" // default setting : 0
```

<br>

Run example :
```bash
cd my_robstride
g++ -o test robstride_driver.cpp test.cpp && ./test
```

<br>
# Hardware Setup
**connection with /dev/ttyUSBX
1. Check and modify ls /dev/ttyUSB* : might be altered in case of motor power dis/reconncection, etc...
2. kill process related to /dev/ttyUSBX

```bash
fuser -k /dev/ttyUSBX (X : 0, 1, 2 ...)
```

