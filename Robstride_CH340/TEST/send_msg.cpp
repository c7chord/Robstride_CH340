// serial 통신하는 함수 모음
// cd /home/poko49/experiments/my_robstride && g++ -o send_msg send_msg.cpp && ./send_msg
#include <sys/time.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>

#include <iostream>
#include <sys/types.h>
#include <sys/stat.h>

int serial_open(char *serial_name, int baud_rate)
{
      struct termios newtermios;
      int fd = open(serial_name, O_RDWR | O_NOCTTY);
      if (fd == -1) return -1;
      
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
      return fd;
}    

void serial_close(int fd)
{
   close(fd);
}

void serial_send(int serial_fd, char *data, int size)
{
  write(serial_fd, data, size);
}

int serial_read(int serial_fd, char *data, int size, int timeout_usec)
{
      fd_set fds;
      struct timeval timeout;
      int count=0;
      int ret;
      int n;
      do {
        FD_ZERO(&fds);
        FD_SET (serial_fd, &fds);
        timeout.tv_sec = 0;
        timeout.tv_usec = timeout_usec;
        ret=select (FD_SETSIZE,&fds, NULL, NULL,&timeout);
        if (ret==1) {
          n=read (serial_fd, &data[count], size-count);
          count+=n;
          data[count]=0;
   }
 } while (count<size && ret==1);
 return count;
}

int main() {
    std::cout << "=== Serial Communication Test ===" << std::endl;
    
    // 시리얼 포트 열기
    const char* serial_name = "/dev/ttyUSB0";
    int baud_rate = B921600;
    
    std::cout << "Opening serial port " << serial_name << " at " << baud_rate << " baud..." << std::endl;
    int serial_fd = serial_open(const_cast<char*>(serial_name), baud_rate);
    
    if (serial_fd == -1) {
        std::cout << "❌ Failed to open serial port " << serial_name << std::endl;
        std::cout << "Please check:" << std::endl;
        std::cout << "1. USB device is connected" << std::endl;
        std::cout << "2. You have permission to access " << serial_name << std::endl;
        std::cout << "3. No other program is using the port" << std::endl;
        return -1;
    }
    
    std::cout << "✅ Serial port opened successfully (fd=" << serial_fd << ")" << std::endl;
    
    // AT 핸드셰이크 전송
    std::cout << "\nSending AT handshake..." << std::endl;
    const char* handshake = "AT+AT\r\n";
    serial_send(serial_fd, const_cast<char*>(handshake), strlen(handshake));
    std::cout << "Sent: " << handshake;
    
    // 응답 대기
    char response[256] = {0};
    int bytes_read = serial_read(serial_fd, response, sizeof(response)-1, 1000000); // 1초 대기
    
    if (bytes_read > 0) {
        std::cout << "Received: " << response << " (bytes: " << bytes_read << ")" << std::endl;
    } else {
        std::cout << "No response received" << std::endl;
    }
    
    // 테스트 메시지 전송
    std::cout << "\nSending test message..." << std::endl;
    const char* test_msg = "Hello from send_msg.cpp\r\n";
    serial_send(serial_fd, const_cast<char*>(test_msg), strlen(test_msg));
    std::cout << "Sent: " << test_msg;
    
    // 응답 대기
    memset(response, 0, sizeof(response));
    bytes_read = serial_read(serial_fd, response, sizeof(response)-1, 1000000);
    
    if (bytes_read > 0) {
        std::cout << "Received: " << response << " (bytes: " << bytes_read << ")" << std::endl;
    } else {
        std::cout << "No response received" << std::endl;
    }
    
    // 시리얼 포트 닫기
    std::cout << "\nClosing serial port..." << std::endl;
    serial_close(serial_fd);
    
    std::cout << "✅ Test completed successfully!" << std::endl;
    
    return 0;
}
