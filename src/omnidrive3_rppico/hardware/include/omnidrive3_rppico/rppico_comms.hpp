#ifndef omnidrive3_RPPICO__OMNIBOT_COMMS_HPP_
#define omnidrive3_RPPICO__OMNIBOT_COMMS_HPP_

#include <sstream>
#include <libserial/SerialPort.h>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include <sys/stat.h>

//what's needed for socket communication
#include <sys/socket.h>
#include <arpa/inet.h>


LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  // Just handle some common baud rates
  switch (baud_rate)
  {
    case 1200: return LibSerial::BaudRate::BAUD_1200;
    case 1800: return LibSerial::BaudRate::BAUD_1800;
    case 2400: return LibSerial::BaudRate::BAUD_2400;
    case 4800: return LibSerial::BaudRate::BAUD_4800;
    case 9600: return LibSerial::BaudRate::BAUD_9600;
    case 19200: return LibSerial::BaudRate::BAUD_19200;
    case 38400: return LibSerial::BaudRate::BAUD_38400;
    case 57600: return LibSerial::BaudRate::BAUD_57600;
    case 115200: return LibSerial::BaudRate::BAUD_115200;
    case 230400: return LibSerial::BaudRate::BAUD_230400;
    default:
      std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 57600" << std::endl;
      return LibSerial::BaudRate::BAUD_57600;
  }
}

class RpPicoComs
{

public:

  void connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
  {  
    RCLCPP_INFO(rclcpp::get_logger("omnidrive3RpPicoHardware"), "Connecting to serial device: %s at baud rate: %d", serial_device.c_str(), baud_rate);
    timeout_ms_ = timeout_ms;
    
    // Check if the serial device exists
    if (!rcpputils::fs::exists(serial_device))
    {
      sim_mode = true;

      clientSocket = socket(AF_INET, SOCK_STREAM, 0);
      if (clientSocket == -1) {
          std::cerr << "Failed to create socket" << std::endl;
          return;
      }

      // Specify server details
      serverAddress.sin_family = AF_INET;
      serverAddress.sin_port = htons(12346); // Use the same port as the server
      if (inet_pton(AF_INET, "127.0.0.1", &serverAddress.sin_addr) <= 0) { // Replace with server IP
          std::cerr << "Invalid address or address not supported" << std::endl;
          return;
      }

      // Connect to the server
      
      if (::connect(clientSocket, (struct sockaddr*)&serverAddress, sizeof(serverAddress)) < 0) {
          std::cerr << "Connection failed" << std::endl;
          return;
      }
      socket_connected = true;
      return;
      
    }





    
    serial_conn_.Open(serial_device);
    serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
  }

  void disconnect()
  {
    serial_conn_.Close();
  }


  bool connected() const
  {
    return sim_mode || serial_conn_.IsOpen();
  }


  std::string send_msg(const std::string &msg_to_send, bool print_output = false)
  {

    std::string response = "";
    if (!sim_mode){
      serial_conn_.FlushIOBuffers(); // Just in case
      serial_conn_.Write(msg_to_send);

      if (print_output)
      {
        std::cout << "Sent: " << msg_to_send << " Recv: " << response << std::endl;
      }
      

      return response;
    }else{
      if (!socket_connected){
        if (::connect(clientSocket, (struct sockaddr*)&serverAddress, sizeof(serverAddress)) < 0) {
          return response;
        }
        socket_connected = true;
        std::cerr << "### Connected to socket ###" << std::endl;
        return response;
      }

      const char* message = msg_to_send.c_str();

      errno = 0;
      if (send(clientSocket, message, strlen(message), 0) < 0) {
          std::cerr << "Failed to send message" << std::endl;
          std::cerr << strerror(errno) << std::endl;
          return response;
      }
  
      return response;
    }
  }


  void send_empty_msg()
  {
    std::string response = send_msg("\n");
  }

  void read_encoder_values(int &val_1, int &val_2,int &val_3)
  {
    std::string response = send_msg("e\r");

    std::string delimiter = " ";
    size_t del_pos = response.find(delimiter);
    std::string token_1 = response.substr(0, del_pos);
    std::string token_2 = response.substr(del_pos + delimiter.length());
    std::string token_3 = response.substr(del_pos + delimiter.length());
    
    val_1 = std::atoi(token_1.c_str());
    val_2 = std::atoi(token_2.c_str());
    val_3 = std::atoi(token_3.c_str());
  }
  void set_motor_values(int val_1, int val_2, int val_3)
  {
    std::stringstream ss;
    ss << "m " << val_1 << " " << val_2 << " " << val_3 << "\n";
    send_msg(ss.str());
    }

  void set_pid_values(int k_p, int k_d, int k_i, int k_o)
  {
    std::stringstream ss;
    ss << "u " << k_p << ":" << k_d << ":" << k_i << ":" << k_o << "\r";
    send_msg(ss.str());
  }

private:
  bool sim_mode = false;
  bool socket_connected = false;
  LibSerial::SerialPort serial_conn_;
  int sock;
  int timeout_ms_;
  int clientSocket;
  sockaddr_in serverAddress{};
};

#endif // omnidrive3_RPPICO__OMNIBOT_SYSTEM_HPP_