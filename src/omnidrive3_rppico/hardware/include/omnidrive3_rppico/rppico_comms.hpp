#ifndef omnidrive3_RPPICO__OMNIBOT_COMMS_HPP_
#define omnidrive3_RPPICO__OMNIBOT_COMMS_HPP_

#include <sstream>
#include <libserial/SerialPort.h>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include <sys/stat.h>



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
    std::cout << "Connecting to serial device: " << serial_device << " at baud rate: " << baud_rate << std::endl;
    timeout_ms_ = timeout_ms;
    
    // Check if the serial device exists
    if (!rcpputils::fs::exists(serial_device))
    {
      RCLCPP_WARN(rclcpp::get_logger("omnidrive3RpPicoHardware"), "Serial device %s does not exist", serial_device.c_str());
      
      // the interface is not created in /dev but in /tmp because of permission issues 
      pid_t pid = fork();
      if (pid == 0)
      {
        execlp("socat", "socat", "-d", "-d", "pty,raw,echo=0,link=/tmp/rppicoTX", "pty,raw,echo=0,link=/tmp/rppicoRX", (char*)NULL);
        
      }else if (pid < 0)
        {
        RCLCPP_ERROR(rclcpp::get_logger("omnidrive3RpPicoHardware"), "Failed to create socat");

      }else{
        RCLCPP_INFO(rclcpp::get_logger("omnidrive3RpPicoHardware"), "Created socat, currently in parent process");
        std::this_thread::sleep_for(std::chrono::seconds(2));
        serial_conn_.Open("/dev/pts/11");
        serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
        return;
      }
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
    return serial_conn_.IsOpen();
  }


  std::string send_msg(const std::string &msg_to_send, bool print_output = false)
  {
    serial_conn_.FlushIOBuffers(); // Just in case
    serial_conn_.Write(msg_to_send);

    std::string response = "";
    //skip the response part for now
    /*
    try
    {
      // Responses end with \r\n so we will read up to (and including) the \n.
      serial_conn_.ReadLine(response, '\n', timeout_ms_);
    }
    catch (const LibSerial::ReadTimeout&)
    {
        std::cerr << "The ReadByte() call has timed out." << std::endl ;
    }
    */
    if (print_output)
    {
      std::cout << "Sent: " << msg_to_send << " Recv: " << response << std::endl;
    }
    

    return response;
  }


  void send_empty_msg()
  {
    std::string response = send_msg("\r");
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
    ss << "m " << val_1 << " " << val_2 << " " << val_3 << "\r";
    send_msg(ss.str());
    }

  void set_pid_values(int k_p, int k_d, int k_i, int k_o)
  {
    std::stringstream ss;
    ss << "u " << k_p << ":" << k_d << ":" << k_i << ":" << k_o << "\r";
    send_msg(ss.str());
  }

private:
    LibSerial::SerialPort serial_conn_;
    int timeout_ms_;
};

#endif // omnidrive3_RPPICO__OMNIBOT_SYSTEM_HPP_