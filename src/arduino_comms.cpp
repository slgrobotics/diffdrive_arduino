#include "diffdrive_arduino/arduino_comms.h"
// #include <ros/console.h>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <cstdlib>


void ArduinoComms::setup(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
{  
    serial_conn_.setPort(serial_device);
    serial_conn_.setBaudrate(baud_rate);
    serial::Timeout tt = serial::Timeout::simpleTimeout(timeout_ms);
    tt.write_timeout_constant = 100;
    tt.inter_byte_timeout = 10;
    serial_conn_.setTimeout(tt); // This should be inline except setTimeout takes a reference and so needs a variable
    serial_conn_.open();
    serial_conn_.flush();
    //serial_conn_.readline();    // read/ignore all garbage from Arduino
    // serial_conn_.(serial_device, baud_rate, serial::Timeout::simpleTimeout(timeout_ms));


}


void ArduinoComms::sendEmptyMsg()
{
    serial_conn_.write("\r");
    //serial_conn_.flush();

    int tries = 10;
    std::string response;
    do {
        sleep(1);
        // read any garbled response from Arduino:
        response = serial_conn_.readline();
        RCLCPP_INFO_STREAM(rclcpp::get_logger("DiffDriveArduino"),"Arduino garbage: '" << response << "'");
    } while (response.length() > 0 && --tries > 0);
}

void ArduinoComms::readEncoderValues(int &val_1, int &val_2)
{
    //std::cout << "...polling encoders..." << std::endl;

    std::string response = sendMsg("e\r");

    //std::cout << "response: " << response << std::endl;

    if(response.length() >= 5)
    {
        std::string delimiter = " ";
        size_t del_pos = response.find(delimiter);
        std::string token_1 = response.substr(0, del_pos);
        std::string token_2 = response.substr(del_pos + delimiter.length());

        val_1 = std::atoi(token_1.c_str());
        val_2 = std::atoi(token_2.c_str());
    }
}

void ArduinoComms::readHealthValues(int &battery_mv, int &current_ma, int &free_mem_bytes)
{
    std::string response = sendMsg("h\r");

    std::istringstream iss(response);
    std::vector<std::string> results(std::istream_iterator<std::string>{iss},
                                     std::istream_iterator<std::string>());
    
    if(results.size() == 3)
    {
        battery_mv = std::atoi(results.at(0).c_str());      // normally around 12600 (mv)
        current_ma = std::atoi(results.at(1).c_str());      // not measured - -1
        free_mem_bytes = std::atoi(results.at(2).c_str());  // normally around 7150
    }
}

void ArduinoComms::readPingValues(int &front_right, int &front_left, int &back_right, int &back_left)
{
    std::string response = sendMsg("p\r");

    //std::cout << "Sonar: '" << response << "'" << std::endl;

    std::istringstream iss(response);
    std::vector<std::string> results(std::istream_iterator<std::string>{iss},
                                     std::istream_iterator<std::string>());
    
    if(results.size() == 4)
    {
        // Ranges in centimeters:
        front_right = std::atoi(results.at(0).c_str());
        front_left = std::atoi(results.at(1).c_str());
        back_right = std::atoi(results.at(2).c_str());
        back_left = std::atoi(results.at(3).c_str());
    }
}

void ArduinoComms::setMotorValues(int val_1, int val_2)
{
    // "m" - set speeds, "o" - set raw PWM
    std::stringstream ss;
    ss << "m " << val_1 << " " << val_2 << "\r";
    sendMsg(ss.str(), false);
}

void ArduinoComms::setPidValues(float k_p, float k_d, float k_i, float k_o)
{
    std::stringstream ss;
    ss << "u " << k_p << ":" << k_d << ":" << k_i << ":" << k_o << "\r";
    sendMsg(ss.str());
}

#define ANALOG_READ    'a'
#define GET_BAUDRATE   'b'
#define PIN_MODE       'c'
#define DIGITAL_READ   'd'
#define READ_ENCODERS  'e'
#define READ_HEALTH    'h'
#define MOTOR_SPEEDS   'm'
#define MOTOR_RAW_PWM  'o'
#define SONAR_PING     'p'
#define RESET_ENCODERS 'r'
#define SERVO_WRITE    's'
#define SERVO_READ     't'
#define UPDATE_PID     'u'
#define DIGITAL_WRITE  'w'
#define ANALOG_WRITE   'x'

#define EXPECT_RESPONSE_OK

std::string ArduinoComms::sendMsg(const std::string &msg_to_send, bool print_output)
{
    //std::cout << "...sending: " << msg_to_send << std::endl;

    serial_conn_.write(msg_to_send);
    serial_conn_.flush();

    char cmd = msg_to_send.at(0);

#ifdef EXPECT_RESPONSE_OK

    // Always expect response, even if it is just "<cmd> OK" or an empty string

    std::string response = serial_conn_.readline(1024, "\r");

#else // EXPECT_RESPONSE_OK

    // Only expect response for some commands:

    std::string response = "";

    switch(cmd)
    {
        // These requests do cause response with data:
        case ANALOG_READ:
        case GET_BAUDRATE:
        case DIGITAL_READ:
        case READ_ENCODERS:
        case READ_HEALTH:
        case SONAR_PING:
        case SERVO_READ:
            response = serial_conn_.readline(1024, "\r");
            break;
        default:
            return; // no response expected
    }

#endif // EXPECT_RESPONSE_OK

    if(response.length() == 0)
    {
        std::cout << "Error: Arduino empty response for cmd '" << msg_to_send << "'" << std::endl;
        return "";
    }

    char cmd_r = response.at(0);

    if(cmd != cmd_r)
    {
        std::cout << "Error: Arduino unmatched response '" << response << "' for cmd '" << msg_to_send << "'" << std::endl;
        return "";
    }

    //std::cout << "response: '" << response << "'" << std::endl;


    if (print_output)
    {
        // RCLCPP_INFO_STREAM(rclcpp::get_logger("DiffDriveArduino"),"Sent: '" << msg_to_send << "' received: '" << response << "'");
    }

    return response.substr(2); // remove command char and space
}
