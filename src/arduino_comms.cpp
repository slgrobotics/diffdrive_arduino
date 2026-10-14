#include "diffdrive_arduino/arduino_comms.h"
#include "diffdrive_arduino/encoder_parser.h"
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <cstdlib>
#include <iterator>
#include <vector>
#include <thread>


void ArduinoComms::setup(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
{
    serial_conn_.open(serial_device, baud_rate, timeout_ms);
}

void ArduinoComms::sendEmptyMsg()
{
    // Opening the port can reset an Arduino. Allow it to finish booting.
    std::this_thread::sleep_for(std::chrono::seconds(1));
    try
    {
        const auto response = serial_conn_.exchange("\r");
        RCLCPP_INFO_STREAM(rclcpp::get_logger("DiffDriveArduino"), "Arduino startup response: '" << response << "'");
    }
    catch (const std::exception &error)
    {
        RCLCPP_ERROR(rclcpp::get_logger("DiffDriveArduino"), "Arduino startup: %s", error.what());
    }
}


bool ArduinoComms::readEncoderValues(int &val_1, int &val_2)
{
    //std::cout << "...polling encoders..." << std::endl;

    std::string response = sendMsg("e\r");

    //std::cout << "response: " << response << std::endl;

    return parseEncoderValues(response, val_1, val_2);
}

void ArduinoComms::readHealthValues(int &voltage_mv, int &current_ma, int &free_mem_bytes)
{
    std::string response = sendMsg("h\r");

    std::istringstream iss(response);
    std::vector<std::string> results(std::istream_iterator<std::string>{iss},
                                     std::istream_iterator<std::string>());
    
    if(results.size() == 3)
    {
        voltage_mv = std::atoi(results.at(0).c_str());      // normally around 12600 (mv)
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

std::string ArduinoComms::sendMsg(const std::string &msg_to_send, bool print_output)
{
    try
    {
        // Every command expects a CR-terminated reply prefixed with its command letter.
        const std::string response = serial_conn_.exchange(msg_to_send);
        if (response.size() < 3 || response[1] != ' ' || response.back() != '\r')
        {
            RCLCPP_ERROR(rclcpp::get_logger("DiffDriveArduino"), "Incomplete or malformed Arduino response");
            return {};
        }
        if (response.front() != msg_to_send.front())
        {
            RCLCPP_ERROR(rclcpp::get_logger("DiffDriveArduino"), "Arduino reply does not match command '%c'", msg_to_send.front());
            return {};
        }
        if (print_output)
        {
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("DiffDriveArduino"), "Sent: '" << msg_to_send << "' received: '" << response << "'");
        }
        return response.substr(2);
    }
    catch (const std::exception &error)
    {
        RCLCPP_ERROR(rclcpp::get_logger("DiffDriveArduino"), "Arduino serial transaction: %s", error.what());
        return {};
    }
}
