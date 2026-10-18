#include "diffdrive_arduino/arduino_comms.h"
#include "diffdrive_arduino/encoder_parser.h"
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <array>
#include <cctype>
#include <stdexcept>
#include <thread>


namespace
{
template<size_t N>
bool parseIntegers(const std::string &payload, std::array<int, N> &values)
{
    std::istringstream stream(payload);
    for (size_t i = 0; i < N; ++i)
    {
        if (i != 0 && !std::isspace(stream.peek())) return false;
        if (!(stream >> values[i])) return false;
    }
    stream >> std::ws;
    return stream.eof();
}

void requireAcknowledgment(const std::string &payload, const char *command)
{
    if (payload != "OK\r")
        throw std::runtime_error(std::string("Arduino did not acknowledge ") + command);
}
}


void ArduinoComms::setup(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
{
    serial_conn_.open(serial_device, baud_rate, timeout_ms);
}

void ArduinoComms::sendEmptyMsg()
{
    // Opening the port can reset an Arduino. Allow it to finish booting.
    std::this_thread::sleep_for(std::chrono::seconds(1));
    requireAcknowledgment(sendMsg("\r"), "startup handshake");
}


bool ArduinoComms::readEncoderValues(int &val_1, int &val_2)
{
    //std::cout << "...polling encoders..." << std::endl;

    std::string response = sendMsg("e\r");

    //std::cout << "response: " << response << std::endl;

    return parseEncoderValues(response, val_1, val_2);
}

bool ArduinoComms::readHealthValues(int &voltage_mv, int &current_ma, int &free_mem_bytes)
{
    std::array<int, 3> values;
    if (!parseIntegers(sendMsg("h\r"), values)) return false;
    voltage_mv = values[0];
    current_ma = values[1];
    free_mem_bytes = values[2];
    return true;
}

bool ArduinoComms::readPingValues(int &front_right, int &front_left, int &back_right, int &back_left)
{
    std::array<int, 4> values;
    if (!parseIntegers(sendMsg("p\r"), values)) return false;
    front_right = values[0];
    front_left = values[1];
    back_right = values[2];
    back_left = values[3];
    return true;
}

void ArduinoComms::setMotorValues(int val_1, int val_2)
{
    // "m" - set speeds, "o" - set raw PWM
    std::stringstream ss;
    ss << "m " << val_1 << " " << val_2 << "\r";
    requireAcknowledgment(sendMsg(ss.str()), "motor command");
}

void ArduinoComms::setPidValues(float k_p, float k_d, float k_i, float k_o)
{
    std::stringstream ss;
    ss << "u " << k_p << ":" << k_d << ":" << k_i << ":" << k_o << "\r";
    requireAcknowledgment(sendMsg(ss.str()), "PID command");
}

/*
These are the command letters used in the Arduino sketch.
They are defined here for reference, but not used in this file.

Matching Arduino code can be found in the following sketches:
    https://github.com/slgrobotics/Misc/blob/master/Arduino/Sketchbook/WheelsROS_Seggy/Comm.ino
    https://github.com/slgrobotics/Misc/blob/master/Arduino/Sketchbook/DraggerROS/DraggerComm.ino

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
*/

std::string ArduinoComms::sendMsg(const std::string &msg_to_send, bool print_output)
{
    try
    {
        // Every command expects a CR-terminated reply prefixed with its command letter.
        const std::string response = serial_conn_.exchange(msg_to_send);
        // Debug tracing includes every reply, even replies rejected below.
        // Escape CR/LF so each transaction remains a readable log line.
        auto escaped = [](const std::string & value) {
            std::string result;
            for (char c : value) {
                if (c == '\r') result += "\\r";
                else if (c == '\n') result += "\\n";
                else result += c;
            }
            return result;
        };
        RCLCPP_DEBUG(rclcpp::get_logger("DiffDriveArduino"), "TX '%s' RX '%s'",
                     escaped(msg_to_send).c_str(), escaped(response).c_str());
        if (response.size() < 3 || response[1] != ' ' || response.back() != '\r')
        {
            RCLCPP_ERROR(rclcpp::get_logger("DiffDriveArduino"), "Incomplete or malformed Arduino response");
            return {};
        }
        if (response.front() == '?')
        {
            RCLCPP_ERROR(rclcpp::get_logger("DiffDriveArduino"), "Firmware rejected command: %s", response.c_str());
            return {};
        }
        // An empty command is acknowledged with '*', rather than an echoed CR.
        const char expected_command = msg_to_send == "\r" ? '*' : msg_to_send.front();
        if (response.front() != expected_command)
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
