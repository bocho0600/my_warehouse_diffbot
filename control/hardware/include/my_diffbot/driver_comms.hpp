#ifndef I2C_INTERFACE_HPP
#define I2C_INTERFACE_HPP

#include <string>
#include <vector>
#include <stdexcept>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <iostream>

class I2CInterface
{
public:
      I2CInterface(const std::string &i2c_bus, uint8_t addr)
          : addr_(addr), prev_left_speed_(0), prev_right_speed_(0)
      {
            bus_fd_ = open(i2c_bus.c_str(), O_RDWR); // Open the I2C bus
            if (bus_fd_ < 0)
            {
                  throw std::runtime_error("Failed to open I2C bus"); 
            }
            if (ioctl(bus_fd_, I2C_SLAVE, addr_) < 0) // Set the I2C slave address
            {
                  throw std::runtime_error("Failed to connect to I2C device");
            }
      }

      ~I2CInterface() // Destructor - close the I2C bus
      {
            if (bus_fd_ >= 0) // Check if the file descriptor is valid
            {
                  close(bus_fd_); // Close the I2C bus
            }
      }

      void LedWrite(int number, const std::string &state)
      {
            if (number < 0 || number > 3)
            {
                  std::cerr << "Invalid LED number\n";
                  return;
            }
            if (state != "ON" && state != "OFF")
            {
                  std::cerr << "Invalid LED state\n";
                  return;
            }
            writeCommand("L" + std::to_string(number) + " " + state);
      }

      void ServoWrite(int number, int angle)
      {
            if (number < 1 || number > 4 || angle < 0 || angle > 180)
            {
                  std::cerr << "Invalid Servo parameters\n";
                  return;
            }
            writeCommand("S" + std::to_string(number) + " " + std::to_string(angle));
      }

      void DCWrite(int number, const std::string &direction, int speed)
      {
            if (number < 1 || number > 2 || speed < 0 || speed > 255)
            {
                  std::cerr << "Invalid DC motor parameters\n"; // Print an error message
                  return;
            }
            if (direction != "0" && direction != "1" && direction != "S")
            {
                  std::cerr << "Invalid DC motor direction\n"; // Print an error message
                  return;
            }
            writeCommand("M" + std::to_string(number) + " " + direction + " " + std::to_string(speed));
      }

      void MoveForward(int speed)
      {
            if (speed > 0)
            {
                  DCWrite(1, "0", speed);
                  DCWrite(2, "0", speed);
            }
            else if (speed < 0)
            {
                  DCWrite(1, "1", -speed);
                  DCWrite(2, "1", -speed);
            }
            else
            {
                  StopAll();
            }
      }

      void Rotate(int speed)
      {
            if (speed > 0)
            {
                  DCWrite(1, "0", speed);
                  DCWrite(2, "1", speed);
            }
            else if (speed < 0)
            {
                  DCWrite(1, "1", -speed);
                  DCWrite(2, "0", -speed);
            }
      }

      void StopAll()
      {
            DCWrite(1, "S", 0);
            DCWrite(2, "S", 0);
      }

      void SetMotor(int motor, int speed)
      {
            if (speed == 0)
            {
                  DCWrite(motor, "S", 0);
            }
            else if (speed < 0)
            {
                  DCWrite(motor, "1", -speed);
            }
            else
            {
                  DCWrite(motor, "0", speed);
            }
      }

      void Movement(int left_speed, int right_speed)
      {
            if (prev_left_speed_ != left_speed)
            {
                  SetMotor(1, left_speed);
                  prev_left_speed_ = left_speed;
            }
            if (prev_right_speed_ != right_speed)
            {
                  SetMotor(2, -right_speed);
                  prev_right_speed_ = right_speed;
            }
      }

private:
      int bus_fd_;
      uint8_t addr_;
      int prev_left_speed_;
      int prev_right_speed_;

      void writeCommand(const std::string &command)
      {
            std::vector<uint8_t> buffer(command.begin(), command.end());
            if (write(bus_fd_, buffer.data(), buffer.size()) != static_cast<ssize_t>(buffer.size()))
            {
                  throw std::runtime_error("Failed to write to I2C device");
            }
      }
};

#endif // I2C_INTERFACE_HPP
