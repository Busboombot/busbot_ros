#pragma once

#include "serial/serial.h"

#define N_AXES 6

enum class CommandCode : uint8_t {
  
  ACK =     1,
  NACK =    2,
  RUN =     3,
  STOP =    4,
  RESET =   5,  // Payload is a debug message; the next packet is text
  MESSAGE = 6,  // Payload is a message; the next packet is text
  ERROR =   7,  // Some error
  ECHO  =   8,  // Echo the incomming header
  NOOP  =   9,  // Does nothing, but get ACKED
  CONFIG =  10, // Reset the configuration
  INFO   =  11,
  EMPTY  =  12, // Queue is empty, nothing to do. 
  POSITIONS=19,  // Position report. 
  
  DONE =    20,  // A Movemenbt comment is finished
  MOVE =    21  // A movement segment, with just the relative distance.   

};


struct PacketHeader {
  uint16_t seq;       // Packet sequence number
  CommandCode code;   // Command code      
  uint8_t crc;    // Payload CRC8
  
  //PacketHeader(uint16_t seq, CommandCode code, uint8_t crc)
  
}; 


struct AxisHeader {
  uint32_t segment_time = 0; // total segment time, in microseconds // 4
  uint16_t unused; 
  uint16_t n_axes = 0; // Number of axes in message
}; // 8

struct AxisSegment {
  uint8_t axis_number;
  int8_t  scale; 
  uint8_t unused2; 
  uint8_t unused3; 
  int32_t steps; // Number of steps to move the axis
};

struct AxisMove {
  uint8_t axis_number;
  uint8_t unused1; 
  uint8_t unused2; 
  uint8_t unused3; 
  int32_t x; // Number of steps to move the axis
};


struct CurrentState {
  int32_t queue_length = 0;
  uint32_t queue_time = 0;
  int32_t positions[N_AXES] = {0};
  
}; 

#define MESSAGE_BUF_SIZE 256

class MessageProcessor {
    
protected:
    uint8_t buffer[MESSAGE_BUF_SIZE]; // Outgoing message buffer
    
    
public: 
    MessageProcessor(serial::Serial &ser);
    
};


