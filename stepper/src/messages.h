#pragma once

#include <vector>
#include "serial/serial.h"

using std::vector;

#define N_AXES 6

enum class CommandCode : uint8_t {
  
  NONE =    0,
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


// Payload of the move command
struct Moves {
  uint32_t segment_time = 0; // total segment time, in microseconds // 4
  int32_t x[N_AXES];
}; // 8


struct AxisConfig {

    uint8_t mode;           // STEPDIR, QUAD or PWM
    uint8_t step_pin;       // Step output, or quadture b
    uint8_t direction_pin;  // Direction output, or quadrature b
    uint8_t enable_pin;
    uint32_t v_max;
    uint32_t a_max;
};

// Main Configuration class, 68 bytes
struct Config {

    uint8_t n_axes;         // Number of axes
    uint8_t interrupt_delay;    // How often interrupt is called, in microseconds
    bool debug_print ;
    bool debug_tick;
    
    AxisConfig axis_config[N_AXES];  // Up to 8 axes
};


struct CurrentState {
  int32_t queue_length = 0;
  uint32_t queue_time = 0;
  int32_t positions[N_AXES] = {0};
  
}; 

#define MESSAGE_BUF_SIZE 256

class MessageProcessor {
    
protected:
    
    uint8_t data_buf[MESSAGE_BUF_SIZE]; // For composing messages
    uint8_t send_buf[MESSAGE_BUF_SIZE]; // Encoded and send
    uint8_t rcv_buf[MESSAGE_BUF_SIZE];  // encoded and recieved
    
    uint16_t seq = 0;
    
    CommandCode last_code = CommandCode::NONE;
    int last_ack = 0;
    int last_done = 0;
    
    
    CurrentState current_state;
    
    serial::Serial &ser;
    
    void handle(uint8_t* data, size_t len);
    
public: 
    
    MessageProcessor(serial::Serial &ser): ser(ser) {}

    size_t send(CommandCode code, const uint8_t* payload, size_t payload_len);

    void sendConfig(Config& config);
    void sendInfo();

    void sendMove(uint32_t t, vector<int> x);
    void sendMove(uint32_t t, vector<double> x);
    void sendMove(vector<int> x);
    void sendMove(uint32_t t, std::initializer_list<int> il);
    void sendMove(uint32_t t, std::initializer_list<double> il);

    bool update();
    bool read_next(float timeout);

    int getLastAck() { return last_ack;}
    int getLastDone() { return last_done;}
    int32_t getQueueLength(){return current_state.queue_length;}
    uint32_t getQueueTime(){return current_state.queue_time;}

    
   
};


