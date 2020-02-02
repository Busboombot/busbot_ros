#include <string>
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include "messages.h"
#include <string>

#include "CRC.h"
#include "serial/serial.h"
#include "cobs.h"

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;


PacketHeader h  = { 11,  CommandCode::ECHO, 8 };

string sbuffer("\0",256);

CRC::Table<std::uint8_t, 8> table(CRC::CRC_8());

size_t send_echo(serial::Serial &serial, uint16_t seq,  string& str){
    
    uint8_t data_buf[256];
    uint8_t send_buf[256];
  
    PacketHeader h  = { seq,  CommandCode::ECHO, 0};
    
    memcpy(data_buf, &h, sizeof(PacketHeader));
    memcpy(data_buf+sizeof(PacketHeader), str.c_str(), str.length());
    
    size_t  data_len = sizeof(PacketHeader)+str.length();

    ((PacketHeader* )data_buf)->crc = CRC::Calculate(data_buf,data_len, table);

    size_t l = cobs_encode(data_buf, data_len, send_buf);
    send_buf[l] = 0;
    l++;
    
    size_t bytes_written = serial.write(send_buf, l);
    
    
    return bytes_written;
    
}

int main(int argc, char **argv) {
    
    uint8_t rcv_buffer[256];
    uint8_t cobs_buffer[256];
   
    string port(argv[1]);
    string test_string("FOOABR");
    
    unsigned long  baud = 3e6;
    
    serial::Serial serial(port, baud, serial::Timeout::simpleTimeout(1000));
    
    for(int i=0; i < 10; i++){
        h.seq = i;
        
        size_t send_len = send_echo(serial, i,  test_string);

        size_t n=0;
        for(; n<256; n++){
            serial.read(&rcv_buffer[n], 1);
            if(rcv_buffer[n] == 0)
                break;
        }

        //size_t bytes_read = serial.readline(sbuffer, 256, "\0");
        size_t bytes_read = n;
        
        size_t l2 = cobs_decode(rcv_buffer, bytes_read, (uint8_t*)cobs_buffer);
        
        PacketHeader &h = *((PacketHeader*)cobs_buffer);
        char * payload = ((char*)(cobs_buffer+sizeof(PacketHeader)));
        
        cout << "Read: br=" << bytes_read << "\tbd=" << l2 << "\tseq=" << h.seq << "\tcrc=" << (int)h.crc << endl;
        
    }
    
    
    
}
