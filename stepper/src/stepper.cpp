#include <string>
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include "messages.h"
#include <string>


#include "messages.h"

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;


uint32_t a_max = 3e6;
uint32_t v_max = 10e6;

Config config = {
    N_AXES, // n_axes
    4, // interrupt_delay
    false, // debug_print
    false, // debug_tick
    
    {
        //mode, step, dir, en, v_max, a_max
        {0, 5,  2,  3, v_max, a_max},
        {0, 6,  7,  4, v_max, a_max},
        {0, 8,  9,  10, v_max, a_max},
        {0, 11, 12, 13, v_max, a_max},
        {0, 14, 14, 14, v_max, a_max},
        {0, 14, 14, 14, v_max, a_max}
       
        //{0, 14, 15, 16, v_max, a_max},
        //{0, 17, 18, 19, v_max, a_max}
    }
    
};

void report(MessageProcessor &mp){
    cout << "la="<<mp.getLastAck()<<"\tld="<<mp.getLastDone()
        << "\tql="<<mp.getQueueLength()<<"\tqt="<<mp.getQueueTime()<<endl;
}

int main(int argc, char **argv) {
    
    uint8_t rcv_buffer[256];
    uint8_t cobs_buffer[256];

    string port(argv[1]);

    char test_str[] = "FOOABR";

    unsigned long  baud = 3e6;

    serial::Serial serial(port, baud, serial::Timeout::simpleTimeout(1000));
    MessageProcessor mp(serial);

    mp.sendInfo();
  
    mp.sendConfig(config);

    mp.sendInfo();
    
    for(int i = 0; i < 10; i++){
        mp.send(CommandCode::ECHO, (const uint8_t*)test_str, strlen(test_str));

        mp.read_next(1);

    }

    while(mp.read_next(.2));
    
    
    mp.sendMove(0,{int(10e3),0});
    mp.sendMove(0,{int(10e3),int(10e3)});
    mp.sendMove(0,{int(10e3),0});
    mp.sendMove(0,{int(10e3),int(10e3)});
    
    while(mp.read_next(2)){
        report(mp);
    }
    
    
}
