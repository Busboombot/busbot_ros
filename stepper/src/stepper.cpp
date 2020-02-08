#include <string>
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include "messages.h"
#include <string>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

#include "messages.h"

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;


uint32_t a_max = 3e6;
uint32_t v_max = 10e6;


void report(MessageProcessor &mp){
    cout << "la="<<mp.getLastAck()<<"\tld="<<mp.getLastDone()
        << "\tql="<<mp.getQueueLength()<<"\tqt="<<mp.getQueueTime()<<endl;
}

void test_echo(MessageProcessor mp, char * test_str){
    mp.sendInfo();
   
    for(int i = 0; i < 10; i++){
        mp.send(CommandCode::ECHO, (const uint8_t*)test_str, strlen(test_str));

        mp.read_next(1);

    }
}

void foo(){
    string port;
    unsigned long  baud = 115200/*3e6*/;

    serial::Serial serial(port, baud, serial::Timeout::simpleTimeout(1000));
    MessageProcessor mp(serial);

    mp.sendMove({int(10e3),0});
    mp.sendMove({int(10e3),int(10e3)});
    mp.sendMove({int(10e3),0});
    mp.sendMove({int(10e3),int(10e3)});
    
    while(mp.read_next(3)){
        report(mp);
    }
}

int main(int argc, char **argv) {

    string port;
    string baud;


    ros::init(argc, argv, "stepper");

    ros::NodeHandle nh("stepper");
 
    nh.getParam("baud", baud);
    nh.getParam("port", baud);

    cout << port << " " << baud << endl;


    
    
}
