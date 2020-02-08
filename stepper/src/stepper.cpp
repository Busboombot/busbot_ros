#include <string>
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include "messages.h"
#include <string>
#include <ros/console.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Joy.h"

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

void chatterCallback(const sensor_msgs::Joy::ConstPtr& msg){

    for(int i = 0; i < 6; i++)
        cout << msg->axes[i] << " ";

  cout << endl;
}

void foo(int argc, char **argv) {
    string port;
    int baud;

    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
        ros::console::notifyLoggerLevelsChanged();
    }

    ros::init(argc, argv, "stepper");

    ros::NodeHandle nh("stepper");
 
    nh.getParam("baud", baud);
    nh.getParam("port", port);

    cout << " port=" << port << " baud=" << baud << endl;

    ros::Subscriber sub = nh.subscribe("/joy", 10, chatterCallback);
    
    cout << "Starting"<< endl;
    ros::spin();

}




// Read an axis configuration from parameters. 
bool readAxisConfig(int axis, ros::NodeHandle &nh, const MessageProcessor &mp, AxisConfig &as){

    std::string axisn = std::string("axis")+std::to_string(axis);

    if (!nh.hasParam(axisn)){
        return false;
    }

    std::map<std::string,int> pin_map;

    nh.getParam(axisn, pin_map);

    cout << pin_map["step_pin"] << " " << pin_map["direction_pin"] << " " << pin_map["enable_pin"] << endl;

    int step_pin, direction_pin, enable_pin;
    int v_max, a_max;

    step_pin = pin_map["step_pin"];
    direction_pin = pin_map["direction_pin"];
    enable_pin = pin_map["enable_pin"];

    v_max = pin_map["v_max"];

    if (!v_max)
        nh.getParam("v_max", v_max);

    a_max = pin_map["a_max"];

    if (!a_max)
        nh.getParam("a_max", a_max);

    as = {(uint8_t)axis, (uint8_t)step_pin, (uint8_t)direction_pin, (uint8_t)enable_pin, (uint32_t)v_max, (uint32_t)a_max};

    return true;
}
// Get the configuration from parameters 
// and write it to the step generator. 
void config(ros::NodeHandle &nh,  MessageProcessor &mp){


    int n_axes, interrupt_delay, debug_print, debug_tick;

    nh.getParam("n_axes", n_axes);
    nh.getParam("interrupt_delay", interrupt_delay);
    nh.getParam("debug_print", debug_print);
    nh.getParam("debug_tick", debug_tick);

    Config config(n_axes, interrupt_delay, debug_print, debug_tick);
    mp.sendConfig(config);

    for(int i = 0; i < config.n_axes; i++){
        AxisConfig as;
        if(readAxisConfig(i, nh, mp, as)){
            mp.sendAxisConfig(as);
        }
    }

}

int main(int argc, char **argv) {
    string port;
    int baud;

    ros::init(argc, argv, "stepper");
    ros::NodeHandle nh("stepper");
 
    nh.getParam("baud", baud);
    nh.getParam("port", port);

    cout << " port=" << port << " baud=" << baud << endl;
    
    serial::Serial serial(port, baud, serial::Timeout::simpleTimeout(1000));
    MessageProcessor mp(serial);

    config(nh, mp);

    mp.sendInfo();

    while(mp.read_next(1)){
        report(mp);
    }

    mp.sendMove({int(10e3),0});
    mp.sendMove({int(10e3),int(10e3)});
    mp.sendMove({int(10e3),0});
    mp.sendMove({int(10e3),int(10e3)});
    
    while(mp.read_next(3)){
        report(mp);
    }

}
