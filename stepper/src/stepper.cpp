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
#include "stepper/MoveCommand.h"
#include "stepper/PositionReport.h"

#include <sstream>

#include "messages.h"

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::setw;
using std::setprecision;
using std::left;

uint32_t a_max = 3e6;
uint32_t v_max = 10e6;

void report(MessageProcessor &mp){

    std::stringstream ss;

    switch(mp.getLastCode()){
        case CommandCode::ACK:
            ss << "ACK\t";
            break;
        case CommandCode::DONE:
            ss << "DONE\t";
            break;   
        case CommandCode::EMPTY:
            ss << "EMPTY\t";
            break;   
        default:;
            
    }
    
    ss  << mp.getLastSeq();

    ss << "\tla="<<mp.getLastAck()<<"\tld="<<mp.getLastDone()
        << "\tql="<<setw(3)<<mp.getQueueLength()<<"\tqt="<<setw(7)<<setprecision(4)<<left<< mp.getQueueTime();
        
    for(int i : mp.getPositions())
        ss << " " << setw(5) << i;

    ss << " | "; 

    for(int i : mp.getPlannerPositions())
        ss << " " << setw(5) << i;

    //cout << ss.str() << endl;
    ROS_DEBUG_NAMED("stepper", "%s", ss.str().c_str());
    
}


// Publish update messages. 
class Publisher: public MessagePubHelper {

protected:
    
    ros::Publisher pub;

public:

    Publisher(){}    

    void setPub(ros::Publisher pub){ this->pub = pub;}
    
    void publish(PacketHeader &h, CurrentState& current_state){
        
        report(*mp);
        
        stepper::PositionReport pr;
        
        pr.queue_time = current_state.queue_time;
        pr.queue_length = current_state.queue_length;
        for(int i=0; i < N_AXES; i++){
            pr.positions[i] = current_state.positions[i];
            pr.planner_positions[i] = current_state.planner_positions[i];
        }
        
        pr.header.stamp = ros::Time::now();
        
        switch(h.code){
            case CommandCode::ACK:  pr.code = stepper::PositionReport::ACK; break;
            case CommandCode::DONE:  pr.code = stepper::PositionReport::DONE; break;
            case CommandCode::EMPTY:  pr.code = stepper::PositionReport::EMPTY; break;
        }
        
        pub.publish(pr);
    }
    
    void publish(PacketHeader &h){}
    
};

class Listener {

protected:
    
    MessageProcessor mp;

public:

    Listener(MessageProcessor &mp):mp(mp){}


    void callback(const stepper::MoveCommand::ConstPtr& msg){
        
        std::stringstream ss;
        
        vector<int> x;

        ss << msg->t << ": ";
        for(int i = 0; i < N_AXES; i++){
            if (msg->movetype == stepper::MoveCommand::ABSOLUTE){
                //cout << "(" << mp.getPlannerPositions()[i] << ")";
                x.push_back(msg->x[i] - mp.getPlannerPositions()[i]);
            } else {
                x.push_back(msg->x[i]);
            }
        }

        for(int xi: x)
            ss << xi << " ";

        cout << "MOVE: " << ss.str().c_str() << endl; 
        ROS_DEBUG_NAMED("stepper", "MOVE: %s", ss.str().c_str());
        
        mp.sendMove(x);
        mp.read_next(0.01);
    }
};

// Read an axis configuration from parameters. 
bool readAxisConfig(int axis, ros::NodeHandle &nh, const MessageProcessor &mp, AxisConfig &as){

    std::string axisn = std::string("axis")+std::to_string(axis);

    if (!nh.hasParam(axisn)){
        return false;
    }

    std::map<std::string,int> pin_map;

    nh.getParam(axisn, pin_map);


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
    
    serial::Serial serial(port, baud, serial::Timeout::simpleTimeout(200));
    
    Publisher publisher;
    
    MessageProcessor mp(serial);
    mp.setMPH(&publisher);
    
    Listener listener(mp);
    
    ros::Subscriber sub = nh.subscribe("/stepper/cmd", 1000, &Listener::callback, &listener);
    ros::Publisher pub = nh.advertise<stepper::PositionReport>("/stepper/pos", 1000);
    
    publisher.setPub(pub);
    
    config(nh, mp);

    //mp.sendInfo();
    
    while (ros::ok()){
        if(mp.waitReadable())
            mp.read_next(0.05);
      ros::spinOnce();
    }


}
