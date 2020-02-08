
#include "CRC.h"
#include "messages.h"
#include "cobs.h" 

#include <iostream>
#include <cstdio>
#include <chrono>
#include <algorithm>    // std::copy

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;

CRC::Table<std::uint8_t, 8> table(CRC::CRC_8());


size_t MessageProcessor::send(CommandCode code, const uint8_t* payload, size_t payload_len){
  
    PacketHeader h  = { seq,  code, 0};
    printf("A\n");
    memcpy(data_buf, &h, sizeof(PacketHeader));
    size_t  data_len = sizeof(PacketHeader);
        
    if (payload_len > 0){
        memcpy(data_buf+sizeof(PacketHeader), payload, payload_len);
        data_len +=payload_len;
    }
    printf("data_len=%d\n", (int)data_len);
    ((PacketHeader* )data_buf)->crc = CRC::Calculate(data_buf,data_len, table);

    size_t l = cobs_encode(data_buf, data_len, send_buf);
    send_buf[l] = 0;
    l++;
    printf("encode_size=%d\n",(int)l);
    size_t bytes_written = ser.write(send_buf, l);
    
    seq++;
    printf("bytes_writen=%d\n",(int)bytes_written);
    return bytes_written;
}

void MessageProcessor::sendInfo(){
    send(CommandCode::INFO, 0, 0);
    while(read_next(.2));
}

void MessageProcessor::sendConfig(const Config &config, std::vector<AxisConfig> axis_config){
    
    sendConfig(config);

    for(const AxisConfig &as : axis_config){
        sendAxisConfig(as);
    }

    while(read_next(.2));
}

void MessageProcessor::sendConfig(const Config &config){
    send(CommandCode::CONFIG, (const uint8_t*)&config, sizeof(config));
    while(read_next(.2));
}

void MessageProcessor::sendAxisConfig(const AxisConfig &axis_config){
    send(CommandCode::AXES, (const uint8_t*)&axis_config, sizeof(axis_config));
    while(read_next(.2));
}


void  MessageProcessor::sendMove(uint32_t t, vector<int> x){
    Moves m;
    m.segment_time = t;
     
    int i = 0;
    for( auto xi : x)
        if(i < N_AXES)
            m.x[i++] = xi;   
    
    send(CommandCode::MOVE, (const uint8_t*)&m, sizeof(m));
}

void  MessageProcessor::sendMove(uint32_t t, vector<double> x){
    vector<int> vi;
    std::copy(x.begin(), x.end(), vi.begin());
    sendMove(t, vi);
}

void  MessageProcessor::sendMove(vector<int> x) {
    sendMove(0,x);

}
void  MessageProcessor::sendMove(uint32_t t, std::initializer_list<int> il){
    sendMove(t, vector<int>(il.begin(), il.end()));
}
void  MessageProcessor::sendMove(uint32_t t, std::initializer_list<double> il){
    vector<int> vi;
    std::copy(il.begin(), il.end(), vi.begin());
    sendMove(t, vi);
}


bool MessageProcessor::update(){
    
    static int index = 0;
    
    while(ser.available()){
        ser.read(&rcv_buf[index], 1);
        if(rcv_buf[index] == 0){
            handle(rcv_buf, index);
            index = 0;
            return true;
        } else {
            index++;
        }
    }
    return false;
}

bool MessageProcessor::read_next(float timeout = .1){
    
    using namespace std::chrono;
    
    auto t1 = steady_clock::now();
    
    while(true){
        if(update())
            return true;
        
        auto t2 = steady_clock::now();
        auto ts = duration_cast<duration<double>>(t2 - t1).count();
        
        if(ts > timeout){
            return false;
        }  
    }
}


void MessageProcessor::handle(uint8_t* data, size_t len){
    
    size_t bytes_decoded = cobs_decode(data, len, (uint8_t*)data_buf);
    
    PacketHeader &h = *((PacketHeader*)data_buf);
    
    // SHould check CRC here. 

    char * payload = ((char*)(data_buf+sizeof(PacketHeader)));
    payload[bytes_decoded-sizeof(PacketHeader)] = '\0'; // In case the payload is a string. 
    
    last_code = h.code;
    
    switch(h.code){
        
        case CommandCode::ACK: 
        last_ack = h.seq;
        cout << "Ack "<<last_ack<<" ql="<<current_state.queue_length<<endl;
        memcpy((void*)&current_state, payload, sizeof(CurrentState));
        break;
        
        case CommandCode::NACK: 
        break;
        
        case CommandCode::DONE: 
        last_done = h.seq;
        cout << "Done "<<last_done<<" ql="<<current_state.queue_length<<endl;
        memcpy((void*)&current_state, payload, sizeof(CurrentState));
        break;
        
        case CommandCode::ERROR: 
        case CommandCode::MESSAGE: 
        cout << payload << endl;
        break;
        
        case CommandCode::ECHO:
        cout << "ECHO "<< payload << endl; 
        break;
        
    }
    
}
