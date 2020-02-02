
#include "messages.h"


void MessageProcessor::update()
{

    while (_stream.available() > 0){
        
        uint8_t data = _stream.read();
        
        if (data == PacketMarker){
            uint8_t _decodeBuffer[_receiveBufferIndex];

            size_t numDecoded = EncoderType::decode(_receiveBuffer,
                                                    _receiveBufferIndex,
                                                    _decodeBuffer);

            _handler(_decodeBuffer, numDecoded);
            
            _receiveBufferIndex = 0;
            _recieveBufferOverflow = false;
        } else {
            if ((_receiveBufferIndex + 1) < ReceiveBufferSize){
                _receiveBuffer[_receiveBufferIndex++] = data;
               
            } else {
                // The buffer will be in an overflowed state if we write
                // so set a buffer overflowed flag.
             
                _recieveBufferOverflow = true;
            }
        }
    }
}
