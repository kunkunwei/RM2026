#include "Referee.h"
using namespace Device;

namespace Device{
    Referee &__HREFEREE__ = Referee::getInstance();
}

Referee &Referee::getInstance(){
    static Referee instance;
    return instance;
}

void Referee::init() {
    DataFifo.clear();
    RefereeTuple = {};
}

uint32_t Referee::pushData(uint8_t* data, uint32_t len){

    referee_time_out_count = 0;

    if (DataFifo.available()<len) {
        len = DataFifo.available();
    }

    for (uint32_t i = 0; i < len; i++){
        DataFifo.push(data[i]);
    }

    return len;
}

void Referee::packetWrite(uint16_t ID, uint8_t* data, uint16_t len){
    packetWriteTemplate(ID, data, len, RefereeTuple);
}

void Referee::ProcessData(){

    if(referee_time_out_count <= MAX_TIME_COUNT) referee_time_out_count++;

    using namespace boost::sml;

    while (!DataFifo.empty()){

        uint8_t pack_byte = DataFifo.front();
        depackFSM.process_event(RefereeFSM::unitData{pack_byte});

        if(depackFSM.is(state<RefereeFSM::PackChecked>)){
            uint16_t cmd_id = (depackCtx.protocol_packet[6]<<8) | depackCtx.protocol_packet[5];
            packetWrite(cmd_id, &depackCtx.protocol_packet[7], depackCtx.data_len);
        }

        DataFifo.pop();
    }
}

bool Referee::RefereeExist() const{
    return referee_time_out_count < MAX_TIME_COUNT;
}
