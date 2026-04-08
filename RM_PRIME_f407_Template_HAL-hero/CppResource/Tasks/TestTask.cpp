#include "TaskList.h"
#include "RGB_LED.h"
#include "Buzzer.h"
#include "usbd_cdc_if.h"

using namespace Device;


uint32_t MIDI_Seq_DJI[][2] = {
    {10, 50},
    {11, 50},
    {14, 50},
};


uint32_t MIDI_Seq[][2] = {
    {11, 80},
    {0, 16},
    {11, 66},
    {10, 30},
    {11, 42},
    {8, 42},
    {10, 96},
    {8, 36},
    {10, 48},
    {8, 36},
    {12, 48},
    {11, 176},
    {0, 16},

    {11, 78},
    {13, 72},
    {11, 24},
    {14, 48},

    {13, 40},
    {0, 8},
    {13, 32},

    {11, 40},
    {0, 8},
    {11, 66},
    {10, 30},
    {7, 30},
    {10, 60},
    {8, 162},

};

[[noreturn]] void TestTask(void const * argument){

    Buzzer &hBuzzer = Buzzer::getInstance();

    hBuzzer.on();
    for (auto &note : MIDI_Seq_DJI){
        hBuzzer.setFrequency(Buzzer::ToneList[note[0]]);
        osDelay(note[1]*5);
    }
    hBuzzer.off();

    RGB_LED &hLED = RGB_LED::getInstance();


    uint32_t col = 0;
    hLED.setColorPhase(col);


    //hBuzzer.toggle();


    while (true){


        //hBuzzer.toggle();
        //uint8_t msg[] = "Hello World!\r\n";
        //CDC_Transmit_FS(msg, sizeof(msg)-1);

        col = (col + 1) % 360;
        hLED.setColorPhase(col);

        // col = (col + 1) % 20;
        // hBuzzer.setFrequency(Buzzer::ToneList[2+col]);

        osDelay(50);

    }

}
