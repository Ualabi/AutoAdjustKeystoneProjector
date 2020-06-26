#include "A4988.h"

const uint8_t A4988::MS_TABLE[] = {0b000, 0b001, 0b010, 0b011, 0b111};

A4988::A4988(short steps, short dir_pin, short step_pin)
:BasicStepperDriver(steps, dir_pin, step_pin)
{}

A4988::A4988(short steps, short dir_pin, short step_pin, short enable_pin)
:BasicStepperDriver(steps, dir_pin, step_pin, enable_pin)
{}

A4988::A4988(short steps, short dir_pin, short step_pin, short ms1_pin, short ms2_pin, short ms3_pin)
:BasicStepperDriver(steps, dir_pin, step_pin),
    ms1_pin(ms1_pin), ms2_pin(ms2_pin), ms3_pin(ms3_pin)
{}

A4988::A4988(short steps, short dir_pin, short step_pin, short enable_pin, short ms1_pin, short ms2_pin, short ms3_pin)
:BasicStepperDriver(steps, dir_pin, step_pin, enable_pin),
ms1_pin(ms1_pin), ms2_pin(ms2_pin), ms3_pin(ms3_pin)
{}

A4988::A4988(short steps, short dir_pin, short step_pin, short stop_down, short stop_up) //Funcion agregada por JH
:BasicStepperDriver(steps, dir_pin, step_pin, stop_down, stop_up)
{}

void A4988::begin(short rpm, short microsteps){
    BasicStepperDriver::begin(rpm, microsteps);

    if (IS_CONNECTED(ms1_pin) && IS_CONNECTED(ms2_pin) && IS_CONNECTED(ms3_pin)){
        pinMode(ms1_pin, OUTPUT);
        pinMode(ms2_pin, OUTPUT);
        pinMode(ms3_pin, OUTPUT);
    }
}

short A4988::setMicrostep(short microsteps){
    BasicStepperDriver::setMicrostep(microsteps);

    if (!IS_CONNECTED(ms1_pin) || !IS_CONNECTED(ms2_pin) || !IS_CONNECTED(ms3_pin)){
        return this->microsteps;
    }

    const uint8_t* ms_table = getMicrostepTable();
    size_t ms_table_size = getMicrostepTableSize();

    unsigned short i = 0;
    while (i < ms_table_size){
        if (this->microsteps & (1<<i)){
            uint8_t mask = ms_table[i];
            digitalWrite(ms3_pin, mask & 4);
            digitalWrite(ms2_pin, mask & 2);
            digitalWrite(ms1_pin, mask & 1);
            break;
        }
        i++;
    }
    return this->microsteps;
}

const uint8_t* A4988::getMicrostepTable(){
    return A4988::MS_TABLE;
}

size_t A4988::getMicrostepTableSize(){
    return sizeof(A4988::MS_TABLE);
}

short A4988::getMaxMicrostep(){
    return A4988::MAX_MICROSTEP;
}
