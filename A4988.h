#ifndef A4988_H
#define A4988_H
#include <Arduino.h>
#include "BasicStepperDriver.h"

class A4988 : public BasicStepperDriver {
    protected:
        static const uint8_t MS_TABLE[];
        short ms1_pin = PIN_UNCONNECTED;
        short ms2_pin = PIN_UNCONNECTED;
        short ms3_pin = PIN_UNCONNECTED;
        // tA STEP minimum, HIGH pulse width (1us)
        static const int step_high_min = 1;
        // tB STEP minimum, LOW pulse width (1us)
        static const int step_low_min = 1;
        // wakeup time, nSLEEP inactive to STEP (1000us)
        static const int wakeup_time = 1000;
        // also 200ns between ENBL/DIR/MSx changes and STEP HIGH
    
        // Get the microstep table
        virtual const uint8_t* getMicrostepTable();
        virtual size_t getMicrostepTableSize();
    
        // Get max microsteps supported by the device
        short getMaxMicrostep() override;
    
    private:
        // microstep range (1, 16, 32 etc)
        static const short MAX_MICROSTEP = 16;
    
    public:
        A4988(short steps, short dir_pin, short step_pin);
        A4988(short steps, short dir_pin, short step_pin, short enable_pin);
        A4988(short steps, short dir_pin, short step_pin, short ms1_pin, short ms2_pin, short ms3_pin);
        A4988(short steps, short dir_pin, short step_pin, short enable_pin, short ms1_pin, short ms2_pin, short ms3_pin);
        A4988(short steps, short dir_pin, short step_pin, short stop_down, short stop_up); //Funcion agregada por JH

        void begin(short rpm=60, short microsteps=1);
        short setMicrostep(short microsteps) override;
};
#endif // A4988_H
