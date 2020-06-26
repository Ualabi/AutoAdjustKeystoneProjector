#include "BasicStepperDriver.h"

// Funcion sencilla
BasicStepperDriver::BasicStepperDriver(short steps, short dir_pin, short step_pin)
:BasicStepperDriver(steps, dir_pin, step_pin, PIN_UNCONNECTED)
{
}

//Función para enable
BasicStepperDriver::BasicStepperDriver(short steps, short dir_pin, short step_pin, short enable_pin)
:motor_steps(steps), dir_pin(dir_pin), step_pin(step_pin), enable_pin(enable_pin)
{
	steps_to_cruise = 0;
	steps_remaining = 0;
	dir_state = 0;
	steps_to_brake = 0;
	step_pulse = 0;
    cruise_step_pulse = 0;
	rest = 0;
	step_count = 0;
}

//////////// Función agregada JH
// Para límites superiores e inferiores
BasicStepperDriver::BasicStepperDriver(short steps, short dir_pin, short step_pin, short stop_down, short stop_up)
:motor_steps(steps), dir_pin(dir_pin), step_pin(step_pin), stop_down(stop_down), stop_up(stop_up)
{}

void BasicStepperDriver::begin(float rpm, short microsteps){
    pinMode(dir_pin, OUTPUT);
    digitalWrite(dir_pin, HIGH);

    pinMode(step_pin, OUTPUT);
    digitalWrite(step_pin, LOW);

    if IS_CONNECTED(enable_pin){
        pinMode(enable_pin, OUTPUT);
        disable();
    }

    //Funcion agregadas por JH
    if IS_CONNECTED(stop_up){
        pinMode(stop_up, INPUT);
        if IS_CONNECTED(stop_down){
            pinMode(stop_down, INPUT);
        }
    }

    auxa = microsteps*50;

    this->rpm = rpm;
    setMicrostep(microsteps);

    enable();
}

void BasicStepperDriver::setRPM(float rpm){
    if (this->rpm == 0){        // begin() has not been called (old 1.0 code)
        begin(rpm, microsteps);
    }
    this->rpm = rpm;
}

/*
 * Set stepping mode (1:microsteps)
 * Allowed ranges for BasicStepperDriver are 1:1 to 1:128
 */
short BasicStepperDriver::setMicrostep(short microsteps){
    for (short ms=1; ms <= getMaxMicrostep(); ms<<=1){
        if (microsteps == ms){
            this->microsteps = microsteps;
            break;
        }
    }
    return this->microsteps;
}

/*
 * Set speed profile - CONSTANT_SPEED, LINEAR_SPEED (accelerated)
 * accel and decel are given in [full steps/s^2]
 */
void BasicStepperDriver::setSpeedProfile(Mode mode, short accel, short decel){
    profile.mode = mode;
    profile.accel = accel;
    profile.decel = decel;
}
void BasicStepperDriver::setSpeedProfile(struct Profile profile){
    this->profile = profile;
}

/*
 * Move the motor a given number of steps.
 * positive to move forward, negative to reverse
 */
void BasicStepperDriver::move(long steps){
    startMove(steps);
    while (nextAction());
}

//-----------------------------------------------------//
void BasicStepperDriver::moveuntil(short stop_pin){
    dir_state = (stop_pin == stop_up) ? HIGH : LOW;
    
    last_action_end = 0;
    step_count = 0;
    step_pulse = STEP_PULSE(rpm, motor_steps, microsteps);

    while ( digitalRead(stop_pin) == LOW ){
        delayMicros(next_action_interval, last_action_end);
        digitalWrite(dir_pin, dir_state);
        digitalWrite(step_pin, HIGH);
        unsigned m = micros();
        m = micros() - m;
        if (m < step_high_min){
            delayMicros(step_high_min-m);
            m = step_high_min;
        };
        digitalWrite(step_pin, LOW);
        step_count++;
        last_action_end = micros();
        next_action_interval = (step_pulse > m) ? step_pulse - m : 1;
    }
    last_action_end = 0;
    next_action_interval = 0;
}

void BasicStepperDriver::moveuntilAn(short stop_pin){
    dir_state = (stop_pin == stop_up) ? HIGH : LOW;
    
    last_action_end = 0;
    step_count = 0;
    step_pulse = STEP_PULSE(rpm, motor_steps, microsteps);

    while ( analogRead(stop_pin) < 1000 ){
        delayMicros(next_action_interval, last_action_end);
        digitalWrite(dir_pin, dir_state);
        digitalWrite(step_pin, HIGH);
        unsigned m = micros();
        m = micros() - m;
        if (m < step_high_min){
            delayMicros(step_high_min-m);
            m = step_high_min;
        };
        digitalWrite(step_pin, LOW);
        step_count++;
        last_action_end = micros();
        next_action_interval = (step_pulse > m) ? step_pulse - m : 1;
    }
    last_action_end = 0;
    next_action_interval = 0;
}

void BasicStepperDriver::moveuntilDi(short stop_pin){
    dir_state = (stop_pin == stop_up) ? HIGH : LOW;
    
    last_action_end = 0;
    step_count = 0;
    step_pulse = STEP_PULSE(rpm, motor_steps, microsteps);

    while ( digitalRead(stop_pin) == LOW ){
        delayMicros(next_action_interval, last_action_end);
        digitalWrite(dir_pin, dir_state);
        digitalWrite(step_pin, HIGH);
        unsigned m = micros();
        m = micros() - m;
        if (m < step_high_min){
            delayMicros(step_high_min-m);
            m = step_high_min;
        };
        digitalWrite(step_pin, LOW);
        step_count++;
        last_action_end = micros();
        next_action_interval = (step_pulse > m) ? step_pulse - m : 1;
    }
    last_action_end = 0;
    next_action_interval = 0;
}

void BasicStepperDriver::allwayUp(void){
    bool u1 = true, u2 = true, u3 = true;
    
    last_action_end = 0;
    step_pulse = STEP_PULSE(rpm/3, motor_steps, microsteps);

    digitalWrite(8, HIGH);
    digitalWrite(6, HIGH);
    digitalWrite(4, HIGH);
    while ( u1 || u2 || u3 ){
        delayMicros(next_action_interval, last_action_end);
        if (u1) digitalWrite(9, HIGH);
        if (u2) digitalWrite(7, HIGH);
        if (u3) digitalWrite(5, HIGH);
        
        unsigned m = micros();
        m = micros() - m;
        if (m < step_high_min){
            delayMicros(step_high_min-m);
            m = step_high_min;
        };
        
        if (u1) digitalWrite(9, LOW);
        if (u2) digitalWrite(7, LOW);
        if (u3) digitalWrite(5, LOW);

        if(u1 && digitalRead(11) == HIGH) u1 = false;
        if(u2 && digitalRead(A1) == HIGH) u2 = false;
        if(u3 && digitalRead(A3) == HIGH) u3 = false;
        
        last_action_end = micros();
        next_action_interval = (step_pulse > m) ? step_pulse - m : 1;
    }
    last_action_end = 0;
    next_action_interval = 0;
}


void BasicStepperDriver::allwayDo(void){
    bool d1 = true, d2 = true, d3 = true;
    
    last_action_end = 0;
    step_pulse = STEP_PULSE(rpm/3, motor_steps, microsteps);
    
    digitalWrite(8, LOW);
    digitalWrite(6, LOW);
    digitalWrite(4, LOW);
    unsigned m = 0;
    while ( d1 || d2 || d3 ){
        delayMicros(next_action_interval, last_action_end);
        if (d1) digitalWrite(9, HIGH);
        if (d2) digitalWrite(7, HIGH);
        if (d3) digitalWrite(5, HIGH);
        
        m = micros();
        m = micros() - m;
        if (m < step_high_min){
            delayMicros(step_high_min-m);
            m = step_high_min;
        };
        
        if (d1) digitalWrite(9, LOW);
        if (d2) digitalWrite(7, LOW);
        if (d3) digitalWrite(5, LOW);

        if(d1 && digitalRead(12) == HIGH) d1 = false;
        if(d2 && digitalRead(A0) == HIGH) d2 = false;
        if(d3 && digitalRead(A2) == HIGH) d3 = false;
        
        last_action_end = micros();
        next_action_interval = (step_pulse > m) ? step_pulse - m : 1;
    }
    last_action_end = 0;
    next_action_interval = 0;
}

////////////////////////////////////////////////////////
//Funcion agregada por MV
void BasicStepperDriver::centrarMotor(void){
  moveuntil(A6);
  moveuntil(A7);
  move(step_count/2);
  step_count=0;
}
//

/*
 * Move the motor a given number of degrees (1-360)
 */
void BasicStepperDriver::rotate(long deg){
    move(calcStepsForRotation(deg));
}
/*
 * Move the motor with sub-degree precision.
 * Note that using this function even once will add 1K to your program size
 * due to inclusion of float support.
 */
void BasicStepperDriver::rotate(double deg){
    move(calcStepsForRotation(deg));
}

/*
 * Set up a new move (calculate and save the parameters)
 */
void BasicStepperDriver::startMove(long steps, long time){
    float speed;
    // set up new move
    dir_state = (steps >= 0) ? HIGH : LOW;
    last_action_end = 0;
    steps_remaining = abs(steps);
    step_count = 0;
    rest = 0;
    switch (profile.mode){
    case LINEAR_SPEED:
        // speed is in [steps/s]
        speed = rpm * motor_steps / 60;
        if (time > 0){
            // Calculate a new speed to finish in the time requested
            float t = time / (1e+6);                  // convert to seconds
            float d = steps_remaining / microsteps;   // convert to full steps
            float a2 = 1.0 / profile.accel + 1.0 / profile.decel;
            float sqrt_candidate = t*t - 2 * a2 * d;  // in √b^2-4ac
            if (sqrt_candidate >= 0){
                speed = min(speed, (t - (float)sqrt(sqrt_candidate)) / a2);
            };
        }
        // how many microsteps from 0 to target speed
        steps_to_cruise = microsteps * (speed * speed / (2 * profile.accel));
        // how many microsteps are needed from cruise speed to a full stop
        steps_to_brake = steps_to_cruise * profile.accel / profile.decel;
        if (steps_remaining < steps_to_cruise + steps_to_brake){
            // cannot reach max speed, will need to brake early
            steps_to_cruise = steps_remaining * profile.decel / (profile.accel + profile.decel);
            steps_to_brake = steps_remaining - steps_to_cruise;
        }
        // Initial pulse (c0) including error correction factor 0.676 [us]
        step_pulse = (1e+6)*0.676*sqrt(2.0f/profile.accel/microsteps);
        // Save cruise timing since we will no longer have the calculated target speed later
        cruise_step_pulse = 1e+6 / speed / microsteps;
        break;

    case CONSTANT_SPEED:
    default:
        steps_to_cruise = 0;
        steps_to_brake = 0;
        step_pulse = cruise_step_pulse = STEP_PULSE(rpm, motor_steps, microsteps);
        if (time > steps_remaining * step_pulse){
            step_pulse = (float)time / steps_remaining;
        }
    }
}
/*
 * Alter a running move by adding/removing steps
 * FIXME: This is a naive implementation and it only works well in CRUISING state
 */
void BasicStepperDriver::alterMove(long steps){
    switch (getCurrentState()){
    case ACCELERATING: // this also works but will keep the original speed target
    case CRUISING:
        if (steps >= 0){
            steps_remaining += steps;
        } else {
            steps_remaining = max(steps_to_brake, steps_remaining+steps);
        };
        break;
    case DECELERATING:
        // would need to start accelerating again -- NOT IMPLEMENTED
        break;
    case STOPPED:
        startMove(steps);
        break;
    }
}
/*
 * Brake early.
 */
void BasicStepperDriver::startBrake(void){
    switch (getCurrentState()){
    case CRUISING:  // this applies to both CONSTANT_SPEED and LINEAR_SPEED modes
        steps_remaining = steps_to_brake;
        break;

    case ACCELERATING:
        steps_remaining = step_count * profile.accel / profile.decel;
        break;

    default:
        break; // nothing to do if already stopped or braking
    }
}
/*
 * Stop movement immediately and return remaining steps.
 */
long BasicStepperDriver::stop(void){
    long retval = steps_remaining;
    steps_remaining = 0;
    return retval;
}
/*
 * Return calculated time to complete the given move
 */
long BasicStepperDriver::getTimeForMove(long steps){
    float t;
    if (steps == 0){
        return 0;
    }
    switch (profile.mode){
        case LINEAR_SPEED:
            startMove(steps);
            if (steps_remaining >= steps_to_cruise + steps_to_brake){
                float speed = rpm * motor_steps / 60;   // full steps/s
                t = (steps / (microsteps * speed)) + (speed / (2 * profile.accel)) + (speed / (2 * profile.decel)); // seconds
            } else {
                t = sqrt(2.0 * steps_to_cruise / profile.accel / microsteps) +
                    sqrt(2.0 * steps_to_brake / profile.decel / microsteps);
            }
            t *= (1e+6); // seconds -> micros
            break;
        case CONSTANT_SPEED:
        default:
            t = steps * STEP_PULSE(rpm, motor_steps, microsteps);
    }
    return round(t);
}
/*
 * Move the motor an integer number of degrees (360 = full rotation)
 * This has poor precision for small amounts, since step is usually 1.8deg
 */
void BasicStepperDriver::startRotate(long deg){
    startMove(calcStepsForRotation(deg));
}
/*
 * Move the motor with sub-degree precision.
 * Note that calling this function will increase program size substantially
 * due to inclusion of float support.
 */
void BasicStepperDriver::startRotate(double deg){
    startMove(calcStepsForRotation(deg));
}

/*
 * calculate the interval til the next pulse
 */
void BasicStepperDriver::calcStepPulse(void){
    if (steps_remaining <= 0){  // this should not happen, but avoids strange calculations
        return;
    }
    steps_remaining--;
    step_count++;

    if (profile.mode == LINEAR_SPEED){
        switch (getCurrentState()){
        case ACCELERATING:
            if (step_count < steps_to_cruise){
                step_pulse = step_pulse - (2*step_pulse+rest)/(4*step_count+1);
                rest = (step_count < steps_to_cruise) ? (2*step_pulse+rest) % (4*step_count+1) : 0;
            } else {
                // The series approximates target, set the final value to what it should be instead
                step_pulse = cruise_step_pulse;
            }
            break;

        case DECELERATING:
            step_pulse = step_pulse - (2*step_pulse+rest)/(-4*steps_remaining+1);
            rest = (2*step_pulse+rest) % (-4*steps_remaining+1);
            break;

        default:
            break; // no speed changes
        }
    }
}
/*
 * Yield to step control
 * Toggle step and return time until next change is needed (micros)
 */
long BasicStepperDriver::nextAction(void){
    if (steps_remaining > 0){
        delayMicros(next_action_interval, last_action_end);
        /*
         * DIR pin is sampled on rising STEP edge, so it is set first
         */
        digitalWrite(dir_pin, dir_state);
        digitalWrite(step_pin, HIGH);
        unsigned m = micros();
        unsigned long pulse = step_pulse; // save value because calcStepPulse() will overwrite it
        calcStepPulse();
        // We should pull HIGH for at least 1-2us (step_high_min)
        delayMicros(step_high_min);
        digitalWrite(step_pin, LOW);
        // account for calcStepPulse() execution time; sets ceiling for max rpm on slower MCUs
        last_action_end = micros();
        m = last_action_end - m;
        next_action_interval = (pulse > m) ? pulse - m : 1;
    } else {
        // end of move
        last_action_end = 0;
        next_action_interval = 0;
    }
    return next_action_interval;
}

enum BasicStepperDriver::State BasicStepperDriver::getCurrentState(void){
    enum State state;
    if (steps_remaining <= 0){
        state = STOPPED;
    } else {
        if (steps_remaining <= steps_to_brake){
            state = DECELERATING;
        } else if (step_count <= steps_to_cruise){
            state = ACCELERATING;
        } else {
            state = CRUISING;
        }
    }
    return state;
}
/*
 * Configure which logic state on ENABLE pin means active
 * when using SLEEP (default) this is active HIGH
 */
void BasicStepperDriver::setEnableActiveState(short state){
    enable_active_state = state;
}
/*
 * Enable/Disable the motor by setting a digital flag
 */
void BasicStepperDriver::enable(void){
    if IS_CONNECTED(enable_pin){
        digitalWrite(enable_pin, enable_active_state);
    };
    delayMicros(2);
}

void BasicStepperDriver::disable(void){
    if IS_CONNECTED(enable_pin){
        digitalWrite(enable_pin, (enable_active_state == HIGH) ? LOW : HIGH);
    }
}

short BasicStepperDriver::getMaxMicrostep(){
    return BasicStepperDriver::MAX_MICROSTEP;
}
