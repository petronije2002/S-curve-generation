#include "Controller.h"
#include "driver/mcpwm.h"

const float freqCarr = 19200;            // Carrier freq.


Controller::Controller(){

  // Constructor code


  pwm_config_A.frequency = freqCarr * 2;             // Set frequency in Hz
  pwm_config_A.counter_mode = MCPWM_UP_DOWN_COUNTER; // phase freq correct
  pwm_config_A.duty_mode = MCPWM_DUTY_MODE_0;        // active high PWM
  pwm_config_A.cmpr_a = 0.0;                         // duty cycle to 0%
  pwm_config_A.cmpr_b = 0.0;

};

Controller::~Controller() {
    // Destructor code, if needed
}


void Controller::setPWMConfig(float freq, int mode, int duty_mode, float cmpr_a, float cmpr_b) {
    pwm_config_A.frequency = freq;
    pwm_config_A.counter_mode = mode;
    pwm_config_A.duty_mode = duty_mode;
    pwm_config_A.cmpr_a = cmpr_a;
    pwm_config_A.cmpr_b = cmpr_b;
}

