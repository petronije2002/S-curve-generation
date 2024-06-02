


#ifndef CONTROLLER_H
#define CONTROLLER_H



class Controller {

    public: 
        Controller();

        ~Controller();

        void setPWMConfig(float freq, int mode, int duty_mode, float cmpr_a, float cmpr_b);




    private: 

        struct mcpwm_config_t {
            float frequency;    // Frequency in Hz
            int counter_mode;   // Phase frequency correct mode
            int duty_mode;      // Active high PWM mode
            float cmpr_a;       // Duty cycle for channel A
            float cmpr_b;       // Duty cycle for channel B
        };



        mcpwm_config_t pwm_config_A;

       
};




#endif