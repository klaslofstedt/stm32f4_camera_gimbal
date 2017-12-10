#include "pid.h"
#include "uart.h"

// Might wanna have a better error handling when using delta input as rate:
// https://github.com/Lauszus/LaunchPadFlightController/blob/fddbe4eb9303ea4301a714585d7383a2de275d80/src/PID.c

void pid_calc(pid_data_t* pid, unsigned long dt)
{
    // These are used for ease to read
    float p_term, i_term, d_term, error, output;
    //uart_printf(" input %.3f", pid->input);
    // Calculate input rate with derivation of position instead from EKF output
    // *** IMPORTANT *** Does it need low pass-filter?                        
    //pid->rate_calc = filter_lowpass((float)(1000 * (pid->input - pid->last_input) / (float)dt), pid->rate_calc, 0.75);
    
    // Calculate error between current and desired position
    error = pid->setpoint - pid->input;
    //uart_printf(" error %.3f", error);
    // Calculate the P contribution
    p_term = pid->k_p * error;
    //uart_printf(" p_term %.3f", p_term);

    // Calculate the I contribution
    pid->i_term += (float)(pid->k_i * (float)dt * ((error + pid->last_error) / 2.0f));
    // Check boundaries
    if(pid->i_term > pid->boundary_max){
        pid->i_term = pid->boundary_max;
    }
    else if(pid->i_term < pid->boundary_min) {
        i_term = pid->boundary_min;
    }
    i_term = pid->i_term;
    
    // Calculate the D contribution
    pid->rate = (pid->input - pid->last_input) / (float)dt; // Don't use this!!!
    //d_term = pid->k_d * (pid->input - pid->last_input) / (float)dt; 
    d_term = pid->k_d * pid->rate;

    //Calculate output
    output = p_term /*+ i_term */- d_term;
    //uart_printf(" output %.3f", output);

    // Check boundaries
    if(output > pid->boundary_max){
        output = pid->boundary_max;
    }
    else if(output < pid->boundary_min) {
        output = pid->boundary_min;
    }
    /*if(output > 0){
        output = pid->boundary_max - output;
    }
    else if(output < 0) {
        output = pid->boundary_min - output;
    }*/
    
    // Set data for output and next loop
    pid->output = output; 
    pid->last_input = pid->input;
    pid->last_error = error;
    //uart_printf(" pid->output %.3f\n\r", pid->output);
}