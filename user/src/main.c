// C includes
#include "stdio.h"
#include <math.h> // pi and sinf
#include <stdbool.h>
// Hardware includes
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_rcc.h"  
#include "stm32f4xx_tim.h"

// FreeRTOS kernel includes
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "queue.h"
// User includes
#include "main.h"
#include "imu.h"
#include "pid.h"
#include "uart.h"
#include "i2c.h" // i2c2 imu
#include "joystick.h"
#include "board.h"
#include "motordriver.h"
#include "debug.h"

#define M_PI acos(-1.0) // pi
//min = 1 och max = 1000 gives resolution cycle_size/1000 = 1000
#define MOTOR_SPEED_MIN 1
#define MOTOR_SPEED_MAX 1000


static void main_task(void *pvParameters);      // dmp speed (200Hz)
static void motordriver_loop_init(void);          // sinus speed (10kHz)
static void telemetry_task(void *pvParameters); // 20 Hz
// object to measure the main stack size
static UBaseType_t stack_size_main;
// imu object containing pitch, roll, yaw data
static imu_data_t imu;
// 10k update frequency for driver data
static const uint32_t motordriver_interrupt_period = 84000000/10000;
// global arm
static volatile bool arm = false;
// global joystick variables. only used for changing pid values in debugging
static uint16_t adc_ch10 = 0;
static uint16_t adc_ch11 = 0;

static pid_data_t pid_roll = {
    .input = 0,
    .last_input = 0,
    .rate = 0,
	.setpoint = 0,
	.i_term = 0.0f,
    .boundary_max = 1.0f,
    .boundary_min = -1.0f,
    .last_error = 0,
	.output = 0,
    .k_p = 0.03f,       // 0.04
    .k_i = 0.0f,        //
    .k_d = -0.03f       //
};

static pid_data_t pid_pitch = {
    .input = 0,
    .last_input = 0,
    .rate = 0,
	.setpoint = 0,
	.i_term = 0.0f,
    .boundary_max = 1.0f,
    .boundary_min = -1.0f,
    .last_error = 0,
	.output = 0,
    .k_p = 0.0086f,     //0.065f, // 0.015
    .k_i = 0.0f,
    .k_d = -0.025f      //0.24f   // -0.01
};

static volatile driver_data_t driver_roll = {
    .dir = false,
    .step_size = 0,
    .cycle_size = 1000000,
    .step = 0,
    .reminder = 0,
    .power = 0.7f
};

static volatile driver_data_t driver_pitch = {
    .dir = false,
    .step_size = 0,
    .cycle_size = 1000000,
    .step = 0,
    .reminder = 0,
    .power = 0.4f
};

static joystick_data_t joystick_x = {
    .channel = ADC_Channel_10,
    .deadband = 50,
    .middle = 2490,
    .raw_min = 180,
    .raw_max = 4095,
    .min = -90,
    .max = 90,
    .step_size = 0.3,
    .threshold = 20,
    .setpoint = 0
};

static joystick_data_t joystick_y = {
    .channel = ADC_Channel_11,
    .deadband = 50,
    .middle = 2020,
    .raw_min = 0,
    .raw_max = 4095,
    .min = -90,
    .max = 90,
    .step_size = 0.3,
    .threshold = 20,
    .setpoint = 0
};

static void cpu_waste(__IO uint32_t cycles)
{
    while(cycles--){
        __asm("nop"); // do nothing
    }
}

void TIM7_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
        
        GPIO_SetBits(DEBUG_GPIO_PORT, DEBUG_GPIOE_13);
        if(arm){
            // driver 0
            
            // driver 1
            if(driver_roll.dir){
                driver_roll.step += driver_roll.step_size;
                if(driver_roll.step >= driver_roll.cycle_size){
                    driver_roll.reminder = driver_roll.step - driver_roll.cycle_size;
                    driver_roll.step = driver_roll.reminder;
                }
                
            }
            else{
                driver_roll.step -= driver_roll.step_size;
                if(driver_roll.step <= 0){
                    driver_roll.reminder = -driver_roll.step;
                    driver_roll.step = driver_roll.cycle_size - driver_roll.reminder;
                }
                
            }
            //driver 2
            if(driver_pitch.dir){
                driver_pitch.step += driver_pitch.step_size;
                if(driver_pitch.step >= driver_pitch.cycle_size){
                    driver_pitch.reminder = driver_pitch.step - driver_pitch.cycle_size;
                    driver_pitch.step = driver_pitch.reminder;
                }
                
            }
            else{
                driver_pitch.step -= driver_pitch.step_size;
                if(driver_pitch.step <= 0){
                    driver_pitch.reminder = -driver_pitch.step;
                    driver_pitch.step = driver_pitch.cycle_size - driver_pitch.reminder;
                }
                
            }
            // Update the global variables with new values
            motordriver_set1(driver_roll.step, driver_roll.cycle_size, driver_roll.power);
            motordriver_set2(driver_pitch.step, driver_pitch.cycle_size, driver_pitch.power);
        }
        GPIO_ResetBits(DEBUG_GPIO_PORT, DEBUG_GPIOE_13);
    }
    
}

static void motordriver_loop_init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
    
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure; 
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = motordriver_interrupt_period;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);
    TIM_Cmd(TIM7, ENABLE);
    TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
    
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn; 
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 10;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}


void main_task(void *pvParameters)
{
    stack_size_main = uxTaskGetStackHighWaterMark(NULL); // Not needed?
    delay_ms(5000); // wait for imu to stabilize
    arm = true; // used by the driver timer to be activated after IMU
    
    while(1){
        if(xQueueReceive(imu_attitude_queue, &imu, 200)){ // Sample 200Hz
            
            GPIO_SetBits(DEBUG_GPIO_PORT, DEBUG_MAIN_TASK_PIN);
            
            /*adc_ch10 = joystick_read_raw(ADC_Channel_10);
            adc_ch11 = joystick_read_raw(ADC_Channel_11);
            
            if(adc_ch10 > 3500){
                pid_pitch.k_p = pid_pitch.k_p + 0.00005;
            }
            if(adc_ch10 < 1000){
                pid_pitch.k_p = pid_pitch.k_p - 0.00005;
            }
            if(adc_ch11 > 3500){
                pid_pitch.k_d = pid_pitch.k_d + 0.00005;
            }
            if(adc_ch11 < 1000){
                pid_pitch.k_d = pid_pitch.k_d - 0.00005;
            }*/
            // Build pitch PID object ------------------------------------------
            pid_pitch.input = imu.dmp_pitch;
            pid_pitch.setpoint = joystick_read(&joystick_x);
            //pid_pitch.rate = imu.gyro_pitch;
            // Build roll PID object -------------------------------------------
            //pid_roll.setpoint = 0;//joystick_read_setpoint(&joystick_roll);
            pid_roll.input = imu.dmp_roll;
            pid_roll.setpoint = joystick_read(&joystick_y);
            //pid_roll.rate = imu.gyro_roll;
            // Build yaw PID object --------------------------------------------
            //pid_yaw.setpoint = joystick_read_setpoint(&joystick_yaw);
            //pid_yaw.input = imu.gyro_yaw;
            
            
            // Calculate PID outputs
            pid_calc(&pid_pitch, imu.dt);
            pid_calc(&pid_roll, imu.dt);
            
            // Calculate roll motor driver step size and direction
            if(pid_roll.output > 0){
                driver_roll.step_size = (uint32_t)(MOTOR_SPEED_MAX - ((1-pid_roll.output) * (float)(MOTOR_SPEED_MAX - MOTOR_SPEED_MIN)));
                driver_roll.dir = true;
            }
            if(pid_roll.output < 0){
                driver_roll.step_size = (uint32_t)(MOTOR_SPEED_MAX - ((pid_roll.output+1) * (float)(MOTOR_SPEED_MAX - MOTOR_SPEED_MIN)));
                driver_roll.dir = false;
            }
            // Calculate pitch motor driver step size and direction
            if(pid_pitch.output > 0){
                driver_pitch.step_size = (uint32_t)(MOTOR_SPEED_MAX - ((1-pid_pitch.output) * (float)(MOTOR_SPEED_MAX - MOTOR_SPEED_MIN)));
                driver_pitch.dir = true;
            }
            if(pid_pitch.output < 0){
                driver_pitch.step_size = (uint32_t)(MOTOR_SPEED_MAX - ((pid_pitch.output+1) * (float)(MOTOR_SPEED_MAX - MOTOR_SPEED_MIN)));
                driver_pitch.dir = false;
            }
            
            // Read the size of this task in order to refine assigned stack size
            stack_size_main = uxTaskGetStackHighWaterMark(NULL);
            GPIO_ResetBits(DEBUG_GPIO_PORT, DEBUG_MAIN_TASK_PIN);
        }
        else{
            uart_printf("No IMU queue\n\r");
        }
    }
}




void telemetry_task(void *pvParameters)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t frequency = 200; // every 200ms = 5Hz
    UBaseType_t stack_size_tele;
    stack_size_tele = uxTaskGetStackHighWaterMark(NULL);
    while(1)
    {
        vTaskDelayUntil(&last_wake_time, frequency / portTICK_PERIOD_MS);
        GPIO_SetBits(DEBUG_GPIO_PORT, DEBUG_TEL_TASK_PIN);
        //uart_printf("armed: %d", arm());
        //uart_printf(" roll dmp: %.3f", imu.dmp_roll);
        uart_printf(" pitch dmp: %.3f", imu.dmp_pitch);
        //uart_printf(" roll rate: %.3f", pid_roll.rate);
        uart_printf(" stepsize: %d", driver_pitch.step_size);
        uart_printf(" kp: %f", pid_pitch.k_p);
        uart_printf(" kd: %f", pid_pitch.k_d);
        //uart_printf(" p: %f", -pid_roll.k_p * pid_roll.input);
        //uart_printf(" d: %f", pid_roll.k_d * pid_roll.rate);
        //uart_printf(" out: %f", pid_roll.output);
        uart_printf(" setpoint: %.3f", pid_pitch.setpoint);
        //uart_printf(" pitch dmp: %.3f", imu.dmp_pitch);
        //uart_printf(" max: %.3f", pid_roll.boundary_max);
        //uart_printf(" min: %.3f", pid_roll.boundary_min);
        //uart_printf(" yaw dmp ori: %.3f", imu.dmp_yaw);
        //uart_printf(" y: %d", adc_ch10);
        //uart_printf(" x: %d", adc_ch11);
        //uart_printf(" x: %f", joystick_x.value);
        //uart_printf(" y: %f", joystick_y.value);
        uart_printf("\n\r"); 
        
        GPIO_ResetBits(DEBUG_GPIO_PORT, DEBUG_TEL_TASK_PIN);
        stack_size_tele = uxTaskGetStackHighWaterMark(NULL);
    }
}


/*
I/O pins used to control the motor driver.
This for the STM32F4 Discovery board.
Channel 1 IN use channel 1/2/4 of TIM5 on PA0/1/3.
Channel 2 IN use channel 1/3/4 of TIM3 on PC6/8/9.
Channel 3 IN use channel 1/3/4 of TIM2 on PA15/PA2/PB11.
EN use pins PE7-15
*/

int main(void)
{
    // Setup STM32 system (clock, PLL and Flash configuration)
    SystemInit();
    // Update the system clock variable (is this already done?)
    SystemCoreClockUpdate();
    // This on is set to 4 according to FreeRTOS spec. Do not change.
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    
    // Peripherals and stuff
    uart_init();            // printf
    debug_init();           // toggling GPIOs for saleae logic
    i2c2_init();            // imu camera side
    joystick_init();        // joystick
    motordriver_loop_init();// loop updating sinus values
    motordriver_init();     // init L6234 timers
    cpu_waste(2000000);
    
    uart_printf("CoreClock: %d\n\r", SystemCoreClock);
    
    // Reads the imu and pass data on interrupt to main_task
    xTaskCreate(imu_task, (const char *)"imu_task", 350, NULL, 4, NULL);
    // Prints debug data
    xTaskCreate(telemetry_task, (const char *)"telemetry_task", 300, NULL, 1, NULL);
    // Init ESC, reads joystick and process all data before setting new output to motor drivers
    xTaskCreate(main_task, (const char *)"main_task", 300, NULL, 3, NULL);
    
    vTaskStartScheduler();
    
    // should never be reached
    for(;;);
}



