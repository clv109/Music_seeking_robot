/* DriverLib Includes */
#include "driverlib.h"
/* Standard Includes */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../inc/SysTick.h"
#include "../inc/Clock.h"

uint32_t Size;
uint32_t I;
uint16_t c;
int32_t INPUT_P6_1[1024];
float Real_INPUT_P6_1[1024];
int32_t INPUT_P6_0[1024];
float Real_INPUT_P6_0[1024];
float y_left[1024];
float y_right[1024];
float alpha;
uint8_t DIRECTION;
uint16_t dynamic_left_speed = 0;
uint16_t dynamic_right_speed = 0;
#define FORWARD    1
#define BACKWARD   2
#define LEFT       3
#define RIGHT      4
#define STOP       5
uint8_t MODE;
#define SAMPLING_MODE    1
#define RUNNING_MODE     2
/////////////////////////////////////////////////////////////
#define PERIOD   100
/* Timer_A UpMode Configuration Parameter */
const Timer_A_UpModeConfig upConfig =
{
    TIMER_A_CLOCKSOURCE_SMCLK,              // SMCLK Clock Source 12MHz
    TIMER_A_CLOCKSOURCE_DIVIDER_12,         // SMCLK/12 = 1MHz Timer clock
    PERIOD,                                 // a period of 100 timer clocks => 10 KHz Frequency
    TIMER_A_TAIE_INTERRUPT_DISABLE,         // Disable Timer interrupt
    TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE ,    // Enable CCR0 interrupt
    TIMER_A_DO_CLEAR                        // Clear value
};
/* Application Defines */
#define TIMER_PERIOD 15000  // 10 ms PWM Period
#define DUTY_CYCLE1 0
#define DUTY_CYCLE2 0
/* Timer_A UpDown Configuration Parameter */
Timer_A_UpDownModeConfig upDownConfig =
{
    TIMER_A_CLOCKSOURCE_SMCLK,              // SMCLK Clock Source
    TIMER_A_CLOCKSOURCE_DIVIDER_4,          // SMCLK/1 = 1.5MHz
    TIMER_PERIOD,                           // 15000 period
    TIMER_A_TAIE_INTERRUPT_DISABLE,         // Disable Timer interrupt
    TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE,    // Disable CCR0 interrupt
    TIMER_A_DO_CLEAR                        // Clear value
};
/* Timer_A Compare Configuration Parameter  (PWM1) */
Timer_A_CompareModeConfig compareConfig_PWM1 =
{
        TIMER_A_CAPTURECOMPARE_REGISTER_1,          // Use CCR3
        TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE,   // Disable CCR interrupt
        TIMER_A_OUTPUTMODE_TOGGLE_RESET,            // Toggle output
        DUTY_CYCLE1
};
/* Timer_A Compare Configuration Parameter (PWM2) */
Timer_A_CompareModeConfig compareConfig_PWM2 =
{
        TIMER_A_CAPTURECOMPARE_REGISTER_2,          // Use CCR4
        TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE,   // Disable CCR interrupt
        TIMER_A_OUTPUTMODE_TOGGLE_RESET,            // Toggle output
        DUTY_CYCLE2
};
/////////////////////////////////////////////////////////////
void TimerA2_Init(void);
void PWM_Init12(void);
void PWM_duty1(uint16_t duty1, Timer_A_CompareModeConfig* data);
void PWM_duty2(uint16_t duty1, Timer_A_CompareModeConfig* data);
void MotorInit(void);
void motor_forward(uint16_t leftDuty, uint16_t rightDuty);
void motor_right(uint16_t leftDuty, uint16_t rightDuty);
void motor_left(uint16_t leftDuty, uint16_t rightDuty);
void motor_backward(uint16_t leftDuty, uint16_t rightDuty);
void motor_stop(void);
void ADC_Ch14Ch15_Init(void);
// --- ESP32 UART Configuration (9600 Baud at 12 MHz SMCLK) ---
const eUSCI_UART_Config uartESP32Config = {
    EUSCI_A_UART_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source (12 MHz)
    78,                                      // BRDIV = 78
    2,                                       // UCxBRF = 2
    0,                                       // UCxBRS = 0
    EUSCI_A_UART_NO_PARITY,                  // No Parity
    EUSCI_A_UART_LSB_FIRST,                  // LSB First
    EUSCI_A_UART_ONE_STOP_BIT,               // One stop bit
    EUSCI_A_UART_MODE,                       // UART mode
    EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION  // Oversampling
};
// --- Function to transmit text to the ESP32 (for OLED display) ---
void Send_To_ESP32(char *str) {
    while (*str != '\0') {
        while (!(UCA2IFG & UCTXIFG)); // Wait until transmit buffer is ready
        UART_transmitData(EUSCI_A2_BASE, *str);
        str++;
    }
}

int main(void)
{
    Size = 1000;
    I = Size - 1;
    // Set Microcontroller Clock = 48 MHz
    Clock_Init48MHz();
    PWM_Init12();
    SysTick_Init();
    MotorInit();
    // Port 6 Configuration: make P6.4 out
    GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN4);
    // Initialize RGB LED
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2);
    // --- Configure P4.3 and P4.4 as Input to read obstacle alert HIGH/LOW from ESP32 ---
    GPIO_setAsInputPin(GPIO_PORT_P4, GPIO_PIN3);
    GPIO_setAsInputPin(GPIO_PORT_P4, GPIO_PIN4);
    // --- Initialize ESP32 Telemetry UART TX (P3.3 TX) ---
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P3, GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
    UART_initModule(EUSCI_A2_BASE, &uartESP32Config);
    UART_enableModule(EUSCI_A2_BASE);
    // Setup ADC for Channel A14 and A15
    ADC_Ch14Ch15_Init();
    // Timer A2 Configuration
    TimerA2_Init();
    DIRECTION = FORWARD;
    MODE = SAMPLING_MODE;
    while (1) {
        // Main loop remains empty as processing is handled in the timer interrupt
    }
}

void TA2_0_IRQHandler(void)
{
    char telemetry_buffer[50]; // Buffer to hold our text message
    GPIO_toggleOutputOnPin(GPIO_PORT_P6, GPIO_PIN4);
    // IN SAMPLING MODE
    if(MODE == SAMPLING_MODE)
    {
        ADC14_toggleConversionTrigger(); // ask ADC to get data
        while(ADC14_isBusy()){};
        INPUT_P6_0[I] = ADC14_getResult(ADC_MEM1);
        Real_INPUT_P6_0[I] = (INPUT_P6_0[I] * 3.3) / 16384;
        INPUT_P6_1[I] = ADC14_getResult(ADC_MEM0);
        Real_INPUT_P6_1[I] = (INPUT_P6_1[I] * 3.3) / 16384;
      // Extract readings from microphone pre amplifier
        if(I == 0)
        {
            I = Size - 1;
            MODE = RUNNING_MODE;
            // Design a software high pass filter
            alpha = 0.88836;
            y_left[0] = 0.0;
            y_right[0] = 0.0;
            for (c = 1; c < 1000; c++) {
                y_left[c] = alpha * y_left[c-1] + alpha * (Real_INPUT_P6_1[c] - Real_INPUT_P6_1[c-1]);
                y_right[c] = alpha * y_right[c-1] + alpha * (Real_INPUT_P6_0[c] - Real_INPUT_P6_0[c-1]);
            }
            float avgMaxRight = 0.0; // P6.0 is the microphone on the RIGHT
            float avgMaxLeft = 0.0;  // P6.1 is the microphone on the LEFT
            float currentMaxRight = 0.0;
            float currentMaxLeft = 0.0;
            int chunk, j, start;
            for(chunk = 0; chunk < 10; chunk++) {
                currentMaxRight = 0.0;
                currentMaxLeft = 0.0;
                start = chunk * 100;
                for(j = start; j < (start + 100); j++) {
                    if(y_right[j] > currentMaxRight) {
                        currentMaxRight = y_right[j];
                    }
                    if(y_left[j] > currentMaxLeft) {
                        currentMaxLeft = y_left[j];
                    }
              }
             avgMaxRight = avgMaxRight + currentMaxRight;
             avgMaxLeft = avgMaxLeft + currentMaxLeft;
             }
            avgMaxRight = avgMaxRight / 10.0;
            avgMaxLeft = (avgMaxLeft / 10.0);
            avgMaxRight = avgMaxRight * 1.2; //previously 0.915, tuning to balance readings between both microphones
             if(avgMaxRight < 0.1 && avgMaxLeft < 0.1) {
                DIRECTION = STOP;
             }
             else {
                 float error = avgMaxLeft - avgMaxRight;
                 float Kp = 25000;
                 int base_speed = 2800;
                 if (error > -0.01 && error < 0.01) {
                   error = 0.0;
                 }
                 int turn_adjustment = (int)(error * Kp);
                 int calc_left = base_speed - turn_adjustment;
                 int calc_right = base_speed + turn_adjustment;
                 if(calc_left < 1000) calc_left = 1000;
                 if(calc_right < 1000) calc_right = 1000;
                 if(calc_left > 14000) calc_left = 14000;
                 if(calc_right > 14000) calc_right = 14000;
                 if (calc_left - calc_right > 4000) {
                     calc_right = calc_left - 4000;
                 } else if (calc_right - calc_left > 4000) {
                     calc_left = calc_right - 4000;
                 }
                 dynamic_left_speed = calc_left;
                 dynamic_right_speed = calc_right;
                 DIRECTION = FORWARD;
             }
        }
        else
        {
            I--;
        }
    }
    // IN RUNNING MODE
    if(MODE == RUNNING_MODE)
    {
        // Read the obstacle status pin from ESP32 (P4.3) from ultrasonic sensors
        uint8_t escapeRequiredRight = GPIO_getInputPinValue(GPIO_PORT_P4, GPIO_PIN3);
        uint8_t escapeRequiredLeft = GPIO_getInputPinValue(GPIO_PORT_P4, GPIO_PIN4);
        // Obstacle avoidance override 
        if (escapeRequiredRight == 1) {
            motor_stop();
            SysTick_Wait10ms(20);
            motor_backward(4500, 4500);
            SysTick_Wait10ms(75);
            motor_left(3000, 3000);
            SysTick_Wait10ms(60);
            sprintf(telemetry_buffer, "ESCAPE RIGHT,%d,%d\n", 3000, 3000);
            Send_To_ESP32(telemetry_buffer);
        }
        else if (escapeRequiredLeft == 1) {
            // Obstacle is on the Right. Escape by turning LEFT.
            motor_stop();
            SysTick_Wait10ms(20);
            motor_backward(4500, 4500);
            SysTick_Wait10ms(75);
            motor_right(3000, 3000); // Tries to turn away from the right
            SysTick_Wait10ms(60);

            sprintf(telemetry_buffer, "ESCAPE LEFT,%d,%d\n", 3000, 3000);
            Send_To_ESP32(telemetry_buffer);
        }
        else {
            if(DIRECTION == STOP)
            {
               motor_stop();
               sprintf(telemetry_buffer, "STOP,0,0\n");
               Send_To_ESP32(telemetry_buffer);
               SysTick_Wait10ms(100);
            }
            else // Engage dynamic seeking algorithm
            {
                motor_forward(dynamic_left_speed, dynamic_right_speed);
                if (dynamic_right_speed > dynamic_left_speed) {
                    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN2);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN1);
                    sprintf(telemetry_buffer, "LEFT TURN,%d,%d\n", dynamic_left_speed, dynamic_right_speed);
                    Send_To_ESP32(telemetry_buffer);
                } else if (dynamic_right_speed < dynamic_left_speed) {
                    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN0);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1 | GPIO_PIN2);
                    sprintf(telemetry_buffer, "RIGHT TURN,%d,%d\n", dynamic_left_speed, dynamic_right_speed);
                    Send_To_ESP32(telemetry_buffer);
                } else {
                    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN2);
                    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1);
                    sprintf(telemetry_buffer, "FORWARD,%d,%d\n", dynamic_left_speed, dynamic_right_speed);
                    Send_To_ESP32(telemetry_buffer);
                }
                sprintf(telemetry_buffer, "[RUN]\n");
                Send_To_ESP32(telemetry_buffer);
                SysTick_Wait10ms(40);
            }
        }
        sprintf(telemetry_buffer, "[SAMP]\n");
        Send_To_ESP32(telemetry_buffer);
        MODE = SAMPLING_MODE;
        motor_stop();
    }
}

void TimerA2_Init(void){
    Timer_A_configureUpMode(TIMER_A2_BASE, &upConfig);
    Interrupt_enableSleepOnIsrExit();
    Interrupt_enableInterrupt(INT_TA2_0);
    Timer_A_startCounter(TIMER_A2_BASE, TIMER_A_UP_MODE);
    Interrupt_setPriority(INT_TA2_0, 0x20);
    Interrupt_enableMaster();
}
void PWM_Init12(void){
        GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2,GPIO_PIN4 + GPIO_PIN5, GPIO_PRIMARY_MODULE_FUNCTION);
        Timer_A_configureUpDownMode(TIMER_A0_BASE, &upDownConfig);
        Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UPDOWN_MODE);
        Timer_A_initCompare(TIMER_A0_BASE, &compareConfig_PWM1);
        Timer_A_initCompare(TIMER_A0_BASE, &compareConfig_PWM1);
}
void PWM_duty1(uint16_t duty1, Timer_A_CompareModeConfig* data)
{
  if(duty1 >= TIMER_PERIOD) return;
  data->compareValue = duty1;
  Timer_A_initCompare(TIMER_A0_BASE, &compareConfig_PWM1);
}
void PWM_duty2(uint16_t duty2, Timer_A_CompareModeConfig* data)
{
  if(duty2 >= TIMER_PERIOD) return;
  data->compareValue = duty2;
  Timer_A_initCompare(TIMER_A0_BASE, &compareConfig_PWM2);
}
void MotorInit(void){
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN6);
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN5|GPIO_PIN7);
}

// Motor Configurations
void motor_forward(uint16_t leftDuty, uint16_t rightDuty){
    GPIO_setOutputLowOnPin( GPIO_PORT_P3, GPIO_PIN5|GPIO_PIN7);
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN6);
    PWM_duty1(rightDuty, &compareConfig_PWM1);
    PWM_duty2(leftDuty,  &compareConfig_PWM2);
}
void motor_right(uint16_t leftDuty, uint16_t rightDuty){
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN6);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN5);
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN7);
    PWM_duty1(rightDuty, &compareConfig_PWM1);
    PWM_duty2(leftDuty,  &compareConfig_PWM2);
}
void motor_left(uint16_t leftDuty, uint16_t rightDuty){
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN6);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN7);
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN5);
    PWM_duty1(rightDuty, &compareConfig_PWM1);
    PWM_duty2(leftDuty,  &compareConfig_PWM2);
}
void motor_backward(uint16_t leftDuty, uint16_t rightDuty){
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN6);
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN5 | GPIO_PIN7);
    PWM_duty1(rightDuty, &compareConfig_PWM1);
    PWM_duty2(leftDuty,  &compareConfig_PWM2);
}
void motor_stop(void){
    PWM_duty1(0, &compareConfig_PWM1);
    PWM_duty2(0, &compareConfig_PWM2);
}
void ADC_Ch14Ch15_Init(void){
    ADC14_enableModule();
    ADC14_initModule(ADC_CLOCKSOURCE_MCLK, ADC_PREDIVIDER_1, ADC_DIVIDER_1,0);
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6,GPIO_PIN0 | GPIO_PIN1, GPIO_TERTIARY_MODULE_FUNCTION);
    ADC14_configureMultiSequenceMode(ADC_MEM0, ADC_MEM1, false);
    ADC14_configureConversionMemory(ADC_MEM0, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A14, false);
    ADC14_configureConversionMemory(ADC_MEM1, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A15, false);
    ADC14_disableInterrupt(ADC_INT1);
    Interrupt_disableInterrupt(INT_ADC14);
    ADC14_enableSampleTimer(ADC_AUTOMATIC_ITERATION);
    ADC14_enableConversion();
}



