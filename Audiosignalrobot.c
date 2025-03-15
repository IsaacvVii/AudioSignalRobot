/* DriverLib Includes */
#include "driverlib.h"
/* Standard Includes */
#include <stdint.h>
#include "../inc/SysTick.h"
#include "../inc/Clock.h"


/////////////You will define variables here//////////////////

uint32_t Size;
uint32_t I;
uint16_t c;
uint16_t cc;
float avgmax1;
float avgmax2;
float avgmax_combined;
float avgmax1_old;
float avgmax2_old;
float offset;
float offset_old;

int32_t INPUT_P6_1[1024];
float Real_INPUT_P6_1[1024];
float max1[10]= {0};  // find 10 local maximums in the array of Real_Input
float max2[10]= {0};  // find 10 local maximums in the array of Real_Input

int32_t INPUT_P6_0[1024];
float Real_INPUT_P6_0[1024];

float x1[1024];
float y1[1024];

float x2[1024];
float y2[1024];

float z1[1024];
float z2[1024];

float alpha;
float alpha2;

uint8_t DIRECTION;

#define FORWARD    1
#define BACKWARD   2
#define LEFT       3
#define RIGHT      4
#define STOP       5
#define ROTATE_180 6
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
    TIMER_A_CLOCKSOURCE_SMCLK,              // SMCLK Clock SOurce
    TIMER_A_CLOCKSOURCE_DIVIDER_4,          // SMCLK/1 = 1.5MHz
    TIMER_PERIOD,                           // 15000 period
    TIMER_A_TAIE_INTERRUPT_DISABLE,         // Disable Timer interrupt
    TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE,    // Disable CCR0 interrupt
    TIMER_A_DO_CLEAR                        // Clear value

};

/* Timer_A Compare Configuration Parameter  (PWM3) */
Timer_A_CompareModeConfig compareConfig_PWM1 =
{
    TIMER_A_CAPTURECOMPARE_REGISTER_1,          // Use CCR3
    TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE,   // Disable CCR interrupt
    TIMER_A_OUTPUTMODE_TOGGLE_RESET,            // Toggle output but
    DUTY_CYCLE1
};

/* Timer_A Compare Configuration Parameter (PWM4) */
Timer_A_CompareModeConfig compareConfig_PWM2 =
{
    TIMER_A_CAPTURECOMPARE_REGISTER_2,          // Use CCR4
    TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE,   // Disable CCR interrupt
    TIMER_A_OUTPUTMODE_TOGGLE_RESET,            // Toggle output but
    DUTY_CYCLE2
};

/////////////////////////////////////////////////////////////


void TimerA2_Init(void);
void PWM_Init12(void);
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


//////////////////////// MAIN FUNCTION /////////////////////////////////////

int main(void)
{
    Size=1000;
    I=Size-1;

    // Set Microcontroller Clock = 48 MHz
    Clock_Init48MHz();

    PWM_Init12();

    // Systick Configuration
    SysTick_Init();

    // Motor Configuration
    MotorInit();

    /* Sleeping when not in use */

    // Port 5 Configuration: make P6.4 out
    GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN4);

    // Setup ADC for Channel A14 and A15
    ADC_Ch14Ch15_Init();

    // Timer A2 Configuration
    TimerA2_Init();

    DIRECTION  = FORWARD;
    MODE  = SAMPLING_MODE;

    while (1)
    {
    }

}


//////////////////////// FUNCTIONs /////////////////////////////////////


void TA2_0_IRQHandler(void)
{
    GPIO_toggleOutputOnPin(GPIO_PORT_P6, GPIO_PIN4);

    // IN SAMPLING MODE
    if(MODE == SAMPLING_MODE)
    {

        ADC14_toggleConversionTrigger(); // ask ADC to get data

        while(ADC14_isBusy()){};  //Waits for ADC to finish

        INPUT_P6_1[I] = ADC14_getResult(ADC_MEM1);
        Real_INPUT_P6_1[I] = (INPUT_P6_1[I] * 3.3) / 16384;         //left mic
        INPUT_P6_0[I] = ADC14_getResult(ADC_MEM0);
        Real_INPUT_P6_0[I] = (INPUT_P6_0[I] * 3.3) / 16384;         //right mic


        if(I == 0)
        {
            I = Size-1;
            MODE= RUNNING_MODE;


//////////// MAKE DIRECTION DECISION BASED SAMPLING RESULTS HERE ///////////////////////

//// STEP 1:   Design digital filters for ADC data://////

            // Fiter for Real_INPUT_P6_0
            for (c = 0; c < 1000; c++){
                x1[c] = Real_INPUT_P6_1[c];                         //left mic
                x2[c] = Real_INPUT_P6_0[c];                         //right mic
            }

            alpha = 0.888364789987;     //low pass filter value
            alpha2 = 0.653374210035;    //high pass filter value

            y1[0] = x1[0];  //low pass filter initialization step
            y2[0] = x2[0];

            for (c = 1; c < 1000; c++){
                y1[c] = alpha * y1[c-1] + alpha * (x1[c] - x1[c-1]);            // y1 is Real_INPUT_P6_1 after a low pass filter
                y2[c] = alpha * y2[c-1] + alpha * (x2[c] - x2[c-1]);            // y2 is Real_INPUT_P6_0 after a low pass filter
            }

            z1[0] = y1[0];
            z2[0] = y2[0];

            for (c = 1; c < 1000; c++){
                z1[c] = alpha2 * y1[c] + (1 - alpha2) * z1[c-1];                // z1 is y1 after a high pass filter
                z2[c] = alpha2 * y2[c] + (1 - alpha2) * z2[c-1];                // z2 is y2 after a high pass filter
            }

//// STEP 2:   Calculate average maximum values of Real_INPUT_P6_0 and Real_INPUT_P6_1://////


            avgmax1_old = avgmax1;          //copying previous avgmax values
            avgmax2_old = avgmax2;

            avgmax1 = 0;
            avgmax2 = 0;
            avgmax_combined = 0;


            for (c = 0; c < 10; c++)                     //calculating avgmax values from 10 max values
            {
                max1[c]=0;
                max2[c]=0;
                for (cc = 100*c; cc < 100 + 100*c; cc++) {
                    if (z1[cc] > max1[c])
                    {
                        max1[c]  = z1[cc];
                    }
                    if (z2[cc] > max2[c])
                    {
                        max2[c]  = z2[cc];
                    }
                }
                avgmax1 += max1[c];
                avgmax2 += max2[c];
            }


            avgmax1 = avgmax1 / 10;                             //avgmax1 is left microphone
            avgmax2 = avgmax2 / 10;                             //avgmax2 is right microphone
            avgmax_combined = (avgmax1 + avgmax2) / 2;          //avgmax_combined is both microphones






//// STEP 3:   Robot Direction Controlling Codes. Determine robot direction based on left and right microphone data//////


            if(avgmax_combined > 2.4){                             //sound detected

                if(avgmax1 - avgmax2 >= 0.45){                                                                                  //more sound to the left
                    DIRECTION = LEFT;
                }else if(avgmax2 - avgmax1 >= 0.175){                                                                           //more sound to the right
                    DIRECTION = RIGHT;
                } else if((DIRECTION == FORWARD) && ((avgmax1_old > (avgmax1+0.38)) && (avgmax2_old > (avgmax2+0.18))) ){       //sound is further away
                    DIRECTION = ROTATE_180;
                }else{                                                                                                          //sound is straight forward
                    DIRECTION = FORWARD;
                }

            }else{                                                  //no sound
                DIRECTION = STOP;
            }


////////////////////////////////////////////////////////////////////////////////////

        }

        else
        {
            I--;
        }
    }


    // IN RUNNING MODE
    if(MODE == RUNNING_MODE)
    {
        uint16_t turn_speed = 5000;
        uint16_t turn_speed_slow = 2000;
        if(DIRECTION  == FORWARD)
        {
            motor_forward(turn_speed,turn_speed); // Move forward
            SysTick_Wait10ms(100); // Wait 1s for the motor to run
        }

        else if (DIRECTION  == BACKWARD)
        {
            motor_backward(turn_speed,turn_speed); // Move backward
            SysTick_Wait10ms(100); // Wait 1s for the motor to run
        }
        else if (DIRECTION  == LEFT)
        {
            motor_forward(turn_speed_slow,turn_speed); // Move forward left
            SysTick_Wait10ms(50); // Wait 0.5s for the motor to run
        }
        else if (DIRECTION  == RIGHT)
        {
            motor_forward(turn_speed,turn_speed_slow); // Move forward right
            SysTick_Wait10ms(50); // Wait 0.5s for the motor to run
        }
        else if (DIRECTION  == STOP)
        {
            motor_stop(); // Move forward right
            SysTick_Wait10ms(100); // Wait 1s for the motor to run
        }

        else if (DIRECTION  == ROTATE_180)
        {
            motor_right(turn_speed * 1.5, turn_speed * 1.5); // turn right 180degrees
            SysTick_Wait10ms(100); // Wait 1s for the motor to run
        }

        MODE = SAMPLING_MODE;
        motor_stop();
        SysTick_Wait10ms(100);  // Stop 1s to take audio sample without Robot motor noises
    }


    Timer_A_clearCaptureCompareInterrupt(TIMER_A2_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_0);
}


////////////////////////////////////////////////////////////////////////////////////


void TimerA2_Init(void){
    /* Configuring Timer_A1 for Up Mode */
    Timer_A_configureUpMode(TIMER_A2_BASE, &upConfig);

    /* Enabling interrupts and starting the timer */
    Interrupt_enableSleepOnIsrExit();
    Interrupt_enableInterrupt(INT_TA2_0);
    Timer_A_startCounter(TIMER_A2_BASE, TIMER_A_UP_MODE);
    /* Enabling MASTER interrupts */
    Interrupt_setPriority(INT_TA2_0, 0x20);
    Interrupt_enableMaster();

}


void PWM_Init12(void){

    /* Setting P2.4 and P2.5 and peripheral outputs for CCR */
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2,GPIO_PIN4 + GPIO_PIN5, GPIO_PRIMARY_MODULE_FUNCTION);

    /* Configuring Timer_A1 for UpDown Mode and starting */
    Timer_A_configureUpDownMode(TIMER_A0_BASE, &upDownConfig);
    Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UPDOWN_MODE);

    /* Initialize compare registers to generate PWM1 */
    Timer_A_initCompare(TIMER_A0_BASE, &compareConfig_PWM1);

    /* Initialize compare registers to generate PWM2 */
    Timer_A_initCompare(TIMER_A0_BASE, &compareConfig_PWM2);
}


void PWM_duty1(uint16_t duty1, Timer_A_CompareModeConfig* data)  // function definition
{
    if(duty1 >= TIMER_PERIOD) return; // bad input
    data->compareValue = duty1; // access a struct member through a pointer using the -> operator
    /* Initialize compare registers to generate PWM1 */
    Timer_A_initCompare(TIMER_A0_BASE, &compareConfig_PWM1);
}

void PWM_duty2(uint16_t duty2, Timer_A_CompareModeConfig* data)  // function definition
{
    if(duty2 >= TIMER_PERIOD) return; // bad input
    data->compareValue = duty2; // access a struct member through a pointer using the -> operator
    /* Initialize compare registers to generate PWM2 */
    Timer_A_initCompare(TIMER_A0_BASE, &compareConfig_PWM2);
}


void MotorInit(void){

    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN6); // choose P3.0 and P3.6 as outputs

    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN5|GPIO_PIN7); // choose P3.5 and P3.7 as outputs

}


void motor_forward(uint16_t leftDuty, uint16_t rightDuty){

    GPIO_setOutputLowOnPin( GPIO_PORT_P3, GPIO_PIN5|GPIO_PIN7); // choose P3.5 and P3.7 Low
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN6); // choose P3.0 and P3.6 High
    PWM_duty1(rightDuty, &compareConfig_PWM1);
    PWM_duty2(leftDuty,  &compareConfig_PWM2);

}



// ------------Motor_Right------------
// Turn the robot to the right by running the
// left wheel forward and the right wheel
// backward with the given duty cycles.
// Input: leftDuty  duty cycle of left wheel (0 to 14,998)
//        rightDuty duty cycle of right wheel (0 to 14,998)
// Output: none
// Assumes: Motor_Init() has been called
void motor_right(uint16_t leftDuty, uint16_t rightDuty){

    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN7); // choose P3.7 Low
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN6|GPIO_PIN5); // choose P3.0, P3.6, P3.5 High
    PWM_duty1(rightDuty, &compareConfig_PWM1);
    PWM_duty2(leftDuty,  &compareConfig_PWM2);

}

// ------------Motor_Left------------
// Turn the robot to the left by running the
// left wheel backward and the right wheel
// forward with the given duty cycles.
// Input: leftDuty  duty cycle of left wheel (0 to 14,998)
//        rightDuty duty cycle of right wheel (0 to 14,998)
// Output: none
// Assumes: Motor_Init() has been called
void motor_left(uint16_t leftDuty, uint16_t rightDuty){

    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN5); // choose P3.5 Low
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN6|GPIO_PIN7); // choose P3.0, P3.6, P3.7 High
    PWM_duty1(rightDuty, &compareConfig_PWM1);
    PWM_duty2(leftDuty,  &compareConfig_PWM2);

}

// ------------Motor_Backward------------
// Drive the robot backward by running left and
// right wheels backward with the given duty
// cycles.
// Input: leftDuty  duty cycle of left wheel (0 to 14,998)
//        rightDuty duty cycle of right wheel (0 to 14,998)
// Output: none
// Assumes: Motor_Init() has been called
void motor_backward(uint16_t leftDuty, uint16_t rightDuty){
    // write this as part of Lab 6
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN5|GPIO_PIN7); // choose P3.5 and P3.7 High
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN6); // choose P3.0 and P3.6 High
    PWM_duty1(rightDuty, &compareConfig_PWM1);
    PWM_duty2(leftDuty,  &compareConfig_PWM2);
}


void motor_stop(void){

    PWM_duty1(0, &compareConfig_PWM1);
    PWM_duty2(0, &compareConfig_PWM2);

}


void ADC_Ch14Ch15_Init(void){

    /* Initializing ADC (MCLK/1/1) */
    ADC14_enableModule();
    ADC14_initModule(ADC_CLOCKSOURCE_MCLK, ADC_PREDIVIDER_1, ADC_DIVIDER_1,0);

    /* Configuring GPIOs for Analog In */

    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6,GPIO_PIN0 | GPIO_PIN1, GPIO_TERTIARY_MODULE_FUNCTION);


    /* Configuring ADC Memory (ADC_MEM0 - ADC_MEM1 (A14 - A15) */
    ADC14_configureMultiSequenceMode(ADC_MEM0, ADC_MEM1, false);
    ADC14_configureConversionMemory(ADC_MEM0, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A14, false);
    ADC14_configureConversionMemory(ADC_MEM1, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A15, false);

    /* Enabling the interrupt when a conversion on channel 1 (end of sequence)
    *  is complete and enabling conversions */
    ADC14_disableInterrupt(ADC_INT1);

    /* Enabling Interrupts */
    Interrupt_disableInterrupt(INT_ADC14);
    //  Interrupt_enableMaster();

    /* Setting up the sample timer to automatically step through the sequence
    * convert.
    */
    ADC14_enableSampleTimer(ADC_AUTOMATIC_ITERATION);

    /* Triggering the start of the sample */
    ADC14_enableConversion();

}

///////////////////////////////////////END/////////////////////////////////////////////
