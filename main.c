/*
Author: Tuan Nguyen, Kevin Tieu, Ryan Le
The Maze Runner

MODE: PHASE/ENABLE

Pin Connections:
From top down
IN2 EN-----PA7-----PWM
IN1 PH-----PB3-----Forward/Brake
IN2 EN-----PA6-----PWM
IN1 PH-----PB2-----Forward/Brake
MD---------PB6-----Set to 1 (choose Phase/enable mode)

Drive Operation
    INPUT      |    OUTPUT
PHASE | ENABLE |  O1  |  O2
  1   |  PWM   | PWM  |  0 -----Forward/brake at speed PWM %
  0   |  PWM   |  0   | PWM-----Reverse/brake at speed PWM %
  X   |   0    |  0   |  0 -----Brake low (output to GND)

Bluetooth Module (UART1)
TX_O-------PB0
RX_1-------PB1

Distance Sensor
Front Analog---PE3 (ADC0)
Side Analog----PE2 (ADC1)

Light Sensor --- PC5
*/
//XDS Drivers
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/cfg/global.h> //so that the main file can see the variables created in the .cfg file
//-----------------------------------------------------------------------------------------------------------------------------------
//BIOS Drivers
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Swi.h>
//Include Headers
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/pwm.h"
#include "driverlib/adc.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include <driverlib/interrupt.h>
#include <driverlib/timer.h>
#include "driverlib/debug.h"
#include "driverlib/rom.h"
#include <math.h>
#include <time.h>
#include <inc/hw_ints.h>
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "utils/uartstdio.h"
#include "utils/uartstdio.c"
#include "inc/hw_timer.h"
#include <string.h>
//-----------------------------------------------------------------------------------------------------------------------------------
// define some macro
#define PWM_TICKS_IN_PERIOD 3000
#define SETPOINT 1800   // Higher means closer to wall
#define SETPOINT_FRONT 2250
#define P_VAL 0.06
#define I_VAL 0.001
#define D_VAL 0.0001
//-----------------------------------------------------------------------------------------------------------------------------------
float current_distance(char);
uint32_t adc;       // Store  IR sensor reading
void getMsg(void);
// PID Variables
bool motorOn = false;
void PID(void);
int i;
int t;
unsigned char c;
uint32_t pwm_clk;
uint32_t val_load;                  // Store max value to set PMW width
float Setpoint;             // Target distance from right wall to maintain in cm
uint32_t current_front;             // Current front distance of robot in cm
uint32_t current_side;              // Current side distance from right wall in cm
int error_side = 0;
uint32_t adc_side, adc_front;         // adc values stored in this array
uint32_t duty = 80;
uint32_t pwm = 0;
int32_t error;
int32_t y;
int duty1;
float P, I, D;

//ping pong
int32_t ping[20];            //two buffers to send data with 20 values at a time, every 2 seconds
int32_t pong[20];
bool Data_collect = false;
bool ping_sent = false;
bool pong_sent = false;
bool buffer_overwrite = false;
uint32_t countPing = 0;
uint32_t countPong = 0;

//Light Sensor Variables
int thinbl=0;
bool finishline = false;
int counter = 0;
//-----------------------------------------------------------------------------------------------------------------------------------
void Hardware_Init(void);
void offLED(void);
void redLED(void);
void blueLED(void);
void greenLED(void);
void whiteLED(void);
void rgb_switch(float);
void goForward(void);
void goReverse(void);
void stop(void);
void Turn(char);
void adc_read_side();
void adc_read_front();
void LightSensor();
void send_buffer();
//-----------------------------------------------------------------------------------------------------------------------------------
// Lookup Table struct
typedef struct {
    char* cmd;
    void (*function)(void);
} lookup;
lookup directory[] = {
 {"rl", redLED},
 {"bl", blueLED},
 {"gl", greenLED},
 {"wl", whiteLED},
 {"of", offLED},
 {"go", goForward},
 {"re", goReverse},
 {"so", stop},
};

uint8_t directoryCount = 8;

//-----------------------------------------------------------------------------------------------------------------------------------

int main(){
       Hardware_Init();            // Initializion for Robot
       BIOS_start();
}

void Hardware_Init(void){
    uint32_t ui32Period;
//-----Configure System Clock-----
    // Set CPU Clock to 40MHz. 400MHz PLL/2 = 200 DIV 5 = 40MHz
    SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);

//-----LED Intialization-----
     SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); // Enabling Port F  for LED
     GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);

//-----Bluetooth Initialization
      SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);       // Enabling Port E for UART
      SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);       // Enable UART1 for Bluetooth
      GPIOPinConfigure(GPIO_PB0_U1RX);                   // Connect with TX pin on Bluetooth module
      GPIOPinConfigure(GPIO_PB1_U1TX);                   // Connect with RX pin on Bluetooth module
      GPIOPinTypeUART(GPIO_PORTB_BASE,GPIO_PIN_0|GPIO_PIN_1);
      // Configures the UART by setting the word length to 8 bits, no parity bit, and one stop bit
      UARTConfigSetExpClk(UART1_BASE,SysCtlClockGet(),9600,(UART_CONFIG_WLEN_8 | UART_CONFIG_PAR_NONE | UART_CONFIG_STOP_ONE));
      UARTEnable(UART1_BASE);
      UARTStdioConfig(1, 9600,  SysCtlClockGet());
      IntEnable(INT_UART1);   //enable interrupt capability for UART1
      UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);   // enable interrupt for TX and RX

//-----Front Distance Sensor Initialization------
      SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
      SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);       // Enable ADC0 module
      GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);      // PE3 for front sensor
      // Configure sample sequencer
      ADCSequenceDisable(ADC0_BASE, 3);
      ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
      ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);
      ADCSequenceEnable(ADC0_BASE, 3);

//-----Side Distance Sensor Initialization------
     SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);        // Enable ADC1 module
     GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2);       // PE2 for side sensor
     // Configure sample sequencer
     ADCSequenceDisable(ADC1_BASE, 3);
     ADCSequenceConfigure(ADC1_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
     ADCSequenceStepConfigure(ADC1_BASE, 3, 0, ADC_CTL_CH1 | ADC_CTL_IE | ADC_CTL_END);
     ADCSequenceEnable(ADC1_BASE, 3);

//-----Motor Driver Initialization-----
     SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);                                       // Enable port B
     GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_6);      // Enable output pin PB2, PB3, PB6
     GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_6, 0xFF);                                   // Set MD to 1 to choose PHASE/Enable mode

//-----PMW Initialization-----
     SysCtlPWMClockSet(SYSCTL_PWMDIV_64);                        // Set the clock for PWM
     SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);                 // Enable the PWM1 peripheral
     SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);                // Enable port A
     GPIOPinTypePWM(GPIO_PORTA_BASE, GPIO_PIN_6);                //Configure pin PA6 for use by PWM1
     GPIOPinTypePWM(GPIO_PORTA_BASE, GPIO_PIN_7);                //Configure pin PA7 for use by PWM1
     GPIOPinConfigure(GPIO_PA6_M1PWM2);                          //Select Alternate function for PA6 -- Motion Control Module 1 PWM2
     GPIOPinConfigure(GPIO_PA7_M1PWM3);                          //Select Alternate Function for PA7 -- Motion Control Module 1 PWM3
     PWMGenConfigure(PWM1_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);   //Configure PWM generator 1 to counting down mode
     // Setting pulse width
     pwm_clk = SysCtlClockGet()/64;
     val_load = (pwm_clk/100);  // Max value is 6250
     PWMGenConfigure(PWM1_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN);
     PWMGenPeriodSet(PWM1_BASE, PWM_GEN_1,val_load);                // Set maximum period for PMW
     PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, duty*val_load/100); //setting duty cycle to whatever the value of "duty" variable is
     PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, duty*val_load/100); //setting duty cycle to whatever the value of "duty" variable is
     PWMGenEnable(PWM1_BASE, PWM_GEN_1);

     // Timer 2 setup code
     SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);          // enable Timer 2 periph clks
     TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC);       // cfg Timer 2 mode - periodic
     ui32Period = (SysCtlClockGet() /20);                   // period = CPU clk div 2 (500ms)
     TimerLoadSet(TIMER2_BASE, TIMER_A, ui32Period);        // set Timer 2 period
     TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);       // enables Timer 2 to interrupt CPU
     TimerEnable(TIMER2_BASE, TIMER_A); //enable timer 2

     //Timer 0 and 1 initialization
     SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0); // Enable Timer0 Peripheral
     TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC_UP);    //configuring Timer0 to be periodic
     TimerEnable(TIMER0_BASE, TIMER_A); //Enable timer 0

     //GPIO
     SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC); //enabling GPIO

     UARTprintf("Hardware initialized\n"); //notify on terminal that hardware initialized
}
//-----------------------------------------------------------------------------------------------------------------------------------
// Timer interrupt every 50ms
void hwi_timer(void){
    TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT); //clears interrupt
    if (motorOn) {
        Swi_post(swi_PID);
    }
}
// Bluetooth UART interrupt
void hwi_Bluetooth(void) {
    unsigned long ulStatus;
    ulStatus = UARTIntStatus(UART1_BASE, true); //get interrupt status
    UARTIntClear(UART1_BASE, ulStatus); //clear the asserted interrupts
    Semaphore_post(semaphore0);
}

void swi_Turning(void) {
    while(adc_front > 750) //while front sensor is close to wall
    {
        Turn('l'); //turn left
        adc_read_side(); // puts value into adc_side to continuously monitor reading
        adc_read_front(); // puts value into adc_front to continuously monitor reading
    }
}

void RunTask(void) {
    while (1) {
        Semaphore_pend(semaphore0, BIOS_WAIT_FOREVER);
        getMsg();
    }
}

// get bluetooth Message
void getMsg(void) {
    char command[2];
    if (!UARTCharsAvail(UART1_BASE)) {
        return;
    }
    // Get 2 character command
    command[0] = UARTCharGet(UART1_BASE);
    command[1] = UARTCharGet(UART1_BASE);
    uint32_t i;
    for (i = 0; i < directoryCount; ++i) {
        if (command[0] == directory[i].cmd[0] && command[1] == directory[i].cmd[1]) {
            UARTprintf(directory[i].cmd);
            UARTprintf("\n");
            directory[i].function();
            return;
        }
    }
}

float current_distance(char c){ // char arugment 'f' to read using front sensor or 's' for side sensor
    if (c == 'f'){ // Front distance sensor
        ADCIntClear(ADC0_BASE, 3);                  // clear ADC interrupt
        ADCProcessorTrigger(ADC0_BASE, 3);          // trigger ADC sampling
        ADCSequenceDataGet(ADC0_BASE, 3, &adc);     // read voltage
    }
    else if(c == 's'){ // Side distance sensor
        ADCIntClear(ADC1_BASE, 3);                  // clear ADC interrupt
        ADCProcessorTrigger(ADC1_BASE, 3);          // trigger ADC sampling
        ADCSequenceDataGet(ADC1_BASE, 3, &adc);     // read voltage
    }
    float tempV;
    if(adc > 100){                          // Ensure the reading is operating within the right range
        tempV = 3.3*adc/4095;               // Convert sensor reading to voltage
    }
    else{
        tempV = 3.3;    //set voltage to 3.3
    }
    float distance = 13*pow(tempV, -1.1);   // Convert voltage to distance
    return distance;    // Return distance in cm
}
//-----------------------------------------------------------------------------------------------------------------------------------
void rgb_switch(float distance){
    if (distance <= 4 ){ // less than 4 cm critical - red
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 2);
    }
    else if (distance > 4 && distance <= 7){ // 4-7 cm warning - yellow
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 10);
    }
    else { // further than 7 cm away, green
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 8);
    }
}

void adc_read_side(void)
{
    // clear ADC interrupt
    ADCIntClear(ADC1_BASE, 3);
    // trigger ADC sampling
    ADCProcessorTrigger(ADC1_BASE, 3);
    // read adc
    ADCSequenceDataGet(ADC1_BASE, 3, &adc_side);
}

void adc_read_front(void)
{
    // clear ADC interrupt
    ADCIntClear(ADC0_BASE, 3);
    // trigger ADC sampling
    ADCProcessorTrigger(ADC0_BASE, 3);
    // read adc
    ADCSequenceDataGet(ADC0_BASE, 3, &adc_front);
}
//-----------------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------------
void goForward(void) { //move forward
    motorOn = true;
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3 | GPIO_PIN_2, 0);
    PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT, true);
    PWMOutputState(PWM1_BASE, PWM_OUT_3_BIT, true);
    UARTprintf("\nFull Speed Forward\n");
}
//-----------------------------------------------------------------------------------------------------------------------------------
void goReverse(void) { //reverse
    motorOn = true;
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3 | GPIO_PIN_2, 15);
    PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT, true);
    PWMOutputState(PWM1_BASE, PWM_OUT_3_BIT, true);
    UARTprintf("\nFull Speed Reverse\n");
}
//-----------------------------------------------------------------------------------------------------------------------------------
void stop(void) { //stop motor movement
    motorOn = false;
    PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT, false);
    PWMOutputState(PWM1_BASE, PWM_OUT_3_BIT, false);
    UARTprintf("\nMotor Off\r\n");
}
//-----------------------------------------------------------------------------------------------------------------------------------
void Turn(char c){ // char 'l' for turning left and 'r' for turning right
    if(c == 'l' ){ // Turn left
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, 15);  // Reverse left wheel
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, 0);   // Forward right wheel
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, val_load);
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, val_load);
        PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT | PWM_OUT_3_BIT, true);
    }
    else if(c == 'r'){ // Turn right
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, 0);   // Forward left wheel
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, 15);  // Reverse right wheel
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, val_load);
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, val_load);
        PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT | PWM_OUT_3_BIT, true);
    }

}

//-----------------------------------------------------------------------------------------------------------------------------------
// LED control
void offLED(void) {
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0);
    UARTprintf("Tiva LED = off\n");
};
void redLED(void) {
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 2);
    UARTprintf("Tiva LED = red\n");
};
void blueLED(void) {
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 4);
    UARTprintf("Tiva LED = blue\n");
};
void greenLED(void) {
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 8);
    UARTprintf("Tiva LED = green\n");
};
void whiteLED(void) {
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 14);
    UARTprintf("Tiva LED = white\n");
};

//-----------------------------------------------------------------------------------------------------------------------------------

void PID(){
    Swi_post(swi_LightSensor);
    int32_t errorPrev = 0;
    int32_t Sum = 0;
    int32_t dif;
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3 | GPIO_PIN_2, 0); //Ensure all LEDs are off initially
    adc_read_side(); // puts value into adc_side
    adc_read_front(); // puts value into adc_front
    error_side = (SETPOINT - adc_side); //computes error of side distance sensor

    current_side = current_distance('s'); //gives current side distance in cm
    Setpoint=13*pow(3.3*1800/4095, -1.1); //convert setpoint of side sensor from adc to cm
    error = Setpoint- current_side; //prints error in cm

    P = P_VAL * error_side;

    Sum += error_side;
    I = I_VAL * Sum;

    dif = error_side - errorPrev; //error difference
    errorPrev = error_side; //set current error to compare with new error
    D = D_VAL * dif;

    pwm = (uint32_t)abs(P + I + D);

    duty1 = (duty - pwm);

    if(duty1 < 10){ //set duty to 50 if less than 10
        duty1 = 50;
    }


    if(error_side > 0) //if moving farther from wall
    {
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, duty1*val_load/100 ); // right wheel slow down
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, duty*val_load/100); // left wheel max speed
    }

    else if(error_side < 0) // if moving closer to wall
    {
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, duty*val_load/100); //right wheel max speed
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, duty1*val_load/100); // left wheel slow down
    }

    if(adc_front >= SETPOINT_FRONT) //if adc value of front sensor greater than setpoint
    {
        Swi_post(swi_Turn);
    }

    if (Data_collect==true && (i % 2 == 0)){ //data collection every two iterations of PID
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2); // turn on blue led to show data collection
        if(countPing < 20 && !ping_sent) {  //collect data if ping buffer isn't full and not sent
            pong[countPing] = (error); //store error values into ping buffer
            countPing++;
        }
        else if(countPong < 20 && !pong_sent) {//collect data if pong buffer isn't full and not sent
            ping[countPong] = (error); //store error values into pong buffer
            countPong++;
        }
    }

    if (countPing == 20 || countPong == 20) { //ping buffer is full
       Swi_post(swi_PingPongBuffer); //transmit data to bluetooth terminal
    }

    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0x00); // turn off green LED to ensure only data is being collected
    i=i+1; //counter to signify every 2nd PID iteration for data collection
}

void LightSensor() {

     uint32_t timer1, timer2, value;
     HWREG(TIMER0_BASE + TIMER_O_TAV) = 0;
     GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_5); //enable output pin C5
     GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, GPIO_PIN_5); //write into pin C5
     timer1= TimerValueGet(TIMER0_BASE, TIMER_A);

     GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_PIN_5);
     value = GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_5);  //read port C5 and store into variable "value"

     while(value & GPIO_PIN_5){
        value = GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_5); //pin C5 readings values while active
     }
     timer2 = TimerValueGet(TIMER0_BASE, TIMER_A); //gets second recording of timer

     y=timer2-timer1; //used for watcher during debug to observe time difference for testing

     if((timer2-timer1) > 20000) //First line detected so begin recording data
     {
         UARTprintf("Sensor 2 is reading BLACK, ticks elapsed = %u\n",timer2-timer1);

         Data_collect = true;
         counter++;
     }

     else {
         if(counter > 0 && counter < 4) {//Counter to determine width of black line
         Data_collect = true;
         thinbl++;
     }

     if(counter >= 4)
     {
         finishline = true;
     }
     counter=0;
     }

     if(thinbl == 2 && Data_collect) {
         UARTprintf("Second Line\n"); //second line detected
         Data_collect = false;  //stop collecting data
         buffer_overwrite = true; //overwrite algorithm to print the remaining data to terminal
         send_buffer(); //send data to print to terminal
     }

     else if(finishline){
         UARTprintf("Finish Line Reached\n"); //print to terminal indicating finish line has been reached
         UARTprintf("Time Elapsed: %d seconds \n", (i*50)/1000); //print total time elapsed from start to finish line in seconds
         stop(); //stop motor movement
     }


}
void send_buffer(){
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0x00); // turn off blue led
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3); // turn on green LED to show transmission

    int p;
    if ((countPing == 20 && !ping_sent) || (countPing > 0 && buffer_overwrite)) //print ping buffer onto terminal
    {
        // "MODBUS"
        //UARTprintf("Ping print\n");
        UARTprintf(":17");    // Colon and team number

        for (p=0; p<countPing;p++)
        {
            UARTprintf("%X",abs(ping[p])); // ASCII hex
            ping[p] = 0; // Clear Buffer
        }

        ping_sent = true;
        pong_sent = false;
        countPing = 0; //reset ping array to get new data for ping buffer
    }
    else if ((countPong == 20 && !pong_sent) || (countPong > 0 && buffer_overwrite)) //print pong buffer onto terminal
    {
        // "MODBUS"
        //UARTprintf("Pong print\n");
        UARTprintf(":17");    // Colon

        for (p=0; p<countPong;p++) {
            UARTprintf("%X",abs(pong[p])); //ASCII hex
            pong[p] = 0; // Clear Buffer
        }

         pong_sent = true;
         ping_sent = false;
         countPong = 0; //reset pong array to get new data for ping buffer
    }
    UARTprintf("17\r\n");    // CR and LF
}

