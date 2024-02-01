/**
 ******************************************************************************
 * @file           : main.c
 * @author         : team ( ) using the tiva c and the arduino uno 
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2024 .
 * All rights reserved.</center></h2>
 *
 *
 ******************************************************************************
 */

/*================= the included libraries ============================== */
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/uart.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "inc/tm4c123gh6pm.h"
#include "driverlib/pin_map.h"
#include "driverlib/adc.h"


/* =================== the macros we use =========================== */
#define UART_MODULE_BASE UART1_BASE
#define BAUD_RATE 9600
#define RED_LED   0x02
#define BLUE_LED  0x04
#define TIME_INTERVAL_US 10



/*================== the APIs ================================ */
void hc05_init(void);                               // Initialize UART5 module for HC-05
char bluetooth_read(void);                         // Read data from Rx5 pin of TM4C123
void bluetooth_write(unsigned char data);         // Transmit a character to HC-05 over Tx5 pin
void bluetooth_write_string(char *str);          // Transmit a string to HC-05 over Tx5 pin
void Bluetooth_Write_Integer(int value);        // Transmit an integer to HC-05 over Tx5 pin
void initUART();
void portAInit(void);              // Port A initialization
void portDInit(void);             // Port D initialization
void portBInit(void);            // Port B initialization
void portFInit(void);           // Port F initialization
void sysTickInit(void);        // sysTick initialization
void sysTickWait(unsigned long delay);//delay ms using systick
void sysTickWait10us(unsigned long delay);// delay us using systick
void buzzerInit1(void);         // buzzer initialization
void activateBuzzer1(void);    // turn on the buzzer
void deactivateBuzzer1(void); // turn off the buzzer
void buzzerInit2(void);         // buzzer initialization
void activateBuzzer2(void);    // turn on the buzzer
void deactivateBuzzer2(void); // turn off the buzzer
void ddelay(unsigned long counter); // used to add delay
unsigned long measureDistance(void);// measure distance for the ultrasonic sensor
uint32_t readMQ2Sensor(void) ;
void initGPIO(void);
void activateMute(void);
void deactivateMute(void);
void initMute(void);


/*================== the main function ======================= */
int main(void) {
  
    portBInit();
    portFInit();
    portAInit();
    portDInit();
    buzzerInit1();
    buzzerInit2();
    sysTickInit();
    initUART();
    hc05_init();
    initGPIO();
    initMute();
    
    unsigned startButtonState = 0; // Variable to track the state of the start button
    unsigned stopButtonState = 0;  // Variable to track the state of the stop button
    unsigned  distance = measureDistance();
    unsigned sensorValue = readMQ2Sensor();
    
   while (1) {
     bluetooth_write_string("Welcome to the system");
     bluetooth_write_string("\n");
    // Check the state of the push button
     if ( (GPIO_PORTF_DATA_R &0x10) == 0){       
          startButtonState = 1;
          UARTCharPut(UART_MODULE_BASE, 'B');
          bluetooth_write_string("system start");
          bluetooth_write_string("\n");
     }
    else{
       // Start button is not pressed
            startButtonState = 0;
    }
    while(startButtonState)
    {
      sensorValue = readMQ2Sensor();
      
        char str[10]={0};
      int i=0;
      
        distance = measureDistance();
        while(distance ){
          unsigned  y=distance%10;
          str[i]=y+48;
          distance/=10;
          i++;
        }
        bluetooth_write_string("Distance : ");
        while(i){
          
          bluetooth_write( str[i-1]);
          
                  UARTCharPut(UART_MODULE_BASE, str[i-1]);
                  i--;
        }
        i=0;
        bluetooth_write_string("\n");
         if (sensorValue > 1000) {
            // High gas concentration detected. Turn on the RED LED.
                activateBuzzer2();
                UARTCharPut(UART_MODULE_BASE, 'Z');
                bluetooth_write_string(" Fire!!!!!!!!!! \n");
               
            
        } else {
            // Low or no gas concentration detected. Turn on the BLUE LED.
            deactivateBuzzer2();
            UARTCharPut(UART_MODULE_BASE, 'X');
            bluetooth_write_string("No Fire\n");
        }
          if ( (GPIO_PORTD_DATA_R &0x01) == 0 )
    {     
      // do nothing 
      //GPIO_PORTF_DATA_R = 0x04;   // blue LED on
      deactivateBuzzer1();
      UARTCharPut(UART_MODULE_BASE, 'N');
      bluetooth_write_string(" Magnetic not Detected \n");
        
    }
    
   
    else {
      
      // do nothing
     // GPIO_PORTF_DATA_R = 0x04;    // red 
      activateBuzzer1();
      UARTCharPut(UART_MODULE_BASE, 'D');
      bluetooth_write_string(" Magnetic Detected \n ");
      
        if ((GPIO_PORTF_DATA_R |0x01) == 1)
    {
      sysTickWait(500000);
      deactivateBuzzer1();
      UARTCharPut(UART_MODULE_BASE, 'N');
      bluetooth_write_string(" Not Detected \n ");

    }

      
    }
     if ((GPIO_PORTF_DATA_R &0x01) == 0)
    {
        // Stop button is pressed
          //  stopButtonState = 1;
            UARTCharPut(UART_MODULE_BASE, 'S');
            return 0 ;
            exit(0);
    }
         sysTickWait10us(300000);   // delay 0.1 second   
    }
    
   
          
      sysTickWait10us(100000);
    // return 0; // This line is unnecessary in the main function.
    }
   

}





void ddelay(unsigned long counter)
{
  unsigned long i = 0;
  
  for(i=0; i< counter; i++);
}
void hc05_init(void)
{
  
  SYSCTL_RCGCUART_R |= 0x20;  /* enable clock to uart5 */
  SYSCTL_RCGCGPIO_R |= 0x10;  /* enable clock to portE for pe4/rx and re5/tx */
  ddelay(1);
  /* uart0 initialization */
  UART5_CTL_R = 0;         /* uart5 module disbable */
    
  UART5_IBRD_R = 104;      /* for 9600 baud rate, integer = 104 */
  UART5_FBRD_R = 11;       /* for 9600 baud rate, fractional = 11*/
  UART5_CC_R = 0;          /*select system clock*/
  UART5_LCRH_R = 0x60;     /* data lenght 8-bit, not parity bit, no fifo */
  UART5_CTL_R = 0x301;     /* enable uart5 module, rx and tx */
  
  /* uart5 tx5 and rx5 use pe4 and pe5. configure them digital and enable alternate function */
  GPIO_PORTE_DEN_R |= 0x30;      /* set pe4 and pe5 as digital */
  GPIO_PORTE_AFSEL_R |= 0x30;    /* use pe4,pe5 alternate function */
  GPIO_PORTE_AMSEL_R |= 0;    /* turn off analg function*/
  GPIO_PORTE_PCTL_R |= 0x00110000;     /* configure pe4 and pe5 for uart */
  
  
}
char bluetooth_read(void)  
{
  char data;
  while((UART5_FR_R & (1<<4)) != 0); /* wait until rx buffer is not full */
  data =  UART5_DR_R ;  	/* before giving it another byte */
  return (unsigned char) data; 
}
void bluetooth_write(unsigned char data)  
{
  while((UART5_FR_R & (1<<5)) != 0); /* wait until tx buffer not full */
  UART5_DR_R = data;                  /* before giving it another byte */
}


void bluetooth_write_string(char *str)
{
  while(*str)
  {
    bluetooth_write(*(str++));
  }
}

void Bluetooth_Write_Integer(int value) {
    // Convert the integer to a string
    char buffer[20];  // Assuming a reasonable buffer size
    sprintf(buffer, "%d", value); // Convert integer to ASCII string with base 10

    // Transmit the string over Bluetooth
    bluetooth_write_string(buffer);
}
void initUART() {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

    GPIOPinConfigure(GPIO_PC4_U1RX);
    GPIOPinConfigure(GPIO_PC5_U1TX);
    GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    UARTConfigSetExpClk(UART_MODULE_BASE, SysCtlClockGet(), BAUD_RATE,
    (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
}

void portAInit(void) {
    volatile unsigned long delay;
    SYSCTL_RCGC2_R |= 0x00000001;     // 1) activate clock for Port A
    delay = SYSCTL_RCGC2_R;           // allow time for clock to start
    GPIO_PORTA_LOCK_R = 0x4C4F434B;   // 2) unlock GPIO Port A
    GPIO_PORTA_CR_R = 0xFF;           // allow changes to PA7-0
    GPIO_PORTA_AMSEL_R = 0x00;        // 3) disable analog on PA
    GPIO_PORTA_PCTL_R = 0x00000000;   // 4) PCTL GPIO on PA7-0
    GPIO_PORTA_DIR_R = 0xF7;          // 5) PA7-0 out
    GPIO_PORTA_AFSEL_R = 0x00;        // 6) disable alt funct on PA7-0
    GPIO_PORTA_PUR_R = 0x00;          // disable pull-up on all pins
    GPIO_PORTA_DEN_R = 0xFF;          // 7) enable digital I/O on PA7-0
}
void portDInit(void) {
    volatile unsigned long delay;
    SYSCTL_RCGC2_R |= 0x00000008;     // 1) activate clock for Port F
    delay = SYSCTL_RCGC2_R;           // allow time for clock to start
    GPIO_PORTD_LOCK_R = 0x4C4F434B;   // 2) unlock GPIO Port F
    GPIO_PORTD_CR_R = 0x1F;           // allow changes to PF4-0
    // only PF0 needs to be unlocked, other bits can't be locked
    GPIO_PORTD_AMSEL_R = 0x00;        // 3) disable analog on PF
    GPIO_PORTD_PCTL_R = 0x00000000;   // 4) PCTL GPIO on PF4-0
    GPIO_PORTD_DIR_R = 0x0E;          // 5) PF4, PF0 in, PF3-1 out
    GPIO_PORTD_AFSEL_R = 0x00;        // 6) disable alt funct on PF7-0
    GPIO_PORTD_PUR_R = 0x11;          // enable pull-up on PF0 and PF4
    GPIO_PORTD_DEN_R = 0x1F;          // 7) enable digital I/O on PF4-0
}

void portBInit(void) {
    volatile unsigned long delay;
    SYSCTL_RCGC2_R |= 0x00000002;
    delay = SYSCTL_RCGC2_R;
    GPIO_PORTB_AMSEL_R = 0x00;    // disable analog function
    GPIO_PORTB_PCTL_R = 0x00000000;
    GPIO_PORTB_DIR_R = 0x02; // 0010 -> PB0: input/echo, PB1: output/trigger
    GPIO_PORTB_AFSEL_R = 0x00; // no alternate function
    GPIO_PORTB_DEN_R = 0x03; // 0011 -> enable digital pins for PB0, PB1
}

void portFInit(void) {
    volatile unsigned long delay;
    SYSCTL_RCGC2_R |= 0x00000020;     // 1) activate clock for Port F
    delay = SYSCTL_RCGC2_R;           // allow time for clock to start
    GPIO_PORTF_LOCK_R = 0x4C4F434B;   // 2) unlock GPIO Port F
    GPIO_PORTF_CR_R = 0x1F;           // allow changes to PF4-0
    // only PF0 needs to be unlocked, other bits can't be locked
    GPIO_PORTF_AMSEL_R = 0x00;        // 3) disable analog on PF
    GPIO_PORTF_PCTL_R = 0x00000000;   // 4) PCTL GPIO on PF4-0
    GPIO_PORTF_DIR_R = 0x0E;          // 5) PF4, PF0 in, PF3-1 out
    GPIO_PORTF_AFSEL_R = 0x00;        // 6) disable alt funct on PF7-0
    GPIO_PORTF_PUR_R = 0x11;          // enable pull-up on PF0 and PF4
    GPIO_PORTF_DEN_R = 0x1F;          // 7) enable digital I/O on PF4-0
}

void sysTickInit(void) {
    NVIC_ST_CTRL_R = 0;                   // disable SysTick during setup
    NVIC_ST_RELOAD_R = NVIC_ST_RELOAD_M;  // maximum reload value
    NVIC_ST_CURRENT_R = 0;                // any write to current clears it
    NVIC_ST_CTRL_R = NVIC_ST_CTRL_ENABLE + NVIC_ST_CTRL_CLK_SRC; // enable SysTick with core clock
}

void sysTickWait(unsigned long delay) {
    NVIC_ST_RELOAD_R = delay - 1;            // number of counts to wait
    NVIC_ST_CURRENT_R = 0;
    while ((NVIC_ST_CTRL_R & 0x00010000) == 0) {} // wait for count flag
}

void sysTickWait10us(unsigned long delay) {
    unsigned long i;
    for (i = 0; i < delay; i++) {
        sysTickWait(80);
    }
}
void initMute(void) {
    // You need to configure the GPIO pin connected to the buzzer as an output
    // This depends on your hardware connection. For example, if the buzzer is connected to PF2:
    // Configure the GPIO pin connected to the buzzer as an output
    GPIO_PORTA_DIR_R &= ~0x10;  // Clear the bit to set PA4 as input
    GPIO_PORTA_DEN_R |= 0x10;  // Enable digital I/O on PA4
}
void activateMute(void) {
    // You need to set the GPIO pin connected to the buzzer to a high level to activate it
    // This depends on your hardware connection. For example, if the buzzer is connected to PF2:
    GPIO_PORTA_DATA_R &= ~(1<<4);  // Set PA2 high
}
void deactivateMute(void) {
    // You need to set the GPIO pin connected to the buzzer to a high level to activate it
    // This depends on your hardware connection. For example, if the buzzer is connected to PF2:
    GPIO_PORTA_DATA_R |= 0x10;  // Set PA2 high
}

void buzzerInit1(void) {
    // You need to configure the GPIO pin connected to the buzzer as an output
    // This depends on your hardware connection. For example, if the buzzer is connected to PA2:
    GPIO_PORTA_DIR_R |= 0x04;  // Set PA2 as output
    GPIO_PORTA_DEN_R |= 0x04;  // Enable digital I/O on PA2
}

void activateBuzzer1(void) {
    // You need to set the GPIO pin connected to the buzzer to a high level to activate it
    // This depends on your hardware connection. For example, if the buzzer is connected to PF2:
    GPIO_PORTA_DATA_R |= 0x04;  // Set PA2 high
}
void deactivateBuzzer1(void) {
    // You need to set the GPIO pin connected to the buzzer to a high level to activate it
    // This depends on your hardware connection. For example, if the buzzer is connected to PF2:
    GPIO_PORTA_DATA_R &= ~(1<<2);  // Set PA2 high
}
void buzzerInit2(void) {
    // You need to configure the GPIO pin connected to the buzzer as an output
    // This depends on your hardware connection. For example, if the buzzer is connected to PF2:
    GPIO_PORTA_DIR_R |= 0x08;  // Set PF2 as output
    GPIO_PORTA_DEN_R |= 0x08;  // Enable digital I/O on PF2
}

void activateBuzzer2(void) {
    // You need to set the GPIO pin connected to the buzzer to a high level to activate it
    // This depends on your hardware connection. For example, if the buzzer is connected to PF2:
    GPIO_PORTA_DATA_R |= 0x08;  // Set PA3 high
}
void deactivateBuzzer2(void) {
    // You need to set the GPIO pin connected to the buzzer to a high level to activate it
    // This depends on your hardware connection. For example, if the buzzer is connected to PF2:
    GPIO_PORTA_DATA_R &= ~(1<<3);  // Set PA3 low
}
unsigned long measureDistance(void) {
    unsigned long dist = 0;
    unsigned long count = 0;

    GPIO_PORTB_DATA_R &= ~0x02; // resetting trigger/PB1 to 0
    sysTickWait10us(1); // wait 20 us
    GPIO_PORTB_DATA_R |= 0x02; // setting trigger/PB1 to 1
    sysTickWait10us(1); // wait 10 us
    GPIO_PORTB_DATA_R &= ~0x02; // resetting trigger/PB1 to 0

    while ((GPIO_PORTB_DATA_R & 0x01) == 0) {} // busy waiting if echo/PB0 = 0
    while ((GPIO_PORTB_DATA_R & 0x01) == 1) {
        count += TIME_INTERVAL_US;
        sysTickWait10us(1);
    }

    dist = count * (float)340.0 / 20000;  // distance =  time / 2 * speed = time in us * 10e-6 * 340 * 10e2 = time in us * 340/20000

    return dist;
}
void initGPIO(void) {

    // Configure ADC0 sequencer 3 for the MQ-2 sensor on pin AIN0 (PE3).
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);

    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 3);
}
uint32_t readMQ2Sensor(void) {
    uint32_t ADCValues[1];
    ADCIntClear(ADC0_BASE, 3);
    ADCProcessorTrigger(ADC0_BASE, 3);

    while(!ADCIntStatus(ADC0_BASE, 3, false)) {}
    ADCSequenceDataGet(ADC0_BASE, 3, ADCValues);

    return ADCValues[0];
}
