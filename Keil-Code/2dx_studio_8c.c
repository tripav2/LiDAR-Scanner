/*  Time of Flight for 2DX4 -- Studio W8-0
                Code written to support data collection from VL53L1X using the Ultra Light Driver.
                I2C methods written based upon MSP432E4 Reference Manual Chapter 19.
                Specific implementation was based upon format specified in VL53L1X.pdf pg19-21
                Code organized according to en.STSW-IMG009\Example\Src\main.c
                
                The VL53L1X is run with default firmware settings.


            Written by Tom Doyle
            Updated by  Hafez Mousavi Garmaroudi
            Last Update: March 17, 2020
						
						Last Update: March 03, 2022
						Updated by Hafez Mousavi
						__ the dev address can now be written in its original format. 
								Note: the functions  beginTxI2C and  beginRxI2C are modified in vl53l1_platform_2dx4.c file
								
						Modified March 16, 2023 
						by T. Doyle
							- minor modifications made to make compatible with new Keil IDE
						
						Updated by: Viha Tripathi
						Last Update: April 9th, 2025

*/
#include <stdint.h>
#include "PLL.h" //pll configuration
#include "SysTick.h" //delay functionality
#include "uart.h" //uart
#include "onboardLEDs.h" //onboard leds
#include "tm4c1294ncpdt.h" //tiva
#include "VL53L1X_api.h" //api for the tof 


// I2C control bit macros
#define I2C_MCS_ACK             0x00000008
#define I2C_MCS_DATACK          0x00000008
#define I2C_MCS_ADRACK          0x00000004
#define I2C_MCS_STOP            0x00000004
#define I2C_MCS_START           0x00000002
#define I2C_MCS_ERROR           0x00000002
#define I2C_MCS_RUN             0x00000001
#define I2C_MCS_BUSY            0x00000001
#define I2C_MCR_MFE             0x00000010
#define MAXRETRIES              5

uint16_t dev = 0x29; //12c address of the sensor 
int stat = 0; //return status of variable 

//initialize 12c0 for communication with sensor
void I2C_Init(void) {
    SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0; //enabling clock for 12c0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1; // enable clock to port B
    while((SYSCTL_PRGPIO_R & 0x0002) == 0){}; //wating fro port B to be ready 
    GPIO_PORTB_AFSEL_R |= 0x0C; //configures pb2 and pb3 for i2c
    GPIO_PORTB_ODR_R |= 0x08;
    GPIO_PORTB_DEN_R |= 0x0C;
    GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R & 0xFFFF00FF) + 0x00002200; //maps pb2/pb3 to 12c functionaility in port control register 
    I2C0_MCR_R = I2C_MCR_MFE;
    I2C0_MTPR_R = 0x00000027;
}

//initializing port F for led3
void PortF_Init(void) {
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R5) == 0) {}
    GPIO_PORTF_DIR_R |= 0b00010000;     // PF4 output
    GPIO_PORTF_DEN_R |= 0b00010000;
    GPIO_PORTF_PUR_R |= 0b00010000;     // Enable pull-up for PF4 (input if used as button)
}

//intiializing port j for pj0
void PortJ_Init(void) {
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R8) == 0) {}
    GPIO_PORTJ_DIR_R &= ~0x01;      // PJ0 as input
    GPIO_PORTJ_DEN_R |= 0x01;
    GPIO_PORTJ_PUR_R |= 0x01;       // Enable pull-up for PJ0
}

//initializing port h for use of stepper motor 
void PortH_Init(void) {
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R7) == 0) {}
    GPIO_PORTH_DIR_R |= 0x0F;
    GPIO_PORTH_AFSEL_R &= ~0x0F;
    GPIO_PORTH_DEN_R |= 0x0F;
    GPIO_PORTH_AMSEL_R &= ~0x0F;
}

//initializing pg0 for xshut control 
void PortG_Init(void) {
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R6) == 0) {};
    GPIO_PORTG_DIR_R &= 0x00;
    GPIO_PORTG_AFSEL_R &= ~0x01;
    GPIO_PORTG_DEN_R |= 0x01;
    GPIO_PORTG_AMSEL_R &= ~0x01;
}

//initializ port n for pn1
void PortN_Init(void) {
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R12;
    while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R12) == 0) {}
    GPIO_PORTN_DIR_R |= 0x02;   // PN1 output
    GPIO_PORTN_DEN_R |= 0x02;
}

//hardware reset on sensor using xshut pg0
void VL53L1X_XSHUT(void) {
    GPIO_PORTG_DIR_R |= 0x01; //pg0 as the output
    GPIO_PORTG_DATA_R &= ~0x01; //drive Pg0 low to  reset the sensor
    FlashAllLEDs(); //visual effect
    SysTick_Wait10ms(10); //waiting
    GPIO_PORTG_DIR_R &= ~0x01; //release control (input state)
}

//blinks pn1
void FlashMeasurementLED(int count) {
    while(count--) { //takes input count which tells the function how many times to blink the led
        GPIO_PORTN_DATA_R ^= 0x02;  // Toggle PN1
        SysTick_Wait10ms(5); 
        GPIO_PORTN_DATA_R ^= 0x02; //toggles again to return to oringial state 
    }
}

//for additonal functions such as waiting for user to press the onboard button on PJ0:
void FlashAdditionalLED(int count) {
    while(count--) {
        GPIO_PORTF_DATA_R ^= 0x10;  // Toggle PF4
        SysTick_Wait10ms(5);
        GPIO_PORTF_DATA_R ^= 0x10;
    }
}

//rotates stepper mtor one step. clockwise rotation
void spin(void) {
    uint32_t delay = 1;
    GPIO_PORTH_DATA_R = 0b00000011;
    SysTick_Wait10ms(delay);
    GPIO_PORTH_DATA_R = 0b00000110;
    SysTick_Wait10ms(delay);
    GPIO_PORTH_DATA_R = 0b00001100;
    SysTick_Wait10ms(delay);
    GPIO_PORTH_DATA_R = 0b00001001;
    SysTick_Wait10ms(delay);
}

//counter clockwise returns home , avoiding finding fastest way home so the wires do not get tangled with each other 
void returnhome(void) {
    uint32_t delay = 1;
    for(int i = 0; i < 512; i++) {
        GPIO_PORTH_DATA_R = 0b00001001;
        SysTick_Wait10ms(delay);
        GPIO_PORTH_DATA_R = 0b00001100;
        SysTick_Wait10ms(delay);
        GPIO_PORTH_DATA_R = 0b00000110;
        SysTick_Wait10ms(delay);
        GPIO_PORTH_DATA_R = 0b00000011;
        SysTick_Wait10ms(delay);
    }
}

int main(void) {
    uint8_t sensorState = 0, dataReady = 0;
    uint16_t wordData, Distance;
//peripheral intitiazliations
    PLL_Init();  
    SysTick_Init();
    PortF_Init();
    PortH_Init();
    PortG_Init();
    PortJ_Init();
    PortN_Init();
    onboardLEDs_Init();
    I2C_Init();
    UART_Init();

    UART_printf("Deliverable 2: Stepper Scan + ToF");
    SysTick_Wait10ms(100);

    VL53L1X_XSHUT(); //resets sensor
    stat = VL53L1X_GetSensorId(dev, &wordData); //reads sensor id via i2c
    sprintf(printf_buffer, "Sensor ID = 0x%x\r\n", wordData); //prints string
    UART_printf(printf_buffer); //sending string over to uart
    SysTick_Wait10ms(100);

//polls until sesnor reports it is ready:
    while(sensorState == 0) {
        stat = VL53L1X_BootState(dev, &sensorState);
        SysTick_Wait10ms(10);
    }
    FlashAllLEDs();
    UART_printf("Sensor Ready");
    SysTick_Wait10ms(100);

		//initializes sensor and starts continuous distance measurement
    stat = VL53L1X_SensorInit(dev);
    VL53L1X_StartRanging(dev);
		

		//waits for button press
    while(1) {
        UART_printf("Waiting for onboard button press (PJ0) to start scan...\r\n");
        while((GPIO_PORTJ_DATA_R & 0x01) != 0) {
            FlashAdditionalLED(1);  // Additional status on PF4
            SysTick_Wait10ms(10);
        }

				//performingi a 32 angle scan !
        for(int i = 0; i < 32; i++) { //32 scan points
            for(int j = 0; j < 16; j++) { //16 steps between each scan point (should be 512 steps in total, therefore 32*16 = 512
                spin();
            }

						//waits for sensor to complete a new measurement 
            while(dataReady == 0) {
                stat = VL53L1X_CheckForDataReady(dev, &dataReady);
                FlashAdditionalLED(1);  // Additional status on PF4
                VL53L1_WaitMs(dev, 5);
            }
            dataReady = 0;

						//reads distance in mm and clears interrupt to allow next measurement 
            stat = VL53L1X_GetDistance(dev, &Distance);
            VL53L1X_ClearInterrupt(dev);

						//sends distance value over uart to PC
            sprintf(printf_buffer, "%u\r\n", Distance);
            UART_printf(printf_buffer);

						//brief blink afyer each scan point 
            FlashMeasurementLED(1);  // Measurement activity on PN1
            SysTick_Wait10ms(25);
        }

				//returning home and printing completion message 
        returnhome();
        UART_printf("Scan Complete.\r\n");
    }
}
