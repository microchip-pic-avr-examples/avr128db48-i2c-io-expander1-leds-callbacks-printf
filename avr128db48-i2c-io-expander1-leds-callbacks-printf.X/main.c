/*
? [2025] Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms, you may use Microchip 
    software and any derivatives exclusively with Microchip products. 
    You are responsible for complying with 3rd party license terms  
    applicable to your use of 3rd party software (including open source  
    software) that may accompany Microchip software. SOFTWARE IS ?AS IS.? 
    NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS 
    SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT,  
    MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT 
    WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY 
    KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF 
    MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE 
    FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP?S 
    TOTAL LIABILITY ON ALL CLAIMS RELATED TO THE SOFTWARE WILL NOT 
    EXCEED AMOUNT OF FEES, IF ANY, YOU PAID DIRECTLY TO MICROCHIP FOR 
    THIS SOFTWARE.
*/
/**
 * I2C_HOST EXAMPLE Generated Driver File
 * 
 * @file i2c_host example.c
 * 
 * @ingroup i2c_host example
 * 
 * @version I2C_HOST EXAMPLE Example Version 1.0.1
 *
 * @brief Generated file for
 *        Example:           2. I2C IO Expander 1 - LEDs 
 *        Implementation:    Interrupts with callbacks 
 *        Visualization:     Printf   
 *        MCU Device family: AVR
*/
#include "mcc_generated_files/system/system.h"
#include <util/delay.h>

// Note:  MCP23008 - 8-Bit I2C I/O Expander with I2C (& SPI) Serial Interface(s) 
//        Reference to the MCP23008 data sheet:  https://www.microchip.com/DS20001919 
//        MCP23008 register addresses are in Table 1-2 (page 5) of the data sheet
//        The Curiosity Nano Explorer (EV58G97A) has 2 x MCP23008 IO Expanders

#define MCP23008_1_I2C_ADDRESS  ((uint8_t)0x25)
#define MCP23008_REG_ADDR_IODIR     0x00
#define MCP23008_REG_ADDR_GPIO      0x09
#define PINS_DIGITAL_OUTPUT         0x00
#define PINS_DIGITAL_HIGH           0xFF
#define PINS_DIGITAL_LOW            0x00
#define DATALENGTH                  2
#define I2C_ERROR_NONE              0

static uint8_t MCP23008_Write(uint8_t address, uint8_t reg, uint8_t data);
static void to_binary(uint8_t num, char *str);
static void TIMER_Callback_100ms(void);
static void TIMER_Callback_1s(void);

// TODO: Replace TimerX with number of Timer chosen as dependency. 
//       Matches name of const struct TIMER_INTERFACE, from MCC Generated Files > timer > tcXX.c
static const struct TIMER_INTERFACE *Timer = &Timer1; 

// TODO: Go to Header Files/MCC Generated Files/timer/tcXX.h - replace xx in TCxx_CLOCK_FREQ, e.g. TCA1_CLOCK_FREQ. ----------------------
#define MS_TO_TICKS(ms) (((TCA1_CLOCK_FREQ * (ms)) / 1000UL) - 1UL)
#define LED_100_MS (MS_TO_TICKS(100UL))           
#define LED_1000_MS (MS_TO_TICKS(1000UL))

static i2c_host_error_t errorState = I2C_ERROR_NONE;
static uint8_t *data;
static uint8_t *data_length;
static uint8_t pins = 0; 
static uint8_t pins_inverted = 0; 
static char binaryStr[9]; // 8 bits + null terminator
static volatile bool incrementLEDs = false;
static volatile bool updateLEDs = false;

static uint8_t MCP23008_Write(uint8_t address, uint8_t reg, uint8_t data)
{
    size_t txLength = 2;
    uint8_t txBuffer[2] = {0};

    txBuffer[0] = reg;
    txBuffer[1] = data;
    
    I2C_Host.Write(address, txBuffer, txLength);
    while (I2C_Host.IsBusy())
    {
    }
    
    errorState = I2C_Host.ErrorGet();
    return errorState;
}

// Function to convert a number to a binary string
static void to_binary(uint8_t num, char *str) 
{
    for (uint8_t i = 0U; i < 8U; i++) {
        str[7U - i] = ((num & (uint8_t)(1U << (7U - i))) != 0U) ? '1' : '0';
    }
    str[8] = '\0'; // Null-terminate the string
}

static void TIMER_Callback_100ms(void)
{
    IO_LED_Toggle();
    IO_Debug_Toggle(); 
    incrementLEDs = true; 
}

static void TIMER_Callback_1s(void)
{
    IO_LED_Toggle();
    IO_Debug_Toggle(); 
    updateLEDs = true;
}

int main(void)
{
    _delay_ms(200);  // Prevent program running when programming 

    SYSTEM_Initialize();
    Timer->TimeoutCallbackRegister(TIMER_Callback_100ms);
    (int) printf("Example: 2. I2C IO Expander 1 - LEDs, Implementation: Interrupts with callbacks, Visualization: Printf\r\n");
    (int) printf("MCU Device family: AVR \r\n\r\n");
    (int) printf("Note: LEDs assumed to be active LOW \r\n");

    errorState = MCP23008_Write(MCP23008_1_I2C_ADDRESS, MCP23008_REG_ADDR_IODIR, PINS_DIGITAL_OUTPUT);
   
     while(1)
    {
        if(incrementLEDs)
        {
            if (pins < 64U)
            {
                pins_inverted = ~(pins << 2); 
                errorState = MCP23008_Write(MCP23008_1_I2C_ADDRESS, MCP23008_REG_ADDR_GPIO, pins_inverted); 
                
                to_binary(pins_inverted, binaryStr);
                (int) printf("LED state: 0x%02X 0b%s\n", pins_inverted, binaryStr);
                pins++;
                incrementLEDs = false; 
            }
            else
            {
                (int) printf("\r\n");
                pins = (uint8_t) 0xFF;  // Turn off active LOW LEDs
                errorState = MCP23008_Write(MCP23008_1_I2C_ADDRESS, MCP23008_REG_ADDR_GPIO, pins); 
                Timer->Stop();
                Timer->PeriodSet(LED_1000_MS);                      // Change timer period to 1s
                Timer->TimeoutCallbackRegister(TIMER_Callback_1s);  // Change timer callback
                Timer->Start();
                incrementLEDs = false;
            }
        }
        if(updateLEDs)
        {
            errorState = MCP23008_Write(MCP23008_1_I2C_ADDRESS, MCP23008_REG_ADDR_GPIO, pins); 
            
            to_binary(pins, binaryStr);
            (int) printf("LED state: 0x%02X 0b%s\n", pins_inverted, binaryStr);
            pins = ~pins;   // Toggle LEDs  
            updateLEDs = false; 
        } 
    }
}
