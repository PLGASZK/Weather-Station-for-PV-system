/**
  System Interrupts Generated Driver File

  @Company:
    Microchip Technology Inc.

  @File Name:
    pin_manager.h

  @Summary:
    This is the generated manager file for the MPLAB(c) Code Configurator device.  This manager
    configures the pins direction, initial state, analog setting.
    The peripheral pin select, PPS, configuration is also handled by this manager.

  @Description:
    This source file provides implementations for PIN MANAGER.
    Generation Information :
        Product Revision  :  PIC24 / dsPIC33 / PIC32MM MCUs - 1.171.1
        Device            :  PIC24FJ128GA705
    The generated drivers are tested against the following:
        Compiler          :  XC16 v1.70
        MPLAB 	          :  MPLAB X v5.50
*/

/*
    (c) 2020 Microchip Technology Inc. and its subsidiaries. You may use this
    software and any derivatives exclusively with Microchip products.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION
    WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

    MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
    TERMS.
*/
*/

#ifndef _PIN_MANAGER_H
#define _PIN_MANAGER_H
/**
    Section: Includes
*/
#include <xc.h>


#define LED_BLUE_Toggle()           _LATC5 ^= 1
#define LED_BLUE_SetDigitalOutput() _TRISC5 = 0
 
#define LED_GREEN_On()               _LATC4= 1 
#define LED_GREEN_Off()              _LATC4= 0 
#define LED_GREEN_Toggle()           _LATC4 ^= 1    
#define LED_GREEN_SetDigitalOutput() _TRISC4 = 0
/**
    Section: Device Pin Macros
*/
/**
  @Summary
    Sets the GPIO pin, RA10, high using LATA10.

  @Description
    Sets the GPIO pin, RA10, high using LATA10.

  @Preconditions
    The RA10 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RA10 high (1)
    IO_RA10_WAKE_SetHigh();
    </code>

*/
#define LED_RED_Toggle()                  _LATB4 ^= 1

#define IO_RA10_WAKE_SetHigh()          _LATA10 = 1
/**
  @Summary
    Sets the GPIO pin, RA10, low using LATA10.

  @Description
    Sets the GPIO pin, RA10, low using LATA10.

  @Preconditions
    The RA10 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RA10 low (0)
    IO_RA10_WAKE_SetLow();
    </code>

*/
#define IO_RA10_WAKE_SetLow()           _LATA10 = 0
/**
  @Summary
    Toggles the GPIO pin, RA10, using LATA10.

  @Description
    Toggles the GPIO pin, RA10, using LATA10.

  @Preconditions
    The RA10 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RA10
    IO_RA10_WAKE_Toggle();
    </code>

*/
#define IO_RA10_WAKE_Toggle()           _LATA10 ^= 1
/**
  @Summary
    Reads the value of the GPIO pin, RA10.

  @Description
    Reads the value of the GPIO pin, RA10.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RA10
    postValue = IO_RA10_WAKE_GetValue();
    </code>

*/
#define IO_RA10_WAKE_GetValue()         _RA10
/**
  @Summary
    Configures the GPIO pin, RA10, as an input.

  @Description
    Configures the GPIO pin, RA10, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RA10 as an input
    IO_RA10_WAKE_SetDigitalInput();
    </code>

*/
#define IO_RA10_WAKE_SetDigitalInput()  _TRISA10 = 1
/**
  @Summary
    Configures the GPIO pin, RA10, as an output.

  @Description
    Configures the GPIO pin, RA10, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RA10 as an output
    IO_RA10_WAKE_SetDigitalOutput();
    </code>

*/
#define IO_RA10_WAKE_SetDigitalOutput() _TRISA10 = 0
/**
  @Summary
    Sets the GPIO pin, RA14, high using LATA14.

  @Description
    Sets the GPIO pin, RA14, high using LATA14.

  @Preconditions
    The RA14 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RA14 high (1)
    IO_RA14_EXINT_SetHigh();
    </code>

*/
#define IO_RA14_EXINT_SetHigh()          _LATA14 = 1
/**
  @Summary
    Sets the GPIO pin, RA14, low using LATA14.

  @Description
    Sets the GPIO pin, RA14, low using LATA14.

  @Preconditions
    The RA14 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RA14 low (0)
    IO_RA14_EXINT_SetLow();
    </code>

*/
#define IO_RA14_EXINT_SetLow()           _LATA14 = 0
/**
  @Summary
    Toggles the GPIO pin, RA14, using LATA14.

  @Description
    Toggles the GPIO pin, RA14, using LATA14.

  @Preconditions
    The RA14 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RA14
    IO_RA14_EXINT_Toggle();
    </code>

*/
#define IO_RA14_EXINT_Toggle()           _LATA14 ^= 1
/**
  @Summary
    Reads the value of the GPIO pin, RA14.

  @Description
    Reads the value of the GPIO pin, RA14.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RA14
    postValue = IO_RA14_EXINT_GetValue();
    </code>

*/
#define IO_RA14_EXINT_GetValue()         _RA14
/**
  @Summary
    Configures the GPIO pin, RA14, as an input.

  @Description
    Configures the GPIO pin, RA14, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RA14 as an input
    IO_RA14_EXINT_SetDigitalInput();
    </code>

*/
#define IO_RA14_EXINT_SetDigitalInput()  _TRISA14 = 1
/**
  @Summary
    Configures the GPIO pin, RA14, as an output.

  @Description
    Configures the GPIO pin, RA14, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RA14 as an output
    IO_RA14_EXINT_SetDigitalOutput();
    </code>

*/
#define IO_RA14_EXINT_SetDigitalOutput() _TRISA14 = 0
/**
  @Summary
    Sets the GPIO pin, RA15, high using LATA15.

  @Description
    Sets the GPIO pin, RA15, high using LATA15.

  @Preconditions
    The RA15 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RA15 high (1)
    IO_RA15_EXINT_SetHigh();
    </code>

*/
#define IO_RA15_EXINT_SetHigh()          _LATA15 = 1
/**
  @Summary
    Sets the GPIO pin, RA15, low using LATA15.

  @Description
    Sets the GPIO pin, RA15, low using LATA15.

  @Preconditions
    The RA15 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RA15 low (0)
    IO_RA15_EXINT_SetLow();
    </code>

*/
#define IO_RA15_EXINT_SetLow()           _LATA15 = 0
/**
  @Summary
    Toggles the GPIO pin, RA15, using LATA15.

  @Description
    Toggles the GPIO pin, RA15, using LATA15.

  @Preconditions
    The RA15 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RA15
    IO_RA15_EXINT_Toggle();
    </code>

*/
#define IO_RA15_EXINT_Toggle()           _LATA15 ^= 1
/**
  @Summary
    Reads the value of the GPIO pin, RA15.

  @Description
    Reads the value of the GPIO pin, RA15.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RA15
    postValue = IO_RA15_EXINT_GetValue();
    </code>

*/
#define IO_RA15_EXINT_GetValue()         _RA15
/**
  @Summary
    Configures the GPIO pin, RA15, as an input.

  @Description
    Configures the GPIO pin, RA15, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RA15 as an input
    IO_RA15_EXINT_SetDigitalInput();
    </code>

*/
#define IO_RA15_EXINT_SetDigitalInput()  _TRISA15 = 1
/**
  @Summary
    Configures the GPIO pin, RA15, as an output.

  @Description
    Configures the GPIO pin, RA15, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RA15 as an output
    IO_RA15_EXINT_SetDigitalOutput();
    </code>

*/
#define IO_RA15_EXINT_SetDigitalOutput() _TRISA15 = 0
/**
  @Summary
    Sets the GPIO pin, RD0, high using LATD0.

  @Description
    Sets the GPIO pin, RD0, high using LATD0.

  @Preconditions
    The RD0 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RD0 high (1)
    IO_RD0_CE_SetHigh();
    </code>

*/
#define IO_RD0_CE_SetHigh()          _LATD0 = 1
/**
  @Summary
    Sets the GPIO pin, RD0, low using LATD0.

  @Description
    Sets the GPIO pin, RD0, low using LATD0.

  @Preconditions
    The RD0 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RD0 low (0)
    IO_RD0_CE_SetLow();
    </code>

*/
#define IO_RD0_CE_SetLow()           _LATD0 = 0
/**
  @Summary
    Toggles the GPIO pin, RD0, using LATD0.

  @Description
    Toggles the GPIO pin, RD0, using LATD0.

  @Preconditions
    The RD0 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RD0
    IO_RD0_CE_Toggle();
    </code>

*/
#define IO_RD0_CE_Toggle()           _LATD0 ^= 1
/**
  @Summary
    Reads the value of the GPIO pin, RD0.

  @Description
    Reads the value of the GPIO pin, RD0.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RD0
    postValue = IO_RD0_CE_GetValue();
    </code>

*/
#define IO_RD0_CE_GetValue()         _RD0
/**
  @Summary
    Configures the GPIO pin, RD0, as an input.

  @Description
    Configures the GPIO pin, RD0, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RD0 as an input
    IO_RD0_CE_SetDigitalInput();
    </code>

*/
#define IO_RD0_CE_SetDigitalInput()  _TRISD0 = 1
/**
  @Summary
    Configures the GPIO pin, RD0, as an output.

  @Description
    Configures the GPIO pin, RD0, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RD0 as an output
    IO_RD0_CE_SetDigitalOutput();
    </code>

*/
#define IO_RD0_CE_SetDigitalOutput() _TRISD0 = 0
/**
  @Summary
    Sets the GPIO pin, RG0, high using LATG0.

  @Description
    Sets the GPIO pin, RG0, high using LATG0.

  @Preconditions
    The RG0 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RG0 high (1)
    IO_RG0_RESET_SetHigh();
    </code>

*/
#define IO_RG0_RESET_SetHigh()          _LATA2 = 1
/**
  @Summary
    Sets the GPIO pin, RG0, low using LATG0.

  @Description
    Sets the GPIO pin, RG0, low using LATG0.

  @Preconditions
    The RG0 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RG0 low (0)
    IO_RG0_RESET_SetLow();
    </code>

*/
#define IO_RG0_RESET_SetLow()           _LATA2 = 0
/**
  @Summary
    Toggles the GPIO pin, RG0, using LATG0.

  @Description
    Toggles the GPIO pin, RG0, using LATG0.

  @Preconditions
    The RG0 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RG0
    IO_RG0_RESET_Toggle();
    </code>

*/
#define IO_RG0_RESET_Toggle()           _LATA2 ^= 1
/**
  @Summary
    Reads the value of the GPIO pin, RG0.

  @Description
    Reads the value of the GPIO pin, RG0.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RG0
    postValue = IO_RG0_RESET_GetValue();
    </code>

*/
#define IO_RG0_RESET_GetValue()         _RA2
/**
  @Summary
    Configures the GPIO pin, RG0, as an input.

  @Description
    Configures the GPIO pin, RG0, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RG0 as an input
    IO_RG0_RESET_SetDigitalInput();
    </code>

*/
#define IO_RG0_RESET_SetDigitalInput()  _TRISA2 = 1
/**
  @Summary
    Configures the GPIO pin, RG0, as an output.

  @Description
    Configures the GPIO pin, RG0, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RG0 as an output
    IO_RG0_RESET_SetDigitalOutput();
    </code>

*/
#define IO_RG0_RESET_SetDigitalOutput() _TRISA2 = 0
/**
  @Summary
    Sets the GPIO pin, RG1, high using LATG1.

  @Description
    Sets the GPIO pin, RG1, high using LATG1.

  @Preconditions
    The RG1 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RG1 high (1)
    IO_RG1_CE_SetHigh();
    </code>

*/
#define IO_RG1_CE_SetHigh()          _LATA3 = 1
/**
  @Summary
    Sets the GPIO pin, RG1, low using LATG1.

  @Description
    Sets the GPIO pin, RG1, low using LATG1.

  @Preconditions
    The RG1 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RG1 low (0)
    IO_RG1_CE_SetLow();
    </code>

*/
#define IO_RG1_CE_SetLow()           _LATA3 = 0
/**
  @Summary
    Toggles the GPIO pin, RG1, using LATG1.

  @Description
    Toggles the GPIO pin, RG1, using LATG1.

  @Preconditions
    The RG1 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RG1
    IO_RG1_CE_Toggle();
    </code>

*/
#define IO_RG1_CE_Toggle()           _LATA3 ^= 1
/**
  @Summary
    Reads the value of the GPIO pin, RG1.

  @Description
    Reads the value of the GPIO pin, RG1.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RG1
    postValue = IO_RG1_CE_GetValue();
    </code>

*/
#define IO_RG1_CE_GetValue()         _RA3
/**
  @Summary
    Configures the GPIO pin, RG1, as an input.

  @Description
    Configures the GPIO pin, RG1, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RG1 as an input
    IO_RG1_CE_SetDigitalInput();
    </code>

*/
#define IO_RG1_CE_SetDigitalInput()  _TRISA3 = 1
/**
  @Summary
    Configures the GPIO pin, RG1, as an output.

  @Description
    Configures the GPIO pin, RG1, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RG1 as an output
    IO_RG1_CE_SetDigitalOutput();
    </code>

*/
#define IO_RG1_CE_SetDigitalOutput() _TRISA3 = 0
/**
  @Summary
    Sets the GPIO pin, RG14, high using LATG14.

  @Description
    Sets the GPIO pin, RG14, high using LATG14.

  @Preconditions
    The RG14 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RG14 high (1)
    IO_RG14_RESET_SetHigh();
    </code>

*/
#define IO_RG14_RESET_SetHigh()          _LATG14 = 1
/**
  @Summary
    Sets the GPIO pin, RG14, low using LATG14.

  @Description
    Sets the GPIO pin, RG14, low using LATG14.

  @Preconditions
    The RG14 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RG14 low (0)
    IO_RG14_RESET_SetLow();
    </code>

*/
#define IO_RG14_RESET_SetLow()           _LATG14 = 0
/**
  @Summary
    Toggles the GPIO pin, RG14, using LATG14.

  @Description
    Toggles the GPIO pin, RG14, using LATG14.

  @Preconditions
    The RG14 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RG14
    IO_RG14_RESET_Toggle();
    </code>

*/
#define IO_RG14_RESET_Toggle()           _LATG14 ^= 1
/**
  @Summary
    Reads the value of the GPIO pin, RG14.

  @Description
    Reads the value of the GPIO pin, RG14.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RG14
    postValue = IO_RG14_RESET_GetValue();
    </code>

*/
#define IO_RG14_RESET_GetValue()         _RG14
/**
  @Summary
    Configures the GPIO pin, RG14, as an input.

  @Description
    Configures the GPIO pin, RG14, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RG14 as an input
    IO_RG14_RESET_SetDigitalInput();
    </code>

*/
#define IO_RG14_RESET_SetDigitalInput()  _TRISG14 = 1
/**
  @Summary
    Configures the GPIO pin, RG14, as an output.

  @Description
    Configures the GPIO pin, RG14, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RG14 as an output
    IO_RG14_RESET_SetDigitalOutput();
    </code>

*/
#define IO_RG14_RESET_SetDigitalOutput() _TRISG14 = 0
/**
  @Summary
    Sets the GPIO pin, RG7, high using LATG7.

  @Description
    Sets the GPIO pin, RG7, high using LATG7.

  @Preconditions
    The RG7 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RG7 high (1)
    IO_RG7_MISO_SetHigh();
    </code>

*/
#define IO_RG7_MISO_SetHigh()          _LATG7 = 1
/**
  @Summary
    Sets the GPIO pin, RG7, low using LATG7.

  @Description
    Sets the GPIO pin, RG7, low using LATG7.

  @Preconditions
    The RG7 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RG7 low (0)
    IO_RG7_MISO_SetLow();
    </code>

*/
#define IO_RG7_MISO_SetLow()           _LATG7 = 0
/**
  @Summary
    Toggles the GPIO pin, RG7, using LATG7.

  @Description
    Toggles the GPIO pin, RG7, using LATG7.

  @Preconditions
    The RG7 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RG7
    IO_RG7_MISO_Toggle();
    </code>

*/
#define IO_RG7_MISO_Toggle()           _LATG7 ^= 1
/**
  @Summary
    Reads the value of the GPIO pin, RG7.

  @Description
    Reads the value of the GPIO pin, RG7.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RG7
    postValue = IO_RG7_MISO_GetValue();
    </code>

*/
#define IO_RG7_MISO_GetValue()         _RG7
/**
  @Summary
    Configures the GPIO pin, RG7, as an input.

  @Description
    Configures the GPIO pin, RG7, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RG7 as an input
    IO_RG7_MISO_SetDigitalInput();
    </code>

*/
#define IO_RG7_MISO_SetDigitalInput()  _TRISG7 = 1
/**
  @Summary
    Configures the GPIO pin, RG7, as an output.

  @Description
    Configures the GPIO pin, RG7, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RG7 as an output
    IO_RG7_MISO_SetDigitalOutput();
    </code>

*/
#define IO_RG7_MISO_SetDigitalOutput() _TRISG7 = 0
/**
  @Summary
    Sets the GPIO pin, RG9, high using LATG9.

  @Description
    Sets the GPIO pin, RG9, high using LATG9.

  @Preconditions
    The RG9 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RG9 high (1)
    IO_RG9_SS2_SetHigh();
    </code>

*/
#define IO_RG9_SS2_SetHigh()          _LATC1 = 1
/**
  @Summary
    Sets the GPIO pin, RG9, low using LATG9.

  @Description
    Sets the GPIO pin, RG9, low using LATG9.

  @Preconditions
    The RG9 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RG9 low (0)
    IO_RG9_SS2_SetLow();
    </code>

*/
#define IO_RG9_SS2_SetLow()           _LATC1 = 0
/**
  @Summary
    Toggles the GPIO pin, RG9, using LATG9.

  @Description
    Toggles the GPIO pin, RG9, using LATG9.

  @Preconditions
    The RG9 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RG9
    IO_RG9_SS2_Toggle();
    </code>

*/
#define IO_RG9_SS2_Toggle()           _LATC1 ^= 1
/**
  @Summary
    Reads the value of the GPIO pin, RG9.

  @Description
    Reads the value of the GPIO pin, RG9.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RG9
    postValue = IO_RG9_SS2_GetValue();
    </code>

*/
#define IO_RG9_SS2_GetValue()         _RC1
/**
  @Summary
    Configures the GPIO pin, RG9, as an input.

  @Description
    Configures the GPIO pin, RG9, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RG9 as an input
    IO_RG9_SS2_SetDigitalInput();
    </code>

*/
#define IO_RG9_SS2_SetDigitalInput()  _TRISC1 = 1
/**
  @Summary
    Configures the GPIO pin, RG9, as an output.

  @Description
    Configures the GPIO pin, RG9, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RG9 as an output
    IO_RG9_SS2_SetDigitalOutput();
    </code>

*/
#define IO_RG9_SS2_SetDigitalOutput() _TRISC1 = 0

/**
    Section: Function Prototypes
*/
/**
  @Summary
    Configures the pin settings of the PIC24FJ128GA310
    The peripheral pin select, PPS, configuration is also handled by this manager.

  @Description
    This is the generated manager file for the MPLAB(c) Code Configurator device.  This manager
    configures the pins direction, initial state, analog setting.
    The peripheral pin select, PPS, configuration is also handled by this manager.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    void SYSTEM_Initialize(void)
    {
        // Other initializers are called from this function
        PIN_MANAGER_Initialize();
    }
    </code>

*/
void PIN_MANAGER_Initialize(void);

#endif
