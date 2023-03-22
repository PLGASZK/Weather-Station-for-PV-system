/*******************************************************************************
   File Name:
    main.c

  Summary:
    Main entry point for WINC1500 demos.

  Description:
    This file is the main entry point for the WINC1500 demos.  The project is meant 
    as an example of how to create applications for the WINC1500.
*******************************************************************************/

/*==============================================================================
Copyright 2016 Microchip Technology Inc.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/


//==============================================================================
// INCLUDES
//==============================================================================    
#include "winc1500_api.h"   // primary WINC1500 include file
#include "demo_config.h"    // selects which demo to run
#include "bsp.h"            // defines for LED's and push buttons on board
#include "mcc_generated_files/sensors_handling.h"
#include <xc.h> 
#include "stdio.h"
#include "string.h"
#include "stdlib.h"

#define MODBUS_RTU_MAX_ADU_LENGTH 260


bool while_reciving = 0;
//==============================================================================
// FUNCTION PROTOTYPES
//==============================================================================    
static void BlinkLed(void);
static void ResetFunc(void);
static void(* resetFunc) (void) = 0;
void ListenModbus(void);
unsigned short modbus_rtu_calculate_crc(unsigned char *buffer, int length);
int modbus_rtu_encode_request(modbus_rtu_request_t *request, unsigned char *buffer, int length);
int modbus_rtu_decode_request(modbus_rtu_response_t *response, unsigned char *buffer, int length);


typedef struct {
    unsigned char slave_address;
    unsigned char function_code;
    unsigned short data_address;
    unsigned short data_value;
    unsigned short crc;
} modbus_rtu_request_t;

typedef struct {
    unsigned char slave_address;
    unsigned char function_code;
    unsigned short data_length;
    unsigned short data[MODBUS_RTU_MAX_ADU_LENGTH-3];
    unsigned short crc;
} modbus_rtu_response_t;
//==============================================================================
// Main application entry point.
//==============================================================================

//-----------------------------------------------------------------------------
int main(void)
{
    BspInit();

    while (1) 
    {
      //  int Temp = SENSORS_getTempValue();
        //int light = SENSORS_getLightValue();
        
        ApplicationTask();
        m2m_wifi_task();      
        BlinkLed();
        ResetFunc();
        ListenModbus();
       // printf ("\\n temperature  = %d ",Temp);
      //  printf ("\\n Light  = %d ",light);
    }
}

static void BlinkLed(void)
{
    static uint32_t t = 0;
    
    // Blink LED on board every 500ms
    if ((m2mStub_GetOneMsTimer() - t) >= 500) 
    {
        t = m2mStub_GetOneMsTimer();
        _LATB4 ^= 1;
    }
}

static void ResetFunc(void)
{
    static uint32_t t = 0;
    
    // Blink LED on board every 500ms
    if ((m2mStub_GetOneMsTimer() - t) >= 300000) 
    {
        t = m2mStub_GetOneMsTimer();
        resetFunc();
    }
}
unsigned char buffer[24];
unsigned char *p_buffer = &buffer;
unsigned char number_bytes_read = 0;

void ListenModbus(void)
{
    unsigned char byte_read = UART2_Read();

    if(while_reciving)
    {
        if(number_bytes_read < 8)
        {
            *p_buffer = byte_read;
        }
        else
        {
            while_reciving = 0;
            // full message received
            if(modbus_rtu_encode_request(&modbus_rtu_request_t, &buffer, 8) != -1)
            {
                unsigned char bytes[4];
                memcpy(bytes, &get_temperature(void), sizeof(float));
                if(modbus_rtu_request_t.slave_address == 16)
                    if(modbus_rtu_request_t.function_code == 03)
                        if(modbus_rtu_request_t.data_address == 0x4001)
                        {
                            if(modbus_rtu_request_t.data_value == 5)
                            {
                                buffer[2] = 4;                           
                                buffer[3] = bytes[0];
                                send_buffer();
                            }                        
                        }
                        if(modbus_rtu_request_t.data_address == 0x4002)
                        {
                            if(modbus_rtu_request_t.data_value == 1)
                            {
                                buffer[2] = 4;                           
                                buffer[3] = bytes[1];
                                send_buffer();
                            }                        
                        }
                        if(modbus_rtu_request_t.data_address == 0x4003)
                        {
                            if(modbus_rtu_request_t.data_value == 1)
                            {
                                buffer[2] = 4;                           
                                buffer[3] = bytes[2];
                                send_buffer();
                            }                        
                        }
                        if(modbus_rtu_request_t.data_address == 0x4004)
                        {
                            if(modbus_rtu_request_t.data_value == 1)
                            {
                                buffer[2] = 4;                           
                                buffer[3] = bytes[3];
                                send_buffer();
                            }                        
                        }
                        if(modbus_rtu_request_t.data_address == 0x4005)
                        {
                            if(modbus_rtu_request_t.data_value == 1)
                            {
                                buffer[2] = 1;
                                buffer[3] = get_conditions(void);

                                send_buffer();
                            }                        
                            }
            }
        }
        p_buffer++;
        number_bytes_read++;
    }
    else
    {
        if(byte_read)
        {
            while_reciving = 1;
            p_buffer = &buffer;
            *p_buffer = byte_read;
            p_buffer++;
            number_bytes_read = 1;
        }
    }
}

void send_buffer()
{
    p_buffer = &buffer;
    int c;
    for(c = 0; c<21; c++)
    {
        UART2_Write(*p_buffer);
    }
}

unsigned short modbus_rtu_calculate_crc(unsigned char *buffer, int length)
{
    unsigned short crc = 0xFFFF;
    int i, j;

    for(i = 0; i < length; i++) {
        crc = crc ^ buffer[i];
        for(j = 0; j < 8; j++) {
            if(crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc = crc >> 1;
            }
        }
    }

    return crc;
}

int modbus_rtu_encode_request(modbus_rtu_request_t *request, unsigned char *buffer, int length)
{
    if(length < 6){
        return -1;
    }

    buffer[0] = request->slave_address;
    buffer[1] = request->function_code;
    buffer[2] = request->data_address >> 8;
    buffer[3] = request->data_address & 0xFF;
    buffer[4] = request->data_value >> 8;
    buffer[5] = request->data_value & 0xFF;

    request->crc = modbus_rtu_calculate_crc(buffer,6);
    buffer[6] = request->crc & 0xFF;
    buffer[7] = request->crc >> 8;

    return 0;
}

int modbus_rtu_decode_response(modbus_rtu_response_t *response, unsigned char *buffer, int length)
{
    if(length < 6){
        return -1;
    }

    response->slave_address = buffer[0];
    response->function_code = buffer[1];
    response->data_length = length - 2;
    memcpy(response->data, &buffer[2], response->data_length);

    response->crc = modbus_rtu_calculate_crc(buffer, length - 2);
    if(response->crc != ((buffer[length - 1] << 8) | buffer[length - 2])) {
        return -1;
    }

    return 0;
}

//DOM-IGNORE-END