/******************************************************************************
 * Filename:       UI_SERVICE.h
 *
 * Description:    This file contains the UI_SERVICE service definitions and
 *                 prototypes.
 *
 *                 Generated by:
 *                 BDS version: 1.1.3139.0
 *                 Plugin:      Texas Instruments BLE SDK GATT Server plugin 1.0.9
 *                 Time:        Fri Feb 16 2018 11:16:33 GMT+02:00
 *

 * Copyright (c) 2015-2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

#ifndef _UI_SERVICE_H_
#define _UI_SERVICE_H_

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include <bcomdef.h>

/*********************************************************************
 * CONSTANTS
 */
// Service UUID
#define UI_SERVICE_SERV_UUID 0xFFE0

// BUTTON Characteristic defines
#define US_BUTTON_ID                 0
#define US_BUTTON_UUID               0xFFE1
#define US_BUTTON_LEN                1
#define US_BUTTON_LEN_MIN            1

// LED1 Characteristic defines
#define US_LED1_ID                 1
#define US_LED1_UUID               0xFFE3
#define US_LED1_LEN                1
#define US_LED1_LEN_MIN            1

// LED2 Characteristic defines
#define US_LED2_ID                 2
#define US_LED2_UUID               0xFFE4
#define US_LED2_LEN                1
#define US_LED2_LEN_MIN            1

/*********************************************************************
 * TYPEDEFS
 */

// Fields in characteristic "BUTTON"
//   Field "STATE" format: boolean, bits: 8

// Fields in characteristic "LED1"
//   Field "STATE" format: boolean, bits: 8

// Fields in characteristic "LED2"
//   Field "STATE" format: boolean, bits: 8

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * Profile Callbacks
 */

// Callback when a characteristic value has changed
typedef void (*UiServiceChange_t)( uint16_t connHandle, uint16_t svcUuid, uint8_t paramID, uint8_t *pValue, uint16_t len );

typedef struct
{
  UiServiceChange_t        pfnChangeCb;     // Called when characteristic value changes
  UiServiceChange_t        pfnCfgChangeCb;  // Called when characteristic CCCD changes
} UiServiceCBs_t;



/*********************************************************************
 * API FUNCTIONS
 */


/*
 * UiService_AddService- Initializes the UiService service by registering
 *          GATT attributes with the GATT server.
 *
 *    rspTaskId - The ICall Task Id that should receive responses for Indications.
 */
extern bStatus_t UiService_AddService( uint8_t rspTaskId );

/*
 * UiService_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
extern bStatus_t UiService_RegisterAppCBs( UiServiceCBs_t *appCallbacks );

/*
 * UiService_SetParameter - Set a UiService parameter.
 *
 *    param - Profile parameter ID
 *    len   - length of data to write
 *    value - pointer to data to write.  This is dependent on
 *            the parameter ID and may be cast to the appropriate
 *            data type (example: data type of uint16_t will be cast to
 *            uint16_t pointer).
 */
extern bStatus_t UiService_SetParameter( uint8_t param, uint16_t len, void *value );

/*
 * UiService_GetParameter - Get a UiService parameter.
 *
 *    param - Profile parameter ID
 *    len   - pointer to a variable that contains the maximum length that can be written to *value.
              After the call, this value will contain the actual returned length.
 *    value - pointer to data to write.  This is dependent on
 *            the parameter ID and may be cast to the appropriate
 *            data type (example: data type of uint16_t will be cast to
 *            uint16_t pointer).
 */
extern bStatus_t UiService_GetParameter( uint8_t param, uint16_t *len, void *value );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* _UI_SERVICE_H_ */
