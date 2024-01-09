/*
    FreeRTOS V9.0.0 - Copyright (C) 2016 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that has become a de facto standard.             *
     *                                                                       *
     *    Help yourself get started quickly and support the FreeRTOS         *
     *    project by purchasing a FreeRTOS tutorial book, reference          *
     *    manual, or both from: http://www.FreeRTOS.org/Documentation        *
     *                                                                       *
     *    Thank you!                                                         *
     *                                                                       *
    ***************************************************************************

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>!AND MODIFIED BY!<< the FreeRTOS exception.

    >>! NOTE: The modification to the GPL is included to allow you to distribute
    >>! a combined work that includes FreeRTOS without being obliged to provide
    >>! the source code for proprietary components outside of the FreeRTOS
    >>! kernel.

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available from the following
    link: http://www.freertos.org/a00114.html

    1 tab == 4 spaces!

    ***************************************************************************
     *                                                                       *
     *    Having a problem?  Start by reading the FAQ "My application does   *
     *    not run, what could be wrong?"                                     *
     *                                                                       *
     *    http://www.FreeRTOS.org/FAQHelp.html                               *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org - Documentation, books, training, latest versions,
    license and Real Time Engineers Ltd. contact details.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.OpenRTOS.com - Real Time Engineers ltd license FreeRTOS to High
    Integrity Systems to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/

/*
 * This file contains some supporting functions that are used by all the
 * examples.  These fall into three categories:
 *
 * 1) IO functions:  vPrintString() and vPrintStringAndNumber().
 *
 * To allow maximum portability the book examples do not rely on any chip
 * specific IO, and instead just output to a console.  However, printing to a
 * console in this manner is not thread safe, so a function is used so the
 * terminal output can be wrapped in a critical section.
 *
 * 2) RTOS hook functions: vApplicationMallocFailedHook(), vApplicationIdleHook()
 * vApplicationIdleHook(), vApplicationStackOverflowHook() and
 * vApplicationTickHook().
 *
 * These are functions that can optionally be defined by the application writer
 * in order to get notifications of events occurring with the executing
 * application.  More information is provided in the comments within the
 * function definitions in this file, and in the book text.
 *
 * 3) configASSERT() implementation: vAssertCalled()
 *
 * The FreeRTOS source code uses an assert() function to trap user and other
 * errors.  configASSERT() is defined in FreeRTOSConfig.h to call
 * vAssertCalled(), which is implemented in this file.  More information is
 * provided in the book text.
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Standard includes. */
#include <stdio.h>
#include <string.h>

/* Demo includes. */
#include "supportingFunctions.h"

// ------ Public variable  -----------------------------------------
extern UART_HandleTypeDef huart3;

/*-----------------------------------------------------------*/
/**
 * @brief  Retargets the C library printf function to the USART.
 * @param  None
 * @retval None
 */
PUTCHAR_PROTOTYPE {
    /* Place your implementation of fputc here */
    /* e.g. write a character to the USART3 and Loop until the end of transmission */
    HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xFFFF);

    return ch;
}

/*-----------------------------------------------------------*/

void vPrintString(const char * pcString) {
    /* Print the string, using a critical section as a crude method of mutual
    exclusion. */
    taskENTER_CRITICAL();
    {
        printf("%s", pcString);
        fflush(stdout);
    }
    taskEXIT_CRITICAL();
}
/*-----------------------------------------------------------*/

void vPrintStringAndNumber(const char * pcString, uint32_t ulValue) {
    /* Print the string, using a critical section as a crude method of mutual
    exclusion. */
    taskENTER_CRITICAL();
    {
        printf("%s %lu\r\n", pcString, ulValue);
        fflush(stdout);
    }
    taskEXIT_CRITICAL();
}
/*-----------------------------------------------------------*/

void vPrintTwoStrings(const char * pcString1, const char * pcString2) {
    /* Print the string, using a critical section as a crude method of mutual
    exclusion. */
    vTaskSuspendAll();
    { printf("At time %lu: %s %s\r\n", xTaskGetTickCount(), pcString1, pcString2); }
    xTaskResumeAll();
}
/*-----------------------------------------------------------*/
