/*
    FreeRTOS V6.0.5 - Copyright (C) 2010 Real Time Engineers Ltd.

    ***************************************************************************
    *                                                                         *
    * If you are:                                                             *
    *                                                                         *
    *    + New to FreeRTOS,                                                   *
    *    + Wanting to learn FreeRTOS or multitasking in general quickly       *
    *    + Looking for basic training,                                        *
    *    + Wanting to improve your FreeRTOS skills and productivity           *
    *                                                                         *
    * then take a look at the FreeRTOS eBook                                  *
    *                                                                         *
    *        "Using the FreeRTOS Real Time Kernel - a Practical Guide"        *
    *                  http://www.FreeRTOS.org/Documentation                  *
    *                                                                         *
    * A pdf reference manual is also available.  Both are usually delivered   *
    * to your inbox within 20 minutes to two hours when purchased between 8am *
    * and 8pm GMT (although please allow up to 24 hours in case of            *
    * exceptional circumstances).  Thank you for your support!                *
    *                                                                         *
    ***************************************************************************

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation AND MODIFIED BY the FreeRTOS exception.
    ***NOTE*** The exception to the GPL is included to allow you to distribute
    a combined work that includes FreeRTOS without being obliged to provide the
    source code for proprietary components outside of the FreeRTOS kernel.
    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT
    ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
    FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
    more details. You should have received a copy of the GNU General Public
    License and the FreeRTOS license exception along with FreeRTOS; if not it
    can be viewed here: http://www.freertos.org/a00114.html and also obtained
    by writing to Richard Barry, contact details for whom are available on the
    FreeRTOS WEB site.

    1 tab == 4 spaces!

    http://www.FreeRTOS.org - Documentation, latest information, license and
    contact details.

    http://www.SafeRTOS.com - A version that is certified for use in safety
    critical systems.

    http://www.OpenRTOS.com - Commercial support, development, porting,
    licensing and training services.
*/

/*
  Change from V4.2.1:

  + Introduced usage of configKERNEL_INTERRUPT_PRIORITY macro to set the
    interrupt priority used by the kernel.
*/

/*-----------------------------------------------------------
 * Implementation of functions defined in portable.h for the ARM CM3 port.
 *----------------------------------------------------------*/

/* Scheduler includes. */
 /**
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
#include "../../../include/FreeRTOS.h"
#include "../../../include/task.h"
#include <board.h>

/* Constants required to manipulate the NVIC. */
#define portNVIC_SYSTICK_CTRL  ((volatile unsigned long *) 0xe000e010)
#define portNVIC_SYSTICK_LOAD  ((volatile unsigned long *) 0xe000e014)
#define portNVIC_INT_CTRL      ((volatile unsigned long *) 0xe000ed04)
#define portNVIC_SYSPRI2       ((volatile unsigned long *) 0xe000ed20)
#define portNVIC_SYSTICK_CLK   0x00000004
#define portNVIC_SYSTICK_INT   0x00000002
#define portNVIC_SYSTICK_ENABLE  0x00000001
#define portNVIC_PENDSVSET       0x10000000
#define portNVIC_PENDSV_PRI      (((unsigned long) configKERNEL_INTERRUPT_PRIORITY) << 16)
#define portNVIC_SYSTICK_PRI     (((unsigned long) configKERNEL_INTERRUPT_PRIORITY) << 24)

/* Constants required to set up the initial stack. */
#define portINITIAL_XPSR         (0x01000000)

/* For backward compatibility, ensure configKERNEL_INTERRUPT_PRIORITY is
defined.  The value 255 should also ensure backward compatibility.
FreeRTOS.org versions prior to V4.3.0 did not include this definition. */
#ifndef configKERNEL_INTERRUPT_PRIORITY
#define configKERNEL_INTERRUPT_PRIORITY 0
#endif

/* Each task maintains its own interrupt status in the critical nesting
variable. */
static unsigned portBASE_TYPE uxCriticalNesting = 0xaaaaaaaa;

/* Setup the timer to generate the tick interrupts */
static void prvSetupTimerInterrupt(void);

/* Exception handlers */
void xPortSysTickHandler(void);
extern void xPortPendSVHandler(void);
extern void vPortSVCHandler(void);

/* Start first task is a separate function so it can be tested in isolation */
extern void vPortStartFirstTask(void);

/**
 * \brief Handler for Sytem interrupt-driven request.
 */
void PendSV_Handler(void)
{
	xPortPendSVHandler();
}

/**
 * \brief Handler for Sytem supervisor call.
 */
void SVC_Handler(void)
{
	vPortSVCHandler();
}

/*
 * \brief See header file for description.
 */
portSTACK_TYPE *pxPortInitialiseStack(portSTACK_TYPE *pxTopOfStack,
		pdTASK_CODE pxCode, void *pvParameters)
{
	/* Simulate the stack frame as it would be created by a context switch interrupt */
	pxTopOfStack--;  /* Offset added to account for the way the MCU uses the stack on entry/exit of interrupts. */
	*pxTopOfStack = portINITIAL_XPSR;  /* xPSR */
	pxTopOfStack--;
	*pxTopOfStack = (portSTACK_TYPE) pxCode; /* PC */
	pxTopOfStack--;
	*pxTopOfStack = 0; /* LR */
	pxTopOfStack -= 5; /* R12, R3, R2 and R1. */
	*pxTopOfStack = (portSTACK_TYPE) pvParameters; /* R0 */
	pxTopOfStack -= 8; /* R11, R10, R9, R8, R7, R6, R5 and R4. */

	return pxTopOfStack;
}

/*
 * \brief See header file for description.
 */
portBASE_TYPE xPortStartScheduler(void)
{
	/* Make PendSV and SysTick the lowest priority interrupts. */
	*(portNVIC_SYSPRI2) |= portNVIC_PENDSV_PRI;
	*(portNVIC_SYSPRI2) |= portNVIC_SYSTICK_PRI;

	/* Start the timer that generates the tick ISR.  Interrupts are disabled
	here already. */
	prvSetupTimerInterrupt();

	/* Initialise the critical nesting count ready for the first task. */
	uxCriticalNesting = 0;

	/* Start the first task. */
	vPortStartFirstTask();

	/* Should not get here! */
	return 0;
}

/**
 * \brief End schedule task.
 */
void vPortEndScheduler(void)
{
	/* It is unlikely that the CM3 port will require this function as there
	is nothing to return to.  */
}

/**
 * \brief Yield PendSV to request a context switch.
 */
void vPortYieldFromISR(void)
{
	/* Set a PendSV to request a context switch. */
	*(portNVIC_INT_CTRL) = portNVIC_PENDSVSET;
}

/**
 * \brief Enter critical section.
 */
void vPortEnterCritical(void)
{
	portDISABLE_INTERRUPTS();
	uxCriticalNesting++;
}

/**
 * \brief Exit critical section.
 */
void vPortExitCritical(void)
{
	uxCriticalNesting--;
	if(uxCriticalNesting == 0) {
		portENABLE_INTERRUPTS();
	}
}

/**
 * \brief Handler for Sytem interrupt-driven request.
 */
void xPortSysTickHandler(void)
{
	unsigned portLONG ulDummy;

	/* If using preemption, also force a context switch. */
#if configUSE_PREEMPTION == 1
	*(portNVIC_INT_CTRL) = portNVIC_PENDSVSET;
#endif

	ulDummy = portSET_INTERRUPT_MASK_FROM_ISR();
	{
		vTaskIncrementTick();
	}
	portCLEAR_INTERRUPT_MASK_FROM_ISR(ulDummy);
}

/*
 * \brief Setup the systick timer to generate the tick interrupts at the required
 * frequency.
 */
void prvSetupTimerInterrupt(void)
{
	/* Configure SysTick to interrupt at the requested rate. */
	*(portNVIC_SYSTICK_LOAD) =
			(configCPU_CLOCK_HZ / configTICK_RATE_HZ) - 1UL;
	*(portNVIC_SYSTICK_CTRL) =
			portNVIC_SYSTICK_CLK | portNVIC_SYSTICK_INT |
			portNVIC_SYSTICK_ENABLE;
}
