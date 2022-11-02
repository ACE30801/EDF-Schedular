/*
 * FreeRTOS Kernel V10.2.0
 * Copyright (C) 2019 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/* 
	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used.
*/


/*
 * Creates all the demo application tasks, then starts the scheduler.  The WEB
 * documentation provides more details of the demo application tasks.
 * 
 * Main.c also creates a task called "Check".  This only executes every three 
 * seconds but has the highest priority so is guaranteed to get processor time.  
 * Its main function is to check that all the other tasks are still operational.
 * Each task (other than the "flash" tasks) maintains a unique count that is 
 * incremented each time the task successfully completes its function.  Should 
 * any error occur within such a task the count is permanently halted.  The 
 * check task inspects the count of each task to ensure it has changed since
 * the last time the check task executed.  If all the count variables have 
 * changed all the tasks are still executing error free, and the check task
 * toggles the onboard LED.  Should any task contain an error at any time 
 * the LED toggle rate will change from 3 seconds to 500ms.
 *
 */

/* Standard includes. */
#include <stdlib.h>
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "lpc21xx.h"

/* Peripheral includes. */
#include "serial.h"
#include "GPIO.h"


/*****additions*****/
#include "string.h"
#include "message_buffer.h"

/*-----------------------------------------------------------*/

/* Constants to setup I/O and processor. */
#define mainBUS_CLK_FULL	( ( unsigned char ) 0x01 )

/* Constants for the ComTest demo application tasks. */
#define mainCOM_TEST_BAUD_RATE	( ( unsigned long ) 115200 )


#define TASK1_DEADLINE		50
#define TASK2_DEADLINE		50
#define TASK3_DEADLINE		100
#define TASK4_DEADLINE		20
#define TASK5_DEADLINE		10
#define TASK6_DEADLINE		100

char timeStatusBuffer[280]; 


/*
 * Configure the processor for use with the Keil demo board.  This is very
 * minimal as most of the setup is managed by the settings in the project
 * file.
 */
static void prvSetupHardware( void );/*initialize hardware*/
/*-----------------------------------------------------------*/
int i1, i2, i3, i4;
int i_Massage;
volatile int Task1misses = 0;
volatile int Task2misses = 0;
volatile int Task3misses = 0;
volatile int Task4misses = 0;
volatile int Task5misses = 0;
volatile int Task6misses = 0;


volatile int Button1Status = 0;
volatile int Button2Status = 0;

/*
char* Button2Str_Rise = NULL;
char* Button2Str_Fall = NULL;


char* Button1Str_Rise = NULL;
char* Button1Str_Fall = NULL;
*/

#define RISING_STRING_1		("There is a Rising Edge on Button 1")
#define FALLING_STRING_1		("There is a Falling Edge on Button 1")


#define RISING_STRING_2		("There is a Rising Edge on Button 2")
#define FALLING_STRING_2		("There is a Falling Edge on Button 2")

#define RISING_STRING_L		(strlen(RISING_STRING_1))
#define FALLING_STRING_L		(strlen(FALLING_STRING_1))

_Bool RiseFlag1 = 0, RiseFlag2 = 0, FallFlag1 = 0, FallFlag2 = 0;

/*-----------Massage Buffer-------------*/
#define xMassageBufferSizeBytes	100

MessageBufferHandle_t xMessageUartBuffer;



/*Idle Hook*/
void vApplicationIdleHook( void )
{
		
		/*Code*/	
	
}

/*Tick Hook*/
void vApplicationTickHook( void )
{
	/*Code*/
	GPIO_write(PORT_0, PIN0, PIN_IS_HIGH);
	GPIO_write(PORT_0, PIN0, PIN_IS_LOW);
}

void Button_1_Monitor(void* pvParameters )
{
	
	TickType_t LastWakeTime = xTaskGetTickCount();
	int EndTime = 0;
	int StartTime=0;

	for(;;)
	{
		/*excution time = 6us, priodicity = 50ms, priority = 1, deadline=50ms */
		
		/*code here*/
		
		if(!Button1Status && GPIO_read(PORT_1, PIN0))
		{
				Button1Status =1;
				RiseFlag1 = 1;
		}
		
		if(Button1Status && !(GPIO_read(PORT_1, PIN0)))
		{
			Button1Status = 0;
				FallFlag1 = 1;
		}
		
		

		EndTime = xTaskGetTickCount();
		if((EndTime -StartTime) > TASK1_DEADLINE)
		{
			Task1misses++;
		}

		GPIO_write(PORT_0, PIN1, PIN_IS_HIGH);

		vTaskDelayUntil(&LastWakeTime, TASK1_DEADLINE);

		GPIO_write(PORT_0, PIN1, PIN_IS_LOW);
		
		StartTime = xTaskGetTickCount();
		
		
		
	}

}

void Button_2_Monitor(void* pvParameters )
{
	TickType_t LastWakeTime = xTaskGetTickCount();
	int EndTime = 0;
	int StartTime=0;
	uint8_t xBytesSent;
	for(;;)
	{
		/*excution time = 6us, priodicity = 50ms, priority = 1, deadline=50ms */
		
		/*code here*/
		
		if(!Button2Status && GPIO_read(PORT_1, PIN1))
		{
				Button2Status = 1;
				RiseFlag2 = 1;
		}                   
		
		if(Button2Status && !(GPIO_read(PORT_1, PIN1)))
		{
				Button2Status = 0;
				FallFlag2 = 1;
		}
		
		
		EndTime = xTaskGetTickCount();
		
		if((EndTime -StartTime) > TASK2_DEADLINE)
		{
			Task2misses++;
		}

		GPIO_write(PORT_0, PIN2, PIN_IS_HIGH);		
		vTaskDelayUntil(&LastWakeTime, TASK2_DEADLINE);
		GPIO_write(PORT_0, PIN2, PIN_IS_LOW);
		
		StartTime = xTaskGetTickCount();
	}

}

void Periodic_Transmitter(void* pvParameters )
{
	TickType_t LastWakeTime = xTaskGetTickCount();
	int EndTime = 0;
	int StartTime=0;
	uint8_t	xBytesSent;	
	for(;;)
	{
		/*excution time = (5.6us(Idle), 30.6(Active)), priodicity = 100ms, priority = 1, deadline=100ms */
		
		
		/*code here*/
		
		if(RiseFlag1 == 0)
		{
				/*Do Nothing*/
		}
		else
		{
			/*Send massage to Uart Receiver*/
				/*xSerialPutChar('\n');
				vSerialPutString((const signed char *)Button1Str_Rise, RISING_STRING_L);*/
			xBytesSent = xMessageBufferSend( xMessageUartBuffer,
                                    ( void * ) RISING_STRING_1  ,
                                    RISING_STRING_L , 0 );
					if(xBytesSent < RISING_STRING_L)
					{
						vSerialPutString((const signed char *)"the massage buffer is full\n", 27);
					}
			RiseFlag1 =0;
		}
		
		if(FallFlag1 == 0)
		{
				/*Do Nothing*/
		}
		else
		{
			/*Send massage to Uart Receiver*/
			
			/*
				xSerialPutChar('\n');
				vSerialPutString((const signed char *)Button1Str_Fall, FALLING_STRING_L);*/
			xBytesSent = xMessageBufferSend( xMessageUartBuffer,
                                    ( void * ) FALLING_STRING_1  ,
                                    FALLING_STRING_L , 0 );
					if(xBytesSent < FALLING_STRING_L)
					{
						vSerialPutString((const signed char *)"the massage buffer is full\n", 27);
					}
					FallFlag1 =0;
		}
		
		if(RiseFlag2 == 0)
		{
				/*Do Nothing*/
		}
		else
		{
			/*Send massage to Uart Receiver*/
			/*
				xSerialPutChar('\n');
				vSerialPutString((const signed char *)Button2Str_Rise, RISING_STRING_L);*/
			xBytesSent = xMessageBufferSend( xMessageUartBuffer,
                                    ( void * ) RISING_STRING_2  ,
                                    RISING_STRING_L , 0 );
					if(xBytesSent < RISING_STRING_L)
					{
						vSerialPutString((const signed char *)"the massage buffer is full\n", 27);
					}
			RiseFlag2 =0;		
		}
		
		if(FallFlag2 == 0)
		{
				/*Do Nothing*/
		}
		else
		{
			/*Send massage to Uart Receiver*/
			/*
				xSerialPutChar('\n');
				vSerialPutString((const signed char *)Button2Str_Fall, FALLING_STRING_L);*/
			xBytesSent = xMessageBufferSend( xMessageUartBuffer,
                                    ( void * ) FALLING_STRING_2  ,
                                    FALLING_STRING_L , 0 );
					if(xBytesSent < FALLING_STRING_L)
					{
						vSerialPutString((const signed char *)"the massage buffer is full\n", 27);
					}
					FallFlag2 =0;
		}
		
		vTaskGetRunTimeStats(timeStatusBuffer);
	
		xSerialPutChar('\n');
		
		vSerialPutString((const signed char* )timeStatusBuffer, 190);
		

		
		EndTime = xTaskGetTickCount();
		
		if((EndTime -StartTime) > TASK3_DEADLINE)
		{
			Task3misses++;
		}

		GPIO_write(PORT_0, PIN3, PIN_IS_HIGH);		
		vTaskDelayUntil(&LastWakeTime, TASK3_DEADLINE);
		GPIO_write(PORT_0, PIN3, PIN_IS_LOW);
		
		StartTime = xTaskGetTickCount();
	}

}

void Uart_Receiver(void* pvParameters )
{

	char Massage1[xMassageBufferSizeBytes];
	TickType_t LastWakeTime = xTaskGetTickCount();
	int EndTime = 0;
	int StartTime=0;
	
	size_t 	MassageSize;	
    /* Create a message buffer that can hold 100 bytes.  The memory used to hold
    both the message buffer structure and the data in the message buffer is
    allocated dynamically. */
    xMessageUartBuffer = xMessageBufferCreate( xMassageBufferSizeBytes );

    if( xMessageUartBuffer == NULL )
    {
        /* There was not enough heap memory space available to create the
        message buffer. */
    }
    else
    {
        /* The message buffer was created successfully and can now be used. */
    }


	for(;;)
	{
		/*excution time = (6.6us(Idle), 25us(Active)), priodicity = 20ms, priority = 1, deadline=20ms */
		
		/*code here*/
		
//	if(/*Check on Inbox*/)
//	{
//	
//		/*send to Serial*/
//	}
		
		MassageSize = xMessageBufferReceive( xMessageUartBuffer,
                                            ( void * ) Massage1,
                                            sizeof( Massage1 ),
                                            0 );

    if( MassageSize > 0 )
    {
        
				/* A ucRxData contains a message that is xReceivedBytes long.  Process
        the message here.... */
//			for(i_Massage=0;i_Massage<MassageSize;i_Massage++)
//			{
//				xSerialPutChar(Massage1[i_Massage]);
//			}
//			if(Massage1[i_Massage]=='\0')
//			{
//				xSerialPutChar('\n');
//			}
				
				xSerialPutChar('\n');
				vSerialPutString(Massage1, MassageSize);
			
		}
		

		
		EndTime = xTaskGetTickCount();
		
		if((EndTime -StartTime) > TASK4_DEADLINE)
		{
			Task4misses++;
		}

		GPIO_write(PORT_0, PIN4, PIN_IS_HIGH);		
		vTaskDelayUntil(&LastWakeTime, TASK4_DEADLINE);
		GPIO_write(PORT_0, PIN4, PIN_IS_LOW);
		
		StartTime = xTaskGetTickCount();
	}

}


void Load_1_Simulation(void* pvParameters )
{
	TickType_t LastWakeTime = xTaskGetTickCount();
	int EndTime = 0;
	int StartTime=xTaskGetTickCount();
	
	for(;;)
	{
		/*excution time = 5.06ms, priodicity = 10ms, priority = 1, deadline=10ms */
		
		/*code here*/
		for(i1 =0; i1<9607;i1++)i1=i1;
		
		EndTime = xTaskGetTickCount();
		
		if((EndTime -StartTime) > TASK5_DEADLINE)
		{
			Task5misses++;
		}

		GPIO_write(PORT_0, PIN5, PIN_IS_HIGH);		
		vTaskDelayUntil(&LastWakeTime, TASK5_DEADLINE);
		GPIO_write(PORT_0, PIN5, PIN_IS_LOW);
		
		StartTime = xTaskGetTickCount();
	}

}

void Load_2_Simulation(void* pvParameters )
{
	TickType_t LastWakeTime = xTaskGetTickCount();
	int EndTime = 0;
	int StartTime=0;
	
	for(;;)
	{
		/*excution time = 12.1ms, priodicity = 10ms, priority = 1, deadline=10ms */
		
		/*code here*/
		for(i2 =0; i2<23060;i2++)i2=i2;
		
		EndTime = xTaskGetTickCount();
		
		if((EndTime -StartTime) > TASK6_DEADLINE)
		{
			Task6misses++;
		}

		GPIO_write(PORT_0, PIN6, PIN_IS_HIGH);		
		vTaskDelayUntil(&LastWakeTime, TASK6_DEADLINE);
		GPIO_write(PORT_0, PIN6, PIN_IS_LOW);
		
		StartTime = xTaskGetTickCount();
	}

}


TaskHandle_t Task1Handler = NULL;
TaskHandle_t Task2Handler = NULL;
TaskHandle_t Task3Handler = NULL;
TaskHandle_t Task4Handler = NULL;
TaskHandle_t Task5Handler = NULL;
TaskHandle_t Task6Handler = NULL;


/*
 * Application entry point:
 * Starts all the other tasks, then starts the scheduler. 
 */
int main( void )
{
	/* Setup the hardware for use with the Keil demo board. */
	prvSetupHardware();

	
    /* Create Tasks here */


	
	
	
    /* Create the task, storing the handle. */
		
		xTaskCreatePeriodic(
											Button_1_Monitor,       /* Function that implements the task. */
											"Task1",          /* Text name for the task. */
											100,      /* Stack size in words, not bytes. */
											( void * ) 0,    /* Parameter passed into the task. */
											1,/* Priority at which the task is created. */
											&Task1Handler, TASK1_DEADLINE );      /* Used to pass out the created task's handle. */
	
			/* Create the task, storing the handle. */
		xTaskCreatePeriodic(
                    Button_2_Monitor,       /* Function that implements the task. */
                    "Task2",          /* Text name for the task. */
                    100,      /* Stack size in words, not bytes. */
                    ( void * ) 0,    /* Parameter passed into the task. */
                    1,/* Priority at which the task is created. */
                    &Task2Handler,TASK2_DEADLINE);      /* Used to pass out the created task's handle. */

		xTaskCreatePeriodic(
										Periodic_Transmitter,       /* Function that implements the task. */
										"Task3",          /* Text name for the task. */
										100,      /* Stack size in words, not bytes. */
										( void * ) 0,    /* Parameter passed into the task. */
										1,/* Priority at which the task is created. */
										&Task3Handler,TASK3_DEADLINE);      /* Used to pass out the created task's handle. */
	
										
		xTaskCreatePeriodic(
                    Uart_Receiver,       /* Function that implements the task. */
                    "Task4",          /* Text name for the task. */
                    100,      /* Stack size in words, not bytes. */
                    ( void * ) 0,    /* Parameter passed into the task. */
                    1,/* Priority at which the task is created. */
                    &Task4Handler,TASK4_DEADLINE);      /* Used to pass out the created task's handle. */

										
		xTaskCreatePeriodic(
                    Load_1_Simulation,       /* Function that implements the task. */
                    "Task5",          /* Text name for the task. */
                    100,      /* Stack size in words, not bytes. */
                    ( void * ) 0,    /* Parameter passed into the task. */
                    1,/* Priority at which the task is created. */
                    &Task5Handler,TASK5_DEADLINE);      /* Used to pass out the created task's handle. */

		xTaskCreatePeriodic(
                    Load_2_Simulation,       /* Function that implements the task. */
                    "Task6",          /* Text name for the task. */
                    100,      /* Stack size in words, not bytes. */
                    ( void * ) 0,    /* Parameter passed into the task. */
                    1,/* Priority at which the task is created. */
                    &Task6Handler,TASK6_DEADLINE);      /* Used to pass out the created task's handle. */
										
	/* Now all the tasks have been started - start the scheduler.

	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used here. */

	vTaskStartScheduler();

	/* Should never reach here!  If you do then there was not enough heap
	available for the idle task to be created. */
/*Be careful!!*/
	for( ;; );
										
}
/*-----------------------------------------------------------*/

/* Function to reset timer 1 */
void timer1Reset(void)
{
	T1TCR |= 0x2;
	T1TCR &= ~0x2;
}

/* Function to initialize and start timer 1 */
static void configTimer1(void)
{
	T1PR = 1000;
	T1TCR |= 0x1;
}









static void prvSetupHardware( void )
{
	/* Perform the hardware setup required.  This is minimal as most of the
	setup is managed by the settings in the project file. */

	/* Configure UART */
	xSerialPortInitMinimal(mainCOM_TEST_BAUD_RATE);

	/* Configure GPIO */
	GPIO_init();
	
	/* Config trace timer 1 and read T1TC to get current tick */
	configTimer1();

	/* Setup the peripheral bus to be the same as the PLL output. */
	VPBDIV = mainBUS_CLK_FULL;
}
/*-----------------------------------------------------------*/


