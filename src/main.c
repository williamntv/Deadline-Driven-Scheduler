/*
 FreeRTOS V9.0.0 - Copyright (C) 2016 Real Time Engineers Ltd.
 All rights reserved

 VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

 This file is part of the FreeRTOS distribution.

 FreeRTOS is free software; you can redistribute it and/or modify it under
 the terms of the GNU General Public License (version 2) as published by the
 Free Software Foundation >>>> AND MODIFIED BY <<<< the FreeRTOS exception.

 ***************************************************************************
 >>!   NOTE: The modification to the GPL is included to allow you to     !<<
 >>!   distribute a combined work that includes FreeRTOS without being   !<<
 >>!   obliged to provide the source code for proprietary components     !<<
 >>!   outside of the FreeRTOS kernel.                                   !<<
 ***************************************************************************

 FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
 WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 FOR A PARTICULAR PURPOSE.  Full license text is available on the following
 link: http://www.freertos.org/a00114.html

 ***************************************************************************
 *                                                                       *
 *    FreeRTOS provides completely free yet professionally developed,    *
 *    robust, strictly quality controlled, supported, and cross          *
 *    platform software that is more than just the market leader, it     *
 *    is the industry's de facto standard.                               *
 *                                                                       *
 *    Help yourself get started quickly while simultaneously helping     *
 *    to support the FreeRTOS project by purchasing a FreeRTOS           *
 *    tutorial book, reference manual, or both:                          *
 *    http://www.FreeRTOS.org/Documentation                              *
 *                                                                       *
 ***************************************************************************

 http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
 the FAQ page "My application does not run, what could be wwrong?".  Have you
 defined configASSERT()?

 http://www.FreeRTOS.org/support - In return for receiving this top quality
 embedded software for free we request you assist our global community by
 participating in the support forum.

 http://www.FreeRTOS.org/training - Investing in training allows your team to
 be as productive as possible as early as possible.  Now you can receive
 FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
 Ltd, and the world's leading authority on the world's leading RTOS.

 http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
 including FreeRTOS+Trace - an indispensable productivity tool, a DOS
 compatible FAT file system, and our tiny thread aware UDP/IP stack.

 http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
 Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

 http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
 Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
 licenses offer ticketed support, indemnification and commercial middleware.

 http://www.SafeRTOS.com - High Integrity Systems also provide a safety
 engineered and independently SIL3 certified version for use in safety and
 mission critical applications that require provable dependability.

 1 tab == 4 spaces!
 */

/*
 FreeRTOS is a market leading RTOS from Real Time Engineers Ltd. that supports
 31 architectures and receives 77500 downloads a year. It is professionally
 developed, strictly quality controlled, robust, supported, and free to use in
 commercial products without any requirement to expose your proprietary source
 code.

 This simple FreeRTOS demo does not make use of any IO ports, so will execute on
 any Cortex-M3 of Cortex-M4 hardware.  Look for TODO markers in the code for
 locations that may require tailoring to, for example, include a manufacturer
 specific header file.

 This is a starter project, so only a subset of the RTOS features are
 demonstrated.  Ample source comments are provided, along with web links to
 relevant pages on the http://www.FreeRTOS.org site.

 Here is a description of the project's functionality:

 The main() Function:
 main() creates the tasks and software timers described in this section, before
 starting the scheduler.

 The Queue Send Task:
 The queue send task is implemented by the prvQueueSendTask() function.
 The task uses the FreeRTOS vTaskDelayUntil() and xQueueSend() API functions to
 periodically send the number 100 on a queue.  The period is set to 200ms.  See
 the comments in the function for more details.
 http://www.freertos.org/vtaskdelayuntil.html
 http://www.freertos.org/a00117.html

 The Queue Receive Task:
 The queue receive task is implemented by the prvQueueReceiveTask() function.
 The task uses the FreeRTOS xQueueReceive() API function to receive values from
 a queue.  The values received are those sent by the queue send task.  The queue
 receive task increments the ulCountOfItemsReceivedOnQueue variable each time it
 receives the value 100.  Therefore, as values are sent to the queue every 200ms,
 the value of ulCountOfItemsReceivedOnQueue will increase by 5 every second.
 http://www.freertos.org/a00118.html

 An example software timer:
 A software timer is created with an auto reloading period of 1000ms.  The
 timer's callback function increments the ulCountOfTimerCallbackExecutions
 variable each time it is called.  Therefore the value of
 ulCountOfTimerCallbackExecutions will count seconds.
 http://www.freertos.org/RTOS-software-timer.html

 The FreeRTOS RTOS tick hook (or callback) function:
 The tick hook function executes in the context of the FreeRTOS tick interrupt.
 The function 'gives' a semaphore every 500th time it executes.  The semaphore
 is used to synchronise with the event semaphore task, which is described next.

 The event semaphore task:
 The event semaphore task uses the FreeRTOS xSemaphoreTake() API function to
 wait for the semaphore that is given by the RTOS tick hook function.  The task
 increments the ulCountOfReceivedSemaphores variable each time the semaphore is
 received.  As the semaphore is given every 500ms (assuming a tick frequency of
 1KHz), the value of ulCountOfReceivedSemaphores will increase by 2 each second.

 The idle hook (or callback) function:
 The idle hook function queries the amount of free FreeRTOS heap space available.
 See vApplicationIdleHook().

 The malloc failed and stack overflow hook (or callback) functions:
 These two hook functions are provided as examples, but do not contain any
 functionality.
 */

/* Standard includes. */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <stdbool.h>
#include "stm32f4_discovery.h"
/* Kernel includes. */
#include "stm32f4xx.h"
#include "../FreeRTOS_Source/include/FreeRTOS.h"
#include "../FreeRTOS_Source/include/queue.h"
#include "../FreeRTOS_Source/include/semphr.h"
#include "../FreeRTOS_Source/include/task.h"
#include "../FreeRTOS_Source/include/timers.h"

/*-----------------------------------------------------------*/

#define mainQUEUE_LENGTH 					100

// Hardware defines
#define amber_led   						LED3
#define green_led   						LED4
#define red_led     						LED5
#define blue_led    						LED6

#define HYPER_PERIOD						1500

#define USER_DEFINED_TASK1_ID				1
#define USER_DEFINED_TASK2_ID				2
#define USER_DEFINED_TASK3_ID				3
#define USER_DEFINED_NON_PERIODIC_TASK_ID	4

typedef enum boolean
{
	FALSE = 0,
	TRUE
} boolean_t;

// Deadline-Driven task data structure
typedef enum task_type
{
	UNDEFINED,
	PERIODIC,
	NON_PERIODIC
} task_type_t;

typedef enum task_timer
{
	TASK_TIMER_1,
	TASK_TIMER_2,
	TASK_TIMER_3,
	NON_PERIODIC_TASK_TIMER
} task_timer_t;

typedef struct dd_task
{
	TaskHandle_t t_handle;
	task_type_t type;
	uint32_t task_id;
	uint32_t release_time;
	uint32_t completion_time;
	uint32_t absolute_deadline;
} dd_task_t;

typedef struct dd_task_list
{
	dd_task_t task;
	struct dd_task_list *next_task;
} dd_task_list_t;

typedef enum dd_message_type
{
	RELEASE_DD_TASK,
	DD_TASK_COMPLETED,
	GET_ACTIVE_DD_TASK_LIST,
	GET_COMPLETED_DD_TASK_LIST,
	GET_OVERDUE_DD_TASK_lIST
} dd_message_type_t;

typedef struct dd_message
{
	dd_message_type_t message_type;
	void* message_data;
} dd_message_t;

/*
 * TODO: Implement this function for any hardware specific clock configuration
 * that was not already performed before main() was called.
 */
static void prvSetupHardware(void);

dd_task_t *dd_task_allocate();
bool dd_task_free(dd_task_t *task_remove);

static void vPeriodicTask1GeneratorTimerCallBack(xTimerHandle xTimer);
static void vPeriodicTask2GeneratorTimerCallBack(xTimerHandle xTimer);
static void vPeriodicTask3GeneratorTimerCallBack(xTimerHandle xTimer);
static void vNonPeriodicTaskGeneratorTimerCallBack(xTimerHandle xTimer);

static void dd_generator_periodic_task_1(void *pvParameters);
static void dd_generator_periodic_task_2(void *pvParameters);
static void dd_generator_periodic_task_3(void *pvParameters);
static void dd_generator_non_periodic_task(void *pvParameters);

static void dd_user_defined_task_1(void *pvParameters);
static void dd_user_defined_task_2(void *pvParameters);
static void dd_user_defined_task_3(void *pvParameters);
static void dd_user_defined_non_periodic_task(void *pvParameters);

// functions declaration
void release_dd_task(TaskHandle_t t_handle, task_type_t type, uint32_t task_id, uint32_t absolute_deadline);
void dd_task_completed(uint32_t task_id);
dd_task_list_t **get_active_dd_task_list(void);
dd_task_list_t **get_complete_dd_task_list(void);
dd_task_list_t **get_overdue_dd_task_list(void);

void active_dd_task_list();
void complete_dd_task_list();
void overdue_dd_task_list();

// declare four tasks that share several resources that help complete the objectives
// of controlling the traffic light and car movement
static void dd_scheduler_task(void *pvParameters);

static void dd_system_monitor_task(void *pvParameters);
void dd_system_monitor_display_active_task_list(void);
void dd_system_monitor_display_completed_task_list(void);
void dd_system_monitor_display_overdue_task_list(void);

void EXTI0_IRQHandler(void);

// create new queue instances and return handles for
// a car generated, traffic flow rate, traffic light color, and traffic display
xQueueHandle dd_task_generator_message_queue;
xQueueHandle dd_scheduler_message_queue;
xQueueHandle dd_monitor_message_queue;

TaskHandle_t user_defined_periodic_task_1_handle = NULL;
TaskHandle_t user_defined_periodic_task_2_handle = NULL;
TaskHandle_t user_defined_periodic_task_3_handle = NULL;
TaskHandle_t user_defined_non_periodic_task_handle = NULL;

xTimerHandle xPeriodicTask1Timer;
xTimerHandle xPeriodicTask2Timer;
xTimerHandle xPeriodicTask3Timer;
xTimerHandle xNonPeriodicTaskTimer;

/*-----------------------------------------------------------*/

int main(void)
{
	/* call config functions to initialize system clock, GPIO, and ADC */

	/* Configure the system ready to run the demo.  The clock configuration
	 can be done here if it was not done before main() was called. */
	prvSetupHardware();

	/* Ensure all priority bits are assigned as preemption priority bits.
	    http://www.freertos.org/RTOS-Cortex-M3-M4.html */
	NVIC_SetPriorityGrouping(0);

    /* Initialize LEDs */
    STM_EVAL_LEDInit(amber_led);
    STM_EVAL_LEDInit(green_led);
    STM_EVAL_LEDInit(red_led);
    STM_EVAL_LEDInit(blue_led);

    // Initialize the pushbutton (either GPIO: BUTTON_MODE_GPIO or external interrupt: BUTTON_MODE_EXTI)
    STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_EXTI);
    NVIC_SetPriority(USER_BUTTON_EXTI_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1); // Must be above configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY

	// Create the queues used by the queue send and queue receive tasks.
	// queues have equal length of 100 elements
    dd_task_generator_message_queue = xQueueCreate(mainQUEUE_LENGTH, sizeof(task_timer_t));
	dd_scheduler_message_queue = xQueueCreate(mainQUEUE_LENGTH, sizeof(dd_message_t));
	dd_monitor_message_queue = xQueueCreate(mainQUEUE_LENGTH, sizeof(dd_message_t));

	/* Add to the registry, for the benefit of kernel aware debugging. */
	vQueueAddToRegistry(dd_task_generator_message_queue, "DDTaskGeneratorMessageQueue");
	vQueueAddToRegistry(dd_scheduler_message_queue, "DDSchedulerMessageQueue");
	vQueueAddToRegistry(dd_monitor_message_queue, "DDMonitorMessageQueue");

	// Deadline-Driven Scheduler (DDS): dynamically changing the priorities of user-defined FreeRTOS tasks
	// DDS uses an actively-managed list of:
	// -	Periodically-generated Deadline-Driven tasks
	// -	Non-periodically-generated Deadline-Driven tasks
	//
	// If a DD task needs to be scheduled and its deadline is the earliest, the DD scheduler will:
	// -	Set the priority of the FreeRTOS task referenced by the DD task to 'HIGH'
	// -	Set the priorities of other FreeRTOS tasks referenced by other DD tasks to 'LOW'
	//
	// Result: native FreeRTOS scheduler only executes a single user-defined FreeRTOS task with earliest DD-task deadline
	// due to higher priority over other user-defined FreeRTOS tasks
	xTaskCreate(dd_scheduler_task, "Deadline Driven Scheduler Task", configMINIMAL_STACK_SIZE, NULL, 2, NULL);

	// Periodically creates DD tasks to be scheduled by the DD scheduler

	// Deadline-Driven Task Generator is an auxiliary FreeRTOS task
	// -	periodically generated DD tasks
	// -	normally suspended
	// -	resumed when system timer callback function is trigger, so configure timer to expire based on time period of a particular DD task
	// -	prepared all info to create specific instances of DD tasks
	// -	called release_dd_task
	xTaskCreate(dd_generator_periodic_task_1, "DD Generator Task 1", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	xTaskCreate(dd_generator_periodic_task_2, "DD Generator Task 2", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	xTaskCreate(dd_generator_periodic_task_3, "DD Generator Task 3", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	xTaskCreate(dd_generator_non_periodic_task, "DD Generator Non Periodic Task", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

	// Tasks written by user that contain actual deadline-sensitive application code
	// User-Defined Task is an auxiliary FreeRTOS task contained:
	// -	actual deadline-sensitive application code executed at run-time
	// -	self-contained
	// - 	not rely on communication with other FreeRTOS tasks
	//
	// User-Defined Task:
	// - 	runs an empty loop for the duration of its execution time
	// - 	uses on-board LEDs to provide visual indication of which user task is currently executing
	// - 	calls complete_dd_task once it has finished
	xTaskCreate(dd_user_defined_task_1, "DD User-Defined Task 1", configMINIMAL_STACK_SIZE, NULL, 1, user_defined_periodic_task_1_handle);
	xTaskCreate(dd_user_defined_task_2, "DD User-Defined Task 2", configMINIMAL_STACK_SIZE, NULL, 1, user_defined_periodic_task_2_handle);
	xTaskCreate(dd_user_defined_task_3, "DD User-Defined Task 3", configMINIMAL_STACK_SIZE, NULL, 1, user_defined_periodic_task_3_handle);
	xTaskCreate(dd_user_defined_non_periodic_task, "DD User-Defined Non Periodic Task", configMINIMAL_STACK_SIZE, NULL, 1, user_defined_non_periodic_task_handle);

	// FreeRTOS task that extracts information from the DD scheduler and report scheduling information

	// Monitor Task is an auxiliary Task that reports:
	// 1.	number of active DD tasks
	// 2.	number of completed DD tasks
	// 3.	number of overdue DD tasks
	//
	// Monitor Task:
	// -	collects info from DD scheduler using:
	// 		a. get_active_dd_task_list
	// 		b. get_completed_dd_task_list
	// 		c. get_overdue_dd_task_list
	// -	report to users number of tasks on each list
	// -	execute even if active and overdue tasks are active to continue collecting data and reporting system info
	xTaskCreate(dd_system_monitor_task, "DD System Monitor Task", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

	// Create one-shot timers for the user-defined tasks and set the timer periods to expire after 1 tick
	xPeriodicTask1Timer = xTimerCreate("Task 1 timer", pdMS_TO_TICKS(1), pdFALSE, (void*)0, vPeriodicTask1GeneratorTimerCallBack);
	xPeriodicTask2Timer = xTimerCreate("Task 2 timer", pdMS_TO_TICKS(1), pdFALSE, (void*)0, vPeriodicTask2GeneratorTimerCallBack);
	xPeriodicTask3Timer = xTimerCreate("Task 3 timer", pdMS_TO_TICKS(1), pdFALSE, (void*)0, vPeriodicTask3GeneratorTimerCallBack);
	xNonPeriodicTaskTimer = xTimerCreate("Non-periodic Task timer", pdMS_TO_TICKS(1), pdFALSE, (void*)0, vNonPeriodicTaskGeneratorTimerCallBack);

	/* Start the tasks and timer running. */
	vTaskStartScheduler();

	return 0;
}

dd_task_t *create_and_initialize_new_dd_task()
{
	dd_task_t *new_task = (dd_task_t*)pvPortMalloc(sizeof(dd_task_t));

	new_task->t_handle = NULL;
	new_task->type = UNDEFINED;
	new_task->task_id = 0;
	new_task->release_time = 0;
	new_task->completion_time = 0;
	new_task->absolute_deadline = 0;

	return new_task;
}

bool free_dd_task(dd_task_t *task_remove)
{
	task_remove->t_handle = NULL;
	task_remove->type = UNDEFINED;
	task_remove->task_id = 0;
	task_remove->release_time = 0;
	task_remove->completion_time = 0;
	task_remove->absolute_deadline = 0;

	vPortFree((void*)task_remove);

	return true;
}

static void vPeriodicTask1GeneratorTimerCallBack(xTimerHandle xTimer)
{
	task_timer_t timer_is_done = TASK_TIMER_1;

	xQueueSend(dd_task_generator_message_queue, &timer_is_done, pdMS_TO_TICKS(0));
}

static void vPeriodicTask2GeneratorTimerCallBack(xTimerHandle xTimer)
{
	task_timer_t timer_is_done = TASK_TIMER_2;

	xQueueSend(dd_task_generator_message_queue, &timer_is_done, pdMS_TO_TICKS(0));
}

static void vPeriodicTask3GeneratorTimerCallBack(xTimerHandle xTimer)
{
	task_timer_t timer_is_done = TASK_TIMER_3;

	xQueueSend(dd_task_generator_message_queue, &timer_is_done, pdMS_TO_TICKS(0));
}

static void vNonPeriodicTaskGeneratorTimerCallBack(xTimerHandle xTimer)
{
	task_timer_t timer_is_done = NON_PERIODIC_TASK_TIMER;

	xQueueSend(dd_task_generator_message_queue, &timer_is_done, pdMS_TO_TICKS(0));
}

// Execute the dd generator task 1 task
static void dd_generator_periodic_task_1(void *pvParameters)
{
	task_timer_t timer_is_done = TASK_TIMER_1;
	dd_task_t *generated_new_task_1;
	TickType_t release_time_task_1;
	uint32_t absolute_deadline_task_1 = 0;
	uint32_t relative_deadline_task_1 = 0;

	while(1)
	{
		release_time_task_1 = xTaskGetTickCount();
		relative_deadline_task_1 = 500; // relative deadline is the period
		absolute_deadline_task_1 = (uint32_t)release_time_task_1 + relative_deadline_task_1;

		if(xQueueReceive(dd_task_generator_message_queue, &timer_is_done, pdMS_TO_TICKS(0)) == pdTRUE)
		{
			generated_new_task_1 = create_and_initialize_new_dd_task();
			generated_new_task_1->t_handle = user_defined_periodic_task_1_handle;
			generated_new_task_1->task_id = USER_DEFINED_TASK1_ID;
			generated_new_task_1->type = PERIODIC;
			generated_new_task_1->release_time = release_time_task_1;
			generated_new_task_1->absolute_deadline = absolute_deadline_task_1;
			release_dd_task(user_defined_periodic_task_1_handle, PERIODIC, USER_DEFINED_TASK1_ID, absolute_deadline_task_1);
		}

		vTaskDelay(pdMS_TO_TICKS(HYPER_PERIOD));
	}
}

// Execute the dd generator task 2
static void dd_generator_periodic_task_2(void *pvParameters)
{
	task_timer_t timer_is_done = TASK_TIMER_2;
	dd_task_t *generated_new_task_2;
	TickType_t release_time_task_2;
	uint32_t absolute_deadline_task_2 = 0;
	uint32_t relative_deadline_task_2 = 0;

	while(1)
	{
		release_time_task_2 = xTaskGetTickCount();
		relative_deadline_task_2 = 500;
		absolute_deadline_task_2 = (uint32_t)release_time_task_2 + relative_deadline_task_2;

		if(xQueueReceive(dd_task_generator_message_queue, &timer_is_done, pdMS_TO_TICKS(0)) == pdTRUE)
		{
			generated_new_task_2 = create_and_initialize_new_dd_task();
			generated_new_task_2->t_handle = user_defined_periodic_task_2_handle;
			generated_new_task_2->task_id = USER_DEFINED_TASK2_ID;
			generated_new_task_2->type = PERIODIC;
			generated_new_task_2->release_time = release_time_task_2;
			generated_new_task_2->absolute_deadline = absolute_deadline_task_2;
			release_dd_task(user_defined_periodic_task_2_handle, PERIODIC, USER_DEFINED_TASK2_ID, absolute_deadline_task_2);
		}

		vTaskDelay(pdMS_TO_TICKS(HYPER_PERIOD));
	}
}

// Execute dd generator task 3
static void dd_generator_periodic_task_3(void *pvParameters)
{
	task_timer_t timer_is_done = TASK_TIMER_3;
	dd_task_t *generated_new_task_3;
	TickType_t release_time_task_3;
	uint32_t absolute_deadline_task_3 = 0;
	uint32_t relative_deadline_task_3 = 0;

	while(1)
	{
		release_time_task_3 = xTaskGetTickCount();
		relative_deadline_task_3 = 750;
		absolute_deadline_task_3 = (uint32_t)release_time_task_3 + relative_deadline_task_3;

		if(xQueueReceive(dd_task_generator_message_queue, &timer_is_done, pdMS_TO_TICKS(0)) == pdTRUE)
		{
			generated_new_task_3 = create_and_initialize_new_dd_task();
			generated_new_task_3->t_handle = user_defined_periodic_task_3_handle;
			generated_new_task_3->task_id = USER_DEFINED_TASK3_ID;
			generated_new_task_3->type = PERIODIC;
			generated_new_task_3->release_time = release_time_task_3;
			generated_new_task_3->absolute_deadline = absolute_deadline_task_3;
			release_dd_task(user_defined_periodic_task_3_handle, PERIODIC, USER_DEFINED_TASK3_ID, absolute_deadline_task_3);
		}

		vTaskDelay(pdMS_TO_TICKS(HYPER_PERIOD));
	}
}

static void dd_generator_non_periodic_task(void *pvParameters)
{
	task_timer_t timer_is_done = NON_PERIODIC_TASK_TIMER;
	dd_task_t *generated_new_non_periodic_task;
	TickType_t release_time_non_periodic_task;
	uint32_t relative_deadline_non_periodic_task = 900;
	uint32_t absolute_deadline_non_periodic_task = 0;

	while(1)
	{
		ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
		release_time_non_periodic_task = xTaskGetTickCount();
		absolute_deadline_non_periodic_task = (uint32_t)release_time_non_periodic_task + relative_deadline_non_periodic_task;

		if(xQueueReceive(dd_task_generator_message_queue, &timer_is_done, pdMS_TO_TICKS(0)) == pdTRUE)
		{
			generated_new_non_periodic_task = create_and_initialize_new_dd_task();
			generated_new_non_periodic_task->t_handle = user_defined_non_periodic_task_handle;
			generated_new_non_periodic_task->task_id = USER_DEFINED_NON_PERIODIC_TASK_ID;
			generated_new_non_periodic_task->type = NON_PERIODIC;
			generated_new_non_periodic_task->release_time = release_time_non_periodic_task;
			generated_new_non_periodic_task->absolute_deadline = absolute_deadline_non_periodic_task;
			release_dd_task(user_defined_non_periodic_task_handle, NON_PERIODIC, USER_DEFINED_NON_PERIODIC_TASK_ID, absolute_deadline_non_periodic_task);
		}

		vTaskDelay(pdMS_TO_TICKS(HYPER_PERIOD));
	}
}

// Execute dd user-defined task 1
static void dd_user_defined_task_1(void *pvParameters)
{
	dd_task_t *user_task_1 = (dd_task_t*)pvParameters;

	TickType_t current_time = 0;
	TickType_t previous_tick = 0;
	TickType_t execution_time = 95/portTICK_PERIOD_MS;
	TickType_t relative_deadline = 0;
	TickType_t release_time = 0;
	uint32_t counter = 0;
	uint32_t user_task_1_id = USER_DEFINED_TASK1_ID;

	while(1)
	{
		release_time = xTaskGetTickCount();
		previous_tick = release_time; // what is this?
		current_time = release_time;
		counter = 0;

		STM_EVAL_LEDToggle(amber_led);

		while(counter < execution_time)
		{
			current_time = xTaskGetTickCount();

			if(current_time != previous_tick)
			{
				if(current_time % 2 == 0)
				{
					STM_EVAL_LEDToggle(amber_led);
				}
			}

			previous_tick = current_time;
			counter++;
		}

		STM_EVAL_LEDOff(amber_led);
		relative_deadline = user_task_1->absolute_deadline - current_time;
		vTaskDelayUntil(&current_time, relative_deadline);
		dd_task_completed(user_task_1_id);
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		free_dd_task(user_task_1);
	}
}

// Execute dd user-defined task 1
static void dd_user_defined_task_2(void *pvParameters)
{
	dd_task_t *user_task_2 = (dd_task_t*)pvParameters;

	TickType_t current_time = 0;
	TickType_t previous_tick = 0;
	TickType_t execution_time = 150/portTICK_PERIOD_MS;
	TickType_t relative_deadline = 0;
	TickType_t release_time = 0;
	uint32_t counter = 0;
	uint32_t user_task_2_id = USER_DEFINED_TASK2_ID;

	while(1)
	{
		release_time = xTaskGetTickCount();
		previous_tick = release_time; // what is this?
		current_time = release_time;
		counter = 0;

		STM_EVAL_LEDToggle(green_led);

		while(counter < execution_time)
		{
			current_time = xTaskGetTickCount();

			if(current_time != previous_tick)
			{
				if(current_time % 2 == 0)
				{
					STM_EVAL_LEDToggle(green_led);
				}
			}

			previous_tick = current_time;
			counter++;
		}

		STM_EVAL_LEDOff(green_led);
		relative_deadline = user_task_2->absolute_deadline - current_time;
		vTaskDelayUntil(&current_time, relative_deadline);
		dd_task_completed(user_task_2_id);
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		free_dd_task(user_task_2);
	}
}

// Execute dd user-defined task 1
void dd_user_defined_task_3(void *pvParameters)
{
	dd_task_t *user_task_3 = (dd_task_t*)pvParameters;

	TickType_t current_time = 0;
	TickType_t previous_tick = 0;
	TickType_t execution_time = 250/portTICK_PERIOD_MS;
	TickType_t relative_deadline = 0;
	TickType_t release_time = 0;
	uint32_t counter = 0;
	uint32_t user_task_3_id = USER_DEFINED_TASK3_ID;

	while(1)
	{
		release_time = xTaskGetTickCount();
		previous_tick = release_time; // what is this?
		current_time = release_time;
		counter = 0;

		STM_EVAL_LEDToggle(blue_led);

		while(counter < execution_time)
		{
			current_time = xTaskGetTickCount();

			if(current_time != previous_tick)
			{
				if(current_time % 2 == 0)
				{
					STM_EVAL_LEDToggle(blue_led);
				}
			}

			previous_tick = current_time;
			counter++;
		}

		STM_EVAL_LEDOff(blue_led);
		relative_deadline = user_task_3->absolute_deadline - current_time;
		vTaskDelayUntil(&current_time, relative_deadline);
		dd_task_completed(user_task_3_id);
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		free_dd_task(user_task_3);
	}
}

static void dd_user_defined_non_periodic_task(void *pvParameters)
{
	dd_task_t *user_non_periodic_task = (dd_task_t*)pvParameters;

	TickType_t current_time = 0;
	TickType_t previous_tick = 0;
	TickType_t execution_time = 150/portTICK_PERIOD_MS;
	TickType_t relative_deadline = 0;
	TickType_t release_time = 0;
	uint32_t counter = 0;
	uint32_t user_non_periodic_task_id = USER_DEFINED_NON_PERIODIC_TASK_ID;

	while(1)
	{
		release_time = xTaskGetTickCount();
		previous_tick = release_time; // what is this?
		current_time = release_time;
		counter = 0;

		STM_EVAL_LEDToggle(red_led);

		while(counter < execution_time)
		{
			current_time = xTaskGetTickCount();

			if(current_time != previous_tick)
			{
				if(current_time % 2 == 0)
				{
					STM_EVAL_LEDToggle(red_led);
				}
			}

			previous_tick = current_time;
			counter++;
		}

		STM_EVAL_LEDOff(red_led);
		relative_deadline = user_non_periodic_task->absolute_deadline - current_time;
		vTaskDelayUntil(&current_time, relative_deadline);
		dd_task_completed(user_non_periodic_task_id);
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		free_dd_task(user_non_periodic_task);
	}
}

// Push Button Interrupt Handler
void EXTI0_IRQHandler(void)
{
    /* Make sure that interrupt flag is set */
    if (EXTI_GetITStatus(EXTI_Line0) != RESET)
    {
        // Switch an LED
        //STM_EVAL_LEDToggle(green_led);
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        /* Notify the task that the transmission is complete. */
        vTaskNotifyGiveFromISR(user_defined_non_periodic_task_handle, &xHigherPriorityTaskWoken);

        /* Clear interrupt flag (Want to do this as late as possible to avoid triggering the IRQ in the IRQ) */
        EXTI_ClearITPendingBit(EXTI_Line0);

        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    }
}

// release_dd_task
// -	receives all info to create a new dd_task struct excluding release time and completion time
// -	package dd_task struct as a message and send to a queue (xQueueSend(dd_task))
// DD Scheduler receives this message from the queue (xQueueReceive(dd_task))
void release_dd_task(TaskHandle_t t_handle, task_type_t type, uint32_t task_id, uint32_t absolute_deadline)
{
	dd_task_t received_dd_task_info;

	received_dd_task_info.t_handle = t_handle;
	received_dd_task_info.type = type;
	received_dd_task_info.task_id = task_id;
	received_dd_task_info.absolute_deadline = absolute_deadline;
	xQueueSend(dd_scheduler_message_queue, &received_dd_task_info, pdMS_TO_TICKS(0));
}

// complete_dd_task
// - 	receive task ID of DD task that has completed execution
// - 	package task ID as a message and send to a queue (xQueueSend(task ID))
// DD Scheduler receives this message from the queue (xQueueReceive(task ID))
void dd_task_completed(uint32_t task_id)
{
	dd_task_t received_dd_task_id;

	received_dd_task_id.task_id = task_id;
	xQueueSend(dd_scheduler_message_queue, &received_dd_task_id, pdMS_TO_TICKS(0));
}

// get_active_dd_task_list
// -	send a message to a queue requesting Active Task List from DD scheduler
// Once DD Scheduler responds, get_active_dd_task_list function returns the list
dd_task_list_t **get_active_dd_task_list(void)
{
	dd_message_t request_active_dd_task_list = {GET_ACTIVE_DD_TASK_LIST, NULL};

	xQueueSend(dd_scheduler_message_queue, &request_active_dd_task_list, pdMS_TO_TICKS(0));

	return 0;
}

// get_completed_dd_task_list
// -	send a message to a queue requesting Completed Task List from DD scheduler
// Once DD Scheduler responds, get_completed_dd_task_list function returns the list
dd_task_list_t **get_completed_dd_task_list(void)
{
	dd_message_t request_complete_dd_task_list = {GET_COMPLETED_DD_TASK_LIST, NULL};

	xQueueSend(dd_scheduler_message_queue, &request_complete_dd_task_list, pdMS_TO_TICKS(0));

	return 0;
}

// get_overdue_dd_task_list
// send a message to a queue requesting Overdue Task List from DD scheduler
// Once DD Scheduler responds, get_overdue_dd_task_list function returns the list
dd_task_list_t **get_overdue_dd_task_list(void)
{
	dd_message_t request_overdue_dd_task_list = {GET_OVERDUE_DD_TASK_lIST, NULL};

	xQueueSend(dd_scheduler_message_queue, &request_overdue_dd_task_list, pdMS_TO_TICKS(0));

	return 0;
}

// create_active_dd_task_list
// List of DD tasks to be scheduled by the DD scheduler
// -	Sort the list by deadline every time a DD task is added or removed from the list
// -	Select and implement a data structure and sorting algorithm to sort the list
// -	Use singly-linked list for sorting
void active_dd_task_list(dd_task_t active_task_t, dd_task_list_t active_task_list_t)
{

}

// create_complete_dd_task_list
// List of DD tasks that have completed execution before their deadlines
// -	Remove completed tasks before their deadline from Active Task List
// -	Add these completed tasks to Completed Task List
void complete_dd_task_list()
{

}

// create_overdue_dd_task_list
// List of DD tasks that missed their deadlines
// -	Remove tasks missed their deadline from Active Task List
// -	Add these missed deadline tasks to Overdue Task List
void overdue_dd_task_list()
{

}

// Execute deadline-driven scheduler task
// 1.	Implements EDF algorithm
// 2.	Control the priorities of users-define FreeRTOS tasks from an actively-managed list of DD tasks
void dd_scheduler_task(void *pvParameters)
{
	dd_message_t dd_task_list_message;
	dd_task_t dd_task_released_message;
	dd_task_t dd_task_completed_message;

	while(1)
	{
		// Message from release_dd_task
		//	DD scheduler:
		// -	assigns release time for new task
		// -	add DD task to Active task List
		// -	sort the list by deadline
		// -	set priorities of the User-Defined tasks
		if((xQueueReceive(dd_scheduler_message_queue, (void*)&dd_task_released_message, pdMS_TO_TICKS(0))) == pdTRUE)
		{

		}

		// Message from complete_dd_task
		// DD scheduler:
		// -	assign completion time to newly-completed DD task
		// -	remove DD task from Active Task List
		// -	add it to the Completed Task List
		// -	sort Active Task List by deadline
		// -	set priorities of the User-Define tasks
		if((xQueueReceive(dd_scheduler_message_queue, &dd_task_completed_message, pdMS_TO_TICKS(0))) == pdTRUE)
		{

		}

		if((xQueueReceive(dd_scheduler_message_queue, &dd_task_list_message, pdMS_TO_TICKS(0))) == pdTRUE)
		{
			switch(dd_task_list_message.message_type)
			{
				// Message from get_active_dd_task_list
				// DD scheduler	sends Active Task List to a queue
				case GET_ACTIVE_DD_TASK_LIST:

					break;

				// Message from get_completed_dd_task_list
				// DD scheduler	sends Completed Task List to a queue
				case GET_COMPLETED_DD_TASK_LIST:

					break;

				// Message from get_overdue_dd_task_list
				// DD scheduler	sends Overdue Task List to a queue
				case GET_OVERDUE_DD_TASK_lIST:

					break;

				default:
					break;
			}
		}

		vTaskDelay(pdMS_TO_TICKS(500));
	}
}



// This task displays traffic light system and car on the road by updating the LEDs using the shift register
void dd_system_monitor_task(void *pvParameters)
{
	while(1)
	{
		dd_system_monitor_display_active_task_list();
		dd_system_monitor_display_completed_task_list();
		dd_system_monitor_display_overdue_task_list();
		vTaskDelay(pdMS_TO_TICKS(490));
	}
}

void dd_system_monitor_display_active_task_list(void)
{
	dd_message_t active_task_list_message;

	get_active_dd_task_list();

	if(xQueueReceive(dd_monitor_message_queue, &active_task_list_message, portMAX_DELAY) == pdTRUE)
	{
		printf("Active Tasks: \n%s\n", (char*)(active_task_list_message.message_data));
	}
}

void dd_system_monitor_display_completed_task_list(void)
{
	dd_message_t completed_task_list_message;

	get_completed_dd_task_list();

	if(xQueueReceive(dd_monitor_message_queue, &completed_task_list_message, portMAX_DELAY) == pdTRUE)
	{
		printf("Active Tasks: \n%s\n", (char*)(completed_task_list_message.message_data));
	}
}

void dd_system_monitor_display_overdue_task_list(void)
{
	dd_message_t overdue_task_list_message;

	get_overdue_dd_task_list();

	if(xQueueReceive(dd_monitor_message_queue, &overdue_task_list_message, portMAX_DELAY) == pdTRUE)
	{
		printf("Active Tasks: \n%s\n", (char*)(overdue_task_list_message.message_data));
	}
}

void vApplicationMallocFailedHook(void)
{
	/* The malloc failed hook is enabled by setting
	 configUSE_MALLOC_FAILED_HOOK to 1 in FreeRTOSConfig.h.

	 Called if a call to pvPortMalloc() fails because there is insufficient
	 free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	 internally by FreeRTOS API functions that create tasks, queues, software
	 timers, and semaphores.  The size of the FreeRTOS heap is set by the
	 configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */
	for (;;);
}


void vApplicationStackOverflowHook(xTaskHandle pxTask, signed char *pcTaskName)
{
	(void) pcTaskName;
	(void) pxTask;

	/* Run time stack overflow checking is performed if
	 configconfigCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	 function is called if a stack overflow is detected.  pxCurrentTCB can be
	 inspected in the debugger if the task name passed into this function is
	 corrupt. */
	for (;;);
}

void vApplicationIdleHook(void)
{
	volatile size_t xFreeStackSpace;

	/* The idle task hook is enabled by setting configUSE_IDLE_HOOK to 1 in
	 FreeRTOSConfig.h.

	 This function is called on each cycle of the idle task.  In this case it
	 does nothing useful, other than report the amount of FreeRTOS heap that
	 remains unallocated. */
	xFreeStackSpace = xPortGetFreeHeapSize();

	if (xFreeStackSpace > 100)
	{
		/* By now, the kernel has allocated everything it is going to, so
		 if there is a lot of heap remaining unallocated then
		 the value of configTOTAL_HEAP_SIZE in FreeRTOSConfig.h can be
		 reduced accordingly. */
	}
}

static void prvSetupHardware(void)
{
	/* Ensure all priority bits are assigned as preemption priority bits.
	 http://www.freertos.org/RTOS-Cortex-M3-M4.html */
	NVIC_SetPriorityGrouping(0);

	/* TODO: Setup the clocks, etc. here, if they were not configured before
	 main() was called. */
}
