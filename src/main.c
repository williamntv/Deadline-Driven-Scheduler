/* Standard includes. */
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

/* STM/RTOS includes. */
#include "stm32f4_discovery.h"

/* Kernel includes. */
#include "stm32f4xx.h"
#include "../FreeRTOS_Source/include/FreeRTOS.h"
#include "../FreeRTOS_Source/include/queue.h"
#include "../FreeRTOS_Source/include/semphr.h"
#include "../FreeRTOS_Source/include/task.h"
#include "../FreeRTOS_Source/include/timers.h"

/*-----------------------------------------------------------*/
// Hardware defines
#define amber_led   						LED3
#define green_led   						LED4
#define red_led     						LED5
#define blue_led    						LED6

#define HYPER_PERIOD						15000

# define TASK_LOWEST_PRIORITY      			1
# define TASK_MONITOR_PRIORITY				2
# define TASK_EXECUTION_PRIORITY 			3
# define TASK_GENERATOR_PRIORITY      		4
# define TASK_SCHEDULER_PRIORITY      		5

#define schedulerQUEUE_LENGTH				20
#define monitorQUEUE_LENGTH 				3
#define taskgeneratorQUEUE_LENGTH			3
#define taskQUEUE_LENGTH					1

#define TASK1_ID							1
#define TASK2_ID							2
#define TASK3_ID							3
#define APERIODIC_TASK_ID					4

#define TASK_1_EXECUTION_TIME				950
#define TASK_2_EXECUTION_TIME				1500
#define TASK_3_EXECUTION_TIME				2500
#define APERIODIC_TASK_EXECUTION_TIME		5000

#define TASK_1_PERIOD						9000
#define TASK_2_PERIOD						5000
#define TASK_3_PERIOD						7500
#define APERIODIC_TASK_PERIOD				3000

#define	TASK_1_TIMER						1
#define	TASK_2_TIMER						2
#define	TASK_3_TIMER						3
#define	APERIODIC_TASK_TIMER				4

// Deadline-Driven task data structure
typedef enum task_type
{
	UNDEFINED,
	PERIODIC,
	APERIODIC
} task_type_t;

typedef struct dd_task_info
{
	TaskHandle_t task_handle;
	TimerHandle_t timer_handle;
	task_type_t type;
	uint32_t task_id;
	uint32_t release_time;
	uint32_t completion_time;
	uint32_t overdue_time;
	uint32_t absolute_deadline;
} dd_task_info_t;

typedef struct dd_task_node
{
	dd_task_info_t *pnode;
	struct dd_task_node *pnext_node;
} dd_task_node_t;

dd_task_node_t *pActive_list_head = NULL;
dd_task_node_t *pCompletion_list_head = NULL;
dd_task_node_t *pOverdue_list_head = NULL;

typedef enum dd_message_type
{
	RELEASE_TASK = 0,
	COMPLETED_TASK,
	GET_ACTIVE_DD_TASK_LIST,
	GET_COMPLETED_DD_TASK_LIST,
	GET_OVERDUE_DD_TASK_lIST
} dd_message_type_t;

typedef struct dd_message
{
	dd_message_type_t message_type;
	dd_task_info_t *ptask_info;
	dd_task_node_t *ptask_list;
} dd_message_t;

/*
 * TODO: Implement this function for any hardware specific clock configuration
 * that was not already performed before main() was called.
 */
static void prvSetupHardware(void);

static void dd_task_scheduler(void *pvParameters);
static void dd_task_monitor(void *pvParameters);

static void dd_task_generator_1(void *pvParameters);
static void dd_task_generator_2(void *pvParameters);
static void dd_task_generator_3(void *pvParameters);

static void vTaskTimerCallBack(xTimerHandle xTimer);

static void dd_user_defined_task_1(void *pvParameters);
static void dd_user_defined_task_2(void *pvParameters);
static void dd_user_defined_task_3(void *pvParameters);

// functions declaration
dd_task_info_t *pCreate_dd_task_info(TaskHandle_t task_handle, task_type_t type, uint32_t task_id, uint32_t absolute_deadline);
void delete_dd_task_info(dd_task_info_t *ptask_info);
void release_dd_task_info(dd_task_info_t *ptask_info);
void dd_task_completed(dd_task_info_t *ptask_info);
dd_task_node_t **pGetActiveDDTaskList(void);
dd_task_node_t **pGetCompletedDDTaskList(void);
dd_task_node_t **pGetOverdueDDTaskList(void);

dd_task_node_t *insert_new_node_to_active_list(dd_task_info_t *ptask_info);
dd_task_node_t *insert_new_node_to_completed_list(dd_task_info_t *ptask_info);
dd_task_node_t *insert_new_node_to_overdue_list(dd_task_info_t *ptask_info);
uint32_t active_list_length();
void sort_active_list_by_deadline(dd_task_info_t *ptask_info);
dd_task_node_t *pFind_completed_task_node_by_time_stamp(dd_task_info_t *ptask_info);
//dd_task_node_t *pFind_completed_task_node_by_time_stamp(uint32_t time_stamp);
dd_task_node_t *pFind_overdue_task_node_using_time_stamp(uint32_t time_stamp);
dd_task_node_t *pRemove_completed_task_node_by_time_stamp(dd_task_info_t *ptask_info);
//dd_task_node_t *pRemove_overdue_task_node_by_time_stamp(uint32_t time_stamp);
//dd_task_node_t *pRemove_completed_task_node_by_time_stamp(uint32_t time_stamp);
void printActiveList();
void printCompletedList();
void printOverdueList();

void EXTI0_IRQHandler(void);

//QueueHandle_t dd_task_message_queue;
QueueHandle_t dd_scheduler_message_queue;
QueueHandle_t dd_monitor_message_queue;

TaskHandle_t dd_task_generator_1_handle = NULL;
TaskHandle_t dd_task_generator_2_handle = NULL;
TaskHandle_t dd_task_generator_3_handle = NULL;

//TaskHandle_t dd_aperiodic_task_generator_handle = NULL;
//TaskHandle_t user_defined_aperiodic_task_handle = NULL;
//TimerHandle_t xAperiodicTimer;
//static void dd_aperiodic_task_generator(void *pvParameters);
//static void dd_user_defined_aperiodic_task(void *pvParameters);
//static void vAperiodicTaskTimerCallBack(xTimerHandle xTimer);

/*-----------------------------------------------------------*/
int main(void)
{
	// Configure the system ready to run the demo.  The clock configuration
	// can be done here if it was not done before main() was called
	prvSetupHardware();

	// Ensure all priority bits are assigned as preemption priority bits.
	// http://www.freertos.org/RTOS-Cortex-M3-M4.html
	NVIC_SetPriorityGrouping(0);

	/* Initialize LEDs */
	STM_EVAL_LEDInit(amber_led);
	STM_EVAL_LEDInit(green_led);
	STM_EVAL_LEDInit(red_led);
	STM_EVAL_LEDInit(blue_led);

	// Initialize the pushbutton (either GPIO: BUTTON_MODE_GPIO or external interrupt: BUTTON_MODE_EXTI)
	STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_EXTI);
	NVIC_SetPriority(USER_BUTTON_EXTI_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1); // Must be above configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY

	printf("Initialize message queue\n\n");

	// Create the queues used by the queue send and queue receive tasks.
	dd_scheduler_message_queue = xQueueCreate(schedulerQUEUE_LENGTH, sizeof(dd_message_t));
	dd_monitor_message_queue = xQueueCreate(monitorQUEUE_LENGTH, sizeof(dd_message_t));

	// Add to the registry, for the benefit of kernel aware debugging.
	vQueueAddToRegistry(dd_scheduler_message_queue, "DDSchedulerMessageQueue");
	vQueueAddToRegistry(dd_monitor_message_queue, "DDMonitorMessageQueue");

	xTaskCreate(dd_task_scheduler, "DDTaskScheduler", configMINIMAL_STACK_SIZE, NULL, TASK_SCHEDULER_PRIORITY, NULL);
	xTaskCreate(dd_task_monitor, "DDTaskMonitor", configMINIMAL_STACK_SIZE, NULL, TASK_MONITOR_PRIORITY, NULL);

	xTaskCreate(dd_task_generator_1, "DDTaskGenerator1", configMINIMAL_STACK_SIZE, NULL, TASK_GENERATOR_PRIORITY, &dd_task_generator_1_handle);
	xTaskCreate(dd_task_generator_2, "DDTaskGenerator2", configMINIMAL_STACK_SIZE, NULL, TASK_GENERATOR_PRIORITY, &dd_task_generator_2_handle);
	xTaskCreate(dd_task_generator_3, "DDTaskGenerator3", configMINIMAL_STACK_SIZE, NULL, TASK_GENERATOR_PRIORITY, &dd_task_generator_3_handle);
//	//xTaskCreate(dd_aperiodic_task_generator, "DDAperiodicTaskGenerator", configMINIMAL_STACK_SIZE, NULL, DD_TASK_GENERATOR_PRIORITY, &dd_aperiodic_task_generator_handle);

	printf("Done initialized message queue\n\n");

	/* Start the tasks and timer running. */
	vTaskStartScheduler();

	return 0;
}

dd_task_info_t *pCreate_dd_task_info(TaskHandle_t task_handle, task_type_t type, uint32_t task_id, uint32_t absolute_deadline)
{
	dd_task_info_t *ptask_info;

	ptask_info = (dd_task_info_t*)pvPortMalloc(sizeof(dd_task_info_t));

	if(ptask_info == NULL)
	{
		printf("pCreate_dd_task_info: Error no memory!\n");
	}

	ptask_info->task_handle = task_handle;
	ptask_info->type = type;
	ptask_info->task_id = task_id;
	ptask_info->absolute_deadline = absolute_deadline;

	return ptask_info;
}

void delete_dd_task_info(dd_task_info_t *ptask_info)
{
	vPortFree((void *)ptask_info);
}

// release_dd_task
// -	receives all info to create a new dd_task struct excluding release time and completion time
// -	packages dd_task struct as a message and send to a queue (xQueueSend(dd_task))
// DD Scheduler receives this message from the queue (xQueueReceive(dd_task))
void release_dd_task_info(dd_task_info_t *ptask_info)
{
	dd_message_t scheduler_message;

	scheduler_message.message_type = RELEASE_TASK;
	scheduler_message.ptask_info = ptask_info;
	xQueueSend(dd_scheduler_message_queue, (void *)&scheduler_message, portMAX_DELAY);
}

// complete_dd_task
// - 	receive task ID of DD task that has completed execution
// - 	package task ID as a message and send to a queue (xQueueSend(task ID))
// DD Scheduler receives this message from the queue (xQueueReceive(task ID))
void dd_task_completed(dd_task_info_t *ptask_info)
{
	dd_message_t scheduler_message;

	scheduler_message.message_type = COMPLETED_TASK;
	scheduler_message.ptask_info = ptask_info;
	xQueueSend(dd_scheduler_message_queue, (void *)&scheduler_message, portMAX_DELAY);
}

// Execute the dd generator task 1 task
static void dd_task_generator_1(void *pvParameters)
{
	printf("dd_task_generator_1\n");
	const TickType_t xGeneratorDelay1 = TASK_1_PERIOD;
	TickType_t current_time;

	while(1)
	{
		//TaskHandle_t user_defined_task_1_handle = NULL;
		dd_task_info_t *ptask_info_1 = NULL;
		current_time = xTaskGetTickCount();
		//dd_message_t scheduler_message;
		ptask_info_1 = pCreate_dd_task_info(NULL, PERIODIC, TASK1_ID, (current_time + xGeneratorDelay1));
		xTaskCreate(dd_user_defined_task_1, "DDUserDefinedTask1", configMINIMAL_STACK_SIZE, (void*)ptask_info_1, TASK_LOWEST_PRIORITY, &(ptask_info_1->task_handle));
		vTaskSuspend(ptask_info_1->task_handle);
		//BaseType_t returnTaskValue = xTaskCreate(dd_user_defined_task_1, "DDUserDefinedTask1", configMINIMAL_STACK_SIZE, (void*)ptask_info_1, TASK_LOWEST_PRIORITY, &(ptask_info_1->task_handle));
		//printf("dd_task_generator_1 handle, return task value %d\n", returnTaskValue);
		//printf("dd_task_generator_1 handle 2\n");
		printf("dd_task_generator_1 handle = 0x%x: released task!\n", ptask_info_1->task_handle);
		//scheduler_message.message_type = RELEASE_TASK;
		//scheduler_message.message_data = ptask_info_1;
		release_dd_task_info(ptask_info_1);
		//ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		//vTaskResume(ptask_info_1->task_handle);
		vTaskDelay(xGeneratorDelay1);
		//printf("dd_task_generator_1 handle after delay\n");
	}
}

// Execute the dd generator task 2
static void dd_task_generator_2(void *pvParameters)
{
	printf("dd_task_generator_2\n");
	const TickType_t xGeneratorDelay2 = TASK_2_PERIOD;
	TickType_t current_time;

	while(1)
	{
		dd_task_info_t *ptask_info_2 = NULL;
		current_time = xTaskGetTickCount();
		ptask_info_2 = pCreate_dd_task_info(NULL, PERIODIC, TASK2_ID, (current_time + xGeneratorDelay2));
		xTaskCreate(dd_user_defined_task_2,	"DDUserDefinedTask2", configMINIMAL_STACK_SIZE,	(void*)ptask_info_2, TASK_LOWEST_PRIORITY, &(ptask_info_2->task_handle));
		vTaskSuspend(ptask_info_2->task_handle);
		printf("dd_task_generator_2 handle = 0x%x: released task!\n", ptask_info_2->task_handle);
		release_dd_task_info(ptask_info_2);
		vTaskDelay(xGeneratorDelay2);
	}
}

// Execute dd generator task 3
static void dd_task_generator_3(void *pvParameters)
{
	printf("dd_task_generator_3\n");
	const TickType_t xGeneratorDelay3 = TASK_3_PERIOD;
	TickType_t current_time;

	while(1)
	{
		dd_task_info_t *ptask_info_3 = NULL;
		current_time = xTaskGetTickCount();
		ptask_info_3 = pCreate_dd_task_info(NULL, PERIODIC, TASK3_ID, (current_time + xGeneratorDelay3));
		xTaskCreate(dd_user_defined_task_3,	"DDUserDefinedTask3", configMINIMAL_STACK_SIZE,	(void*)ptask_info_3, TASK_LOWEST_PRIORITY, &(ptask_info_3->task_handle));
		vTaskSuspend(ptask_info_3->task_handle);
		printf("dd_task_generator_3 handle = 0x%x: released task!\n", ptask_info_3->task_handle);
		release_dd_task_info(ptask_info_3);
		vTaskDelay(xGeneratorDelay3);
	}
}

//static void dd_aperiodic_task_generator(void *pvParameters)
//{
//	dd_task_info_t *ptask_info = NULL;
//
//	while(1)
//	{
//		ptask_info = create_dd_task(user_defined_aperiodic_task_handle, APERIODIC, APERIODIC_TASK_ID, APERIODIC_TASK_PERIOD);
//		xTaskCreate(dd_user_defined_aperiodic_task,	"DDUserDefinedAperiodicTask", configMINIMAL_STACK_SIZE,	NULL, DD_TASK_PRIORITY, user_defined_aperiodic_task_handle);
//		release_dd_task(ptask_info);
//		vTaskDelay(pdMS_TO_TICKS(APERIODIC_TASK_PERIOD));
//	}
//}

static void vTaskTimerCallBack(xTimerHandle xTimer)
{
	QueueHandle_t dd_task_message_queue_handle = 0;
	uint32_t timer_id = 0;

	dd_task_message_queue_handle = pvTimerGetTimerID(xTimer);
	//printf("vTaskTimerCallBack: queue handle = 0x%x\n", dd_task_message_queue_handle);
	xQueueSend(dd_task_message_queue_handle, (void *)&timer_id, (TickType_t)10);
}

// Execute dd user-defined task 1
static void dd_user_defined_task_1(void *pvParameters)
{
	printf("dd_user_defined_task_1\n");
	dd_task_info_t *pMy_task_info = (dd_task_info_t *)pvParameters;
	QueueHandle_t dd_task1_message_queue_handle = NULL;
	//TimerHandle_t xTaskTimer1 = NULL;
	TaskHandle_t my_task_handle = xTaskGetCurrentTaskHandle();

	TickType_t execution_time1 = 0;
	TickType_t current_time = 0;
	uint32_t task_1_id = TASK1_ID;
	uint32_t *ptimer1_id = 0;
	uint32_t startTick;
	uint32_t endTick;

	dd_task1_message_queue_handle = xQueueCreate(taskQUEUE_LENGTH, sizeof(uint32_t));
	ptimer1_id = dd_task1_message_queue_handle;
	current_time = xTaskGetTickCount();
	execution_time1 = current_time + (TASK_1_EXECUTION_TIME/portTICK_PERIOD_MS);
	pMy_task_info->timer_handle = xTimerCreate("TaskTimer1", execution_time1, pdFALSE, (void *)ptimer1_id, vTaskTimerCallBack);
	xTimerStart(pMy_task_info->timer_handle, pdMS_TO_TICKS(0));
	startTick = xTaskGetTickCount();
	STM_EVAL_LEDOn(amber_led);
	printf("dd_user_defined_task_1 handle = 0x%x: Amber LED On.\n", (unsigned int)my_task_handle);

	if(xQueueReceive(dd_task1_message_queue_handle, &task_1_id, portMAX_DELAY) == pdTRUE)
	{
		// wait here until message is receive
	}

	endTick = xTaskGetTickCount();
	STM_EVAL_LEDOff(amber_led);
	printf("dd_user_defined_task_1 handle = 0x%x, tick = %d: Amber LED Off.\n", (unsigned int)my_task_handle, (int)(endTick - startTick));
	dd_task_completed(pMy_task_info);
	//xTimerDelete(pMy_task_info->timer_handle, pdMS_TO_TICKS(0));
	//vQueueDelete(dd_task1_message_queue_handle);
	//printf("task 1 handle = 0x%x, delete\n", my_task_handle);
	//vTaskDelete(my_task_handle);
	//vTaskDelete(NULL);
	vTaskSuspend(my_task_handle);
	//vTaskDelayUntil(&startTick, ((pMy_task_info->absolute_deadline) - startTick));
}

// Execute dd user-defined task 1
static void dd_user_defined_task_2(void *pvParameters)
{
	printf("dd_user_defined_task_2\n");
	dd_task_info_t *pMy_task_info = (dd_task_info_t *)pvParameters;
	QueueHandle_t dd_task2_message_queue_handle = NULL;
	TaskHandle_t my_task_handle = xTaskGetCurrentTaskHandle();

	//dd_message_t scheduler_message;
	TickType_t execution_time2 = 0;
	TickType_t current_time = 0;
	uint32_t task_2_id = TASK2_ID;
	uint32_t *ptimer2_id = 0;
	uint32_t startTick;
	uint32_t endTick;

	dd_task2_message_queue_handle = xQueueCreate(taskQUEUE_LENGTH, sizeof(uint32_t));
	ptimer2_id = dd_task2_message_queue_handle;
	current_time = xTaskGetTickCount();
	execution_time2 = current_time + (TASK_2_EXECUTION_TIME/portTICK_PERIOD_MS);
	pMy_task_info->timer_handle = xTimerCreate("TaskTimer2", execution_time2, pdFALSE, (void *)ptimer2_id, vTaskTimerCallBack);
	xTimerStart(pMy_task_info->timer_handle, pdMS_TO_TICKS(0));
	startTick = xTaskGetTickCount();
	STM_EVAL_LEDOn(green_led);
	printf("dd_user_defined_task_2 handle = 0x%x: Green LED On.\n", (unsigned int)my_task_handle);

	if(xQueueReceive(dd_task2_message_queue_handle, &task_2_id, portMAX_DELAY) == pdTRUE)
	{
		// wait here until message is receive
	}

	endTick = xTaskGetTickCount();
	STM_EVAL_LEDOff(green_led);
	printf("dd_user_defined_task_2 handle = 0x%x, tick = %d: Green LED Off.\n", (unsigned int)my_task_handle, (int)(endTick - startTick));
	dd_task_completed(pMy_task_info);
	//vTaskDelete(my_task_handle);
	vTaskSuspend(my_task_handle);
}

// Execute dd user-defined task 1
static void dd_user_defined_task_3(void *pvParameters)
{
	printf("dd_user_defined_task_3\n");
	dd_task_info_t *pMy_task_info = (dd_task_info_t *)pvParameters;
	QueueHandle_t dd_task3_message_queue_handle = NULL;
	TaskHandle_t my_task_handle = xTaskGetCurrentTaskHandle();

	TickType_t execution_time3 = 0;
	TickType_t current_time = 0;
	uint32_t task_3_id = TASK3_ID;
	uint32_t *ptimer3_id = 0;
	uint32_t startTick;
	uint32_t endTick;

	dd_task3_message_queue_handle = xQueueCreate(taskQUEUE_LENGTH, sizeof(uint32_t));
	ptimer3_id = dd_task3_message_queue_handle;
	current_time = xTaskGetTickCount();
	execution_time3 = current_time + (TASK_3_EXECUTION_TIME/portTICK_PERIOD_MS);
	pMy_task_info->timer_handle = xTimerCreate("TaskTimer3", execution_time3, pdFALSE, (void *)ptimer3_id, vTaskTimerCallBack);
	xTimerStart(pMy_task_info->timer_handle, pdMS_TO_TICKS(0));
	startTick = xTaskGetTickCount();
	STM_EVAL_LEDOn(blue_led);
	printf("dd_user_defined_task_3 handle = 0x%x: Blue LED On.\n", (unsigned int)my_task_handle);

	if(xQueueReceive(dd_task3_message_queue_handle, &task_3_id, portMAX_DELAY) == pdTRUE)
	{
		// wait here until message is receive
	}

	endTick = xTaskGetTickCount();
	STM_EVAL_LEDOff(blue_led);
	printf("dd_user_defined_task_3 handle = 0x%x, tick = %d: Blue LED Off.\n", (unsigned int)my_task_handle, (int)(endTick - startTick));
	dd_task_completed(pMy_task_info);
	vTaskSuspend(my_task_handle);
}

//static void dd_user_defined_aperiodic_task(void *pvParameters)
//{
//	TickType_t execution_time = APERIODIC_TASK_EXECUTION_TIME/portTICK_PERIOD_MS;
//	uint32_t aperiodic_task_id = APERIODIC_TASK_ID;
//	uint32_t *aperiodictimer_id = NULL;
//
//	while(1)
//	{
//		if(xQueueReceive(dd_task_message_queue, aperiodictimer_id, pdMS_TO_TICKS(0)) == pdTRUE)
//		{
//			break;
//		}
//
//		STM_EVAL_LEDOn(red_led);
//		printf("Red LED On.\n");
//		STM_EVAL_LEDOff(red_led);
//		printf("Red LED Off.\n");
//	}
//
//	xQueueSend(dd_scheduler_message_queue, &aperiodic_task_id, pdMS_TO_TICKS(0));
//	vTaskDelete(user_defined_aperiodic_task_handle);
//}

// Push Button Interrupt Handler
//void EXTI0_IRQHandler(void)
//{
//	/* Make sure that interrupt flag is set */
//	if (EXTI_GetITStatus(EXTI_Line0) != RESET)
//	{
//		// Switch an LED
//		//STM_EVAL_LEDToggle(green_led);
//		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//		/* Notify the task that the transmission is complete. */
//		vTaskNotifyGiveFromISR(user_defined_aperiodic_task_handle, &xHigherPriorityTaskWoken);
//		/* Clear interrupt flag (Want to do this as late as possible to avoid triggering the IRQ in the IRQ) */
//		EXTI_ClearITPendingBit(EXTI_Line0);
//		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
//	}
//}

dd_task_node_t *insert_new_node_to_active_list(dd_task_info_t *ptask_info)
{
	dd_task_node_t *ptemp = (dd_task_node_t*)pvPortMalloc(sizeof(dd_task_node_t)); // create a new node

	ptemp->pnode = ptask_info;
	ptemp->pnext_node = pActive_list_head;
	pActive_list_head = ptemp;

	return ptemp;
}

dd_task_node_t *insert_new_node_to_completed_list(dd_task_info_t *ptask_info)
{
	dd_task_node_t *ptemp = (dd_task_node_t*)pvPortMalloc(sizeof(dd_task_node_t)); // create a new node

	ptemp->pnode = ptask_info;
	ptemp->pnext_node = pCompletion_list_head;
	pCompletion_list_head = ptemp;

	return ptemp;
}

dd_task_node_t *insert_new_node_to_overdue_list(dd_task_info_t *ptask_info)
{
	dd_task_node_t *ptemp = (dd_task_node_t*)pvPortMalloc(sizeof(dd_task_node_t)); // create a new node

	ptemp->pnode = ptask_info;
	ptemp->pnext_node = pOverdue_list_head;
	pOverdue_list_head = ptemp;

	return ptemp;
}

uint32_t active_list_length()
{
	uint32_t list_length = 0;
	dd_task_node_t *pcurrent = NULL;

	for(pcurrent = pActive_list_head; pcurrent != NULL; pcurrent = pcurrent->pnext_node)
	{
		list_length++;
	}

	return list_length;
}

// sort active_dd_task_list
// List of DD tasks to be scheduled by the DD scheduler
// -	Sort the list by deadline every time a DD task is added or removed from the list using selection sort
// -	Select and implement a data structure and sorting algorithm to sort the list
// -	Use singly-linked list for sorting
void sort_active_list_by_deadline(dd_task_info_t *ptask_info)
{
	dd_task_node_t *pcurrent = NULL;
	dd_task_node_t *pnext = NULL;
	dd_task_info_t *ptemp = NULL;

	uint32_t list_size = active_list_length();
	uint32_t stored_list_size = list_size;

	for(int i = 0; i < (list_size - 1); i++, stored_list_size--)
	{
		pcurrent = pActive_list_head;
		pnext = pActive_list_head->pnext_node;

		for(int j = 0; j < stored_list_size; j++)
		{
			if((pcurrent->pnode->absolute_deadline) > (ptask_info->absolute_deadline))
			{
				ptemp = pcurrent->pnode;
				pcurrent->pnode = pnext->pnode;
				pnext->pnode = ptemp;
			}

			pcurrent = pcurrent->pnext_node;
			pnext = pnext->pnext_node;
		}
	}
}

dd_task_node_t *pFind_completed_task_node_by_time_stamp(dd_task_info_t *ptask_info)
{
	dd_task_node_t *pcurrent = pActive_list_head;

	if(pActive_list_head == NULL)
	{
		return NULL;
	}

	while(pcurrent->pnode->completion_time != (ptask_info->completion_time))
	{
		if(pcurrent->pnext_node == NULL)
		{
			return NULL;
		}
		else
		{
			pcurrent = pcurrent->pnext_node;
		}
	}

	return pcurrent;
}

dd_task_node_t *pFind_overdue_task_node_using_time_stamp(uint32_t time_stamp)
{
	dd_task_node_t *pcurrent = pActive_list_head;

	if(pActive_list_head == NULL)
	{
		return NULL;
	}

	while(pcurrent->pnode->absolute_deadline < time_stamp)
	{
		if(pcurrent->pnext_node == NULL)
		{
			return NULL;
		}
		else
		{
			pcurrent = pcurrent->pnext_node;
		}
	}

	return pcurrent;
}

// List of DD tasks that have completed execution before their deadlines
// -	Remove completed tasks before their deadline from Active Task List
// -	Add these completed tasks to Completed Task List
dd_task_node_t *pRemove_completed_task_node_by_time_stamp(dd_task_info_t *ptask_info)
{
	dd_task_node_t *pcurrent = pActive_list_head;
	dd_task_node_t *pprevious = NULL;

	if(pActive_list_head == NULL)
	{
		return NULL;
	}

	while((pcurrent->pnode->completion_time) != ptask_info->completion_time)
	{
		// if it is the last node
		if(pcurrent->pnext_node == NULL)
		{
			return NULL;
		}
		else
		{
			pprevious = pcurrent;
			pcurrent = pcurrent->pnext_node;
		}
	}

	// found a match
	if(pcurrent == pActive_list_head)
	{
		pActive_list_head = pActive_list_head->pnext_node;
	}
	else
	{
		pprevious->pnext_node = pcurrent->pnext_node;
	}

	return pcurrent;
}

dd_task_node_t *pRemove_overdue_task_node_by_time_stamp(uint32_t time_stamp)
{
	dd_task_node_t *pcurrent = pActive_list_head;
	dd_task_node_t *pprevious = NULL;

	if(pActive_list_head == NULL)
	{
		return NULL;
	}

	while((pcurrent->pnode->absolute_deadline) < time_stamp)
	{
		// if it is the last node
		if(pcurrent->pnext_node == NULL)
		{
			return NULL;
		}
		else
		{
			pprevious = pcurrent;
			pcurrent = pcurrent->pnext_node;
		}
	}

	// found a match
	if(pcurrent == pActive_list_head)
	{
		pActive_list_head = pActive_list_head->pnext_node;
	}
	else
	{
		pprevious->pnext_node = pcurrent->pnext_node;
	}

	return pcurrent;
}

void printActiveList()
{
	dd_task_node_t *ptemp = pActive_list_head;

	while(ptemp != NULL)
	{
		printf("Task handle = 0x%x, release time = %d\n", ptemp->pnode->task_handle, ptemp->pnode->release_time);
		ptemp = ptemp->pnext_node;
	}

	printf("dd_scheduler gets here?\n");
}

void printCompletedList()
{
	dd_task_node_t *ptemp = pCompletion_list_head;

	while(ptemp != NULL)
	{
		printf("Task handle = 0x%x, completion time = %d\n", ptemp->pnode->task_handle, ptemp->pnode->completion_time);
		ptemp = ptemp->pnext_node;
	}
}

void printOverdueList()
{
	dd_task_node_t *ptemp = pOverdue_list_head;

	while(ptemp != NULL)
	{
		printf("Task handle = 0x%x, overdue time = %d\n", ptemp->pnode->task_handle, ptemp->pnode->overdue_time);
		ptemp = ptemp->pnext_node;
	}
}

// Execute deadline-driven scheduler task
// 1.	Implements EDF algorithm
// 2.	Control the priorities of users-define FreeRTOS tasks from an actively-managed list of DD tasks
void dd_task_scheduler(void *pvParameters)
{
	printf("dd_task_scheduler: print 1st\n");
	dd_message_t scheduler_message;
	dd_task_node_t *pnode_with_completion_time = NULL;
	dd_task_node_t *pnode_with_completion_time_removed = NULL;
	dd_task_node_t *pnode_with_overdue_time = NULL;
	dd_task_node_t *active_list = NULL;
	dd_task_node_t *completed_list = NULL;
	dd_task_node_t *overdue_list = NULL;

	TickType_t release_time = 0;
	//TickType_t aperodic_task_timer_period = 0;
	TickType_t dd_task_completion_time = 0;
	dd_task_info_t *ptask_info;
	printf("dd_task_scheduler: print 2nd\n");

	while(1)
	{
		printf("dd_task_scheduler waiting for message\n");
		if(xQueueReceive(dd_scheduler_message_queue, &scheduler_message, portMAX_DELAY) == pdTRUE)
		{
			printf("Scheduler message type: %d\n", scheduler_message.message_type);
			ptask_info = scheduler_message.ptask_info;

			//		//ptask_info->release_time = release_time;
			//		pnode_with_overdue_time = pFind_overdue_task_node_using_time_stamp(ptask_info->release_time);
			//		printf("pnode_with_overdue_time = %d", );
			//		overdue_list = insert_new_node_to_overdue_list(pnode_with_overdue_time->pnode);
			//		pRemove_overdue_task_node_by_time_stamp(ptask_info->release_time);
			//		sort_active_list_by_deadline(ptask_info);
			//		vTaskPrioritySet(ptask_info->task_handle, tskIDLE_PRIORITY + 1);

			//printf("dd_task_scheduler: print 3rd\n");


			switch(scheduler_message.message_type)
			{
			// If DDS receives message from release_dd_task
			// then	DD scheduler:
			// -	assigns release time for new task
			// -	inserts DD task to Active task List
			// -	sorts the list by deadline
			// -	sets priorities of the User-Defined tasks
			case RELEASE_TASK:
				printf("dd_task_scheduler: task has been released\n");
				release_time = xTaskGetTickCount();
				ptask_info->release_time = release_time;
				printf("Task 0x%x, released time = %d\n", ptask_info->task_handle, ptask_info->release_time);
				active_list = insert_new_node_to_active_list(ptask_info);
				sort_active_list_by_deadline(ptask_info);
				vTaskPrioritySet(ptask_info->task_handle, tskIDLE_PRIORITY + 1);
				//xAperiodicTimer = xTimerCreate("AperiodicTaskTimer", execution_time, pdFALSE, aperiodictimer_id, vAperiodicTaskTimerCallBack);
				//xTimerStart(xAperiodicTimer, pdMS_TO_TICKS(0));
				vTaskResume(ptask_info->task_handle);
				break;

				// If DDS receives message from complete_dd_task
				// then DD scheduler:
				// -	assigns completion time to newly-completed DD task
				// -	removes DD task from Active Task List
				// -	inserts it to the Completed Task List
				// -	sorts Active Task List by deadline
				// -	sets priorities of the User-Define tasks
			case COMPLETED_TASK:
				printf("dd_task_scheduler: task has been completed\n");
				dd_task_completion_time = xTimerGetPeriod(ptask_info->timer_handle);
				ptask_info->completion_time = dd_task_completion_time;
				printf("Task 0x%x completion time %d \n", ptask_info->task_handle, ptask_info->completion_time);
				pnode_with_completion_time = pFind_completed_task_node_by_time_stamp(ptask_info);
				//printf("dd_scheduler gets here?\n");
				completed_list = insert_new_node_to_completed_list(pnode_with_completion_time->pnode);
				pnode_with_completion_time_removed = pRemove_completed_task_node_by_time_stamp(pnode_with_completion_time->pnode);
				//delete_dd_task_info(pnode_with_completion_time_removed->pnode);
				//sort_active_list_by_deadline(ptask_info);

				//vTaskPrioritySet(ptask_info->task_handle, tskIDLE_PRIORITY + 1);

				break;

				// If DDS receives message from get_active_dd_task_list
				// then DD scheduler sends Active Task List to the monitor queue
			case GET_ACTIVE_DD_TASK_LIST:
				if(uxQueueSpacesAvailable(dd_monitor_message_queue) == 0)
				{
					xQueueReset(dd_monitor_message_queue);
				}

				xQueueSend(dd_monitor_message_queue, (void *)active_list, portMAX_DELAY);

				break;

				// If DDS receives message from get_completed_dd_task_list
				// then DDS sends Completed Task List to the monitor queue
			case GET_COMPLETED_DD_TASK_LIST:
				if(uxQueueSpacesAvailable(dd_monitor_message_queue) == 0)
				{
					xQueueReset(dd_monitor_message_queue);
				}

				xQueueSend(dd_monitor_message_queue, (void *)completed_list, portMAX_DELAY);
				break;

				// Message from get_overdue_dd_task_list
				// DD scheduler	sends Overdue Task List to a queue
			case GET_OVERDUE_DD_TASK_lIST:
				if(uxQueueSpacesAvailable(dd_monitor_message_queue) == 0)
				{
					xQueueReset(dd_monitor_message_queue);
				}

				xQueueSend(dd_monitor_message_queue, (void *)overdue_list, portMAX_DELAY);
				break;

			default:
				printf("Error: Unrecognized message type %d!\n", scheduler_message.message_type);
				break;
			}



//		//ptask_info->release_time = release_time;
//		pnode_with_overdue_time = pFind_overdue_task_node_using_time_stamp(ptask_info->release_time);
//		printf("pnode_with_overdue_time = %d", );
//		overdue_list = insert_new_node_to_overdue_list(pnode_with_overdue_time->pnode);
//		pRemove_overdue_task_node_by_time_stamp(ptask_info->release_time);
//		sort_active_list_by_deadline(ptask_info);
//		vTaskPrioritySet(ptask_info->task_handle, tskIDLE_PRIORITY + 1);

		//printf("dd_task_scheduler: print 3rd\n");


//		switch(scheduler_message.message_type)
//		{
//			// If DDS receives message from release_dd_task
//			// then	DD scheduler:
//			// -	assigns release time for new task
//			// -	inserts DD task to Active task List
//			// -	sorts the list by deadline
//			// -	sets priorities of the User-Defined tasks
//			case RELEASE_TASK:
//				printf("dd_task_scheduler: task has been released\n");
//				release_time = xTaskGetTickCount();
//				ptask_info->release_time = release_time;
//				printf("Task 0x%x, released time = %d\n", ptask_info->task_handle, ptask_info->release_time);
//				active_list = insert_new_node_to_active_list(ptask_info);
//				sort_active_list_by_deadline(ptask_info);
//				vTaskPrioritySet(ptask_info->task_handle, tskIDLE_PRIORITY + 1);
//				//xAperiodicTimer = xTimerCreate("AperiodicTaskTimer", execution_time, pdFALSE, aperiodictimer_id, vAperiodicTaskTimerCallBack);
//				//xTimerStart(xAperiodicTimer, pdMS_TO_TICKS(0));
//				vTaskResume(ptask_info->task_handle);
//				break;
//
//			// If DDS receives message from complete_dd_task
//			// then DD scheduler:
//			// -	assigns completion time to newly-completed DD task
//			// -	removes DD task from Active Task List
//			// -	inserts it to the Completed Task List
//			// -	sorts Active Task List by deadline
//			// -	sets priorities of the User-Define tasks
//			case COMPLETED_TASK:
//				printf("dd_task_scheduler: task has been completed\n");
//				dd_task_completion_time = xTimerGetPeriod(ptask_info->timer_handle);
//				ptask_info->completion_time = dd_task_completion_time;
//				printf("Task 0x%x completion time %d \n", ptask_info->task_handle, ptask_info->completion_time);
//				pnode_with_completion_time = pFind_completed_task_node_by_time_stamp(ptask_info);
//				completed_list = insert_new_node_to_completed_list(pnode_with_completion_time->pnode);
//				pRemove_completed_task_node_by_time_stamp(pnode_with_completion_time->pnode);
//				sort_active_list_by_deadline(ptask_info);
//				vTaskPrioritySet(ptask_info->task_handle, tskIDLE_PRIORITY + 1);
//				break;
//
//			// If DDS receives message from get_active_dd_task_list
//			// then DD scheduler sends Active Task List to the monitor queue
//			case GET_ACTIVE_DD_TASK_LIST:
//				if(uxQueueSpacesAvailable(dd_monitor_message_queue) == 0)
//				{
//					xQueueReset(dd_monitor_message_queue);
//				}
//
//				xQueueSend(dd_monitor_message_queue, (void *)active_list, portMAX_DELAY);
//				break;
//
//			// If DDS receives message from get_completed_dd_task_list
//			// then DDS sends Completed Task List to the monitor queue
//			case GET_COMPLETED_DD_TASK_LIST:
//				if(uxQueueSpacesAvailable(dd_monitor_message_queue) == 0)
//				{
//					xQueueReset(dd_monitor_message_queue);
//				}
//
//				xQueueSend(dd_monitor_message_queue, (void *)completed_list, portMAX_DELAY);
//				break;
//
//			// Message from get_overdue_dd_task_list
//			// DD scheduler	sends Overdue Task List to a queue
//			case GET_OVERDUE_DD_TASK_lIST:
//				if(uxQueueSpacesAvailable(dd_monitor_message_queue) == 0)
//				{
//					xQueueReset(dd_monitor_message_queue);
//				}
//
//				xQueueSend(dd_monitor_message_queue, (void *)overdue_list, portMAX_DELAY);
//				break;
//
//			default:
//				printf("Error: Unrecognized message type %d!\n", scheduler_message.message_type);
//				break;
		}
	}
}

// get_active_dd_task_list sends a message to a queue requesting Active Task List from DD scheduler
// Once DD Scheduler responds, get_active_dd_task_list function returns the list
dd_task_node_t **pGetActiveDDTaskList(void)
{
	dd_message_t request_active_dd_task_list_message;
	request_active_dd_task_list_message.message_type = GET_ACTIVE_DD_TASK_LIST;
	request_active_dd_task_list_message.ptask_list = NULL;

	xQueueSend(dd_scheduler_message_queue, (void*)&(request_active_dd_task_list_message.message_type), portMAX_DELAY);

	if(xQueueReceive(dd_monitor_message_queue, &(request_active_dd_task_list_message.ptask_list), portMAX_DELAY) == pdTRUE)
	{
		printActiveList();
	}

	return 0;
}

// get_completed_dd_task_list
// -	send a message to a queue requesting Completed Task List from DD scheduler
// Once DD Scheduler responds, get_completed_dd_task_list function returns the list
dd_task_node_t **pGetCompletedDDTaskList(void)
{


	dd_message_t request_complete_dd_task_list_message;
	request_complete_dd_task_list_message.message_type = GET_COMPLETED_DD_TASK_LIST;
	request_complete_dd_task_list_message.ptask_list = NULL;

	xQueueSend(dd_scheduler_message_queue, (void *)&(request_complete_dd_task_list_message.message_type), portMAX_DELAY);

	if(xQueueReceive(dd_monitor_message_queue, &(request_complete_dd_task_list_message.ptask_list), portMAX_DELAY) == pdTRUE)
	{
		printCompletedList();
	}

	return 0;
}

// get_overdue_dd_task_list
// send a message to a queue requesting Overdue Task List from DD scheduler
// Once DD Scheduler responds, get_overdue_dd_task_list function returns the list
dd_task_node_t **pGetOverdueDDTaskList(void)
{
	dd_message_t request_overdue_dd_task_list_message;
	request_overdue_dd_task_list_message.message_type = GET_OVERDUE_DD_TASK_lIST;
	request_overdue_dd_task_list_message.ptask_list = NULL;

	xQueueSend(dd_scheduler_message_queue, (void *)&(request_overdue_dd_task_list_message.message_type), portMAX_DELAY);

	if(xQueueReceive(dd_monitor_message_queue, &(request_overdue_dd_task_list_message.ptask_list), portMAX_DELAY) == pdTRUE)
	{
		printOverdueList();
	}

	return 0;
}

// This task displays traffic light system and car on the road by updating the LEDs using the shift register
void dd_task_monitor(void *pvParameters)
{
	vTaskDelay(10000);
	while(1)
	{
		printf("dd_task_monitor: Active Task List\n");
		pGetActiveDDTaskList();
//		printf("dd_task_monitor: Completed Task List\n");
//		pGetCompletedDDTaskList();
//		printf("dd_task_monitor: Overdue Task List\n");
//		pGetOverdueDDTaskList();
		//vTaskDelay(100);
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
