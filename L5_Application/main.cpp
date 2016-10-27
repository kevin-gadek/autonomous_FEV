/*
 *     SocialLedge.com - Copyright (C) 2013
 *
 *     This file is part of free software framework for embedded processors.
 *     You can use it and/or distribute it as long as this copyright header
 *     remains unmodified.  The code is free for personal use and requires
 *     permission to use in a commercial product.
 *
 *      THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 *      OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 *      MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 *      I SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 *      CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 *     You can reach the author of this software at :
 *          p r e e t . w i k i @ g m a i l . c o m
 */

/**
 * @file
 * @brief This is the application entry point.
 * 			FreeRTOS and stdio printf is pre-configured to use uart0_min.h before main() enters.
 * 			@see L0_LowLevel/lpc_sys.h if you wish to override printf/scanf functions.
 *
 */
#include "tasks.hpp"
#include "examples/examples.hpp"
#include "scheduler_task.hpp"
#include "event_groups.h"
#include "storage.hpp"
#include "time.h"
#include "string.h"
#include <io.hpp>
#include <stdio.h>
#include <queue.h>
/**
 * The main() creates tasks or "threads".  See the documentation of scheduler_task class at scheduler_task.hpp
 * for details.  There is a very simple example towards the beginning of this class's declaration.
 *
 * @warning SPI #1 bus usage notes (interfaced to SD & Flash):
 *      - You can read/write files from multiple tasks because it automatically goes through SPI semaphore.
 *      - If you are going to use the SPI Bus in a FreeRTOS task, you need to use the API at L4_IO/fat/spi_sem.h
 *
 * @warning SPI #0 usage notes (Nordic wireless)
 *      - This bus is more tricky to use because if FreeRTOS is not running, the RIT interrupt may use the bus.
 *      - If FreeRTOS is running, then wireless task may use it.
 *        In either case, you should avoid using this bus or interfacing to external components because
 *        there is no semaphore configured for this bus and it should be used exclusively by nordic wireless.
 */
typedef enum {
	shared_SensorQueueId,
} sharedHandleId_t;

typedef enum {
	invalid,
	left,
	right,
	up,
	down,
} orientation_t;

class task_1 : public scheduler_task{
public:
	task_1(uint8_t priority) : scheduler_task("task_1", 2000, priority){}
	bool run(void *p){
		QueueHandle_t myQueue = xQueueCreate(1, sizeof(orientation_t));
		addSharedObject(shared_SensorQueueId, myQueue);
		int x = AS.getX();
		int y = AS.getY();
		//int z = AS.getZ();

		orientation_t orientation = invalid;
		if(x > 300)
		{
			//printf("Tilting left\n");
			orientation = left;
		}
		if(x < -300)
		{
			//printf("Tilting right\n");
			orientation = right;
		}
		if(y > 300)
		{
			//printf("Tilting up\n");
			orientation = up;
		}
		if(y < -300)
		{
			//printf("Tilting down\n");
			orientation = down;
		}
		printf("Task 1 is sending %i to queue.\n", orientation);
		xQueueSend(getSharedObject(shared_SensorQueueId), &orientation, 1000);
		printf("Task 1 has sent %i to queue.\n", orientation);
		vTaskDelay(1000);
		return true;
	}
	bool init(void){
		//empty
		return true;
	}
};

class task_2 : public scheduler_task{
public:
	task_2(uint8_t priority) : scheduler_task("task_2", 2000, priority){}
	bool run(void *p){
		orientation_t orientation = invalid;
		QueueHandle_t myQueue = getSharedObject(shared_SensorQueueId);
		if(xQueueReceive(myQueue, &orientation, 1000))
		{
			printf("Task 2 has received %i from queue\n", orientation);
			if(orientation == left || orientation == right){
				LPC_GPIO1->FIOSET |= (0xF << 0);
				//turn on LEDS
			}
			else
			{
				LPC_GPIO1->FIOCLR |= (0xF << 0);
				//turn off LEDS
			}
		}
		vTaskDelay(1000);
		return true;
	}
	bool init(void){
		//sets bits 0-3 to 0 for GPIO functionality of LEDS 0 and 1
		LPC_PINCON->PINSEL2 &= ~(0xF << 0);
		//sets bits 8-9 for GPIO functionality of LED 2
		LPC_PINCON->PINSEL2 &= ~(0x3 << 8);
		LPC_PINCON->PINSEL2 &= ~(0x3 << 16);
		//Data direction init section
		LPC_GPIO1->FIODIR |= (0xF << 0);


		return true;
	}
};

EventGroupHandle_t event_group;

class producer_task: public scheduler_task
{
public:
	producer_task(uint8_t priority): scheduler_task("producer", 2048, priority)
	{
		puts("Inside Producer Task \n\n");
		current_average = 0;
		current_iteration = 1;
		myqueue = xQueueCreate(5, sizeof(double));
		addSharedObject(shared_SensorQueueId, myqueue);
	}

	bool init(void)
	{
		return true;

	}

	bool run(void *p)
	{
		current_average += LS.getPercentValue();
		if(current_iteration == 99)
		{
			current_iteration = 0;
			current_average /= 100;
			xQueueSend(myqueue, &current_average, portMAX_DELAY);
			current_average = 0;
		}
		current_iteration++;
		vTaskDelay(1);
		xEventGroupSetBits(event_group, 0x2);
		return true;
	}
private:
	double current_average; //since in FREERTOS memory is precious and it doesn't say you need to store individual values
	int current_iteration;
	QueueHandle_t myqueue;
};

class consumer_task: public scheduler_task
{
public:
	consumer_task(uint8_t priority): scheduler_task("consumer", 2048, priority)
	{
		myqueue = getSharedObject(shared_SensorQueueId);
		numValues = 0;
		receivedAverage = 0;
		uptime = 0;
		puts("Inside Consumer Task \n\n");
	}

	bool init(void)
	{
		return true;
	}

	bool run(void *p)
	{

		if(xQueueReceive(myqueue, &receivedAverage, portMAX_DELAY))
		{
			uptime = sys_get_uptime_ms();
			arrayOfTimes[numValues] = uptime;
			arrayOfAverages[numValues] = receivedAverage;
			numValues++;
			if(numValues == 10)
			{
				for(int i = 0; i < 10; i++)
				{
					size = snprintf(print_buffer, 100, "%i, %f\n", arrayOfTimes[i], arrayOfAverages[i]);
					Storage::append("1:sensor.txt", &print_buffer, size, 0);
					vTaskDelay(1000);
					//printf("Value %i for Time %i, is: %f\n", i, arrayOfTimes[i], arrayOfAverages[i]);
				}
				vTaskDelay(1000);
				//printf("Write to sensor.txt completed\n");
				numValues = 0; //reset the Buffer

			}

		}
		xEventGroupSetBits(event_group, 0x4);
		return true;
	}
private:
	QueueHandle_t myqueue;
	int numValues;
	unsigned int size;
	double receivedAverage;
	char print_buffer[100];
	double arrayOfAverages[10];
	int arrayOfTimes[10];

	uint64_t uptime;

};
const uint32_t all_task_bits = ((0x2) | (0x4));
class watchdog_task : public scheduler_task{
public:
	watchdog_task(uint8_t priority) : scheduler_task("watchdog_task", 2000, priority){}
	bool run(void *p){

		//printf("Entered watchdog task, counter: %i\n", watchdog_counter++);
		//check for 0x6 so 2nd and 3rd bits
		uint32_t event_bits = xEventGroupWaitBits(event_group, all_task_bits,pdTRUE,pdTRUE, 2000);
			//printf("xEventGroupWaitBits function called.\n");
		                        char print[100];
		                        //counter of bytes that would need to be written to stuck.txt
		                        int size;
		                        bool flag = false;

		                        size = snprintf(print, 100, "Tasks that aren't responding: ");
		                        if(!(event_bits & 0x2))
		                        {
		                        	//time_t current_time = time(NULL);
		                        	printf("Producer task currently suspended\n");
		                            strncat(print, " Producer", 9);
		                            size += 10;
		                            flag = true;
		                        }

		                        if(!(event_bits & 0x4))
		                        {
		                        	//time_t current_time = time(NULL);
		                        	//printf("Consumer task suspended at time: %li\n", current_time);
		                            strncat(print, " Consumer", 9);
		                            size += 10;
		                            flag = true;
		                        }

		                        strncat(print, "\n", 1);
		                        size++;
		                        if(flag)
		                        {
		                            Storage::append("1:stuck.txt", &print, size , 0);
		                        }

		                        return true;
	}

	bool init(void){
		return true;
	}
};

int main(void)
{
    /**
     * A few basic tasks for this bare-bone system :
     *      1.  Terminal task provides gateway to interact with the board through UART terminal.
     *      2.  Remote task allows you to use remote control to interact with the board.
     *      3.  Wireless task responsible to receive, retry, and handle mesh network.
     *
     * Disable remote task if you are not using it.  Also, it needs SYS_CFG_ENABLE_TLM
     * such that it can save remote control codes to non-volatile memory.  IR remote
     * control codes can be learned by typing the "learn" terminal command.
     */

    scheduler_add_task(new terminalTask(PRIORITY_HIGH));

    /* Consumes very little CPU, but need highest priority to handle mesh network ACKs */
    scheduler_add_task(new wirelessTask(PRIORITY_CRITICAL));
	#if 0
    scheduler_add_task(new task_1(PRIORITY_MEDIUM));
    scheduler_add_task(new task_2(PRIORITY_MEDIUM));
	#endif
    event_group = xEventGroupCreate();
    scheduler_add_task(new producer_task(PRIORITY_MEDIUM));
    scheduler_add_task(new consumer_task(PRIORITY_MEDIUM));
    scheduler_add_task(new watchdog_task(PRIORITY_HIGH));
    /* Change "#if 0" to "#if 1" to run period tasks; @see period_callbacks.cpp */
    #if 0
    scheduler_add_task(new periodicSchedulerTask());
    #endif

    /* The task for the IR receiver */
    // scheduler_add_task(new remoteTask  (PRIORITY_LOW));

    /* Your tasks should probably used PRIORITY_MEDIUM or PRIORITY_LOW because you want the terminal
     * task to always be responsive so you can poke around in case something goes wrong.
     */

    /**
     * This is a the board demonstration task that can be used to test the board.
     * This also shows you how to send a wireless packets to other boards.
     */
    #if 0
        scheduler_add_task(new example_io_demo());
    #endif

    /**
     * Change "#if 0" to "#if 1" to enable examples.
     * Try these examples one at a time.
     */
    #if 0
        scheduler_add_task(new example_task());
        scheduler_add_task(new example_alarm());
        scheduler_add_task(new example_logger_qset());
        scheduler_add_task(new example_nv_vars());
    #endif

    /**
	 * Try the rx / tx tasks together to see how they queue data to each other.
	 */
    #if 0
        scheduler_add_task(new queue_tx());
        scheduler_add_task(new queue_rx());
    #endif

    /**
     * Another example of shared handles and producer/consumer using a queue.
     * In this example, producer will produce as fast as the consumer can consume.
     */
    #if 0
        scheduler_add_task(new producer());
        scheduler_add_task(new consumer());
    #endif

    /**
     * If you have RN-XV on your board, you can connect to Wifi using this task.
     * This does two things for us:
     *   1.  The task allows us to perform HTTP web requests (@see wifiTask)
     *   2.  Terminal task can accept commands from TCP/IP through Wifly module.
     *
     * To add terminal command channel, add this at terminal.cpp :: taskEntry() function:
     * @code
     *     // Assuming Wifly is on Uart3
     *     addCommandChannel(Uart3::getInstance(), false);
     * @endcode
     */
    #if 0
        Uart3 &u3 = Uart3::getInstance();
        u3.init(WIFI_BAUD_RATE, WIFI_RXQ_SIZE, WIFI_TXQ_SIZE);
        scheduler_add_task(new wifiTask(Uart3::getInstance(), PRIORITY_LOW));
    #endif

    scheduler_start(); ///< This shouldn't return
    return -1;
}

/*
QueueHandle_t sensor_queue;
EventGroupHandle_t event_group;

class producer_task : public scheduler_task{
public:
	producer_task(uint8_t priority) : scheduler_task("producer_task", 512*4, priority){}
	bool run(void *p){

		//queue length: 10 items, item size: sizeof(int)
		//QueueHandle_t sensor_queue = xQueueCreate(10, sizeof(int));
		//addSharedObject(shared_SensorQueueId, sensor_queue);
		int lightAverage = 0;
		for(int i = 0; i < 100; i++){
		//add 100 samples into lightAverage
			lightAverage += LS.getRawValue();
			//assuming 1 tick is 1 ms
			vTaskDelay(10);
		}
		//average out samples
		lightAverage = lightAverage/100;

		//printf("Light Average: %i\n", lightAverage);
		printf("Producer about to send item to queue.\n");
		xQueueSend(sensor_queue, &lightAverage, 1000);
		printf("Producer has just sent item to queue.\n");
		xEventGroupSetBits(event_group, 0x2);
		return true;
	}
	bool init(void){
		return true;
	}
};

class consumer_task : public scheduler_task{
public:
	consumer_task(uint8_t priority) : scheduler_task("consumer_task", 512*4, priority){}
	bool run(void *p){
		int buffer[10];
		int light = 0;
		//store time for each buffer value
		time_t time_buffer[10];
		//QueueHandle_t sensor_queue = getSharedObject(shared_SensorQueueId);
		//this should get 10 samples of the averaged data from producer
		for(int j = 0; j < 10; j++){
			//replace with max block time
			if(xQueueReceive(sensor_queue, &light, portMAX_DELAY) == pdTRUE){
					printf("Consumer has just received item from queue.\n");
					//should pull 10 values from queue
						buffer[j] = light;
						//assign corresponding time value to time_buffer
						time_buffer[j] = time(NULL);
						printf("Inside receive loop, j: %i\n", j);
						//printf("Light Average (consumer): %li, data: %u\n", time_buffer[j], buffer[j]);
					}else{
						printf("Item not received from queue\n");
					}
		}
		//need to format the data into printable statement
		char print_buffer[100];
		//need to use Storage::append()
		for(int k = 0; k < 10; k++){
			//reformat data buffer and time buffer into print_buffer
			unsigned int size = snprintf(print_buffer, 100, "%li, %u\n", time_buffer[k], buffer[k]);
			Storage::append("0:sensor.txt", &print_buffer, size, 0);
			//printf("Light Average (consumer): %i\n", buffer[i]);
		}
		xEventGroupSetBits(event_group, 0x4);
		//Storage::read("sensor.txt", );

		return true;
	}

	bool init(void){
		return true;
	}

};

*/
