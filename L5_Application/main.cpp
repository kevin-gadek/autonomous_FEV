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
					vTaskDelay(100);
					//printf("Value %i for Time %i, is: %f\n", i, arrayOfTimes[i], arrayOfAverages[i]);
				}
				//vTaskDelay(1000);
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

class cpu_usage_task : public scheduler_task{
public:
	cpu_usage_task(uint8_t priority) : scheduler_task("cpu_usage_task", 2000, priority){}
	bool run(void *p){
		//runs every 20 seconds
		vTaskDelayUntil(&current_time, 20000);
		const char * const taskStatusTbl[] = { "RUN", "RDY", "BLK", "SUS", "DEL" };
	    const unsigned portBASE_TYPE maxTasks = 16;
	    TaskStatus_t status[maxTasks];
	    uint32_t totalRunTime = 0;
	        uint32_t tasksRunTime = 0;
	        const unsigned portBASE_TYPE uxArraySize =
	                uxTaskGetSystemState(&status[0], maxTasks, &totalRunTime);

	        char print_buffer[100];
	       unsigned int size = snprintf(print_buffer, 100, "\n\n10s     Sta     Pr   Stack    CPU%%          Time\n");
	        Storage::append("1:cpu.txt", &print_buffer, size, 0);

	        for(unsigned priorityNum = 0; priorityNum < configMAX_PRIORITIES; priorityNum++)
	            {
	                /* Print in sorted priority order */
	                for (unsigned i = 0; i < uxArraySize; i++) {
	                    TaskStatus_t *e = &status[i];
	                    if (e->uxBasePriority == priorityNum) {
	                        tasksRunTime += e->ulRunTimeCounter;

	                        const uint32_t cpuPercent = (0 == totalRunTime) ? 0 : e->ulRunTimeCounter / (totalRunTime/100);
	                        const uint32_t timeUs = e->ulRunTimeCounter;
	                        const uint32_t stackInBytes = (4 * e->usStackHighWaterMark);

	                        size = snprintf(print_buffer, 100, "Name: %10s %s %2u %5u %4u %10u us\n", e->pcTaskName, taskStatusTbl[e->eCurrentState], e->uxBasePriority,
                                    stackInBytes, cpuPercent, timeUs);
	                        Storage::append("1:cpu.txt", &print_buffer, size, 0);
	                    }
	                }
	            }
		return true;
	}
	bool init(void){

	}
private:
	TickType_t current_time = xTaskGetTickCount();

};
//using P0.26(AD0.3) for test; set PINSEL1->bits 20-21 to 01
//Only 3 accessible ADC pins on board, need analog mux tied to GPIO sel signal if want to use more
class ir_task : public scheduler_task{
public:
	ir_task(uint8_t priority) : scheduler_task("ir_task", 2000, priority){}
	bool run(void *p){
		while(1){
			//if done flag for channel 3 is set
					if(LPC_ADC->ADSTAT & (1 << 3)){
						//12 bits of data
						char data = LPC_ADC->ADDR3;
						//puts("Inside data loop\n");
						printf("Data: %x\n", data);
						//vTaskDelay(1);
					}
		}

		return true;
	}
	bool init(void){
		//set bit 12 of PCONP
		LPC_SC->PCONP |= (1 << 12);
		//peripheral clock select
		LPC_SC->PCLKSEL0 &= ~(3 << 24); //clear
		LPC_SC->PCLKSEL0 |= (1 << 24); //set to CCLK/1

		//disable pull-up and pull-down resistors
		LPC_PINCON->PINMODE1 &= ~(3 << 20); //clear
		LPC_PINCON->PINMODE1 |= (0x2 << 20); //set to 10 to disable pull-up and pull-down
		//set pin functionality in PINSEL
		//select P0.26(AD0.3) as 01 functionality or AD0.3
		LPC_PINCON->PINSEL1 &= ~(3 << 20); //clear
		LPC_PINCON->PINSEL1 |= (1 << 20); //set to 01

		//disable interrupts on channel 3 for hardware scan mode
		LPC_ADC->ADINTEN &= ~(1 << 3); //clear bit 3
		//need to set AD control register
		//select all 8 AD pins so hardware scan mode works better
		//burst(bit 16) should be set for hardware scan mode
		//PDN (bit 21) should be set to enable ADC
		//START (bits 24-27 must be 0s because BURST bit is set)
		//0000 0000 0010 0001 0000 0000 1111 1111
		//select all ad pins 0x2100FF
		LPC_ADC->ADCR |= ((1 << 3) | (1 << 16) | (1 << 21));
		LPC_ADC->ADCR &= ~(7 << 24); //clear start bits
		//LPC_ADC->ADCR = 0x002100FF;

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
//	#if 0
//    scheduler_add_task(new task_1(PRIORITY_MEDIUM));
//    scheduler_add_task(new task_2(PRIORITY_MEDIUM));
//	#endif
//#if 0
//    event_group = xEventGroupCreate();
//    scheduler_add_task(new producer_task(PRIORITY_MEDIUM));
//    scheduler_add_task(new consumer_task(PRIORITY_MEDIUM));
//    scheduler_add_task(new watchdog_task(PRIORITY_HIGH));
//    scheduler_add_task(new cpu_usage_task(PRIORITY_CRITICAL));
//#endif
    scheduler_add_task(new ir_task(PRIORITY_MEDIUM));
    /* Change "#if 0" to "#if 1" to run period tasks; @see period_callbacks.cpp */
    #if 1
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

