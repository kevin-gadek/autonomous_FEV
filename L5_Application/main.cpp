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

class producer : public scheduler_task{
public:
	producer(uint8_t priority) : scheduler_task("producer", 2000, priority){}
	bool run(void *p){

		int light[10];
		for(int i = 0; i < 10; i++){
			int light[0] = LS.getRawValue();
			printf("%i\n", light);
			vTaskDelay(1);
		}

		int lightAverage = 0;
		for(int i = 0; i < 10; i++){
		lightAverage += light[0];
		}
		lightAverage = lightAverage/10;
		return true;
	}
	bool init(void){
		return true;
	}
};

class consumer : public scheduler_task{
public:
	consumer(uint8_t priority) : scheduler_task("consumer", 2000, priority){}
	bool run(void *p){

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
    scheduler_add_task(new producer(PRIORITY_MEDIUM));
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
