// header file
#include "integration.h"

// integration thread
#define INTEGRATION_THREAD_STACKSIZE 1024
#define INTEGRATION_THREAD_PRIORITY 0
K_THREAD_STACK_DEFINE(integrationThreadStack, INTEGRATION_THREAD_STACKSIZE); 
struct k_thread integrationThreadData;

// logging
LOG_MODULE_REGISTER(info_integration, LOG_LEVEL_DBG); 

void initialise_bluetooth(void)
{
    int err;
	LOG_DBG("Initialisation in process");
	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Initialisation failed (err %d)", err);
		return;
	}
	LOG_DBG("Initialisation successful");
}

/*
 observes the bluetooth signals that are inputted to the mobile node.
 */
void integration_thread(void* arg1, void* arg2, void* arg3) 
{
    return;
}

void create_integration_thread(void)
{
    // integration thread
    k_thread_create(
        &integrationThreadData,                            // pointer to thread data structure
        integrationThreadStack,                            // stack area allocated for thread 
        K_THREAD_STACK_SIZEOF(integrationThreadStack),     // total size of stack
        integration_thread,                                         // thread function 
        NULL,                                           // arg 1
        NULL,                                           // arg 2
        NULL,                                           // arg 3
        INTEGRATION_THREAD_PRIORITY,                       // thread priority          
        0,                                              // thread options
        K_NO_WAIT                                       // start without waiting
    ); 
}