// header files
#include "mouse.h"
#include "integration.h"
#include "ultrasonic.h"

// main execution
int main(void)
{
    initialise_bluetooth();

    // create_integration_thread();
    create_mouse_thread();
    // create_ultrasonic_thread();
}