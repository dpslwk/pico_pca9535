#include <stdio.h>
#include <stdint.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"

#include "FreeRTOS.h"
#include "task.h"

#include "pca9535.h"

// Standard Task priority
#define MAIN_TASK_STACK_SIZE (1024 * 2)
static const char *MAIN_TASK_NAME = "MainThread";
#define MAIN_TASK_PRIORITY ( tskIDLE_PRIORITY + 1UL )

void clearscreen()
{
    printf("\033[2J\033[1;1H");
}


void main_task(void *params)
{
    printf("Main task: start\n");

    gpio_init(10);
    gpio_set_dir(10, GPIO_OUT);

    // This example will use I2C0 on the default SDA and SCL pins (4, 5 on a Pico)
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    PICO_PCA9535 bus1(i2c_default, 0x20);
    PICO_PCA9535 bus2(i2c_default, 0x21);

    bus1.set_port_direction(0, 0xFF); // set bank 0 to be inputs
    bus1.set_port_direction(1, 0xFF); // set bank 1 to be inputs

    bus2.set_port_direction(0, 0xFF); // set bank 0 to be inputs
    bus2.set_port_direction(1, 0xFF); // set bank 1 to be inputs

    printf("Main task: Entering loop\n");
    while (true) {
        gpio_put(10, ! gpio_get(10));

        clearscreen();
        printf("Bus 1 Pin 1:  %x        Bus 2 Pin 1:  %x\n", bus1.read_pin(1), bus2.read_pin(1));
        printf("Bus 1 Pin 2:  %x        Bus 2 Pin 2:  %x\n", bus1.read_pin(2), bus2.read_pin(2));
        printf("Bus 1 Pin 3:  %x        Bus 2 Pin 3:  %x\n", bus1.read_pin(3), bus2.read_pin(3));
        printf("Bus 1 Pin 4:  %x        Bus 2 Pin 4:  %x\n", bus1.read_pin(4), bus2.read_pin(4));
        printf("Bus 1 Pin 5:  %x        Bus 2 Pin 5:  %x\n", bus1.read_pin(5), bus2.read_pin(5));
        printf("Bus 1 Pin 6:  %x        Bus 2 Pin 6:  %x\n", bus1.read_pin(6), bus2.read_pin(6));
        printf("Bus 1 Pin 7:  %x        Bus 2 Pin 7:  %x\n", bus1.read_pin(7), bus2.read_pin(7));
        printf("Bus 1 Pin 8:  %x        Bus 2 Pin 8:  %x\n", bus1.read_pin(8), bus2.read_pin(8));
        printf("Bus 1 Pin 9:  %x        Bus 2 Pin 9:  %x\n", bus1.read_pin(9), bus2.read_pin(9));
        printf("Bus 1 Pin 10: %x        Bus 2 Pin 10: %x\n", bus1.read_pin(10), bus2.read_pin(10));
        printf("Bus 1 Pin 11: %x        Bus 2 Pin 11: %x\n", bus1.read_pin(11), bus2.read_pin(11));
        printf("Bus 1 Pin 12: %x        Bus 2 Pin 12: %x\n", bus1.read_pin(12), bus2.read_pin(12));
        printf("Bus 1 Pin 13: %x        Bus 2 Pin 13: %x\n", bus1.read_pin(13), bus2.read_pin(13));
        printf("Bus 1 Pin 14: %x        Bus 2 Pin 14: %x\n", bus1.read_pin(14), bus2.read_pin(14));
        printf("Bus 1 Pin 15: %x        Bus 2 Pin 15: %x\n", bus1.read_pin(15), bus2.read_pin(15));
        printf("Bus 1 Pin 16: %x        Bus 2 Pin 16: %x\n", bus1.read_pin(16), bus2.read_pin(16));

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void vLaunch()
{
    TaskHandle_t task;

    xTaskCreate(main_task, MAIN_TASK_NAME, MAIN_TASK_STACK_SIZE, NULL, MAIN_TASK_PRIORITY, &task);

    // more tasks, they can set vTaskCoreAffinitySet if needed

    // Start the tasks and timer running.
    vTaskStartScheduler();
}

int main() {
    stdio_init_all();
    sleep_ms(2000);
    printf("GO\n");


    vLaunch();

    return 0;
}
