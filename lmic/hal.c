/*
 * Copyright (c) 2014-2016 IBM Corporation.
 * All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of the <organization> nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "hal.h"

static const char* TAG = "LMIC_HAL";
int x_irq_level = 0;

// Function that gets executed when GPIO interrupt is triggered
void IRAM_ATTR gpio_isr_handler(void* arg){

    // printf("GPIO ISR HANDLER\n");
    // ESP_LOGI(TAG, "ISR handler called with value: %d", gpio_num);

    xTaskCreate(&isr_handler_task, "isr_handler_task", 2048, arg, 8, NULL);
}

// Task for handling interrupt
void isr_handler_task(void *pvParameter){
    uint32_t gpio_num = (uint32_t) pvParameter;
    ESP_LOGI(TAG, "GPIO Interrupt triggered. GPIO_PIN: %d", gpio_num);
    radio_irq_handler((u1_t) gpio_num);
    vTaskDelete(NULL);
}

void io_init(){
    ESP_LOGI(TAG, "Starting initialisation of IO");
    esp_err_t ret;
    // Set output
    gpio_config_t io_config;
    io_config.intr_type = GPIO_PIN_INTR_DISABLE;
    io_config.mode = GPIO_MODE_OUTPUT;
    io_config.pin_bit_mask = ((1 << LORA_RST) | (1 << SPI_CS) );
    io_config.pull_down_en = 0;
    io_config.pull_up_en = 0;
    gpio_config(&io_config);

    io_config.intr_type = GPIO_PIN_INTR_POSEDGE;
    io_config.pin_bit_mask = ((1 << LORA_DO0) | (1 << LORA_DO1) | (1 << LORA_DO2));
    io_config.mode = GPIO_MODE_INPUT;
    io_config.pull_down_en = 1;
    gpio_config(&io_config);

    ret = gpio_install_isr_service(0);
    assert(ret == ESP_OK);

    // Set handlers for interrupts
    ret = gpio_isr_handler_add(LORA_DO0, gpio_isr_handler, (void *) LORA_DO0);
    assert(ret == ESP_OK);
    ret = gpio_isr_handler_add(LORA_DO1, gpio_isr_handler, (void *) LORA_DO1);
    assert(ret == ESP_OK);
    ret = gpio_isr_handler_add(LORA_DO2, gpio_isr_handler, (void *) LORA_DO2);
    assert(ret == ESP_OK);

    ESP_LOGI(TAG, "Finished initialisation of IO");
}

void spi_init(){
    ESP_LOGI(TAG, "Starting initialisation of SPI");
    esp_err_t ret;

    // init master
    spi_bus_config_t buscfg={
        .miso_io_num = SPI_MISO,
        .mosi_io_num = SPI_MOSI,
        .sclk_io_num = SPI_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    // init device
    spi_device_interface_config_t devcfg={
        .clock_speed_hz = 10000000,
        .mode = 1,
        .spics_io_num = SPI_CS,
        .queue_size = 7,
        .flags = SPI_DEVICE_HALFDUPLEX,
    };

    ret = spi_bus_initialize(HSPI_HOST, &buscfg, 1);
    assert(ret==ESP_OK);

    ret = spi_bus_add_device(HSPI_HOST, &devcfg, &spi_handle);
    assert(ret==ESP_OK);

    ESP_LOGI(TAG, "Finished initialisation of SPI");

}

/*
 * initialize hardware (IO, SPI, TIMER, IRQ).
 */
void hal_init (void){
    ESP_LOGI(TAG, "Starting initialisation of HAL");

    spi_init();
    io_init();

    //TODO: init timer

    ESP_LOGI(TAG, "Finished initialisation of HAL");
}

/*
 * drive radio NSS pin (0=low, 1=high).
 */
void hal_pin_nss (u1_t val){
    //ESP_LOGD(TAG, "Settting NSS Pin to: %d", val);
    if(!val)
        gpio_set_level(SPI_CS, 0);
    else
        gpio_set_level(SPI_CS, 1);
}

/*
 * drive radio RX/TX pins (0=rx, 1=tx).
 */
void hal_pin_rxtx (u1_t val){
    return;
}

/*
 * control radio RST pin (0=low, 1=high, 2=floating)
 */
void hal_pin_rst (u1_t val){
    ESP_LOGI(TAG, "Resetting LORA radio");
    gpio_set_level(LORA_RST, val);
    ESP_LOGI(TAG, "Finished resetting LORA radio");
}

// Write data. first element of data pointer is address
void hal_spi_write (uint8_t *data, int length){
    ESP_LOGI(TAG, "Writing values via SPI with length: %d | address: 0x%X, data: 0x%X", length, address, data[1]);
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = length * 8;
    t.tx_buffer = data;
    esp_err_t ret = spi_device_transmit(spi_handle, &t);
    assert(ret == ESP_OK);

}

//Read data
u1_t hal_spi_read (u1_t address){
    uint8_t buf;
    uint8_t readRegister = (uint8_t) address;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.tx_buffer = &readRegister;
    t.rx_buffer= &buf;
    esp_err_t ret = spi_device_transmit(spi_handle, &t);
    assert(ret == ESP_OK);
    return (u1_t) buf;
}

/*
 * disable all CPU interrupts.
 *   - might be invoked nested
 *   - will be followed by matching call to hal_enableIRQs()
 */
void hal_disableIRQs (void){
    ESP_LOGD(TAG, "Disabling interrupts");
    if(x_irq_level < 1){
        //taskDISABLE_INTERRUPTS();
    }
    x_irq_level++;
}

/*
 * enable CPU interrupts.
 */
void hal_enableIRQs (void){
    ESP_LOGD(TAG, "Enable interrupts");
    if(!x_irq_level){
        //taskENABLE_INTERRUPTS();
    }
    x_irq_level--;
}

/*
 * put system and CPU in low-power mode, sleep until interrupt.
 */
void hal_sleep (void){
    // Dont sleep
}

/*
 * return 32-bit system time in ticks.
 */
u4_t hal_ticks (void){
    uint64_t val;
    timer_get_counter_value(TIMER_GROUP_0, TIMER_1, &val);
    if(val > 0xFFFFFFFF){
        printf("TIMER VALUE: %d\n",(int)val);
    }
    ESP_LOGD(TAG, "Getting time ticks");
    uint32_t t = (uint32_t) val;
    //u4_t result = (u4_t) us2osticks(t);
    return t;
}

s4_t delta_time(u4_t time){
    return (s4_t)(time - hal_ticks());
}

/*
 * busy-wait until specified timestamp (in ticks) is reached.
 */
void hal_waitUntil (u4_t time){
    ESP_LOGI(TAG, "Wait until");
    s4_t delta = delta_time(time);

    while( delta > 2000){
        vTaskDelay(1);
        delta -= 1000;
    } if(delta > 0){
        vTaskDelay(delta / 1000);
    }
    ESP_LOGI(TAG, "Done waiting until");
}

/*
 * check and rewind timer for target time.
 *   - return 1 if target time is close
 *   - otherwise rewind timer for target time or full period and return 0
 */
u1_t hal_checkTimer (u4_t targettime){
    // Not used
    return 1;
}

/*
 * perform fatal failure action.
 *   - called by assertions
 *   - action could be HALT or reboot
 */
void hal_failed (void){
    ESP_LOGE(TAG, "Fatal error occurred");
    vTaskDelay(4000 / portTICK_PERIOD_MS);
    esp_restart();
}
