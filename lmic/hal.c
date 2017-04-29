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

void time_init(){
    ESP_LOGI(TAG, "Starting initialisation of timer");
    int timer_group = TIMER_GROUP_0;
    int timer_idx = TIMER_1;
    timer_config_t config;
    config.alarm_en = 0;
    config.auto_reload = 0;
    config.counter_dir = TIMER_COUNT_UP;
    config.divider = 120;
    config.intr_type = 0;
    config.counter_en = TIMER_PAUSE;
    /*Configure timer*/
    timer_init(timer_group, timer_idx, &config);
    /*Stop timer counter*/
    timer_pause(timer_group, timer_idx);
    /*Load counter value */
    timer_set_counter_value(timer_group, timer_idx, 0x0);
    /*Start timer counter*/
    timer_start(timer_group, timer_idx);

    ESP_LOGI(TAG, "Finished initalisation of timer");
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

    //set input
    io_config.pin_bit_mask = ((1 << LORA_DO0) | (1 << LORA_DO1) | (1 << LORA_DO2));
    io_config.mode = GPIO_MODE_INPUT;
    gpio_config(&io_config);

    ESP_LOGI(TAG, "Finished initialisation of IO");
}

static int NUM_DIO = 3;
bool dio_states[3];

void hal_io_check() {
    if (dio_states[0] != gpio_get_level(LORA_DO0)) {
        dio_states[0] = !dio_states[0];
        if (dio_states[0])
            printf("Fired IRQ0\n");
            radio_irq_handler(0);
    }

    if (dio_states[1] != gpio_get_level(LORA_DO1)) {
        dio_states[1] = !dio_states[1];
        if (dio_states[1])
            printf("Fired IRQ1\n");
            radio_irq_handler(1);
    }

    if (dio_states[2] != gpio_get_level(LORA_DO2)) {
        dio_states[2] = !dio_states[2];
        if (dio_states[2])
            printf("Fired IRQ2\n");
            radio_irq_handler(2);
    }
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
        .spics_io_num = -1,
        .queue_size = 7,
        //.flags = SPI_DEVICE_HALFDUPLEX,
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
    time_init();

    ESP_LOGI(TAG, "Finished initialisation of HAL");
}

/*
 * drive radio NSS pin (0=low, 1=high).
 */
void hal_pin_nss (u1_t val){
    //ESP_LOGI(TAG, "Settting NSS Pin to: %d", val);
    gpio_set_level(SPI_CS, val);
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
    if(val == 0 || val == 1) { // drive pin
      gpio_config_t io_conf;
      io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
      io_conf.mode = GPIO_MODE_OUTPUT;
      io_conf.pin_bit_mask = (1 << LORA_RST);
      io_conf.pull_down_en = 0;
      io_conf.pull_up_en = 0;
      gpio_config(&io_conf);

      gpio_set_level(LORA_RST, val);
    } else { // keep pin floating
      gpio_config_t io_conf;
      io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
      io_conf.mode = GPIO_MODE_INPUT;
      io_conf.pin_bit_mask = (1 << LORA_RST);
      io_conf.pull_down_en = 0;
      io_conf.pull_up_en = 0;
      gpio_config(&io_conf);
    }
    ESP_LOGI(TAG, "Finished resetting LORA radio");
}

// Write data. first element of data pointer is address
u1_t hal_spi(u1_t data){
    uint8_t rxData = 0;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.rxlength = 8;
    t.tx_buffer = &data;
    t.rx_buffer = &rxData;
    esp_err_t ret = spi_device_transmit(spi_handle, &t);
    assert(ret == ESP_OK);

    return (u1_t) rxData;
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
    if(--x_irq_level == 0){
        //taskENABLE_INTERRUPTS();
        hal_io_check();
    }
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
