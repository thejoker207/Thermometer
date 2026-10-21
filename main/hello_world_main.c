/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_adc/adc_oneshot.h"
static adc_oneshot_unit_init_cfg_t adcconf; 
static adc_oneshot_unit_handle_t adc_h; //Speichert Handle ode rInstanz der ADC Baugruppe
static adc_cali_handle_t cal_h;


void app_main (void)
{
    adc_oneshot_chan_cfg_t meinADCkanal;
    adc_cali_line_fitting_config_t KalibrationsKonfiguration;
    int adcRohwert;
    int calivoltage;
    printf("Hello World!\n");
    adcconf.unit_id = ADC_UNIT_1;
    adcconf.ulp_mode = ADC_ULP_MODE_DISABLE;
    adcconf.clk_src = ADC_RTC_CLK_SRC_DEFAULT;
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adcconf,&adc_h));
    meinADCkanal.atten = ADC_ATTEN_DB_12; 
    meinADCkanal.bitwidth = ADC_BITWIDTH_12;
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_h, ADC_CHANNEL_0, &meinADCkanal));
    KalibrationsKonfiguration.unit_id = ADC_UNIT_1;
    KalibrationsKonfiguration.atten = ADC_ATTEN_DB_12;
    KalibrationsKonfiguration.bitwidth = ADC_BITWIDTH_12; 
    KalibrationsKonfiguration.default_vref = 0; 

    adc_cali_create_scheme_line_fitting(&KalibrationsKonfiguration, &cal_h);
    
    //test


    while(1)
    {
        printf("waiting\n");
        adc_oneshot_read(adc_h,ADC_CHANNEL_0, &adcRohwert);
        printf("ADC - Rohwert: %d \n", adcRohwert);
        printf("ADC - Spannung: %f \n", (float)adcRohwert/4095.0f*3.3f);
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(cal_h, adcRohwert,&calivoltage ));
        printf("ADC - Spannung Kalibriert: %f \n", (float)calivoltage/1000.0f);
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Warten 1s
        
    
    }

}
#if 0

//kommentar
    /* Print chip information */
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), %s%s%s%s, ",
           CONFIG_IDF_TARGET,
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
           (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
           (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    printf("silicon revision v%d.%d, ", major_rev, minor_rev);
    if(esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
        printf("Get flash size failed");
        return;
    }

    printf("%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());

    for (int i = 10; i >= 0; i--) {
        printf("Restarting in %d seconds...\n", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    printf("Restarting now.\n");
    fflush(stdout);
    esp_restart();
}
#endif