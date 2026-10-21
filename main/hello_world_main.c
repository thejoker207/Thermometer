#include <stdio.h>
#include <math.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"

// --- NTC KONSTANTEN ---
#define BETA 3435.0f        
#define R0 10000.0f         
#define T0 298.15f          
#define R_FIXED 10000.0f    

// --- PIN-ZUORDNUNG DIGIT 1 (Ziffer links) ---
#define D1_A GPIO_NUM_25
#define D1_B GPIO_NUM_26
#define D1_C GPIO_NUM_27
#define D1_D GPIO_NUM_14
#define D1_E GPIO_NUM_13
#define D1_F GPIO_NUM_23
#define D1_G GPIO_NUM_22
#define D1_DP GPIO_NUM_19
#define DIG_1_COMMON GPIO_NUM_14 // Anzeigenpin 14

// --- PIN-ZUORDNUNG DIGIT 2 (Ziffer rechts) ---
#define D2_A GPIO_NUM_18
#define D2_B GPIO_NUM_5
#define D2_C GPIO_NUM_17
#define D2_D GPIO_NUM_16
#define D2_E GPIO_NUM_4
#define D2_F GPIO_NUM_21
#define D2_G GPIO_NUM_33
#define D2_DP GPIO_NUM_2
#define DIG_2_COMMON GPIO_NUM_13 // Anzeigenpin 13

// ADC Handles
static adc_oneshot_unit_init_cfg_t adcconf; 
static adc_oneshot_unit_handle_t adc_h; 
static adc_cali_handle_t cal_h;

// Hilfsarray für 7-Segment-Muster (0 bis 9)
// Index = Ziffer, Bit-Muster für Segmente: A, B, C, D, E, F, G (1 = an, 0 = aus)
const uint8_t segment_map[10] = {
    0b00111111, // 0 -> A, B, C, D, E, F an
    0b00000110, // 1 -> B, C an
    0b01011011, // 2 -> A, B, D, E, G an
    0b01001111, // 3 -> A, B, C, D, G an
    0b01100110, // 4 -> B, C, F, G an
    0b01101101, // 5 -> A, C, D, F, G an
    0b01111101, // 6 -> A, C, D, E, F, G an
    0b00000111, // 7 -> A, B, C an
    0b01111111, // 8 -> Alle an
    0b0110111f  // 9 -> A, B, C, D, F, G an
};

// Funktion zum Initialisieren aller Display-Pins als Ausgänge
void init_display_pins(void) {
    uint64_-bit pin_mask = (1ULL << D1_A) | (1ULL << D1_B) | (1ULL << D1_C) | (1ULL << D1_D) |
                          (1ULL << D1_E) | (1ULL << D1_F) | (1ULL << D1_G) | (1ULL << D1_DP) |
                          (1ULL << D2_A) | (1ULL << D2_B) | (1ULL << D2_C) | (1ULL << D2_D) |
                          (1ULL << D2_E) | (1ULL << D2_F) | (1ULL << D2_G) | (1ULL << D2_DP) |
                          (1ULL << DIG_1_COMMON) | (1ULL << DIG_2_COMMON);

    gpio_config_t io_conf = {
        .pin_bit_mask = pin_mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
}

// Funktion zum Anzeigen einer Ziffer auf Digit 1 oder Digit 2
void set_digit(int digit_num, int number, int show_dp) {
    uint8_t pattern = segment_map[number];

    if (digit_num == 1) {
        gpio_set_level(D1_A, (pattern >> 0) & 1);
        gpio_set_level(D1_B, (pattern >> 1) & 1);
        gpio_set_level(D1_C, (pattern >> 2) & 1);
        gpio_set_level(D1_D, (pattern >> 3) & 1);
        gpio_set_level(D1_E, (pattern >> 4) & 1);
        gpio_set_level(D1_F, (pattern >> 5) & 1);
        gpio_set_level(D1_G, (pattern >> 6) & 1);
        gpio_set_level(D1_DP, show_dp ? 1 : 0);
    } else if (digit_num == 2) {
        gpio_set_level(D2_A, (pattern >> 0) & 1);
        gpio_set_level(D2_B, (pattern >> 1) & 1);
        gpio_set_level(D2_C, (pattern >> 2) & 1);
        gpio_set_level(D2_D, (pattern >> 3) & 1);
        gpio_set_level(D2_E, (pattern >> 4) & 1);
        gpio_set_level(D2_F, (pattern >> 5) & 1);
        gpio_set_level(D2_G, (pattern >> 6) & 1);
        gpio_set_level(D2_DP, show_dp ? 1 : 0);
    }
}

void clear_display(void) {
    // Alle Segmente ausschalten
    gpio_set_level(D1_A, 0); gpio_set_level(D1_B, 0); gpio_set_level(D1_C, 0); gpio_set_level(D1_D, 0);
    gpio_set_level(D1_E, 0); gpio_set_level(D1_F, 0); gpio_set_level(D1_G, 0); gpio_set_level(D1_DP, 0);
    gpio_set_level(D2_A, 0); gpio_set_level(D2_B, 0); gpio_set_level(D2_C, 0); gpio_set_level(D2_D, 0);
    gpio_set_level(D2_E, 0); gpio_set_level(D2_F, 0); gpio_set_level(D2_G, 0); gpio_set_level(D2_DP, 0);
}

void app_main (void)
{
    adc_oneshot_chan_cfg_t meinADCkanal;
    adc_cali_line_fitting_config_t KalibrationsKonfiguration;
    int adcRohwert;
    int calivoltage;

    printf("Thermometer mit 7-Segment-Anzeige gestartet!\n");
    
    // Display-Pins initialisieren
    init_display_pins();
    clear_display();

    // ADC Unit konfigurieren
    adcconf.unit_id = ADC_UNIT_1;
    adcconf.ulp_mode = ADC_ULP_MODE_DISABLE;
    adcconf.clk_src = ADC_RTC_CLK_SRC_DEFAULT;
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adcconf, &adc_h));
    
    // NTC ist an GPIO 32 angeschlossen -> entspricht ADC1 Kanal 4
    meinADCkanal.atten = ADC_ATTEN_DB_12; 
    meinADCkanal.bitwidth = ADC_BITWIDTH_12;
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_h, ADC_CHANNEL_4, &meinADCkanal));
    
    // Kalibrierung konfigurieren
    KalibrationsKonfiguration.unit_id = ADC_UNIT_1;
    KalibrationsKonfiguration.atten = ADC_ATTEN_DB_12;
    KalibrationsKonfiguration.bitwidth = ADC_BITWIDTH_12; 
    KalibrationsKonfiguration.default_vref = 0; 

    adc_cali_create_scheme_line_fitting(&KalibrationsKonfiguration, &cal_h);
    
    while(1)
    {
        // 1. ADC Wert einlesen (GPIO 32 / Kanal 4)
        adc_oneshot_read(adc_h, ADC_CHANNEL_4, &adcRohwert);
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(cal_h, adcRohwert, &calivoltage));
        
        float voltage_v = (float)calivoltage / 1000.0f;
        float temperatur_celsius = 25.0f; 

        // 2. Temperatur über NTC-Spannungsteiler berechnen
        if (voltage_v > 0.05f && voltage_v < 3.25f) {
            float r_ntc = R_FIXED * (voltage_v / (3.3f - voltage_v));

            float steinhart = r_ntc / R0;              
            steinhart = log(steinhart);          
            steinhart /= BETA;                   
            steinhart += 1.0f / T0;              
            steinhart = 1.0f / steinhart;        
            temperatur_celsius = steinhart - 273.15f; 
        }

        // Aufteilen in Vorkomma (z.B. 23) und Nachkommastelle (z.B. 4)
        int ganzzahl = (int)temperatur_celsius;
        int zehner = ganzzahl / 10;
        int einer = ganzzahl % 10;
        int kommastelle = (int)((temperatur_celsius - (float)ganzzahl) * 10.0f);
        if (kommastelle < 0) kommastelle = -kommastelle;

        printf("Temperatur: %.2f °C (Ganze: %d, Komma: %d)\n", temperatur_celsius, ganzzahl, kommastelle);

        // --- SCHRITT 1: Normale Temperatur anzeigen (1 Sekunde) ---
        // Zehnerstelle auf Digit 1, Einerstelle auf Digit 2
        set_digit(1, zehner, 0);
        set_digit(2, einer, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));

        // --- SCHRITT 2: Kommastelle anzeigen (1 Sekunde) ---
        // Z.B. Digit 1 zeigt nichts oder 0, Digit 2 zeigt die Nachkommastelle mit Punkt
        clear_display();
        set_digit(2, kommastelle, 1); // Mit Dezimalpunkt (DP)
        vTaskDelay(pdMS_TO_TICKS(1000));

        // --- SCHRITT 3: Pause (2 Sekunden) ---
        clear_display();
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}