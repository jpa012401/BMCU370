#include <Arduino.h>
#include "main.h"

#include "BambuBus.h"
#include "Adafruit_NeoPixel.h"

extern void debug_send_run();
// For testing with 8 LEDs
// #define LED_PA11_NUM 8
#define LED_PA11_NUM 2
#define LED_PA8_NUM 2
#define LED_PB1_NUM 2
#define LED_PB0_NUM 2
#define LED_PD1_NUM 1

// Channel RGB objects, strip_channel[Chx], 0~3 for PA11/PA8/PB1/PB0
Adafruit_NeoPixel strip_channel[4] = {
    Adafruit_NeoPixel(LED_PA11_NUM, PA11, NEO_GRB + NEO_KHZ800),
    Adafruit_NeoPixel(LED_PA8_NUM, PA8, NEO_GRB + NEO_KHZ800),
    Adafruit_NeoPixel(LED_PB1_NUM, PB1, NEO_GRB + NEO_KHZ800),
    Adafruit_NeoPixel(LED_PB0_NUM, PB0, NEO_GRB + NEO_KHZ800)
};
// Mainboard 5050 RGB
Adafruit_NeoPixel strip_PD1(LED_PD1_NUM, PD1, NEO_GRB + NEO_KHZ800);

void RGB_Set_Brightness() {
    // Brightness value 0-255
    // Mainboard brightness
    strip_PD1.setBrightness(35);
    // Channel 1 RGB
    strip_channel[0].setBrightness(15);
    // Channel 2 RGB
    strip_channel[1].setBrightness(15);
    // Channel 3 RGB
    strip_channel[2].setBrightness(15);
    // Channel 4 RGB
    strip_channel[3].setBrightness(15);
}

void RGB_init() {
    strip_PD1.begin();
    strip_channel[0].begin();
    strip_channel[1].begin();
    strip_channel[2].begin();
    strip_channel[3].begin();
}
void RGB_show_data() {
    strip_PD1.show();
    strip_channel[0].show();
    strip_channel[1].show();
    strip_channel[2].show();
    strip_channel[3].show();
}

// Stores RGB colors for filament in 4 channels
uint8_t channel_colors[4][4] = {
    {0xFF, 0xFF, 0xFF, 0xFF},
    {0xFF, 0xFF, 0xFF, 0xFF},
    {0xFF, 0xFF, 0xFF, 0xFF},
    {0xFF, 0xFF, 0xFF, 0xFF}
};

// Stores RGB colors for 4 channels, avoids communication failure from frequent color refreshes
uint8_t channel_runs_colors[4][2][3] = {
    // R,G,B  ,, R,G,B
    {{1, 2, 3}, {1, 2, 3}}, // Channel 1
    {{3, 2, 1}, {3, 2, 1}}, // Channel 2
    {{1, 2, 3}, {1, 2, 3}}, // Channel 3
    {{3, 2, 1}, {3, 2, 1}}  // Channel 4
};

extern void BambuBUS_UART_Init();
extern void send_uart(const unsigned char *data, uint16_t length);

void setup()
{
    WWDG_DeInit();
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, DISABLE); // Disable watchdog
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_PD01, ENABLE);
    // Initialize RGB LEDs
    RGB_init();
    // Update RGB display
    RGB_show_data();
    // Set RGB brightness - this maintains color ratio while limiting max value
    RGB_Set_Brightness();

    BambuBus_init();
    DEBUG_init();
    Motion_control_init();
    delay(1);
}

void Set_MC_RGB(uint8_t channel, int num, uint8_t R, uint8_t G, uint8_t B)
{
    int set_colors[3] = {R, G, B};
    bool is_new_colors = false;

    for (int colors = 0; colors < 3; colors++)
    {
        if (channel_runs_colors[channel][num][colors] != set_colors[colors]) {
            channel_runs_colors[channel][num][colors] = set_colors[colors]; // Record new color
            is_new_colors = true; // Color has been updated
        }
    }
    // Check each channel, if changed, update it
    if (is_new_colors) {
        strip_channel[channel].setPixelColor(num, strip_channel[channel].Color(R, G, B));
        strip_channel[channel].show(); // Display new color
        is_new_colors = false; // Reset state
    }
}

bool MC_STU_ERROR[4] = {false, false, false, false};
void Show_SYS_RGB(int BambuBUS_status)
{
    // Update mainboard RGB LED
    if (BambuBUS_status == -1) // Offline
    {
        strip_PD1.setPixelColor(0, strip_PD1.Color(8, 0, 0)); // Red
        strip_PD1.show();
    }
    else if (BambuBUS_status == 0) // Online
    {
        strip_PD1.setPixelColor(0, strip_PD1.Color(8, 9, 9)); // White
        strip_PD1.show();
    }
    // Update error channels, light up red
    for (int i = 0; i < 4; i++)
    {
        if (MC_STU_ERROR[i])
        {
            // Red
            strip_channel[i].setPixelColor(0, strip_channel[i].Color(255, 0, 0));
            strip_channel[i].show(); // Display new color
        }
    }
}

BambuBus_package_type is_first_run = BambuBus_package_type::NONE;
void loop()
{
    while (1)
    {
        BambuBus_package_type stu = BambuBus_run();
        // int stu =-1;
        static int error = 0;
        bool motion_can_run = false;
        uint16_t device_type = get_now_BambuBus_device_type();
        if (stu != BambuBus_package_type::NONE) // have data/offline
        {
            motion_can_run = true;
            if (stu == BambuBus_package_type::ERROR) // offline
            {
                error = -1;
                // Offline - red light
            }
            else // have data
            {
                error = 0;
                // if (stu == BambuBus_package_type::heartbeat)
                // {
                // Normal operation - white light
                // }
            }
            // Refresh LEDs every 3 seconds
            static unsigned long last_sys_rgb_time = 0;
            unsigned long now = get_time64();
            if (now - last_sys_rgb_time >= 3000) {
                Show_SYS_RGB(error);
                last_sys_rgb_time = now;
            }
        }
        else
        {
        } // wait for data
        // Log output
        if (is_first_run != stu)
        {
            is_first_run = stu;
            if (stu == BambuBus_package_type::ERROR)
            {                                   // offline
                DEBUG_MY("BambuBus_offline\n"); // Offline
            }
            else if (stu == BambuBus_package_type::heartbeat)
            {
                DEBUG_MY("BambuBus_online\n"); // Online
            }
            else if (device_type == BambuBus_AMS_lite)
            {
                DEBUG_MY("Run_To_AMS_lite\n"); // Online
            }
            else if (device_type == BambuBus_AMS)
            {
                DEBUG_MY("Run_To_AMS\n"); // Online
            }
            else
            {
                DEBUG_MY("Running Unknown ???\n");
            }
        }

        if (motion_can_run)
        {
            Motion_control_run(error);
        }
    }
}
