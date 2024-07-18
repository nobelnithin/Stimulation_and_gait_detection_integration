#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <math.h>
#include "driver/ledc.h"

#include "ssd1306.h"
#include "font8x8_basic.h"

#define THRESH_LOW -1.5
#define THRESH_HIGH 0

#define TAG "SSD1306"
SSD1306_t dev;

xQueueHandle FIRE_ONQUEUE;
xQueueHandle FIRE_OFFQUEUE;

#define IN1 GPIO_NUM_3
#define IN2 GPIO_NUM_5
#define EN1 GPIO_NUM_4
#define EN2 GPIO_NUM_6
#define nRESET GPIO_NUM_34
#define nSLEEP GPIO_NUM_33

int16_t accel_x, accel_y, accel_z;
int16_t gyro_x, gyro_y, gyro_z;

static esp_timer_handle_t fire_timer;
static bool fire_on = false;
static bool below_neg_15 = false;
bool t_detect_flag = true;

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (21) // Define the output GPIO
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY               (0) // 8192 for 100%. To be decided.
#define LEDC_FREQUENCY          (5000) // Frequency in Hertz. Set frequency at 5 kHz


int strength_current = 0;
float angle_roll = 0;

int STIMStrength[] = {0, 819, 1638, 2457, 3276, 4095, 4914, 5733, 6552, 7371, 8190};

int freq_list[11] = {0,1,2,3,4,5,6,7,8,9};

bool trough_detected(float curr_val, float prev_val)
{
	if(curr_val > prev_val)
		return true;
	return false;
}


static void example_ledc_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 5 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO,
        .duty           = LEDC_DUTY, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}


void STIMTask(void *params)
{
    example_ledc_init();
    gpio_set_direction(IN1, GPIO_MODE_OUTPUT);
    gpio_set_direction(IN2, GPIO_MODE_OUTPUT);
    gpio_set_direction(EN1, GPIO_MODE_OUTPUT);
    gpio_set_direction(EN2, GPIO_MODE_OUTPUT);
    gpio_set_direction(nRESET, GPIO_MODE_OUTPUT);
    gpio_set_direction(nSLEEP, GPIO_MODE_OUTPUT);
    gpio_set_level(nRESET, 1);
    gpio_set_level(nSLEEP, 1);
    gpio_set_level(EN1, 1);
    gpio_set_level(EN2, 1);
    float btn_num=0.0;
    int i;
    ESP_LOGI("NO TAG","STIMTask setup complete");

    while(1)
    {

                gpio_set_level(IN1, 0);
                gpio_set_level(IN2, 0);
                ESP_LOGI("NO TAG","Stimulation fired : driver off for 2 sec");
                ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0));
                ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));

        if(xQueueReceive(FIRE_ONQUEUE,&btn_num,portMAX_DELAY))
        {
                printf("Stimualtion on\n");
                ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL,STIMStrength[1]));
                ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
                ESP_LOGI("NO TAG ","Driver on, strength set : %d \n", STIMStrength[1]);
                for(i= 0;i<20;i++)
                {

                    gpio_set_level(IN1, 1);
                    gpio_set_level(IN2, 0);
                    vTaskDelay(10/ portTICK_PERIOD_MS);
                    gpio_set_level(IN1, 0);
                    gpio_set_level(IN2, 1);
                    vTaskDelay(10/ portTICK_PERIOD_MS);
        
                    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0));
                    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
     
                    gpio_set_level(IN1, 0);
                    gpio_set_level(IN2, 0);
                    vTaskDelay(12.5/ portTICK_PERIOD_MS);
                    
                    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, STIMStrength[1]));
                    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));

            }

            if(xQueueReceive(FIRE_OFFQUEUE,&btn_num,portMAX_DELAY))
                {
                    gpio_set_level(IN1, 0);
                    gpio_set_level(IN2, 0);
                    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0));
                    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
                    printf("Stimulation off\n");
                    xQueueReset(FIRE_OFFQUEUE);

                }

            xQueueReset(FIRE_ONQUEUE);
            
        }
                

                vTaskDelay(200 / portTICK_PERIOD_MS);
                }


}


void fire_off_callback(void* arg) {
    printf("-----Fire OFF (Timer)----\n");
    gpio_set_level(IN1, 0);
    gpio_set_level(IN2, 0);
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
    printf("Stimulation off(timer)\n");
    ssd1306_clear_screen(&dev, false);
    xQueueSend(FIRE_OFFQUEUE,&angle_roll,NULL);
    fire_on = false;
    below_neg_15 = false;
    t_detect_flag = true;
    vTaskDelay(100 / portTICK_PERIOD_MS);
}

void gait_detection(void *params) {

    float prev_value = 0.0;
	float curr_value = 0.0;
    float fire_angle = 0;
    while (1) {
        mpu6050_read_data(&accel_x, &accel_y, &accel_z, &gyro_x, &gyro_y, &gyro_z);
        angle_roll = atan2(accel_z, sqrt(accel_x * accel_x + accel_y * accel_y)) * 180.0 / M_PI;
        prev_value = curr_value;
		curr_value = angle_roll;
        if(curr_value < THRESH_LOW)
		{
			if(trough_detected(curr_value, prev_value) && t_detect_flag && !fire_on && angle_roll<-20)
			{
				//stim on
                below_neg_15 = true; 
				t_detect_flag = false;
                fire_angle = prev_value+5.0;
                // printf("Previous Value: %.2f   Fire angle%.2f\n",prev_value,fire_angle);

			}
		}


        if (below_neg_15 && angle_roll > fire_angle && !fire_on) {
            ssd1306_clear_screen(&dev, true);         
            printf("-----Fire ON----\n");  
            xQueueSend(FIRE_ONQUEUE,&angle_roll,NULL);
            fire_on = true;
            below_neg_15 = false; // Reset monitoring flag after firing starts
            esp_timer_start_once(fire_timer, 1000000); // 1 second in microseconds
        }

        if (angle_roll > (fire_angle+50.0) && fire_on) {
            printf("-----Fire OFF----\n");
            xQueueSend(FIRE_OFFQUEUE,&angle_roll,NULL);
            ssd1306_clear_screen(&dev, false);
            fire_on = false;
            t_detect_flag = true;
            esp_timer_stop(fire_timer);
        }

        vTaskDelay(100 / portTICK_PERIOD_MS); // Delay for 100 milliseconds
    }
}

void app_main(void) {
#if CONFIG_I2C_INTERFACE
    ESP_LOGI(TAG, "INTERFACE is i2c");
    ESP_LOGI(TAG, "CONFIG_SDA_GPIO=%d", CONFIG_SDA_GPIO);
    ESP_LOGI(TAG, "CONFIG_SCL_GPIO=%d", CONFIG_SCL_GPIO);
    ESP_LOGI(TAG, "CONFIG_RESET_GPIO=%d", CONFIG_RESET_GPIO);
    i2c_master_init(&dev, CONFIG_SDA_GPIO, CONFIG_SCL_GPIO, CONFIG_RESET_GPIO);
#endif // CONFIG_I2C_INTERFACE

#if CONFIG_FLIP
    dev._flip = true;
    ESP_LOGW(TAG, "Flip upside down");
#endif

#if CONFIG_SSD1306_128x64
    ESP_LOGI(TAG, "Panel is 128x64");
    ssd1306_init(&dev, 128, 64);
    mpu6050_init(MPU6050_PWR_MGMT_1);
#endif // CONFIG_SSD1306_128x64

#if CONFIG_SSD1306_128x32
    ESP_LOGI(TAG, "Panel is 128x32");
    ssd1306_init(&dev, 128, 32);
#endif // CONFIG_SSD1306_128x32

    ssd1306_contrast(&dev, 0xff);
    ssd1306_clear_screen(&dev, false);

    FIRE_ONQUEUE = xQueueCreate(10, sizeof(int));
    FIRE_OFFQUEUE = xQueueCreate(10, sizeof(int));

    const esp_timer_create_args_t fire_timer_args = {
        .callback = &fire_off_callback,
        .name = "fire_timer"
    };

    if (esp_timer_create(&fire_timer_args, &fire_timer) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create fire timer");
        return;
    }

    xTaskCreate(gait_detection, "Gait", 8000, NULL, 1, NULL);
    xTaskCreate(STIMTask, "STIMUL", 8000, NULL, 1, NULL);
}