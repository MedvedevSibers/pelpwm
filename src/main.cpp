#include <lvgl.h>
#include <TFT_eSPI.h>
#include <ui.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
extern "C" {
  #include "freertos/FreeRTOS.h"
  #include "freertos/timers.h"
}

#define FANPIN 13
#define FANPIN2 16
#define PWMFREQ 25000
#define PELPIN 27
#define PELPIN2 4

#define PUMPPIN 14
#define PUMPFREQ 5000

#define B 3950
#define SERIAL_R 10000
#define THERMISTOR_R 10000
#define NOMINAL_T 25

#define radTempPin 35
#define radTempPin2 25
#define waterTempPin 34

TaskHandle_t TaskPel_t;
TaskHandle_t TaskPel2_t;
TaskHandle_t TaskFan_t;
TaskHandle_t TaskFan1_t;
TaskHandle_t TaskFan2_t;
TaskHandle_t TaskPump_t;
TimerHandle_t TimerCooldown_t;
TimerHandle_t TimerCooldown2_t;
QueueHandle_t QueuePump;

Adafruit_SSD1306 display(128, 64, &Wire, -1);

int power = 0;
bool debug = true;
bool pumpstate = false;
bool pumpstate2 = false;
bool pumpflag = false;
int8_t correction = -2;
byte pumpPower = 255;

char errRadOverheat[] = "RADIATOR OVERHEATING, PELTIET POWER SUPPLY DISCONNECTED, PUMP STOPPED";
char errRadTempSensor[] = "Radiator temp sensor malfunction, initializing procedure stopped, check connections and restart the system";
char errWaterTempSensor [] = "Water temp sensor malfunction, initializing procedure stopped, check connections and restart the system";
char msgSuccessStart[] = "Temp sensors initialized, starting...";
char msgHotRadiator [] = "Radiator is still hot";
char msgRadCooled[] = "Radiotor cooled, peltiet power restored, pump power restored";

struct pumpData {
    byte pelNumber;
    bool reqState;
};

pumpData pel;


/*Don't forget to set Sketchbook location in File/Preferencesto the path of your UI project (the parent foder of this INO file)*/

/*Change to your screen resolution*/
static const uint16_t screenWidth  = 320;
static const uint16_t screenHeight = 240;

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[ screenWidth * screenHeight / 10 ];

// TFT_eSPI tft = TFT_eSPI(screenHeight, screenWidth); /* TFT instance */

// #if LV_USE_LOG != 0
// /* Serial debugging */
// void my_print(const char * buf)
// {
//     Serial.printf(buf);
//     Serial.flush();
// }
// #endif

float temp (byte tpin) {
    float tdata;
    char temptext[12];
    int t = analogRead(tpin);
    double voltage = (float)t / 4095.0 * 3.3;
    double Rt = 10 * voltage / (3.3 - voltage); 
    double tempK = 1 / (1 / (273.15 + 25) + log(Rt / 10) / 3950.0);
    double tempCRaw = tempK - 273.15;
    double tempC = tempCRaw + correction;
    // lv_bar_set_value(ui_tempbar, tempC, LV_ANIM_ON);
    // lv_snprintf(temptext, sizeof(temptext), "%d",(int)tempC);
    // lv_label_set_text(ui_tempval, temptext);
    return tempC;
}

int sort_desc(const void *cmp1, const void *cmp2)
{
  int a = *((int *)cmp1);
  int b = *((int *)cmp2);
  return a > b ? -1 : (a < b ? 1 : 0);
}


void errprint (char *err) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 8);
    display.println("/////////////////////");
    display.setCursor(0, 16);
    display.println(err);
    display.display();    
}

void cooldownTimer () {
    int radTemp = temp(radTempPin);
    if (radTemp < 50) {
        vTaskResume (TaskPel_t);
        pel.pelNumber = 1;
        pel.reqState = true;
        xQueueSend(QueuePump, &pel, portMAX_DELAY);
        Serial.println (msgRadCooled);
        errprint (msgRadCooled);
        xTimerStop (TimerCooldown_t, 0);
        xTimerReset (TimerCooldown_t, 0);
        if (debug == true);{
            Serial.println("CDTIMER1 - COOLED AND STARTED");
        }
    }
    else {
        Serial.println (msgHotRadiator);
        errprint(msgHotRadiator);
        Serial.println (radTemp);
        if (debug == true) {
            Serial.println("CDTIMER1 - STILLHOT");
        }
    }
}

void cooldownTimer2 () {
    int radTemp = temp(radTempPin2);
    if (radTemp < 50) {
        vTaskResume (TaskPel_t);
        pel.pelNumber = 2;
        pel.reqState = true;
        xQueueSend(QueuePump, &pel, portMAX_DELAY);
        Serial.println (msgRadCooled);
        errprint (msgRadCooled);
        xTimerStop (TimerCooldown2_t, 0);
        xTimerReset (TimerCooldown2_t, 0);
        if (debug == true);{
            Serial.println("CDTIMER2 - COOLED AND STARTED");
        }
    }
    else {
        Serial.println (msgHotRadiator);
        errprint(msgHotRadiator);
        Serial.println (radTemp);
        if (debug == true) {
            Serial.println("CDTIMER2 - STILLHOT");
        }
    }
}

// bool fancontrol (byte channel, int temp, byte pel) {
//         if (temp < 30) {
//             ledcWrite (channel, 0);
//             if (debug == true) {
//             Serial.println("FANCONTROL - POWER ZERO");
//             }
//             return true;
//         }
//         else if (temp >= 30 && temp <= 70 ) {
//             int pow = map(temp, 30, 70, 60, 255);
//             ledcWrite (channel, pow);
//             if (debug == true) {
//             Serial.println("FANCONTROL - POWER: ");
//             Serial.println(pow);
//             }
//             return true;  
//         }
//         else if (temp > 70) {
//             vTaskSuspend (TaskPel_t);
//             digitalWrite(pel, LOW);
//             ledcWrite (channel, 255);
//             if (debug == true) {
//                 Serial.println(errRadOverheat);
//             }
//             errprint(errRadOverheat);
//             if (channel == 1) {
//                 if (debug == true) {
//                     Serial.println("FANCONTROL - 1 TIMER STARTED");
//                 }
//                 xTimerStart (TimerCooldown_t, 0);
//             }
//             else if (channel == 2) {
//                 if (debug == true) {
//                     Serial.println("FANCONTROL - 2 TIMER STARTED");
//                 }
//                 xTimerStart (TimerCooldown2_t, 0);
//             }
//             return false;
//     }
// }

void TaskPel (void *pvParameters) {
    for (;;) {
        int waterTemp = temp (waterTempPin);
        display.clearDisplay();
        display.setCursor(0, 20);
        display.setTextSize(3);
        display.print(waterTemp);
        display.display();
        if (waterTemp >= 20) {
            Serial.println("cooling");
            digitalWrite (PELPIN, HIGH);
            display.setCursor(90, 20);
            display.print("*");
            display.display();
            pel.pelNumber = 1;
            pel.reqState = true;
            xQueueSend(QueuePump, &pel, portMAX_DELAY);
        }
        else if (waterTemp <= 17) {
            Serial.println("no cooling");
            digitalWrite (PELPIN, LOW);
            pel.pelNumber = 1;
            pel.reqState = false;
            xQueueSend(QueuePump, &pel, portMAX_DELAY);
        }
        vTaskDelay (2000);
    }
}

void TaskPel2 (void *pvParameters) {
    for (;;) {
        int waterTemp = temp (waterTempPin);
        if (waterTemp >= 20) {
            Serial.println("cooling");
            digitalWrite (PELPIN2, HIGH);
            pel.pelNumber = 2;
            pel.reqState = true;
            xQueueSend(QueuePump, &pel, portMAX_DELAY);
        }
        else if (waterTemp <= 17) {
            Serial.println("no cooling");
            digitalWrite (PELPIN2, LOW);
            pel.pelNumber = 2;
            pel.reqState = false;
            xQueueSend(QueuePump, &pel, portMAX_DELAY);
        }
        vTaskDelay (2000);
    }
}

// void TaskFan (void *pvParameters) {
//     for (;;) {
//         int radTemp = temp (radTempPin);
//         if (radTemp < 35) {
//             ledcWrite (0, 0);
//         }
//         else if (radTemp >= 40 && radTemp < 45 ) {
//             ledcWrite (0, 55);
//         }
//         else if (radTemp >= 45 && radTemp < 50 ) {
//             ledcWrite (0, 105);
//         }
//         else if (radTemp >= 50 && radTemp < 60 ) {
//             ledcWrite (0, 185);
//         }
//         else if (radTemp >= 60 && radTemp < 70 ) {
//             ledcWrite (0, 215);
//         }
//         else if (radTemp >= 75 && radTemp < 80 ) {
//             ledcWrite (0, 255);
//         }
//         else if (radTemp > 85) {
//             vTaskSuspend (TaskPel_t);
//             digitalWrite(PELPIN, LOW);
//             digitalWrite(PELPIN2, LOW);
//             ledcWrite (0, 255);
//             ledcWrite(1, 0);
//             Serial.print(errRadOverheat);
//             errprint(errRadOverheat);
//             xTimerStart (TimerCooldown_t, 0);
//         }
//         vTaskDelay(1000);
//     }
// }
// void TaskFan (void *pvParameters) {
//     for (;;) {
//         int radTemp = temp (radTempPin);
//         int radTemp2 = temp (radTempPin2);
//         bool fanstat1 = fancontrol(1, radTemp, PELPIN);
//         if (debug == true) {
//             Serial.println("FANTASK - FIRST RAD STATE:");
//             Serial.print(fanstat1);
//         }
//         bool fanstat2 = fancontrol(2, radTemp2, PELPIN2);
//         if (debug == true) {
//             Serial.println("FANTASK - SECOND RAD STATE:");
//             Serial.print(fanstat2);
//         }
//         if (fanstat1 == false && fanstat2 == false) {
//             pumpstate = false;
//             Serial.println ("Pump stopped");
//         }
//         vTaskDelay(1000);
//     }
// }

void TaskFan1 (void *pvParameters) {
    for (;;) {
        int rtemp = temp (radTempPin);
        if (rtemp < 30) {
            ledcWrite (0, 0);
            if (debug == true) {
            Serial.println("FANCONTROL1 - POWER ZERO");
            }
        }
        else if (rtemp >= 30 && rtemp <= 70 ) {
            int pow = map(rtemp, 30, 70, 60, 255);
            ledcWrite (0, pow);
            if (debug == true) {
            Serial.println("FANCONTROL - POWER: ");
            Serial.println(pow);
            }
        }
        else if (rtemp > 70) {
            vTaskSuspend (TaskPel_t);
            digitalWrite(PELPIN, LOW);
            ledcWrite (0, 255);
            if (debug == true) {
                Serial.println(errRadOverheat);
            }
            errprint(errRadOverheat);
            pel.pelNumber = 1;
            pel.reqState = false;
            xQueueSend(QueuePump, &pel, portMAX_DELAY);
            xTimerStart (TimerCooldown_t, 0);
    }
    vTaskDelay(1000);
    }
}

void TaskFan2 (void *pvParameters) {
    for (;;) {
        int rtemp = temp (radTempPin2);
        if (rtemp < 30) {
            ledcWrite (2, 0);
            if (debug == true) {
            Serial.println("FANCONTROL2 - POWER ZERO");
            }
        }
        else if (rtemp >= 30 && rtemp <= 70 ) {
            int pow = map(rtemp, 30, 70, 60, 255);
            ledcWrite (2, pow);
            if (debug == true) {
            Serial.println("FANCONTROL - POWER: ");
            Serial.println(pow);
            }
        }
        else if (rtemp > 70) {
            vTaskSuspend (TaskPel_t);
            digitalWrite(PELPIN2, LOW);
            ledcWrite (0, 255);
            if (debug == true) {
                Serial.println(errRadOverheat);
            }
            pel.pelNumber = 2;
            pel.reqState = false;
            xQueueSend(QueuePump, &pel, portMAX_DELAY);
            errprint(errRadOverheat);
            xTimerStart (TimerCooldown2_t, 0);
    }
    vTaskDelay(1000);
}
}

void TaskPump (void *pvParameters) {
    struct pumpData received;
    static bool pel1ReqState;
    static bool pel2ReqState;
    for (;;) { 
        xQueueReceive(QueuePump, &received, portMAX_DELAY);
        if (received.pelNumber == 1) {
            pel1ReqState = received.reqState;
        }
        else if (received.pelNumber == 2)  {
            pel2ReqState = received.reqState;
        }
        if (pel1ReqState == true || pel2ReqState == true) {
            digitalWrite (PUMPPIN, HIGH);
            if (debug == true) {
                Serial.println("pump started");
            }
        }
        else if (pel1ReqState == false && pel2ReqState == false) {
            digitalWrite (PUMPPIN, LOW);
            if (debug == true) {
                Serial.println("pump stopped");
            }
        }
        vTaskDelay(100);
    }
}

// void TaskPump (void *pvParameters) {
//     struct pumpData received; -- старая функция помпы
//     static bool pel1ReqState;
//     static bool pel2ReqState;
//     for (;;) { 
//         if (pumpstate == false && pumpflag == true) {
//             digitalWrite (PUMPPIN, LOW);
//             pumpflag = false;
//             if (debug == true) {
//             Serial.println("pump stopped");
//             }
//         }
//         else if (pumpstate == false && pumpflag == false) {
//             if (debug == true) {
//                 Serial.println ("pump already stopped");
//             }
//         }
//         else if (pumpstate == true && pumpflag == false) {
//             digitalWrite(PUMPPIN, HIGH);
//             pumpflag = true;
//             if (debug == true) {
//             Serial.println("pump started");
//             }
//         }
//         else if (pumpstate == true && pumpflag == true); {
//             if (debug == true) {
//             Serial.println("pump already started");
//             }
//         }
//         vTaskDelay(1000);
//     }
// }
// void init () {
//     byte iterations = 30;
//     int wtemps [iterations];
//     int rtemps [iterations];
//     for (int i = 0; i < iterations; i++) {
//         wtemps[i] = temp(waterTempPin);
//         delay(5);
//     }
//     qsort (wtemps, iterations, sizeof(int), sort_desc);
//     int midwtemp = wtemps[iterations / 2];

//     for (int i = 0; i < iterations; i++) {
//         rtemps[i] = temp(radTempPin);
//         delay(5);
//     }
//     qsort (rtemps, iterations, sizeof(int), sort_desc);
//     Serial.println ("Temps");
//     int midrtemp = rtemps[iterations / 2];
//     Serial.println (midwtemp);
//     Serial.println (midrtemp);
//     if (midwtemp < -20 || midwtemp > 60) {
//         Serial.println("Water temp sensor malfunction, initializing procedure stopped");
//         Serial.println("Check connections and restart the system");
//         oled.clear();
//         oled.home();
//         oled.autoPrintln("Water temp sensor malfunction, initializing procedure stopped. Check connections and restart the system");
//         oled.update();
//     }
//     else if (midrtemp < -20 || midrtemp > 100) {
//         Serial.println("Radiator temp sensor malfunction, initializing procedure stopped");
//         Serial.println("Check connections and restart the system");
//         // oled.clear();
//         // oled.home();
//         // oled.autoPrintln("Radiator temp sensor malfunction, initializing procedure stopped. Check connections and restart the system");
//         // oled.update();
//     }
//     else {
//         Serial.println("Temp sensors initialized, starting...");
//         // oled.clear();
//         // oled.home();
//         // oled.autoPrintln("Temp sensors initialized, starting...");
//         // oled.update();
//         ledcWrite (1, pumpPower);
//         xTaskCreatePinnedToCore(TaskFan, "TaskFan", 2048, NULL, 1, &TaskFan_t, 0);
//         xTaskCreatePinnedToCore(TaskPel, "TaskPel", 2048, NULL, 1, &TaskPel_t, 0);
//         TimerCooldown_t = xTimerCreate("TimerCooldown", pdMS_TO_TICKS(20000), pdTRUE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(cooldownTimer));
//     }
// }
/* Display flushing */
// void my_disp_flush( lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p )
// {
//     uint32_t w = ( area->x2 - area->x1 + 1 );
//     uint32_t h = ( area->y2 - area->y1 + 1 );

//     tft.startWrite();
//     tft.setAddrWindow( area->x1, area->y1, w, h );
//     tft.pushColors( ( uint16_t * )&color_p->full, w * h, true );
//     tft.endWrite();

//     lv_disp_flush_ready( disp );
// }

/*Read the touchpad*/
// void my_touchpad_read( lv_indev_drv_t * indev_driver, lv_indev_data_t * data )
// {
//     uint16_t touchX = 0, touchY = 0;

//     bool touched = tft.getTouch( &touchX, &touchY, 600 );

//     if( !touched )
//     {
//         data->state = LV_INDEV_STATE_REL;
//     }
//     else
//     {
//         data->state = LV_INDEV_STATE_PR;

//         /*Set the coordinates*/
//         data->point.x = touchX;
//         data->point.y = touchY;

//         // Serial.print( "Data x " );
//         // Serial.println( touchX );

//         // Serial.print( "Data y " );
//         // Serial.println( touchY );
//     }
// }
// void event_cb(lv_event_t * event) {
//     char powtext[12];
//     lv_snprintf(powtext, sizeof(powtext), "%d",(int)lv_slider_get_value(ui_power));
//     lv_label_set_text(ui_VALUE, powtext);
//     power = lv_slider_get_value(ui_power);
//      ledcWrite(1, power);
//     Serial.println(power);
//     // Serial.println("temp");
//     // Serial.println(temp());
// }

void setup()
{
    Serial.begin( 115200 );
    pinMode (radTempPin, INPUT_PULLDOWN);
    pinMode (waterTempPin, INPUT_PULLDOWN);
    pinMode (PELPIN, OUTPUT);
    pinMode (PELPIN2, OUTPUT);
    pinMode (PUMPPIN, OUTPUT);
    pinMode (FANPIN2, OUTPUT);
    ledcSetup(0, PWMFREQ, 8);
    ledcSetup(1, PUMPFREQ, 8);
    ledcSetup(2, PWMFREQ, 8); 
    ledcAttachPin(FANPIN, 0);
    ledcAttachPin(FANPIN2, 2);
    // ledcAttachPin(PUMPPIN, 1);
    uint16_t calData[5] = { 285, 3645, 291, 3506, 1 };
    QueuePump = xQueueCreate( 30  , sizeof( struct pumpData) );
    // tft.setTouch(calData);
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display.clearDisplay();
    delay(2000);

    byte iterations = 30;
    int wtemps [iterations];
    int rtemps [iterations];
    int rtemps2 [iterations];
    for (int i = 0; i < iterations; i++) {
        wtemps[i] = temp(waterTempPin);
        delay(5);
    }
    qsort (wtemps, iterations, sizeof(int), sort_desc);
    int midwtemp = wtemps[iterations / 2];

    for (int i = 0; i < iterations; i++) {
        rtemps[i] = temp(radTempPin);
        delay(5);
    }
    qsort (rtemps, iterations, sizeof(int), sort_desc);
    Serial.println ("Temps");

    for (int i = 0; i < iterations; i++) {
        rtemps2[i] = temp(radTempPin2);
        delay(5);
    }
    int midrtemp = rtemps[iterations / 2];
    int midrtemp2 = rtemps2[iterations / 2];
    Serial.println (midwtemp);
    Serial.println (midrtemp);
    Serial.println (midrtemp2);
    if (midwtemp < -20 || midwtemp > 60) {
        Serial.println(errWaterTempSensor);
        errprint(errWaterTempSensor);
    }
    else if (midrtemp < -20 || midrtemp > 100) {
        Serial.println(errRadTempSensor);
        errprint(errRadTempSensor);
    }
    else if (midrtemp2 < -20 || midrtemp2 > 100) {
        Serial.println(errRadTempSensor);
        errprint(errRadTempSensor);
    }
    else {
        Serial.println(msgSuccessStart);
        errprint(msgSuccessStart);
        delay(1000);
        display.clearDisplay();
        pel.pelNumber = 1;
        pel.reqState = true;
        xQueueSend(QueuePump, &pel, portMAX_DELAY);
        pel.pelNumber = 2;
        pel.reqState = true;
        xQueueSend(QueuePump, &pel, portMAX_DELAY);
        xTaskCreatePinnedToCore(TaskFan1, "TaskFan", 2048, NULL, 1, &TaskFan_t, 0);
        xTaskCreatePinnedToCore(TaskFan2, "TaskFan", 2048, NULL, 1, &TaskFan2_t, 0);
        xTaskCreatePinnedToCore(TaskPel, "TaskPel", 2048, NULL, 1, &TaskPel_t, 0);
        xTaskCreatePinnedToCore(TaskPel2, "TaskPel2", 2048, NULL, 1, &TaskPel2_t, 0);
        xTaskCreatePinnedToCore(TaskPump, "TaskPump", 1024, NULL, 1, &TaskPump_t, 0);
        TimerCooldown_t = xTimerCreate("TimerCooldown", pdMS_TO_TICKS(20000), pdTRUE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(cooldownTimer));
        TimerCooldown2_t = xTimerCreate("TimerCooldown2", pdMS_TO_TICKS(20000), pdTRUE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(cooldownTimer2));
    }

    // init();


    pinMode(17, OUTPUT);
    pinMode(12, OUTPUT);

//     lv_init();

// #if LV_USE_LOG != 0
//     lv_log_register_print_cb( my_print ); /* register print function for debugging */
// #endif

//     tft.begin();          /* TFT init */
//     tft.setRotation( 1 ); /* Landscape orientation, flipped */

//     lv_disp_draw_buf_init( &draw_buf, buf, NULL, screenWidth * screenHeight / 10 );

//     /*Initialize the display*/
//     static lv_disp_drv_t disp_drv;
//     lv_disp_drv_init( &disp_drv );
//     /*Change the following line to your display resolution*/
//     disp_drv.hor_res = screenWidth;
//     disp_drv.ver_res = screenHeight;
//     disp_drv.flush_cb = my_disp_flush;
//     disp_drv.draw_buf = &draw_buf;
//     lv_disp_drv_register( &disp_drv );

//     /*Initialize the (dummy) input device driver*/
//     static lv_indev_drv_t indev_drv;
//     lv_indev_drv_init( &indev_drv );
//     indev_drv.type = LV_INDEV_TYPE_POINTER;
//     indev_drv.read_cb = my_touchpad_read;
//     lv_indev_drv_register( &indev_drv );



//     ui_init();
//     lv_obj_add_event_cb(ui_power, event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    Serial.println( "Setup done" );
}


void loop()
{
}