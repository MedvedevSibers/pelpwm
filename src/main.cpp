#include <TFT_eSPI.h>
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

TaskHandle_t TaskTempCheck_t;
TaskHandle_t TaskPelControl_t;
TaskHandle_t TaskFan1_t;
TaskHandle_t TaskFan2_t;
TaskHandle_t TaskPump_t;
TimerHandle_t TimerCooldown_t;
TimerHandle_t TimerCooldown2_t;
QueueHandle_t QueuePump;
QueueHandle_t QueueScreenData;
QueueHandle_t QueueCoolingCommands;

Adafruit_SSD1306 display(128, 64, &Wire, -1);

int power = 0;
bool debug = true;
int8_t correction = -2;
byte pumpPower = 255;
bool pelOverheat1 = false;
bool pelOverheat2 = false;

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

struct CoolingCommand {
    byte pelNumber;
    bool reqState;
};

struct MessageData {
    char topic [10];
    char message [255];
    byte textsize;
};

pumpData pel;

float temp (byte tpin) {
    float tdata;
    char temptext[12];
    int t = analogRead(tpin);
    double voltage = (float)t / 4095.0 * 3.3;
    double Rt = 10 * voltage / (3.3 - voltage); 
    double tempK = 1 / (1 / (273.15 + 25) + log(Rt / 10) / 3950.0);
    double tempCRaw = tempK - 273.15;
    double tempC = tempCRaw + correction;
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
    struct CoolingCommand command;
    struct pumpData pump;
    int radTemp = temp(radTempPin);
    if (radTemp < 50) {
        pelOverheat1 = false;
        pump.pelNumber = 1;
        pump.reqState = true;
        xQueueSend(QueuePump, &pel, portMAX_DELAY);
        int waterTemp = temp(waterTempPin);
        if (waterTemp >= 20) {
            command.pelNumber = 1;
            command.reqState = true;
            xQueueSend(QueueCoolingCommands, &command, 500);
        }
        Serial.println (msgRadCooled);
        errprint (msgRadCooled);
        vTaskResume(TaskFan1_t);
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
    struct CoolingCommand command;
    struct pumpData pump;
    int radTemp = temp(radTempPin2);
    if (radTemp < 50) {
        pelOverheat2 = false;
        pump.pelNumber = 2;
        pump.reqState = true;
        xQueueSend(QueuePump, &pel, portMAX_DELAY);
        int waterTemp = temp(waterTempPin);
        if (waterTemp >= 20) {
            command.pelNumber = 2;
            command.reqState = true;
            xQueueSend(QueueCoolingCommands, &command, 500);
        }
        Serial.println (msgRadCooled);
        errprint (msgRadCooled);
        vTaskResume(TaskFan2_t);
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

void TaskTempCheck (void *pvParameters) {
    struct CoolingCommand command;
    struct MessageData message;
    static int lastWaterTemp;
    bool reqCoolingState;
    for (;;) {
    int currentWaterTemp = temp (waterTempPin);
    if (lastWaterTemp != currentWaterTemp) {
        lastWaterTemp = currentWaterTemp;
        itoa(currentWaterTemp,message.message,DEC);
        strcpy(message.topic, "temp");
        message.textsize = 3;
        xQueueSend(QueueScreenData, &message, portMAX_DELAY);
    }
    if (currentWaterTemp >= 20) {
        if (pelOverheat1 == false) {
            command.pelNumber = 1;
            command.reqState = true;
            xQueueSend(QueueCoolingCommands, &command, portMAX_DELAY);
        }
        if (pelOverheat2 == false) {
            command.pelNumber = 2;
            command.reqState = true;
            xQueueSend(QueueCoolingCommands, &command, portMAX_DELAY);
        }
        if (debug == true) {
            Serial.println("COOLING COMMAND SEND - true");
        }
    }
    else if (currentWaterTemp <= 17) {
        command.pelNumber = 1;
        command.reqState = false;
        xQueueSend(QueueCoolingCommands, &command, portMAX_DELAY);
        command.pelNumber = 2;
        command.reqState = false;
        xQueueSend(QueueCoolingCommands, &command, portMAX_DELAY);
        if (debug == true) {
            Serial.println("COOLING COMMAND SEND - false");
        }
    }
    vTaskDelay(1000);
    }
}

void TaskPelControl (void *pvParameters) {
    struct CoolingCommand receivedCommand;
    for (;;) {
    xQueueReceive(QueueCoolingCommands, &receivedCommand, portMAX_DELAY);
    if (receivedCommand.reqState == true) {
        digitalWrite(receivedCommand.pelNumber, HIGH);
        if (debug == true) {
            Serial.printf("PELCONTROL - ", receivedCommand.pelNumber, " pin enabled");
        }
    }
    else if (receivedCommand.reqState == false) {
        digitalWrite(receivedCommand.pelNumber, LOW);
        if (debug == true) {
            Serial.printf("PELCONTROL - ", receivedCommand.pelNumber, " pin disabled");
        }
    }
    vTaskDelay(100);
    }
}

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
            display.setCursor(70, 20);
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

void TaskFan1 (void *pvParameters) {
    struct pumpData pump;
    struct CoolingCommand command;
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
            command.pelNumber = PELPIN;
            command.reqState = false;
            pelOverheat1 = true;
            xQueueSend(QueueCoolingCommands, &command, portMAX_DELAY);
            ledcWrite (0, 255);
            if (debug == true) {
                Serial.println(errRadOverheat);
            }
            errprint(errRadOverheat);
            pump.pelNumber = 1;
            pump.reqState = false;
            xQueueSend(QueuePump, &pel, portMAX_DELAY);
            xTimerStart (TimerCooldown_t, 0);
            vTaskSuspend(NULL);
    }
    vTaskDelay(1000);
    }
}

void TaskFan2 (void *pvParameters) {
    struct pumpData pump;
    struct CoolingCommand command;
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
            command.pelNumber = PELPIN2;
            command.reqState = false;
            pelOverheat2 = true;
            xQueueSend(QueueCoolingCommands, &command, portMAX_DELAY);
            ledcWrite (2, 255);
            if (debug == true) {
                Serial.println(errRadOverheat);
            }
            pel.pelNumber = 2;
            pel.reqState = false;
            xQueueSend(QueuePump, &pel, portMAX_DELAY);
            errprint(errRadOverheat);
            xTimerStart (TimerCooldown2_t, 0);
            vTaskSuspend(NULL);
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

void TaskScreenControl (void *pvParameters) {
    struct MessageData received;
    for (;;) {
        xQueueReceive(QueueScreenData, &received, portMAX_DELAY);
        if (received.topic == "temp") {
            display.clearDisplay();
            display.setCursor(0, 20);
            display.setTextSize(received.textsize);
            display.print(received.message);
            display.display();
        }
        vTaskDelay(100);
    }
}

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
    uint16_t calData[5] = { 285, 3645, 291, 3506, 1 };
    QueuePump = xQueueCreate(50  , sizeof( struct pumpData) );
    QueueScreenData = xQueueCreate(20, sizeof(struct MessageData));
    QueueCoolingCommands = xQueueCreate(100, sizeof(struct CoolingCommand));

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
        xTaskCreatePinnedToCore(TaskFan1, "TaskFan", 2048, NULL, 1, &TaskFan1_t, 0);
        xTaskCreatePinnedToCore(TaskFan2, "TaskFan2", 2048, NULL, 1, &TaskFan2_t, 0);
        xTaskCreatePinnedToCore(TaskTempCheck, "TaskTempCheck", 2048, NULL, 1, &TaskTempCheck_t, 0);
        xTaskCreatePinnedToCore(TaskPelControl, "TaskPelControl", 2048, NULL, 1, &TaskPelControl_t, 0);
        xTaskCreatePinnedToCore(TaskPump, "TaskPump", 1024, NULL, 1, &TaskPump_t, 0);
        TimerCooldown_t = xTimerCreate("TimerCooldown", pdMS_TO_TICKS(20000), pdTRUE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(cooldownTimer));
        TimerCooldown2_t = xTimerCreate("TimerCooldown2", pdMS_TO_TICKS(20000), pdTRUE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(cooldownTimer2));
    }
    pinMode(17, OUTPUT);
    pinMode(12, OUTPUT);

    Serial.println( "Setup done" );
}


void loop()
{
}