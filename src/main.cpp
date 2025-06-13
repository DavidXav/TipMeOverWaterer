#include <Arduino.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <esp_sleep.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include "wifi_config.h" // Include WiFi credentials from a separate file


#define SIX_HOURS_MS   (6ULL*60*60) // 6 hours in milliseconds
#define SIX_HOURS_TICKS       (SIX_HOURS_MS*configTICK_RATE_HZ)
#define WATER 1082//sensor specific
#define DRY 2526

#define MOTOR1A 27
#define MOTOR2A 26

const int LCDAddress = 0x27; // I2C address of the LCD
const int LCDColumns = 16; // Number of columns in the LCD
const int LCDRows = 2; // Number of rows in the LCD
const int LCDSDA = 21; // SDA pin for I2C
const int LCDSCL = 22; // SCL pin for I2C
const int LCDButton = 25; // Button pin for the LCD
const int SoilMoisturePin = 35; // Pin for soil moisture sensor



//telegram Credentials
const char* botToken = "8039461251:AAGBOKUoEoFB0y7QCb6TrMubOUcoRq64dnI";
const char* chatID = "-4800388392"; // Replace with your Telegram chat ID

LiquidCrystal_I2C lcd(LCDAddress, LCDColumns, LCDRows); // Create an LCD object
TaskHandle_t ledTask1Handle; // Task handle for the first task
TaskHandle_t LCDTask2Handle; // Task handle for the second task
TaskHandle_t wateringTaskHandle; // Task handle for the watering task
TaskHandle_t moistureCheckTaskHandle; // Task handle for the moisture check task
volatile bool buttonPressed = false; // Flag to indicate button press
int moistureValue = 0; // Variable to store soil moisture value

// Interrupt service routine for the button press
void IRAM_ATTR lcdButtonISR();

// put function declarations here:
void LCDTask(void *);
void wateringTask(void *);
void moistureCheckTask(void *);
// put function definitions here:

void loop(){
  // Empty loop
}

void sleepNow() {
  vTaskSuspendAll(); // Suspend all tasks before entering deep sleep
  esp_light_sleep_start(); // Put the ESP32 into deep sleep
}
void setup() {
  Serial.begin(115200);
  delay(100);


  pinMode(LCDButton, INPUT_PULLUP); // Set button pin as input with pull-up resistor
  attachInterrupt(digitalPinToInterrupt(LCDButton), lcdButtonISR, FALLING); // Attach interrupt to button pin

  pinMode(MOTOR1A, OUTPUT); // Set MOTOR1A pin as output
  pinMode(MOTOR2A, OUTPUT); // Set MOTOR2B pin as output


  WiFi.begin(ssid, password);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected.");

  // esp_sleep_enable_ext0_wakeup(GPIO_NUM_25, LCDButton); // Wake on LOW (e.g., button press)
  // esp_sleep_enable_timer_wakeup(SLEEP_DURATION);

  BaseType_t xReturned = xTaskCreate(
    LCDTask,
    "LCD Task",
    2048,        // Increase stack size to be safe
    NULL,
    1,
    &LCDTask2Handle
  );

  BaseType_t xReturned2 = xTaskCreate(
    wateringTask,
    "Watering Task",
    2048,        // Increase stack size to be safe
    NULL,
    1,
    &wateringTaskHandle
  );

  BaseType_t xReturned3 = xTaskCreate(
    moistureCheckTask,
    "Moisture Check Task",
    4096,        // Increase stack size to be safe
    NULL,
    1,
    &moistureCheckTaskHandle
  );

  configASSERT(xReturned == pdPASS);
}

// Function to send a message to Telegram, could (should) function as task
void sendTelegramMessage(const String& message){
  if(WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected, attempting to connect...");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
      delay(1000);
      Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to WiFi");
  }

  HTTPClient http;
  String url = "https://api.telegram.org/bot" + String(botToken) + "/sendMessage?chat_id=" + String(chatID) + "&text=" + message;
  http.begin(url);
  int httpResponseCode = http.GET();
  if(httpResponseCode > 0) {
    Serial.println("Telegram message sent successfully");
    Serial.println(http.getString());
  }
  else {
    Serial.println("Error sending Telegram message");
    Serial.println(http.errorToString(httpResponseCode));
  }
  http.end();
}

void wateringTask(void *pvParameters) {
  while (1) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    digitalWrite(MOTOR1A, HIGH); // Turn on the motor
    digitalWrite(MOTOR2A, LOW); // Set the direction of the motor
    vTaskDelay(pdMS_TO_TICKS(4000)); // Run the motor for 4 seconds
    digitalWrite(MOTOR1A, LOW); // Turn off the motor
    digitalWrite(MOTOR2A, LOW);
  }
}

void moistureCheckTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xDelay = SIX_HOURS_TICKS;
  // const TickType_t xDelay = pdMS_TO_TICKS(10000);
  while (1) {
    // Read soil moisture sensor
    moistureValue = analogRead(SoilMoisturePin);
    Serial.print("Soil Moisture Value: ");
    Serial.println(moistureValue);
    
    // Check if the soil is dry
    if (moistureValue > DRY - 400) {
      xTaskNotifyGive(wateringTaskHandle); // Notify the watering task
      vTaskDelay(pdMS_TO_TICKS(10000)); // Wait for 10 seconds to allow the watering task to run
      if(abs(analogRead(SoilMoisturePin) - moistureValue) < 40){
        sendTelegramMessage("Water tank appears to be empty, please refill it.");
      }
      else {
        sendTelegramMessage("Plant was watered Successfully.");
      }
    }
    else{
      sendTelegramMessage("Soil moisture is at an acceptabl level: " + String(moistureValue));
    } 
    Serial.println("Delaying for:");
    Serial.println(xDelay);
    vTaskDelayUntil(&xLastWakeTime, xDelay); // Delay for 6 hours before next check
  }
}

void IRAM_ATTR lcdButtonISR() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(LCDTask2Handle, &xHigherPriorityTaskWoken); // Notify the LCD task
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken); // Yield to the LCD task if needed
}

void LCDTask(void *pvParameters) {
  // Initialize LCD here, inside the task
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Hello, World!");
  uint8_t count = 0;

  TickType_t lastTriggerTime = 0;
  const TickType_t debounceDelay = pdMS_TO_TICKS(500); // 200 ms debounce

  while (1) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Wait for notification from ISR
    lcd.backlight();

    TickType_t now = xTaskGetTickCount();
    
    int soilMoisture = analogRead(SoilMoisturePin); // Read soil moisture sensor
    //debounce logic
    if(now - lastTriggerTime > debounceDelay) {
      lastTriggerTime = now;

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Moistness: ");
      lcd.print(soilMoisture);
      vTaskDelay(pdMS_TO_TICKS(5000)); // Delay for 5 seconds
      lcd.noBacklight(); //turn off backlight
    }
  }
}
