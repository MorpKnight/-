#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <PubSubClient.h>  // Include the PubSubClient library for MQTT
#include <PulseSensorPlayground.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// Declaration for the OLED display
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// MQTT Broker settings
const char* mqtt_server = "45.80.181.181";
const char* mqtt_user = "forback";
const char* mqtt_pass = "forback2024";
const char* topic_heart_rate = "safeyou/wristband/heartRate";
const char* topic_temperature = "safeyou/wristband/temperature";
const char* topic_position = "safeyou/wristband/position";
const char* topic_location = "safeyou/wristband/location";

WiFiClient espClient;
PubSubClient client(espClient);
PulseSensorPlayground pulseSensor;

// Wi-Fi settings
const char* ssid = "";
const char* password = "";

bool isWiFiConnected = false;

// Fallback dummy data
int heartRate = 72;
float temperature = 36.5;
String position = "Standing";
float latitude = -6.2;
float longitude = 106.8;

//HeartPulseSensor
const int PULSE_INPUT = 34;
const int PULSE_BLINK = 2;  // LED blinks with heartbeat (optional)
const int PULSE_FADE = 5;   // LED fades with heartbeat (optional)
const int THRESHOLD = 550;  // Threshold to detect heartbeat
const int OUTPUT_TYPE = SERIAL_PLOTTER;
const int BUZZER_THRESHOLD = 50;  //THRESHOLD HEARTBEAT UNTUK MEMBUNYIKAN BUZZER

const int buzzer = 12;           // Pin connected to the piezo buzzer
const int toneFrequency = 1000;  // Frequency of the tone (in Hz)
const int toneDuration = 200;    // Duration of the tone (in milliseconds)

// MQTT callback function
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.print("Message received on topic ");
  Serial.print(topic);
  Serial.print(": ");
  Serial.println(message);

  if (strcmp(topic, topic_heart_rate) == 0) {
    heartRate = message.toInt();
  } else if (strcmp(topic, topic_temperature) == 0) {
    temperature = message.toFloat();
  } else if (strcmp(topic, topic_position) == 0) {
    position = message;
  } else if (strcmp(topic, topic_location) == 0) {
    int commaIndex = message.indexOf(',');
    if (commaIndex != -1) {
      latitude = message.substring(0, commaIndex).toFloat();
      longitude = message.substring(commaIndex + 1).toFloat();
    }
  }
}

void connectToWiFi(void* parameter) {
  WiFi.begin(ssid, password);

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 30);
  display.println("Connecting to WiFi...");
  display.display();

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  Serial.println("Connected to WiFi");
  isWiFiConnected = true;
  vTaskDelete(NULL);
}

void reconnectToMQTT() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");

    if (client.connect("ESP32Client", mqtt_user, mqtt_pass)) {
      Serial.println("Connected to MQTT");
      client.subscribe(topic_heart_rate);
      client.subscribe(topic_temperature);
      client.subscribe(topic_position);
      client.subscribe(topic_location);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void splashAnimation() {
  String text = "SAFEYOU";
  int textHeight = 24;
  int textWidth = text.length() * 18;
  int xStart = (SCREEN_WIDTH - textWidth) / 2;
  int yStart = -textHeight;
  int yCenter = (SCREEN_HEIGHT - textHeight) / 2;

  for (int y = yStart; y <= yCenter; y += 2) {
    display.clearDisplay();
    display.setTextSize(3);
    display.setTextColor(WHITE);
    display.setCursor(xStart, y);
    display.println(text);
    display.display();
    delay(40);
  }
  delay(3000);
}

void initializePulseSensor() {

  pulseSensor.analogInput(PULSE_INPUT);
  pulseSensor.blinkOnPulse(PULSE_BLINK);
  pulseSensor.fadeOnPulse(PULSE_FADE);
  pulseSensor.setSerial(Serial);
  pulseSensor.setOutputType(OUTPUT_TYPE);
  pulseSensor.setThreshold(THRESHOLD);
  if (pulseSensor.begin()) {
    Serial.println('Initialization Successful!');
  }

  // Start the PulseSensor and check for errors
  // if (!pulseSensor.begin()) {
  //   while (true) {  // Infinite loop if initialization fails
  //     digitalWrite(PULSE_BLINK, LOW);
  //     delay(50);
  //     Serial.println('Initialization failed!');
  //     digitalWrite(PULSE_BLINK, HIGH);
  //     delay(50);
  //   }
}

void heartbeatTask(void* pvParameters) {
  while (true) {
    // Check if a sample is ready from the PulseSensor
    if (pulseSensor.UsingHardwareTimer) {
      // delay(20);  // Avoid overwhelming the serial output
    } else {
      if (pulseSensor.sawNewSample()) {
        if (--pulseSensor.samplesUntilReport == 0) {
          pulseSensor.samplesUntilReport = SAMPLES_PER_SERIAL_SAMPLE;
          // pulseSensor.outputSample(); // Uncomment if you need sample output
        }
      }
    }

    // Check if a heartbeat is detected
    if (pulseSensor.sawStartOfBeat()) {
      int myBPM = pulseSensor.getBeatsPerMinute();
      Serial.println("♥  A HeartBeat Happened!");
      Serial.print("BPM: ");
      Serial.println(myBPM);
      if (myBPM >BUZZER_THRESHOLD) {
        tone(buzzer, toneFrequency, toneDuration);  
        delay(toneDuration);                       
        noTone(buzzer);  
        // Serial.println("Buzzer Bunyi");                           
      }
    }

    vTaskDelay(pdMS_TO_TICKS(100));  // Delay 100 ms (adjust as needed)
  }
}

void setup() {
  Serial.begin(115200);


  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    // for (;;)
    //   ;
  }

  splashAnimation();

  initializePulseSensor();
  pinMode(buzzer, OUTPUT);

  xTaskCreate(connectToWiFi, "ConnectToWiFi", 4096, NULL, 1, NULL);
  xTaskCreate(heartbeatTask, "heartbeatTask", 4096, NULL, 1, NULL);
  Serial.println("Wifi Task Created");
  Serial.println("Pulse Sensor Task Created");

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(10, 10);
  display.println("Connecting to WiFi...");
  display.display();

  client.setServer(mqtt_server, 1883);
  client.setCallback(mqttCallback);
}

void loop() {
  if (isWiFiConnected) {
    if (!client.connected()) {
      reconnectToMQTT();
    }
    client.loop();

    display.clearDisplay();

    display.setTextSize(1);
    display.setCursor(10, 0);
    display.println("SafeYou Wristband");

    display.setCursor(0, 15);
    display.print("Heart Rate: ");
    display.print(heartRate);
    display.println(" bpm");

    display.setCursor(0, 25);
    display.print("Temp: ");
    display.print(temperature, 1);
    display.println(" C");

    display.setCursor(0, 35);
    display.print("Position: ");
    display.println(position);

    display.setCursor(0, 45);
    display.print("Location: ");
    display.print(latitude, 1);
    display.print(", ");
    display.println(longitude, 1);

    display.setCursor(0, 55);
    if (isWiFiConnected) {
      display.println("Status: Connected");
    } else {
      display.println("Status: Disconnected");
    }
    display.display();
  }

  delay(1000);
}