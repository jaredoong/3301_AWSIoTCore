#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include "WiFi.h"
#include "secrets.h"

// IMU Libs
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// FFT Lib
#include "arduinoFFT.h"
#include <numeric>
#include <vector>

#define AWS_IOT_PUBLISH_TOPIC "device/1/data"
#define AWS_IOT_SUBSCRIBE_TOPIC "esp32/sub"

#define UPDATE_INTERVAL 1 * 60  // Time interval in seconds to update data
#define INF                 1e9 // Arbitary large constant to indicate to gateway that there is no data stored
#define TAKEREADING         1 // For node to switch between states for taking reading and deepsleep
#define DEEPSLEEP           0 // For node to switch between states for taking reading and deepsleep

unsigned long uS_TO_S_FACTOR = 1000000;  // micro seconds to seconds

#define DEEPSLEEPDURATION 1 * 60 * 60 // Time interval between readings, in seconds
RTC_DATA_ATTR unsigned int state = TAKEREADING; // Set intial state to take reading

// Sensor Specific
Adafruit_MPU6050 mpu;
#define GPIO_WAKEUP 39 // Interrupt GPIO pin number
#define VIBECOOLDOWN 1 * 1000 // number of seconds between each detection

// FFT Specific
arduinoFFT FFT = arduinoFFT(); // Create FFT object
const int MPU_ADDR = 0x68;
double gravity[3] = {0, 0, 0};
int mpureadcounter = 0;
int SamplingRate = 1000; // Read 1000 values in one second.
int freqstart = 2;
int freqend = 205;
const int numsamples = 1024;
int16_t AcX, AcY, AcZ;
double accX, accY, accZ;
std::vector<double> accXvec(numsamples);
std::vector<double> accYvec(numsamples);
std::vector<double> accZvec(numsamples);
uint8_t exponent;
hw_timer_t *timer = NULL;
volatile bool timerbool1 = false; // For checking if timer triggered
int accel_range;
std::vector<double> FFTData(freqend - freqstart);

WiFiClientSecure net = WiFiClientSecure();
PubSubClient client(net);

// Sensor specific data
enum
{
  X_READING,
  Y_READING,
  Z_READING,
  FFT_READING,
};

/**
 * @brief Timer used to collect data for FFT.
 */
void IRAM_ATTR onTimer()
{
  timerbool1 = true; // Indicates that the interrupt has been entered since the last time its value was changed to false
}

/*
 * FFT related functions
*/

/**
 * @brief Performs FFT based on data collected from MPU.
 * 
 * @param vReal Vector containing real values
 * @param vImag Vector containing imaginary values
 */
void conductFFT(std::vector<double> &vReal, std::vector<double> &vImag)
{
  FFT.Windowing(vReal.data(), numsamples, FFT_WIN_TYP_HAMMING, FFT_FORWARD); // Weigh data
  FFT.Compute(vReal.data(), vImag.data(), numsamples, FFT_FORWARD);          // Compute FFT
  FFT.ComplexToMagnitude(vReal.data(), vImag.data(), numsamples);

  double peak = FFT.MajorPeak(vReal.data(), numsamples, SamplingRate);

  Serial.printf("Peak Frequency is %f", peak);     // Print out what frequency is the most dominant.

  for (int i = 2; i < 205; i++)
  {
    // View all these three lines in serial terminal to see which frequencies has which amplitudes

    Serial.print((i * 1.0 * SamplingRate) / numsamples, 1);
    Serial.print(" ");
    // Serial.println(vReal[i], 1); // View only this line in serial plotter to visualize the bins
  }
}

/**
 * @brief Collecting of data using MPU.
 */
void updateaccel()
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6); // request a total of 6 registers
  int t = Wire.read();
  AcX = (t << 8) | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  t = Wire.read();
  AcY = (t << 8) | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  t = Wire.read();
  AcZ = (t << 8) | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  double accel_scale = 1;
  if (accel_range == MPU6050_RANGE_16_G)
    accel_scale = 2048;
  if (accel_range == MPU6050_RANGE_8_G)
    accel_scale = 4096;
  if (accel_range == MPU6050_RANGE_4_G)
    accel_scale = 8192;
  if (accel_range == MPU6050_RANGE_2_G)
    accel_scale = 16384;

  // setup range dependant scaling
  accX = ((double)AcX) / accel_scale * 9.81;
  accX = accX - gravity[0];
  accY = ((double)AcY) / accel_scale * 9.81;
  accY = accY - gravity[1];
  accZ = ((double)AcZ) / accel_scale * 9.81;
  accZ = accZ - gravity[2];
}

// Base example
void messageHandler(char* topic, byte* payload, unsigned int length) {
  Serial.print("incoming: ");
  Serial.println(topic);

  StaticJsonDocument<200> doc;
  deserializeJson(doc, payload);
  const char* message = doc["message"];
  Serial.println(message);
}

void connectAWS() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.println("Connecting to Wi-Fi");

  for (int i = 0; i < 10 && WiFi.status() != WL_CONNECTED; i++) {
    Serial.print(".");
    delay(1000);
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Failed to connect to Wi-Fi");
    esp_restart();
  }

  Serial.println("WiFi connected");

  // Configure WiFiClientSecure to use the AWS IoT device credentials
  net.setCACert(AWS_CERT_CA);
  net.setCertificate(AWS_CERT_CRT);
  net.setPrivateKey(AWS_CERT_PRIVATE);

  // Connect to the MQTT broker on the AWS endpoint we defined earlier
  client.setServer(AWS_IOT_ENDPOINT, 8883);

  // Create a message handler
  client.setCallback(messageHandler);

  Serial.println("Connecting to AWS IOT");

  for (int i = 0; i < 10 && !client.connect(THINGNAME); i++) {
    client.connect(THINGNAME);
    Serial.print(".");
    delay(1000);
  }

  if (!client.connected()) {
    Serial.println("AWS IoT Timeout!");
    esp_restart();
  }

  // Subscribe to a topic
  // client.subscribe(AWS_IOT_SUBSCRIBE_TOPIC);

  Serial.println("AWS IoT Connected!");
}

void publishMessage() {
  StaticJsonDocument<200> doc;
  for (int i = 2; i < FFTData.size(); i++) {
    doc["FFTData"][i] = FFTData[i];
  }

  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer);  // print to client

  client.publish(AWS_IOT_PUBLISH_TOPIC, jsonBuffer);

  Serial.println("Publish message:");
  Serial.println(jsonBuffer);
}

void sendFFTData()
{
  Serial.println("Sending FFT Data");
  std::vector<double> currentData(4);

  // Sensor specific data
  Serial.println("Collecting Data");
  currentData[X_READING] = accumulate(accXvec.begin(), accXvec.end(), 0.0) / accXvec.size();
  currentData[Y_READING] = accumulate(accYvec.begin(), accYvec.end(), 0.0) / accYvec.size();
  currentData[Z_READING] = accumulate(accZvec.begin(), accZvec.end(), 0.0) / accZvec.size();
  Serial.println("Data Collected");


  // Send message to AWS
  Serial.printf("Current X Reading: %f\n", currentData[X_READING]);
  Serial.printf("Current Y Reading: %f\n", currentData[Y_READING]);
  Serial.printf("Current Z Reading: %f\n", currentData[Z_READING]);

  std::vector<double> imaginary(numsamples);
  conductFFT(accYvec, imaginary);
  currentData[X_READING] = INF;
  currentData[Y_READING] = INF;
  currentData[Z_READING] = INF;
  for (int i = freqstart; i < freqend; i++)
  {
    currentData[FFT_READING] = accYvec[i];
    FFTData[i] = accYvec[i];
    Serial.printf("FFT reading %i: %f\n", i, currentData[FFT_READING]);
    delay(10);
  }

  Serial.println("Sending message to AWS");
  publishMessage();
  Serial.println("Message Sent");
}

void setup() {
  Serial.begin(115200);

  setCpuFrequencyMhz(80); // Reduce CPU frequency to 80MHz (battery saving technique)

  // MPU Timer setup
  timer = timerBegin(0, 80, true); // Begin timer with 1 MHz frequency (80MHz/80)
  timerAttachInterrupt(timer, &onTimer, true); // Attach the interrupt to Timer1
  unsigned int timerFactor = 1000000 / SamplingRate; // Calculate the time interval between two readings, or more accurately, the number of cycles between two readings
  timerAlarmWrite(timer, timerFactor, true); // Initialize the timer
  timerAlarmEnable(timer);

  // Try to initialize MPU
  Wire.begin(21, 22); // ! sda, scl, clock speed. PINS to be edited once PCB arrives
  if (!mpu.begin(MPU_ADDR, &Wire))
  {
    Serial.println("Failed to find MPU6050 chip");
    while (1)
    {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  // Setup MPU once initialization done
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  accel_range = mpu.getAccelerometerRange();
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
  mpu.setGyroStandby(true, true, true);
  mpu.setTemperatureStandby(true);
  mpu.setAccelerometerStandby(false, false, false);
  exponent = FFT.Exponent(SamplingRate);
  updateaccel();
  gravity[0] = accX;
  gravity[1] = accY;
  gravity[2] = accZ;
  delay(100);

  if (state == TAKEREADING)
  {
    Serial.println("In reading mode");
    state = DEEPSLEEP;
  }
  else if (state == DEEPSLEEP)
  {
    state = TAKEREADING;
    Serial.println("Waking up to go to deep sleep");
    esp_sleep_enable_timer_wakeup(DEEPSLEEPDURATION * uS_TO_S_FACTOR);
    sei();
    delay(100);
    Serial.printf("Node entering deep sleep now, waking up in %i seconds.\r\n", DEEPSLEEPDURATION);
    esp_deep_sleep_start();
  }
  sei();

  Serial.println("Setup done");
}

void loop() {
  // Serial.println("Looping");
  if (timerbool1)
  {
    Serial1.println("Timer 1 triggered");
    timerbool1 = false;
    updateaccel();
    accXvec[mpureadcounter] = accX;
    accYvec[mpureadcounter] = accY;
    accZvec[mpureadcounter] = accZ;
    mpureadcounter++;
    if (mpureadcounter == numsamples + 1)
    {
      mpureadcounter = 0;
      connectAWS();
      sendFFTData();
      Serial.printf("Going to sleep now for %d seconds\n", UPDATE_INTERVAL);
      esp_sleep_enable_timer_wakeup(UPDATE_INTERVAL * uS_TO_S_FACTOR);
      esp_deep_sleep_start();
    }
  }

  // publishMessage();
  // Serial.printf("Going to sleep now for %d seconds\n", UPDATE_INTERVAL);
  // esp_deep_sleep_start();
}