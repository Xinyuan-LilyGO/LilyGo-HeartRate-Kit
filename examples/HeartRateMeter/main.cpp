/*
 * LilyGo-HeartRate-Kit , Simple heart rate meter
 * main.cpp
 * Wirte by  Lewis he , 2020
 */

#include <ArduinoOTA.h>
#include <Wire.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <TFT_eSPI.h>
#include <pcf8563.h>

#include "MAX30105.h"
#include "MPU6050.h"
#include "heartRate.h"
#include "esp_adc_cal.h"

#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESPDash.h>

// Has been defined in the TFT_eSPI library
// #define TFT_RST                  26
// #define TFT_MISO                 -1
// #define TFT_MOSI                 19
// #define TFT_SCLK                 18
// #define TFT_CS                   5
// #define TFT_DC                   23
// #define TFT_BL                   27

#define  I2C_SDA_PIN                21
#define  I2C_SCL_PIN                22
#define  RTC_INT_PIN                34
#define  BATT_ADC_PIN               35
#define  VBUS_PIN                   37
#define  LED_PIN                    33
#define  CHARGE_PIN                 32
#define  BUTTON_PIN                 38
#define  MPU_INT                    39
#define  HEATRATE_SDA               15
#define  HEATRATE_SCL               13
#define  HEATRATE_INT               4

#define  ENABLE_SOFT_AP //Uncomment this line will connect to the specified SSID

#define  WIFI_SSID                  "WIFI SSID"
#define  WIFI_PASSWD                "WIFI PASSWD"


#define  SCREEN_TIMEOUT              10000  //Enter deep sleep after timeout


TFT_eSPI        tft = TFT_eSPI();  // Invoke library, pins defined in User_Setup.h
PCF8563_Class   rtc;
MPU6050     mpu;
MAX30105    particleSensor;

bool        freefallDetected = false;
bool        rtcIrq = false;
uint32_t    targetTime = 0;
int         vref = 1100;
bool        charge_indication = false;
bool        update = false;
bool        touch_out = false;

/*MAX30105*/
const uint8_t RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
uint8_t     rates[RATE_SIZE]; //Array of heart rates
uint8_t     rateSpot = 0;
long        lastBeat = 0; //Time at which the last beat occurred
float       beatsPerMinute;
int         beatAvg;
bool        find_max30105 = false;
bool        showError = false;

// Line Chart Data
int         x_axis_size = 7;
String      x_axis[7] = {" ", " ", " ", " ", " ", " ", " "};
int         y_axis_size = 7;
int         y_axis[7] = {2, 5, 10, 12, 18, 8, 5};

AsyncWebServer server(80);

void enterDeepsleep(void);


extern const unsigned short heart[0x1000];

TFT_eSprite spr = TFT_eSprite(&tft);  // Declare Sprite object "spr" with pointer to "tft" object
TFT_eSprite stext1 = TFT_eSprite(&tft); // Sprite object stext2
TFT_eSprite stext2 = TFT_eSprite(&tft); // Sprite object stext2

#define DATA_STORAGE_SIZE 10

typedef struct  {
    int buffer[DATA_STORAGE_SIZE];
    int head;
    int tail;
} Record_t;

Record_t record;

bool setupMAX30105(void)
{
    // Initialize sensor
    if (!particleSensor.begin(Wire1, 400000)) { //Use default I2C port, 400kHz speed
        Serial.println("MAX30105 was not found");
        return false;
    }
    particleSensor.setup(); //Configure sensor with default settings
    particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
    particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
    find_max30105 = true;
    return true;
}

void loopMAX30105(void)
{
    if (!find_max30105 && !showError) {
        tft.fillScreen(TFT_BLACK);
        tft.drawString("No sensor", 40, 30);
        showError = true;
        return;
    }

    if (showError) {
        return;
    }

    long irValue = particleSensor.getIR();
    if (checkForBeat(irValue) == true) {
        //We sensed a beat!
        long delta = millis() - lastBeat;
        lastBeat = millis();
        beatsPerMinute = 60 / (delta / 1000.0);
        if (beatsPerMinute < 255 && beatsPerMinute > 20) {
            rates[rateSpot++] = (uint8_t)beatsPerMinute; //Store this reading in the array
            rateSpot %= RATE_SIZE; //Wrap variable
            //Take average of readings
            beatAvg = 0;
            for (uint8_t x = 0 ; x < RATE_SIZE ; x++)
                beatAvg += rates[x];
            beatAvg /= RATE_SIZE;
        }
    }

    if (irValue < 50000 ) {
        digitalWrite(LED_PIN, LOW);
        if (!touch_out) {
            tft.fillScreen(TFT_BLACK);
            touch_out = true;
            spr.fillSprite(TFT_BLACK);
            spr.pushImage(0, 0,  64, 64, heart);
            spr.pushSprite(10, 10);
            stext1.fillSprite(TFT_BLACK);
            stext1.drawString("--", 0, 0);
            stext1.pushSprite(100, 30);
        }
    } else {
        touch_out = false;
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    }

    if (touch_out) {
        return;
    }

    if (targetTime < millis()) {
        stext1.fillSprite(TFT_BLACK);
        stext1.drawString(String(beatAvg), 0, 0);

        if (((record.tail + 1) % DATA_STORAGE_SIZE) == record.head) {
            record.head = (record.head + 1) % DATA_STORAGE_SIZE;
        }
        record.buffer[record.tail]  = beatAvg;
        record.tail = (record.tail + 1) %  DATA_STORAGE_SIZE;
        int temp[DATA_STORAGE_SIZE] = {0};
        int i = 0;
        int index = record.head;
        while (index != record.tail) {
            temp[i++] = record.buffer[index];
            index = (index + 1) %  DATA_STORAGE_SIZE;
        }

        ESPDash.updateNumberCard("num1", beatAvg);
        ESPDash.updateLineChart("chart1", x_axis, 0, temp, DATA_STORAGE_SIZE - 1);

        Vector accel = mpu.readNormalizeAccel();
        uint16_t absX = abs(accel.XAxis);
        uint16_t absY = abs(accel.YAxis);
        uint16_t absZ = abs(accel.ZAxis);
        if ((absY > absX) && (absY > absZ)) {
            if (accel.YAxis > 0) {
                if (tft.getRotation() != 3) {
                    tft.setRotation(3);
                    tft.fillScreen(TFT_BLACK);
                }
            } else {
                if (tft.getRotation() != 1) {
                    tft.setRotation(1);
                    tft.fillScreen(TFT_BLACK);
                }
            }
        }
        targetTime += 1000;
        update = true;
    }
}

void checkSettings(void)
{
    Serial.println();

    Serial.print(" * Sleep Mode:            ");
    Serial.println(mpu.getSleepEnabled() ? "Enabled" : "Disabled");

    Serial.print(" * Clock Source:          ");
    switch (mpu.getClockSource()) {
    case MPU6050_CLOCK_KEEP_RESET:     Serial.println("Stops the clock and keeps the timing generator in reset"); break;
    case MPU6050_CLOCK_EXTERNAL_19MHZ: Serial.println("PLL with external 19.2MHz reference"); break;
    case MPU6050_CLOCK_EXTERNAL_32KHZ: Serial.println("PLL with external 32.768kHz reference"); break;
    case MPU6050_CLOCK_PLL_ZGYRO:      Serial.println("PLL with Z axis gyroscope reference"); break;
    case MPU6050_CLOCK_PLL_YGYRO:      Serial.println("PLL with Y axis gyroscope reference"); break;
    case MPU6050_CLOCK_PLL_XGYRO:      Serial.println("PLL with X axis gyroscope reference"); break;
    case MPU6050_CLOCK_INTERNAL_8MHZ:  Serial.println("Internal 8MHz oscillator"); break;
    }

    Serial.print(" * Accelerometer:         ");
    switch (mpu.getRange()) {
    case MPU6050_RANGE_16G:            Serial.println("+/- 16 g"); break;
    case MPU6050_RANGE_8G:             Serial.println("+/- 8 g"); break;
    case MPU6050_RANGE_4G:             Serial.println("+/- 4 g"); break;
    case MPU6050_RANGE_2G:             Serial.println("+/- 2 g"); break;
    }

    Serial.print(" * Accelerometer offsets: ");
    Serial.print(mpu.getAccelOffsetX());
    Serial.print(" / ");
    Serial.print(mpu.getAccelOffsetY());
    Serial.print(" / ");
    Serial.println(mpu.getAccelOffsetZ());

    Serial.println();
}

void doInt(void)
{
    freefallDetected = true;
}

bool setupMPU6050(void)
{
    if (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_16G)) {
        Serial.println("setupMPU FAIL!!!");
        return false;
    }

    // If you want, you can set accelerometer offsets
    // mpu.setAccelOffsetX();
    // mpu.setAccelOffsetY();
    // mpu.setAccelOffsetZ();

    // Calibrate gyroscope. The calibration must be at rest.
    // If you don't want calibrate, comment this line.
    mpu.calibrateGyro();

    // Set threshold sensivty. Default 3.
    // If you don't want use threshold, comment this line or set 0.
    mpu.setThreshold(3);

    mpu.setAccelPowerOnDelay(MPU6050_DELAY_3MS);

    mpu.setIntFreeFallEnabled(true);
    mpu.setIntZeroMotionEnabled(true);
    mpu.setIntMotionEnabled(true);

    mpu.setDHPFMode(MPU6050_DHPF_5HZ);

    mpu.setFreeFallDetectionThreshold(17);
    mpu.setFreeFallDetectionDuration(2);

    checkSettings();
    pinMode(MPU_INT, INPUT);
    attachInterrupt(MPU_INT, doInt, CHANGE);

    return true;
}

bool setupWiFi(void)
{
#ifdef ENABLE_SOFT_AP
    WiFi.softAP("HeartKit");
#else
    WiFi.begin(WIFI_SSID, WIFI_PASSWD);
    if (WiFi.waitForConnectResult() != WL_CONNECTED) {
        Serial.println("Connected FAIL");
        return false;
    }
#endif

    return true;
}

void setupMonitor()
{
    esp_adc_cal_characteristics_t adc_chars;

    if (esp_adc_cal_characterize((adc_unit_t)ADC_UNIT_1,
                                 (adc_atten_t)ADC1_CHANNEL_7,
                                 (adc_bits_width_t)ADC_WIDTH_BIT_12,
                                 1100,
                                 &adc_chars) == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        vref = adc_chars.vref;
    }

    pinMode(CHARGE_PIN, INPUT);
    attachInterrupt(CHARGE_PIN, [] {
        charge_indication = true;
    }, CHANGE);

    if (digitalRead(CHARGE_PIN) == LOW) {
        charge_indication = true;
    }
}

bool setupRTC(void)
{
    Wire.beginTransmission(PCF8563_SLAVE_ADDRESS);
    if (Wire.endTransmission() != 0) {
        return false;
    }

    rtc.begin(Wire);

    pinMode(RTC_INT_PIN, INPUT_PULLUP);
    attachInterrupt(RTC_INT_PIN, [] {
        rtcIrq = 1;
    }, FALLING);

    //Just test rtc interrupt start...
    esp_sleep_wakeup_cause_t reason = esp_sleep_get_wakeup_cause();
    if ( reason != ESP_SLEEP_WAKEUP_EXT0 || reason != ESP_SLEEP_WAKEUP_TIMER) {
        rtc.disableAlarm();
        rtc.setDateTime(2019, 4, 7, 9, 5, 58);
        rtc.setAlarmByMinutes(6);
        rtc.enableAlarm();
        for (;;) {
            Serial.println(rtc.formatDateTime());
            if (rtcIrq) {
                rtcIrq = 0;
                detachInterrupt(RTC_INT_PIN);
                rtc.resetAlarm();
                break;
            }
            delay(500);
        }
    }
    //Just test rtc interrupt end ...

    //Check if the RTC clock matches, if not, use compile time
    rtc.check();

    return true;
}

void updateRTCTime()
{
    stext2.fillSprite(TFT_BLACK);
    stext2.setCursor(0, 0);
    RTC_Date time = rtc.getDateTime();
    if (time.hour < 10) {
        stext2.print("0");
    }
    stext2.print(time.hour);
    stext2.print(":");
    if (time.minute < 10) {
        stext2.print("0");
    }
    stext2.print(time.minute);
    stext2.pushSprite(120, 5);
}

void setup(void)
{
    Serial.begin(115200);

    memset(&record, 0, sizeof(record));

    tft.init();
    tft.setRotation(3);

    pinMode(LED_PIN, OUTPUT);
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire1.begin(HEATRATE_SDA, HEATRATE_SCL);

    tft.fillScreen(TFT_BLACK);
    tft.setTextFont(2);
    tft.setCursor(0, 0);
    tft.setTextColor(TFT_GREEN);

    if (!setupMPU6050()) {
        tft.setTextColor(TFT_RED);
        tft.println("Check MPU6050 FAIL");
        tft.setTextColor(TFT_GREEN);
    } else {
        tft.println("Check MPU6050 PASS");
    }

    if (!setupMAX30105()) {
        tft.setTextColor(TFT_RED);
        tft.println("Check MAX30105 FAIL");
        tft.setTextColor(TFT_GREEN);
    } else {
        tft.println("Check MAX30105 PASS");
    }

    if (!setupRTC()) {
        tft.setTextColor(TFT_RED);
        tft.println("Check PCF8563 FAIL");
        tft.setTextColor(TFT_GREEN);
    } else {
        tft.println("Check PCF8563 PASS");
    }

    setupMonitor();
    tft.print("Correction Vref=");
    tft.print(vref);
    tft.println(" mv");

    if (!setupWiFi()) {
        tft.setTextColor(TFT_RED);
        tft.println("Starting WiFi FAIL");
        tft.setTextColor(TFT_GREEN);
    } else {
#ifdef ENABLE_SOFT_AP
        tft.print("http://");
        tft.println(WiFi.softAPIP().toString());
        Serial.print("http://");
        Serial.println(WiFi.softAPIP().toString());
#else
        tft.println(WiFi.localIP().toString());
        Serial.print("http://");
        Serial.println(WiFi.localIP().toString());
#endif
    }

    setCpuFrequencyMhz(80);

    ESPDash.init(server);   // Initiate ESPDash and attach your Async webserver instance
    ESPDash.addLineChart("chart1", "Heart Rate", x_axis, x_axis_size, "BPM", y_axis, y_axis_size);
    ESPDash.addGaugeChart("chart2", "Sop2", 0);
    ESPDash.addNumberCard("num1", "Heart Rate", 0);
    server.begin();

    delay(3000);

    spr.createSprite(64, 64);
    stext1.setColorDepth(8);
    stext1.createSprite(32, 32);
    stext1.fillSprite(TFT_BLACK); // Fill sprite with black
    stext1.setTextColor(TFT_WHITE); // White text, no background
    stext1.setTextDatum(MC_DATUM);  // Bottom right coordinate datum
    stext1.setTextFont(4);
    stext1.drawString("99", 0, 0);

    stext2.createSprite(64, 16);
    stext2.fillSprite(TFT_BLACK);
    stext2.setTextColor(TFT_WHITE); // White text, no background
    stext2.setTextDatum(MC_DATUM);  // Bottom right coordinate datum
    stext2.setTextFont(1);
    updateRTCTime();


    tft.fillScreen(TFT_BLACK);
    spr.setSwapBytes(false);
    spr.pushImage(0, 0,  64, 64, heart);
}

//No use
String getVoltage()
{
    uint16_t v = analogRead(BATT_ADC_PIN);
    float battery_voltage = ((float)v / 4095.0) * 2.0 * 3.3 * (vref / 1000.0);
    return String(battery_voltage) + "V";
}

void loop()
{
    int y = 1;
    uint32_t updateTime = 0;       // time for next update
    uint32_t sleppTimeOut = millis();
    uint32_t updateDateTime = 0;
    bool rever = false;
    while (1) {
        loopMAX30105();

        if (millis() - updateDateTime > 1000) {
            updateDateTime = millis();
            updateRTCTime();
        }

        if (update) {
            update = false;
            stext1.pushSprite(100, 30);
        }
        if (updateTime <= millis() && !touch_out) {
            updateTime = millis() + 30;
            sleppTimeOut = millis();
            if (rever) {
                y--;
                if (y <= -10) {
                    rever = false;
                }
            } else {
                y++;
                if (y + 64 >= 80) {
                    rever = true;
                }
            }
            spr.pushSprite(10, y);
        }
        if (millis() - sleppTimeOut > SCREEN_TIMEOUT) {
            enterDeepsleep();
        }
    }
}

void enterDeepsleep()
{
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("Press again to wake up",  tft.width() / 2, tft.height() / 2 );
    mpu.setSleepEnabled(true);
    Serial.println("Go to Sleep");
    delay(3000);
    tft.writecommand(ST7735_DISPOFF);
    tft.writecommand(ST7735_SLPIN);
    esp_sleep_enable_ext1_wakeup(GPIO_SEL_38, ESP_EXT1_WAKEUP_ALL_LOW);
    esp_deep_sleep_start();
}


