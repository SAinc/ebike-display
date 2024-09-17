#include <TFT_eSPI.h>
#include <SPI.h>
#include <VescUart.h>
#include <EEPROM.h>

// images and fonts
#include "Background.h"
#include "7-Segment34.h"
#include "7-Segment56.h"
#include "7-Segment154.h"
#include "7-Segment80-FULL.h"

// #define DEBUG
// #define TEST

// DEFINES
#define LOOP_PERIOD         35                  // loop period in ms
#define PIN_TX              28                  // UART tx pin
#define PIN_RX              29                  // UART rx pin
#define PIN_POWER           7                   // soft power button control pin
#define PIN_POWER_SW        6                   // power switch pin
#define TIMEOUT_MS          100                 // timeout for recv
#define BAUD                115200              // baudrate for VESC UART
#define NUM_CELLS           14                  // number of battery cells
#define V_MAX               NUM_CELLS * 4.2     // max battery voltage
#define V_MIN               NUM_CELLS * 3.2     // min battery voltage
#define WHEEL_DIA           285.75              // wheel diameter in mm
#define NUM_MAGNETS         30                  // number of magnets on the stator
#define CAN_ID_1            0                   // CAN id of first controller
#define CAN_ID_2            85                  // CAN id of second controller
#define BATTERY_CAPACITY    1398.6              // capacity of battery in Wh
#define POWER_OFF_DELAY     2500                // delay before system turns off in ms

// COLORS
#define COLOR_METER_BACKGROUND 0x18c3     // 0x1a1a1a
#define COLOR_METER_GOOD 0x4746     // 0x45eb32
#define COLOR_METER_REGULAR 0x25be  // 24b5fa
#define COLOR_METER_REGULAR_2 0xe401    // 0xe38205
#define COLOR_METER_WARNING 0xff83  // 0xfcf115
#define COLOR_METER_BAD 0xf124     // 0xfa2424
#define COLOR_WARNING 0xfbc5        // 0xff7a2a
#define COLOR_TEXT_SECONDARY 0xcc79 // 0xcccccc

#define POWER_METER_X 159           // power meter x coord
#define POWER_METER_Y 108           // power meter y coord
#define BATTERY_X 300               // battery meter x coord
#define BATTERY_Y 165               // battery meter y coord
#define TEMP_X 5                    // temp meter x coord
#define TEMP_Y 165                  // temp meter y coord
#define POWER_METER_THICKNESS 10    // thickness of power meter segment
#define POWER_METER_RADIUS 82       // radius of power meter segment

#define EEPROM_ADDR 0       // address to store odometer value
#define NUM_SAMPLES_POWER 1 // number of samples to use for rolling average

// CONSTANTS
static const double CONVERSION_FACTOR_MPH = PI * WHEEL_DIA * 0.0000372902; // obtained by mm/min -> mi/hr
static const double CONVERSION_FACTOR_MI = PI * WHEEL_DIA / 1.609e+6;

// Initialize TFT and sprites
TFT_eSPI tft = TFT_eSPI();

// Initialize VESC connections
VescUart vesc(100);

// globl vars
uint32_t updateTime = 0; // time for next update
float batteryPercentage = 0;
float tripCounter = 0;
float odometer = 10;
float power1 = 0;
float power2 = 0;
float speed_mph = 0;
float vbat = 0;
float temp = 0;
float batteryCurrentCapacity = 0;
bool batteryFull = false;

bool powerSwPressed = false;
long delayTime;

enum State {RUN, OFF, POWER_HELD};
State state = RUN;

float powerSamples[NUM_SAMPLES_POWER] = {0};
uint16_t powerSampleIndex = 0;

// initialize as NAN so that we always draw values the first execution
float odometerPrev = NAN;
float tripCounterPrev = NAN;
float speedPrev = NAN;
float power1Prev = NAN;
float power2Prev = NAN;
float batteryPercentagePrev = NAN;
float vbatPrev = NAN;
float tempPrev = NAN;

// initialize as max so we always draw on the first execution
uint16_t powerAngle1_prev = 315;
uint16_t powerAngle2_prev = 315;
uint16_t batteryMeterPrev = 101;
uint16_t tempMeterPrev = 101;

uint16_t powerMeterColor = COLOR_METER_REGULAR;
uint16_t powerMeterColor_prev = 0;

uint16_t batteryMeterColor = COLOR_METER_GOOD;
uint16_t batteryMeterColorPrev = 0;

uint16_t tempMeterColor = COLOR_METER_REGULAR;
uint16_t tempMeterColor_prev = 0;

void setup(void)
{
#ifdef DEBUG
    Serial.begin(115200);
#endif

    Serial1.setRX(PIN_RX);
    Serial1.setTX(PIN_TX);
    Serial1.begin(BAUD); // start uart0

    pinMode(PIN_POWER, OUTPUT);
    digitalWrite(PIN_POWER, HIGH);

    pinMode(PIN_POWER_SW, INPUT_PULLUP);

    EEPROM.begin(256);  // start emulated eeprom with 256 byte length

#ifdef DEBUG
    vesc.setDebugPort(&Serial);
#endif

    tft.begin();
    tft.setRotation(1); // set orientation to landscape
    tft.fillScreen(TFT_BLACK);
    // draw ON screen
    tft.loadFont(Seven_Segment80_FULL);
    tft.setTextColor(TFT_WHITE, TFT_BLACK, true);
    tft.setCursor(115, 110);
    tft.printf("Starting...");
    tft.unloadFont();

    delay(5000);

    vesc.setSerialPort(&Serial1); // set serial port to use

#ifndef TEST
    while (!vesc.getFWversion(CAN_ID_1))
    {
        delay(1);
#ifdef DEBUG
        Serial.println("Error: Could not connect to VESC!");
#endif
    }
#endif

#ifdef DEBUG
    delay(5000);
#endif

    // draw initial background image
    tft.pushImage(0, 0, BACKGROUND_WIDTH, BACKGROUND_HEIGHT, BackgroundImage);

    // restore odometer value and battery capacity
    EEPROM.get(EEPROM_ADDR, odometer);
    EEPROM.get(EEPROM_ADDR + sizeof(odometer), batteryCurrentCapacity);
}

void loop(void)
{
    // run state machine
    switch (state) {
        case RUN: 
        if (updateTime <= millis()) 
        {
            updateTime = millis() + LOOP_PERIOD;

            #ifdef TEST
            // update values without needing VESC connection
            updateDisplay();

            if (++batteryPercentage > 100)
                batteryPercentage = 0;
            if (++tripCounter > 15)
                tripCounter = 0;
            if ((odometer += 10) > 1000)
                odometer = 0;
            if ((power1 += 50) > 6000)
                power1 = -2000;
            if ((power2 += 50) > 6000)
                power2 = -2000;
            if (++speed_mph > 45)
                speed_mph = 0;
            if (++vbat > V_MAX)
                vbat = V_MIN;
            if (++temp > 90)
                temp = 12;
            #endif
            #ifndef TEST
            if (getProcessTelemData())
            {
                updateDisplay();
            }
        #endif
        }
        // handle power button presses
        if (!digitalRead(PIN_POWER_SW)) {
            delayTime = millis() + POWER_OFF_DELAY;
            state = POWER_HELD;
        }
        break;

        case POWER_HELD:
        if ((!digitalRead(PIN_POWER_SW)) && (delayTime <= millis())) {
            state = OFF;    // power button has been held for the delay
        } else if (digitalRead(PIN_POWER_SW)) {
            state = RUN;    // power button was released, reset delay
        } else {
            state = POWER_HELD; // keep waiting
        }
        
        if (updateTime <= millis()) 
        {
            updateTime = millis() + LOOP_PERIOD;

            #ifdef TEST
            // update values without needing VESC connection
            updateDisplay();

            if (++batteryPercentage > 100)
                batteryPercentage = 0;
            if (++tripCounter > 15)
                tripCounter = 0;
            if ((odometer += 10) > 1000)
                odometer = 0;
            if ((power1 += 50) > 6000)
                power1 = -2000;
            if ((power2 += 50) > 6000)
                power2 = -2000;
            if (++speed_mph > 45)
                speed_mph = 0;
            if (++vbat > V_MAX)
                vbat = V_MIN;
            if (++temp > 90)
                temp = 12;
            #endif
            #ifndef TEST
            if (getProcessTelemData())
            {
                updateDisplay();
            }
            #endif
        }
        break;

        case OFF:
        shutdown();
        break;
        
    }

    
}

/**
 * @brief Get and process telemetry data from the connected VESC
 *
 * @return int return code
 */
int getProcessTelemData()
{
    // only recalculate if we have an update
    if (vesc.getVescValues(CAN_ID_1))
    {
        vbat = vesc.data.inpVoltage;

        // reset battery capacity tracking when battery is full
        if (equalFloat(vbat, V_MAX, 0.5) && !batteryFull)
        {
            batteryCurrentCapacity = BATTERY_CAPACITY;
            batteryFull = true;
        }
        // allow battery capacity to be reset if battery is below full threshold by 10v
        // (prevents capacity from constantly being reset)
        else if (!equalFloat(vbat, V_MAX, 10))
        {
            batteryFull = false;
        }

        // track battery capacity reported by VESC 1
        batteryCurrentCapacity = batteryCurrentCapacity - vesc.data.wattHours + vesc.data.wattHoursCharged;
        if (batteryCurrentCapacity > BATTERY_CAPACITY)
            batteryCurrentCapacity = BATTERY_CAPACITY;
        else if (batteryCurrentCapacity < 0)
            batteryCurrentCapacity = 0;

        // calculate battery percentage
        batteryPercentage = batteryCurrentCapacity / BATTERY_CAPACITY * 100;
        
        // calculate motor power
        power1 = vesc.data.inpVoltage * vesc.data.avgMotorCurrent;

        // // take running average of power to smooth it out
        // powerSamples[powerSampleIndex] = power1Raw;
        // // wrap powerSampleIndex at sample size
        // if (++powerSampleIndex <= NUM_SAMPLES_POWER) {
        //     powerSampleIndex = 0;
        // }
        // power1 = 0;
        // power2 = 0;
        // for (int i = 0; i < NUM_SAMPLES_POWER; i++) {
        //     power1 += powerSamples[i];
        // }
        // power1 = power1 / NUM_SAMPLES_POWER;

        // calculate speedometer value
        speed_mph = vesc.data.rpm / (NUM_MAGNETS / 2) * CONVERSION_FACTOR_MPH;
        // calculate trip distance
        tripCounter = vesc.data.tachometer / (3 * (NUM_MAGNETS)) * CONVERSION_FACTOR_MI;

        temp = vesc.data.tempMosfet;
        vesc.getVescValues(CAN_ID_2);
        power2 = vesc.data.inpVoltage * vesc.data.avgMotorCurrent;
        // track battery capacity reported by VESC 2
        batteryCurrentCapacity = batteryCurrentCapacity - vesc.data.wattHours + vesc.data.wattHoursCharged;
        if (batteryCurrentCapacity > BATTERY_CAPACITY)
            batteryCurrentCapacity = BATTERY_CAPACITY;
        else if (batteryCurrentCapacity < 0)
            batteryCurrentCapacity = 0;

        return 1;
    }
    return 0;
}

void updateDisplay()
{
    // check if odometer has changed
    if (!equalFloat(odometer, odometerPrev, 0.1))
    {
        // draw odometer text
        tft.loadFont(Seven_Segment34);
        tft.setCursor(47, 216);
        tft.setTextColor(COLOR_METER_BACKGROUND, TFT_BLACK);
        tft.printf("88888.8");
        tft.setCursor(47, 216);
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.printf("%7.1f", round(odometer, 1));
        tft.unloadFont();
        // store current odometer value
        EEPROM.put(EEPROM_ADDR, &odometer);
        odometerPrev = odometer;
    }
    // check if trip counter has changed
    if (!equalFloat(tripCounter, tripCounterPrev, 0.1))
    {
        // draw trip (text)
        odometer += tripCounter;
        tft.loadFont(Seven_Segment34);
        tft.setTextColor(COLOR_METER_BACKGROUND, TFT_BLACK);
        tft.setCursor(271, 216);
        tft.printf("888.8");
        tft.setCursor(271, 216);
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.printf("%5.1f", round(tripCounter, 1));
        tft.unloadFont();
        // store current trip value
        tripCounterPrev = tripCounter;
    }
    // check if speedometer value has changed
    if (!equalFloat(speed_mph, speedPrev, 0.1))
    {
        // draw speedometer value (text)
        tft.loadFont(Seven_Segment154);
        tft.setTextColor(COLOR_METER_BACKGROUND, TFT_BLACK);
        tft.setCursor(107, 55);
        tft.printf("88");
        tft.setCursor(107 , 55);
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.printf("%2.0f", round(std::abs(speed_mph), 0));
        tft.unloadFont();
        // store current speed value
        speedPrev = speed_mph;
    }
    // check if power has changed
    if ((!equalFloat(power1, power1Prev, 10)) || (!equalFloat(power2, power2Prev, 10)))
    {
        // draw power indicator
        // draw power value (text)
        tft.loadFont(Seven_Segment56);
        tft.setTextColor(COLOR_METER_BACKGROUND, TFT_BLACK);
        tft.setCursor(144, 173);
        tft.printf("8.8");
        tft.setCursor(144, 173);
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        // average both power values
        tft.printf("%2.1f", round(std::abs(power1 + power2) / 200.0, 1));
        tft.unloadFont();
        drawPowerMeter(power1, power2);
        // store current power value
        power1Prev = power1;
        power2Prev = power2;
    }
    // check if battery has changed
    if (!equalFloat(batteryPercentage, batteryPercentagePrev, 0.5))
    {
        // draw battery meter
        // draw battery percent (text)
        tft.loadFont(Seven_Segment34);
        tft.setTextColor(COLOR_METER_BACKGROUND, TFT_BLACK);
        tft.setCursor(253, 170);
        tft.printf("188");
        tft.setCursor(253, 170);
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.printf("%3.0f", round(batteryPercentage, 0));
        tft.unloadFont();
        drawBatteryMeter(batteryPercentage);
        // store current battery percentage
        EEPROM.put(EEPROM_ADDR + sizeof(odometer), &batteryCurrentCapacity);
        batteryPercentagePrev = batteryPercentage;
    }
    // check if vbat has changed
    if (!equalFloat(vbat, vbatPrev, 0.1))
    {
        // draw battery voltage (text)
        tft.loadFont(Seven_Segment34);
        tft.setTextColor(COLOR_METER_BACKGROUND, TFT_BLACK);
        tft.setCursor(253, 190);
        tft.printf("88.8");
        tft.setCursor(253, 190);
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.printf("%4.1f", round(vbat, 1));
        tft.unloadFont();
        // store current vbat value
        vbatPrev = vbat;
    }
    // check if temp has changed
    if (!equalFloat(temp, tempPrev, 0.1))
    {
        // draw temp indicator
        // draw temp (text)
        tft.loadFont(Seven_Segment34);
        tft.setTextColor(COLOR_METER_BACKGROUND, TFT_BLACK);
        tft.setCursor(1, 178);
        tft.printf("188.8");
        tft.setCursor(1, 178);
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.printf("%5.1f", round(temp, 1));
        tft.unloadFont();
        drawTempMeter(temp);
        // store current temp value
        tempPrev = temp;
    }
}

/**
 * @brief Draws an indicator needle for the power meter
 *
 * @param
 */
void drawPowerMeter(float val1, float val2)
{
    if (val1 < 0)
    {
        powerMeterColor = COLOR_METER_GOOD;
    }
    else
    {
        powerMeterColor = COLOR_METER_REGULAR;
    }

    val1 = abs(val1);
    val2 = abs(val2);

    val1 = constrain(val1, 0, 6000);
    val2 = constrain(val2, 0, 6000);

    int angle1 = map(val1, 0, 6000, 45, 315);
    int angle2 = map(val2, 0, 6000, 45, 315);

    // if color has changed, erase active portion first
    if (powerMeterColor != powerMeterColor_prev)
    {
        tft.drawArc(POWER_METER_X, POWER_METER_Y, POWER_METER_RADIUS + POWER_METER_THICKNESS, 
                    POWER_METER_RADIUS, 45, powerAngle1_prev, powerMeterColor, COLOR_METER_BACKGROUND);
        tft.drawArc(POWER_METER_X, POWER_METER_Y, POWER_METER_RADIUS + POWER_METER_THICKNESS*2, 
                    POWER_METER_RADIUS + POWER_METER_THICKNESS, 45, powerAngle2_prev, powerMeterColor, COLOR_METER_BACKGROUND);
        powerMeterColor_prev = powerMeterColor;
    }

    // Update the arc, only the zone between last_angle and new val_angle is updated
    if (angle1 > powerAngle1_prev)
    {
        tft.drawArc(POWER_METER_X, POWER_METER_Y, POWER_METER_RADIUS + POWER_METER_THICKNESS, 
                    POWER_METER_RADIUS, powerAngle1_prev, angle1, powerMeterColor, COLOR_METER_BACKGROUND);
    }
    else
    {
        tft.drawArc(POWER_METER_X, POWER_METER_Y, POWER_METER_RADIUS + POWER_METER_THICKNESS, 
                    POWER_METER_RADIUS, angle1, powerAngle1_prev, COLOR_METER_BACKGROUND, COLOR_METER_BACKGROUND);
    }
    powerAngle1_prev = angle1; // Store meter arc position for next redraw

    // update meter 2
    if (angle2 > powerAngle2_prev)
    {
        tft.drawArc(POWER_METER_X, POWER_METER_Y, POWER_METER_RADIUS + POWER_METER_THICKNESS*2, 
                    POWER_METER_RADIUS + POWER_METER_THICKNESS, powerAngle2_prev, angle2, powerMeterColor, COLOR_METER_BACKGROUND);
    }
    else
    {
        tft.drawArc(POWER_METER_X, POWER_METER_Y, POWER_METER_RADIUS + POWER_METER_THICKNESS*2, 
                    POWER_METER_RADIUS + POWER_METER_THICKNESS, angle2, powerAngle2_prev, COLOR_METER_BACKGROUND, COLOR_METER_BACKGROUND);
    }
    powerAngle2_prev = angle2; // Store meter arc position for next redraw
}

/**
 * @brief Draws the battery meter
 *
 * @param
 */
void drawBatteryMeter(float val)
{
    val = constrain(val, 0, 100);

    if (val < 20)
    {
        batteryMeterColor = COLOR_METER_BAD;
    }
    else if (val < 60)
    {
        batteryMeterColor = COLOR_METER_WARNING;
    }
    else
    {
        batteryMeterColor = COLOR_METER_GOOD;
    }

    int height = map(val, 0, 100, 160, 0);

    // if color has changed, redraw completely
    if (batteryMeterColor != batteryMeterColorPrev)
    {
        tft.fillRect(BATTERY_X, batteryMeterPrev+5, 15, 160-batteryMeterPrev, batteryMeterColor);
        batteryMeterColorPrev = batteryMeterColor;
    }

    // only draw the section that changed
    if (height < batteryMeterPrev)
    {
        tft.fillRect(BATTERY_X, height+5, 15, 160-height, batteryMeterColor);
    }
    else
    {
        tft.fillRect(BATTERY_X, 5, 15, height, COLOR_METER_BACKGROUND);
    }

    batteryMeterPrev = height;    // save height for next redraw
}

/**
 * @brief Draws the temperature meter
 *
 * @param
 */
void drawTempMeter(float val)
{
    if (val >= 60)
    {
        tempMeterColor = COLOR_METER_BAD;
    }
    else
    {
        tempMeterColor = COLOR_METER_REGULAR;
    }

    val = constrain(val, 0, 100);

    int height = map(val, 0, 100, 160, 0);

    // if color has changed, redraw completely
    if (tempMeterColor != tempMeterColor_prev)
    {
        tft.fillRect(TEMP_X, tempMeterPrev+5, 15, 160-tempMeterPrev, tempMeterColor);
        tempMeterColor_prev = tempMeterColor;
    }

    // only draw the section that changed
    if (height < tempMeterPrev)
    {
        tft.fillRect(TEMP_X, height+5, 15, 160-height, tempMeterColor);
    }
    else
    {
        tft.fillRect(TEMP_X, 5, 15, height, COLOR_METER_BACKGROUND);
    }

    tempMeterPrev = height;    // save height for next redraw
}

/**
 * @brief Compares two floats with tolerance
 *
 * @param n1
 * @param n2
 * @param epsilon tolerance to check
 * @return true if floats are within tolerance specified by epsilon
 * @return false if not
 */
bool equalFloat(float n1, float n2, float epsilon)
{
    return std::abs(n1 - n2) < epsilon;
}

/**
 * @brief Rounds float to specified decimal places
 *
 * @param val value to round
 * @param places number of places to round to
 * @return float rounded value
 */
float round(float val, int places)
{
    float temp = (int)(val * pow(10, places) + 0.5);
    return (float)(temp / pow(10, places));
}

/**
 * @brief Gets a subselection of an image
 *
 * @param dst destination image pointer
 * @param x0 starting x coord
 * @param y0 starting y coord
 * @param w_dst width of dst image
 * @param h_dst height of dst image
 * @param w_src width of src image
 * @param src source image pointer
 */
void getImageSelection(uint16_t *dst, int16_t x0, int16_t y0, int16_t w_dst,
                       int16_t h_dst, int16_t w_src, const uint16_t *src)
{
    const uint16_t *srcPtr = src + (((y0)*w_src) + x0); // start pointer at starting pixel coords
    uint16_t *dstPtr = dst;

    // loop through dst image size, reset ptr to start of next line after each iteration
    for (uint16_t i = 0; i < h_dst; i++)
    {
        for (uint16_t j = 0; j < w_dst; j++)
        {
            *(dst++) = *(srcPtr + j); // copy pixel from src to dst
        }
        srcPtr += w_src; // move ptr to next line
    }
}

/**
 * @brief called when power is lost
 *
 */
void shutdown()
{
    tft.fillScreen(TFT_BLACK);
    // draw OFF screen
    tft.loadFont(Seven_Segment80_FULL);
    tft.setTextColor(TFT_WHITE, TFT_BLACK, true);
    tft.setCursor(115, 110);
    tft.printf("OFF");
    tft.unloadFont();

    EEPROM.commit();

    delay(1000);
    digitalWrite(PIN_POWER, 0);
    while(1);

}