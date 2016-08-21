#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include "Status.h"
#include "Filter.h"
#include "Console.h"
#include "Controller.h"
#include "NewI2C.h"
#include "Storage.h"
#include "Interrupt.h"
#include "RxInput.h"
#include "Logging.h"
#include "NVState.h"
#include "PWMOutput.h"
#include "PPM.h"
#include "Command.h"
#include "Button.h"
#include <AP_Progmem/AP_Progmem.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_AVR/AP_HAL_AVR.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_AHRS/AP_AHRS.h>

extern "C" {
#include "CRC16.h"
}

//
//
//

const float stabilityElevExp1_c = -1.5;
const float stabilityElevExp2_c = 0.0;
const float stabilityAileExp1_c = -1.5;
const float stabilityAileExp2_c = 0.5;

const float G = 9.81, RADIAN = 360/2/PI, FOOT = 12*25.4/1000, KNOT = 1852.0/60/60, PSF = 47.880259;

const float alphaWindow_c = 1.0/30;

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;
AP_HAL::BetterStream* cliSerial;

AP_Baro barometer;
AP_InertialSensor ins;
AP_GPS gps;
AP_AHRS_DCM ahrs {ins,  barometer, gps};

//
// Threshold speed margin (IAS)
//

const float thresholdMargin_c = 15/100.0;

//
// HW timer declarations
//

const struct HWTimer hwTimer1 =
       { &TCCR1A, &TCCR1B, &ICR1, { &OCR1A, &OCR1B, &OCR1C } };
const struct HWTimer hwTimer3 =
       { &TCCR3A, &TCCR3B, &ICR3, { &OCR3A, &OCR3B, &OCR3C } };
const struct HWTimer hwTimer4 =
       { &TCCR4A, &TCCR4B, &ICR4, { &OCR4A, &OCR4B, &OCR4C } };

const struct HWTimer *hwTimers[] = 
  { &hwTimer1, &hwTimer3, &hwTimer4 };

//
// LED output
//

const struct PinDescriptor led[] = {{ PortA, 3 }, { PortA, 4 }, { PortA, 5 }};

#define GREEN_LED led[0]
#define BLUE_LED led[1]
#define RED_LED led[2]

//
// RC input
//

struct PinDescriptor ppmInputPin = { PortL, 1 }; 
struct RxInputRecord aileInput, elevInput, throttleInput, rudderInput,
  modeSwitchInput, tuningKnobInput, flapInput, auxInput;
struct RxInputRecord *ppmInputs[] = 
  { &aileInput, &elevInput, &throttleInput, &rudderInput, &modeSwitchInput, &tuningKnobInput, &flapInput, &auxInput };

ButtonInputChannel buttonInput;
Button rightDownButton(-0.60), rightUpButton(0.26),
  leftDownButton(-0.13), leftUpButton(0.66);

#define AILEMODEBUTTON rightUpButton
#define ELEVMODEBUTTON rightDownButton
#define TRIMBUTTON leftUpButton
#define GEARBUTTON leftDownButton

int8_t flapSwitchValue;

//
// Servo PWM output
//

#define NEUTRAL 1500
#define RANGE 500

struct PWMOutput pwmOutput[] = {
  { { PortB, 6 }, &hwTimer1, COMnB },
  { { PortB, 5 }, &hwTimer1, COMnA },
  { { PortH, 5 }, &hwTimer4, COMnC },
  { { PortH, 4 }, &hwTimer4, COMnB },
  { { PortH, 3 }, &hwTimer4, COMnA },
  { { PortE, 5 }, &hwTimer3, COMnC },
  { { PortE, 4 }, &hwTimer3, COMnB },
  { { PortE, 3 }, &hwTimer3, COMnA }
};

//
// Piezo
//

// const struct PinDescriptor piezo =  { PortE, 3 };

#define PIEZO pwmOutput[3]

//
// Function to servo output mapping
//

#define aileHandle \
  (vpParam.servoAile < 0 ? NULL : &pwmOutput[vpParam.servoAile])
#define elevatorHandle \
  (vpParam.servoElev < 0 ? NULL : &pwmOutput[vpParam.servoElev])
#define flapHandle \
  (vpParam.servoFlap < 0 ? NULL : &pwmOutput[vpParam.servoFlap])
#define flap2Handle \
  (vpParam.servoFlap2 < 0 ? NULL : &pwmOutput[vpParam.servoFlap2])
#define gearHandle \
  (vpParam.servoGear < 0 ? NULL : &pwmOutput[vpParam.servoGear])
#define brakeHandle \
  (vpParam.servoBrake < 0 ? NULL : &pwmOutput[vpParam.servoBrake])
#define rudderHandle \
  (vpParam.servoRudder < 0 ? NULL : &pwmOutput[vpParam.servoRudder])

//
// Periodic task stuff
//

#define CONTROL_HZ 50
#define CONFIG_HZ (CONTROL_HZ/4)
#define ALPHA_HZ (CONTROL_HZ*10)
#define AIRSPEED_HZ (CONTROL_HZ*5)
#define BEEP_HZ 5
#define TRIM_HZ 10
#define LED_HZ 3
#define LED_TICK 100
#define LOG_HZ_ALPHA CONTROL_HZ
#define LOG_HZ_SLOW (CONTROL_HZ/3)
#define LOG_HZ_SAVE 2
#define HEARTBEAT_HZ 1
  
struct Task {
  void (*code)(void);
  uint32_t period, lastExecuted;
};

#define HZ_TO_PERIOD(f) ((uint32_t) (1.0e6/(f)))

struct ModeRecord {
  bool test;
  bool rattle;
  bool alphaFailSafe;
  bool sensorFailSafe;
  bool rxFailSafe;
  bool wingLeveler;
  bool bankLimiter;
  bool takeOff;
  bool slowFlight;
  bool autoTest;
  bool alwaysLog;
};

struct FeatureRecord {
  bool stabilizeBank;
  bool stabilizePitch;
  bool pitchHold;
  bool alphaHold;
  bool pusher;
  bool autoBall;
};

struct GPSFix {
  float altitude;
  float track;
  float lat;
  float lon;
  float speed;
};

struct ModeRecord vpMode;
struct FeatureRecord vpFeature;
struct StatusRecord vpStatus;
// struct GPSFix gpsFix;

uint32_t currentTime;
float controlCycle = 10.0e-3;
uint32_t idleMicros;
float idleAvg, logBandWidth, ppmFreq, simInputFreq;
float testGain = 0;
float iAS, dynPressure, alpha, effAlpha, aileStick, elevStick, throttleStick, rudderStick, tuningKnob;
bool ailePilotInput, elevPilotInput, rudderPilotInput;
uint32_t controlCycleEnded;
float elevTrim, effTrim, elevTrimSub, targetAlpha;
Controller elevCtrl, aileCtrl, pushCtrl, rudderCtrl;
float autoAlphaP, maxAlpha, shakerAlpha, thresholdAlpha, rudderMix;
float accX, accY, accZ, accTotal, altitude,  heading, bankAngle, pitchAngle, rollRate, pitchRate, targetPitchRate, yawRate, levelBank;
float parameter;  
NewI2C I2c = NewI2C();
Damper ball(1.5*CONTROL_HZ), iasFilterSlow(3*CONTROL_HZ), iasFilter(2), accAvg(2*CONTROL_HZ), iasEntropyAcc(CONFIG_HZ), alphaEntropyAcc(CONFIG_HZ);
AlphaBuffer pressureBuffer;
RunningAvgFilter alphaFilter;
uint32_t simTimeStamp;
RateLimiter aileRateLimiter, flapRateLimiter, trimRateLimiter;
float elevOutput, elevOutputFeedForward, aileOutput = 0, flapOutput = 0, gearOutput = 0, brakeOutput = 0, rudderOutput = 0;
uint16_t iasEntropy, alphaEntropy, sensorHash = 0xFFFF;
bool beepGood;
const int maxParams = 8;
int beepDuration, gaugeCount, gaugeVariable[maxParams];

const int maxTests_c = 4;
float autoTestIAS[maxTests_c], autoTestK[maxTests_c], autoTestT[maxTests_c], autoTestKxIAS[maxTests_c];
int autoTestCount;

typedef enum { idle_c, start_c, trim_c, init_c, pert0_c, pert1_c, wait_c, measure_c, stop_c } TestState_t;

TestState_t testState;

const int cycleWindow_c = 1<<2;
uint32_t cycleBuffer[cycleWindow_c];
int cycleBufferPtr, cycleCount;
bool oscillating = false, rising = false, oscillationStopped = false;
float oscCycleMean;
typedef enum { ac_elev, ac_aile } analyzerInputCh_t;
analyzerInputCh_t analyzerInputCh;
float analyzerInput = 0;
float pertubPolarity = 1;

float testPertub()
{
  if(testState == pert0_c)
    return pertubPolarity;
  else if(testState == pert1_c)
    return -pertubPolarity;
  else
    return 0;
}

bool testPertubActive()
{
  return vpMode.test && vpMode.autoTest
    && (testState == pert0_c || testState == pert1_c);
}

bool testActive()
{
  return vpMode.test && vpMode.autoTest
    && testState != idle_c && testState != start_c && testState != trim_c;
}

//
// Datagram protocol integration
//

#include "Serial.h"

#define MAX_DG_SIZE  (1<<7)

extern "C" {

#include "Datagram.h"

int maxDatagramSize = MAX_DG_SIZE;
uint8_t datagramRxStore[MAX_DG_SIZE];

void executeCommand(const char *buf);

struct SimLinkSensor sensorData;
uint16_t simFrames;
int linkDownCount = 0, heartBeatCount = 0;
  
void datagramInterpreter(uint8_t t, const uint8_t *data, int size)
{  
  switch(t) {
  case DG_HEARTBEAT:
    if(!vpStatus.consoleLink) {
      consoleNoteLn_P(PSTR("Console CONNECTED"));
      vpStatus.consoleLink = true;
    }
    heartBeatCount++;
    linkDownCount = 0;
    break;
    
  case DG_CONSOLE:
    executeCommand((const char*) data);
    break;

  case DG_SIMLINK:
    if(vpStatus.consoleLink && size == sizeof(sensorData)) {
      if(!vpStatus.simulatorLink) {
	consoleNoteLn_P(PSTR("Simulator CONNECTED"));
	vpStatus.simulatorLink = true;
      }

      memcpy(&sensorData, data, sizeof(sensorData));
      simTimeStamp = hal.scheduler->micros();
      simFrames++;    
    }
    break;
    
  default:
    consoleNote_P(PSTR("FUNNY DATAGRAM TYPE "));
    consolePrintLn(t);
  }
}
  
void datagramSerialOut(uint8_t c)
{
  serialOut(c);
}
}

const uint8_t addr5048B_c = 0x40;
const uint8_t addr4525_c = 0x28;

bool read5048B(uint8_t addr, uint8_t *storage, int bytes) 
{
  return I2c.read(addr5048B_c, addr, storage, bytes) == 0;
}

bool read5048B_word14(uint8_t addr, uint16_t *result)
{
  uint8_t buf[sizeof(uint16_t)];
  bool success = false;
  
  success = read5048B(addr, buf, sizeof(buf));
  
  if(success)
    *result = ((((uint16_t) buf[0]) << 6) + (buf[1] & 0x3F))<<2;

  return success;
}

bool read4525DO(uint8_t *storage, int bytes) 
{
  return I2c.read(addr4525_c, NULL, 0, storage, bytes) == 0;
}

bool read4525DO_word14(uint16_t *result)
{
  uint8_t buf[sizeof(uint16_t)];
  bool success = false;
  
  success = read4525DO(buf, sizeof(buf));
  
  if(success)
    *result = (((uint16_t) (buf[0] & 0x3F)) << 8) + buf[1];

  return success && (buf[0]>>6) == 0;
}

void delayMicros(int x)
{
  uint32_t current = hal.scheduler->micros();
  while(hal.scheduler->micros() < current+x);
}

void beepPrim(int hz, long millis)
{
  for(long i = 0; i < hz*millis/1000; i++) {
    setPinState(&PIEZO.pin, 1);
    delayMicros(1e6/hz/2);
    setPinState(&PIEZO.pin, 0);
    delayMicros(1e6/hz/2);
  }
}

void beep(float dur, bool good)
{
  beepGood = good;
  beepDuration = dur*BEEP_HZ;
}

void goodBeep(float dur)
{
  beep(dur, true);
}

void badBeep(float dur)
{
  beep(dur, false);
}

void logAlpha(void)
{
  logGeneric(lc_alpha, alpha*360);
}

void logConfig(void)
{
  bool mode[] = { vpMode.rxFailSafe,
		  vpMode.sensorFailSafe,
		  vpMode.alphaFailSafe,
		  vpMode.takeOff,
		  vpMode.slowFlight,
		  vpMode.bankLimiter,
		  vpMode.wingLeveler };

  float modeSum = 0;
  
  for(uint16_t i = 0; i < sizeof(mode)/sizeof(bool); i++)
    if(mode[i])
      modeSum += 1.0/(2<<i);
  
  logGeneric(lc_mode, modeSum);
  logGeneric(lc_target, targetAlpha*360);
  logGeneric(lc_target_pr, targetPitchRate*360);
  logGeneric(lc_trim, effTrim*100);

  if(vpMode.test) {
    logGeneric(lc_gain, testGain);
    logGeneric(lc_test, nvState.testNum);
  } else {
    logGeneric(lc_gain, 0);
    logGeneric(lc_test, 0);
  }
}

void logPosition(void)
{
  logGeneric(lc_alt, altitude);
}
  
void logInput(void)
{
  logGeneric(lc_ailestick, aileStick);
  logGeneric(lc_elevstick, elevStick);
  logGeneric(lc_thrstick, throttleStick);
  logGeneric(lc_rudstick, rudderStick);
}

void logActuator(void)
{
  logGeneric(lc_aileron, aileRateLimiter.output());
  logGeneric(lc_elevator, elevOutput);
  logGeneric(lc_elevator_ff, elevOutputFeedForward);
  logGeneric(lc_rudder, rudderOutput);
}

void logAttitude(void)
{
  logGeneric(lc_dynpressure, dynPressure);
  logGeneric(lc_accx, accX);
  logGeneric(lc_accy, accY);
  logGeneric(lc_accz, accZ);
  logGeneric(lc_roll, bankAngle);
  logGeneric(lc_rollrate, rollRate*360);
  logGeneric(lc_pitch, pitchAngle);
  logGeneric(lc_pitchrate, pitchRate*360);
  logGeneric(lc_heading, heading);
  logGeneric(lc_yawrate, yawRate*360);
}

float readParameter()
{
  return tuningKnob/0.95 - (1/0.95 - 1);
}

#define AS5048_ADDRESS 0x40 
#define AS5048B_PROG_REG 0x03
#define AS5048B_ADDR_REG 0x15
#define AS5048B_ZEROMSB_REG 0x16 //bits 0..7
#define AS5048B_ZEROLSB_REG 0x17 //bits 0..5
#define AS5048B_GAIN_REG 0xFA
#define AS5048B_DIAG_REG 0xFB
#define AS5048B_MAGNMSB_REG 0xFC //bits 0..7
#define AS5048B_MAGNLSB_REG 0xFD //bits 0..5
#define AS5048B_ANGLMSB_REG 0xFE //bits 0..7
#define AS5048B_ANGLLSB_REG 0xFF //bits 0..5
#define AS5048B_RESOLUTION ((float) (1<<14))

bool readAlpha_5048B(int16_t *result) {
  uint16_t raw = 0;
  
  if(vpStatus.alphaFailed)
    // Stop trying
    
    return false;
  
  if(!read5048B_word14(AS5048B_ANGLMSB_REG, &raw))
    // Failed
    return false;

  // The value is good, use it

  if(result)
    *result = (int16_t) (raw - vpParam.alphaRef);
  
  return true;
}

bool readPressure(int16_t *result) 
{
  static uint32_t acc;
  static int accCount;
  static bool done = false;
  const int log2CalibWindow = 8;
  
  if(vpStatus.iasFailed)
    // Stop trying
    return false;
  
  uint16_t raw = 0;

  if(!read4525DO_word14(&raw))
    return false;

  if(accCount < 1<<log2CalibWindow) {
    acc += raw;
    accCount++;
  } else {
    if(!done)
      consoleNoteLn_P(PSTR("Airspeed calibration DONE"));
    
    done = true;
    
    if(result)
      *result = (raw<<2) - (acc>>(log2CalibWindow - 2));
  }
  
  return true;
}

//
//
//

const int cycleTimeSampleWindow_c = CONTROL_HZ;
Damper cycleTimeAcc(cycleTimeSampleWindow_c);
float cycleTimeMin = -1.0, cycleTimeMax = -1.0;
RunningAvgFilter cycleTimeAverage(cycleTimeSampleWindow_c);
Damper cycleTimeSampleFraction(CONTROL_HZ, 1.0);
int cycleTimeSampleCount = 0;
bool cycleTimeSampleAvailable = false;

void cycleTimeSampleReset(void)
{
  cycleTimeSampleCount = 0;
  cycleTimeSampleAvailable = false;
  cycleTimeSampleFraction.reset(1.0);
}
  
void cycleTimeSample(float value)
{
  if(randomNum(0, 1) < cycleTimeSampleFraction.output()) {
    cycleTimeAverage.input(value);
    cycleTimeSampleFraction.input(1.0/100);

    if(cycleTimeSampleCount < cycleTimeSampleWindow_c)
      cycleTimeSampleCount++;
    else if(!cycleTimeSampleAvailable) {
      consoleNoteLn_P(PSTR("Cycle time sample available"));
      cycleTimeSampleAvailable = true;
    }
  }
}
  
void cycleTimeMonitorReset(void)
{
  cycleTimeSampleReset();
  cycleTimeMin = cycleTimeMax = -1;
  consoleNoteLn_P(PSTR("Cycle time monitor RESET"));
}
  
void cycleTimeMonitor(float value)
{
  if(beepDuration > 0)
    // We're beeping, fuggetaboutit
    return;
  
  //
  // Track min and max
  //
  
  if(cycleTimeMin < 0.0) {
    cycleTimeMin = cycleTimeMax = value;
  } else {
    cycleTimeMin = fminf(cycleTimeMin, value);
    cycleTimeMax = fmaxf(cycleTimeMax, value);
  }

  //
  // Cumulative average
  //
  
  cycleTimeAcc.input(value);

  //
  // Random sampling for statistics
  //

  cycleTimeSample(value);  
}

//
// Takeoff configuration test
//

typedef enum {
  toc_ram,
  toc_load,
  toc_ppm,
  toc_timing,
  toc_mode,
  toc_trim,
  toc_eeprom,
  toc_attitude,
  toc_turn,
  toc_alpha_sensor,
  toc_alpha_range,
  toc_pitot_sensor,
  toc_pitot_block,
  toc_button,
  toc_aile_neutral,
  toc_aile_range,
  toc_elev_neutral,
  toc_elev_range,
  toc_rudder_neutral,
  toc_rudder_range,
  toc_throttle_zero,
  toc_throttle_range,
  toc_tuning_zero,
  toc_tuning_range } testCode_t;

#define TOC_TEST_NAME_MAX 16

struct TakeoffTest {
  char description[TOC_TEST_NAME_MAX];
  bool (*function)(bool);
};

const float toc_margin_c = 0.02;

bool toc_test_mode(bool reset)
{
  return !vpMode.test && vpMode.wingLeveler;
}

bool toc_test_trim(bool reset)
{
  return fabsf(elevTrim - vpParam.takeoffTrim) < toc_margin_c;
}

bool toc_test_ppm(bool reset)
{
  if(reset)
    ppmWarnShort = ppmWarnSlow = false;

  return !ppmWarnShort && !ppmWarnSlow;
}

bool toc_test_ram(bool reset)
{
  return hal.util->available_memory() > (1<<10);
}

bool toc_test_timing(bool reset)
{
  static bool measured = false, started = false, result = false;
  static uint32_t startTime = 0;
  
  if(reset) {
    result = measured = started = false;

  } else if(!started) {
    if(!beepDuration) {
      cycleTimeMonitorReset();
      startTime = currentTime;
      started = true;
    }
  } else if(!measured && currentTime > startTime + 3.0e6) {
    result =
      cycleTimeSampleAvailable
      && (cycleTimeMin >= 1.0/CONTROL_HZ)
      && (cycleTimeMax < 2.0/CONTROL_HZ)
      && (cycleTimeAverage.output() < 1.1/CONTROL_HZ);
    
    measured = true;
  }

  return measured && result;
}

bool toc_test_load(bool reset)
{
  return idleAvg > 0.2;
}

bool toc_test_eeprom(bool reset)
{
  if(reset)
    vpStatus.eepromWarn = vpStatus.eepromFailed = false;

  return !vpStatus.eepromWarn && !vpStatus.eepromFailed;
}

bool toc_test_alpha_sensor(bool reset)
{
  if(reset)
    vpStatus.alphaWarn = vpStatus.alphaFailed = false;
  return (!vpStatus.alphaWarn && !vpStatus.alphaFailed
	  && alphaEntropyAcc.output() > 10);
}

bool toc_test_alpha_range(bool reset)
{
  static bool bigAlpha, zeroAlpha, bigAlphaAgain;
  static uint32_t lastNonZeroAlpha, lastSmallAlpha;

  if(reset) {
    zeroAlpha = bigAlpha = bigAlphaAgain = false;
    lastNonZeroAlpha = lastSmallAlpha = currentTime;
  } else if(!bigAlpha) {
    if(fabsf(alpha - 90.0/360) > 15.0/360) {
      lastSmallAlpha = currentTime;
    } else if(currentTime > lastSmallAlpha + 1.0e6) {
      consoleNoteLn_P(PSTR("Stable BIG ALPHA"));
      bigAlpha = true;
    }
  } else if(!zeroAlpha) {
    if(fabs(alpha) > 1.5/360) {
      lastNonZeroAlpha = currentTime;
    } else if(currentTime > lastNonZeroAlpha + 1.0e6) {
      consoleNoteLn_P(PSTR("Stable ZERO ALPHA"));
      zeroAlpha = true;
    }
  } else if(!bigAlphaAgain) {
    if(fabsf(alpha - 90.0/360) > 15.0/360) {
      lastSmallAlpha = currentTime;
    } else if(currentTime > lastSmallAlpha + 1.0e6) {
      consoleNoteLn_P(PSTR("Stable BIG ALPHA seen again"));
      bigAlphaAgain = true;
    }
  }
  
  return bigAlpha && zeroAlpha && bigAlphaAgain;
}

bool toc_test_pitot_sensor(bool reset)
{
  if(reset)
    vpStatus.iasWarn = vpStatus.iasFailed = false;
  return (!vpStatus.iasWarn && !vpStatus.iasFailed
	  && alphaEntropyAcc.output() > 10);
}

bool toc_test_pitot_block(bool reset)
{
  return !vpStatus.pitotBlocked;
}

bool toc_test_attitude(bool reset)
{
  return fabsf(pitchAngle) < 10.0 && fabsf(bankAngle) < 5.0;
}

bool toc_test_turn(bool reset)
{
  return (fabsf(pitchRate) < 1.0/360
	  && fabsf(rollRate) < 1.0/360
	  && fabsf(yawRate) < 1.0/360);
}

struct TOCRangeTestState {
  float valueMin, valueMax;
};

bool toc_test_range_generic(struct TOCRangeTestState *state, bool reset, struct RxInputRecord *input, float expectedMin, float expectedMax)
{
  const float value = inputValue(input);
  
  if(reset)
    state->valueMin = state->valueMax = value;
  else {
    state->valueMin = fminf(state->valueMin, value);
    state->valueMax = fmaxf(state->valueMax, value);
  }
  
  return (fabsf(state->valueMin - expectedMin) < toc_margin_c
	  && fabsf(state->valueMax - expectedMax) < toc_margin_c);
}

bool toc_test_elev_range(bool reset)
{
  static struct TOCRangeTestState state;
  return toc_test_range_generic(&state, reset, &elevInput, -1, 1);
}

bool toc_test_aile_range(bool reset)
{
  static struct TOCRangeTestState state;
  return toc_test_range_generic(&state, reset, &aileInput, -1, 1);
}

bool toc_test_throttle_range(bool reset)
{
  static struct TOCRangeTestState state;
  return toc_test_range_generic(&state, reset, &throttleInput, 0, 1);
}

bool toc_test_rudder_range(bool reset)
{
  static struct TOCRangeTestState state;
  return toc_test_range_generic(&state, reset, &rudderInput, -1, 1);
}

bool toc_test_tuning_range(bool reset)
{
  static struct TOCRangeTestState state;
  return toc_test_range_generic(&state, reset, &tuningKnobInput, 0, 1);
}

bool toc_test_aile_neutral(bool reset)
{
  return fabsf(inputValue(&aileInput)) < toc_margin_c/2;
}

bool toc_test_elev_neutral(bool reset)
{
  return fabsf(inputValue(&elevInput)) < toc_margin_c/2;
}

bool toc_test_rudder_neutral(bool reset)
{
  return fabsf(inputValue(&rudderInput)) < toc_margin_c/2;
}

bool toc_test_throttle_zero(bool reset)
{
  return fabsf(inputValue(&throttleInput)) < toc_margin_c/2;
}

bool toc_test_tuning_zero(bool reset)
{
  return fabsf(inputValue(&tuningKnobInput)) < toc_margin_c/2;
}

bool toc_test_button(bool reset)
{
  static bool leftUp, leftDown, rightUp, rightDown;

  if(reset)
    leftUp = leftDown = rightUp = rightDown = false;
  else {
    leftUp |= leftUpButton.state();
    leftDown |= leftDownButton.state();
    rightUp |= rightUpButton.state();
    rightDown |= rightDownButton.state();
  }

  return (leftUp && leftDown && rightUp && rightDown
	  && !leftUpButton.state() && !leftDownButton.state()
	  && !rightUpButton.state() && !rightDownButton.state());
}

const struct TakeoffTest tocTest[] PROGMEM =
  { [toc_ram] = { "RAM", toc_test_ram },
    [toc_load] = { "LOAD", toc_test_load },
    [toc_ppm] = { "PPM", toc_test_ppm },
    [toc_timing] = { "TIMING", toc_test_timing },
    [toc_mode] = { "MODE", toc_test_mode },
    [toc_trim] = { "TRIM", toc_test_trim },
    [toc_eeprom] = { "EEPROM", toc_test_eeprom },
    [toc_attitude] = { "ATTI", toc_test_attitude },
    [toc_turn] = { "TURN", toc_test_turn },
    [toc_alpha_sensor] = { "ALPHA_S", toc_test_alpha_sensor },
    [toc_alpha_range] = { "ALPHA_R", toc_test_alpha_range },
    [toc_pitot_sensor] = { "PITOT_S", toc_test_pitot_sensor },
    [toc_pitot_block] = { "PITOT_B", toc_test_pitot_block },
    [toc_button] = { "BUTTON", toc_test_button },
    [toc_aile_neutral] = { "AILE_N", toc_test_aile_neutral },
    [toc_aile_range] = { "AILE_R", toc_test_aile_range },
    [toc_elev_neutral] = { "ELEV_N", toc_test_elev_neutral },
    [toc_elev_range] = { "ELEV_R", toc_test_elev_range },
    [toc_rudder_neutral] = { "RUD_N", toc_test_rudder_neutral },
    [toc_rudder_range] = { "RUD_R", toc_test_rudder_range },
    [toc_throttle_zero] = { "THR_Z", toc_test_throttle_zero },
    [toc_throttle_range] = { "THR_R", toc_test_throttle_range },
    [toc_tuning_zero] = { "TUNE_Z", toc_test_tuning_zero },
    [toc_tuning_range] = { "TUNE_R", toc_test_tuning_range } };

const int tocNumOfTests = sizeof(tocTest)/sizeof(struct TakeoffTest);

bool tocTestInvoke(bool reset, bool challenge, bool verbose)
{
  bool fail = false;
  struct TakeoffTest cache;
    
  for(int i = 0; i < tocNumOfTests; i++) {
    memcpy_P(&cache, &tocTest[i], sizeof(cache));

    bool result = (*cache.function)(reset);
    
    if(challenge && !result) {
      if(verbose) {
	if(!fail)
	  consoleNote_P(PSTR("T/O/C FAIL : "));

	consolePrint(cache.description);
	consolePrint(" ");
      }
      
      (*cache.function)(true); // Reset the failed test
      fail = true;
    }
  }

  if(fail && verbose)
    consolePrintLn("");
  
  return !fail;
}

void tocTestReset()
{
  tocTestInvoke(true, false, false);
}

void tocTestUpdate()
{
  tocTestInvoke(false, false, false);
}

bool tocTestStatus(bool verbose)
{
  return tocTestInvoke(false, true, verbose);
}

//
// Command interpreter
//

int indexOf(const char *s, const char c, int index)
{
  while(s[index] != '\0') {
    if(s[index] == c)
      return index;
  
    index++;
  }
    
  return -1;
}

int indexOf(const char *s, const char c)
{
  return indexOf(s, c, 0);
}

void executeCommand(const char *buf)
{
  int bufLen = strlen(buf);
  
  consolePrint("// % ");
  consolePrint(buf, bufLen);
  consolePrintLn("");

  if(bufLen < 1 || buf[0] == '/') {
    gaugeCount = 0;
    calibStop(nvState.rxMin, nvState.rxCenter, nvState.rxMax);
    return;
  } else if(atoi(buf) > 0) {
    gaugeCount = 1;
    gaugeVariable[0] = atoi(buf);
    return;
  }
  
  int index = 0, prevIndex = 0, numParams = 0, tokenLen = bufLen;
  float param[maxParams];
  const char *paramText[maxParams];

  for(int i = 0; i < maxParams; i++)
    param[i] = 0.0;

  if((index = indexOf(buf, ' ')) > 0) {
    tokenLen = index;
    
    do {
      prevIndex = index;
    
      index = indexOf(buf, ' ', index+1);
      
      if(index < 0)
        index = bufLen;

      if(numParams < maxParams) {
	paramText[numParams] = &buf[prevIndex+1];
        param[numParams] = atof(paramText[numParams]);
	numParams++;
      }	
    } while(index < bufLen);
  }
  
  int matches = 0, j = 0;
  struct Command command;
  
  while(1) {
    struct Command cache;
  
    memcpy_P(&cache, &commands[j++], sizeof(cache));

    if(cache.token == c_invalid)
      break;
    
    if(!strncmp(buf, cache.name, tokenLen)) {
      command = cache;
      matches++;
    }
  }
  
  if(matches < 1) {
    consolePrint_P(PSTR("Command not recognized: \""));
    consolePrint(buf, tokenLen);
    consolePrintLn("\"");
    
  } else if(matches > 1) {
    consolePrint_P(PSTR("Ambiguos command: \""));
    consolePrint(buf, tokenLen);
    consolePrintLn("\"");
    
  } else if(command.var[0]) {
    //
    // Simple variable
    //
    
    for(int i = 0; i < numParams && command.var[i]; i++) {
      switch(command.varType) {
      case e_string:
	strncpy((char*) command.var[i], paramText[i], NAME_LEN-1);
	break;
      
      case e_uint16:
	*((uint16_t*) command.var[i]) = (uint16_t) param[i];
	break;
      
      case e_int8:
	*((int8_t*) command.var[i]) = (uint8_t) param[i];
	break;
      
      case e_float:
	*((float*) command.var[i]) = param[i];
	break;

      case e_percent:
	*((float*) command.var[i]) = param[i]/100;
	break;

      case e_angle90:
	*((float*) command.var[i]) = param[i]/90;
	break;

      case e_angle360:
	*((float*) command.var[i]) = param[i]/360;
	break;
      }
    }
  } else {
    //
    // Complex
    //

    int currentModel = nvState.model;
    float offset = 0.0;
    
    switch(command.token) {
    case c_atrim:
      vpParam.aileNeutral += vpParam.aileDefl*param[0];
      break;
      
    case c_etrim:
      vpParam.elevNeutral += vpParam.elevDefl*param[0];
      break;
      
    case c_rtrim:
      vpParam.rudderNeutral += vpParam.rudderDefl*param[0];
      break;
      
    case c_arm:
      vpMode.rattle = false;
      vpStatus.armed = true;
      break;
    
    case c_rattle:
      vpMode.rattle = true;
      vpStatus.armed = false;
      break;
    
    case c_disarm:
      vpStatus.armed = false;
      consoleNoteLn_P(PSTR("We're DISARMED"));
      break;
    
    case c_talk:
      vpStatus.silent = false;
      consoleNoteLn_P(PSTR("Hello world"));
      break;
    
    case c_test:
      if(numParams > 0)
	logTestSet(param[0]);

      consoleNote_P(PSTR("Current test channel = "));
      consolePrintLn(nvState.testNum);
      break;

    case c_calibrate:
      consoleNoteLn_P(PSTR("Receiver calibration STARTED"));
      calibStart();
      break;

    case c_rollrate:
      if(numParams > 0) {
	vpParam.roll_C
	  = param[0]/360/powf(vpParam.iasMin, stabilityAileExp2_c);
	consoleNote_P(PSTR("Roll rate K = "));
	consolePrintLn(vpParam.roll_C);
	storeNVState();
      }
      break;
          
    case c_pitchrate:
      if(numParams > 0) {
	vpParam.pitch_C
	  = param[0]/360/powf(vpParam.iasMin, stabilityElevExp2_c);
	consoleNote_P(PSTR("Pitch rate K = "));
	consolePrintLn(vpParam.pitch_C);
	storeNVState();
      }
      break;
          
    case c_alpha:
      if(numParams > 0)
	offset = param[0];
      
      vpParam.alphaRef += (int16_t) ((1L<<16) * (alpha - offset / 360));
      consoleNoteLn_P(PSTR("Alpha calibrated"));
      break;

    case c_gauge:
      if(numParams < 1) {
	gaugeCount = 1;
	gaugeVariable[0] = 1;
      } else {
	gaugeCount = numParams;
	
	for(int i = 0; i < numParams; i++)
	  gaugeVariable[i] = param[i];
      }
      break;
	
    case c_store:
      consoleNoteLn_P(PSTR("Params & NV state stored"));
      storeParams();
      storeNVState();
      break;

    case c_defaults:
      consoleNoteLn_P(PSTR("Default params restored"));
      defaultParams();
      storeNVState();
      break;

    case c_dump:
      logDumpBinary();
      break;
    
    case c_backup:
      for(int i = 0; i < maxModels(); i++) {
	if(setModel(i))
	  backupParams();
      }
      setModel(currentModel);
      break;

    case c_stamp:
      if(numParams > 0) {
	nvState.logStamp = param[0];
	storeNVState();
      }
      consoleNote_P(PSTR("Current log stamp is "));
      consolePrintLn(nvState.logStamp);  
      break;

    case c_beep:
      if(numParams > 0) {
	if(numParams > 1)
	  beepPrim(param[0], 1e3*param[1]);
	else
	  beepPrim(param[0], 1e3);
      }
      break;

    case c_model:
      if(numParams > 0) {
	if(param[0] > maxModels()-1)
	  param[0] = maxModels()-1;
	setModel(param[0]);
	storeNVState();
      } else { 
	consoleNote_P(PSTR("Current model is "));
	consolePrintLn(nvState.model); 
      }
      break;
    
   case c_params:
     consoleNote_P(PSTR("SETTINGS (MODEL "));
      consolePrint(nvState.model);
      consolePrintLn(")");
      printParams();
      break;

    case c_clear:
      logClear();
      break;

    case c_stop:
      logDisable();
      break;

    case c_start:
      logEnable();
      break;

    case c_log:
      vpMode.alwaysLog = true;
      break;
      
    case c_report:
      consoleNote_P(PSTR("Idle avg = "));
      consolePrintLn(idleAvg*100,1);
      consoleNote_P(PSTR("PPM frequency = "));
      consolePrintLn(ppmFreq);
      consoleNote_P(PSTR("Sim link frequency = "));
      consolePrintLn(simInputFreq);
      consoleNote_P(PSTR("Alpha = "));
      consolePrint(360*alpha);
      if(vpStatus.alphaFailed)
	consolePrintLn_P(PSTR(" FAIL"));
      else
	consolePrintLn_P(PSTR(" OK"));

      consoleNoteLn_P(PSTR("Sensor entropy"));
      consoleNote_P(PSTR("  Alpha = "));
      consolePrint(alphaEntropyAcc.output());
      consolePrint_P(PSTR("  IAS = "));
      consolePrintLn(iasEntropyAcc.output());

      consoleNoteLn_P(PSTR("Cycle time (ms)"));
      consoleNote_P(PSTR("  min        = "));
      consolePrintLn(cycleTimeMin*1e3);
      consoleNote_P(PSTR("  max        = "));
      consolePrintLn(cycleTimeMax*1e3);
      consoleNote_P(PSTR("  mean       = "));
      consolePrintLn(cycleTimeAverage.output()*1e3);
      consoleNote_P(PSTR("  cum. value = "));
      consolePrintLn(cycleTimeAcc.output()*1e3);
      consoleNote_P(PSTR("Warning flags :"));
      if(pciWarn)
	consolePrint_P(PSTR(" SPURIOUS_PCINT"));
      if(vpStatus.alphaWarn)
	consolePrint_P(PSTR(" ALPHA_SENSOR"));
      if(ppmWarnShort)
	consolePrint_P(PSTR(" PPM_SHORT"));
      if(ppmWarnSlow)
	consolePrint_P(PSTR(" PPM_SLOW"));
      if(vpStatus.eepromWarn)
	consolePrint_P(PSTR(" EEPROM"));
      if(vpStatus.eepromFailed)
	consolePrint_P(PSTR(" EEPROM_FAILED"));
      if(vpStatus.iasWarn)
	consolePrint_P(PSTR(" IAS_WARN"));
      if(vpStatus.iasFailed)
	consolePrint_P(PSTR(" IAS_FAILED"));
      if(pushCtrl.warn)
	consolePrint_P(PSTR(" PUSHER"));
      if(elevCtrl.warn)
	consolePrint_P(PSTR(" AUTOSTICK"));
      if(aileCtrl.warn)
	consolePrint_P(PSTR(" STABILIZER"));
      
      consolePrintLn("");

      if(vpMode.takeOff && tocTestStatus(true))
	consoleNoteLn_P(PSTR("T/O CONFIG GOOD"));

      consoleNote_P(PSTR("Log write bandwidth = "));
      consolePrint(logBandWidth);
      consolePrintLn_P(PSTR(" bytes/sec"));
      break;

    case c_reset:
      pciWarn = vpStatus.alphaWarn = vpStatus.alphaFailed = pushCtrl.warn = elevCtrl.warn
	= vpStatus.eepromWarn = vpStatus.eepromFailed = ppmWarnShort
	= ppmWarnSlow = aileCtrl.warn = false;
      consoleNoteLn_P(PSTR("Warning flags reset"));
      cycleTimeMonitorReset();
      break;
    
    default:
      consolePrint_P(PSTR("Sorry, command not implemented: \""));
      consolePrint(buf, tokenLen);
      consolePrintLn("\"");
      break;
    }
  }
}

//
// Periodic tasks
//

void cacheTask()
{
  cacheFlush();
}

void alphaTask()
{
  int16_t raw = 0;
  static int failCount = 0;
  static int16_t prev = 0;
  
  if(!handleFailure("alpha", !readAlpha_5048B(&raw), &vpStatus.alphaWarn, &vpStatus.alphaFailed, &failCount)) {
    alphaFilter.input((float) raw / (1L<<(8*sizeof(raw))));
    alphaEntropy += population(raw ^ prev);
    sensorHash = crc16(sensorHash, (uint8_t*) &raw, sizeof(raw));
    prev = raw;
  }
}

void airspeedTask()
{
  int16_t raw = 0;
  static int failCount = 0;
  static int16_t prev = 0;
  
  if(!handleFailure("airspeed", !readPressure(&raw), &vpStatus.iasWarn, &vpStatus.iasFailed, &failCount)) {
    pressureBuffer.input((float) raw);
    iasEntropy += population(raw ^ prev);
    sensorHash = crc16(sensorHash, (uint8_t*) &raw, sizeof(raw));
    prev = raw;
  }
}

DelayLine elevatorDelay, aileronDelay;

void receiverTask()
{
  if(inputValid(&aileInput))
    aileStick = applyNullZone(inputValue(&aileInput), &ailePilotInput);
  
  if(inputValid(&rudderInput))
    rudderStick = applyNullZone(inputValue(&rudderInput), &rudderPilotInput);
  
  if(inputValid(&elevInput))
    elevStick = applyNullZone(inputValue(&elevInput), &elevPilotInput);
    
  if(inputValid(&tuningKnobInput))
    tuningKnob = inputValue(&tuningKnobInput);
    
  if(inputValid(&throttleInput))
    throttleStick = inputValue(&throttleInput);

  flapSwitchValue = inputValue(&flapInput);
  
  buttonInput.input(inputValue(&modeSwitchInput));  

  AILEMODEBUTTON.input(buttonInput.value());
  ELEVMODEBUTTON.input(buttonInput.value());
  TRIMBUTTON.input(buttonInput.value());
  GEARBUTTON.input(buttonInput.value());

  //
  // Receiver fail detection
  //
  
  if(buttonInput.value() > 0.90 && aileStick < -0.90 && elevStick > 0.90) {
    if(!vpMode.rxFailSafe) {
      consoleNoteLn_P(PSTR("Receiver failsafe mode ENABLED"));
      vpMode.rxFailSafe = true;
      vpMode.alphaFailSafe = vpMode.sensorFailSafe = vpMode.takeOff = false;
    }
  } else if(vpMode.rxFailSafe) {
    consoleNoteLn_P(PSTR("Receiver failsafe mode DISABLED"));
    vpMode.rxFailSafe = false;
  }

  //
  // Apply rx failsafe settings
  //
  
  if(vpMode.rxFailSafe) {
    vpFeature.stabilizePitch = vpFeature.stabilizeBank
      = vpFeature.alphaHold = vpMode.bankLimiter = true;
    vpFeature.pusher = false;

    trimRateLimiter.setRate(1.5/360);
    elevTrim = elevFromAlpha(thresholdAlpha) - elevTrimSub;
  } else
    trimRateLimiter.setRate(1);
      
  // Delay the controls just to make sure we always detect the failsafe
  // mode before doing anything abrupt
  
  elevStick = elevatorDelay.input(elevStick);
  aileStick = aileronDelay.input(aileStick);
  
  //
  //
  //

  if(testPertubActive())
    switch(analyzerInputCh) {
    case ac_aile:
      aileStick = testPertub();
      break;

    case ac_elev:
      elevStick = testPertub();
      break;
    }
}

Damper simulatorAccX(50), simulatorAccY(50), simulatorAccZ(50);

void sensorTaskFast()
{
  // Alpha input
  
  alpha = alphaFilter.output();
  
  // Airspeed
  
  const float pascalsPerPSI_c = 6894.7573, range_c = 2*1.1;
  const float factor_c = pascalsPerPSI_c * range_c / (1L<<(8*sizeof(uint16_t)));
    
  dynPressure = pressureBuffer.output() * factor_c;
  
  // Attitude

  ins.wait_for_sample();
  
  ahrs.update();

  Vector3f gyro = ins.get_gyro();
  
  rollRate = gyro.x * RADIAN / 360;
  pitchRate = gyro.y * RADIAN / 360;
  yawRate = gyro.z * RADIAN / 360;

  bankAngle = ahrs.roll * RADIAN;
  pitchAngle = ahrs.pitch * RADIAN;
  heading = ahrs.yaw * RADIAN;

  // Acceleration
  
  Vector3f acc = ins.get_accel(0);

  accX = acc.x;
  accY = acc.y;
  accZ = -acc.z;

  ball.input(accY);
  
  // Altitude data acquisition

  barometer.update();
  barometer.accumulate();

  // Simulator link overrides
  
  if(vpStatus.simulatorLink) {
    alpha = sensorData.alpha/360;
    dynPressure = sensorData.qbar*PSF;
    rollRate = sensorData.rrate*RADIAN / 360;
    pitchRate = sensorData.prate*RADIAN / 360;
    yawRate = sensorData.yrate*RADIAN / 360;
    bankAngle = sensorData.roll*RADIAN;
    pitchAngle = sensorData.pitch*RADIAN;
    heading = sensorData.heading*RADIAN;
    accX = simulatorAccX.input(sensorData.accx*FOOT);
    accY = simulatorAccY.input(sensorData.accy*FOOT);
    accZ = simulatorAccZ.input(sensorData.accz*FOOT);
  }  

  //
  // Derived values
  //
  
  if(dynPressure > 0)
    iAS = sqrtf(2*dynPressure);
  else
    iAS = 0;
    
  effAlpha = clamp(alpha, -1.0/8, 1.0/8);
  iasFilter.input(iAS);
  iasFilterSlow.input(iAS);
}

void sensorTaskSlow()
{
  // Altitude

  if(vpStatus.simulatorLink)
    altitude = sensorData.alt*FOOT;
  else
    altitude = (float) barometer.get_altitude();
}

void sensorMonitorTask()
{
  //
  // Entropy monitor
  //

  iasEntropyAcc.input(iasEntropy);
  alphaEntropyAcc.input(alphaEntropy);
  iasEntropy = alphaEntropy = 0;

  //
  // Random seed from hashed sensor data
  //
  
  srand(sensorHash);
  
  //
  // Pitot block detection
  //
  
  static uint32_t iasLastAlive;

  if(iAS < vpParam.iasMin/3 || fabsf(iAS - iasFilterSlow.output()) > 0.5) {
    if(vpStatus.pitotBlocked) {
      consoleNoteLn_P(PSTR("Pitot block CLEARED"));
      vpStatus.pitotBlocked = false;
    }
    
    iasLastAlive = currentTime;
  } else if(!vpStatus.simulatorLink
	    && currentTime - iasLastAlive > 10.0e6 && !vpStatus.pitotBlocked) {
    consoleNoteLn_P(PSTR("Pitot appears BLOCKED"));
    vpStatus.pitotBlocked = true;
  }
}

void alphaLogTask()
{
  logAlpha();  
}

void slowLogTask()
{
  logAttitude();
  logInput();
  logActuator();
  logConfig();
  logPosition();
}

void measurementTask()
{
  static uint32_t prevMeasurement;
 
  // Idle measurement
  
  idleAvg = 7*idleAvg/8 + (float) idleMicros/1e6/8;
  idleMicros = 0;

  // PPM monitoring
  
  FORBID;
  ppmFreq = 1.0e6 * ppmFrames / (currentTime - prevMeasurement);
  ppmFrames = 0;
  PERMIT;

  // Sim link monitoring

  simInputFreq = 1.0e6 * simFrames / (currentTime - prevMeasurement);
  simFrames = 0;

  // Log bandwidth

  logBandWidth = 1.0e6 * writeBytesCum / (currentTime - prevMeasurement);
  writeBytesCum = 0;
  
  prevMeasurement = currentTime;
}

//
// Auto test stuff
//

const float testGainStep_c = 0.05;

bool increasing, autoTestCompleted;
int oscCount;
float finalK, finalKxIAS, finalT, finalIAS;

void testStateMachine()
{
  static uint32_t nextTransition;

  if(!vpMode.test || !vpMode.autoTest) {
    nextTransition = 0;
    testState = idle_c;
    return;
  }
  
  if(currentTime < nextTransition)
    return;
  
  switch(testState) {
  case start_c:
    increasing = true;
    oscCount = 0;
    testState = trim_c;
    nextTransition = currentTime+4*1e6;
    break;

  case trim_c:
    testState = init_c;
    break;
    
  case init_c:
    if(analyzerInputCh == ac_aile)
      pertubPolarity = -pertubPolarity;
    else
      pertubPolarity = 1.0;
    testState = pert0_c;
    nextTransition = currentTime+1e6/4;
    autoTestCompleted = false;
    break;

  case pert0_c:
    testState = pert1_c;
    nextTransition = currentTime+1e6/4;
    break;

  case pert1_c:
    testState = wait_c;
    nextTransition = currentTime+4*1e6;
    break;

  case wait_c:
    testState = measure_c;
    break;

  case measure_c:
    if(oscillating && ++oscCount > 1) {
      if(increasing) {
	increasing = false;	
	consoleNoteLn_P(PSTR("Reached stable oscillation"));
      }
      
      nextTransition = currentTime + 3*1e6/2;
      testGain /= 1 + testGainStep_c/5;
      consoleNote_P(PSTR("Gain decreased to = "));
      consolePrintLn(testGain);	
      testState = wait_c;

    } else if(increasing) {
      if(!oscillating)
	oscCount = 0;
      
      testGain *= 1 + testGainStep_c;
      testState = init_c;

      consoleNote_P(PSTR("Gain increased to = "));
      consolePrintLn(testGain);
    } else {
      finalK = testGain;
      finalIAS = iAS;
      finalT = oscCycleMean/1e6;
      finalKxIAS = finalK/powf(finalIAS, (analyzerInputCh == ac_aile ? stabilityAileExp1_c : stabilityElevExp1_c));
      
      consoleNote_P(PSTR("Test "));
      consolePrint(autoTestCount);
      consolePrint_P(PSTR(" COMPLETED, final K/IAS^exp1 = "));
      consolePrintLn(finalKxIAS);

      testState = idle_c;
      vpMode.test = vpMode.autoTest = false;
      autoTestCompleted = true;
    }
    break;
    
  default:
    break;
  }
}

Damper analAvg(10), analLowpass(2);
float upSwing, downSwing, prevUpSwing, prevDownSwing;

void analyzerTask()
{
  if(!vpMode.test || !vpMode.autoTest)
    return;
  
  switch(analyzerInputCh) {
  case ac_aile:
    analyzerInput = aileOutput;
    break;

  case ac_elev:
    analyzerInput = elevOutput;
    break;
  }

  analLowpass.input(analyzerInput);
  analAvg.input(analyzerInput);

  analyzerInput = analLowpass.output() - analAvg.output();

  bool crossing = false;
  static uint32_t prevCrossing;

  const float threshold_c = increasing ? 0.03 : 0.005;
  const float hystheresis_c = threshold_c/3;
  
  if(rising) {
    upSwing = fmaxf(upSwing, analyzerInput);
    
    if(analyzerInput < -hystheresis_c) {
      crossing = true;
      rising = false;
      prevUpSwing = upSwing;
      downSwing = analyzerInput;
    }
  } else {
    downSwing = fminf(downSwing, analyzerInput);
    
    if(analyzerInput > hystheresis_c) {
      crossing = true;
      rising = true;
      prevDownSwing = downSwing;
      upSwing = analyzerInput;
    }
  }

  float lastSwing = prevUpSwing - prevDownSwing;
  
  uint32_t halfCycle = currentTime - prevCrossing;  
  static uint32_t prevHalfCycle;

  if(crossing && lastSwing > threshold_c) {
    if(prevHalfCycle > 0) {
      cycleBuffer[cycleBufferPtr++] = prevHalfCycle + halfCycle;
      cycleBufferPtr %= cycleWindow_c;
      cycleCount++;

      if(cycleCount > cycleWindow_c-1) {
	oscCycleMean = 0.0;
      
	for(int i = 0; i < cycleWindow_c; i++)
	  oscCycleMean += cycleBuffer[i];

	oscCycleMean /= cycleWindow_c;

	if(!oscillating && oscCycleMean > 1/10.0) {
	  oscillating = true;

	  for(int i = 0; i < cycleWindow_c; i++)
	    if(cycleBuffer[i] < oscCycleMean/1.3
	       || cycleBuffer[i] > oscCycleMean*1.3)
	      oscillating = false;

	  	  if(oscillating)
	  	    consoleNoteLn_P(PSTR("Oscillation DETECTED"));
	}
      }
    }
    
    prevHalfCycle = halfCycle;
  } else if(oscillating && prevHalfCycle + halfCycle > oscCycleMean*1.3) {
    // Not oscillating (anymore)
    
    oscillating = false;
    prevHalfCycle = 0;
    upSwing = downSwing = prevUpSwing = prevDownSwing = 0.0;

     consoleNoteLn_P(PSTR("Oscillation STOPPED"));    
  }

  if(crossing)
    prevCrossing = currentTime;
}

//
//
//

const int paramSteps = 20;

static float testGainExpoGeneric(float range, float param)
{
  static float state;
  return exp(log(4)*(2*quantize(param, &state, paramSteps)-1))*range;
}

float testGainExpo(float range, float param)
{
  return testGainExpoGeneric(range, param);
}

float testGainExpoReversed(float range, float param)
{
  return testGainExpoGeneric(range, 1 - param);
}

float testGainLinear(float start, float stop, float param)
{
  static float state;
  float q = quantize(param, &state, paramSteps);
  return start + q*(stop - start);
}

float testGainExpo(float range)
{
  return testGainExpo(range, parameter);
}

float testGainExpoReversed(float range)
{
  return testGainExpoReversed(range, parameter);
}

float testGainLinear(float start, float stop)
{
  return testGainLinear(start, stop, parameter);
}

float s_Ku_ref, i_Ku_ref, p_Ku_ref;

const float minAlpha = (-2.0/360);
const float origoAlpha = (-5.0/360);

static float scaleByIAS(float k, float p)
{
  return k * powf(fmaxf(iasFilter.output(), vpParam.iasMin), p);
}

void configurationTask()
{
  //
  // Are we armed yet or being armed now?
  //
  
  if(!vpStatus.armed) {
    if(leftUpButton.doublePulse() && aileStick < -0.90 && elevStick > 0.90) {
      consoleNoteLn_P(PSTR("We're now ARMED"));
      vpStatus.armed = true;
      badBeep(1);
      
      if(!vpStatus.consoleLink)
	vpStatus.silent = true;

      leftDownButton.reset();
      rightUpButton.reset();
      rightDownButton.reset();
    } else
      return;
  }
  
  //
  // Flight detection
  //

  static uint32_t lastNegativeIAS;

  if(vpStatus.pitotBlocked || iasFilter.output() < vpParam.iasMin/2) {
    if(vpStatus.positiveIAS) {
      consoleNoteLn_P(PSTR("Positive airspeed LOST"));
      vpStatus.positiveIAS = false;
    }
    
    lastNegativeIAS = currentTime;

  } else if(currentTime - lastNegativeIAS > 1e6 && !vpStatus.positiveIAS) {
    consoleNoteLn_P(PSTR("We have POSITIVE AIRSPEED"));
    vpStatus.positiveIAS = true;
  }
  
  accTotal = sqrtf(square(accX) + square(accY) + square(accZ));
  
  accAvg.input(accTotal);

  float turnRate = sqrt(square(rollRate) + square(pitchRate) + square(yawRate));
  
  bool motionDetected = vpStatus.positiveIAS || turnRate > 5.0/360
    || fabsf(accTotal - accAvg.output()) > 0.3;
  
  static uint32_t lastMotion;

  if(motionDetected) {
    if(vpStatus.fullStop) {
      consoleNoteLn_P(PSTR("We appear to be MOVING"));
      vpStatus.fullStop = false;
    }
    
    lastMotion = currentTime;

  } else if(currentTime - lastMotion > 10.0e6 && !vpStatus.fullStop) {
    consoleNoteLn_P(PSTR("We have FULLY STOPPED"));
    vpStatus.fullStop = true;
  }

  //
  // Logging control
  //

  if(!vpStatus.fullStop && vpStatus.positiveIAS
     && (!vpStatus.consoleLink || vpMode.alwaysLog))
    logEnable();
  
  else if(vpStatus.fullStop)
    logDisable();

  //
  // T/O config test
  //

  if(vpMode.takeOff)
    tocTestUpdate();

  //
  // Configuration control
  //
  //   GEAR BUTTON

  if(GEARBUTTON.doublePulse()) {
    //
    // DOUBLE PULSE: FAILSAFE MODE SELECT
    //
    
    if(!vpMode.alphaFailSafe) {
      consoleNoteLn_P(PSTR("Alpha FAILSAFE"));
      vpMode.alphaFailSafe = true;
      logMark();
      
    } else if(!vpMode.sensorFailSafe) {
      consoleNoteLn_P(PSTR("Total sensor FAILSAFE"));
      vpMode.sensorFailSafe = true;
      logMark();
      
    } else if(!vpStatus.positiveIAS)
      logDisable();
    
  } else if(GEARBUTTON.singlePulse() && !gearOutput) {
    //
    // SINGLE PULSE: GEAR UP
    //
    
    consoleNoteLn_P(PSTR("Gear UP"));
    gearOutput = 1;

  } else if(GEARBUTTON.depressed() && gearOutput) {
    //
    // CONTINUOUS: GEAR DOWN
    //
    
    consoleNoteLn_P(PSTR("Gear DOWN"));
    gearOutput = 0;
  }

  //
  // AILE MODE BUTTON
  //
  
  if(AILEMODEBUTTON.singlePulse()) {
    //
    // PULSE : DISABLE BANK LIMITER
    //
  
    if(vpMode.alphaFailSafe || vpMode.sensorFailSafe) {
      consoleNoteLn_P(PSTR("Alpha/Sensor failsafe DISABLED"));
      vpMode.alphaFailSafe = vpMode.sensorFailSafe = false;
            
    } else {
      if(!vpMode.takeOff && iasFilter.output() < vpParam.iasMin*2/3) {
	consoleNoteLn_P(PSTR("TakeOff mode ENABLED"));
	goodBeep(0.5);
	
	vpMode.takeOff = true;
	vpMode.slowFlight = false;
	elevTrim = vpParam.takeoffTrim;
	
	tocTestReset();
	
      } else if(vpMode.takeOff) {
	if(tocTestStatus(true)) {
	  consoleNoteLn_P(PSTR("T/o configuration is GOOD"));
	  goodBeep(1);
	} else {
	  consoleNoteLn_P(PSTR("T/o configuration test FAILED"));
	  badBeep(5);
	}
      } else if(vpMode.bankLimiter) {
	consoleNoteLn_P(PSTR("Bank limiter DISABLED"));
	vpMode.wingLeveler = vpMode.bankLimiter = false;
	logMark();
      }
    }
  } else if(AILEMODEBUTTON.depressed()) {
    //
    // CONTINUOUS : LEVEL WINGS
    //
  
    if(!vpMode.bankLimiter) {
      consoleNoteLn_P(PSTR("Bank limiter ENABLED"));
      vpMode.bankLimiter = true;
    }
	
    if(!vpMode.wingLeveler) {
      consoleNoteLn_P(PSTR("Wing leveler ENABLED"));
      vpMode.wingLeveler = true;
    } 
  }

  //
  // ELEV MODE BUTTON
  //

  if(ELEVMODEBUTTON.singlePulse() && vpMode.slowFlight) {
    //
    // PULSE : DISABLE ALPHA HOLD
    //
  
    consoleNoteLn_P(PSTR("Slow flight mode DISABLED"));
    vpMode.slowFlight = false;
    logMark();
    
  } else if(ELEVMODEBUTTON.depressed() && !vpMode.slowFlight && !vpMode.takeOff) {
    //
    // CONTINUOUS : ENABLE ALPHA HOLD / SLOW FLIGHT
    //
  
    consoleNoteLn_P(PSTR("Slow flight mode ENABLED"));
    vpMode.slowFlight = true;
    logMark();
  }

  // Test parameter

  parameter = readParameter();

  if(!vpMode.test && parameter > 0.5) {
    vpMode.test = true;
    consoleNoteLn_P(PSTR("Test mode ENABLED"));

    if(autoTestCompleted) {
      if(autoTestCount < maxTests_c) {
	autoTestIAS[autoTestCount] = finalIAS;
	autoTestT[autoTestCount] = finalT;
	autoTestK[autoTestCount] = finalK;
	autoTestKxIAS[autoTestCount] = finalKxIAS;
	autoTestCount++;
      }

      autoTestCompleted = false;

      //      alphaTrim = (alphaTrim - origoAlpha) * 1.0555555 + origoAlpha;
      
      //      if(alphaTrim > shakerAlpha)
      //	alphaTrim -= shakerAlpha - minAlpha;
    }
  } else if(vpMode.test && parameter < 0) {
    vpMode.test = vpMode.autoTest = false;
    consoleNoteLn_P(PSTR("Test mode DISABLED"));

    if(autoTestCount > 0) {
      consolePrint_P(PSTR("testR = ["));

      for(int i = 0; i < autoTestCount; i++) {
	if(i > 0)
	  consolePrint(";\n  ");
	
	consolePrint(autoTestIAS[i]);
	consolePrint(", ");
	consolePrint(autoTestK[i]);
	consolePrint(", ");
	consolePrint(autoTestT[i]);
	consolePrint(", ");
	consolePrint(autoTestKxIAS[i]);
      }

      consolePrintLn_P(PSTR("]"));
      
      autoTestCount = 0;
    }
  }

  // Wing leveler disable when stick input detected
  
  if(vpMode.wingLeveler && ailePilotInput) {
    consoleNoteLn_P(PSTR("Wing leveler DISABLED"));
    vpMode.wingLeveler = false;
  }

  // TakeOff mode disabled when airspeed detected (or fails)

  if(vpMode.takeOff && (vpStatus.iasFailed
			|| iasFilter.output() > vpParam.iasMin*2/3)) {
    consoleNoteLn_P(PSTR("TakeOff mode DISABLED"));
    vpMode.takeOff = false;
  }

  //
  // Map mode to features (unless we're in receiver failsafe mode)
  //

  if(!vpMode.rxFailSafe) {
    // Default
    
    vpFeature.stabilizeBank = !vpMode.takeOff;
    vpFeature.pusher = !vpMode.takeOff && !vpMode.slowFlight;
    vpFeature.stabilizePitch = vpFeature.alphaHold
      = vpMode.slowFlight && !vpMode.takeOff;
    vpFeature.pitchHold = vpFeature.autoBall = false;
  
    // Failsafe mode interpretation

    if(vpMode.sensorFailSafe) {
      vpFeature.stabilizePitch = vpFeature.stabilizeBank
	= vpFeature.pitchHold = vpFeature.alphaHold = vpFeature.pusher
	= vpMode.bankLimiter = vpMode.wingLeveler = vpMode.takeOff
	= vpMode.slowFlight = false;

    } else if(vpMode.alphaFailSafe)
      vpFeature.stabilizePitch = vpFeature.pitchHold = vpFeature.alphaHold
	= vpFeature.pusher = vpMode.takeOff = vpMode.slowFlight = false;
  }
  
  // Safety scaling (test mode 0)
  
  float scale = 1.0;
  
  if(vpMode.test && nvState.testNum == 0)
    scale = testGainLinear(1.0/3, 1.0);
  
  // Default controller settings

  float s_Ku = scaleByIAS(vpParam.s_Ku_C, stabilityAileExp1_c);
  float i_Ku = scaleByIAS(vpParam.i_Ku_C, stabilityElevExp1_c);
  float p_Ku = scaleByIAS(vpParam.p_Ku, stabilityElevExp1_c);
  
  aileCtrl.setZieglerNicholsPID(s_Ku*scale, vpParam.s_Tu);
  elevCtrl.setZieglerNicholsPID(i_Ku*scale, vpParam.i_Tu);
  pushCtrl.setZieglerNicholsPID(p_Ku*scale, vpParam.p_Tu);
  pushCtrl.limit(0.3, 1);

  autoAlphaP = vpParam.o_P;
  maxAlpha = vpParam.alphaMax;
  rudderMix = vpParam.r_Mix;
  levelBank = 0;
  
  aileRateLimiter.setRate(vpParam.servoRate/(90.0/2)/vpParam.aileDefl);
  flapRateLimiter.setRate(0.5);
  
  // Then apply test modes
  
  if(vpMode.test && !vpMode.takeOff) {
    switch(nvState.testNum) {
    case 1:
      // Wing stabilizer gain
         
      vpFeature.stabilizeBank = vpMode.bankLimiter = vpMode.wingLeveler = true;
      aileCtrl.setPID(testGain = testGainExpo(s_Ku_ref), 0, 0);
      break;
            
    case 21:
      // Wing stabilizer gain autotest

      analyzerInputCh = ac_aile;
      vpFeature.stabilizeBank = vpMode.bankLimiter = vpMode.wingLeveler = true;
	
      if(!vpMode.autoTest) {
	vpMode.autoTest = true;
	testGain = 1.3*s_Ku;
	testState = start_c;
      } else if(testActive())
	aileCtrl.setPID(testGain, 0, 0);
      break;
      
    case 2:
      // Elevator stabilizer gain, outer loop disabled
         
      vpFeature.stabilizePitch = true;
      vpFeature.alphaHold = false;
      elevCtrl.setPID(testGain = testGainExpo(i_Ku_ref), 0, 0);
      break;
         
    case 22:
      // Elevator stabilizer gain, outer loop disabled
   
      analyzerInputCh = ac_elev;
      vpFeature.stabilizePitch = true;
      vpFeature.alphaHold = false;
	
      if(!vpMode.autoTest) {
	vpMode.autoTest = true;
	testGain = 1.3*i_Ku;
	testState = start_c;
      } else if(testActive())
	elevCtrl.setPID(testGain, 0, 0);
      break;
      
    case 3:
      // Elevator stabilizer gain, outer loop enabled
         
      vpFeature.stabilizePitch = vpFeature.alphaHold = true;
      elevCtrl.setPID(testGain = testGainExpo(i_Ku_ref), 0, 0);
      break;
         
    case 23:
      // Elevator stabilizer gain, outer loop enabled
         
      analyzerInputCh = ac_elev;
      vpFeature.stabilizePitch = vpFeature.alphaHold = true;
	
      if(!vpMode.autoTest) {
	vpMode.autoTest = true;
	testGain = 1.3*i_Ku;
	testState = start_c;
      } else if(testActive())
	elevCtrl.setPID(testGain, 0, 0);
      break;
         
    case 4:
      // Auto alpha outer loop gain
         
      vpFeature.stabilizePitch = vpFeature.alphaHold = true;
      autoAlphaP = testGain = testGainExpo(vpParam.o_P);
      break;
               
    case 5:
      // Pusher gain
         
      pushCtrl.setPID(testGain = testGainExpo(p_Ku_ref), 0, 0);
      break;
      
    case 6:
      // Aileron and rudder calibration, straight and level flight with
      // ball centered, reduced controller gain to increase stability
         
      vpFeature.stabilizeBank = vpMode.bankLimiter = vpMode.wingLeveler = true;
      vpFeature.autoBall = true;
      rudderMix = 0;
      aileCtrl.
	setZieglerNicholsPID(s_Ku*testGain, vpParam.s_Tu);
      //      rudderCtrl.
      //	setZieglerNicholsPI(vpParam.r_Ku*testGain, vpParam.r_Tu);
      break;

    case 7:
      // Auto ball gain
         
      vpFeature.stabilizeBank = vpMode.bankLimiter = vpMode.wingLeveler = true;
      vpFeature.autoBall = true;
      rudderMix = 0;
      //      rudderCtrl.setPID(testGain = testGainExpo(vpParam.r_Ku), 0, 0);
      break;
            
    case 8:
      // Auto ball empirical gain, PI
       
      vpFeature.autoBall = true;
      //      rudderCtrl.setZieglerNicholsPI(testGain = testGainExpo(vpParam.r_Ku),
      //				     vpParam.r_Tu);
      break;
       
    case 9:
      // Max alpha

      vpFeature.stabilizeBank = vpMode.bankLimiter = vpMode.wingLeveler = true;
      maxAlpha = testGain = testGainLinear(20.0/360, 10.0/360);
      break;         

    case 10:
      // Aileron to rudder mix

      rudderMix = testGain = testGainLinear(0.9, 0.0);
      break;
    }
  } else { 
    // Track s_Ku until a test is activated
    
    s_Ku_ref = s_Ku;
    i_Ku_ref = i_Ku;
    p_Ku_ref = p_Ku;
  }

  //
  // Effective alpha limits
  //
  
  thresholdAlpha = maxAlpha/square(1 + thresholdMargin_c);
  shakerAlpha = maxAlpha/square(1 + thresholdMargin_c/2);

  //
  // Trim adjustment by mode
  //

  if(!vpFeature.alphaHold)
    elevTrimSub =
      elevFromAlpha(clamp(effAlpha, vpParam.alphaZeroLift, maxAlpha))
      - elevStick - elevTrim;
}

void gaugeTask()
{
  if(gaugeCount > 0) {
    uint16_t tmp = 0;
	
    for(int g = 0; g < gaugeCount; g++) {
      switch(gaugeVariable[g]) {
      case 1:
	consolePrint_P(PSTR(" alpha = "));
	consolePrint(alpha*360, 1);
	consoleTab(15);
	consolePrint_P(PSTR(" KIAS = "));
	consolePrint((int) (iAS/KNOT));
	consoleTab(30);
	consolePrint_P(PSTR(" hdg = "));
	consolePrint((int) heading);
	consoleTab(45);
	consolePrint_P(PSTR(" alt = "));

	tmp = altitude/FOOT;
	
	if(tmp < 100)
	  consolePrint(tmp);
	else
	  consolePrint(((tmp+5)/10)*10);
	
	break;

      case 2:
	consolePrint_P(PSTR(" alpha(target) = "));
	consolePrint(alpha*360);
	consolePrint_P(PSTR(" ("));
	consolePrint(targetAlpha*360);
	consolePrint_P(PSTR(")"));
	consoleTab(25);
	consolePrint_P(PSTR(" pitchRate(target) = "));
	consolePrint(pitchRate*360, 1);
	consolePrint_P(PSTR(" ("));
	consolePrint(targetPitchRate*360);
	consolePrint_P(PSTR(")"));
	break;
	
      case 3:
	consolePrint_P(PSTR(" Cycle time (min, mean, max) = "));
	consolePrint(cycleTimeMin*1e3);
	consolePrint_P(PSTR(", "));
	consolePrint(cycleTimeAverage.output()*1e3);
	consolePrint_P(PSTR(", "));
	consolePrint(cycleTimeMax*1e3);
	break;
	
      case 4:
	consolePrint_P(PSTR(" bank = "));
	consolePrint(bankAngle, 2);
	consolePrint_P(PSTR(" pitch = "));
	consolePrint(pitchAngle, 2);
	consolePrint_P(PSTR(" heading = "));
	consolePrint(heading);
	consolePrint_P(PSTR(" alt = "));
	consolePrint(altitude);
	consolePrint_P(PSTR(" ball = "));
	consolePrint(ball.output(), 2);
	break;

      case 5:
	consolePrint_P(PSTR(" rollRate = "));
	consolePrint(rollRate*360, 1);
	consolePrint_P(PSTR(" pitchRate = "));
	consolePrint(pitchRate*360, 1);
	consolePrint_P(PSTR(" yawRate = "));
	consolePrint(yawRate*360, 1);
	break;

      case 6:
        consolePrint_P(PSTR(" ppmFreq = "));
	consolePrint(ppmFreq);
	consolePrint_P(PSTR(" InputVec = ( "));
	for(uint8_t i = 0; i < sizeof(ppmInputs)/sizeof(void*); i++) {
	  consolePrint(inputValue(ppmInputs[i]), 2);
	  consolePrint(" ");
	}      
	consolePrint(")");
	break;

      case 7:
	consolePrint_P(PSTR(" aileStick = "));
	consolePrint(aileStick);
	consolePrint_P(PSTR(" elevStick = "));
	consolePrint(elevStick);
	consolePrint_P(PSTR(" throttleStick = "));
	consolePrint(throttleStick);
	consolePrint_P(PSTR(" rudderStick = "));
	consolePrint(rudderStick);
	break;

      case 8:
	consolePrint_P(PSTR(" acc(avg) = "));
	consolePrint(accTotal);
	consolePrint_P(PSTR("("));
	consolePrint(accAvg.output());
	consolePrint_P(PSTR(") acc = ("));
	consolePrint(accX, 2);
	consolePrint_P(PSTR(", "));
	consolePrint(accY, 2);
	consolePrint_P(PSTR(", "));
	consolePrint(accZ, 2);
	consolePrint_P(PSTR(")"));
	break;

      case 9:
	consoleNote_P(PSTR(" entropy(alpha,ias) = "));
	consolePrint(alphaEntropyAcc.output());
	consolePrint_P(PSTR(", "));
	consolePrintLn(iasEntropyAcc.output());
	consolePrint_P(PSTR(" hash = "));
	
	tmp = sensorHash;

	for(int i = 0; i < 16; i++) {
	  consolePrint((tmp & 1) ? "+" : " ");
	  tmp = tmp >> 1;
	}
	break;
      }
    }

    consolePrint_P(PSTR("      "));
    consoleCR();
  }
}

void communicationTask()
{
  int len = 0;
       
  while((len = cliSerial->available()) > 0) {
    while(len-- > 0)
      datagramRxInputChar(cliSerial->read());
  }
}

/*
const int gpsBufLen = 1<<7, gpsMaxParam = 1<<5;
char gpsBuf[gpsBufLen], gpsMsg[gpsBufLen], gpsParam[gpsMaxParam+1];
int gpsBufIndex = 0, gpsMsgLen = 0;

const char *gpsParamIndex(int n)
{
  int start = 0, end = -1;
  int i = 0;

  do {
    if(end > -1)
      start = end+1;
      
    end = start;
    
    while(gpsMsg[end] != '\0' && gpsMsg[end] != ',')
      end++;
          
    if(i == n) {
      int len = min(end-start, gpsMaxParam);
      strncpy(gpsParam, &gpsMsg[start], len);
      gpsParam[len] = '\0';      
      break;
    }
      
    i++;
  } while(gpsMsg[end] != '\0');
  
  return gpsParam;
}

int hexDigit(char c)
{
  if(isdigit(c))
    return c - '0';
  else if(c >= 'A' && c <= 'F')
    return 10 + c - 'A';
  else if(c >= 'a' && c <= 'f')
    return 10 + c - 'a';
  else
    return -1;
}

bool gpsChecksum(void)
{
  if(gpsMsgLen < 3 || gpsMsg[gpsMsgLen-3] != '*')
    return false;
    
  uint8_t chkSum = hexDigit(gpsMsg[gpsMsgLen-2])*16+
    hexDigit(gpsMsg[gpsMsgLen-1]);
    
  uint8_t sum = 0;
  
  for(int i = 1; i < gpsMsgLen - 3; i++)
    sum ^= gpsMsg[i];
    
  return sum == chkSum;
}

void gpsSentence(const char *type)
{
  if(!strncmp("RMC", type, 3)) {
    if(!strcmp("A", gpsParamIndex(2))) {
      gpsFix.speed = atof(gpsParamIndex(7));
         gpsFix.track = atof(gpsParamIndex(8));
//      consoleNote("GPS speed = ");
//      consolePrintLn(gpsFix.speed);
    }    
  } else if(!strncmp("GGA", type, 3)) {
    if(atoi(gpsParamIndex(6)) > 0) {
      gpsFix.lat = atof(gpsParamIndex(2));
      gpsFix.lon = atof(gpsParamIndex(4));
      gpsFix.altitude = atof(gpsParamIndex(9));
//      consoleNote("GPS fix = ");
//      consolePrint(gpsFix.lat);
//      consolePrint(", ");
//      consolePrintLn(gpsFix.lon);
    }
  }
}

void gpsInput(const char *buf, int len)
{
  for(int i = 0; i < len; i++) {
    if(buf[i] == '\r') {
      if(!strncmp(gpsMsg, "$GP", 3) && gpsChecksum()) {
        gpsSentence(&gpsMsg[3]);
      } else
        consoleNoteLn("Corrupt GPS sentence");

      gpsMsgLen = 0;        
      gpsMsg[0] = '\0';
    } else if(buf[i] != '\n' && gpsMsgLen < gpsBufLen-1) {
      gpsMsg[gpsMsgLen++] = buf[i];
      gpsMsg[gpsMsgLen] = '\0';
    }
  }
}
*/

void gpsTask()
{
  /*
  int len = 0;
  bool dirty = false;
       
  while((len = Serial1.available()) > 0) {
    dirty = true;
    
    int spaceLeft = gpsBufLen - gpsBufIndex;
    
    if(len > spaceLeft) {
      for(int i = 0; i < len - spaceLeft; i++)
        Serial1.read();
      consoleNote("Lost ");
      consolePrintLn(len - spaceLeft);
    }
    
    len = min(len, spaceLeft);
    
    if(len > 0) {
      Serial1.readBytes(&gpsBuf[gpsBufIndex], len);
      gpsBufIndex += len;
    }
    
    if(gpsBufIndex > 0) {
//      consoleNote("Proc ");
//      consolePrintLn(gpsBufIndex);
      gpsInput(gpsBuf, gpsBufIndex);
      gpsBufIndex = 0;
    }        
  }
  */
}

float nominalPitchRate(float bank, float pitch, float target)
{
  const float alphaCL0 = vpParam.alphaZeroLift,
    ratio = (target - alphaCL0) / (vpParam.alphaMax - alphaCL0);
  
  //  return square(square(sin(bank/RADIAN)))
  //    *ratio*iasFilter.output()*G/square(vpParam.iasMin)*RADIAN/360;

  return G/iAS*(ratio*square(iAS/vpParam.iasMin)
  		- cos(bank/RADIAN))*RADIAN/360;
}

float nominalPitchRate(float bank, float target)
{
  const float alphaCL0 = vpParam.alphaZeroLift,
    ratio = (target - alphaCL0) / (vpParam.alphaMax - alphaCL0);
  
  return square(sin(bank/RADIAN))
    *ratio*iasFilter.output()*G/square(vpParam.iasMin)*RADIAN/360;
}

void controlTask()
{
  // Cycle time bookkeeping 
  
  if(controlCycleEnded > 0) {
    controlCycle = (currentTime - controlCycleEnded)/1.0e6;
    cycleTimeMonitor(controlCycle);
  }
  
  controlCycleEnded = currentTime;

  //
  // Elevator control
  //

  const float maxPitchRate
    = scaleByIAS(vpParam.pitch_C, stabilityElevExp2_c);  
  const float shakerLimit = (float) 2/3;
  const float effStick = vpMode.rxFailSafe ? shakerLimit : elevStick;
  const float stickStrength = fmaxf(effStick-shakerLimit, 0)/(1-shakerLimit);

  effTrim = vpFeature.alphaHold ? elevTrim + elevTrimSub : elevTrim;

  elevOutput = effStick + effTrim;
  
  const float effMaxAlpha = mixValue(stickStrength, shakerAlpha, maxAlpha);
    
  targetAlpha = trimRateLimiter.input
    (clamp(alphaFromElev(elevOutput), -vpParam.alphaMax, effMaxAlpha),
     controlCycle);

  if(vpFeature.alphaHold)
    targetPitchRate = nominalPitchRate(bankAngle, targetAlpha)
      + (targetAlpha - effAlpha)*autoAlphaP*maxPitchRate;
  
  else if(vpFeature.pitchHold)
    targetPitchRate = (5 + effStick*30 - pitchAngle)/90 * maxPitchRate;

  else
    targetPitchRate = effStick * maxPitchRate;

  elevOutputFeedForward = elevFromAlpha(targetAlpha);

  if(vpFeature.stabilizePitch) {
    elevCtrl.input(targetPitchRate - pitchRate, controlCycle);
    
    elevOutput = elevCtrl.output();

    static float elevTestBias = 0;

    if((vpMode.test && (nvState.testNum == 2 || nvState.testNum == 3))
    || (testActive() && analyzerInputCh == ac_elev))
      elevOutput += elevTestBias;
    else
      elevTestBias = elevOutput;
      
    if(vpFeature.alphaHold)
      elevOutput += elevOutputFeedForward;
  } else
    elevCtrl.reset(elevOutput - elevOutputFeedForward, 0.0);

  // Pusher

  if(vpFeature.pusher) {
    pushCtrl.input((effMaxAlpha - effAlpha)/effMaxAlpha, controlCycle);
    elevOutput = fminf(elevOutput, pushCtrl.output());
  } else
    pushCtrl.reset(elevOutput, (effMaxAlpha - effAlpha)/effMaxAlpha);
  
  elevOutput = clamp(elevOutput, -1, 1);
  
  // Aileron

  aileOutput = aileStick;
    
  const float maxRollRate = scaleByIAS(vpParam.roll_C, stabilityAileExp2_c);
  float maxBank = 45.0;

  if(vpMode.rxFailSafe)
    maxBank = 10.0;
  else if(vpFeature.alphaHold)
    maxBank /= 1 + alphaFromElev(effTrim) / thresholdAlpha / 2;
  
  float targetRollRate = maxRollRate*aileStick;

  if(!vpFeature.stabilizeBank) {

    if(vpMode.wingLeveler)
      aileOutput = clamp(aileOutput - bankAngle/60, -1, 1);
    
    aileCtrl.reset(aileOutput, 0);
    
  } else {
    
    const float factor_c = maxRollRate/60;

    if(vpMode.wingLeveler)
      // Strong leveler enabled
        
      targetRollRate =
	clamp((levelBank + aileStick*maxBank - bankAngle)*factor_c,
	      -maxRollRate, maxRollRate);

    else if(vpMode.bankLimiter) {
      // Bank limiter + weak leveling

      targetRollRate -=
	factor_c*clamp(bankAngle, -vpParam.wl_Limit, vpParam.wl_Limit);
      
      targetRollRate =
	clamp(targetRollRate,
	      (-maxBank - bankAngle)*factor_c, (maxBank - bankAngle)*factor_c);
    }
      
    aileCtrl.input(targetRollRate - rollRate, controlCycle);
    aileOutput = aileCtrl.output();
  }

  aileRateLimiter.input(clamp(aileOutput, -1, 1), controlCycle);
    
  // Rudder
    
  rudderOutput = rudderStick;
    
  if(vpFeature.autoBall) {
    const float factor_c = 1/9.81/4;

    if(!rudderPilotInput)
      rudderCtrl.input(-ball.output()*factor_c, controlCycle);
    
    rudderOutput += rudderCtrl.output();
  } else
    rudderCtrl.reset(rudderOutput, 0);

  // Aileron/rudder mix
  
  rudderOutput += aileRateLimiter.output()*rudderMix;

  rudderOutput = clamp(rudderOutput, -1, 1);

  // Flaps
  
  flapOutput = flapSwitchValue + 1;
  
  flapRateLimiter.input(flapOutput, controlCycle);
    
  // Brake
    
  if(gearOutput == 1 || elevStick > 0)
    brakeOutput = 0;
  else
    brakeOutput = -elevStick;
}

void actuatorTask()
{
  if(!vpStatus.armed)
    return;
  
  pwmOutputWrite(aileHandle, NEUTRAL
		 + RANGE*clamp(vpParam.aileDefl*aileRateLimiter.output()
			       + vpParam.aileNeutral, -1, 1));

  pwmOutputWrite(elevatorHandle, NEUTRAL
		 + RANGE*clamp(vpParam.elevDefl*elevOutput 
			       + vpParam.elevNeutral, -1, 1));

  pwmOutputWrite(rudderHandle, NEUTRAL
		 + RANGE*clamp(vpParam.rudderNeutral + 
			       vpParam.rudderDefl*rudderOutput, -1, 1));                        
  pwmOutputWrite(flapHandle, NEUTRAL
		 + RANGE*clamp(vpParam.flapNeutral 
			       + vpParam.flapStep*flapRateLimiter.output(), -1, 1));                              
  pwmOutputWrite(flap2Handle, NEUTRAL
		 + RANGE*clamp(vpParam.flap2Neutral 
			       - vpParam.flapStep*flapRateLimiter.output(), -1, 1));                              

  pwmOutputWrite(gearHandle, NEUTRAL + RANGE*(gearOutput*2-1));

  pwmOutputWrite(brakeHandle, NEUTRAL
		 + RANGE*clamp(vpParam.brakeDefl*brakeOutput 
			       + vpParam.brakeNeutral, -1, 1));
}

void trimTask()
{
  const float trimRateMin_c = 7.5/100, trimRateRange_c = 2*trimRateMin_c;

  if(TRIMBUTTON.state() && elevPilotInput) {
    const float trimRate = trimRateMin_c + fabsf(elevStick)*trimRateRange_c;
    elevTrim += sign(elevStick) * trimRate / TRIM_HZ;
  }
  
  const float trimMin = -0.20, trimMax = 0.80;
  
  if(!vpFeature.alphaHold)
    elevTrim = clamp(elevTrim, trimMin, trimMax);
  
  else
    elevTrim = clamp(elevTrim,
		     trimMin - elevTrimSub,
		     elevFromAlpha(thresholdAlpha) - elevTrimSub);
}

bool logInitialized = false;

void backgroundTask(uint32_t durationMicros)
{
  uint32_t idleStart = hal.scheduler->micros();
  
  if(!logInitialized)
    logInitialized = logInit(2*durationMicros);
  else
    hal.scheduler->delay(durationMicros/1000);

  idleMicros += hal.scheduler->micros() - idleStart;
}

void heartBeatTask()
{
  if(!heartBeatCount && linkDownCount++ > 2)
    vpStatus.consoleLink = vpStatus.simulatorLink = false;

  if(vpStatus.simulatorLink && currentTime - simTimeStamp > 1.0e6) {
    consoleNoteLn_P(PSTR("Simulator link LOST"));
    vpStatus.simulatorLink = false;
  }    
  
  heartBeatCount = 0;
  
  if(vpStatus.consoleLink) {
    static uint32_t count = 0;

    datagramTxStart(DG_HEARTBEAT);
    datagramTxOut((uint8_t*) &count, sizeof(count));
    datagramTxEnd();
  
    count++;   
  }
}

void blinkTask()
{
  float ledRatio = vpMode.test ? 0.0 : !logInitialized ? 1.0 : (vpMode.sensorFailSafe || !vpStatus.armed) ? 0.5 : alpha > 0.0 ? 0.90 : 0.10;
  static int tick = 0;
  
  tick = (tick + 1) % (LED_TICK/LED_HZ);

  setPinState(&RED_LED, tick < ledRatio*LED_TICK/LED_HZ ? 0 : 1);
}

void beepTask()
{
  static int phase = 0;

  if(beepDuration > 0) {
    if(beepGood)
      beepPrim(800, 1e3/BEEP_HZ);
    else {
      if(phase < 2) {
	beepPrim(phase < 1 ? 1000 : 750, 1e3/BEEP_HZ);
	phase++;
      } else
	phase = 0;
    }

    beepDuration--;
    controlCycleEnded = 0;
  } else
    phase = 0;
}

void simulatorLinkTask()
{
  if(vpStatus.simulatorLink && vpStatus.armed) {
    struct SimLinkControl control = { .aileron = aileRateLimiter.output(),
				      .elevator = -elevOutput,
				      .throttle = throttleStick,
				      .rudder = rudderOutput };

    datagramTxStart(DG_SIMLINK);
    datagramTxOut((const uint8_t*) &control, sizeof(control));
    datagramTxEnd();
  }
}

static void logStartCallback()
{
  alphaLogTask();
  slowLogTask();
}

void logSaveTask()
{
  logSave(logStartCallback);
}

void controlTaskGroup()
{
  testStateMachine();
  receiverTask();
  sensorTaskFast();
  controlTask();
  actuatorTask();
  analyzerTask();
}

struct Task taskList[] = {
  { communicationTask,
    HZ_TO_PERIOD(100) },
  //  { gpsTask, HZ_TO_PERIOD(100) },
  { alphaTask,
    HZ_TO_PERIOD(ALPHA_HZ) },
  { airspeedTask,
    HZ_TO_PERIOD(AIRSPEED_HZ) },
  { blinkTask,
    HZ_TO_PERIOD(LED_TICK) },
  { controlTaskGroup,
    HZ_TO_PERIOD(CONTROL_HZ) },
  { simulatorLinkTask,
    HZ_TO_PERIOD(CONTROL_HZ) },
  { sensorTaskSlow,
    HZ_TO_PERIOD(CONTROL_HZ/5) },
  { trimTask,
    HZ_TO_PERIOD(TRIM_HZ) },
  { sensorMonitorTask,
    HZ_TO_PERIOD(CONFIG_HZ) },
  { configurationTask,
    HZ_TO_PERIOD(CONFIG_HZ) },
  { alphaLogTask,
    HZ_TO_PERIOD(LOG_HZ_ALPHA) },
  { slowLogTask,
    HZ_TO_PERIOD(LOG_HZ_SLOW) },
  { logSaveTask,
    HZ_TO_PERIOD(LOG_HZ_SAVE) },
  { cacheTask,
    HZ_TO_PERIOD(LOG_HZ_SAVE) },
  { measurementTask,
    HZ_TO_PERIOD(1) },
  { heartBeatTask,
    HZ_TO_PERIOD(HEARTBEAT_HZ) },
  { gaugeTask,
    HZ_TO_PERIOD(10) },
  { beepTask,
    HZ_TO_PERIOD(BEEP_HZ) },
  { NULL } };

int scheduler()
{
  struct Task *task = taskList;
  
  while(task->code) {
    if(task->lastExecuted + task->period < currentTime
      || task->lastExecuted > currentTime) {
      task->code();
      task->lastExecuted = currentTime;
      
      if(task->period > 0)
        // Staggered execution for all but the critical tasks
        return 1;
    }
    
    task++;
  }

  // Nothing to do right now
  
  return 0;
}

void setup() {
  // HAL

  hal.init(0, NULL);
  
  // initialise serial port
  
  cliSerial = hal.console;

  vpStatus.consoleLink = true;
  
  consoleNoteLn_P(PSTR("Project | Alpha"));   
  consoleNote_P(PSTR("Init Free RAM: "));
  consolePrintLn((unsigned long) hal.util->available_memory());

  // PWM output

  consoleNoteLn_P(PSTR("Initializing PWM output"));

  pwmTimerInit(hwTimers, sizeof(hwTimers)/sizeof(struct HWTimer*));
  pwmOutputInitList(pwmOutput, sizeof(pwmOutput)/sizeof(struct PWMOutput));

  // I2C
  
  consoleNote_P(PSTR("Initializing I2C... "));
  
  I2c.begin();
  I2c.setSpeed(true);
  I2c.pullup(false);
  I2c.timeOut(2+EXT_EEPROM_LATENCY/1000);

  consolePrintLn_P(PSTR("done. "));
  
  // Read the non-volatile state

  readNVState();
    
  consoleNote_P(PSTR("Current model is "));
  consolePrintLn(nvState.model);
  
  // Param record
  
  setModel(nvState.model);

  // Set I2C speed
  
  TWBR = vpParam.i2c_clkDiv;
                
  // RC input
  
  consoleNoteLn_P(PSTR("Initializing PPM receiver"));

  configureInput(&ppmInputPin, true);
  
  ppmInputInit(ppmInputs, sizeof(ppmInputs)/sizeof(struct RxInputRecord*),
	       nvState.rxMin, nvState.rxCenter, nvState.rxMax);

  // Misc sensors
  
  consoleNote_P(PSTR("Initializing barometer... "));
  consoleFlush();

  barometer.init();
  barometer.calibrate();
  
  consolePrintLn_P(PSTR("  done"));
  
  consoleNote_P(PSTR("Initializing INS/AHRS... "));
  consoleFlush();
  
  ins.init(AP_InertialSensor::COLD_START, AP_InertialSensor::RATE_50HZ);
  ahrs.init();

  consolePrintLn_P(PSTR("  done"));
  
  // LED output

  configureOutput(&RED_LED);
  configureOutput(&GREEN_LED);
  configureOutput(&BLUE_LED);

  setPinState(&RED_LED, 1);
  setPinState(&GREEN_LED, 1);
  setPinState(&BLUE_LED, 1);

  // Piezo element
  
  pwmDisable(&PIEZO);
  setPinState(&PIEZO.pin, 0);
  configureOutput(&PIEZO.pin);

  // Alpha filter (sliding average over alphaWindow_c/seconds)
  
  alphaFilter.setWindow(alphaWindow_c*ALPHA_HZ);

  // Elevator delay

  elevatorDelay.setDelay(1);
  aileronDelay.setDelay(1);
  
  // Misc filters

  accAvg.reset(G);
  
  // Done
  
  consoleNoteLn_P(PSTR("Initialized"));
  goodBeep(0.5);
  
  datagramTxStart(DG_INITIALIZED);
  datagramTxEnd();
}

void loop() 
{
  // Invoke scheduler
  
  currentTime = hal.scheduler->micros();

  if(!scheduler())
    // Idle
      
    backgroundTask(1000);
}

AP_HAL_MAIN();
