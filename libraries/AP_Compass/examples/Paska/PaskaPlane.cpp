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

// #define USE_COMPASS  1

//
//
//

const float airDensity_c = 1.225;

const float stabilityElevExp_c = -1.5;
const float stabilityAileExp1_c = -1.5;
const float stabilityAileExp2_c = 0.5;
const float stabilityPusherExp_c = -0.5;

const float alphaWindow_c = 1.0/30;

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;
AP_HAL::BetterStream* cliSerial;

AP_Baro barometer;
AP_InertialSensor ins;
AP_GPS gps;
AP_AHRS_DCM ahrs {ins,  barometer, gps};
#ifdef USE_COMPASS
static Compass compass;
#endif

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
  buttonInput, tuningKnobInput, auxInput, modeInput;
struct RxInputRecord *ppmInputs[] = 
  { &aileInput, &elevInput, &throttleInput, &rudderInput, &buttonInput, &tuningKnobInput, &modeInput, &auxInput };

Button rightDownButton(-1.0), rightUpButton(0.33),
  leftDownButton(-0.3), leftUpButton(1);
struct SwitchRecord modeSelector = { &modeInput };
int8_t modeSelectorValue;

#define LEVELBUTTON rightUpButton
#define FLAPBUTTON rightDownButton
#define TRIMBUTTON leftUpButton
#define GEARBUTTON leftDownButton

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

#define PIEZO pwmOutput[4]

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
#define steerHandle \
  (vpParam.servoSteer < 0 ? NULL : &pwmOutput[vpParam.servoSteer])

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
#define LOG_HZ_FAST CONTROL_HZ
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
  bool bankLimiter;
  bool takeOff;
  bool wingLeveler;
  bool slowFlight;
  bool autoTest;
  bool loggingSuppressed;
};

struct FeatureRecord {
  bool keepLevel;
  bool stabilizeBank;
  bool stabilizePitch;
  bool pitchHold;
  bool alphaHold;
  bool pusher;
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

uint32_t currentTime, lastPPMWarn;
float controlCycle = 10.0e-3;
uint32_t idleMicros;
float idleAvg, logBandWidth, ppmFreq, simInputFreq;
float testGain = 0;
float iAS, dynPressure, alpha, aileStick, elevStick, throttleStick, rudderStick, tuningKnob;
bool ailePilotInput, elevPilotInput, rudderPilotInput;
uint32_t controlCycleEnded;
float elevTrim, elevTrimSub, targetAlpha;
Controller elevCtrl, pushCtrl;
UnbiasedController aileCtrl;
float autoAlphaP, rudderMix, stallAlpha, shakerAlpha, pusherAlpha;
float accX, accY, accZ, accTotal, altitude,  bankAngle, pitchAngle, rollRate, pitchRate, targetPitchRate, yawRate, levelBank;
uint16_t heading;
NewI2C I2c = NewI2C();
Damper ball(1.5*CONTROL_HZ), iasFilterSlow(3*CONTROL_HZ), iasFilter(2), accAvg(2*CONTROL_HZ), iasEntropyAcc(CONFIG_HZ), alphaEntropyAcc(CONFIG_HZ);
AlphaBuffer pressureBuffer;
RunningAvgFilter alphaFilter;
uint32_t simTimeStamp;
RateLimiter aileRateLimiter, flapRateLimiter, trimRateLimiter;
uint8_t flapOutput, gearOutput;
float elevOutput, elevOutputFeedForward, aileOutput = 0, brakeOutput = 0, rudderOutput = 0, steerOutput = 0;
uint16_t iasEntropy, alphaEntropy, sensorHash = 0xFFFF;
bool beepGood;
const int maxParams = 8;
int beepDuration, gaugeCount, gaugeVariable[maxParams];
I2CDevice alphaDevice(&I2c, 0, "alpha"), pitotDevice(&I2c, 0, "pitot");
I2CDevice eepromDevice(&I2c, 0, "EEPROM"), displayDevice(&I2c, 0, "display");

const int maxTests_c = 1;
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
// Link test
//

uint32_t pingTestTxCount, pingTestRxCount, pingTestFailCount, pingTestData, pingTestTxTime;

void pingTestRx(uint32_t value)
{
  if(!pingTestRxCount) {
    consoleNoteLn_P(PSTR("Unexpected PING datagram"));
    pingTestFailCount++;
  } else {
    if(pingTestData != value) {
      pingTestFailCount++;
      consoleNote_P(PSTR("Datagram PING FAIL, total = "));
      consolePrintLn(pingTestFailCount);
    }
    
    pingTestRxCount--;

    if(!pingTestRxCount) {
      if(pingTestFailCount > 0)
	consoleNoteLn_P(PSTR("Datagram ping test FAILED"));
      else
	consoleNoteLn_P(PSTR("Datagram ping test SUCCESS"));
    }
  }
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

void executeCommand(char *buf);

struct SimLinkSensor sensorData;
uint16_t simFrames;
int linkDownCount = 0, heartBeatCount = 0;
  
void datagramRxError(const char *error)
{
  consoleNote_P(PSTR("DG "));
  consolePrintLn(error);
}
  
void datagramInterpreter(uint8_t t, uint8_t *data, int size)
{
  uint32_t pingBuffer = 0;
  
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
    executeCommand((char*) data);
    break;

  case DG_SIMLINK:
    if(vpStatus.consoleLink && size == sizeof(sensorData)) {
      if(!vpStatus.simulatorLink) {
	consoleNoteLn_P(PSTR("Simulator CONNECTED"));
	vpStatus.simulatorLink = vpMode.loggingSuppressed = true;
      }

      memcpy(&sensorData, data, sizeof(sensorData));
      simTimeStamp = hal.scheduler->micros();
      simFrames++;    
    }
    break;

  case DG_PING:
    memcpy(&pingBuffer, data, sizeof(pingBuffer));
    pingTestRx(pingBuffer);
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

void datagramSerialFlush()
{
  serialFlush();
}
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
  if(!vpStatus.silent) {
    beepGood = good;
    beepDuration = dur*BEEP_HZ;
  } else
    beepDuration = 0;
}

void goodBeep(float dur)
{
  beep(dur, true);
}

void badBeep(float dur)
{
  beep(dur, false);
}

//
//
//

bool pitotFailed()
{
  return !vpStatus.simulatorLink && pitotDevice.status();
}

bool alphaFailed()
{
  return !vpStatus.simulatorLink && alphaDevice.status();
}

//
// Log interface
//

void logAlpha(void)
{
  logGeneric(lc_alpha, alpha*RADIAN);
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
  logGeneric(lc_target, targetAlpha*RADIAN);
  logGeneric(lc_target_pr, targetPitchRate*RADIAN);
  logGeneric(lc_trim, elevTrim*100);

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
  logGeneric(lc_roll, bankAngle*RADIAN);
  logGeneric(lc_rollrate, rollRate*RADIAN);
  logGeneric(lc_pitch, pitchAngle*RADIAN);
  logGeneric(lc_pitchrate, pitchRate*RADIAN);
  logGeneric(lc_heading, heading);
  logGeneric(lc_yawrate, yawRate*RADIAN);
}

//
// AS5048B (alpha) sensor interface
//

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

bool AS5048B_read(uint8_t addr, uint8_t *storage, uint8_t bytes) 
{
  return I2c.read(AS5048_ADDRESS, addr, storage, bytes) == 0;
}

bool AS5048B_read(uint8_t addr, uint16_t *result)
{
  uint8_t buf[sizeof(uint16_t)];
  bool success = false;
  
  success = AS5048B_read(addr, buf, sizeof(buf));
  
  if(success)
    *result = ((((uint16_t) buf[0]) << 6) + (buf[1] & 0x3F))<<2;

  return success;
}

bool AS5048B_alpha(int16_t *result)
{
  uint16_t raw = 0;
  bool success = AS5048B_read(AS5048B_ANGLMSB_REG, &raw);
  
  if(success && result)
    *result = (int16_t) (raw - vpParam.alphaRef);
  
  return success;
}

//
// OLED display interface
//

#define SSD1306_ADDR 0x3C
#define BLOCK 16

bool SSD1306_data(const uint8_t *storage, uint8_t bytes) 
{
  uint8_t buffer[1+BLOCK] = { (1<<6) };

  if(displayDevice.hasFailed())
    return false;
     
  if(bytes > BLOCK)
    bytes = BLOCK;
  
  for(int i = 0; i < bytes; i++)
    buffer[1+i] = storage[i];
  
  return displayDevice.handleStatus(I2c.write(SSD1306_ADDR, buffer, bytes+1));
}

bool SSD1306_zero(uint8_t bytes) 
{
  uint8_t buffer[1+BLOCK] = { (1<<6) };

  if(displayDevice.hasFailed())
    return false;
     
  memset(&buffer[1], 0, sizeof(buffer)-1);
  
  while(bytes > 0) {
    uint8_t block = bytes;
    
    if(block > BLOCK)
      block = BLOCK;
  
    if(!displayDevice.handleStatus(I2c.write(SSD1306_ADDR, buffer, block+1)))
      return false;

    bytes -= block;
  }

  return true;
}

bool SSD1306_command(const uint8_t value)
{
  uint8_t buffer[] = { 0, value };
  
  return !displayDevice.hasFailed()
    && displayDevice.handleStatus(I2c.write(SSD1306_ADDR, buffer, sizeof(buffer)));
}

//
// MS4525DO (dynamic pressure) sensor interface
//

bool MS4525DO_read(uint8_t *storage, uint8_t bytes) 
{
  const uint8_t addr_c = 0x28;
 
  return I2c.read(addr_c, NULL, 0, storage, bytes) == 0;
}

bool MS4525DO_read(uint16_t *result)
{
  uint8_t buf[sizeof(uint16_t)];
  bool success = false;
  
  success = MS4525DO_read(buf, sizeof(buf));
  
  if(success)
    *result = (((uint16_t) (buf[0] & 0x3F)) << 8) + buf[1];

  return success && (buf[0]>>6) == 0;
}

bool MS4525DO_pressure(int16_t *result) 
{
  static uint32_t acc;
  static int accCount;
  static bool done = false;
  const int log2CalibWindow = 8;
  
  uint16_t raw = 0;

  if(!MS4525DO_read(&raw))
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
// Cycle time monitoring
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
  toc_attitude,
  toc_gyro,
  toc_alpha,
  toc_mode,
  toc_link,
  toc_pitot,
  toc_lstick,
  toc_rstick,
  toc_timing,
  toc_tuning,
  toc_button,
  toc_fdr,
  toc_ram,
  toc_load,
} testCode_t;

#define TOC_TEST_NAME_MAX 16

struct TakeoffTest {
  char description[TOC_TEST_NAME_MAX];
  bool (*function)(bool);
};

const float toc_margin_c = 0.03;

bool toc_test_mode(bool reset)
{
  return !vpMode.test
    && vpMode.wingLeveler && vpMode.bankLimiter && vpMode.takeOff;
}

bool toc_test_link(bool reset)
{
  return currentTime - lastPPMWarn > 10e6 &&
    (!vpParam.virtualOnly || vpStatus.simulatorLink);
}

bool toc_test_ram(bool reset)
{
  return hal.util->available_memory() > (1<<9);
}

bool toc_test_timing(bool reset)
{
  return cycleTimeSampleAvailable
    && (cycleTimeMin >= 1.0/CONTROL_HZ)
    && (cycleTimeAverage.output() < 1.2/CONTROL_HZ);
}

bool toc_test_load(bool reset)
{
  return idleAvg > 0.20;
}

bool toc_test_fdr(bool reset)
{
  return !eepromDevice.status() && logReady(false) && logLen == 0;
}

bool toc_test_alpha_sensor(bool reset)
{
  return !alphaFailed() && alphaEntropyAcc.output() > 50;
}

bool toc_test_alpha_range(bool reset)
{
  static bool bigAlpha, zeroAlpha;
  static uint32_t lastNonZeroAlpha, lastSmallAlpha;

  if(reset) {
    zeroAlpha = bigAlpha = false;
    lastNonZeroAlpha = lastSmallAlpha = currentTime;
    
  } else if(!zeroAlpha) {
    if(fabs(alpha) > 1.5/RADIAN) {
      lastNonZeroAlpha = currentTime;
    } else if(currentTime > lastNonZeroAlpha + 1.0e6) {
      consoleNoteLn_P(PSTR("Stable ZERO ALPHA"));
      zeroAlpha = true;
    }
  } else if(!bigAlpha) {
    if(fabsf(alpha - 90/RADIAN) > 30/RADIAN) {
      lastSmallAlpha = currentTime;
    } else if(currentTime > lastSmallAlpha + 1.0e6) {
      consoleNoteLn_P(PSTR("Stable BIG ALPHA"));
      bigAlpha = true;
    }
  }
  
  return (bigAlpha && zeroAlpha) || vpStatus.simulatorLink;
}

bool toc_test_alpha(bool reset)
{
  return toc_test_alpha_sensor(reset) && toc_test_alpha_range(reset);
}

bool toc_test_pitot(bool reset)
{
  static bool positiveIAS;
  
  if(reset)
    positiveIAS = false;
  else if(vpStatus.positiveIAS)
    positiveIAS = true;
  
  return (!pitotFailed() && iasEntropyAcc.output() > 50
	  && !vpStatus.pitotBlocked && positiveIAS && iAS < 5)
    || vpStatus.simulatorLink;
}

bool toc_test_attitude(bool reset)
{
  return fabsf(pitchAngle) < 10.0/RADIAN && fabsf(bankAngle) < 2.5/RADIAN;
}

bool toc_test_gyro(bool reset)
{
  return (fabsf(pitchRate) < 1.0/RADIAN
	  && fabsf(rollRate) < 1.0/RADIAN
	  && fabsf(yawRate) < 1.0/RADIAN);
}

struct TOCRangeTestState {
  float valueMin, valueMax;
};

bool toc_test_range_generic(struct TOCRangeTestState *state, bool reset, struct RxInputRecord *input, float expectedMin, float expectedMax)
{
  const float value = inputValue(input);
  
  if(reset) {
    state->valueMin = state->valueMax = value;
    return true;
  } else {
    state->valueMin = fminf(state->valueMin, value);
    state->valueMax = fmaxf(state->valueMax, value);
  }
  
  return (fabsf(state->valueMin - expectedMin) < toc_margin_c
	  && fabsf(state->valueMax - expectedMax) < toc_margin_c);
}

bool toc_test_rstick_range(bool reset)
{
  static struct TOCRangeTestState stateElev, stateAile;
  return toc_test_range_generic(&stateElev, reset, &elevInput, -1, 1)
    && toc_test_range_generic(&stateAile, reset, &aileInput, -1, 1);
}

bool toc_test_rstick_neutral(bool reset)
{
  return ( fabsf(inputValue(&aileInput)) < toc_margin_c/2 )
    && ( fabsf(inputValue(&elevInput)) < toc_margin_c/2 );
}

bool toc_test_rstick(bool reset)
{
  return toc_test_rstick_range(reset) && toc_test_rstick_neutral(reset);
}

bool toc_test_lstick_range(bool reset)
{
  static struct TOCRangeTestState stateThr, stateRudder;
  return toc_test_range_generic(&stateThr, reset, &throttleInput, 0, 1)
    && toc_test_range_generic(&stateRudder, reset, &rudderInput, -1, 1);
}

bool toc_test_lstick_neutral(bool reset)
{
  return ( fabsf(inputValue(&rudderInput)) < toc_margin_c/2 )
    && ( fabsf(inputValue(&throttleInput)) < toc_margin_c/2 );
}

bool toc_test_lstick(bool reset)
{
  return toc_test_lstick_range(reset) && toc_test_lstick_neutral(reset);
}

bool toc_test_tuning_range(bool reset)
{
  static struct TOCRangeTestState state;
  return toc_test_range_generic(&state, reset, &tuningKnobInput, 0, 1);
}

bool toc_test_tuning_zero(bool reset)
{
  return fabsf(inputValue(&tuningKnobInput)) < toc_margin_c/2;
}

bool toc_test_tuning(bool reset)
{
  return toc_test_tuning_range(reset) && toc_test_tuning_zero(reset);
}

bool toc_test_button_range(bool reset)
{
  static struct TOCRangeTestState state;
  return toc_test_range_generic(&state, reset, &buttonInput, -1, 1);
}

bool toc_test_button_neutral(bool reset)
{
  return  !leftUpButton.state() && !leftDownButton.state()
    && !rightUpButton.state() && !rightDownButton.state();
}

bool toc_test_button(bool reset)
{
  return toc_test_button_range(reset) && toc_test_button_neutral(reset);
}

const struct TakeoffTest tocTest[] PROGMEM =
  {
    [toc_attitude] = { "ATTI", toc_test_attitude },
    [toc_gyro] = { "GYRO", toc_test_gyro },
    [toc_alpha] = { "ALPHA", toc_test_alpha },
    [toc_mode] = { "MODE", toc_test_mode },
    [toc_link] = { "LINK", toc_test_link },
    [toc_pitot] = { "PITOT", toc_test_pitot },
    [toc_lstick] = { "LSTK", toc_test_lstick },
    [toc_rstick] = { "RSTK", toc_test_rstick },
    [toc_timing] = { "TIMNG", toc_test_timing },
    [toc_tuning] = { "TUNE", toc_test_tuning },
    [toc_button] = { "BUTN", toc_test_button },
    [toc_fdr] = { "FDR", toc_test_fdr },
    [toc_ram] = { "RAM", toc_test_ram },
    [toc_load] = { "LOAD", toc_test_load }
  };

const int tocNumOfTests = sizeof(tocTest)/sizeof(struct TakeoffTest);

bool tocTestInvoke(bool reset, bool challenge, void (*report)(bool, int, const char *))
{
  bool fail = false;
  struct TakeoffTest cache;
    
  for(int i = 0; i < tocNumOfTests; i++) {
    memcpy_P(&cache, &tocTest[i], sizeof(cache));

    bool result = (*cache.function)(reset);
    
    if(challenge) {
      (*report)(result, i, cache.description);

      if(!result)
	fail = true;
    }
  }

  return !fail;
}

bool tocTestReset()
{
  return tocTestInvoke(true, false, NULL);
}

void tocTestUpdate()
{
  tocTestInvoke(false, false, NULL);
}

bool tocStatusFailed;

void tocReportConsole(bool result, int i, const char *s)
{
  if(result)
    return;
  
  if(!tocStatusFailed) {
    consoleNote_P(PSTR("T/O/C FAIL :"));
    tocStatusFailed = true;
  }

  consolePrint(" ");
  consolePrint(s);
}

bool tocTestStatus(void (*reportFn)(bool, int, const char*))
{
  tocStatusFailed = false;
  return tocTestInvoke(false, true, reportFn);
}

//
// Command interpreter
//

const prog_char_t *applyParamUpdate()
{
  /*
  vpParam.i_Ku_C /= 2*PI;
  vpParam.s_Ku_C /= 2*PI;
  vpParam.p_Ku_C /= 2*PI;
  vpParam.o_P /= 2*PI;
  return PSTR("i_Ku, s_Ku, p_Ku, o_P scaled by 1/2/PI");
  */
  /*  
  vpParam.ff_B *= RADIAN;
  return PSTR("ff_B scaled by RADIAN");
  */
  return NULL;
}

char *parse(char *ptr)
{
  while(*ptr && !isblank(*ptr))
    ptr++;

  if(*ptr) {
    *ptr++ = '\0';
    while(*ptr && isblank(*ptr))
      ptr++;
  }

  return ptr;
}

void printCoeffElement(float y0, float y1, float x, float v)
{
    consoleNote("");
    consolePrint(x);
    consoleTab(10);

    const int col1 = 78, col0 = 10+(col1-10)*-y0/(y1-y0);
    int y = col0 + (col1-col0) * v / y1;

    if(y < col0) {
      consoleTab(y);
      consolePrint("*");
      consoleTab(col0);
      consolePrint("|");
      consoleTab(col1);
      consolePrintLn("|");
    } else if(y > col0) {
      consoleTab(col0);
      consolePrint("|");
      consoleTab(y);
      if(y < col1) {
	consolePrint("*");
	consoleTab(col1);
	consolePrintLn("|");
      } else
	consolePrintLn("*");
    } else {
      consoleTab(col0);
      consolePrint("*");
      consoleTab(col1);
      consolePrintLn("|");
    }
}

void executeCommand(char *buf)
{
  while(*buf && isblank(*buf))
    buf++;
  
  consolePrint_P(PSTR("// % "));
  consolePrintLn(buf);

  if(!*buf || buf[0] == '/') {
    gaugeCount = 0;
    calibStop(nvState.rxMin, nvState.rxCenter, nvState.rxMax);
    return;
  } else if(atoi(buf) > 0) {
    gaugeCount = 1;
    gaugeVariable[0] = atoi(buf);
    return;
  }
  
  int numParams = 0;
  float param[maxParams];
  const char *paramText[maxParams];

  for(int i = 0; i < maxParams; i++)
    param[i] = 0.0;

  char *parsePtr = buf;
  
  while(*(parsePtr = parse(parsePtr))) {
    if(numParams < maxParams) {
      paramText[numParams] = parsePtr;
      param[numParams] = atof(paramText[numParams]);
      numParams++;
    }
  }
  
  int matches = 0, j = 0;
  struct Command command;
  
  while(1) {
    struct Command cache;
  
    memcpy_P(&cache, &commands[j++], sizeof(cache));

    if(cache.token == c_invalid)
      break;
    
    if(!strncmp(buf, cache.name, strlen(buf))) {
      command = cache;
      matches++;
    }
  }
  
  if(matches < 1) {
    consolePrint_P(PSTR("Command not recognized: \""));
    consolePrint(buf);
    consolePrintLn("\"");
    
  } else if(matches > 1) {
    consolePrint_P(PSTR("Ambiguos command: \""));
    consolePrint(buf);
    consolePrintLn("\"");
    
  } else if(command.var[0]) {
    //
    // Simple variable
    //
    
    for(int i = 0; command.var[i]; i++) {
      const char *txt = i < numParams ? paramText[i] : "";
	
      switch(command.varType) {
      case e_string:
	strncpy((char*) command.var[i], txt, NAME_LEN-1);
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

      case e_angle:
	*((float*) command.var[i]) = param[i]/RADIAN;
	break;

      case e_angle90:
	*((float*) command.var[i]) = param[i]/90;
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
    case c_ping:
      pingTestTxCount = pingTestRxCount = param[0];
      pingTestFailCount = 0;
      break;
      
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
	  = param[0]/RADIAN/powf(vpDerived.stallIAS, stabilityAileExp2_c);
	consoleNote_P(PSTR("Roll rate K = "));
	consolePrintLn(vpParam.roll_C);
	storeNVState();
      }
      break;
          
    case c_alpha:
      if(numParams > 0)
	offset = param[0];
      
      vpParam.alphaRef +=
	(int16_t) ((1L<<16) * (alpha - offset / RADIAN) / CIRCLE);
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
      break;

    case c_dump:
      logDumpBinary();
      break;
    
    case c_backup:
      for(int i = 0; i < maxModels(); i++) {
	if(setModel(i, false))
	  backupParams();
      }
      setModel(currentModel, false);
      break;

    case c_update:
      if(!updateDescription) {
	for(int i = 0; i < maxModels(); i++) {
	  if(setModel(i, false)) {
	    updateDescription = applyParamUpdate();
	    storeParams();
	  }
	}
	
	setModel(currentModel, false);
	
	if(updateDescription) {
	  consoleNote_P(PSTR("Param update applied : "));
	  consolePrintLn_P(updateDescription);
	}
      }
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
	setModel(param[0], true);
	storeNVState();
      } else { 
	consoleNote_P(PSTR("Current model is "));
	consolePrintLn(nvState.model); 
      }
      break;
    
    case c_trim:
      if(numParams > 0)
	elevTrim = param[0]/100;
      consoleNote_P(PSTR("Current elev trim(%) = "));
      consolePrintLn(elevTrim*100); 
      break;
      
    case c_params:
     consoleNote_P(PSTR("SETTINGS (MODEL "));
      consolePrint(nvState.model);
      consolePrintLn(")");
      printParams();
      break;

    case c_delete:
      if(numParams > 0) {
	if(param[0] > maxModels()-1)
	  param[0] = maxModels()-1;
	deleteModel(param[0]);
      }
      break;

    case c_curve:
      consoleNoteLn_P(PSTR("Feed-forward curve"));
  
      for(float aR = -1; aR <= 1; aR += 0.07)
	printCoeffElement(-1, 1, vpParam.alphaMax*aR*RADIAN, elevFromAlpha(vpParam.alphaMax*aR));

      consoleNoteLn_P(PSTR("Inverse feed-forward curve"));
  
      for(float e = 1; e >= -1; e -= 0.07)
	printCoeffElement(-vpParam.alphaMax/2, vpParam.alphaMax, e, alphaFromElev(e));

      consoleNoteLn_P(PSTR("Coeff of lift per Wing Load"));
  
      for(float aR = -0.2; aR <= 1.2; aR += 0.07)
	printCoeffElement(-0.2, 1, vpParam.alphaMax*aR*RADIAN, coeffOfLift(vpParam.alphaMax*aR)/vpParam.cL_max);
      break;
      
    case c_clear:
      logClear();
      break;

    case c_init:
      logInit();
      break;

    case c_stop:
      logDisable();
      break;

    case c_start:
      logEnable();
      break;

    case c_log:
      vpMode.loggingSuppressed = false;
      break;

    case c_zl:
      if(numParams > 0) {
	vpParam.cL_B = vpParam.cL_max/(vpParam.alphaMax - param[0]/RADIAN);
	vpParam.cL_A = -vpParam.cL_B*param[0]/RADIAN;
      }
      break;
      
    case c_peak:
      if(numParams > 0)
	vpParam.cL_B =
	  (1+param[0])*(vpParam.cL_max - vpParam.cL_A)/vpParam.alphaMax;
      break;
      
    case c_stall:
      if(numParams > 0) {
	vpParam.cL_max = 2*G / square(param[0]);
	if(numParams > 1)
	  vpParam.alphaMax = param[1]/RADIAN;
      }
      break;
      
    case c_max:
      if(numParams > 0)
	vpParam.alphaMax = param[0]/RADIAN;
      break;
      
    case c_report:
      consoleNote_P(PSTR("Idle avg = "));
      consolePrintLn(idleAvg*100,1);
      consoleNote_P(PSTR("PPM frequency = "));
      consolePrintLn(ppmFreq);
      consoleNote_P(PSTR("Sim link frequency = "));
      consolePrintLn(simInputFreq);
      consoleNote_P(PSTR("Alpha = "));
      consolePrint(alpha*RADIAN);
      if(alphaFailed())
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
      if(ppmWarnShort)
	consolePrint_P(PSTR(" PPM_SHORT"));
      if(ppmWarnSlow)
	consolePrint_P(PSTR(" PPM_SLOW"));
      if(eepromDevice.status())
	consolePrint_P(PSTR(" EEPROM_FAILED"));
      if(pitotFailed())
	consolePrint_P(PSTR(" IAS_FAILED"));
      
      consolePrintLn("");

      consoleNote_P(PSTR("Log write bandwidth = "));
      consolePrint(logBandWidth);
      consolePrintLn_P(PSTR(" bytes/sec"));
      
      break;
      
    case c_reset:
      pciWarn = ppmWarnShort = ppmWarnSlow = false;
      cycleTimeMonitorReset();
      consoleNoteLn_P(PSTR("Warning flags reset"));
      break;

    default:
      consolePrint_P(PSTR("Sorry, command not implemented: \""));
      consolePrint(buf);
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
  static int16_t prev = 0;
  
  if(!alphaDevice.hasFailed()
     && !alphaDevice.handleStatus(!AS5048B_alpha(&raw))) {
    alphaFilter.input(CIRCLE*(float) raw / (1L<<(8*sizeof(raw))));
    alphaEntropy += ABS(raw - prev);
    sensorHash = crc16(sensorHash, (uint8_t*) &raw, sizeof(raw));
    prev = raw;
  }
}

#define SSD1306_128_64 1

#if defined SSD1306_128_64
  #define SSD1306_LCDWIDTH                  128
  #define SSD1306_LCDHEIGHT                 64
#endif
#if defined SSD1306_128_32
  #define SSD1306_LCDWIDTH                  128
  #define SSD1306_LCDHEIGHT                 32
#endif
#if defined SSD1306_96_16
  #define SSD1306_LCDWIDTH                  96
  #define SSD1306_LCDHEIGHT                 16
#endif

#define SSD1306_SETCONTRAST 0x81
#define SSD1306_DISPLAYALLON_RESUME 0xA4
#define SSD1306_DISPLAYALLON 0xA5
#define SSD1306_NORMALDISPLAY 0xA6
#define SSD1306_INVERTDISPLAY 0xA7
#define SSD1306_DISPLAYOFF 0xAE
#define SSD1306_DISPLAYON 0xAF

#define SSD1306_SETDISPLAYOFFSET 0xD3
#define SSD1306_SETCOMPINS 0xDA

#define SSD1306_SETVCOMDETECT 0xDB

#define SSD1306_SETDISPLAYCLOCKDIV 0xD5
#define SSD1306_SETPRECHARGE 0xD9

#define SSD1306_SETMULTIPLEX 0xA8

#define SSD1306_SETLOWCOLUMN 0x00
#define SSD1306_SETHIGHCOLUMN 0x10

#define SSD1306_SETSTARTLINE 0x40

#define SSD1306_MEMORYMODE 0x20
#define SSD1306_COLUMNADDR 0x21
#define SSD1306_PAGEADDR   0x22

#define SSD1306_COMSCANINC 0xC0
#define SSD1306_COMSCANDEC 0xC8

#define SSD1306_SEGREMAP 0xA0

#define SSD1306_CHARGEPUMP 0x8D

#define SSD1306_EXTERNALVCC 0x1
#define SSD1306_SWITCHCAPVCC 0x2

// Scrolling #defines
#define SSD1306_ACTIVATE_SCROLL 0x2F
#define SSD1306_DEACTIVATE_SCROLL 0x2E
#define SSD1306_SET_VERTICAL_SCROLL_AREA 0xA3
#define SSD1306_RIGHT_HORIZONTAL_SCROLL 0x26
#define SSD1306_LEFT_HORIZONTAL_SCROLL 0x27
#define SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL 0x29
#define SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL 0x2A

uint8_t displayBuffer[16*8];
uint8_t cursorCol, cursorRow;
int8_t modifiedLeft[8], modifiedRight[8];

void cursorMove(uint8_t col, uint8_t row)
{
  cursorCol = col;
  cursorRow = row;
}

void markModified(uint8_t col)
{
  if(col > 15)
    col = 15;
  
  if(modifiedLeft[cursorRow] < 0)
    modifiedLeft[cursorRow] = modifiedRight[cursorRow] = col;
  else {
    modifiedLeft[cursorRow] = MIN(modifiedLeft[cursorRow], col);
    modifiedRight[cursorRow] = MAX(modifiedRight[cursorRow], col);
  }
}

bool inverseVideo = false;

void setAttr(bool inverse)
{
  inverseVideo = inverse;
}

void print(const char *s, int l)
{
  for(int i = 0; i < l; i++) {
    uint8_t c = s ? s[i] : '\0';

    if(inverseVideo)
      c |= 0x80;
    
    if(displayBuffer[cursorRow*16 + cursorCol] != c) {
      displayBuffer[cursorRow*16 + cursorCol] = c;
      markModified(cursorCol);
    }

    if(cursorCol < 16-1)
      cursorCol++;
    else {
      cursorCol = 0;

      if(cursorRow < 8-1)
	cursorRow++;
      else
	cursorRow = 0;
    }
  }

  inverseVideo = false;
}

void printNL()
{
  print(NULL, 16-cursorCol);
}

void print(const char *s)
{
  print(s, strlen(s));
}

const uint8_t fontData[] PROGMEM = {
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0x0, 0x0, 0x80, 0x60, 0x60, 0x0, 0x0, 0x0,  // Char ','
0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x0,  // Char '-'
0x0, 0x0, 0x0, 0xC0, 0xC0, 0x0, 0x0, 0x0,  // Char '.'
0x80, 0x40, 0x20, 0x10, 0x8, 0x4, 0x2, 0x0,  // Char '/'
0x7C, 0xC2, 0xA2, 0x92, 0x8A, 0x86, 0x7C, 0x0,  // Char '0'
0x0, 0x0, 0x84, 0xFE, 0x80, 0x0, 0x0, 0x0,  // Char '1'
0x84, 0xC2, 0xA2, 0xA2, 0x92, 0x92, 0x8C, 0x0,  // Char '2'
0x44, 0x82, 0x82, 0x82, 0x92, 0x92, 0x6C, 0x0,  // Char '3'
0x0, 0x1E, 0x10, 0x10, 0x10, 0x10, 0xFC, 0x0,  // Char '4'
0x9E, 0x92, 0x92, 0x92, 0x92, 0x92, 0x62, 0x0,  // Char '5'
0x7C, 0x92, 0x92, 0x92, 0x92, 0x92, 0x60, 0x0,  // Char '6'
0x82, 0x42, 0x22, 0x12, 0xA, 0x6, 0x2, 0x0,  // Char '7'
0x6C, 0x92, 0x92, 0x92, 0x92, 0x92, 0x6C, 0x0,  // Char '8'
0xC, 0x92, 0x92, 0x92, 0x92, 0x92, 0x7C, 0x0,  // Char '9'
0x0, 0x0, 0xCC, 0xCC, 0x0, 0x0, 0x0, 0x0,  // Char ':'
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0x0, 0x28, 0x28, 0x28, 0x28, 0x28, 0x0, 0x0,  // Char '='
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0xC0, 0x30, 0x2C, 0x22, 0x2C, 0x30, 0xC0, 0x0,  // Char 'A'
0xFE, 0x92, 0x92, 0x92, 0x92, 0x92, 0x6C, 0x0,  // Char 'B'
0x38, 0x44, 0x82, 0x82, 0x82, 0x82, 0x44, 0x0,  // Char 'C'
0xFE, 0x82, 0x82, 0x82, 0x82, 0x44, 0x38, 0x0,  // Char 'D'
0xFE, 0x92, 0x92, 0x92, 0x92, 0x82, 0x82, 0x0,  // Char 'E'
0xFE, 0x12, 0x12, 0x12, 0x12, 0x2, 0x2, 0x0,  // Char 'F'
0x7C, 0x82, 0x82, 0x82, 0x92, 0x92, 0x64, 0x0,  // Char 'G'
0xFE, 0x10, 0x10, 0x10, 0x10, 0x10, 0xFE, 0x0,  // Char 'H'
0x0, 0x0, 0x82, 0xFE, 0x82, 0x0, 0x0, 0x0,  // Char 'I'
0x60, 0x80, 0x80, 0x80, 0x82, 0x7E, 0x2, 0x0,  // Char 'J'
0xFE, 0x20, 0x10, 0x28, 0x44, 0x82, 0x0, 0x0,  // Char 'K'
0xFE, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x0,  // Char 'L'
0xFE, 0x4, 0x8, 0x10, 0x8, 0x4, 0xFE, 0x0,  // Char 'M'
0xFE, 0x4, 0x8, 0x10, 0x20, 0x40, 0xFE, 0x0,  // Char 'N'
0x7C, 0x82, 0x82, 0x82, 0x82, 0x82, 0x7C, 0x0,  // Char 'O'
0xFE, 0x12, 0x12, 0x12, 0x12, 0x12, 0xC, 0x0,  // Char 'P'
0xFC, 0x42, 0xA2, 0x92, 0x82, 0x82, 0x7C, 0x0,  // Char 'Q'
0xFE, 0x12, 0x12, 0x32, 0x52, 0x92, 0xC, 0x0,  // Char 'R'
0x4C, 0x92, 0x92, 0x92, 0x92, 0x92, 0x64, 0x0,  // Char 'S'
0x2, 0x2, 0x2, 0xFE, 0x2, 0x2, 0x2, 0x0,  // Char 'T'
0x7E, 0x80, 0x80, 0x80, 0x80, 0x80, 0x7E, 0x0,  // Char 'U'
0x6, 0x18, 0x60, 0x80, 0x60, 0x18, 0x6, 0x0,  // Char 'V'
0x7E, 0x80, 0x80, 0x70, 0x80, 0x80, 0x7E, 0x0,  // Char 'W'
0x82, 0x44, 0x28, 0x10, 0x28, 0x44, 0x82, 0x0,  // Char 'X'
0x2, 0x4, 0x8, 0xF0, 0x8, 0x4, 0x2, 0x0,  // Char 'Y'
0x82, 0xC2, 0xA2, 0x92, 0x8A, 0x86, 0x82, 0x20,  // Char 'Z'
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x0,  // Char '_'
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0xC0, 0x30, 0x2C, 0x22, 0x2C, 0x30, 0xC0, 0x0,  // Char 'a'
0xFE, 0x92, 0x92, 0x92, 0x92, 0x92, 0x6C, 0x0,  // Char 'b'
0x38, 0x44, 0x82, 0x82, 0x82, 0x82, 0x44, 0x0,  // Char 'c'
0xFE, 0x82, 0x82, 0x82, 0x82, 0x44, 0x38, 0x0,  // Char 'd'
0xFE, 0x92, 0x92, 0x92, 0x92, 0x82, 0x82, 0x0,  // Char 'e'
0xFE, 0x12, 0x12, 0x12, 0x12, 0x2, 0x2, 0x0,  // Char 'f'
0x7C, 0x82, 0x82, 0x82, 0x92, 0x92, 0x64, 0x0,  // Char 'g'
0xFE, 0x10, 0x10, 0x10, 0x10, 0x10, 0xFE, 0x0,  // Char 'h'
0x0, 0x0, 0x82, 0xFE, 0x82, 0x0, 0x0, 0x0,  // Char 'i'
0x60, 0x80, 0x80, 0x80, 0x82, 0x7E, 0x2, 0x0,  // Char 'j'
0xFE, 0x20, 0x10, 0x28, 0x44, 0x82, 0x0, 0x0,  // Char 'k'
0xFE, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x0,  // Char 'l'
0xFE, 0x4, 0x8, 0x10, 0x8, 0x4, 0xFE, 0x0,  // Char 'm'
0xFE, 0x4, 0x8, 0x10, 0x20, 0x40, 0xFE, 0x0,  // Char 'n'
0x7C, 0x82, 0x82, 0x82, 0x82, 0x82, 0x7C, 0x0,  // Char 'o'
0xFE, 0x12, 0x12, 0x12, 0x12, 0x12, 0xC, 0x0,  // Char 'p'
0xFC, 0x42, 0xA2, 0x92, 0x82, 0x82, 0x7C, 0x0,  // Char 'q'
0xFE, 0x12, 0x12, 0x32, 0x52, 0x92, 0xC, 0x0,  // Char 'r'
0x4C, 0x92, 0x92, 0x92, 0x92, 0x92, 0x64, 0x0,  // Char 's'
0x2, 0x2, 0x2, 0xFE, 0x2, 0x2, 0x2, 0x0,  // Char 't'
0x7E, 0x80, 0x80, 0x80, 0x80, 0x80, 0x7E, 0x0,  // Char 'u'
0x6, 0x18, 0x60, 0x80, 0x60, 0x18, 0x6, 0x0,  // Char 'v'
0x7E, 0x80, 0x80, 0x70, 0x80, 0x80, 0x7E, 0x0,  // Char 'w'
0x82, 0x44, 0x28, 0x10, 0x28, 0x44, 0x82, 0x0,  // Char 'x'
0x2, 0x4, 0x8, 0xF0, 0x8, 0x4, 0x2, 0x0,  // Char 'y'
0x82, 0xC2, 0xA2, 0x92, 0x8A, 0x86, 0x82, 0x20,  // Char 'z'
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0, // NULL
0, 0, 0, 0, 0, 0, 0, 0 // NULL
};

void displayRefreshRow()
{
  static bool initialized = false;
  static uint8_t row;

  if(displayDevice.hasFailed()) {
    initialized = false;
    return;
  }
  
  if(!initialized) {
    SSD1306_command(SSD1306_DISPLAYOFF);          // 0xAE
    SSD1306_command(SSD1306_SETDISPLAYCLOCKDIV);  // 0xD5
    SSD1306_command(0x80);                        // the suggested ratio 0x80
    SSD1306_command(SSD1306_SETMULTIPLEX);        // 0xA8
    SSD1306_command(SSD1306_LCDHEIGHT - 1);
    SSD1306_command(SSD1306_SETDISPLAYOFFSET);    // 0xD3
    SSD1306_command(0x0);                         // no offset
    SSD1306_command(SSD1306_SETSTARTLINE | 0x0);  // line #0
    SSD1306_command(SSD1306_CHARGEPUMP);          // 0x8D
    SSD1306_command(0x14);
    SSD1306_command(SSD1306_MEMORYMODE);          // 0x20
    SSD1306_command(0x00);                        // 0x0 act like ks0108
    SSD1306_command(SSD1306_SEGREMAP | 0x1);
    SSD1306_command(SSD1306_COMSCANDEC);
    SSD1306_command(SSD1306_SETCOMPINS);          // 0xDA
    SSD1306_command(0x12);
    SSD1306_command(SSD1306_SETCONTRAST);         // 0x81
    SSD1306_command(0xCF);
    SSD1306_command(SSD1306_SETPRECHARGE);        // 0xd9
    SSD1306_command(0xF1);
    SSD1306_command(SSD1306_SETVCOMDETECT);       // 0xDB
    SSD1306_command(0x40);
    SSD1306_command(SSD1306_DISPLAYALLON_RESUME); // 0xA4
    SSD1306_command(SSD1306_NORMALDISPLAY);       // 0xA6
    SSD1306_command(SSD1306_DEACTIVATE_SCROLL);
    
    SSD1306_command(SSD1306_DISPLAYON);
    
    for(int i = 0; i < 8; i++) {
      modifiedLeft[i] = 0;
      modifiedRight[i] = 15;
    }

    initialized = true;
  }

  while(row < 8) {    
    if(modifiedLeft[row] > -1) {
      SSD1306_command(SSD1306_PAGEADDR);
      SSD1306_command(row);
      SSD1306_command(row);
      
      SSD1306_command(SSD1306_COLUMNADDR);
      SSD1306_command(modifiedLeft[row]*8);
      SSD1306_command((uint8_t) ~0U);

      for(int col = modifiedLeft[row]; col < modifiedRight[row]+1; col++) {
	uint8_t buffer[8], chr = displayBuffer[row*16+col];

	memcpy_P(buffer, &fontData[(chr & 0x7F)*8], sizeof(buffer));

	if(chr & 0x80) {
	  for(uint8_t i = 0; i < sizeof(buffer); i++)
	    buffer[i] = ~buffer[i];
	}

	SSD1306_data(buffer, sizeof(buffer));
      }

      modifiedLeft[row++] = -1;
      
      return; // We refresh no more than one row at a time
    }

    row++;
  }

  row = 0;
}

void displayClear()
{
  for(int i = 0; i < 8; i++) {
    cursorMove(0, i);
    printNL();
  }
}

void displayRefreshTask()
{
  displayRefreshRow();
}

void tocReportDisplay(bool result, int i, const char *s)
{
  cursorMove((i % 3)*5, i/3 + 2);
  
  if(!result) {
    setAttr(true);
    print(s);
    tocStatusFailed = true;
  } else
    print(NULL, 5);
}

void displayTask()
{
  static bool cleared = false;
  static int count = 0;
    
  count++;
  
  if(vpStatus.silent) {
    if(!cleared) {
      displayClear();
      cleared = true;
    }
    
    return;    
  } else
    cleared = false;

  // Model name
  
  cursorMove(0, 0);
  print(vpParam.name);
  printNL();

  // Status
  
  if(!vpStatus.armed) {
    cursorMove(16-8, 0);
    setAttr(true);
    print("DISARMED");
    return;
  } else if(vpMode.takeOff) {
    cursorMove(16-7, 0);
    setAttr((count>>2) & 1);
    print("TAKEOFF");
    setAttr(0);
  }

  // T/O/C test status

  tocTestStatus(tocReportDisplay);

  cursorMove(0,7);
  print("T/O/C ");

  if(tocStatusFailed) {
    setAttr((count>>2) & 1);
    print("WARNING");
  } else {
    print("GOOD   ");
  }
}

void airspeedTask()
{
  int16_t raw = 0;
  static int16_t prev = 0;
  
  if(!pitotDevice.hasFailed()
     && !pitotDevice.handleStatus(!MS4525DO_pressure(&raw))) {
    pressureBuffer.input((float) raw);
    iasEntropy += ABS(raw - prev);
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
    tuningKnob = inputValue(&tuningKnobInput)*1.05 - 0.05;
    
  if(inputValid(&throttleInput))
    throttleStick = inputValue(&throttleInput);

  modeSelectorValue = readSwitch(&modeSelector);
  
  float buttonValue = inputValue(&buttonInput);
  
  LEVELBUTTON.input(buttonValue);
  FLAPBUTTON.input(buttonValue);
  TRIMBUTTON.input(buttonValue);
  GEARBUTTON.input(buttonValue);

  //
  // Receiver fail detection
  //
  
  if(LEVELBUTTON.state()
     && throttleStick < 0.1 && aileStick < -0.90 && elevStick > 0.90) {
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
  
  if(vpMode.rxFailSafe && vpStatus.armed) {
    vpFeature.stabilizePitch = vpFeature.stabilizeBank
      = vpFeature.alphaHold = vpMode.bankLimiter = true;
    vpFeature.pusher = false;

    trimRateLimiter.setRate(1.5/RADIAN);
    elevTrim = elevFromAlpha(vpDerived.thresholdAlpha);
  } else
    trimRateLimiter.setRate(CIRCLE);
      
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

const float simulatedAttitudeErr_c = 0.7/RADIAN;

void sensorTaskFast()
{
  // Alpha input
  
  alpha = alphaFilter.output();
  
  // Dynamic pressure, corrected for alpha (assuming zero setting angle)
  
  const float pascalsPerPSI_c = 6894.7573, range_c = 2*1.1;
  const float factor_c = pascalsPerPSI_c * range_c / (1L<<(8*sizeof(uint16_t)));
    
  dynPressure = pressureBuffer.output() * factor_c
    / cos(clamp(alpha, -vpParam.alphaMax, vpParam.alphaMax));
  
  // Attitude

  ins.wait_for_sample();
  
  ahrs.update();

  bankAngle = ahrs.roll;
  pitchAngle = ahrs.pitch;
  heading = (360 + (int) (ahrs.yaw*RADIAN)) % 360;
  
  // Angular velocities
  
  Vector3f gyro = ins.get_gyro();
  
  rollRate = gyro.x;
  pitchRate = gyro.y;
  yawRate = gyro.z;

  // Acceleration
  
  Vector3f acc = ins.get_accel(0);

  accX = acc.x;
  accY = acc.y;
  accZ = -acc.z;

  ball.input(accY);
  
  // Altitude data acquisition

  barometer.update();
  barometer.accumulate();

  // Compass

#ifdef USE_COMPASS
  compass.accumulate();
#endif
  
  // Simulator link overrides
  
  if(vpStatus.simulatorLink) {
    alpha = sensorData.alpha/RADIAN + simulatedAttitudeErr_c/2;
    iAS = sensorData.ias*1852/60/60;
    rollRate = sensorData.rrate;
    pitchRate = sensorData.prate;
    yawRate = sensorData.yrate;
    bankAngle = sensorData.roll/RADIAN + simulatedAttitudeErr_c;
    pitchAngle = sensorData.pitch/RADIAN + simulatedAttitudeErr_c;
    heading = (int) (sensorData.heading + 0.5);
    accX = sensorData.accx*FOOT;
    accY = sensorData.accy*FOOT;
    accZ = -sensorData.accz*FOOT;

    dynPressure = airDensity_c * square(iAS) / 2;
    
  } else if(dynPressure > 0)
    iAS = sqrtf(2 * dynPressure / airDensity_c);
  else
    iAS = 0;

  //
  // Derived values
  //
    
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

  // Compass

#ifdef USE_COMPASS
  compass.read();
#endif
  
  // Weight on wheels switch not available for now

  vpStatus.weightOnWheels = (gearOutput == 0);
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

  if(iAS < vpDerived.stallIAS/3 || fabsf(iAS - iasFilterSlow.output()) > 0.5) {
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

void fastLogTask()
{
  //  logAlpha();  
}

void slowLogTask()
{
  logAlpha();  
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

  if(!beepDuration)
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

  // PPM monitoring

  if(ppmWarnSlow || ppmWarnShort) {
    lastPPMWarn = currentTime;
    ppmWarnSlow = ppmWarnShort = false;
  }
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
      finalKxIAS = finalK/powf(finalIAS, (analyzerInputCh == ac_aile ? stabilityAileExp1_c : stabilityElevExp_c));
      
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
const float testRange_c = 4;

static float testGainExpoGeneric(float range, float param)
{
  static float state;
  return exp(log(testRange_c)*(1.3*quantize(param, &state, paramSteps)-0.3))*range;
}

float testGainExpo(float range)
{
  return testGainExpoGeneric(range, tuningKnob);
}

float testGainExpoReversed(float range)
{
  return testGainExpoGeneric(range, 1 - tuningKnob);
}

float testGainLinear(float start, float stop)
{
  static float state;
  float q = quantize(tuningKnob, &state, paramSteps);
  return start + q*(stop - start);
}

float s_Ku_ref, i_Ku_ref, p_Ku_ref;

const float minAlpha = -2.0/RADIAN;
const float origoAlpha = -5.0/RADIAN;

static float scaleByIAS(float k, float p)
{
  float effIAS = fmaxf(iasFilter.output(), vpDerived.stallIAS);
  
  if(pitotFailed() || vpStatus.pitotBlocked)
    // Failsafe value chosen to be ... on the safe side
    effIAS = p > 0 ? vpDerived.stallIAS : vpDerived.stallIAS * 3/2;
  
  return k * powf(effIAS, p);
}

static void failsafeDisable()
{
  if(vpMode.alphaFailSafe || vpMode.sensorFailSafe) {
    consoleNoteLn_P(PSTR("Alpha/Sensor failsafe DISABLED"));
    vpMode.alphaFailSafe = vpMode.sensorFailSafe = false;
  }
}

void statusTask()
{
  if(!vpStatus.armed)
    return;
  
  //
  // Do we have positive airspeed?
  //

  static uint32_t lastNegativeIAS, lastStall;

  if(vpStatus.pitotBlocked || iasFilter.output() < vpDerived.stallIAS/2) {
    if(vpStatus.positiveIAS) {
      consoleNoteLn_P(PSTR("Positive airspeed LOST"));
      vpStatus.positiveIAS = false;
    }
    
    lastNegativeIAS = currentTime;

  } else if(currentTime - lastNegativeIAS > 1e6 && !vpStatus.positiveIAS) {
    consoleNoteLn_P(PSTR("We have POSITIVE AIRSPEED"));
    vpStatus.positiveIAS = true;
  }

  //
  // Movement detection
  //
  
  accTotal = sqrtf(square(accX) + square(accY) + square(accZ));
  
  accAvg.input(accTotal);

  float turnRate = sqrt(square(rollRate) + square(pitchRate) + square(yawRate));
  
  bool motionDetected = vpStatus.positiveIAS || turnRate > 10.0/RADIAN
    || fabsf(accTotal - accAvg.output()) > 0.3;
  
  static uint32_t lastMotion;

  if(motionDetected) {
    if(vpStatus.fullStop) {
      consoleNoteLn_P(PSTR("We appear to be MOVING"));
      vpStatus.fullStop = false;
    }
    
    lastMotion = currentTime;

  } else if(currentTime - lastMotion > 5.0e6 && !vpStatus.fullStop) {
    consoleNoteLn_P(PSTR("We have FULLY STOPPED"));
    vpStatus.fullStop = true;
  }

  //
  // Stall detection
  //
  
  if(vpMode.alphaFailSafe || vpMode.sensorFailSafe || vpMode.takeOff
     || alpha < stallAlpha*1.05) {
    if(!vpStatus.stall)
      lastStall = currentTime;
    else if(currentTime - lastStall > 0.5e6) {
      consoleNoteLn_P(PSTR("We've RECOVERED"));
      vpStatus.stall = false;
    }
  } else {
    if(vpStatus.stall)
      lastStall = currentTime;
    else if(currentTime - lastStall > 0.5e6) {
      consoleNoteLn_P(PSTR("We're STALLING"));
      vpStatus.stall = true;
    }
  }
}
  
void configurationTask()
{
  //
  // Being armed?
  //
  
  if(leftUpButton.doublePulse() && !vpStatus.armed
     && aileStick < -0.90 && elevStick > 0.90) {
    consoleNoteLn_P(PSTR("We're now ARMED"));
    vpStatus.armed = true;
    badBeep(1);
    leftDownButton.reset();
    rightUpButton.reset();
    rightDownButton.reset();
    tocTestReset();
  }

  // We skip the rest unless we're armed

  if(!vpStatus.armed)
    return;
  
  //
  // T/O config test
  //

  if(!vpStatus.silent)
    tocTestUpdate();

  //
  //   GEAR BUTTON
  //
  
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
  // FLAP BUTTON
  //

  if(FLAPBUTTON.singlePulse() && flapOutput > 0) {
    //
    // SINGLE PULSE: FLAPS UP one step
    //
    
    consoleNote_P(PSTR("Flaps RETRACTED to "));
    consolePrintLn(--flapOutput);

  } else if(FLAPBUTTON.depressed() && flapOutput < 3) {
    //
    // CONTINUOUS: FLAPS DOWN one step
    //
    
    consoleNote_P(PSTR("Flaps EXTENDED to "));
    consolePrintLn(++flapOutput);
  }

  //
  // WING LEVELER BUTTON
  //

  if(LEVELBUTTON.singlePulse()) {
    //
    // PULSE : Takeoff mode enable
    //
  
    if(vpMode.alphaFailSafe || vpMode.sensorFailSafe) {
      failsafeDisable();
      
    } else if(!vpStatus.positiveIAS) {
	    
      vpStatus.silent = false;

      bool prevMode = vpMode.takeOff;
      
      if(!vpMode.takeOff) {
	consoleNoteLn_P(PSTR("TakeOff mode ENABLED"));
	vpMode.takeOff = true;
      }
	
      if(tocTestStatus(tocReportConsole)) {
	consoleNoteLn_P(PSTR("T/o configuration is GOOD"));
	goodBeep(1);
      } else {
	consolePrintLn("");
	consoleNoteLn_P(PSTR("T/o configuration test FAILED"));
	vpMode.takeOff = prevMode;
	badBeep(2);
      }
    }
  } else if(LEVELBUTTON.depressed()) {
    //
    // CONTINUOUS : LEVEL WINGS
    //
  
    failsafeDisable();
    
    if(!vpMode.wingLeveler) {
      consoleNoteLn_P(PSTR("Wing leveler ENABLED"));
      vpMode.wingLeveler = true;
    } 
  }

  //
  // Logging control
  //

  if(vpStatus.fullStop && logLen > logSize*0.90)
    logDisable();

  //
  // Direct mode selector input
  //

  if(modeSelectorValue == -1) {
    if(!vpMode.slowFlight)
      consoleNoteLn_P(PSTR("Slow flight mode ENABLED"));
    vpMode.slowFlight = vpMode.bankLimiter = true;
  } else if(vpMode.slowFlight) {
    consoleNoteLn_P(PSTR("Slow flight mode DISABLED"));
    vpMode.slowFlight = false;
  }

  if(modeSelectorValue == 0) {
    if(vpMode.bankLimiter)
      consoleNoteLn_P(PSTR("Bank limiter DISABLED"));
    
    vpMode.bankLimiter = false;
    
  } else if(!vpMode.bankLimiter) {
    consoleNoteLn_P(PSTR("Bank limiter ENABLED"));
    vpMode.bankLimiter = true;
  }

  //
  // Test mode control
  //

  if(!vpMode.test && tuningKnob > 0.5) {
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
  } else if(vpMode.test && tuningKnob < 0) {
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

  if(vpMode.takeOff
     && (pitotFailed() || iasFilter.output() > 2*vpDerived.stallIAS/3)) {
    consoleNoteLn_P(PSTR("TakeOff COMPLETED"));
    vpMode.takeOff = false;
    
    if(!vpStatus.consoleLink)
      vpStatus.silent = true;
    
    if(!vpMode.loggingSuppressed)
      logEnable();
  }

  //
  // Map mode to features (unless we're in receiver failsafe mode)
  //

  if(!vpMode.rxFailSafe) {
    // Default

    vpFeature.stabilizeBank
      = !(vpStatus.stall || vpStatus.weightOnWheels || vpMode.takeOff);
    vpFeature.keepLevel
      = !vpStatus.stall
      && (vpMode.wingLeveler || vpMode.takeOff || vpStatus.weightOnWheels);
    vpFeature.pusher
      = !alphaFailed() && !vpMode.takeOff && !vpMode.slowFlight;
    vpFeature.stabilizePitch = vpFeature.alphaHold
      = !alphaFailed() && vpMode.slowFlight && !vpMode.takeOff;
    vpFeature.pitchHold = false;

    // Slow flight implies bank limiter

    if(vpMode.slowFlight)
      vpMode.bankLimiter = true;
  
    // Failsafe mode interpretation

    if(vpMode.sensorFailSafe) {
      vpFeature.stabilizePitch = vpFeature.stabilizeBank
	= vpFeature.pitchHold = vpFeature.alphaHold = vpFeature.pusher
	= vpMode.bankLimiter = vpFeature.keepLevel = vpMode.takeOff
	= vpMode.slowFlight = false;

    } else if(vpMode.alphaFailSafe)
      vpFeature.stabilizePitch = vpFeature.pitchHold = vpFeature.alphaHold
	= vpFeature.pusher = vpMode.takeOff = vpMode.slowFlight = false;
  }
  
  // Safety scaling (test mode 0)
  
  float scale = 1.0;
  
  if(vpMode.test && nvState.testNum == 0)
    scale = testGainLinear(1.0/3, 1.5);
  
  // Default controller settings

  float s_Ku = scaleByIAS(vpParam.s_Ku_C, stabilityAileExp1_c);
  float i_Ku = scaleByIAS(vpParam.i_Ku_C, stabilityElevExp_c);
  float p_Ku = scaleByIAS(vpParam.p_Ku_C, stabilityPusherExp_c);
  
  aileCtrl.setZieglerNicholsPID(s_Ku*scale, vpParam.s_Tu);
  elevCtrl.setZieglerNicholsPID(i_Ku*scale, vpParam.i_Tu);
  pushCtrl.setZieglerNicholsPID(p_Ku*scale, vpParam.p_Tu);

  autoAlphaP = vpParam.o_P;
  stallAlpha = vpParam.alphaMax;
  shakerAlpha = vpDerived.shakerAlpha;
  pusherAlpha = vpDerived.pusherAlpha;
  rudderMix = vpParam.r_Mix;
  levelBank = 0;
  
  aileRateLimiter.setRate(vpParam.servoRate/(90.0/2)/vpParam.aileDefl);

  //
  // Apply test mode
  //
  
  if(vpMode.test && !vpMode.takeOff) {
    switch(nvState.testNum) {
    case 1:
      // Wing stabilizer gain
         
      vpFeature.stabilizeBank = vpMode.bankLimiter = vpFeature.keepLevel = true;
      aileCtrl.setPID(testGain = testGainExpo(s_Ku_ref), 0, 0);
      break;
            
    case 21:
      // Wing stabilizer gain autotest

      analyzerInputCh = ac_aile;
      vpFeature.stabilizeBank = vpMode.bankLimiter = vpFeature.keepLevel = true;
	
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
      pusherAlpha = shakerAlpha = 0.8*vpParam.alphaMax;
      break;

    case 6:
      // Pusher used as alpha hold

      vpFeature.alphaHold = true;
      vpFeature.stabilizePitch = false;
      pushCtrl.setPID(testGain = testGainExpo(p_Ku_ref), 0, 0);
      break;

    case 8:
      // Stall behavior test
      
      vpFeature.pusher = false;
      break;
      
    case 9:
      // Max alpha

      vpFeature.stabilizeBank = vpMode.bankLimiter = vpFeature.keepLevel = true;
      shakerAlpha = pusherAlpha = stallAlpha = testGain
	= testGainLinear(vpParam.alphaMax+10/RADIAN, vpParam.alphaMax);
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
}

void gaugeTask()
{
  if(gaugeCount > 0) {
    uint16_t tmp = 0;
	
    for(int g = 0; g < gaugeCount; g++) {
      switch(gaugeVariable[g]) {
      case 1:
	consolePrint_P(PSTR(" alpha = "));
	consolePrint(alpha*RADIAN, 1);
	consoleTab(15);
	consolePrint_P(PSTR(" IAS,K(m/s) = "));
	consolePrint((int) (iAS/KNOT));
	consolePrint_P(PSTR(" ("));
	consolePrint(iAS, 1);
	consolePrint_P(PSTR(")"));
	consoleTab(40);
	consolePrint_P(PSTR(" hdg = "));
	consolePrint(heading);
	consoleTab(60);
	consolePrint_P(PSTR(" alt = "));

	tmp = altitude/FOOT;
	
	if(tmp < 100)
	  consolePrint(tmp);
	else
	  consolePrint(((tmp+5)/10)*10);
	
	break;

      case 2:
	consolePrint_P(PSTR(" alpha(target) = "));
	consolePrint(alpha*RADIAN);
	consolePrint_P(PSTR(" ("));
	consolePrint(targetAlpha*RADIAN);
	consolePrint_P(PSTR(")"));
	consoleTab(25);
	consolePrint_P(PSTR(" pitchRate(target) = "));
	consolePrint(pitchRate*RADIAN, 1);
	consolePrint_P(PSTR(" ("));
	consolePrint(targetPitchRate*RADIAN);
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
	consolePrint(bankAngle*RADIAN, 2);
	consolePrint_P(PSTR(" pitch = "));
	consolePrint(pitchAngle*RADIAN, 2);
	consolePrint_P(PSTR(" heading = "));
	consolePrint(heading);
	consolePrint_P(PSTR(" alt = "));
	consolePrint(altitude);
	consolePrint_P(PSTR(" ball = "));
	consolePrint(ball.output(), 2);
	break;

      case 5:
	consolePrint_P(PSTR(" rollRate = "));
	consolePrint(rollRate*RADIAN, 1);
	consolePrint_P(PSTR(" pitchRate = "));
	consolePrint(pitchRate*RADIAN, 1);
	consolePrint_P(PSTR(" yawRate = "));
	consolePrint(yawRate*RADIAN, 1);
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
	consolePrint_P(PSTR(" thrStick = "));
	consolePrint(throttleStick);
	consolePrint_P(PSTR(" rudderStick = "));
	consolePrint(rudderStick);
	consolePrint_P(PSTR(" knob = "));
	consolePrint(tuningKnob);
	break;

      case 8:
	consolePrint_P(PSTR(" aileOut = "));
	consolePrint(aileOutput);
	consolePrint_P(PSTR(" elevOut = "));
	consolePrint(elevOutput);
	consolePrint_P(PSTR(" rudderOut = "));
	consolePrint(rudderOutput);
	break;

      case 9:
	consolePrint_P(PSTR(" gain*IAS^("));
	for(float j = 0; j < 2; j += 0.5) {
	  if(j > 0)
	    consolePrint(", ");
	  consolePrint(j);
	}
	
	consolePrint_P(PSTR(") = "));
	
	for(float j = 0; j < 2; j += 0.5) {
	  if(j > 0)
	    consolePrint(", ");
	  consolePrint(testGain*powf(iAS, j));
	}
	break;
	
      case 10:
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

      case 11:
	consolePrint_P(PSTR(" alpha = "));
	consolePrint(alpha*RADIAN, 1);
	consoleTab(15);
	consolePrint_P(PSTR(" CoL(rel) = "));
	consolePrint(2*cos(alpha)*accZ/square(iAS), 3);
	break;
	
      case 12:
	consoleNote_P(PSTR(" entropy(alpha,ias) = "));
	consolePrint(alphaEntropyAcc.output());
	consolePrint_P(PSTR(", "));
	consolePrint(iasEntropyAcc.output());
	consolePrint_P(PSTR(" hash = "));
	
	tmp = sensorHash;

	for(int i = 0; i < 16; i++) {
	  consolePrint((tmp & 1) ? "+" : " ");
	  tmp = tmp >> 1;
	}

	consolePrintLn("");
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

float nominalPitchRate(float bank, float target)
{
  return square(sin(bank))*coeffOfLift(target)*iasFilter.output()/2;
}

void controlTask()
{
  // Cycle time bookkeeping 
  
  if(beepDuration > 0) {
    // We're beeping, fuggetaboutit
    controlCycleEnded = 0;
  } else if(controlCycleEnded > 0) {
    controlCycle = (currentTime - controlCycleEnded)/1.0e6;
    cycleTimeMonitor(controlCycle);
  }
  
  controlCycleEnded = currentTime;

  //
  // Elevator control
  //

  const float shakerLimit = (float) 2/3;
  const float effStick = vpMode.rxFailSafe ? shakerLimit : elevStick;
  const float stickStrength = fmaxf(effStick-shakerLimit, 0)/(1-shakerLimit);
  const float effMaxAlpha = mixValue(stickStrength, shakerAlpha, pusherAlpha);
    
  elevOutput = effStick + elevTrim;
  
  targetAlpha = trimRateLimiter.input
    (clamp(alphaFromElev(elevOutput), -vpParam.alphaMax, effMaxAlpha),
     controlCycle);

  if(vpFeature.alphaHold)
    targetPitchRate = nominalPitchRate(bankAngle, targetAlpha)
      + clamp(targetAlpha - alpha,
	      -30/RADIAN - pitchAngle,
	      vpParam.maxPitch - pitchAngle)*autoAlphaP;

  else if(vpFeature.pitchHold)
    targetPitchRate = (0.1 + effStick/2 - pitchAngle)*autoAlphaP;

  else
    targetPitchRate = effStick*PI/2;

  elevOutputFeedForward = elevFromAlpha(targetAlpha);

  if(vpFeature.stabilizePitch) {
    elevCtrl.input(targetPitchRate - pitchRate, controlCycle);
    
    elevOutput = elevCtrl.output();

    if(vpFeature.alphaHold)
      elevOutput += elevOutputFeedForward;
  } else
    elevCtrl.reset(elevOutput - elevOutputFeedForward, 0.0);

  // Pusher

  if(vpFeature.alphaHold && !vpFeature.stabilizePitch) {
    pushCtrl.input(targetAlpha - alpha, controlCycle);
    elevOutput = elevOutputFeedForward + pushCtrl.output();
  } else if(vpFeature.pusher) {
    pushCtrl.input(effMaxAlpha - alpha, controlCycle);
    elevOutput = fminf(elevOutput,
		       elevFromAlpha(effMaxAlpha) + pushCtrl.output());
  } else
    pushCtrl.reset(elevOutput - elevFromAlpha(effMaxAlpha),
		   effMaxAlpha - alpha);
  
  elevOutput = clamp(elevOutput, -1, 1);

  //
  // Aileron
  //
  
  aileOutput = aileStick;

  const float rollP = 2.5;
  const float maxRollRate = scaleByIAS(vpParam.roll_C, stabilityAileExp2_c);
  float maxBank = 45/RADIAN;

  if(vpMode.rxFailSafe)
    maxBank = 10/RADIAN;
  else if(vpFeature.alphaHold)
    maxBank /= 1 + alphaFromElev(elevTrim) / vpDerived.thresholdAlpha / 2;
  
  float targetRollRate = maxRollRate*aileStick;

  if(!vpFeature.stabilizeBank) {

    if(vpFeature.keepLevel)
      // Simple leveling for situations where we want to avoid I term wind-up
      
      aileOutput = clamp(aileStick - bankAngle - rollRate/32, -1, 1);
    
    aileCtrl.reset(aileOutput, 0);
    
  } else {
    
    if(vpFeature.keepLevel)
      // Strong leveler enabled
        
      targetRollRate =
	clamp((levelBank + aileStick*maxBank - bankAngle)*rollP,
	      -maxRollRate, maxRollRate);

    else if(vpMode.bankLimiter) {
      // Bank limiter + weak leveling

      targetRollRate -= rollP*clamp(bankAngle, -0.5/RADIAN, 0.5/RADIAN);
      
      targetRollRate =
	clamp(targetRollRate, (-maxBank - bankAngle)*rollP, (maxBank - bankAngle)*rollP);
    }
      
    aileCtrl.input(targetRollRate - rollRate, controlCycle);
    aileOutput = aileCtrl.output();
  }

  aileRateLimiter.input(clamp(aileOutput, -1, 1), controlCycle);
    
  // Rudder
    
  steerOutput = rudderStick;
  
  rudderOutput = rudderStick + aileRateLimiter.output()*rudderMix;

  // Flaps
  
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

  if(vpParam.elevon) {
    pwmOutputWrite(aileHandle, NEUTRAL
		   + RANGE*clamp(vpParam.aileDefl*aileRateLimiter.output()
				 - vpParam.elevDefl*elevOutput
				 + vpParam.aileNeutral, -1, 1));

    pwmOutputWrite(elevatorHandle, NEUTRAL
		   + RANGE*clamp(vpParam.aileDefl*aileRateLimiter.output()
				 + vpParam.elevDefl*elevOutput 
				 + vpParam.elevNeutral, -1, 1));
  } else {
    pwmOutputWrite(aileHandle, NEUTRAL
		   + RANGE*clamp(vpParam.aileDefl*aileRateLimiter.output()
				 + vpParam.aileNeutral, -1, 1));

    pwmOutputWrite(elevatorHandle, NEUTRAL
		   + RANGE*clamp(vpParam.elevDefl*elevOutput 
				 + vpParam.elevNeutral, -1, 1));
  }
  
  pwmOutputWrite(rudderHandle, NEUTRAL
		 + RANGE*clamp(vpParam.rudderNeutral + 
			       vpParam.rudderDefl*rudderOutput, -1, 1));                        
  pwmOutputWrite(steerHandle, NEUTRAL
		 + RANGE*clamp(vpParam.steerNeutral + 
			       vpParam.steerDefl*steerOutput, -1, 1));                        
  pwmOutputWrite(flapHandle, NEUTRAL
		 + RANGE*clamp(vpParam.flapNeutral 
			       + vpParam.flapStep*flapRateLimiter.output(), -1, 1));                              
  pwmOutputWrite(flap2Handle, NEUTRAL
		 + RANGE*clamp(vpParam.flap2Neutral 
			       - vpParam.flapStep*flapRateLimiter.output(), -1, 1));                              

  pwmOutputWrite(gearHandle, NEUTRAL - RANGE*(gearOutput*2-1)*0.6);

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

  //
  // Adjust for alpha-elev predictor error when moving in/out of slow flight
  //
  
  static bool prevMode;

  if(prevMode != vpFeature.alphaHold && vpStatus.positiveIAS) {

    const float predictError =
      clamp(elevFromAlpha(alpha) - elevOutput, -0.2, 0.2);
      
    if(vpFeature.alphaHold)
      elevTrim += predictError;
    else
      elevTrim -= predictError;
  }

  prevMode = vpFeature.alphaHold;

  //
  // Trim limits
  //
  
  if(vpMode.takeOff) {
    // Takeoff mode enabled, trim is fixed
    
    if(vpMode.slowFlight)
      elevTrim = elevFromAlpha(vpDerived.thresholdAlpha);
    else
      elevTrim = vpParam.takeoffTrim;
      
  } else
    elevTrim = clamp(elevTrim, 0, elevFromAlpha(vpDerived.thresholdAlpha));
}

void pingTestTask()
{
  if(pingTestRxCount < 1)
    // We're done testing
    return;

  else if(pingTestTxCount < pingTestRxCount) {
    // Waiting for the ping back
    
    if(currentTime > pingTestTxTime + 1e6) {
      consoleNoteLn_P(PSTR("Ping reception TIMED OUT"));
      pingTestTxCount = pingTestRxCount = 0;
    }
  } else {
    // Transmit a new packet

    switch(pingTestTxCount % 4) {
    case 0:
      pingTestData = 0UL;
      break;
    case 1:
      pingTestData = ~0UL;
      break;
    case 2:
      pingTestData = randomUInt32();
      break;
    case 3:
      pingTestData = currentTime;
      break;
    }

    datagramTxStart(DG_PING);
    datagramTxOut((const uint8_t*) &pingTestData, sizeof(pingTestData));
    datagramTxEnd();

    pingTestTxTime = currentTime;
    pingTestTxCount--;
  } 
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
	beepPrim(phase < 1 ? 950 : 750, 1e3/BEEP_HZ);
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
  fastLogTask();
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
  { displayRefreshTask,
    HZ_TO_PERIOD(20) },
  { displayTask,
    HZ_TO_PERIOD(8) },
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
  { statusTask,
    HZ_TO_PERIOD(CONFIG_HZ) },
  { fastLogTask,
    HZ_TO_PERIOD(LOG_HZ_FAST) },
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
  { pingTestTask,
    HZ_TO_PERIOD(30) },
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

void setup()
{
  // PWM output

  // consoleNoteLn_P(PSTR("Initializing PWM output"));

  pwmTimerInit(hwTimers, sizeof(hwTimers)/sizeof(struct HWTimer*));
  pwmOutputInitList(pwmOutput, sizeof(pwmOutput)/sizeof(struct PWMOutput));

  // HAL

  hal.init(0, NULL);
  
  // initialise serial port
  
  cliSerial = hal.console;

  vpStatus.consoleLink = true;
  
  consoleNoteLn_P(PSTR("Project | Alpha"));   

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
  
  setModel(nvState.model, true);

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

#ifdef USE_COMPASS
  consoleNote_P(PSTR("Initializing compass... "));
  consoleFlush();

  if(compass.init()) {
    consolePrint_P(PSTR("  done, "));
    consolePrint(compass.get_count());
    consolePrintLn_P(PSTR(" sensor(s) detected."));
  
    ahrs.set_compass(&compass);
  
    compass.set_and_save_offsets(0,0,0,0); // set offsets to account for surrounding interference
    compass.set_declination(ToRad(0.0f));
    
  } else {
    consolePrintLn_P(PSTR("  FAILED."));
    consoleFlush();
    while (1) ;
  }
#endif

  // LED output

  configureOutput(&RED_LED);
  configureOutput(&GREEN_LED);
  configureOutput(&BLUE_LED);

  setPinState(&RED_LED, 1);
  setPinState(&GREEN_LED, 1);
  setPinState(&BLUE_LED, 1);

  // Piezo element
  
  setPinState(&PIEZO.pin, 0);
  configureOutput(&PIEZO.pin);

  // Alpha filter (sliding average over alphaWindow_c/seconds)
  
  alphaFilter.setWindow(alphaWindow_c*ALPHA_HZ);

  // Elevator delay

  elevatorDelay.setDelay(1);
  aileronDelay.setDelay(1);

  // Static controller settings
  
  pushCtrl.limit(-0.5, fmaxf(1 - elevFromAlpha(vpDerived.pusherAlpha), 0.0));
  flapRateLimiter.setRate(0.5);
  
  // Misc filters

  accAvg.reset(G);

  // Cycle time monitor
  
  cycleTimeMonitorReset();

  // Initial gear state
  
  gearOutput = 0;

  // Done
  
  consoleNote_P(PSTR("Initialized, "));
  consolePrint((unsigned long) hal.util->available_memory());
  consolePrintLn_P(PSTR(" bytes free."));
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
