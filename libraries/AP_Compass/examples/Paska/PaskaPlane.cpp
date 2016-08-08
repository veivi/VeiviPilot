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

//
//
//

const float stabilityElevExp1_c = -1.5;
const float stabilityElevExp2_c = 0.0;
const float stabilityAileExp1_c = -1.5;
const float stabilityAileExp2_c = 0.5;

const float G = 9.81, RAD = 360/2/PI;

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
  modeSwitchInput, tuningKnobInput, gearInput, flapInput;
struct RxInputRecord *ppmInputs[] = 
  { &aileInput, &elevInput, &throttleInput, &rudderInput, &modeSwitchInput, &tuningKnobInput, &gearInput, &flapInput };

ButtonInputChannel buttonInput;
Button rightDownButton(-0.60), rightUpButton(0.26),
  leftDownButton(-0.13), leftUpButton(0.66);

#define aileModeButton rightUpButton
#define elevModeButton rightDownButton
#define trimButton leftUpButton
#define gearButton leftDownButton

int8_t flapSwitchValue;

//
// Servo PWM output
//

#define NEUTRAL 1500
#define RANGE 500

const struct PWMOutput pwmOutput[] = {
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
#define CONFIG_HZ (CONTROL_HZ/2)
#define ALPHA_HZ (CONTROL_HZ*10)
#define AIRSPEED_HZ (CONTROL_HZ*5)
#define TRIM_HZ 10
#define LED_HZ 3
#define LED_TICK 100
#define LOG_HZ_ALPHA CONTROL_HZ
#define LOG_HZ_CONTROL (CONTROL_HZ/3)
#define LOG_HZ_SLOW 2
  
struct Task {
  void (*code)(uint32_t time);
  uint32_t period, lastExecuted;
};

#define HZ_TO_PERIOD(f) ((uint32_t) (1.0e6/(f)))

struct ModeRecord {
  bool test;
  bool rattle;
  bool loop;
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
struct GPSFix gpsFix;

float testGain = 0;
const int cycleTimeWindow = 31;
float cycleTimeStore[cycleTimeWindow];
int cycleTimePtr = 0;
bool cycleTimesValid;
float cycleMin = -1.0, cycleMax = -1.0, cycleMean = -1.0;
float iAS, dynPressure, alpha, aileStick, elevStick, throttleStick, rudderStick;
bool ailePilotInput, elevPilotInput, rudderPilotInput;
uint32_t controlCycleEnded;
float elevTrim, effTrim, elevTrimSub, targetAlpha;
float tuningKnobValue;
Controller elevCtrl, aileCtrl, pushCtrl, rudderCtrl;
float autoAlphaP, maxAlpha, shakerAlpha, thresholdAlpha, rudderMix;
float accX, accY, accZ, altitude,  heading, rollAngle, pitchAngle, rollRate, pitchRate, targetPitchRate, yawRate, levelBank;
int cycleTimeCounter = 0;
float parameter;  
NewI2C I2c = NewI2C();
Accumulator ball, cycleTimeAcc, iasFilterSlow, iasFilter, accFilter;
AlphaBuffer pressureBuffer;
RunningAvgFilter alphaFilter;
float controlCycle = 10.0e-3;
uint32_t idleMicros;
float idleAvg, logBandWidth, ppmFreq, simInputFreq;
uint32_t simTimeStamp;
RateLimiter aileRateLimiter, flapRateLimiter, alphaRateLimiter;
float elevOutput, elevOutputFeedForward, aileOutput = 0, flapOutput = 0, gearOutput = 0, brakeOutput = 0, rudderOutput = 0;

const int maxTests_c = 32;
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
    logGeneric(lc_test, nvState.testChannel);
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
  logGeneric(lc_roll, rollAngle);
  logGeneric(lc_rollrate, rollRate*360);
  logGeneric(lc_pitch, pitchAngle);
  logGeneric(lc_pitchrate, pitchRate*360);
  logGeneric(lc_heading, heading);
  logGeneric(lc_yawrate, yawRate*360);
}

float readParameter()
{
  return tuningKnobValue/0.95 - (1/0.95 - 1);
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
    vpMode.loop = false;
    calibStop(nvState.rxMin, nvState.rxCenter, nvState.rxMax);
    return;
  }
  
  const int maxParams = 8;

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
      if(numParams > 0) {
	nvState.testChannel = param[0];
	storeNVState();
      }

      consoleNote_P(PSTR("Current test channel = "));
      consolePrintLn(nvState.testChannel);
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
          
    case c_zero:
      vpParam.alphaRef += (int16_t) ((1L<<16) * alpha);
      consoleNoteLn_P(PSTR("Alpha zero set"));
      break;

    case c_alpha:
      if(numParams > 0) {
	vpParam.alphaRef +=
	  (int16_t) ((1L<<16) * (alpha - (float) param[0] / 360));
	consoleNoteLn_P(PSTR("Alpha calibrated"));
      }
      break;

    case c_loop:
      vpMode.loop = true;
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
    case c_dumpz:
      logDumpBinary();
      break;
    
    case c_backup:
      dumpParams();
      break;

    case c_stamp:
      if(numParams > 0) {
	nvState.logStamp = param[0];
	storeNVState();
      }
      consoleNote_P(PSTR("Current log stamp is "));
      consolePrintLn(nvState.logStamp);  
      break;

    case c_model:
      if(numParams > 0) {
	if(param[0] > MAX_MODELS-1)
	  param[0] = MAX_MODELS-1;
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
      cycleMin = cycleMax = cycleMean = -1;
      cycleTimesValid = false;
      cycleTimePtr = 0;
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
      
    case c_cycle:
      cycleTimeCounter = 0;
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
      
      consoleNoteLn_P(PSTR("Cycle time (ms)"));
      consoleNote_P(PSTR("  median     = "));
      consolePrintLn(controlCycle*1e3);
      consoleNote_P(PSTR("  min        = "));
      consolePrintLn(cycleMin*1e3);
      consoleNote_P(PSTR("  max        = "));
      consolePrintLn(cycleMax*1e3);
      consoleNote_P(PSTR("  mean       = "));
      consolePrintLn(cycleMean*1e3);
      consoleNote_P(PSTR("  cum. value = "));
      consolePrintLn(cycleTimeAcc.output());
      consoleNote_P(PSTR("Warning flags :"));
      if(pciWarn)
	consolePrint_P(PSTR(" SPURIOUS_PCINT"));
      if(vpStatus.alphaWarn)
	consolePrint_P(PSTR(" ALPHA_SENSOR"));
      if(ppmWarnShort)
	consolePrint_P(PSTR(" PPM_SHORT"));
      if(ppmWarnSlow)
	consolePrint_P(PSTR(" PPM_SLOW"));
      if(eepromWarn)
	consolePrint_P(PSTR(" EEPROM"));
      if(eepromFailed)
	consolePrint_P(PSTR(" EEPROM_FAILED"));
      if(vpStatus.iasWarn)
	consolePrint_P(PSTR(" IAS_WARN"));
      if(vpStatus.iasFailed)
	consolePrint_P(PSTR(" IAS_FAILED"));
      //      if(alphaBuffer.warn)
      //	consolePrint_P(PSTR(" ALPHA_BUFFER"));
      if(pushCtrl.warn)
	consolePrint_P(PSTR(" PUSHER"));
      if(elevCtrl.warn)
	consolePrint_P(PSTR(" AUTOSTICK"));
      if(aileCtrl.warn)
	consolePrint_P(PSTR(" STABILIZER"));
      
      consolePrintLn("");

      consoleNote_P(PSTR("Log write bandwidth = "));
      consolePrint(logBandWidth);
      consolePrintLn_P(PSTR(" bytes/sec"));
      break;

    case c_reset:
      pciWarn = vpStatus.alphaWarn = vpStatus.alphaFailed = pushCtrl.warn = elevCtrl.warn
	= eepromWarn = eepromFailed = ppmWarnShort
	= ppmWarnSlow = aileCtrl.warn = false;
      consoleNoteLn_P(PSTR("Warning flags reset"));
      break;
    
    default:
      consolePrint_P(PSTR("Sorry, command not implemented: \""));
      consolePrint(buf, tokenLen);
      consolePrintLn("\"");
      break;
    }
  }
}

float scaleByIAS(float k, float p)
{
  return k * powf(fmax(iasFilter.output(), vpParam.iasMin), p);
}

void cacheTask(uint32_t currentMicros)
{
  cacheFlush();
}

void logStartCallback()
{
  logAlpha();
  logAttitude();
  logInput();
  logActuator();
  logConfig();
  logPosition();
}

void logSaveTask(uint32_t currentMicros)
{
  logSave(logStartCallback);
}

void alphaTask(uint32_t currentMicros)
{
  int16_t raw = 0;
  static int failCount = 0;
  
  if(!handleFailure("alpha", !readAlpha_5048B(&raw), &vpStatus.alphaWarn, &vpStatus.alphaFailed, &failCount))
    alphaFilter.input((float) raw / (1L<<(8*sizeof(raw))));
}

void airspeedTask(uint32_t currentMicros)
{
  int16_t raw = 0;
  static int failCount = 0;
  
  if(!handleFailure("airspeed", !readPressure(&raw), &vpStatus.iasWarn, &vpStatus.iasFailed, &failCount))
    pressureBuffer.input((float) raw);
}

DelayLine elevatorDelay;

void receiverTask(uint32_t currentMicros)
{
  if(inputValid(&aileInput))
    aileStick = applyNullZone(inputValue(&aileInput), &ailePilotInput);
  
  if(inputValid(&rudderInput))
    rudderStick = applyNullZone(inputValue(&rudderInput), &rudderPilotInput);
  
  if(inputValid(&elevInput))
    elevStick = applyNullZone(inputValue(&elevInput), &elevPilotInput);
  //    elevStick = inputValue(&elevInput);
    
  if(inputValid(&tuningKnobInput))
    tuningKnobValue = inputValue(&tuningKnobInput);
    
  if(inputValid(&throttleInput))
    throttleStick = inputValue(&throttleInput);

  flapSwitchValue = inputValue(&flapInput);
  
  buttonInput.input(inputValue(&modeSwitchInput));  

  aileModeButton.input(buttonInput.value());
  elevModeButton.input(buttonInput.value());
  trimButton.input(buttonInput.value());
  gearButton.input(buttonInput.value());

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

  // Delay the elevator so we always detect the failsafe mode before
  // doing anything with the raw elevator
  
  elevStick = elevatorDelay.input(elevStick);
  
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

void sensorTaskFast(uint32_t currentMicros)
{
  if(vpStatus.simulatorLink) {
    // We take sensor inputs from the simulator (sensorData record)

    alpha = sensorData.alpha/360;
    iAS = 1852*sensorData.ias/60/60;
    rollRate = sensorData.rrate / 360;
    pitchRate = sensorData.prate / 360;
    yawRate = sensorData.yrate / 360;
    rollAngle = sensorData.roll;
    pitchAngle = sensorData.pitch;
    heading = sensorData.heading;
    
    // accX = sensorData.accx*12*0.0254;
    // accY = sensorData.accy*12*0.0254;
    // accZ = sensorData.accz*12*0.0254;
    accX = accY = accZ = 0;
    
    dynPressure = square(iAS)/2;

    iasFilter.input(iAS);
    iasFilterSlow.input(iAS);
    
    return;
  }
  
  // Alpha input
  
  alpha = clamp(alphaFilter.output(), -1.0/8, 1.0/8);
  
  // Airspeed
  
  const float pascalsPerPSI_c = 6894.7573, range_c = 2*1.1;
  const float factor_c = pascalsPerPSI_c * range_c / (1L<<(8*sizeof(uint16_t)));
    
  dynPressure = pressureBuffer.output() * factor_c;
  
  if(dynPressure > 0)
    iAS = sqrtf(2*dynPressure);
  else
    iAS = 0;
  
  iasFilter.input(iAS);
  iasFilterSlow.input(iAS);
    
  // Attitude

  ins.wait_for_sample();
  
  ahrs.update();
  
  Vector3f acc = ins.get_accel(0);

  accX = acc.x;
  accY = acc.y;
  accZ = -acc.z;

  ball.input(accY);
  
  Vector3f gyro = ins.get_gyro();
  
  rollRate = gyro.x * 180/PI / 360;
  pitchRate = gyro.y * 180/PI / 360;
  yawRate = gyro.z * 180/PI / 360;

  rollAngle = ahrs.roll * 180/PI;
  pitchAngle = ahrs.pitch * 180/PI;
  heading = ahrs.yaw * 180/PI;

  // Altitude data acquisition

  barometer.update();
  barometer.accumulate();
}

void sensorTaskSlow(uint32_t currentMicros)
{
  // Altitude

  if(vpStatus.simulatorLink)
    altitude = 12*25.4*sensorData.alt / 1000;
  else
    altitude = (float) barometer.get_altitude();
}

void sensorMonitorTask(uint32_t currentMicros)
{
  static uint32_t iasLastAlive;

  if(fabsf(iAS - iasFilterSlow.output()) > 0.5) {
    if(vpStatus.pitotBlocked) {
      consoleNoteLn_P(PSTR("IAS sensor block CLEARED"));
      vpStatus.pitotBlocked = false;
    }
    
    iasLastAlive = currentMicros;
  } else if(!vpStatus.simulatorLink
	    && currentMicros - iasLastAlive > 10.0e6 && !vpStatus.pitotBlocked) {
    consoleNoteLn_P(PSTR("IAS sensor appears BLOCKED"));
    vpStatus.pitotBlocked = true;
  }
}

void alphaLogTask(uint32_t currentMicros)
{
  logAlpha();  
}

void controlLogTask(uint32_t currentMicros)
{
  logAttitude();
  logInput();
  logActuator();
  logConfig();
}

void positionLogTask(uint32_t currentMicros)
{
  logPosition();
}

void cycleTimeMonitor(float value)
{
  cycleTimeStore[cycleTimePtr] = value;
  
  if(cycleTimePtr < cycleTimeWindow-1)
    cycleTimePtr++;
  else {
    cycleTimePtr = 0;
    cycleTimesValid = true;
  }
  
  if(cycleMin < 0.0) {
    cycleMin = cycleMax = value;
  } else {
    cycleMin = fminf(cycleMin, value);
    cycleMax = fmaxf(cycleMax, value);
  }

  cycleTimeAcc.input(value);
}

int compareFloat(const void *a, const void *b)
{
  if(*(float*)a < *(float*)b)
    return -1;
  else if(*(float*)a > *(float*)b)
    return 1;
  else return 0;    
}

void measurementTask(uint32_t currentMicros)
{
  static uint32_t prevMeasurement;
 
  // Idle measurement
  
  idleAvg = 7*idleAvg/8 + (float) idleMicros/1e6/8;
  idleMicros = 0;

  // PPM monitoring
  
  FORBID;
  ppmFreq = 1.0e6 * ppmFrames / (currentMicros - prevMeasurement);
  ppmFrames = 0;
  PERMIT;

  // Sim link monitoring

  simInputFreq = 1.0e6 * simFrames / (currentMicros - prevMeasurement);
  simFrames = 0;

  // Log bandwidth

  logBandWidth = 1.0e6 * writeBytesCum / (currentMicros - prevMeasurement);
  writeBytesCum = 0;
  
  prevMeasurement = currentMicros;

  // Cycle time monitoring
  
  if(cycleTimeCounter > 3)
    return;
    
  if(cycleTimesValid) {
    qsort((void*) cycleTimeStore, cycleTimeWindow, sizeof(float), compareFloat);
    
    consoleNote_P(PSTR("Cycle time (min, median, max) = "));
    consolePrint(cycleTimeStore[0]*1e3);
    consolePrint(", ");
    consolePrint(cycleTimeStore[cycleTimeWindow/2]*1e3);
    consolePrint(", ");
    consolePrintLn(cycleTimeStore[cycleTimeWindow-1]*1e3);
    
    float sum = 0;
    for(int i = 0; i < cycleTimeWindow; i++)
      sum += cycleTimeStore[i];
    cycleMean = sum / cycleTimeWindow;
    cycleTimesValid = false;
    cycleTimePtr = 0;
    cycleTimeCounter++;
  }
}

const float testGainStep_c = 0.05;

bool increasing, autoTestCompleted;
int oscCount;
float finalK, finalKxIAS, finalT, finalIAS;

void testStateMachine(uint32_t currentMicros)
{
  static uint32_t nextTransition;

  if(!vpMode.test || !vpMode.autoTest) {
    nextTransition = 0;
    testState = idle_c;
    return;
  }
  
  if(currentMicros < nextTransition)
    return;
  
  switch(testState) {
  case start_c:
    increasing = true;
    oscCount = 0;
    testState = trim_c;
    nextTransition = currentMicros+4*1e6;
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
    nextTransition = currentMicros+1e6/4;
    autoTestCompleted = false;
    break;

  case pert0_c:
    testState = pert1_c;
    nextTransition = currentMicros+1e6/4;
    break;

  case pert1_c:
    testState = wait_c;
    nextTransition = currentMicros+4*1e6;
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
      
      nextTransition = currentMicros + 3*1e6/2;
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

Accumulator analAvg, analLowpass;
float upSwing, downSwing, prevUpSwing, prevDownSwing;

void analyzerTask(uint32_t currentMicros)
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
  
  uint32_t halfCycle = currentMicros - prevCrossing;  
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
    prevCrossing = currentMicros;
}

float quantize(float param)
{
  static int state;
  const int steps = 20;

  if((int) ((param-1.0/steps/2)*steps) > state)
    state = (param-1.0/steps/2)*steps;
  else if((int) ((param+1.0/steps/2)*steps) < state)
    state = (param+1.0/steps/2)*steps;

  return (float) state / steps;
}

float testGainExpoFunction(float range, float param)
{
  return exp(log(4)*(2*quantize(param)-1))*range;
}

float testGainExpo(float range, float param)
{
  return testGainExpoFunction(range, param);
}

float testGainExpoReversed(float range, float param)
{
  return testGainExpoFunction(range, 1 - param);
}

float testGainLinear(float start, float stop, float param)
{
  float q = quantize(param);
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

float s_Ku_ref, i_Ku_ref;

const float minAlpha = (-2.0/360);
const float origoAlpha = (-5.0/360);

void configurationTask(uint32_t currentMicros)
{
  //
  // Are we armed yet or being armed now?
  //
  
  if(!vpStatus.armed) {
    if(trimButton.doublePulse() && aileStick < -0.90 && elevStick > 0.90) {
      consoleNoteLn_P(PSTR("We're now ARMED"));
      vpStatus.armed = true;
      
      if(!vpStatus.consoleLink)
	vpStatus.silent = true;

      elevModeButton.reset();
      aileModeButton.reset();
      gearButton.reset();
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
    
    lastNegativeIAS = currentMicros;

  } else if(currentMicros - lastNegativeIAS > 1e6 && !vpStatus.positiveIAS) {
    consoleNoteLn_P(PSTR("We have POSITIVE AIRSPEED"));
    vpStatus.positiveIAS = true;
  }
  
  float turnRate = fabsf(rollRate) + fabsf(pitchRate) + fabsf(yawRate);
  float acceleration = sqrtf(square(accX) + square(accY) + square(accZ));

  accFilter.input(acceleration);
  
  bool motionDetected = vpStatus.positiveIAS || turnRate > 3.0/360
    || fabsf(acceleration - accFilter.output()) > 0.3;
  
  static uint32_t lastMotion;

  if(motionDetected) {
    if(vpStatus.fullStop) {
      consoleNoteLn_P(PSTR("We appear to be MOVING"));
      vpStatus.fullStop = false;
    }
    
    lastMotion = currentMicros;

  } else if(currentMicros - lastMotion > 10.0e6 && !vpStatus.fullStop) {
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
  // Configuration control
  //

  if(gearButton.doublePulse()) {
    //
    // GEAR DOUBLE PULSE: FAILSAFE MODE SELECT
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
    
  } else if(gearButton.singlePulse() && !gearOutput) {
    //
    // GEAR SINGLE PULSE: GEAR UP
    //
    
    consoleNoteLn_P(PSTR("Gear UP"));
    gearOutput = 1;

  } else if(gearButton.depressed() && gearOutput) {
    //
    // GEAR CONTINUOUS: GEAR DOWN
    //
    
    consoleNoteLn_P(PSTR("Gear DOWN"));
    gearOutput = 0;
  }

  if(aileModeButton.singlePulse()) {
    //
    // AILE MODE PULSE : DISABLE BANK LIMITER
    //
  
    if(vpMode.alphaFailSafe || vpMode.sensorFailSafe) {
      vpMode.alphaFailSafe = vpMode.sensorFailSafe = false;
      consoleNoteLn_P(PSTR("Alpha/Sensor failsafe DISABLED"));
            
    } else {
      if(vpMode.bankLimiter || vpMode.slowFlight) {
	consoleNoteLn_P(PSTR("Bank limiter/slow flight mode DISABLED"));
	vpMode.wingLeveler = vpMode.bankLimiter = vpMode.slowFlight = false;
	
      } else if(!vpMode.takeOff && iasFilter.output() < vpParam.iasMin*2/3) {
	consoleNoteLn_P(PSTR("TakeOff mode ENABLED"));
	vpMode.takeOff = true;
	vpMode.slowFlight = false;
	elevTrim = vpParam.takeoffTrim;
      }
    }

    logMark();
  } else if(aileModeButton.depressed()) {
    //
    // AILE MODE CONTINUOUS : LEVEL WINGS
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

  if(elevModeButton.singlePulse() && vpMode.slowFlight) {
    //
    // ELEV MODE PULSE : DISABLE ALPHA HOLD
    //
  
    consoleNoteLn_P(PSTR("Slow flight mode DISABLED"));
    vpMode.slowFlight = false;
  } else if(elevModeButton.depressed() && !vpMode.slowFlight) {
    //
    // ELEV MODE CONTINUOUS : ENABLE ALPHA HOLD / SLOW FLIGHT
    //
  
    consoleNoteLn_P(PSTR("Slow flight mode ENABLED"));
    vpMode.slowFlight = true;
    vpMode.takeOff = false;
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
  // Map mode to features
  //
  
  vpFeature.stabilizeBank = vpFeature.pusher = !vpMode.takeOff;
  vpFeature.stabilizePitch = vpFeature.alphaHold
    = vpMode.slowFlight && !vpMode.takeOff;
  vpFeature.pitchHold = vpFeature.autoBall = false;
  
  //
  // Failsafe mode interpretation
  //

  if(vpMode.rxFailSafe) {
    vpFeature.stabilizePitch = vpFeature.stabilizeBank
      = vpFeature.pusher = vpFeature.alphaHold = vpMode.bankLimiter = true;

    elevTrim = elevFromAlpha(thresholdAlpha) - elevTrimSub;
    
  } else if(vpMode.sensorFailSafe) {
    vpFeature.stabilizePitch = vpFeature.stabilizeBank
      = vpFeature.pitchHold = vpFeature.alphaHold = vpFeature.pusher
      = vpMode.bankLimiter = vpMode.wingLeveler = vpMode.takeOff
      = vpMode.slowFlight = false;

  } else if(vpMode.alphaFailSafe)
    vpFeature.stabilizePitch = vpFeature.pitchHold = vpFeature.alphaHold
      = vpFeature.pusher = vpMode.takeOff = vpMode.slowFlight = false;

  // Safety scaling (test mode 0)
  
  float scale = 1.0;
  
  if(vpMode.test && nvState.testChannel == 0)
    scale = testGainLinear(1.0/3, 1.0);
  
  // Default controller settings

  float s_Ku = scaleByIAS(vpParam.s_Ku_C, stabilityAileExp1_c);
  float i_Ku = scaleByIAS(vpParam.i_Ku_C, stabilityElevExp1_c);
  
  aileCtrl.setZieglerNicholsPID(s_Ku*scale, vpParam.s_Tu);
  elevCtrl.setZieglerNicholsPID(i_Ku*scale, vpParam.i_Tu);
  pushCtrl.setZieglerNicholsPID(i_Ku*scale, vpParam.i_Tu);
  
  rudderCtrl.setZieglerNicholsPI(vpParam.r_Ku*scale, vpParam.r_Tu);

  autoAlphaP = vpParam.o_P;
  maxAlpha = vpParam.alphaMax;
  rudderMix = vpParam.r_Mix;
  levelBank = 0;
  
  aileRateLimiter.setRate(vpParam.servoRate/(90.0/2)/vpParam.aileDefl);
  flapRateLimiter.setRate(0.5);
  alphaRateLimiter.setRate(1.5/360);
  
  // Then apply test modes
  
  if(vpMode.test) {
    switch(nvState.testChannel) {
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
      // Auto ball gain
         
      vpFeature.stabilizeBank = vpMode.bankLimiter = vpMode.wingLeveler = true;
      vpFeature.autoBall = true;
      rudderMix = 0;
      rudderCtrl.setPID(testGain = testGainExpo(vpParam.r_Ku), 0, 0);
      break;
            
    case 6:
      // Aileron and rudder calibration, straight and level flight with
      // ball centered, reduced controller gain to increase stability
         
      vpFeature.stabilizeBank = vpMode.bankLimiter = vpMode.wingLeveler = true;
      vpFeature.autoBall = true;
      rudderMix = 0;
      aileCtrl.
	setZieglerNicholsPID(s_Ku*testGain, vpParam.s_Tu);
      rudderCtrl.
	setZieglerNicholsPI(vpParam.r_Ku*testGain, vpParam.r_Tu);
      break;

    case 7:
      // Auto ball empirical gain, PI
       
      vpFeature.autoBall = true;
      rudderCtrl.setZieglerNicholsPI(testGain = testGainExpo(vpParam.r_Ku),
				     vpParam.r_Tu);
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
      elevFromAlpha(clamp(alpha, vpParam.alphaZeroLift, maxAlpha))
      - elevStick - elevTrim;
}

void loopTask(uint32_t currentMicros)
{
  if(vpMode.loop) {
    consolePrint("alpha = ");
    consolePrint(alpha*360);

    /*    
    consolePrint(" InputVec = ( ");
    for(uint8_t i = 0; i < sizeof(ppmInputs)/sizeof(void*); i++) {
      consolePrint(inputValue(ppmInputs[i]), 2);
      consolePrint(" ");
    }      
    consolePrint(")");
    */
    consolePrint(" IAS = ");
    consolePrint(iAS);
    /*
    consolePrint(" IAS(filt) = ");
    consolePrint(iasFilter.output());
    consolePrint(" IAS(slow) = ");
    consolePrint(iasFilterSlow.output());
    */
    
    consolePrint(" avg G = ");
    consolePrint(accFilter.output());

    /*    consolePrint(" ppmFreq = ");
    consolePrint(ppmFreq);
    consolePrint(" aileStick = ");
    consolePrint(aileStick);
    consolePrint(" elevStick = ");
    consolePrint(elevStick);
    consolePrint(" rudderStick = ");
    consolePrint(rudderStick);
    */
    /*
    consolePrint(" ball = ");
    consolePrint(ball.output(), 2);
    */
    
    /*
    consolePrint(" acc = (");
    consolePrint(accX, 2);
    consolePrint(", ");
    consolePrint(accY, 2);
    consolePrint(", ");
    consolePrint(accZ, 2);
    consolePrint(")");
    */
    /*
    consolePrint(" roll = ");
    consolePrint(rollAngle, 2);
    consolePrint(" (rate = ");
    consolePrint(rollRate*360, 1);

    consolePrint(") pitch = ");
    consolePrint(pitchAngle, 2);
    consolePrint(" (rate = ");
    consolePrint(pitchRate*360, 1);
    consolePrint(")");
    */
/*    consolePrint(" heading = ");
    consolePrint(heading);
*/
    /*
    consolePrint(" alt(GPS) = ");
    consolePrint(altitude);
    consolePrint(" m (");
    consolePrint(gpsFix.altitude);
    consolePrint(" m)");
    */
    /*
    consolePrint(" speed = ");
    consolePrint(gpsFix.speed);
    */
    /* 
    consolePrint(" targetPR = ");
    consolePrint(targetPitchRate*360);
    */
    /*
    consolePrint(" elevOutput% = ");
    consolePrint(elevOutput*100);
    */
    consolePrint(" target = ");
    consolePrint(targetAlpha*360);
    consolePrint(" trim% = ");
    consolePrint(effTrim*100);

    /*    
    consolePrint(" param = ");
    consolePrint(parameter);  
    */
    /*
    consolePrint(" testGain = ");
    consolePrint(testGain);
    */
    /* 
    consolePrint(" Qparam = ");
    consolePrint(quantize(parameter));  
    */
    consolePrint("      \r");
    consoleFlush();
  }
}

void communicationTask(uint32_t currentMicros)
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

void gpsTask(uint32_t currentMicros)
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

float randomNum(float small, float large)
{
  return small + (large-small)*(float) (rand() % 1000) / 1000;
}

float levelTurnPitchRate(float bank, float target)
{
  const float alphaCL0 = vpParam.alphaZeroLift,
    ratio = (target - alphaCL0) / (vpParam.alphaMax - alphaCL0);
  
  return square(square(sin(bank/RAD)))
    *ratio*iasFilter.output()*G/square(vpParam.iasMin)*RAD/360;
}

void controlTask(uint32_t currentMicros)
{
  // Cycle time bookkeeping 
  
  if(controlCycleEnded > 0) {
    controlCycle = (currentMicros - controlCycleEnded)/1.0e6;
    cycleTimeMonitor(controlCycle);
  }
  
  controlCycleEnded = currentMicros;

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
    
  targetAlpha =
    clamp(alphaFromElev(elevOutput), -vpParam.alphaMax, effMaxAlpha);

  if(vpMode.rxFailSafe)
    targetAlpha = alphaRateLimiter.input(targetAlpha, controlCycle);
  else
    alphaRateLimiter.reset(targetAlpha);

  if(vpFeature.alphaHold)
    targetPitchRate = levelTurnPitchRate(rollAngle, targetAlpha)
      + (targetAlpha - alpha)*autoAlphaP*maxPitchRate;
  
  else if(vpFeature.pitchHold)
    targetPitchRate = (5 + effStick*30 - pitchAngle)/90 * maxPitchRate;

  else
    targetPitchRate = effStick * maxPitchRate;

  elevOutputFeedForward = elevFromAlpha(targetAlpha);

  if(vpFeature.stabilizePitch) {
    elevCtrl.input(targetPitchRate - pitchRate, controlCycle);
    
    elevOutput = elevCtrl.output();

    static float elevTestBias = 0;

    if((vpMode.test && (nvState.testChannel == 2
		       || nvState.testChannel == 3))
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
    pushCtrl.input(levelTurnPitchRate(rollAngle, effMaxAlpha)
		   + (effMaxAlpha - alpha)*autoAlphaP*maxPitchRate - pitchRate,
		   controlCycle);

    elevOutput = fminf(elevOutput, pushCtrl.output());
  } else
    pushCtrl.reset(elevOutput, 0);
  
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
      aileOutput = clamp(aileOutput - rollAngle/60, -1, 1);
    
    aileCtrl.reset(aileOutput, 0);
    
  } else {
    
    const float factor_c = maxRollRate/60;

    if(vpMode.wingLeveler)
      // Strong leveler enabled
        
      targetRollRate =
	clamp((levelBank + aileStick*maxBank - rollAngle)*factor_c,
	      -maxRollRate, maxRollRate);

    else if(vpMode.bankLimiter) {
      // Bank limiter + weak leveling

      targetRollRate -=
	factor_c*clamp(rollAngle, -vpParam.wl_Limit, vpParam.wl_Limit);
      
      targetRollRate =
	clamp(targetRollRate,
	      (-maxBank - rollAngle)*factor_c, (maxBank - rollAngle)*factor_c);
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

void actuatorTask(uint32_t currentMicros)
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

void trimTask(uint32_t currentMicros)
{
  if(trimButton.state())
    elevTrim += clamp(elevStick/TRIM_HZ, -0.125/TRIM_HZ, 0.125/TRIM_HZ);

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

void heartBeatTask(uint32_t currentMicros)
{
  if(!heartBeatCount && linkDownCount++ > 2)
    vpStatus.consoleLink = vpStatus.simulatorLink = false;

  if(vpStatus.simulatorLink && currentMicros - simTimeStamp > 1.0e6) {
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

void blinkTask(uint32_t currentMicros)
{
  float ledRatio = vpMode.test ? 0.0 : !logInitialized ? 1.0 : (vpMode.sensorFailSafe || !vpStatus.armed) ? 0.5 : alpha > 0.0 ? 0.90 : 0.10;
  static int tick = 0;
  
  tick = (tick + 1) % (LED_TICK/LED_HZ);

  setPinState(&RED_LED, tick < ledRatio*LED_TICK/LED_HZ ? 0 : 1);
}

void simulatorLinkTask(uint32_t currentMicros)
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

void controlTaskGroup(uint32_t currentMicros)
{
  testStateMachine(currentMicros);
  receiverTask(currentMicros);
  sensorTaskFast(currentMicros);
  controlTask(currentMicros);
  actuatorTask(currentMicros);
  analyzerTask(currentMicros);
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
  { controlLogTask,
    HZ_TO_PERIOD(LOG_HZ_CONTROL) },
  { positionLogTask,
    HZ_TO_PERIOD(LOG_HZ_SLOW) },
  { logSaveTask,
    HZ_TO_PERIOD(LOG_HZ_SLOW) },
  { cacheTask,
    HZ_TO_PERIOD(LOG_HZ_SLOW) },
  { measurementTask,
    HZ_TO_PERIOD(1) },
  { heartBeatTask,
    HZ_TO_PERIOD(1) },
  { loopTask,
    HZ_TO_PERIOD(10) },
  { NULL } };

int scheduler(uint32_t currentMicros)
{
  struct Task *task = taskList;
  
  while(task->code) {
    if(task->lastExecuted + task->period < currentMicros
      || task->lastExecuted > currentMicros) {
      task->code(currentMicros);
      task->lastExecuted = currentMicros;
      
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
  
  // serial_manager.init_console();

  cliSerial = hal.console;

  vpStatus.consoleLink = true;
  
  consoleNoteLn_P(PSTR("Project | Alpha"));   
  consoleNote_P(PSTR("Init Free RAM: "));
  consolePrintLn((unsigned long) hal.util->available_memory());

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

  // Servos

  consoleNoteLn_P(PSTR("Initializing servos"));

  pwmTimerInit(hwTimers, sizeof(hwTimers)/sizeof(struct HWTimer*));
  pwmOutputInitList(pwmOutput, sizeof(pwmOutput)/sizeof(struct PWMOutput));

  // Misc sensors
  
  consoleNote_P(PSTR("Initializing barometer... "));

  barometer.init();
  barometer.calibrate();
  
  consolePrintLn_P(PSTR("  done"));
  
  consoleNote_P(PSTR("Initializing INS/AHRS... "));
  consoleFlush();
  
  ins.init(AP_InertialSensor::COLD_START, AP_InertialSensor::RATE_50HZ);
  ahrs.init();

  consoleNoteLn_P(PSTR("  done"));
  
  // LED output

  configureOutput(&RED_LED);
  configureOutput(&GREEN_LED);
  configureOutput(&BLUE_LED);

  setPinState(&RED_LED, 1);
  setPinState(&GREEN_LED, 1);
  setPinState(&BLUE_LED, 1);

  // Alpha filter (sliding average over alphaWindow_c/seconds)
  
  alphaFilter.setWindow(alphaWindow_c*ALPHA_HZ);

  // Elevator delay

  elevatorDelay.setDelay(2);
  
  // Misc filters

  ball.setTau(70);
  cycleTimeAcc.setTau(10);
  analLowpass.setTau(2);
  analAvg.setTau(10);
  iasFilter.setTau(2);
  iasFilterSlow.setTau(100);
  accFilter.setTau(100);
  accFilter.reset(G);
  
  // Done
  
  consoleNoteLn_P(PSTR("Initialized"));
  
  datagramTxStart(DG_INITIALIZED);
  datagramTxEnd();
}

void loop() 
{
  // Invoke scheduler
  
  uint32_t currentTime = hal.scheduler->micros();
    
  if(!scheduler(currentTime))
    // Idle
      
    backgroundTask(1000);
}

AP_HAL_MAIN();
