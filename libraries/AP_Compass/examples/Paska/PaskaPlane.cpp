#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include <avr/interrupt.h>
#include <avr/io.h>
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
#include "Datagram.h"
#include "Command.h"
#include <AP_Progmem/AP_Progmem.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_AVR/AP_HAL_AVR.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_AHRS/AP_AHRS.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;
AP_HAL::BetterStream* cliSerial;

AP_Baro barometer;
AP_InertialSensor ins;
AP_GPS gps;
AP_AHRS_DCM ahrs {ins,  barometer, gps};

//
// HW config
//

#define MEGAMINI

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

#ifdef MEGAMINI

struct PinDescriptor ppmInputPin = { PortL, 1 }; 
struct RxInputRecord aileInput, elevInput, rudderInput,
  switchInput, tuningKnobInput;
struct RxInputRecord *ppmInputs[] = 
{ &aileInput, &elevInput, NULL, &rudderInput, &switchInput, &tuningKnobInput };

#else

struct RxInputRecord aileInput = { { PortK, 0 } }; 
struct RxInputRecord elevInput = { { PortK, 1 } };
struct RxInputRecord switchInput = { { PortK, 3 } };
struct RxInputRecord tuningKnobInput = { { PortK, 3 } };
struct RxInputRecord rudderInput = { { PortK, 4 } };

#endif

//
// Servo PWM output
//

#define NEUTRAL 1500
#define RANGE 500

#ifdef MEGAMINI

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

#else

const struct PWMOutput pwmOutput[] = {
  { { PortE, 4 }, &hwTimer3, COMnB },
  { { PortE, 5 }, &hwTimer3, COMnC },
  { { PortE, 3 }, &hwTimer3, COMnA },
  { { PortH, 3 }, &hwTimer4, COMnA },
  { { PortH, 4 }, &hwTimer4, COMnB },
  { { PortH, 5 }, &hwTimer4, COMnC }
};

#endif

//
// Function to servo output mapping
//

#define aileHandle     (paramRecord.servoAile < 0 ? NULL : (&pwmOutput[paramRecord.servoAile]))
#define elevatorHandle (paramRecord.servoElev < 0 ? NULL : (&pwmOutput[paramRecord.servoElev]))
#define flapHandle     (paramRecord.servoFlap < 0 ? NULL : (&pwmOutput[paramRecord.servoFlap]))
#define flap2Handle    (paramRecord.servoFlap2 < 0 ? NULL : (&pwmOutput[paramRecord.servoFlap2]))
#define gearHandle     (paramRecord.servoGear < 0 ? NULL : (&pwmOutput[paramRecord.servoGear]))
#define brakeHandle    (paramRecord.servoBrake < 0 ? NULL : (&pwmOutput[paramRecord.servoBrake]))
#define rudderHandle      (paramRecord.servoRudder < 0 ? NULL : (&pwmOutput[paramRecord.servoRudder]))

//
// Periodic task stuff
//

#define CONTROL_HZ 50.0
#define ALPHA_HZ (CONTROL_HZ*10)
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
  bool sensorFailSafe;
  bool rxFailSafe;
  bool autoAlpha;
  bool autoTrim;
  bool autoRudder;
  bool yawDamper;
  bool stabilizer;
  bool wingLeveler;
  bool bankLimiter;
};

struct ModeRecord mode;

struct GPSFix {
  float altitude;
  float track;
  float lat;
  float lon;
  float speed;
};

struct GPSFix gpsFix;

bool testMode = false;
bool rattling = false;
float testGain = 0;
bool switchState = false, switchStateLazy = false, echoEnabled = true;
bool iasFailed = false, iasWarn = false, alphaFailed = false, alphaWarn = false;
bool rxElevatorAlive = true, rxAileronAlive = true, rpmAlive = 0;
const int cycleTimeWindow = 31;
float cycleTimeStore[cycleTimeWindow];
int cycleTimePtr = 0;
bool cycleTimesValid;
float cycleMin = -1.0, cycleMax = -1.0, cycleMean = -1.0, cycleCum = -1.0;
const float tau = 0.1;
float dynPressure, alpha, aileStick, elevStick, aileStickRaw, elevStickRaw, rudderStick, rudderStickRaw;
uint32_t controlCycleEnded;
int initCount = 5;
bool armed = false;
float neutralStick = 0.0, neutralAlpha, targetAlpha;
float switchValue, tuningKnobValue, rpmOutput;
Controller elevCtrl, aileCtrl, pushCtrl;
float autoAlphaP, maxAlpha, yawDamperP, rudderMix;
float accX, accY, accZ, altitude,  heading, rollAngle, pitchAngle, rollRate, pitchRate, yawRate;
int cycleTimeCounter = 0;
uint32_t prevMeasurement;
float parameter;  
NewI2C I2c = NewI2C();
RunningAvgFilter alphaFilter;
DecayFilter yawFilter;
AlphaBuffer alphaBuffer, pressureBuffer;
float controlCycle = 10.0e-3;
uint32_t idleMicros;
float idleAvg, logBandWidth, ppmFreq;
bool looping, consoleConnected;
    
float elevOutput = 0, aileOutput = 0, flapOutput = 0, gearOutput = 1, brakeOutput = 0, rudderOutput = 0;

#ifdef rpmPin
struct RxInputRecord rpmInput = { rpmPin };
#endif

const uint8_t addr5048B_c = 0x40;
const uint8_t addr4525_c = 0x28;

bool read5048B(uint8_t addr, uint8_t *storage, int bytes) 
{
  return I2c.read(addr5048B_c, addr, storage, bytes) == 0;
}

bool read5048B_byte(uint8_t addr, uint8_t *result)
{
  return read5048B(addr, result, sizeof(*result));
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
  logGeneric(lc_mode, 
    (mode.rxFailSafe ? 32 : 0) 
    + (mode.sensorFailSafe ? 16 : 0) 
    + (mode.wingLeveler ? 8 : 0) 
    + (mode.bankLimiter ? 4 : 0) 
    + (mode.autoAlpha ? 1 : 0)); 
    
  logGeneric(lc_target, targetAlpha*360);
  logGeneric(lc_trim, neutralAlpha*360);

  if(testMode) {
    logGeneric(lc_gain, testGain);
    logGeneric(lc_test, stateRecord.testChannel);
  } else {
    logGeneric(lc_gain, 0);
    logGeneric(lc_test, 0);
  }
}

void logPosition(void)
{
  logGeneric(lc_speed, gpsFix.speed);
  logGeneric(lc_track, gpsFix.track);
  logGeneric(lc_altgps, gpsFix.altitude);
  logGeneric(lc_altbaro, altitude);
}
  
void logInput(void)
{
  logGeneric(lc_ailestick, aileStick);
  logGeneric(lc_elevstick, elevStick);
  logGeneric(lc_rudstick, rudderStick);
}

void logActuator(void)
{
  logGeneric(lc_aileron, aileOutput);
  logGeneric(lc_elevator, elevOutput);
  logGeneric(lc_rudder, rudderOutput);
}

void logRPM(void)
{
  logGeneric(lc_rpm, rpmOutput);
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

bool readSwitch()
{
  static bool state = false;
  const float value = decodePWM(switchValue);
  
  if(fabsf(value) > 0.3)
    state = value < 0.0;
  
  return state;
}

float readParameter()
{
  return ((decodePWM(tuningKnobValue) + 1.0) / 2 - 0.05) / 0.95;
}

float readRPM()
{
  return rpmOutput;
}

void rpmMeasure(bool on)
{
#if defined(rpmPin)
  if(on)
    PCMSK2 |= 1<<rpmInput.index;
  else
    PCMSK2 &= ~(1<<rpmInput.index);
#endif
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
#define AS5048B_RESOLUTION 16384.0 //14 bits

bool readAlpha_5048B(int16_t *result) {
  uint16_t raw = 0;
  
  if(alphaFailed)
    // Stop trying
    
    return false;
  
  if(!read5048B_word14(AS5048B_ANGLMSB_REG, &raw))
    // Failed
    return false;

  // The value is good, use it

  if(result)
    *result = (int16_t) (raw - paramRecord.alphaRef);
  
  return true;
}

bool readPressure(int16_t *result) 
{
  static uint32_t acc;
  static int accCount;
  const int log2CalibWindow = 6;
  
  if(iasFailed)
    // Stop trying
    return false;
  
  uint16_t raw = 0;

  if(!read4525DO_word14(&raw))
    return false;

  if(accCount < 1<<log2CalibWindow) {
    acc += raw;
    accCount++;
  } else if(result)
    *result = (raw - (acc>>log2CalibWindow))<<2;
  
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

void executeCommand(const char *buf, int bufLen)
{
  if(echoEnabled) {
    consolePrint("\r// % ");
    consolePrint(buf, bufLen);  
    consolePrintLn("          ");
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
    case c_arm:
      rattling = false;
      armed = true;
      break;
    
    case c_rattle:
      rattling = true;
      armed = false;
      break;
    
    case c_disarm:
      armed = false;
      consoleNoteLn_P(PSTR("We're DISARMED"));
      break;
    
    case c_talk:
      talk = true;
      consoleNoteLn_P(PSTR("Hello world"));
      break;
    
    case c_test:
      if(numParams > 0) {
	stateRecord.testChannel = param[0];
	storeNVState();
      }

      consoleNote_P(PSTR("Current test channel = "));
      consolePrintLn(stateRecord.testChannel);
      break;

    case c_center:
      paramRecord.elevZero = elevStickRaw;
      paramRecord.aileZero = aileStickRaw;
      paramRecord.rudderZero = rudderStickRaw;
      consoleNoteLn_P(PSTR("Stick center set"));
      break;
    
    case c_zero:
      paramRecord.alphaRef += (int16_t) ((1L<<16) * alpha);
      consoleNoteLn_P(PSTR("Alpha zero set"));
      break;

    case c_alpha:
      if(numParams > 0) {
	paramRecord.alphaRef +=
	  (int16_t) ((1L<<16) * (alpha - (float) param[0] / 360));
	consoleNoteLn_P(PSTR("Alpha calibrated"));
      }
      break;

    case c_loop:
      looping = true;
      rpmMeasure(true);
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

    case c_console:
      consoleNoteLn_P(PSTR("Console CONNECTED"));
      consoleConnected = true;
      break;
      
    case c_dump:
      if(numParams > 0)
	logDump(param[0]);
      else
	logDump(-1);
      break;
    
    case c_dumpz:
      logDumpBinary();
      break;
    
    case c_backup:
      dumpParams();
      break;

    case c_stamp:
      if(numParams > 0) {
	stateRecord.logStamp = param[0];
	storeNVState();
      }
      consoleNote_P(PSTR("Current log stamp is "));
      consolePrintLn(stateRecord.logStamp);  
      break;

    case c_model:
      if(numParams > 0) {
	if(param[0] > MAX_MODELS-1)
	  param[0] = MAX_MODELS-1;
	setModel(param[0]);
	storeNVState();
      } else { 
	consoleNote_P(PSTR("Current model is "));
	consolePrintLn(stateRecord.model); 
      }
      break;
    
   case c_params:
     consoleNote_P(PSTR("SETTINGS (MODEL "));
      consolePrint(stateRecord.model);
      consolePrintLn(")");
      printParams();
      break;

    case c_clear:
      logClear();
      cycleMin = cycleMax = cycleCum = cycleMean = -1;
      cycleTimesValid = false;
      cycleTimePtr = 0;
      break;

    case c_stop:
      logDisable();
      break;

    case c_start:
      logEnable();
      break;

    case c_cycle:
      cycleTimeCounter = 0;
      break;

    case c_report:
      consoleNote_P(PSTR("Idle avg = "));
      consolePrintLn(idleAvg*100,1);
      consoleNote_P(PSTR("PPM frequency = "));
      consolePrintLn(ppmFreq);
      consoleNote_P(PSTR("Alpha = "));
      consolePrint(360*alpha);
      if(alphaFailed)
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
      consolePrintLn(cycleCum*1e3);
      consoleNote_P(PSTR("Warning flags :"));
      if(pciWarn)
	consolePrint_P(PSTR(" SPURIOUS_PCINT"));
      if(alphaWarn)
	consolePrint_P(PSTR(" ALPHA_SENSOR"));
      if(ppmWarnShort)
	consolePrint_P(PSTR(" PPM_SHORT"));
      if(ppmWarnSlow)
	consolePrint_P(PSTR(" PPM_SLOW"));
      if(eepromWarn)
	consolePrint_P(PSTR(" EEPROM"));
      if(eepromFailed)
	consolePrint_P(PSTR(" EEPROM_FAILED"));
      if(iasWarn)
	consolePrint_P(PSTR(" IAS_WARN"));
      if(iasFailed)
	consolePrint_P(PSTR(" IAS_FAILED"));
      if(alphaBuffer.warn)
	consolePrint_P(PSTR(" ALPHA_BUFFER"));
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
      pciWarn = alphaWarn = alphaFailed = pushCtrl.warn = elevCtrl.warn
	= alphaBuffer.warn = eepromWarn = eepromFailed = ppmWarnShort
	= ppmWarnSlow = aileCtrl.warn = false;
      consoleNoteLn_P(PSTR("Warning flags reset"));
      break;
    
    case c_rpm:
      stateRecord.logRPM = param[0] > 0.5 ? true : false;
      consoleNote_P(PSTR("RPM logging "));
      consolePrintLn(stateRecord.logRPM ? "ENABLED" : "DISABLED");
      rpmMeasure(stateRecord.logRPM);
      storeNVState();
      break;
      
    default:
      consolePrint_P(PSTR("Sorry, command not implemented: \""));
      consolePrint(buf, tokenLen);
      consolePrintLn("\"");
      break;
    }
  }
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
  logRPM();
}

void logSaveTask(uint32_t currentMicros)
{
  logSave(logStartCallback);
}

void alphaTask(uint32_t currentMicros)
{
  int16_t raw = 0;
  static int failCount = 0;
  
  if(!handleFailure("alpha", !readAlpha_5048B(&raw), &alphaWarn, &alphaFailed, &failCount))
    alphaBuffer.input((float) raw / (1L<<(8*sizeof(raw))));
}

#define NULLZONE 0.075

float applyNullZone(float value)
{
  if(value < -NULLZONE)
    return (value + NULLZONE) / (1.0 - NULLZONE);
  else if(value > NULLZONE)
    return (value - NULLZONE) / (1.0 - NULLZONE);

  return 0.0;
}

void receiverTask(uint32_t currentMicros)
{
  if(inputValid(&aileInput)) {
    aileStickRaw = decodePWM(inputValue(&aileInput));
    aileStick = applyNullZone(clamp(aileStickRaw - paramRecord.aileZero, -1, 1));
  }
  
  if(inputValid(&rudderInput)) {
    rudderStickRaw = decodePWM(inputValue(&rudderInput));
    rudderStick = applyNullZone(clamp(rudderStickRaw - paramRecord.rudderZero, -1, 1));
  }
  
  if(inputValid(&elevInput)) {
    elevStickRaw = decodePWM(inputValue(&elevInput));
    elevStick = clamp(elevStickRaw - paramRecord.elevZero, -1, 1);
  }

  if(inputValid(&switchInput))
    switchValue = inputValue(&switchInput);
    
  if(inputValid(&tuningKnobInput))
    tuningKnobValue = inputValue(&tuningKnobInput);
}

void sensorTaskFast(uint32_t currentMicros)
{
  // Alpha input
  
  alpha = alphaBuffer.output();
  
  // Attitude

  ins.wait_for_sample();
  
  ahrs.update();
  
  Vector3f acc = ins.get_accel(0);

  accX = acc.x;
  accY = acc.y;
  accZ = -acc.z;
  
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

  // Airspeed data acquisition
  
  int16_t raw = 0;
  static int failCount = 0;
  
  if(!handleFailure("airspeed", !readPressure(&raw), &iasWarn, &iasFailed, &failCount))
    pressureBuffer.input((float) raw);
}

void sensorTaskSlow(uint32_t currentMicros)
{
  // Altitude

  altitude = (float) barometer.get_altitude();

  // Airspeed
  
  const float pascalsPerPSI_c = 6894.7573, range_c = 2*1.1;
  const float factor_c = pascalsPerPSI_c * range_c / (1L<<(8*sizeof(uint16_t)));
    
  dynPressure = pressureBuffer.output() * factor_c;
}

const int numPoles = 4;

void rpmTask(uint32_t currentMicros)
{
#if defined(rpmPin)
  static uint32_t prev;

  FORBID;
  
  uint32_t count = rpmInput.pulseCount;
  rpmInput.pulseCount = 0;
  
  PERMIT;
  
  uint32_t delta = currentMicros - prev;
  
  prev = currentMicros;
  
  if(prev > 0)
    rpmOutput = 1.0e6*2.0*60*count/numPoles/delta;
#endif
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
  logRPM();
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
    cycleMin = cycleMax = cycleCum = value;
  } else {
    cycleMin = min(cycleMin, value);
    cycleMax = max(cycleMax, value);
    cycleCum = cycleCum*(1-tau) + value*tau;
  }
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
  // Idle measurement
  
  idleAvg = 7*idleAvg/8 + (float) idleMicros/1e6/8;
  idleMicros = 0;

  // PPM monitoring
  
  FORBID;
  ppmFreq = 1.0e6 * ppmFrames / (currentMicros - prevMeasurement);
  ppmFrames = 0;
  PERMIT;

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

float testGainExpo(float range)
{
  return exp(log(3)*(2*parameter-1))*range;
}

float testGainLinear(float start, float stop)
{
  return start + parameter*(stop - start);
}

#define maxAutoAlpha (maxAlpha/square(1.1))  // Stall speed + 10%

void configurationTask(uint32_t currentMicros)
{   
  static bool pulseArmed = false, pulsePolarity = false;
  static int pulseCount = 0; 
   
  int prev = switchState;
  switchState = readSwitch();
  bool switchStateChange = switchState != prev;

  if(initCount > 0) {
    if(rxElevatorAlive && rxAileronAlive)
      initCount--;
      
    return;
  }

  static uint32_t lastUpdate;
          
  if(switchStateChange) {
    if(switchState != switchStateLazy) {
      pulseArmed = true;
      pulsePolarity = switchState;
    } else if(pulseArmed) {
      // We detected a pulse
            
      pulseCount++;            
      
      if(pulseCount > 1) {
        pulseCount = 0;
             
        if(armed) {   
          if(!pulsePolarity) {
            if(!mode.sensorFailSafe) {
              consoleNoteLn_P(PSTR("Failsafe ENABLED"));
              mode.sensorFailSafe = true;
            } else {
              logDisable();
            }
          } else {
            consoleNoteLn_P(PSTR("Climbing out"));
            gearOutput = 1;
            if(flapOutput > 2)
              flapOutput = 2;
          }
        }
      }
      
      pulseArmed = false;
    }
          
    lastUpdate = hal.scheduler->micros();
  } else if(hal.scheduler->micros() - lastUpdate > 1.0e6/3) {
    if(switchState != switchStateLazy) {
      switchStateLazy = switchState;
        
      consoleNote_P(PSTR("Lazy switch "));
      consolePrintLn(switchState ? "ON" : "OFF");

      if(switchState) {
        if(armed && !logEnabled && !consoleConnected)
          logEnable();
          
        consoleNoteLn_P(PSTR("Wing leveler ENABLED"));
        mode.wingLeveler = true;
      } else {
        if(armed && logEnabled)
          logMark();
      }
    }
          
    if(pulseCount > 0) {
      if(pulsePolarity) {
        if(mode.sensorFailSafe) {
            mode.sensorFailSafe = false;
            consoleNoteLn_P(PSTR("Sensor failsafe DISABLED"));
            
        } else if(!armed) {
          consoleNoteLn_P(PSTR("We're now ARMED"));
          armed = true;
          talk = false;
          
        } /* else if(testMode) {
          consoleNote_P(PSTR("Test channel incremented to "));
          consolePrintLn(++stateRecord.testChannel);

	  } */ else if(paramRecord.servoGear < 0 || gearOutput > 0) {
          if(flapOutput > 0) {
            flapOutput--;
            consoleNote_P(PSTR("Flaps RETRACTED to "));
            consolePrintLn(flapOutput);
          }
        } else {
          consoleNoteLn_P(PSTR("Gear UP"));
          gearOutput = 1;
        }
      } else {
	/* if(testMode) {
          consoleNoteLn_P(PSTR("Test channel RESET"));
          // stateRecord.testChannel = 0;
	  } else */
	if(paramRecord.servoGear > -1 && gearOutput > 0) {
          consoleNoteLn_P(PSTR("Gear DOWN"));
          gearOutput = 0;
        } else if(flapOutput < 3) {
          flapOutput++;
          consoleNote_P(PSTR("Flaps EXTENDED to "));
          consolePrintLn(flapOutput);
        }
      }
    }
          
    pulseArmed = false;
    pulseCount = 0;
  }

  // Test parameter

  parameter = readParameter();

  if(!testMode && parameter > 0.5) {
    testMode = true;

    consoleNoteLn_P(PSTR("Test mode ENABLED"));
    
  } else if(testMode && parameter < 0) {
    testMode = false;
    
    consoleNoteLn_P(PSTR("Test mode DISABLED"));
  }

  // Wing leveler disable when stick input detected
  
  if(mode.wingLeveler && fabsf(aileStick) > 0.1) {
    consoleNoteLn_P(PSTR("Wing leveler DISABLED"));
    mode.wingLeveler = false;
  }
  
  // Mode-to-feature mapping: first nominal values
      
  mode.stabilizer = true;
  mode.bankLimiter = switchStateLazy;
  mode.autoAlpha = mode.autoTrim = flapOutput > 0;
  mode.autoRudder = false;
  mode.yawDamper = false; // !switchStateLazy;
  
  // Receiver fail detection

  if(switchState && aileStick < -0.90 && elevStick > 0.90) {
    if(!mode.rxFailSafe) {
      consoleNoteLn_P(PSTR("Receiver failsafe mode ENABLED"));
      mode.rxFailSafe = true;
      mode.sensorFailSafe = false;
    }
  } else if(mode.rxFailSafe) {
    consoleNoteLn_P(PSTR("Receiver failsafe mode DISABLED"));
    mode.rxFailSafe = false;
  }
  
  if(mode.rxFailSafe) {    
    // Receiver failsafe mode settings
    
    mode.autoAlpha = mode.autoTrim = mode.bankLimiter = true;
    mode.autoRudder = mode.yawDamper = false;
    neutralStick = 0;
  }
  
  // Default controller settings

  if(paramRecord.c_PID)
    aileCtrl.setZieglerNicholsPID(paramRecord.s_Ku, paramRecord.s_Tu);
  else
    aileCtrl.setZieglerNicholsPI(paramRecord.s_Ku, paramRecord.s_Tu);

  elevCtrl.setZieglerNicholsPID(paramRecord.i_Ku, paramRecord.i_Tu);
  pushCtrl.setZieglerNicholsPID(paramRecord.i_Ku, paramRecord.i_Tu);

  autoAlphaP = paramRecord.o_P;
  maxAlpha = paramRecord.alphaMax;
  yawDamperP = paramRecord.yd_P;
  rudderMix = paramRecord.r_Mix;
  yawFilter.setTau(paramRecord.yd_Tau/log(CONTROL_HZ));
  
  // Then apply test modes
  
  if(testMode) {
    testGain = testGainLinear(0, 1);
     switch(stateRecord.testChannel) {
     case 1:
       // Wing stabilizer gain
         
       mode.stabilizer = mode.bankLimiter = mode.wingLeveler = true;
       aileCtrl.setPID(testGain = testGainExpo(paramRecord.s_Ku), 0, 0);
       break;
            
     case 3:
       // Elevator stabilizer gain, outer loop enabled
         
       mode.autoTrim = mode.autoAlpha = true;
       elevCtrl.setPID(testGain = testGainExpo(paramRecord.i_Ku), 0, 0);
       break;
         
     case 4:
       // Auto alpha outer loop gain
         
       mode.autoTrim = mode.autoAlpha = true;
       autoAlphaP = testGain = testGainExpo(paramRecord.o_P);
       break;
         
     case 7:
       // Max alpha

       mode.stabilizer = mode.bankLimiter = mode.wingLeveler = true;
       maxAlpha = testGain = testGainLinear(10.0/360, 20.0/360);
       break;         

     case 10:
       // Aileron to rudder mix

       rudderMix = testGain = testGainExpo(paramRecord.r_Mix);
       break;
 
     case 11:
       // Yaw damper gain

       mode.yawDamper = true;
       yawDamperP = testGain = testGainExpo(paramRecord.yd_P);
       break;

     case 12:
       // Yaw damper tau

       mode.yawDamper = true;
       yawFilter.setTau((testGain = testGainLinear(0, 1)) / log(CONTROL_HZ));
       break;
    }
  }
      
  alphaFilter.input(alpha);
  
  if(!mode.autoAlpha) { 
    neutralStick = elevStick;
    neutralAlpha = clamp(alphaFilter.output(), paramRecord.alphaMin, maxAlpha);
  }
}

void loopTask(uint32_t currentMicros)
{
  if(looping) {
    consolePrint("alpha = ");
    consolePrint(alpha*360);
    consolePrint(" targAlpha = ");
    consolePrint(targetAlpha*360);
    /*    consolePrint(" dynPress = ");
    consolePrint(dynPressure);
    */

    /*    consolePrint(" ppmFreq = ");
    consolePrint(ppmFreq);
    consolePrint(" aileStick = ");
    consolePrint(aileStick);
    consolePrint(" elevStick = ");
    consolePrint(elevStick);
    consolePrint(" rudderStick = ");
    consolePrint(rudderStick);
    */
    consolePrint(" acc = (");
    consolePrint(accX, 2);
    consolePrint(", ");
    consolePrint(accY, 2);
    consolePrint(", ");
    consolePrint(accZ, 2);
    consolePrint(")");
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
    /*
    consolePrint(" rpm = ");
    consolePrint(readRPM());
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
    consolePrint(" target = ");
    consolePrint(targetAlpha*360);
    consolePrint(" trim = ");
    consolePrint(neutralAlpha*360);
*/
    
    consolePrint(" param = ");
    consolePrint(parameter);
    consolePrint(" testGain = ");
    consolePrint(testGain);
    
    consolePrint("      \r");
    consoleFlush();
  }
}

const int serialBufLen = 1<<6;
char serialBuf[serialBufLen+1];
int serialBufIndex = 0;

void communicationTask(uint32_t currentMicros)
{
  int len = 0;
       
  while((len = cliSerial->available()) > 0) {    
    int spaceLeft = serialBufLen - serialBufIndex;
    
    if(len > spaceLeft) {
      for(int i = 0; i < len - spaceLeft; i++)
        cliSerial->read();
    }
    
    len = min(len, spaceLeft);

    while(len > 0) {
      char c =  cliSerial->read();

      if(c == '\b' || c == 127) {
	if(serialBufIndex > 0) {
	  consolePrint("\b \b");
	  consoleFlush();
	  serialBufIndex--;
	}
	
      } else if(c == '\n' || c == '\r') {
	looping = false;

	if(serialBufIndex < 1)
	  consolePrintLn("// %");
	else if(serialBuf[0] == '/')
	  consolePrintLn("\n// %");
	else {
	  serialBuf[serialBufIndex] = '\0';
	  executeCommand(serialBuf, serialBufIndex);
	}

	serialBufIndex = 0;
	controlCycleEnded = 0;
	
      } else if(c != ' ' || serialBufIndex > 0) {
	const char buf[] = { c, '\0' };
	consolePrint(buf);
	consoleFlush();
	serialBuf[serialBufIndex++] = c;
      }
      
      len--;
    }
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

void controlTask(uint32_t currentMicros)
{
  // Cycle time bookkeeping 
  
  if(controlCycleEnded > 0) {
    controlCycle = (currentMicros - controlCycleEnded)/1.0e6;
    cycleTimeMonitor(controlCycle);
  }
  
  controlCycleEnded = currentMicros;
  
  // Elevator control
    
  targetAlpha = 0.0;
  elevOutput = elevStick;
 
  const float effStick = applyNullZone(elevStick - neutralStick);
    
  if(mode.rxFailSafe)
    targetAlpha = maxAutoAlpha;
  else {
    const float fract_c = 1.0/3;
    const float strength_c = max(effStick-(1.0-fract_c), 0)/fract_c;
    const float maxTargetAlpha_c =
      mixValue(strength_c, maxAutoAlpha, maxAlpha);
    const float stickRange_c = min(20.0, 90/paramRecord.ff_A);

    targetAlpha = clamp(neutralAlpha + effStick*stickRange_c/360,
			paramRecord.alphaMin, maxTargetAlpha_c);
  }
	
  float feedForward = targetAlpha*360/90*paramRecord.ff_A + paramRecord.ff_B;

  float targetPitchRate = (targetAlpha - alpha) * autoAlphaP;      

  if(mode.autoAlpha)
    elevCtrl.input(targetPitchRate - pitchRate, controlCycle);
  else
    elevCtrl.reset(elevStick - feedForward, 0.0);
    
  if(!mode.sensorFailSafe && mode.autoAlpha) {
    // Feed-forward part
    elevOutput = feedForward;
    
    if(!alphaFailed)
      // Feedback (PID output)
      elevOutput += elevCtrl.output();
  }

  // Pusher

  pushCtrl.input((maxAlpha - alpha)*paramRecord.o_P - pitchRate, controlCycle);

  if(!mode.sensorFailSafe && !alphaFailed)
    elevOutput = min(elevOutput, pushCtrl.output());
  
  // Aileron

  float maxRollRate = 270/360.0;
  float maxBank = 45.0;
    
  if(mode.autoTrim) {
    const float slowDown = neutralAlpha / maxAutoAlpha;
    
    maxBank /= 1 + slowDown;
    maxRollRate /= 1 + slowDown;
  }
  
  float targetRollRate = maxRollRate*aileStick;

  if(mode.sensorFailSafe || !mode.stabilizer) {

    // Failsafe/stabilizer disabled
    
    aileOutput = aileStick;
    aileCtrl.reset(aileOutput, 0);
    
  } else {
    
    // Roll stabilizer enabled

    const float factor_c = maxRollRate/60;
    
    if(mode.wingLeveler)
      // Strong leveler enabled
        
      targetRollRate = clamp(-rollAngle*factor_c, -maxRollRate, maxRollRate);

    else if(mode.bankLimiter) {
      // Bank limiter + weak leveling

      targetRollRate -=
	factor_c*clamp(rollAngle, -paramRecord.wl_Limit, paramRecord.wl_Limit);
      
      targetRollRate =
	clamp(targetRollRate,
	      (-maxBank - rollAngle)*factor_c, (maxBank - rollAngle)*factor_c);
    }
      
    aileCtrl.input(targetRollRate - rollRate, controlCycle);
    aileOutput = aileCtrl.output();
  }
 
  // Rudder
    
  rudderOutput = rudderStick + aileOutput*rudderMix;

  // Yaw damper

  yawFilter.input(yawRate);
  
  if(mode.yawDamper)
    rudderOutput -= yawFilter.output() * yawDamperP;
  
  if(mode.sensorFailSafe)
    rudderOutput = rudderStick;
    
  // Brake
    
  if(!mode.sensorFailSafe && gearOutput == 1)
    brakeOutput = 0;
  else
    brakeOutput = max(-elevStick, 0);
}

void actuatorTask(uint32_t currentMicros)
{
  // Actuators

  if(rattling) {
    pwmOutputWrite(aileHandle, NEUTRAL
		   + RANGE*clamp(paramRecord.aileDefl*randomNum(-1, 1) 
				 + paramRecord.aileNeutral, -1, 1));

    pwmOutputWrite(elevatorHandle, NEUTRAL
		   + RANGE*clamp(paramRecord.elevDefl*randomNum(-1, 1) 
				 + paramRecord.elevNeutral, -1, 1));
                              
    pwmOutputWrite(flapHandle, NEUTRAL
		   + RANGE*clamp(paramRecord.flapNeutral 
				 + randomNum(0, 3)*paramRecord.flapStep, -1, 1));                              

    pwmOutputWrite(flap2Handle, NEUTRAL
		   + RANGE*clamp(paramRecord.flap2Neutral 
				 - randomNum(0, 3)*paramRecord.flapStep, -1, 1));                              

    pwmOutputWrite(gearHandle, NEUTRAL - RANGE*randomNum(-1, 1));

    pwmOutputWrite(rudderHandle, NEUTRAL
		   + RANGE*clamp(paramRecord.rudderNeutral + 
		   paramRecord.rudderDefl*randomNum(0, 1), -1, 1));                

  } else if(armed) {
    pwmOutputWrite(aileHandle, NEUTRAL
		   + RANGE*clamp(paramRecord.aileDefl*aileOutput 
				 + paramRecord.aileNeutral, -1, 1));

    pwmOutputWrite(elevatorHandle, NEUTRAL
		   + RANGE*clamp(paramRecord.elevDefl*elevOutput 
				 + paramRecord.elevNeutral, -1, 1));
                              
    pwmOutputWrite(flapHandle, NEUTRAL
		   + RANGE*clamp(paramRecord.flapNeutral 
				  + flapOutput*paramRecord.flapStep, -1, 1));                              
    pwmOutputWrite(flap2Handle, NEUTRAL
		   + RANGE*clamp(paramRecord.flap2Neutral 
				  - flapOutput*paramRecord.flapStep, -1, 1));                              

    pwmOutputWrite(gearHandle, NEUTRAL - RANGE*(gearOutput*2-1));

    pwmOutputWrite(rudderHandle, NEUTRAL
		   + RANGE*clamp(paramRecord.rudderNeutral + 
				 paramRecord.rudderDefl*rudderOutput, -1, 1));                        
  } 
}

void trimTask(uint32_t currentMicros)
{
  if(mode.autoTrim && fabsf(rollAngle) < 15)
    neutralAlpha +=
      clamp((min(targetAlpha, maxAutoAlpha) - neutralAlpha)/2/TRIM_HZ,
	    -1.5/360/TRIM_HZ, 1.5/360/TRIM_HZ);
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

void heartBeatTask(uint32_t durationMicros)
{  
  static uint32_t count = 0;

  if(!talk)
    return;
  
  datagramTxStart(DG_HEARTBEAT);
  datagramTxOut((uint8_t*) &count, sizeof(count));
  datagramTxEnd();
  
  count++;   
}

void blinkTask(uint32_t currentMicros)
{
  float ledRatio = testMode ? 0.0 : !logInitialized ? 1.0 : (mode.sensorFailSafe || !armed) ? 0.5 : alpha > 0.0 ? 0.90 : 0.10;
  static int tick = 0;
  
  tick = (tick + 1) % (LED_TICK/LED_HZ);

  setPinState(&RED_LED, tick < ledRatio*LED_TICK/LED_HZ ? 0 : 1);
}

void controlTaskGroup(uint32_t currentMicros)
{
  receiverTask(currentMicros);
  sensorTaskFast(currentMicros);
  controlTask(currentMicros);
  actuatorTask(currentMicros);
}

struct Task taskList[] = {
  { communicationTask,
    HZ_TO_PERIOD(100) },
  //  { gpsTask, HZ_TO_PERIOD(100) },
  { alphaTask,
    HZ_TO_PERIOD(ALPHA_HZ) },
  { blinkTask,
    HZ_TO_PERIOD(LED_TICK) },
  { controlTaskGroup,
    HZ_TO_PERIOD(CONTROL_HZ) },
  { sensorTaskSlow,
    HZ_TO_PERIOD(CONTROL_HZ/5) },
  { trimTask,
    HZ_TO_PERIOD(TRIM_HZ) },
  { configurationTask,
    HZ_TO_PERIOD(LOG_HZ_CONTROL) },
  { rpmTask,
    HZ_TO_PERIOD(LOG_HZ_CONTROL) },
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
  // Serial comms

  // HAL

  hal.init(0, NULL);
  
  // initialise serial port
  
  // serial_manager.init_console();

  cliSerial = hal.console;
  
  consoleNoteLn_P(PSTR("Project | Alpha"));   
  consoleNote_P(PSTR("Init Free RAM: "));
  consolePrintLn((unsigned long) hal.util->available_memory());

  // I2C
  
  consoleNote_P(PSTR("Initializing I2C... "));
  
  I2c.begin();
  I2c.setSpeed(true);
  I2c.pullup(true);
  I2c.timeOut(2+EXT_EEPROM_LATENCY/1000);

  consolePrintLn_P(PSTR("done. "));
  
  // Read the non-volatile state

  readNVState();
    
  consoleNote_P(PSTR("Current model is "));
  consolePrintLn(stateRecord.model);
  
  // Param record
  
  setModel(stateRecord.model);

  // Set I2C speed
  
  TWBR = paramRecord.i2c_clkDiv;
                
  // RC input
  
#ifdef MEGAMINI

  consoleNoteLn_P(PSTR("Initializing PPM receiver"));

  configureInput(&ppmInputPin, true);
  ppmInputInit(ppmInputs, sizeof(ppmInputs)/sizeof(struct RxInputRecord*));

#else
  
  consoleNoteLn_P(PSTR("Initializing individual PWM inputs"));

  rxInputInit(&elevInput);
  rxInputInit(&aileInput);
  rxInputInit(&rudderInput);
  rxInputInit(&switchInput);
  rxInputInit(&tuningKnobInput);  

#endif

  // RPM sensor int control

#ifdef rpmPin
  pinMode(rpmPin, INPUT_PULLUP);  
  rpmMeasure(stateRecord.logRPM);
#endif

  // Servos

  consoleNoteLn_P(PSTR("Initializing servos"));

  pwmTimerInit(hwTimers, sizeof(hwTimers)/sizeof(struct HWTimer*));
  pwmOutputInitList(pwmOutput, sizeof(pwmOutput)/sizeof(struct PWMOutput));

  //
  
  alphaFilter.setWindowLen(-1);

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

  // Done
  
  consoleNoteLn_P(PSTR("Initialized"));
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

/*

// Param backup 2016/2/11
// 
// MODEL 0 L-39
// 

model 0
name L-39
min -3.00
max 13.00
5048b_ref 32268
inner_pid_zn 1.140 0.270
outer_p 10.000
stabilizer_pid_zn 1.390 0.360
rudder_pid_zn 0.500 0.330
yd_p 2.000
edefl -50.00
eneutral 0.00
ezero 3.08
eservo 1
adefl -35.00
aneutral 0.00
azero 5.77
aservo 0
rdefl 35.00
rneutral -5.00
rzero 8.42
rservo 2
fstep 0.00
fneutral 45.00 45.00
fservo -1 -1
bdefl -45.00
bneutral -45.00
bservo -1
gservo -1
store

// 
// MODEL 1 Viper
// 

model 1
name Viper
min -3.00
max 12.00
5048b_ref 0
inner_pid_zn 1.000 0.250
outer_p 10.000
stabilizer_pid_zn 1.299 0.250
rudder_pid_zn 1.000 0.330
yd_p 2.000
edefl 45.00
eneutral 0.00
ezero 0.00
eservo 1
adefl -45.00
aneutral 0.00
azero 0.00
aservo 0
rdefl 45.00
rneutral 0.00
rzero 0.00
rservo 6
fstep -15.00
fneutral 0.00 -15.00
fservo 2 3
bdefl 45.00
bneutral 0.00
bservo -1
gservo -1
store

// Param backup L-39 2016/2/2
//
// MODEL 0
//
 
model 0
name L-39
min -3.00
max 13.00
5048b_ref 32268
inner_pid_zn 1.14 0.27000
outer_p 10.00000
stabilizer_pid_zn 1.39000 0.36000
rudder_pid_zn 0.50000 0.33000
yd_p 2.00000
edefl -50.00
eneutral 0.00
ezero 3.08
eservo 1
adefl -35.00
aneutral 0.00
azero 5.77
aservo 0
rdefl 35.00
rneutral -5.00
rzero 8.42
rservo 2
fstep 0.00
fneutral 45.00 45.00
fservo -1 -1
bdefl -45.00
bneutral -45.00
bservo -1
gservo -1
store

//
// MODEL 1 : VIPER 2016/2/1
//
 
echo 0; model 1 \
min -3.00; max 12.00; 5048b_ref 0 \
inner_pid_zn 1.000 0.250; outer_p 10.00 \
stabilizer_pid_zn 1.299 0.250 \
rudder_pid_zn 1.000 0.330; yd_p 2.000 \
edefl 45.000; eneutral 0.000; ezero 3.111; eservo 1 \
adefl -45.000; aneutral 0.000; azero 6.000; aservo 0 \
fstep -15.00; fneutral 0.00 -15.00; fservo 2 3 \
bdefl 45.00; bneutral 0.00; bservo -1 \
rdefl 45.000; rneutral 5.000; rzero 8.222; rservo 6 \
gservo -1 \
echo 1; store

//
// MODEL 0 : L-39 2016/2/1
//
 
echo 0; model 0 \
min -3.00; max 13.00; 5048b_ref 32268 \
inner_pid_zn 1.148 0.270; outer_p 10.00 \
stabilizer_pid_zn 1.398 0.360 \
rudder_pid_zn 1.000 0.330; yd_p 2.000 \
edefl -50.000; eneutral 0.000; ezero 2.885; eservo 1 \
adefl -35.000; aneutral 0.000; azero 5.555; aservo 0 \
fstep 0.00; fneutral 45.00 45.00; fservo -1 -1 \
bdefl -45.00; bneutral -45.00; bservo -1 \
rdefl 45.000; rneutral 0.000; rzero -0.888; rservo -1 \
gservo -1 \
echo 1; store

 */
