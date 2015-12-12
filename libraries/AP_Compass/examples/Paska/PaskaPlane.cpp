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
struct RxInputRecord aileInput, elevInput, switchInput, tuningKnobInput;
struct RxInputRecord *ppmInputs[] = 
{ &aileInput, &elevInput, NULL, NULL, &switchInput, &tuningKnobInput };

#else

struct RxInputRecord aileInput = { { PortK, 0 } }; 
struct RxInputRecord elevInput = { { PortK, 1 } };
struct RxInputRecord switchInput = { { PortK, 2 } };
struct RxInputRecord tuningKnobInput = { { PortK, 3 } };

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

//
// Periodic task stuff
//

#define CONTROL_HZ 100
#define ALPHA_HZ (CONTROL_HZ*6)
#define TRIM_HZ 10
#define LED_HZ 3
#define LED_TICK 100

struct Task {
  void (*code)(uint32_t time);
  uint32_t period, lastExecuted;
};

#define HZ_TO_PERIOD(f) ((uint32_t) (1.0e6/(f)))

struct ModeRecord {
  bool sensorFailSafe;
  bool rxFailSafe;
  bool autoStick;
  bool autoAlpha;
  bool autoTrim;
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
bool calibrate, switchState = false, switchStateLazy = false, echoEnabled = true;
bool iasFailed = false, iasWarn = false, alphaFailed = false, alphaWarn = false;
bool calibrateStart = false, calibrateStop = false;
bool rxElevatorAlive = true, rxAileronAlive = true, rpmAlive = 0;
const int cycleTimeWindow = 31;
float cycleTimeStore[cycleTimeWindow];
int cycleTimePtr = 0;
bool cycleTimesValid;
float cycleMin = -1.0, cycleMax = -1.0, cycleMean = -1.0, cycleCum = -1.0;
const float tau = 0.1;
float dynPressure, alpha, aileStick, elevStick, aileStickRaw, elevStickRaw;
uint32_t controlCycleEnded;
int initCount = 5;
bool armed = false;
float neutralStick = 0.0, neutralAlpha, targetAlpha;
float switchValue, tuningKnobValue, rpmOutput;
Controller elevController, aileController, pusher;
float autoAlphaP, maxAlpha;
float acc,altitude,  heading, rollAngle, pitchAngle, rollRate, pitchRate;
int cycleTimeCounter = 0;
uint32_t prevMeasurement;
float parameter;  
NewI2C I2c = NewI2C();
RunningAvgFilter alphaFilter;
AlphaBuffer alphaBuffer, pressureBuffer;
float controlCycle = 10.0e-3;
uint32_t idleMicros;
float idleAvg;

float elevOutput = 0, aileOutput = 0, flapOutput = 0, gearOutput = 1, brakeOutput = 0;

#ifdef rpmPin
struct RxInputRecord rpmInput = { rpmPin };
#endif

void printParams(struct ParamRecord *p)
{
  consoleNote_P(PSTR(" AS5048B ref = "));
  consolePrintLn(p->alphaRef);
  consoleNoteLn_P(PSTR("  Autostick/pusher"));
  consoleNote_P(PSTR("    Inner P = "));
  consolePrint(p->i_Kp, 4);
  consolePrint_P(PSTR(" I = "));
  consolePrint(p->i_Ki, 4);
  consolePrint_P(PSTR(" D = "));
  consolePrint(p->i_Kd, 4);
  consolePrint_P(PSTR(" Outer P = "));
  consolePrintLn(p->o_P, 4);
  consoleNoteLn_P(PSTR("  Stabilizer"));
  consoleNote_P(PSTR("    P = "));
  consolePrint(p->s_Kp, 4);
  consolePrint_P(PSTR(" I = "));
  consolePrint(p->s_Ki, 4);
  consolePrint_P(PSTR(" D = "));
  consolePrintLn(p->s_Kd, 4);
  consoleNote_P(PSTR("  Alpha min = "));
  consolePrint(p->alphaMin*360);
  consolePrint_P(PSTR("  max = "));
  consolePrintLn(p->alphaMax*360);
  consoleNoteLn_P(PSTR("  Elevator"));
  consoleNote_P(PSTR("    deflection = "));
  consolePrint(p->elevDefl*90);
  consolePrint_P(PSTR(" neutral = "));
  consolePrintLn(p->elevNeutral*90);
  consoleNoteLn_P(PSTR("  Aileron"));
  consoleNote_P(PSTR("    deflection = "));
  consolePrint(p->aileDefl*90);
  consolePrint_P(PSTR(" neutral = "));
  consolePrintLn(p->aileNeutral*90);
  consoleNoteLn_P(PSTR("  Flap"));
  consoleNote_P(PSTR("    step = "));
  consolePrint(p->flapStep*90);
  consolePrint_P(PSTR(" neutral = "));
  consolePrint(p->flapNeutral*90);
  consolePrint_P(PSTR(" ("));
  consolePrint(p->flap2Neutral*90);
  consolePrintLn_P(PSTR(")"));
}

void dumpParams(struct ParamRecord *p)
{
  consolePrint_P(PSTR("5048b_ref "));
  consolePrint(p->alphaRef);
  consolePrint_P(PSTR("; inner_pid "));
  consolePrint(p->i_Kp, 4);
  consolePrint(" ");
  consolePrint(p->i_Ki, 4);
  consolePrint(" ");
  consolePrint(p->i_Kd, 4);
  consolePrint_P(PSTR("; outer_p "));
  consolePrint(p->o_P);
  consolePrint_P(PSTR("; stabilizer_pid "));
  consolePrint(p->s_Kp, 4);
  consolePrint(" ");
  consolePrint(p->s_Ki, 4);
  consolePrint(" ");
  consolePrint(p->s_Kd, 4);
  consolePrint_P(PSTR("; min ")); consolePrint(p->alphaMin*360);
  consolePrint_P(PSTR("; max ")); consolePrint(p->alphaMax*360);
  consolePrint_P(PSTR("; edefl ")); consolePrint(p->elevDefl*90);
  consolePrint_P(PSTR("; eneutral ")); consolePrint(p->elevNeutral*90);
  consolePrint_P(PSTR("; ezero ")); consolePrint(p->elevZero*90);
  consolePrint_P(PSTR("; adefl ")); consolePrint(p->aileDefl*90);
  consolePrint_P(PSTR("; aneutral ")); consolePrint(p->aileNeutral*90);
  consolePrint_P(PSTR("; azero ")); consolePrint(p->aileZero*90);
  consolePrint_P(PSTR("; fstep ")); consolePrint(p->flapStep*90);
  consolePrint_P(PSTR("; fneutral ")); consolePrint(p->flapNeutral*90);
  consolePrint_P(PSTR("; fneutral ")); consolePrint(p->flapNeutral*90); consolePrint(" "); consolePrint(p->flap2Neutral*90); 
  consolePrint_P(PSTR("; bdefl ")); consolePrint(p->brakeDefl*90);
  consolePrint_P(PSTR("; bneutral ")); consolePrint(p->brakeNeutral*90);
}

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
  logGeneric(l_alpha, alpha*360);
}

void logConfig(void)
{
  logGeneric(l_mode, 
    (mode.rxFailSafe ? 32 : 0) 
    + (mode.sensorFailSafe ? 16 : 0) 
    + (mode.wingLeveler ? 8 : 0) 
    + (mode.bankLimiter ? 4 : 0) 
    + (mode.autoStick ? 2 : 0) 
    + (mode.autoAlpha ? 1 : 0)); 
    
  logGeneric(l_target, targetAlpha*360);
  logGeneric(l_trim, neutralAlpha*360);

  if(testMode) {
    logGeneric(l_gain, testGain);
    logGeneric(l_test, stateRecord.testChannel);
  } else {
    logGeneric(l_gain, 0);
    logGeneric(l_test, 0);
  }
}

void logPosition(void)
{
  logGeneric(l_speed, gpsFix.speed);
  logGeneric(l_track, gpsFix.track);
  logGeneric(l_altgps, gpsFix.altitude);
  logGeneric(l_altbaro, altitude);
}
  
void logInput(void)
{
  logGeneric(l_ailestick, aileStick);
  logGeneric(l_elevstick, elevStick);
}

void logActuator(void)
{
  logGeneric(l_aileron, aileOutput);
  logGeneric(l_elevator, elevOutput);
}

void logRPM(void)
{
  logGeneric(l_rpm, rpmOutput);
}

void logAttitude(void)
{
  logGeneric(l_dynpressure, dynPressure);
  logGeneric(l_acc, acc);
  logGeneric(l_roll, rollAngle);
  logGeneric(l_rollrate, rollRate*360);
  logGeneric(l_pitch, pitchAngle);
  logGeneric(l_pitchrate, pitchRate*360);
  logGeneric(l_heading, heading);
}

bool readSwitch() {
  return decodePWM(switchValue) < 0.0;
}

float readParameter() {
  return (decodePWM(tuningKnobValue) + 1.0) / 2;
}

float readRPM() {
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
  uint16_t raw = 0;
  
  if(iasFailed)
    // Stop trying
    return false;
  
  if(!read4525DO_word14(&raw))
    return false;

  if(result)
    *result = (raw - 8176)<<2;
  
  return true;
}

typedef enum {
  c_, c_adefl, c_edefl, c_clear, c_dump, c_min, c_max, c_zero,
  c_eneutral, c_aneutral, c_store, c_report, c_stop, c_cycle,
  c_read, c_write, c_5048b_addr, c_5048b_read, c_start, c_init, c_filtlen,
  c_params, c_defaults, c_reset, c_24l256_addr, c_24l256_clk, c_5048b_clk, c_center,
  c_loop, c_stamp, c_model, c_alpha, c_flapneutral, c_flapstep, c_backup, c_echo,
  c_ezero, c_azero, c_5048b_ref, c_bdefl, c_bneutral, c_rpm, c_baud, c_dumpz,
  c_stabilizer_pid, c_stabilizer_pid_zn, c_stabilizer_pi_zn, c_outer_p, c_rattle,
  c_inner_pid, c_inner_pid_zn, c_inner_pi_zn, c_arm, c_disarm, c_test, c_talk } command_t;

struct command {
  const char *c_string;
  command_t c_token;
};

const struct command commands[] = {
  { "edefl", c_edefl },
  { "adefl", c_adefl },
  { "eneutral", c_eneutral },
  { "aneutral", c_aneutral },
  { "ezero", c_ezero },
  { "azero", c_azero },
  { "zero", c_zero },
  { "alpha", c_alpha },
  { "max", c_max },
  { "min", c_min },
  { "dump", c_dump },
  { "clear", c_clear },
  { "store", c_store },
  { "report", c_report },
  { "stop", c_stop },
  { "cycle", c_cycle },
  { "read", c_read },
  { "write", c_write },
  { "5048b_addr", c_5048b_addr },
  { "5048b_ref", c_5048b_ref },
  { "24l256_addr", c_24l256_addr },
  { "5048b_read", c_5048b_read },
  { "start", c_start },
  { "init", c_init },
  { "filtlen", c_filtlen },
  { "defaults", c_defaults },
  { "params", c_params },
  { "reset", c_reset },
  { "5048b_clk", c_5048b_clk },
  { "24l256_clk", c_24l256_clk },
  { "center", c_center },
  { "loop", c_loop },
  { "stamp", c_stamp },
  { "model", c_model },
  { "alpha", c_alpha },
  { "fneutral", c_flapneutral },
  { "fstep", c_flapstep },
  { "backup", c_backup },
  { "echo", c_echo },
  { "bdefl", c_bdefl },
  { "bneutral", c_bneutral },
  { "rpm", c_rpm },
  { "baud", c_baud },
  { "dumpz", c_dumpz },
  { "stabilizer_pid", c_stabilizer_pid },
  { "stabilizer_pid_zn", c_stabilizer_pid_zn },
  { "stabilizer_pi_zn", c_stabilizer_pi_zn },
  { "outer_p", c_outer_p },
  { "inner_pid", c_inner_pid },
  { "inner_pid_zn", c_inner_pid_zn },
  { "inner_pi_zn", c_inner_pi_zn },
  { "arm", c_arm },
  { "disarm", c_disarm },
  { "test", c_test },
  { "cycle", c_cycle },
  { "talk", c_talk },
  { "rattle", c_rattle },
  { "", c_ }
};

bool looping;
    
const int maxCmdLen = 40;
char cmdBuf[maxCmdLen];
int cmdBufLen;

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
  if(echoEnabled)
    consolePrintf("\r// %% %s          \n", buf);  
  
  const int maxParams = 8;

  int index = 0, prevIndex = 0, numParams = 0, tokenLen = bufLen;
  float param[maxParams];

  for(int i = 0; i < maxParams; i++)
    param[i] = 0.0;

  if((index = indexOf(buf, ' ')) > 0) {
    tokenLen = index;
    
    do {
      prevIndex = index;
    
      index = indexOf(buf, ' ', index+1);
      
      if(index < 0)
        index = bufLen;
        
      float value = 0.0;
      int exponent = 0;
      bool sign = false, deci = false;

      for(int i = prevIndex+1; i < index; i++) {
        char c = buf[i];

        switch(c) {
          case '-':
            sign = true;
            break;
            
          case '.':
            deci = true;
            break;
            
          case '0':
          case '1':
          case '2':
          case '3':
          case '4':
          case '5':
          case '6':
          case '7':
          case '8':
          case '9':
            if(deci) {
              value += (float) (c - '0') / pow(10, ++exponent);
            } else {
               value *= (float) 10;
               value += (float) (c - '0');
            }
            break;
            
          default:
            consoleNote_P(PSTR("weird char = "));
            consolePrintLn(c);
        }
      }
            
      if(sign)
        value = -value;
    
      if(numParams < maxParams)
        param[numParams++] = value;
    } while(index < bufLen);
  }
  
  int j = 0;
  
  while(commands[j].c_token != c_) {
    if(!strncmp(buf, commands[j].c_string, tokenLen))
      break;
    j++;
  }

  switch(commands[j].c_token) {
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
    if(numParams > 0)
      stateRecord.testChannel = param[0];
    else {
      consoleNote_P(PSTR("Current test channel = "));
      consolePrintLn(stateRecord.testChannel);
    }
    break;
    
  case c_stabilizer_pid:
    if(numParams > 0)
      paramRecord.s_Kp = param[0];
    if(numParams > 1)
      paramRecord.s_Ki = param[1];
    if(numParams > 2)
      paramRecord.s_Kd = param[2];

    aileController.setPID(paramRecord.s_Kp, paramRecord.s_Ki, paramRecord.s_Kd);

    consoleNote_P(PSTR("Stabilizer P = "));
    consolePrint(paramRecord.s_Kp);
    consolePrint_P(PSTR(", I = "));
    consolePrint(paramRecord.s_Ki);
    consolePrint_P(PSTR(", D = "));
    consolePrintLn(paramRecord.s_Kd);    
    break;
    
  case c_stabilizer_pid_zn:
    if(numParams > 1) {
      consoleNote_P(PSTR("Stabilizer Z-N PID Ku, Tu = "));
      consolePrint(param[0]);
      consolePrint_P(PSTR(", "));
      consolePrintLn(param[1]);
    
      aileController.setZieglerNicholsPID(param[0], param[1]);

      paramRecord.s_Kp = aileController.getP();
      paramRecord.s_Ki = aileController.getI();
      paramRecord.s_Kd = aileController.getD();

      consoleNote_P(PSTR("  Resulting P = "));
      consolePrint(paramRecord.s_Kp);
      consolePrint_P(PSTR(", I = "));
      consolePrint(paramRecord.s_Ki);
      consolePrint_P(PSTR(", D = "));
      consolePrintLn(paramRecord.s_Kd);
    } else {
      float Ku, Tu;
      aileController.getZieglerNicholsPID(&Ku, &Tu);
      consoleNote_P(PSTR("Current stabilizer "));
      consolePrint(aileController.getD() > 0.0 ? "PID" : "PI");
      consolePrint_P(PSTR(" Ku, Tu = "));
      consolePrint(Ku, 4);
      consolePrint_P(PSTR(", "));
      consolePrintLn(Tu, 4);      
    }
    break;
    
  case c_stabilizer_pi_zn:
    if(numParams > 1) {
      consoleNote_P(PSTR("Stabilizer Z-N PI Ku, Tu = "));
      consolePrint(param[0]);
      consolePrint_P(PSTR(", "));
      consolePrintLn(param[1]);
    
      aileController.setZieglerNicholsPI(param[0], param[1]);

      paramRecord.s_Kp = aileController.getP();
      paramRecord.s_Ki = aileController.getI();
      paramRecord.s_Kd = aileController.getD();

      consoleNote_P(PSTR("  Resulting P = "));
      consolePrint(paramRecord.s_Kp);
      consolePrint_P(PSTR(", I = "));
      consolePrint(paramRecord.s_Ki);
      consolePrint_P(PSTR(", D = "));
      consolePrintLn(paramRecord.s_Kd);
    }
    break;
    
  case c_inner_pid:
    if(numParams > 0)
      paramRecord.i_Kp = param[0];
    if(numParams > 1)
      paramRecord.i_Ki = param[1];
    if(numParams > 2)
      paramRecord.i_Kd = param[2];

    consoleNote_P(PSTR("Autostick/Pusher inner P = "));
    consolePrint(paramRecord.i_Kp);
    consolePrint_P(PSTR(", I = "));
    consolePrint(paramRecord.i_Ki);
    consolePrint_P(PSTR(", D = "));
    consolePrintLn(paramRecord.i_Kd);
    
    elevController.setPID(paramRecord.i_Kp, paramRecord.i_Ki, paramRecord.i_Kd);
    pusher.setPID(paramRecord.i_Kp, paramRecord.i_Ki, paramRecord.i_Kd);
    break;
    
  case c_inner_pid_zn:
    if(numParams > 1) {
      consoleNote_P(PSTR("Autostick/Pusher inner Z-N PID Ku, Tu = "));
      consolePrint(param[0]);
      consolePrint_P(PSTR(", "));
      consolePrintLn(param[1]);
    
      elevController.setZieglerNicholsPID(param[0], param[1]);
      pusher.setZieglerNicholsPID(param[0], param[1]);

      paramRecord.i_Kp = elevController.getP();
      paramRecord.i_Ki = elevController.getI();
      paramRecord.i_Kd = elevController.getD();

      consoleNote_P(PSTR("  Resulting P = "));
      consolePrint(paramRecord.i_Kp);
      consolePrint_P(PSTR(", I = "));
      consolePrint(paramRecord.i_Ki);
      consolePrint_P(PSTR(", D = "));
      consolePrintLn(paramRecord.i_Kd);
    } else {
      float Ku, Tu;
      elevController.getZieglerNicholsPID(&Ku, &Tu);
      consoleNote_P(PSTR("Current autostick "));
      consolePrint(elevController.getD() > 0.0 ? "PID" : "PI");
      consolePrint_P(PSTR(" Ku, Tu = "));
      consolePrint(Ku, 4);
      consolePrint_P(PSTR(", "));
      consolePrintLn(Tu, 4);      
    }
    break;
    
  case c_inner_pi_zn:
    if(numParams > 1) {
      consoleNote_P(PSTR("Autostick/Pusher inner Z-N PI Ku, Tu = "));
      consolePrint(param[0]);
      consolePrint_P(PSTR(", "));
      consolePrintLn(param[1]);
    
      elevController.setZieglerNicholsPI(param[0], param[1]);
      pusher.setZieglerNicholsPI(param[0], param[1]);

      paramRecord.i_Kp = elevController.getP();
      paramRecord.i_Ki = elevController.getI();
      paramRecord.i_Kd = elevController.getD();

      consoleNote_P(PSTR("  Resulting P = "));
      consolePrint(paramRecord.i_Kp);
      consolePrint_P(PSTR(", I = "));
      consolePrint(paramRecord.i_Ki);
      consolePrint_P(PSTR(", D = "));
      consolePrintLn(paramRecord.i_Kd);
    }
    break;
    
  case c_outer_p:
    if(numParams > 0)
      autoAlphaP = paramRecord.o_P = param[0];

    consoleNote_P(PSTR("Autostick outer P = "));
    consolePrintLn(paramRecord.o_P);
    break;
    
  case c_center:
    paramRecord.elevZero = elevStickRaw;
    paramRecord.aileZero = aileStickRaw;
    consoleNoteLn_P(PSTR("Stick center set"));
    consolePrintLn(paramRecord.o_P);
    break;
    
  case c_eneutral:
    if(numParams > 0)
      paramRecord.elevNeutral = param[0]/90.0;
    else {
      consoleNote_P(PSTR("Elev neutral = "));
      consolePrintLn(paramRecord.elevNeutral*90.0);
    }
    break;
    
  case c_edefl:
    if(numParams > 0)
      paramRecord.elevDefl = param[0] / 90.0;
    else {
      consoleNote_P(PSTR("Elev defl = "));
      consolePrintLn(paramRecord.elevDefl*90.0);
    }
    break;
    
  case c_aneutral:
    if(numParams > 0)
      paramRecord.aileNeutral = param[0]/90.0;
    else {
      consoleNote_P(PSTR("Aile neutral = "));
      consolePrintLn(paramRecord.aileNeutral*90.0);
    }
    break;
    
  case c_adefl:
    if(numParams > 0)
      paramRecord.aileDefl = param[0] / 90.0;
    else {
      consoleNote_P(PSTR("Aile defl = "));
      consolePrintLn(paramRecord.aileDefl*90.0);
    }
    break;
    
  case c_bneutral:
    if(numParams > 0)
      paramRecord.brakeNeutral = param[0]/90.0;
    else {
      consoleNote_P(PSTR("Brake neutral = "));
      consolePrintLn(paramRecord.brakeNeutral*90.0);
    }
    break;
    
  case c_bdefl:
    if(numParams > 0)
      paramRecord.brakeDefl = param[0] / 90.0;
    else {
      consoleNoteLn_P(PSTR("Brake defl = "));
      consolePrintLn(paramRecord.brakeDefl*90.0);
    }
    break;
    
  case c_flapneutral:
    if(numParams > 0) {
      paramRecord.flapNeutral = param[0] / 90.0;
      if(numParams > 1)
	paramRecord.flap2Neutral = param[1] / 90.0;
    } else {
      consoleNote_P(PSTR("Flap neutral = "));
      consolePrint(paramRecord.flapNeutral*90.0);
      consolePrint_P(PSTR(", "));
      consolePrintLn(paramRecord.flap2Neutral*90.0);
    }
    break;
    
  case c_flapstep:
    if(numParams > 0)
      paramRecord.flapStep = param[0] / 90.0;
    else {
      consoleNote_P(PSTR("Flap step = "));
      consolePrintLn(paramRecord.flapStep*90.0);
    }
    break;
    
  case c_min:
    if(numParams > 0)
      paramRecord.alphaMin = param[0]/360.0;
    break;

  case c_max:
    if(numParams > 0)
      maxAlpha = paramRecord.alphaMax = param[0]/360.0;
    break;

  case c_zero:
    paramRecord.alphaRef += (int16_t) ((1L<<16) * alpha);
    break;

  case c_5048b_ref:
    if(numParams > 0)
      paramRecord.alphaRef = (int16_t) param[0];
    break;

  case c_alpha:
    if(numParams > 0)
      paramRecord.alphaRef += (int16_t) ((1L<<16) * (alpha - (float) param[0] / 360));
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

  case c_dump:
    consoleNoteLn_P(PSTR("Log contents:"));
    if(numParams > 0)
      logDump(param[0]);
    else
      logDump(-1);
    break;
    
  case c_dumpz:
    consoleNoteLn_P(PSTR("Compressed log contents:"));
    //    logDumpBinary();
    break;
    
  case c_backup:
    consoleNoteLn_P(PSTR("Param backup"));
    consolePrintLn("//");
    consoleNote_P(PSTR("MODEL "));
    consolePrintLn(stateRecord.model);
    consolePrintLn("//");
    consolePrintLn("");
    consolePrint_P(PSTR("echo 0; model "));
    consolePrint(stateRecord.model);
    consolePrint("; ");
    dumpParams(&paramRecord);
    consolePrintLn_P(PSTR("; echo 1; store"));
    consolePrintLn("");
    break;

  case c_stamp:
    if(numParams > 0) {
      consoleNote_P(PSTR("Log stamp set to "));
      consolePrintLn(param[0]);  
      stateRecord.logStamp = param[0];
      storeNVState();
    } else {
      consoleNote_P(PSTR("Current log stamp is "));
      consolePrintLn(stateRecord.logStamp);  
    }
    break;

  case c_model:
    if(numParams < 1) {
      consoleNote_P(PSTR("Current model is "));
      consolePrintLn(stateRecord.model); 
    } else { 
      if(param[0] > MAX_MODELS-1)
        param[0] = MAX_MODELS-1;
      setModel(param[0]);
      if(echoEnabled)
	storeNVState();
    }
    break;

  case c_echo:
    if(numParams > 0 && param[0] < 1.0) 
      echoEnabled = talk = false;
    else {
      consoleNoteLn_P(PSTR("Echo enabled"));
      echoEnabled = talk = true;
    }
    break;
    
  case c_init:
//    logEndStamp = ENTRY_VALUE(-100);
//    logPtr = logSize - 1;

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
    if(pusher.warn)
      consolePrint_P(PSTR(" PUSHER"));
    if(elevController.warn)
      consolePrint_P(PSTR(" AUTOSTICK"));
      
    consolePrintLn("");

    consoleNote_P(PSTR("Log write bandwidth = "));
    consolePrint(logBandWidth);
    consolePrintLn_P(PSTR(" bytes/sec"));
    break;

  case c_reset:
    pciWarn = alphaWarn = alphaFailed = pusher.warn = elevController.warn
      = alphaBuffer.warn = eepromWarn = eepromFailed = ppmWarnShort
      = ppmWarnSlow = false;
    consoleNoteLn_P(PSTR("Warning flags reset"));
    break;
    
  case c_5048b_addr:
    paramRecord.i2c_5048B = param[0];
    break;
    
  case c_24l256_addr:
    paramRecord.i2c_24L256 = param[0];
    break;
    
  case c_5048b_read:
//    consolePrintLn(read5048B_byte((int) param), DEC);
    break;
    
  case c_rpm:
    stateRecord.logRPM = param[0] > 0.5 ? true : false;
    consoleNote_P(PSTR("RPM logging "));
    consolePrintLn(stateRecord.logRPM ? "ENABLED" : "DISABLED");
    rpmMeasure(stateRecord.logRPM);
    storeNVState();
    break;
    
   case c_defaults:
//      paramRecord[stateRecord.model] = paramDefaults;
//      consoleNoteLn("Defaults restored");
      break;
      
   case c_params:
     consoleNote_P(PSTR("SETTINGS (MODEL "));
      consolePrint(stateRecord.model);
      consolePrintLn(")");
      printParams(&paramRecord);
      break;
  
  case c_5048b_clk:
    //    paramRecord.clk_5048B = param[0];
    break;
    
  case c_24l256_clk:
    //    paramRecord.clk_24L256 = param[0];
    break;
    
  default:
    consolePrintLn_P(PSTR("Command not recognized"));
    break;
  }
}

void executeCommandSeries(const char *buffer, int len)
{
  int index = 0;
  bool nothing = true;
    
  while(index < len) {
    cmdBufLen = 0;
    
    while(index < len && buffer[index] != ';') {
      if(cmdBufLen < maxCmdLen-1 && (buffer[index] != ' ' || cmdBufLen > 0)) {
        cmdBuf[cmdBufLen++] = buffer[index];
      }
      index++;
    }
        
    cmdBuf[cmdBufLen] = '\0';
    
    if(cmdBufLen > 0) {
      executeCommand(cmdBuf, cmdBufLen);
      nothing = false;
    }

    index++;    
  }

  if(nothing)
    consolePrintLn("// %");
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
  
  if(inputValid(&elevInput)) {
    elevStickRaw = decodePWM(inputValue(&elevInput));
    elevStick = clamp(elevStickRaw - paramRecord.elevZero, -1, 1);

    if(mode.autoStick) {
      elevStick = applyNullZone(elevStick - neutralStick);
    }
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
  
  Vector3f gyro = ins.get_gyro();
  
  rollRate = gyro.x * 180/PI / 360;
  pitchRate = gyro.y * 180/PI / 360;

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
  
  /*
  acc = (float) imu.accSmooth[2] / (1<<9);
  */
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

float ppmFreq;

void measurementTask(uint32_t currentMicros)
{
  // Idle measurement
  
  idleAvg = 7*idleAvg/8 + (float) idleMicros/1e6/8;
  idleMicros = 0;

  // Cycle time monitoring
  
  if(cycleTimeCounter > 5)
    return;
    
  FORBID;
  ppmFreq = 1.0e6 * ppmFrames / (currentMicros - prevMeasurement);
  ppmFrames = 0;
  PERMIT;

  logBandWidth = 1.0e6 * logBytesCum / (currentMicros - prevMeasurement);
  logBytesCum = 0;
  prevMeasurement = currentMicros;

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
  return exp(3*(parameter-1))*range;
}

float testGainLinear(float start, float stop)
{
  return start + parameter*(stop - start);
}

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
        if(armed && !logEnabled)
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
            consoleNoteLn_P(PSTR("Failsafe DISABLED"));
            
        } else if(!armed) {
          consoleNoteLn_P(PSTR("We're now ARMED"));
          armed = true;
          talk = false;
          
        } else if(testMode) {
          consoleNote_P(PSTR("Test channel incremented to "));
          consolePrintLn(++stateRecord.testChannel);

        } else if(paramRecord.servoGear < 0 || gearOutput > 0) {
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
        if(testMode) {
          consoleNoteLn_P(PSTR("Test channel RESET"));
          // stateRecord.testChannel = 0;
        } else if(paramRecord.servoGear > -1 && gearOutput > 0) {
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
    
  } else if(testMode && parameter < 0.1) {
    testMode = false;
    
    consoleNoteLn_P(PSTR("Test mode DISABLED"));
  }

  if(mode.wingLeveler && absVal(aileStick) > 0.1)
    mode.wingLeveler = false;
  
  // Mode-to-feature mapping: first nominal values
      
  mode.stabilizer = true;
  mode.bankLimiter = switchStateLazy;
  mode.autoStick = mode.autoAlpha = mode.autoTrim = false;

/*  
  if(!gearOutput || flapOutput > 0)
    // Auto stick 
    mode.autoStick = true;

  if(!gearOutput && flapOutput > 0)
    // Gear down and flaps out, auto alpha    
    mode.autoAlpha = mode.autoTrim = true;
   */
  
  // mode.autoStick = mode.autoAlpha = mode.autoTrim = !gearOutput;
  mode.autoStick = mode.autoAlpha = mode.autoTrim = flapOutput > 0;
   
  // Detect transmitter fail

  if(mode.bankLimiter && aileStick < -0.90 && elevStick > 0.90) {
    if(!mode.rxFailSafe) {
      consoleNoteLn_P(PSTR("Receiver failsafe mode ENABLED"));
      mode.rxFailSafe = true;
      mode.sensorFailSafe = false;
    }
  } else if(mode.rxFailSafe) {
    consoleNoteLn_P(PSTR("Receiver failsafe mode DISABLED"));
    mode.rxFailSafe = false;
  }
      
  // Default controller settings
     
  elevController.setPID(
    paramRecord.i_Kp,
    paramRecord.i_Ki,
    paramRecord.i_Kd);
  
  pusher.setPID(
    paramRecord.i_Kp,
    paramRecord.i_Ki,
    paramRecord.i_Kd);
  
  aileController.setPID(
    paramRecord.s_Kp,
    paramRecord.s_Ki,
    paramRecord.s_Kd);

  autoAlphaP = paramRecord.o_P;
  maxAlpha = paramRecord.alphaMax;
 
  // Then apply test modes
  
  if(testMode) {
     switch(stateRecord.testChannel) {
       case 1:
         // Wing stabilizer gain
         
         mode.stabilizer = mode.bankLimiter = mode.wingLeveler = true;
         aileController.setPID(testGain = testGainExpo(5), 0, 0);
         break;
            
       case 2:
         // Elevator stabilizer gain, outer loop disabled
         
         mode.autoStick = true;
         mode.autoTrim = mode.autoAlpha = false;
         elevController.setPID(testGain = testGainExpo(6), 0, 0);
         break;
         
       case 3:
         // Elevator stabilizer gain, outer loop enabled
         
         mode.autoStick = mode.autoTrim = mode.autoAlpha = true;
         elevController.setPID(testGain = testGainExpo(6), 0, 0);
         break;
         
       case 4:
         // Auto alpha outer loop gain
         
         mode.autoStick = mode.autoTrim = mode.autoAlpha = true;
         autoAlphaP = testGain = testGainExpo(21);
         break;
         
       case 11:
         // Elevator stabilizer gain, mode depends on config
         
         testGain = testGainExpo(4);
         elevController.setPID(testGain, 0, 0);
         pusher.setPID(testGain, 0, 0);
         break;
         
       case 12:
         // Auto alpha outer loop gain, mode depends on config
         
         autoAlphaP = testGain = testGainExpo(15);
         break;

       case 21:
         // Elevator stabilizer ZN gain = 0... 1.5, fixed period = 0.5
         
         testGain = testGainExpo(1.5);
         elevController.setZieglerNicholsPID(testGain, 0.5);
         pusher.setZieglerNicholsPID(testGain, 0.5);
         break;
         
       case 22:
         // Elevator stabilizer ZN period = 0... 1, fixed gain = 0.9
         
         testGain = testGainExpo(1);
         elevController.setZieglerNicholsPID(0.9, testGain);
         pusher.setZieglerNicholsPID(0.9, testGain);
         break;
         
       case 31:
         // Elevator stabilizer ZN empirical period

         float Ku, Tu;
         elevController.getZieglerNicholsPID(&Ku, &Tu);
         parameter *= neutralAlpha/maxAlpha;
         testGain = testGainLinear(Tu, 0.5);
         elevController.setZieglerNicholsPID(Ku, testGain);
         pusher.setZieglerNicholsPID(Ku, testGain);
         break;
         
       case 7:
         // Max alpha
         
         maxAlpha = testGain = testGainLinear(10, 20);
         break;         
     }
  }
      
  if(!mode.autoStick)
    neutralStick = elevStick;

  alphaFilter.input(alpha);
  
  if(!mode.autoAlpha)
    neutralAlpha = clamp(alphaFilter.output(), paramRecord.alphaMin, maxAlpha);
}

void loopTask(uint32_t currentMicros)
{
  if(looping) {
    consolePrint("alpha = ");
    consolePrint(alpha*360);
    consolePrint(" IAS = ");
    consolePrint(sqrt(2*dynPressure));

/*
    consolePrint(" ppmFreq = ");
    consolePrint(ppmFreq);
    consolePrint(" aileStick = ");
    consolePrint(aileStick);
    consolePrint(" elevStick = ");
    consolePrint(elevStick);
*/    

    consolePrint(" roll = ");
    consolePrint(rollAngle, 2);
    consolePrint(" (rate = ");
    consolePrint(rollRate*360, 1);
    consolePrint(") pitch = ");
    consolePrint(pitchAngle, 2);
    consolePrint(" (rate = ");
    consolePrint(pitchRate*360, 1);
    consolePrint(")");

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
    //    consolePrint(" testGain = ");
    // consolePrint(testGain);
    consolePrint("      \r");
  }
}

const int serialBufLen = 1<<9;
char serialBuf[serialBufLen];
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
      
      if(c == '\n') {
	looping = false;
	executeCommandSeries(serialBuf, serialBufIndex);
	serialBufIndex = 0;
	controlCycleEnded = 0;

      } else if(c == '\b') {
	if(serialBufIndex > 0) {
	  consolePrint("\b \b");
	  serialBufIndex--;
	}
	
      } else {
	consolePrintf("%c", c);
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
  
  if(calibrateStart) {
    paramRecord.elevZero += elevStick;
    paramRecord.aileZero += aileStick;
    paramRecord.elevNeutral = paramRecord.aileNeutral = 0.0;
    calibrateStart = false;
  } else if(calibrateStop) {       
    paramRecord.elevNeutral = elevStick;
    paramRecord.aileNeutral = aileStick;
    calibrateStop = false;
  } else if(calibrate) {
    elevOutput = elevStick;
    aileOutput = aileStick;
  } else {
    // Elevator control
    
    targetAlpha = 0.0;

    if(mode.autoStick) {
      float targetRate = clamp(elevStick, -0.5, 0.5);

      if(mode.autoAlpha) {  
        float maxAutoAlpha = maxAlpha/square(1.1);
        
        targetAlpha = clamp(neutralAlpha + elevStick*maxAutoAlpha,
          paramRecord.alphaMin, maxAutoAlpha);
 
        targetRate = (targetAlpha - alpha) * autoAlphaP;
        
      } else
        targetRate = min(targetRate, (maxAlpha - alpha) * autoAlphaP);
      
      elevController.input(targetRate - pitchRate, controlCycle);
    } else {
      elevController.reset(elevStick, 0.0);
    }
    
    if(mode.autoStick && !mode.sensorFailSafe && !alphaFailed) {
      const float fract_c = 1.0/3;
      float strength = max(absVal(elevStick)-(1.0-fract_c), 0)/fract_c;

      elevOutput = elevController.output() + square(strength)*elevStick;
    } else
      elevOutput = elevStick;

    elevOutput = clamp(elevOutput, -1, 1);
    
    // Pusher

    pusher.input((maxAlpha - alpha)*paramRecord.o_P - pitchRate, controlCycle);

    if(!mode.sensorFailSafe && !alphaFailed)
      elevOutput = min(elevOutput, pusher.output());
  
    // Aileron
    
    float maxBank = 60.0;
    
    float targetRate = 270.0/360*aileStick;
    
    if(mode.rxFailSafe)
      maxBank = 15.0;

    else if(mode.autoTrim)
      maxBank -= 30.0*(neutralAlpha / maxAlpha);

    if(mode.sensorFailSafe)
      aileOutput = aileStick;
    
    else if(!mode.stabilizer) {
      // Simple proportional wing leveler
        
      aileOutput = (aileStick*maxBank - rollAngle) / 90;
      aileController.reset(aileOutput, targetRate - rollRate);
      
    } else {
      // Roll stabilizer enabled
      
      if(mode.wingLeveler)
        // Wing leveler enabled
        
        targetRate = clamp((aileStick*maxBank - rollAngle) / 90, -0.75, 0.75);
      else if(mode.bankLimiter) {
        // No leveling but limit bank
                
        targetRate = clamp(targetRate,
			   (-maxBank - rollAngle) / 90, (maxBank - rollAngle) / 90);
      }
      
      aileController.input(targetRate - rollRate, controlCycle);
      aileOutput = aileController.output();
    }
 
    // Brake
    
    if(!mode.sensorFailSafe && gearOutput == 1)
      brakeOutput = 0;
    else
      brakeOutput = max(-elevStick, 0);
  } 
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

    pwmOutputWrite(brakeHandle, NEUTRAL
		   + RANGE*clamp(paramRecord.brakeNeutral + 
		   paramRecord.brakeDefl*randomNum(0, 1), -1, 1));                

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

    pwmOutputWrite(brakeHandle, NEUTRAL
		   + RANGE*clamp(paramRecord.brakeNeutral + 
				 paramRecord.brakeDefl*brakeOutput, -1, 1));                        
  } 
}

void trimTask(uint32_t currentMicros)
{
  if(mode.autoTrim && absVal(rollAngle) < 30) {
    neutralAlpha += clamp((targetAlpha - neutralAlpha)/2/TRIM_HZ,
      -1.5/360/TRIM_HZ, 1.5/360/TRIM_HZ);
//    neutralAlpha = clamp(neutralAlpha, paramRecord.alphaMin, maxAlpha*0.9);
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
    HZ_TO_PERIOD(50) },
  { cacheTask,
    HZ_TO_PERIOD(4) },
  { rpmTask,
    HZ_TO_PERIOD(10) },
  { alphaLogTask,
    HZ_TO_PERIOD(45) },
  { controlLogTask,
    HZ_TO_PERIOD(15) },
  { positionLogTask,
    HZ_TO_PERIOD(2) },
  { logSaveTask,
    HZ_TO_PERIOD(2) },
  { measurementTask,
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
  consolePrintfLn("%u", hal.util->available_memory());

  // Read the non-volatile state

  readNVState();
    
  consoleNote_P(PSTR("Current model is "));
  consolePrintLn(stateRecord.model);
  
  // I2C
  
  I2c.begin();
  I2c.setSpeed(true);
  I2c.pullup(true);
  I2c.timeOut(2+EXT_EEPROM_LATENCY/1000);

  // Param record
  
  setModel(stateRecord.model);

  consoleNote_P(PSTR("MODEL "));
  consolePrintLn(stateRecord.model);
  printParams(&paramRecord);    

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
  rxInputInit(&switchInput);
  rxInputInit(&tuningKnobInput);  

#endif

  // RPM sensor int control

#ifdef rpmPin
  pinMode(rpmPin, INPUT_PULLUP);  
  rpmMeasure(stateRecord.logRPM);
#dndif

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
  
  consoleNote_P(PSTR("Initializing INS / AHRS... "));
  
  ins.init(AP_InertialSensor::COLD_START, AP_InertialSensor::RATE_100HZ);
  ahrs.init();

  consoleNoteLn_P(PSTR("  done"));
  
  // LED output

  configureOutput(&RED_LED);
  configureOutput(&GREEN_LED);
  configureOutput(&BLUE_LED);

  setPinState(&RED_LED, 1);
  setPinState(&GREEN_LED, 1);
  setPinState(&BLUE_LED, 1);
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
//
// Backup VIPER 2015/12/12

echo 0; model 0; ; 5048b_ref 2068; inner_pid 0.5100 4.6398 0.0140; outer_p 10.00; stabilizer_pid 0.4799 4.0000 0.0130; min -3.00; max 12.00; edefl 45.00; eneutral -8.00; ezero 3.33; adefl -45.00; aneutral 0.00; azero 6.00; fstep -40.00; fneutral 55.00; fneutral 55.00 -80.00; bdefl -45.00; bneutral -45.00; echo 1; store
*/
