#include "InputOutput.h"

#define RC_OUTPUT_MIN_PULSEWIDTH 400
#define RC_OUTPUT_MAX_PULSEWIDTH 2100

typedef enum { COMnA = 0, COMnB = 1, COMnC = 2 } PWM_Ch_t;

struct HWTimer {
  volatile uint8_t *TCCRA, *TCCRB;
  volatile uint16_t *ICR;
  volatile uint16_t *OCR[3]; // 0... 2 = A ... C
};

struct PWMOutput {
  struct PinDescriptor pin;
  const struct HWTimer *timer;
  PWM_Ch_t pwmCh; // COMnA / COMnB / COMnC
};

void pwmTimerInit(const struct HWTimer *timer[], int num);
void pwmEnable(const struct PWMOutput *output);
void pwmDisable(const struct PWMOutput *output);
void pwmOutputInit(const struct PWMOutput *output);
void pwmOutputInitList(const struct PWMOutput output[], int num);
void pwmOutputWrite(const struct PWMOutput *output, uint16_t value);


