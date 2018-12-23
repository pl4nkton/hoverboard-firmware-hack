
#include "stm32f1xx_hal.h"
#include "defines.h"
#include "setup.h"
#include "config.h"


volatile int posl = 0;
volatile int posr = 0;
volatile int pwml = 0;
volatile int pwmr = 0;
volatile int weakl = 0;
volatile int weakr = 0;


extern volatile adc_buf_t adc_buffer;

extern volatile uint32_t timeout;

uint32_t buzzerFreq = 0;
uint32_t buzzerPattern = 0;

uint8_t enable = 0;

static const int pwm_res = 64000000 / 2 / PWM_FREQ; // = 2000

static const uint8_t hall_to_pos[8] = {
  0,
  0,
  2,
  1,
  4,
  5,
  3,
  0,
};

static const GPIO_TypeDef *port_l[] = {
  LEFT_HALL_U_PORT,
  LEFT_HALL_V_PORT,
  LEFT_HALL_W_PORT
};

static const unsigned int pins_l[] = {
  LEFT_HALL_U_PIN,
  LEFT_HALL_V_PIN,
  LEFT_HALL_W_PIN
};

static const GPIO_TypeDef *port_r[] = {
  RIGHT_HALL_U_PORT,
  RIGHT_HALL_V_PORT,
  RIGHT_HALL_W_PORT
};

static const unsigned int pins_r[] = {
  RIGHT_HALL_U_PIN,
  RIGHT_HALL_V_PIN,
  RIGHT_HALL_W_PIN
};


//determine next position based on hall sensors
static inline void getPos(volatile int *pos, const GPIO_TypeDef *port[], const unsigned int *pin) {
  int pos_i;

  uint8_t hall_u = !(port[0]->IDR & pin[0]);
  uint8_t hall_v = !(port[1]->IDR & pin[1]);
  uint8_t hall_w = !(port[2]->IDR & pin[2]);

  uint8_t hall = hall_u * 1 + hall_v * 2 + hall_w * 4;
  pos_i  = hall_to_pos[hall];
  pos_i += 2;
  pos_i %= 6;
  *pos = pos_i;
}


static inline void blockPWM(int pwm, int pos, int *u, int *v, int *w) {
  switch (pos) {
    case 0:
      *u = 0;
      *v = pwm;
      *w = -pwm;
      break;
    case 1:
      *u = -pwm;
      *v = pwm;
      *w = 0;
      break;
    case 2:
      *u = -pwm;
      *v = 0;
      *w = pwm;
      break;
    case 3:
      *u = 0;
      *v = -pwm;
      *w = pwm;
      break;
    case 4:
      *u = pwm;
      *v = -pwm;
      *w = 0;
      break;
    case 5:
      *u = pwm;
      *v = 0;
      *w = -pwm;
      break;
    default:
      *u = 0;
      *v = 0;
      *w = 0;
  }
}

inline void blockPhaseCurrent(int pos, int u, int v, int *q) {
  switch(pos) {
    case 0:
      *q = u - v;
      // *u = 0;
      // *v = pwm;
      // *w = -pwm;
      break;
    case 1:
      *q = u;
      // *u = -pwm;
      // *v = pwm;
      // *w = 0;
      break;
    case 2:
      *q = u;
      // *u = -pwm;
      // *v = 0;
      // *w = pwm;
      break;
    case 3:
      *q = v;
      // *u = 0;
      // *v = -pwm;
      // *w = pwm;
      break;
    case 4:
      *q = v;
      // *u = pwm;
      // *v = -pwm;
      // *w = 0;
      break;
    case 5:
      *q = -(u - v);
      // *u = pwm;
      // *v = 0;
      // *w = -pwm;
      break;
    default:
      *q = 0;
      // *u = 0;
      // *v = 0;
      // *w = 0;
  }
}

uint32_t buzzerTimer        = 0;
//disable PWM when current limit is reached (current chopping)
static inline void chopCurrent(int cur, TIM_TypeDef *tim) {
  //disable PWM when current limit is reached (current chopping)
  if (ABS((cur) * MOTOR_AMP_CONV_DC_AMP) > DC_CUR_LIMIT || timeout > TIMEOUT || enable == 0) {
    tim->BDTR &= ~TIM_BDTR_MOE;
  } else {
    tim->BDTR |= TIM_BDTR_MOE;
  }
}


int offsetcount = 0;
int offsetrl1   = 2000;
int offsetrl2   = 2000;
int offsetrr1   = 2000;
int offsetrr2   = 2000;
int offsetdcl   = 2000;
int offsetdcr   = 2000;

float batteryVoltage = BAT_NUMBER_OF_CELLS * 4.0;

int curl = 0;
// int errorl = 0;
// int kp = 5;
// volatile int cmdl = 0;

int last_pos = 0;
int timer = 0;
const int max_time = PWM_FREQ / 10;
volatile int vel = 0;

//scan 8 channels with 2ADCs @ 20 clk cycles per sample
//meaning ~80 ADC clock cycles @ 8MHz until new DMA interrupt =~ 100KHz
//=640 cpu cycles
void DMA1_Channel1_IRQHandler() {
  DMA1->IFCR = DMA_IFCR_CTCIF1;
  // HAL_GPIO_WritePin(LED_PORT, LED_PIN, 1);

  if (offsetcount < 1000) {  // calibrate ADC offsets
    offsetcount++;
    offsetrl1 = (adc_buffer.rl1 + offsetrl1) / 2;
    offsetrl2 = (adc_buffer.rl2 + offsetrl2) / 2;
    offsetrr1 = (adc_buffer.rr1 + offsetrr1) / 2;
    offsetrr2 = (adc_buffer.rr2 + offsetrr2) / 2;
    offsetdcl = (adc_buffer.dcl + offsetdcl) / 2;
    offsetdcr = (adc_buffer.dcr + offsetdcr) / 2;
    return;
  }

  if (buzzerTimer % 1000 == 0) {  // because you get float rounding errors if it would run every time
    batteryVoltage = batteryVoltage * 0.99 + ((float)adc_buffer.batt1 * ((float)BAT_CALIB_REAL_VOLTAGE / (float)BAT_CALIB_ADC)) * 0.01;
  }


  chopCurrent(adc_buffer.dcl - offsetdcl, LEFT_TIM);
  chopCurrent(adc_buffer.dcr - offsetdcr, RIGHT_TIM);

  blockPhaseCurrent(posl, adc_buffer.rl1 - offsetrl1, adc_buffer.rl2 - offsetrl2, &curl);
  getPos(&posl, port_l, pins_l);
  getPos(&posr, port_r, pins_r);

  //setScopeChannel(2, (adc_buffer.rl1 - offsetrl1) / 8);
  //setScopeChannel(3, (adc_buffer.rl2 - offsetrl2) / 8);


  // uint8_t buzz(uint16_t *notes, uint32_t len){
    // static uint32_t counter = 0;
    // static uint32_t timer = 0;
    // if(len == 0){
        // return(0);
    // }
    
    // struct {
        // uint16_t freq : 4;
        // uint16_t volume : 4;
        // uint16_t time : 8;
    // } note = notes[counter];
    
    // if(timer / 500 == note.time){
        // timer = 0;
        // counter++;
    // }
    
    // if(counter == len){
        // counter = 0;
    // }

    // timer++;
    // return(note.freq);
  // }


  //create square wave for buzzer
  buzzerTimer++;
  if (buzzerFreq != 0 && (buzzerTimer / 5000) % (buzzerPattern + 1) == 0) {
    if (buzzerTimer % buzzerFreq == 0) {
      HAL_GPIO_TogglePin(BUZZER_PORT, BUZZER_PIN);
    }
  } else {
      HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, 0);
  }

  int ul, vl, wl;
  int ur, vr, wr;

  //update PWM channels based on position
  blockPWM(pwml, posl, &ul, &vl, &wl);
  blockPWM(pwmr, posr, &ur, &vr, &wr);

  int weakul, weakvl, weakwl;
  if (pwml > 0) {
    blockPWM(weakl, (posl+5) % 6, &weakul, &weakvl, &weakwl);
  } else {
    blockPWM(-weakl, (posl+1) % 6, &weakul, &weakvl, &weakwl);
  }
  ul += weakul;
  vl += weakvl;
  wl += weakwl;

  int weakur, weakvr, weakwr;
  if (pwmr > 0) {
    blockPWM(weakr, (posr+5) % 6, &weakur, &weakvr, &weakwr);
  } else {
    blockPWM(-weakr, (posr+1) % 6, &weakur, &weakvr, &weakwr);
  }
  ur += weakur;
  vr += weakvr;
  wr += weakwr;

  LEFT_TIM->LEFT_TIM_U = CLAMP(ul + pwm_res / 2, 10, pwm_res-10);
  LEFT_TIM->LEFT_TIM_V = CLAMP(vl + pwm_res / 2, 10, pwm_res-10);
  LEFT_TIM->LEFT_TIM_W = CLAMP(wl + pwm_res / 2, 10, pwm_res-10);

  RIGHT_TIM->RIGHT_TIM_U = CLAMP(ur + pwm_res / 2, 10, pwm_res-10);
  RIGHT_TIM->RIGHT_TIM_V = CLAMP(vr + pwm_res / 2, 10, pwm_res-10);
  RIGHT_TIM->RIGHT_TIM_W = CLAMP(wr + pwm_res / 2, 10, pwm_res-10);
}
