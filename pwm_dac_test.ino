/**
 * PWM Audio for Arduino101 (Intel Curie)
 *
 * 20161005 Robert Poor <rdpoor@gmail.com> 
 * First edit.
 *
 * 
 * https://github.com/01org/corelibs-arduino101/blob/master/variants/arduino_101/variant.cpp
 *
 * Overview:
 *
 * This module uses two hardware timer/counters.  The first timer is configured
 * to generate a pulse-width modulated output (PWM) signal.  The second timer is
 * configured as a sample clock: it rolls over and generates an interrupt once
 * every sample period.  The interrupt routine fetches the next audio sample and
 * adjusts the duty cycle of the PWM timer.
 * 
 * We use ARC_V2_TRM1 for the sample clock and one of the Quark PWM timers for
 * PWM.
 *
 * This pencil sketch plays 8KHz 8-bit precomputed samples from in-memory array
 * (sounddata.h).  A more comprehensive version would fetch data from an SD
 * card.
 * 
 */

#include <Arduino.h>
#include <aux_regs.h>
#include <interrupt.h>
#include <conf.h>
#include <scss_registers.h>

// #include "sounddata.h"
#include "great_horned.h"

const int SAMPLE_RATE = 8192;
const int SAMPLE_CLOCK_PIN = 3;     // make the sample clock scope'able
const int PWM_PIN = 5;
const int DEBUG06_PIN = 6;
const int DEBUG07_PIN = 7;
const int DEBUG08_PIN = 8;

typedef uint8_t sample_t;

// use ARC_V2_TMR1 for the sample clock
#define SCLOCK_TMR_IRQ ARCV2_IRQ_TIMER1
#define SCLOCK_TMR_RATE ARCV2_TIMER1_CLOCK_FREQ
#define SCLOCK_TMR_CONTROL ARC_V2_TMR1_CONTROL
#define SCLOCK_TMR_LIMIT ARC_V2_TMR1_LIMIT
#define SCLOCK_TMR_COUNT ARC_V2_TMR1_COUNT

void setup_io_pins() {
  pinMode(SAMPLE_CLOCK_PIN, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(DEBUG06_PIN, OUTPUT);
  pinMode(DEBUG07_PIN, OUTPUT);
  pinMode(DEBUG08_PIN, OUTPUT);
}

// ================================================================
// ================================================================

uint32_t g_pwmChan;

// update the PWM with a sample.  Sample is (today) assumed to be unsigned, excess 128
void load_pwm(sample_t sample) {
  sample_t lo_count = sample;
  sample_t hi_count = 256 - sample;
  uint32_t offset;

  offset = (g_pwmChan * QRK_PWM_N_LCNT2_LEN) + QRK_PWM_N_LOAD_COUNT2;
  MMIO_REG_VAL(QRK_PWM_BASE_ADDR + offset) = hi_count;
  offset = (g_pwmChan * QRK_PWM_N_REGS_LEN) + QRK_PWM_N_LOAD_COUNT1;
  MMIO_REG_VAL(QRK_PWM_BASE_ADDR + offset) = lo_count;
}
  
void setup_pwm(int pin) {
  PinDescription *p = &g_APinDescription[pin];
  uint32_t offset;
  
  if (p->ulPwmChan == INVALID) {
    // Serial.println("Invalid pin number for PWM.  Must be one of 3, 5, 6, 9");
    return;
  };
  g_pwmChan = p->ulPwmChan;

  // preload to 50%
  load_pwm(128);

  // configure for PWM mode, no interrupts, enabled.
  offset = (g_pwmChan * QRK_PWM_N_REGS_LEN) + QRK_PWM_N_CONTROL;
  MMIO_REG_VAL(QRK_PWM_BASE_ADDR + offset) =
    QRK_PWM_CONTROL_PWM_OUT | QRK_PWM_CONTROL_INT_MASK | QRK_PWM_CONTROL_MODE_PERIODIC | QRK_PWM_CONTROL_ENABLE;

  SET_PIN_PULLUP(p->ulSocPin, 0);
  SET_PIN_MODE(p->ulSocPin, PWM_MUX_MODE);
  p->ulPinMode = PWM_MUX_MODE;
}

// ================================================================
// ================================================================

int sample_index = 0;

sample_t getNextSample() {
  if (sample_index == sounddata_length) {
    sample_index = 0;
  }
  return sounddata_data[sample_index++];
}

void sclock_isr() {
  // clear the interrupt (by writing 0 to IP bit of the control register)
  aux_reg_write(SCLOCK_TMR_CONTROL, ARC_V2_TMR_CTRL_NH | ARC_V2_TMR_CTRL_IE);
  sample_t sample = getNextSample();
  load_pwm(sample);
  digitalWrite(SAMPLE_CLOCK_PIN, !digitalRead(SAMPLE_CLOCK_PIN));
}

void setup_sclock() {
  uint32_t count = SCLOCK_TMR_RATE / SAMPLE_RATE;
  // Serial.print("count = "); Serial.println(count);
  aux_reg_write(SCLOCK_TMR_CONTROL, 0);  // disable interrupts
  aux_reg_write(SCLOCK_TMR_LIMIT, count);
  aux_reg_write(SCLOCK_TMR_CONTROL, ARC_V2_TMR_CTRL_NH | ARC_V2_TMR_CTRL_IE);
  // reset count to zero (becuase if it's larger than count, it will first count to ffffffff
  // before rolling over)
  aux_reg_write(SCLOCK_TMR_COUNT, 0);
}

void enable_sclock_interrupts() {
  /* connect specified routine/parameter to the timer 1 interrupt vector */
  interrupt_connect(SCLOCK_TMR_IRQ, sclock_isr);
  /* Everything has been configured. It is now safe to enable the interrupt */
  interrupt_enable(SCLOCK_TMR_IRQ);
}

// ================================================================
// ================================================================
// entry point

void setup() {
  // Serial.begin(9600);
  // while(!Serial)  ;             // wait for serial
  setup_io_pins();
  setup_sclock();
  setup_pwm(PWM_PIN);
  enable_sclock_interrupts();
}

void loop() {
  // Serial.println(aux_reg_read(SCLOCK_TMR_COUNT), HEX);
  digitalWrite(DEBUG08_PIN, !digitalRead(DEBUG08_PIN));
  delay( 1000 );
}


