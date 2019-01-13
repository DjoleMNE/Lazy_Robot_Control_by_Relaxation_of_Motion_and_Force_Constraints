/*
 * Copyright (c) 2014-2018 LAAS/CNRS
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 *                                           Anthony Mallet on Tue Sep 23 2014
 */
#include "acheader.h"

#include <avr/pgmspace.h>
#include <util/atomic.h>
#include <util/delay_basic.h>

#include "common/tk3-mikrokopter.h"
#include "mkbl.h"

#define PWM_FREQ	16		/* 8/16/32 kHz */


/* --- local data ---------------------------------------------------------- */

#define MAX_PWM		(8192/PWM_FREQ -1)
#define kDC_TO_PWM(x)	((x) / (1024/(MAX_PWM+1)))
#define PWM_TO_kDC(x)	((x) * (1024/(MAX_PWM+1)))

static int8_t phase_incr;		/* motor direction, set once at init */
static int8_t phase;			/* brushless phase */
static uint8_t start_count;		/* startup attempts */
static uint32_t start_delay;		/* open loop startup delay */
static uint8_t good_zc;			/* zero crossings during startup */
static uint16_t max_kdc;		/* saturation for current limitation */

static tk3_time wdog;			/* date of last velocity command */

struct tk3_motor tk3_motor;

static void	tk3_phase_step(uint8_t p);
static void	tk3_phase_step_schedule(uint8_t p, uint8_t sched);
static void	tk3_motor_timeout(void);
static void	tk3_motor_control(void);


/* --- MOSFET control ------------------------------------------------------ */

/* hardware config (port, pin and back emf channel) */
#define MOSFET_AP_PORT	B
#define MOSFET_AP_PIN	3
#define MOSFET_AN_PORT	D
#define MOSFET_AN_PIN	3
#define MOSFET_A_BEMF	0

#define MOSFET_BP_PORT	B
#define MOSFET_BP_PIN	2
#define MOSFET_BN_PORT	D
#define MOSFET_BN_PIN	4
#define MOSFET_B_BEMF	1

#define MOSFET_CP_PORT	D
#define MOSFET_CP_PIN	2
#define MOSFET_CN_PORT	D
#define MOSFET_CN_PIN	5
#define MOSFET_C_BEMF	2


/* P MOSFETs are switched on by not driving the GPIO (input pin with no
 * pullup). The line is then driven by the PWM signal of Timer1A. */
#define MOSFET_P_ON(port, pin)	\
  do { PORT ## port &= ~_BV(pin); DDR ## port &= ~_BV(pin); } while(0)

/* P MOSFETs are switched off by driving the GPIO to 0. */
#define MOSFET_P_OFF(port, pin)	\
  do { PORT ## port &= ~_BV(pin); DDR ## port |= _BV(pin); } while(0)

/* N MOSFETs are switched on by driving the GPIO to 1. */
#define MOSFET_N_ON(port, pin)	\
  do { DDR ## port |= _BV(pin); PORT ## port |= _BV(pin); } while(0)

/* N MOSFETs are switched off by driving the GPIO to 0. */
#define MOSFET_N_OFF(port, pin)	\
  do { DDR ## port |= _BV(pin); PORT ## port &= ~_BV(pin); } while(0)

/* individual MOSFETs control */
#define MOSFET_X(X, port, pin) X(port, pin)
#define MOSFET(X, kind, state)                                  \
  MOSFET_X(MOSFET_ ## kind ## _ ## state,                       \
           MOSFET_ ## X ## kind ## _PORT,                       \
           MOSFET_ ## X ## kind ## _PIN)


/* phase commutation */
#define PHASE_HI(X)	do { MOSFET(X, N, OFF); MOSFET(X, P, ON); } while(0)
#define PHASE_OFF(X)	do { MOSFET(X, N, OFF); MOSFET(X, P, OFF); } while(0)
#define PHASE_LOW(X)	do { MOSFET(X, P, OFF); MOSFET(X, N, ON); } while(0)

/* back emf detection */
#define PHASE_BEMF(X) \
  do { ADMUX = (ADMUX & 0xf0) | MOSFET_ ## X ## _BEMF; } while(0)

#define PHASE_BEMF_RISE(X) \
  do { PHASE_BEMF(X); ACSR = (1 << ACIS1); } while(0)
#define PHASE_BEMF_FALL(X) \
  do { PHASE_BEMF(X); ACSR = (1 << ACIS1)|(1 << ACIS0); } while(0)


/* --- tk3_motor_init ------------------------------------------------------ */

int
tk3_motor_init(enum mkbldir direction)
{
  /* switch phases off*/
  PHASE_OFF(A);
  PHASE_OFF(B);
  PHASE_OFF(C);
  OCR1A = 0;

  /* intialize constants */
  switch(direction) {
    case MKBL_CLOCKWISE:	phase_incr = 1; break;
    case MKBL_ANTICLOCKWISE:	phase_incr = -1; break;
    default: return -1;
  }
  max_kdc = 1023;
  tk3_motor.max_period = 0xffff / settings.motor_period_mul;
  tk3_motor.sign = 1;

  /* configure analog comparator */
  DIDR1 |= (1 << AIN0D);

  /* configure timer1 for fast PWM on PB1 */
  TIMSK1 = 0;
#if PWM_FREQ == 8
  TCCR1A = (1 << WGM11) | (1 << WGM10);
#elif PWM_FREQ == 16
  TCCR1A = (1 << WGM11) | (0 << WGM10);
#elif PWM_FREQ == 32
  TCCR1A = (0 << WGM11) | (1 << WGM10);
#else
# error "wrong PWM_FREQ"
#endif
  TCCR1A |= (1 << COM1A1);
  TCCR1B = (1 << CS10) | (1 << WGM12);
  PORTB &= ~(1 << PB1);
  DDRB |= (1 << DDB1);

  tk3_motor_stop();
  tk3_motor.emerg = 0;
  return 0;
}


/* --- tk3_motor_test ------------------------------------------------------ */

/* test MOSFETs for short circuits ... */

int
tk3_motor_test()
{
  uint16_t noise;
  uint8_t i;

  /* switch phases off, set 100% PWM */
  PHASE_OFF(A); PHASE_OFF(B); PHASE_OFF(C);
  OCR1A = MAX_PWM;

  /* measure background noise */
  tk3_sensors.peak_current = 0;
  for(i=0; i<100; i++)
    tk3_sensor_update(TK3_SENSOR_CURRENT);
  noise = tk3_sensors.peak_current;

  /* switch on individual MOSTFETs and check no current is flowing */
  for(i=0; i<60; i++) {
    switch(i % 6) {
      case 0: PHASE_HI(A); break;
      case 1: PHASE_LOW(A); break;
      case 2: PHASE_HI(B); break;
      case 3: PHASE_LOW(B); break;
      case 4: PHASE_HI(C); break;
      case 5: PHASE_LOW(C); break;
    }

    /* tiny delay ~ 50µs */
    tk3_sensor_update(TK3_SENSOR_CURRENT);
    tk3_sensor_update(TK3_SENSOR_CURRENT);
    PHASE_OFF(A); PHASE_OFF(B); PHASE_OFF(C);

    if (tk3_sensors.peak_current > 2 * noise) {
      tk3_motor_stop();
      red_led_abort();
    }
  }

  tk3_motor_stop();
  return 0;
}


/* --- tk3_motor_start ----------------------------------------------------- */

static void	tk3_motor_start_cb(void);

void
tk3_motor_start(void)
{
  tk3_clock_callback(0, NULL);

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    OCR1A = 0;
    tk3_motor.align = 1;
    tk3_motor.starting = 1;
    tk3_motor.spinning = 0;
    tk3_motor.servo = 0;
    tk3_motor.reverse = 0;
    tk3_motor.emerg = 0;
    tk3_motor.period = 0xffff;
    tk3_motor.pwm = 0;
  }
  start_count = 0;
  start_delay = 0;

  /* open loop activation sequence */
  tk3_motor_start_cb();
}


static void
tk3_motor_start_cb(void)
{
  tk3_time t = tk3_clock_gettime();

  /* open-loop acceleration */
  tk3_phase_step(phase);

  /* over-current protection */
  NONATOMIC_BLOCK(NONATOMIC_RESTORESTATE) {
    tk3_sensor_update(TK3_SENSOR_CURRENT);
  }
  if (tk3_sensors.peak_current > settings.mcurrent) {
    tk3_motor_stop();
    tk3_motor.emerg = 1;
    return;
  }

  /* PWM ramp during first 25% of align phase */
  if (tk3_motor.align) {
    uint8_t pwm = 0;

    NONATOMIC_BLOCK(NONATOMIC_RESTORESTATE) {
      start_delay += 10;
      if (start_delay <= settings.sdelay/4) {
        pwm = MAX_PWM * settings.spwm / 100 * 4 * start_delay/settings.sdelay;
      } else if (start_delay > settings.sdelay) {
        pwm = MAX_PWM * settings.spwm / 100;

        start_delay = settings.speriod;
        tk3_motor.align = 0;

        tk3_phase_step_schedule(phase, 0);
      } else
        pwm = MAX_PWM * settings.spwm / 100;
    }
    OCR1A = pwm;
    tk3_motor.target = OCR1A;
    tk3_motor.pwm = PWM_TO_kDC(pwm);

    tk3_clock_callback(t + 10000, tk3_motor_start_cb);
    return;
  }

  good_zc = 0;
  tk3_motor.stamp = t;
  tk3_motor.period = start_delay;
  start_delay -= start_delay/32;
  if (start_delay < 768) {
    /* failure... */
    if (++start_count > 4) {
      tk3_motor_stop();
      tk3_motor.emerg = 1;
      return;
    }

    tk3_motor.align = 1;
    tk3_motor.period = 0xffff;
    start_delay = 0;
    ACSR &= ~(1<<ACIE);
    tk3_clock_callback(t + 10000, tk3_motor_start_cb);
    return;
  }

  tk3_phase_step_schedule(phase, 1);
  tk3_clock_callback(t + start_delay, tk3_motor_start_cb);
}


/* --- tk3_motor_stop ------------------------------------------------------ */

void
tk3_motor_stop(void)
{
  tk3_clock_callback(0, NULL);
  ACSR &= ~(1<<ACIE);
  ADCSRB &= ~(1 << ACME);
  OCR1A = 0;
  PHASE_OFF(A); PHASE_OFF(B); PHASE_OFF(C);

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    tk3_motor.starting = 0;
    tk3_motor.spinning = 0;
    tk3_motor.servo = 0;
    tk3_motor.braking = 0;
    tk3_motor.stamp = tk3_clock_gettime();
    if (tk3_motor.sign < 0) {
      tk3_motor.sign = 1;
      phase_incr = -phase_incr;
    }
    tk3_motor.period = 0xffff;
    tk3_motor.target = 0xffff;
    tk3_motor.pwm = 0;
    tk3_motor.smc_bias = 0;
    tk3_motor.smc_gain = 0;
    tk3_motor.smc_err = 0;
  }
}


/* --- tk3_motor_timeout --------------------------------------------------- */

static void
tk3_motor_timeout(void)
{
  tk3_motor_stop();
  tk3_motor.emerg = 1;
}


/* --- tk3_motor_tone ------------------------------------------------------ */

int
tk3_motor_tone(uint16_t freq)
{
  tk3_time t, delay;
  uint8_t i;

  tk3_motor_stop();

  delay = 1000000/freq;
  t = tk3_clock_gettime();

  /* heuristic PWM based on battery level */
  tk3_sensor_update(TK3_SENSOR_BATTERY);
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    if (tk3_sensors.battery < 6000)
      OCR1A = kDC_TO_PWM(300);
    else if (tk3_sensors.battery < 13000)
      OCR1A = kDC_TO_PWM(100);
    else
      OCR1A = kDC_TO_PWM(40);
  }

  for(i = 0; i < 127; i++) {
    tk3_phase_step(i % 2);
    t += delay;
    tk3_clock_delay(t);
  }

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    OCR1A = 0;
  }
  return 0;
}


/* --- tk3_motor_pwm ------------------------------------------------------- */

void
tk3_motor_pwm(int8_t sign, uint16_t u)
{
  if (!tk3_motor.spinning) return;

  if (sign != tk3_motor.sign) {
    if (tk3_motor.reverse) return;

    /* brake */
    tk3_motor_period(sign, 0);
    return;
  } else
    tk3_motor.reverse = 0;

  if (u > max_kdc) u = max_kdc;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    if (tk3_motor.servo) {
      tk3_motor.servo = 0;
      tk3_motor.smc_bias = 0;
      tk3_motor.smc_gain = 0;
      tk3_motor.smc_err = 0;
    }
    wdog = tk3_motor.stamp;

    tk3_motor.target = kDC_TO_PWM(u);
  }
}


/* --- tk3_motor_period ---------------------------------------------------- */

void
tk3_motor_period(int8_t sign, uint16_t u)
{
  if (!tk3_motor.spinning) return;

  if (sign != tk3_motor.sign) {
    if (tk3_motor.reverse) return;

    /* brake */
    u = 0xffff;
    OCR1A = 0;
    tk3_motor.smc_bias = 0;
    tk3_motor.smc_err = 256;

    tk3_motor.reverse = 1;
  } else {
    tk3_motor.reverse = 0;
    u /= settings.motor_period_mul;
  }

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    if (!tk3_motor.servo) {
      tk3_motor.servo = 1;
      tk3_motor.smc_bias = PWM_TO_kDC(OCR1A);
      tk3_motor.smc_gain = 1;
      tk3_motor.smc_err = 0;
    }
    wdog = tk3_motor.stamp;

    tk3_motor.target = u;
  }
}


/* --- tk3_motor_control --------------------------------------------------- */

static void
tk3_motor_control(void)
{
  int16_t u;
  uint16_t t, p;

  /* velocity / pwm target */
  if (tk3_motor.servo)
    t = tk3_motor.target;
  else {
    if (tk3_motor.period < tk3_motor.max_period) {
      OCR1A = tk3_motor.target;
      return;
    }

    t = tk3_motor.max_period + 128;
  }

  p = tk3_motor.period;

  /* error sign */
  if (t < p)
    tk3_motor.smc_err = (3 * tk3_motor.smc_err - 256) / 4;
  else
    tk3_motor.smc_err = (3 * tk3_motor.smc_err + 256) / 4;

  /* gain adaptation */
  if (tk3_motor.smc_err < -192 || tk3_motor.smc_err > 192 /* 75% */) {
    if (tk3_motor.smc_gain < tk3_motor.pwm/2) tk3_motor.smc_gain += 4;
  } else if (tk3_motor.smc_err < -128 || tk3_motor.smc_err > 128 /* 50% */) {
    if (tk3_motor.smc_gain < tk3_motor.pwm/2) tk3_motor.smc_gain += 2;
  } else {
    tk3_motor.smc_gain -= 2;
    if (tk3_motor.smc_gain < 1) tk3_motor.smc_gain = 1;
  }

  /* bias adaptation */
  if (tk3_motor.smc_err < -253) {
    tk3_motor.smc_bias += 6;
    if (tk3_motor.smc_bias > max_kdc) tk3_motor.smc_bias = max_kdc;
  } else if (tk3_motor.smc_err < -192) {
    if (tk3_motor.smc_bias < max_kdc) tk3_motor.smc_bias += 1;
  } else if (tk3_motor.smc_err > 253) {
    tk3_motor.smc_bias -= 6;
    if (tk3_motor.smc_bias < 1) tk3_motor.smc_bias = 1;
  } else if (tk3_motor.smc_err > 192) {
    if (tk3_motor.smc_bias > 1) tk3_motor.smc_bias -= 1;
  }

  /* command */
  if (t < p) {
    u = tk3_motor.smc_bias + tk3_motor.smc_gain;
    if (u > max_kdc) u = max_kdc;
  } else {
    u = tk3_motor.smc_bias - tk3_motor.smc_gain;
    if (u < 0) u = 0;
  }

  if (tk3_motor.servo)
    OCR1A = kDC_TO_PWM(u);
  else if (kDC_TO_PWM(u) > tk3_motor.target)
    OCR1A = kDC_TO_PWM(u);
  else
    OCR1A = tk3_motor.target;
}


/* --- tk3_phase_step ------------------------------------------------------ */

/*
 * Clockwise direction:
 *       | 0 | 1 | 2 | 3 | 4 | 5 |
 * ------|---|---|---|---|---|---|
 *   Hi  |===|   |   |   |   |===|
 * A Off |   |===|   |   |===|   |
 *   Low |   |   |===|===|   |   |
 * ------|---|---|---|---|---|---|
 *   Hi  |   |===|===|   |   |   |
 * B Off |===|   |   |===|   |   |
 *   Low |   |   |   |   |===|===|
 * ------|---|---|---|---|---|---|
 *   Hi  |   |   |   |===|===|   |
 * C Off |   |   |===|   |   |===|
 *   Low |===|===|   |   |   |   |
 * ------|---|---|---|---|---|---|
 */
static void
tk3_phase_step(uint8_t p)
{
  ACSR &= ~(1<<ACIE);
  ADCSRB &= ~(1 << ACME);

  if (OCR1A == 0 &&
      tk3_motor.period < tk3_motor.max_period &&
      (tk3_motor.smc_err > 192 || !tk3_motor.servo) &&
      tk3_sensors.battery < 23000)
    tk3_motor.braking = 1;
  else if (tk3_motor.reverse)
    tk3_motor.braking = 1;
  else
    tk3_motor.braking = 0;

  switch(p) {
    case 0: case 3: PHASE_OFF(B); break;
    case 1: case 4: PHASE_OFF(A); break;
    case 2: case 5: PHASE_OFF(C); break;
  }

  switch(p) {
    case 0: case 1: PHASE_LOW(C); break;
    case 2: case 3: PHASE_LOW(A); break;
    case 4: case 5: PHASE_LOW(B); break;
  }

  if (tk3_motor.braking) {
    /* dynamic brake */
    switch(p) {
      case 0: case 5: PHASE_LOW(A); break;
      case 1: case 2: PHASE_LOW(B); break;
      case 3: case 4: PHASE_LOW(C); break;
    }
  } else {
    switch(p) {
      case 0: case 5: PHASE_HI(A); break;
      case 1: case 2: PHASE_HI(B); break;
      case 3: case 4: PHASE_HI(C); break;
    }
  }
}


/* --- tk3_phase_step_schedule --------------------------------------------- */

/* configure next analog comparator event */
static void
tk3_phase_step_schedule(uint8_t p, uint8_t sched)
{
  if (phase_incr > 0) {
    switch(p) {
      case 0: PHASE_BEMF_RISE(B); break;
      case 1: PHASE_BEMF_FALL(A); break;
      case 2: PHASE_BEMF_RISE(C); break;
      case 3: PHASE_BEMF_FALL(B); break;
      case 4: PHASE_BEMF_RISE(A); break;
      case 5: PHASE_BEMF_FALL(C); break;

      default: /* error, should not happen */
        return;
    }
  } else {
    switch(p) {
      case 0: PHASE_BEMF_FALL(B); break;
      case 1: PHASE_BEMF_RISE(A); break;
      case 2: PHASE_BEMF_FALL(C); break;
      case 3: PHASE_BEMF_RISE(B); break;
      case 4: PHASE_BEMF_FALL(A); break;
      case 5: PHASE_BEMF_RISE(C); break;

      default: /* error, should not happen */
        return;
    }
  }

  phase += phase_incr;
  if (phase > 5) phase = 0; else if (phase < 0) phase = 5;

  /* enable analog comparator interrupts */
  ADCSRB |= (1 << ACME);
  if (sched) ACSR |= (1 << ACIE) | (1 << ACI);
}


/* --- ANALOG_COMP_vect ---------------------------------------------------- */

ISR(ANALOG_COMP_vect)
{
  tk3_time time;
  uint16_t period, delay;

  /* spinning period */
  time = tk3_clock_gettime();
  delay = time - tk3_motor.stamp;
  period = tk3_motor.period;

  /* reject early callbacks (noise) */
  if (delay < 128) return;
  else if (tk3_motor.starting && delay < 768) {
    ACSR &= ~(1 << ACIE);
    return;
  }
  tk3_motor.stamp = time;

  period = (3 * period + delay)/4;
  tk3_motor.period = period;

  /* phase commutation */
  tk3_phase_step(phase);

  /* reverse */
  if (tk3_motor.reverse && period > 2048) {
    tk3_motor.sign = -tk3_motor.sign;
    phase_incr = -phase_incr;
    tk3_motor_start();
    return;
  }

  /* velocity/pwm control */
  if (tk3_motor.spinning)
    tk3_motor_control();

  /* re-enable interrupts temporarily + make sure to not reenter here
   * this is to allow twi communication to take place during velocity control
   * and sensor reading, which can last for ~50µs (about two TWI bytes). */
  NONATOMIC_BLOCK(NONATOMIC_FORCEOFF) {
    /* sensors reading */
    tk3_sensor_update(-1);

    if (tk3_sensors.current > 40000) {
      tk3_motor_stop();
      flash_red_led_abort();
    } else if (tk3_sensors.current > settings.mcurrent) {
      if (max_kdc > 16) {
        max_kdc -= 16;
        if (OCR1A > kDC_TO_PWM(max_kdc)) OCR1A = kDC_TO_PWM(max_kdc);
      }
    } else if (max_kdc < 1023)
      max_kdc++;
  }
  tk3_motor.pwm = PWM_TO_kDC(OCR1A);

  /* watchdog */
  if (tk3_motor.starting) {
    if (++good_zc < 84) {
      tk3_clock_callback(time + 2 * period, tk3_motor_start_cb);
    } else {
      tk3_motor.spinning = 1;
      tk3_motor.starting = 0;
    }
  }
  if (tk3_motor.spinning) {
    if (tk3_motor.braking)
      /* sometimes, it misses ZC. But we're safe since pwm is 0. */
      tk3_clock_callback(time + 2 * period, ANALOG_COMP_vect);
    else
      tk3_clock_callback(time + 4 * period, tk3_motor_timeout);

    if (time - wdog > settings.wdog)
      tk3_motor_period(tk3_motor.sign, 0xffff);
  }

  /* schedule next event */
  tk3_phase_step_schedule(phase, 1);
}
