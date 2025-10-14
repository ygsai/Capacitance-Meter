#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/io.h>

#include "lcd.h"

/* Hardware IO abstraction macros */

/* AIN0 out also discharges cap */
#define DISCHARGE_ON DDRB |= (1 << PB2)
#define DISCHARGE_OFF DDRB &= ~(1 << PB2)

/* Range control */
#define HIGH_RANGE                                                             \
  PORTB |= (1 << PB1);                                                         \
  DDRB |= (1 << PB1)
#define LOW_RANGE                                                              \
  DDRB &= ~(1 << PB1);                                                         \
  PORTB &= ~(1 << PB1)
#define PULLDOWN_RANGE                                                         \
  PORTB &= ~(1 << PB1);                                                        \
  DDRB |= (1 << PB1)

/* Threshold selection */
#define ADMUX_LOW 1
#define ADMUX_MEDIUM 2
#define ADMUX_HIGH 3

/* Timer abstraction */
#define TIMER_VALUE TCNT1
#define TIMER_START TCCR1B = (1 << CS10)
#define TIMER_STOP TCCR1B = 0

/* Led abstraction */
#define LED_ON PORTB &= ~(1 << PB3)
#define LED_OFF PORTB |= (1 << PB3)

/* Button abstraction */
#define BUTTON_PUSHED (!(PINB & (1 << PB0)))

char decades[5] = {'p', 'n', 'u', 'm', ' '};

char lcdbuffer[32];

unsigned short volatile timer_highword;

/* Program states: */
#define STATE_IDLE 0
#define STATE_LOW_THRESH 1
#define STATE_HIGH_THRESH 2
#define STATE_DONE 3
// #define STATE_BUTTONDOWN 4

unsigned char volatile measure_state;

/* The following is the value the analog compare interrupt will set ADMUX: */
unsigned char volatile set_admux;

/* The rangemode defines the measurement operation */
#define RANGE_HIGH_THRESH 1 /* If missing: threshold low */
#define RANGE_HIGH 2        /* If missing: range low */
#define RANGE_AUTO 4
#define RANGE_OVERFLOW 8 /* If set: cap was out of range */
unsigned char rangemode = RANGE_AUTO;

/* Constants defining measuring operation: */
#define EXTRA_DISCHARGE_MS                                                     \
  100 /* Extra discharging that is done even after a threshold is crossed */
#define LOW_RANGE_TIMEOUT 500    /* At autorange, when to go to high range */
#define HIGH_RANGE_TIMEOUT 10000 /* When to give up completely */

/* Menu system */
#define MENU_SPEED 800 /* ms each menu item is shown */

#define MENU_ITEMS 6
char *menu_item[MENU_ITEMS] = {"Range: Auto",     "Range: Low ",
                               "Range: High",     "Calibrate: Zero",
                               "Calibrate: 1 uF", "Save calibration"};

/* Teoretisk beregning af capacitans

t = tau * ln((V0 - Vinf)/(V - Vinf)

De forskellige threshold v�rdier, og deres forhold til tau:

V      threshold    t fra 0 til x:        fra low til x:

0.88   low          tau * 0.1918910       tau * 0.240725   (=K1)
1.77   medium       tau * 0.4326161       tau * 0.554222   (=K2)
2.65   high         tau * 0.7461127

5.04   top

R1 = 1.588.000 ohm
R2 = 1752 ohm

t = tclocks / fcpu
tau * K = t
tau = R*C              =>

C = tau / R = t / (K * R) = tclocks / (K * R * fcpu)

  = tclocks / (0.240725 * 1588000 * 8000000) = tclocks * 3.270e-13 = tclocks *
21430 / 65536 * 1E-12 = tclocks / (0.554222 * 1588000 * 8000000) = tclocks *
9308 / 65536 * 1E-12

// // // alternatively
// // //   = tclocks / (0.240725 * 1588000 * 8000000) = tclocks * 5.96856E-13 =
tclocks * 53575 / 65536 * 4 * 1E-13
// // //   = tclocks / (0.554222 * 1588000 * 8000000) = tclocks * 5.96856E-13 =
tclocks * 46540 / 65536 * 2 * 1E-13
// // //
  = tclocks / (0.240725 * 1752 * 8000000) = tclocks * 19423 / 65536 * 1E-9
  = tclocks / (0.554222 * 1752 * 8000000) = tclocks * 8437 / 65536 * 1E-9

*/

/*
  Calibration:

  C = tclocks * Kc / 65536 * 1E-12

  Kc = C * 65536 * 1E12 / tclocks

     = CALIB_CONST / tclocks

Low range:                        C = 1uF
  CALIB_CONST = C * 65536e12      = 256000000  (*256)

High range:
  CALIB_CONST = C * 65536e9       = 65536000

*/

#define CALIB_LOW 256000000 /* for 1uF reference prescale: >> 8 */
#define CALIB_HIGH 65536000 /* for 1uF reference */

/* Calibration values are stored in eeprom in the following format:

   Starting from byte 1:  (not 0)
   'C' 'D'
   <data>

*/
#define EEPROM_HEADER 1
#define EEPROM_DATA 3

unsigned short calib[4] = {21430, 9308, 19423, 8437};
// unsigned short calib[4] = {53575, 46540, 19423, 8437};

uint32_t calib_offset[4] = {0, 0, 0, 0};

#define SIZE_OF_CALIB 8
#define SIZE_OF_CALIBOFFSET 16

/* Interrupt implementation */
ISR(ANA_COMP_vect) {
  if (measure_state == STATE_LOW_THRESH) {
    /* We just got low threshold interrupt, start timer and set high threshold
     */
    TIMER_START;
    ADMUX = set_admux;
    measure_state = STATE_HIGH_THRESH;
  } else if (measure_state == STATE_HIGH_THRESH) {
    /* High threshold interrupt, verify it, then stop timer */
    if (ACSR & (1 << ACO)) {
      TIMER_STOP;
      measure_state = STATE_DONE;
    }
  }
}

ISR(TIMER1_OVF_vect) {
  /* Timer 1 counts the low 16 bits, this interrupt updates the high 16 bits */
  timer_highword++;
}

/*
   The measure function does the cyclus of a capacitance measurement
   Returned is the number of clocks measured

   The function relies on flags in the global rangemode value
   Input flags:
     RANGE_AUTO
     RANGE_HIGH
     RANGE_HIGH_THRESH

   Output flags:
     RANGE_HIGH     (if RANGE_AUTO)
     RANGE_OVERFLOW

*/

void eeprom_read(void) {
  if (eeprom_read_byte((uint8_t *)EEPROM_HEADER) != 'C')
    return;

  if (eeprom_read_byte((uint8_t *)EEPROM_HEADER + 1) != 'D')
    return;

  eeprom_read_block(calib_offset, (void *)EEPROM_DATA, SIZE_OF_CALIBOFFSET);
  eeprom_read_block(calib, (void *)EEPROM_DATA + SIZE_OF_CALIBOFFSET,
                    SIZE_OF_CALIB);
}

void eeprom_write(void) {
  eeprom_write_byte((uint8_t *)EEPROM_HEADER, 'C');
  eeprom_write_byte((uint8_t *)EEPROM_HEADER + 1, 'D');

  eeprom_write_block(calib_offset, (void *)EEPROM_DATA, SIZE_OF_CALIBOFFSET);
  eeprom_write_block(calib, (void *)(EEPROM_DATA + SIZE_OF_CALIBOFFSET),
                     SIZE_OF_CALIB);
}

uint32_t measure(void) {
  unsigned short i;

  measure_state = STATE_IDLE;

  /* Discharge cap until below low threshold + some extra */
  ADMUX = ADMUX_LOW;
  PULLDOWN_RANGE; /* Use range signal as pull down */

  while (1) {
    /* Enable comperator and check value */
    DISCHARGE_OFF;
    ms_spin(1);

    /* This value must be checked in every loop */
    if (BUTTON_PUSHED)
      return 0;

    if (!(ACSR & (1 << ACO)))
      break;

    /* Discharge for a while */
    DISCHARGE_ON;
    ms_spin(10);
  }

  DISCHARGE_ON;
  ms_spin(EXTRA_DISCHARGE_MS);

  /* Prepare: reset timer, low range */
  TIMER_STOP;
  TIMER_VALUE = 0;
  timer_highword = 0;

  LOW_RANGE;

  measure_state = STATE_LOW_THRESH;

  /* High or medium threshold */
  if (rangemode & RANGE_HIGH_THRESH)
    set_admux = ADMUX_HIGH;
  else
    set_admux = ADMUX_MEDIUM;

  /* Apply step */
  LED_ON;
  DISCHARGE_OFF;

  if (rangemode & RANGE_AUTO) {

    /* Autorange: See if low range produces something before LOW_RANGE_TIMEOUT
     * ms */
    i = 0;
    while ((measure_state == STATE_LOW_THRESH) && (++i < LOW_RANGE_TIMEOUT)) {
      ms_spin(1);

      /* This value must be checked in every loop */
      if (BUTTON_PUSHED)
        return 0;
    }

    if (i >= LOW_RANGE_TIMEOUT) {
      /* low range timeout, go to high range (better discharge a little first)
       */
      DISCHARGE_ON;
      ms_spin(EXTRA_DISCHARGE_MS);
      DISCHARGE_OFF;
      HIGH_RANGE;
      rangemode |= RANGE_HIGH;
    } else {
      /* low range was ok, set flag accordingly */
      rangemode &= ~RANGE_HIGH;
    }
  } else if (rangemode & RANGE_HIGH) {
    HIGH_RANGE;
  }

  /* Wait for completion, timing out after HIGH_RANGE_TIMEOUT */
  i = 0;
  while ((measure_state != STATE_DONE) && (++i < HIGH_RANGE_TIMEOUT)) {
    ms_spin(1);

    /* This value must be checked in every loop */
    if (BUTTON_PUSHED)
      return 0;
  }

  /* Done, discharge cap now */
  LOW_RANGE;
  DISCHARGE_ON;
  LED_OFF;

  if (measure_state != STATE_DONE)
    rangemode |= RANGE_OVERFLOW;
  else
    rangemode &= ~RANGE_OVERFLOW;

  measure_state = STATE_IDLE;

  return ((uint32_t)timer_highword << 16) + TIMER_VALUE;
}

/*
   This function deals with value according to the global rangemode flag,
   and shows the result on LCD.

   LCD should preferably be cleared.

   Routine is rather slow
*/

void calc_and_show(uint32_t value) {
  unsigned char b;

  if (rangemode & RANGE_AUTO)
    lcd_string("Auto ", 0);
  else
    lcd_string("Force", 0);

  if (rangemode & RANGE_HIGH)
    lcd_string(" high", 16);
  else
    lcd_string(" low ", 16);

  if (rangemode & RANGE_OVERFLOW) {
    /* Todo - this smarter */
    lcdbuffer[0] = ' ';
    lcdbuffer[1] = ' ';
    lcdbuffer[2] = ' ';
    lcdbuffer[3] = 'E';
    lcdbuffer[4] = 'r';
    lcdbuffer[5] = 'r';
    lcdbuffer[6] = 'o';
    lcdbuffer[7] = 'r';
    lcdbuffer[8] = ' ';
    lcdbuffer[9] = 0;
  } else {
    /* Select calibration value */
    b = rangemode & 3;

    if (calib_offset[b] > value) {
      lcdbuffer[0] = '-';
      value = calib_offset[b] - value;
    } else {
      lcdbuffer[0] = ' ';
      value = value - calib_offset[b];
    }

    const uint32_t l = ((uint64_t)value * calib[b]) >> 16;

    b = long2ascii(lcdbuffer + 1, l);

    /* High range shifts 1E3 */
    if (rangemode & RANGE_HIGH)
      b++;

    lcdbuffer[6] = ' ';
    lcdbuffer[7] = decades[b]; /* range = 1 shifts 1E3 */
    lcdbuffer[8] = 'F';
    lcdbuffer[9] = 0;
  }

  /* Write high threshold in first line, low threshold in second */
  if (rangemode & RANGE_HIGH_THRESH)
    b = 7;
  else
    b = 23;

  lcd_string(lcdbuffer, b);
}

void calibrate_zero(void) {
  const unsigned char oldrange = rangemode;

  rangemode = 0;

  uint32_t l = measure();

  calib_offset[rangemode] = l;

  rangemode = RANGE_HIGH_THRESH;

  l = measure();

  calib_offset[rangemode] = l;

  rangemode = oldrange;
}

void calibrate(void) {
  const unsigned char oldrange = rangemode;

  rangemode = 0;
  uint32_t value = measure();
  value -= calib_offset[rangemode];
  calib[rangemode] = CALIB_LOW / (value >> 8) + 1;

  rangemode = RANGE_HIGH_THRESH;
  value = measure();
  value -= calib_offset[rangemode];
  calib[rangemode] = CALIB_LOW / (value >> 8) + 1;

  rangemode = RANGE_HIGH;
  value = measure();
  value -= calib_offset[rangemode];
  calib[rangemode] = CALIB_HIGH / value + 1;

  rangemode = RANGE_HIGH | RANGE_HIGH_THRESH;
  value = measure();
  value -= calib_offset[rangemode];
  calib[rangemode] = CALIB_HIGH / value + 1;

  rangemode = oldrange;
}

/* Hold-down-button menu implementation: */

unsigned char menu(void) {
  unsigned char i;

  lcd_clear();

  for (i = 0; i < MENU_ITEMS; i++) {
    lcd_string(menu_item[i], 0);
    ms_spin(MENU_SPEED);

    if (!BUTTON_PUSHED)
      break;
  }

  if (i == MENU_ITEMS) {
    /* Just clear display, if user went out of menu */
    lcd_clear();

    /* Wait for release of button */
    while (BUTTON_PUSHED)
      ;
    ms_spin(10);

  } else {
    /* Flash selected item */
    lcd_clear();
    ms_spin(MENU_SPEED >> 2);
    lcd_string(menu_item[i], 0);
    ms_spin(MENU_SPEED >> 1);
    lcd_clear();
    ms_spin(MENU_SPEED >> 2);
  }

  return i;
}

void init(void) {

  /* Set datadirections */
  DDRB = (1 << PB3);    /* led output, rest input */
  PORTB &= ~(1 << PB2); /* AIN0 port must be 0 */

  /* Enable button pull up resistor */
  PORTB |= (1 << PB0);

  /* Setup timer1 to normal operation */
  TCCR1A = 0;
  TCCR1B = 0;
  TIMSK = (1 << TOIE1); //(mega8)

  /* Setup analog comperator to generate rising edge interrupt */
  ACSR = (1 << ACIS0) | (1 << ACIS1) | (1 << ACIE);

  /* Setup analog comperator to use ADMUX */
  ADMUX = ADMUX_LOW;
  SFIOR |= (1 << ACME);
  ADCSRA &= ~(1 << ADEN); // disable ADC
  // DIDR1 |= (1<<AIN1D)|(1<<AIN0D);
}

int main(void) {
  uint32_t measurement;

  init();

  lcd_init();

  eeprom_read();

  sei();

  LED_OFF;

  rangemode = RANGE_AUTO;

  // ReSharper disable once CppDFAEndlessLoop
  while (1) {
    /* Toggle high/low threshold */
    rangemode ^= RANGE_HIGH_THRESH;
    measurement = measure();

    if (!BUTTON_PUSHED) {
      calc_and_show(measurement);
      continue;
    }

    /* Stop any cap. charging */
    LED_OFF;
    LOW_RANGE;
    DISCHARGE_ON;

    /* Menu implementation */
    switch (menu()) {
    case 0: /* auto range */
      rangemode |= RANGE_AUTO;
      break;
    case 1: /* low range */
      rangemode &= ~(RANGE_AUTO | RANGE_HIGH);
      break;
    case 2: /* high range */
      rangemode &= ~RANGE_AUTO;
      rangemode |= RANGE_HIGH;
      break;
    case 3:
      calibrate_zero();
      break;
    case 4:
      calibrate();
      break;
    case 5:
      eeprom_write();
      break;
    default:;
    }
  }
}
