#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <avr/io.h>
#define F_CPU 16000000
#include <util/delay.h>
// see for constants configurable at compile time
#include "atmega-power-supply.h"

/**
 * Initialize the port used for the LCD.
 */
void lcd_init_port () {
  DDRD |= LCD_PORT_D_DIRECTION_MASK;
}

/**
 * Functions to support the LCD display.
 *
 * Datasheet title is "Specification for LCD Module 1602A-1(V1.2)" by
 * Shenzhen Eone Electronics Co. LTD.
 *
 * Google for eone-1602a1.pdf will probably give results.
 */

/**
 * Send an instruction to the LCD.
 */
void lcd_send_instruction(unsigned char instruction) {
  PORTD = (PORTD & 0x0F) | (instruction & 0xF0);
  _delay_us(LCD_DELAY_US);
  PORTD &= ~LCD_RS;
  PORTD |=  LCD_E;
  _delay_us(LCD_DELAY_US);
  PORTD &= ~LCD_E;
  _delay_us(LCD_DELAY_SEND_US*2);
  PORTD = (PORTD & 0x0F) | (instruction << 4);
  _delay_us(LCD_DELAY_US);
  PORTD |=  LCD_E;
  _delay_us(LCD_DELAY_US);
  PORTD &= ~LCD_E;
  _delay_ms(LCD_DELAY_US*2);
}

/**
 * Send data to the LCD.
 */
void lcd_send_data(unsigned char data) {
  PORTD = (PORTD & 0x0F) | (data & 0xF0);
  _delay_us(LCD_DELAY_US);
  PORTD |=  LCD_RS;
  PORTD |=  LCD_E;
  _delay_us(LCD_DELAY_US);
  PORTD &= ~LCD_E;
  _delay_us(LCD_DELAY_SEND_US*2);
  PORTD = (PORTD & 0x0F) | (data << 4);
  _delay_us(LCD_DELAY_US);
  PORTD |=  LCD_E;
  _delay_us(LCD_DELAY_US);
  PORTD &= ~LCD_E;
  _delay_us(LCD_DELAY_SEND_US);
}

/**
 * Put LCD into 4 bit mode.
 */
void lcd_set_4bit_mode(void) {
  lcd_send_instruction(LCD_COMMAND_4bit_mode);
}

/**
 * Put LCD into 2 line mode.
 */
void lcd_2line_init(void) {
  lcd_send_instruction(LCD_COMMAND_2line_mode);
}

/**
 * Clear the LCD initially.
 */
void lcd_clear_init(void) {
  // cursor and clear
  lcd_send_instruction(0x0c);
  lcd_send_instruction(0x06);
  lcd_send_instruction(0x01);
  _delay_ms(2);
  lcd_send_instruction(0x80);
  _delay_ms(2);
}

/**
 * Move to the LCDs next line.
 */
void lcd_next_line(void) {
  lcd_send_instruction(0xC0);
}

/**
 * Clear the LCD in a non-initialization situation.
 */
void lcd_clear(void) {
  lcd_send_instruction(0x01);
  lcd_send_instruction(0x80);
}

/**
 * Move to the LCDs first line.
 */
void lcd_first_line(void) {
  lcd_send_instruction(0x83);
}

/**
 * Move to the LCDs second line.
 */
void lcd_second_line(void) {
  lcd_send_instruction(0xC3);
}

/**
 * Send string to LCD.
 */
void lcd_send_string(char *str) {
  for (int i=0; str[i] != 0; i++) {
    if (str[i] == ' ') {
      lcd_send_data(0xFE);
    } else {
      lcd_send_data(str[i]);
    }
  }
}

/**
 * Functions to support the Rottary encoder.
 *
 * There are no specific datasheets for the ones I use but search 5pin
 * rotary encoder for representative datasheets.
 */

/**
 * Initialize the rotary encoders.  See "atmega-power-supply.h"
 * descriptions of the constants.
 */
void init_rotary_encoders () {
  // set up the pins
  DDRB  |= P_PWM_V;
  DDRB  |= P_PWM_I;
  DDRC  &= ~P_ROT_V_CLK;
  PORTC |= P_ROT_V_CLK;
  DDRC  &= ~P_ROT_V_DT;
  PORTC |= P_ROT_V_DT;
  DDRC  &= ~P_ROT_I_CLK;
  PORTC |= P_ROT_I_CLK;
  DDRC  &= ~P_ROT_I_DT;
  PORTC |= P_ROT_I_DT;
}

/**
 * Get the rotary encoder position for current.
 *
 * @param i_raw                Pointer to hold the raw value for i.
 * @param i_encoder_last_state Pointer to hold the last state of the rotary encoder.
 */
void get_i_control_raw (volatile int *i_raw, volatile uint16_t *i_encoder_last_state) {
  volatile uint16_t i_current_state;
  volatile uint16_t i_current_state_2;
  i_current_state=(PINC & P_ROT_I_CLK) >> P_ROT_I_CLK_S;
  i_current_state_2=(PINC & P_ROT_I_DT) >> P_ROT_I_DT_S;
  if(i_current_state != *i_encoder_last_state) {
    if(i_current_state_2 != i_current_state) {
#if I_DIRECTION == 0
      if (*i_raw < i_full_scale) {
        (*i_raw)++;
      }
    } else {
      if (*i_raw > 0) {
        (*i_raw)--;
      }
#else
      if (*i_raw > 0) {
        (*i_raw)--;
      }
    } else {
      if ((*i_raw) < i_full_scale) {
        (*i_raw)++;
      }
#endif
    }
    OCR1B=*i_raw;
    *i_encoder_last_state = i_current_state;
  }
}

/**
 * Get the rotary encoder position for voltage.
 *
 * @param v_raw            Pointer to hold the raw value for v.
 * @param v_encoder_last_state Pointer to hold the last state of the rotary encoder.
 */
void get_v_control_raw (volatile int *v_raw, volatile uint16_t *v_encoder_last_state) {
  volatile uint16_t v_current_state;
  volatile uint16_t v_current_state_2;
  v_current_state=(PINC & P_ROT_V_CLK) >> P_ROT_V_CLK_S;
  v_current_state_2=(PINC & P_ROT_V_DT) >> P_ROT_V_DT_S;
  if(v_current_state != *v_encoder_last_state) {
    if(v_current_state_2 != v_current_state) {
#if V_DIRECTION == 0
      if (*v_raw < v_full_scale) {
        (*v_raw)++;
      }
    } else {
      if (*v_raw > 0) {
        (*v_raw)--;
      }
#else
      if (*v_raw > 0) {
        (*v_raw)--;
      }
    } else {
      if (*v_raw < v_full_scale) {
        (*v_raw)++;
      }
#endif
    }
    OCR1A=*v_raw;
    *v_encoder_last_state = v_current_state;
  }
}

/*
 * Convert raw current limit values to an actual floating point value.
 *
 * @param raw The raw value.
 * @return The actual value.
 */
float raw_to_current_limit (uint16_t raw) {
  float fraction = 5.0 * ((float)raw/(float)1024);
  float current_limit=(fraction*5.0/10.0);
  return current_limit;
}

/*
 * Convert raw voltage values to an actual floating point value.
 *
 * @param raw The raw value.
 * @return The actual value.
 */
float raw_to_voltage (uint16_t raw) {
  float fraction_voltage = 5.0 * ((float)raw/(float)1024);
  float regulated_voltage=(2.0*fraction_voltage)+1.25;
  return regulated_voltage;
}

/**
 * Inititialize the PWM.
 */
void init_pwm() {
  // should set up as 10 bit phase correct PWM
  TCCR1A=_BV(COM1A1) | _BV(COM1B1) | _BV(WGM11) | _BV(WGM10);
  TCCR1B=_BV(CS10) | _BV(WGM12) | _BV(CS10);
  OCR1A=INITIAL_V;
  OCR1B=INITIAL_I;
}

/**
 * Inititialize the A/D convertor.
 */
void init_ad () {
#ifndef DEBUG_TEMPERATURE_SENSOR
  ADMUX &= 0;
#else
  ADMUX &= (1<<REFS1) | (1<<REFS0) | (1<<MUX2);
#endif
  ADCSRA |= (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
}

/**
 * Read a value from the A/D convertor.
 *
 * @param channel The A/D channel to read from.
 */
uint16_t read_ad (uint8_t channel) {
  volatile uint16_t adc_value;
#ifndef DEBUG_TEMPERATURE_SENSOR
  ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
#else
  // temperature sensor is channel 8 on ATMEGA328/168
  ADMUX = (ADMUX & 0xF0) | (8 & 0x0F);
#endif
  ADCSRA |= (1<<ADSC);
  while(ADCSRA & (1<<ADSC));
  _delay_us(ADC_POST_DELAY_US);
  adc_value=ADC;
  return adc_value;
}

/**
 * Convert raw data from the A/D convertor to a current value.
 *
 * @param raw The raw A/D convertor value.
 * @return The actual current value.
 */
float raw_monitor_to_current (uint16_t raw) {
  float i_monitor_v=(((float)raw/(float)MONITOR_RAW_MAX)*VREF_VALUE);
  float i_sense_v=i_monitor_v/I_SCALE_MONITOR/I_SCALE_DIFF;
  float i_monitor=i_sense_v/I_SENSE_R;
  return i_monitor;
}

/**
 * Convert raw data from the A/D convertor to a voltage value.
 *
 * @param raw The raw A/D convertor value.
 * @return The actual voltage value.
 */
float raw_monitor_to_voltage (uint16_t raw) {
  // TODO: get rid of division
  float v_monitor=(((float)raw/(float)MONITOR_RAW_MAX)*VREF_VALUE)/V_SCALE;
  return v_monitor;
}


int main(void) {
  volatile int i_control_raw = INITIAL_I;
  volatile int v_control_raw = INITIAL_V;

  volatile uint16_t i_encoder_control_last_state;
  volatile uint16_t v_encoder_control_last_state;

  volatile int i_control_raw_last=-1;
  volatile int v_control_raw_last=-1;

  float i_actual=-1.0;
  float v_actual=-1.0;

  float i_actual_monitor=-1.0;
  float v_actual_monitor=-1.0;

  uint16_t v_raw_monitor=0,i_raw_monitor=0;

  int loop_count=0;
  int display_update=0;

  // The following character sequence is superflous but is good for
  // putting things to test and debug
  char display_string[16];
  lcd_init_port();
  _delay_ms(50);
  lcd_set_4bit_mode();
  lcd_clear_init();
  lcd_send_string("Starting...");
  _delay_ms(3000);
  lcd_next_line();
  // this is a unique test string that makes sure floating point is working correctly
  float test_float=5.123;
  sprintf(display_string, "%.4f", test_float);
  lcd_send_string(display_string);
  _delay_ms(3000);
  init_rotary_encoders();
  init_ad();
  // initial states
  v_encoder_control_last_state = (PINC & P_ROT_V_CLK) >> P_ROT_V_CLK_S;
  i_encoder_control_last_state = (PINC & P_ROT_I_CLK) >> P_ROT_I_CLK_S;
  init_pwm();
  // done the LCD initialization
  // finally clear out the LCD display
  lcd_clear();
  // add the I and V
  lcd_send_string("A: ");
  lcd_next_line();
  lcd_send_string("V: ");
  // first line
  lcd_first_line();
  i_actual=raw_to_current_limit(INITIAL_I);
#ifndef DEBUG_RAW
  sprintf(display_string,"%02.3fA %04d",i_actual,INITIAL_I);
#else
  sprintf(display_string,"%04d %04d",INITIAL_I,INITIAL_I);
#endif
  lcd_send_string(display_string);
  // second line
  lcd_second_line();
  v_actual=raw_to_voltage(INITIAL_V);
#ifndef DEBUG_RAW
  sprintf(display_string,"%02.3fV %04d",v_actual,INITIAL_V);
#else
  sprintf(display_string,"%04d %04d",INITIAL_V,INITIAL_V);
#endif
  lcd_send_string(display_string);

  // the main loop
  while (1) {
    // loop skip is necessary here until the rotary encoders are on
    // interupts
    if (loop_count > AD_LOOP_SKIP) {
      v_raw_monitor=read_ad(0);
      v_actual_monitor=raw_monitor_to_voltage(v_raw_monitor);
      _delay_us(ADC_POST_DELAY_US);
      // ADC channel 1 (current)
      i_raw_monitor=read_ad(1);
      i_actual_monitor=raw_monitor_to_current(i_raw_monitor);
      _delay_us(ADC_POST_DELAY_US);
      // reset the loop count
      loop_count=0;
      display_update=1;
    }
    // let's count with the rotary encoder
    get_i_control_raw(&i_control_raw, &i_encoder_control_last_state);
    get_v_control_raw(&v_control_raw, &v_encoder_control_last_state);
    if (v_control_raw_last != v_control_raw || i_control_raw_last != i_control_raw || display_update==1) {
      v_control_raw_last=v_control_raw;
      i_control_raw_last=i_control_raw;
      v_actual=raw_to_voltage(v_control_raw_last);
      i_actual=raw_to_current_limit(i_control_raw_last);
      lcd_first_line();
#ifndef DEBUG_RAW
      sprintf(display_string,"%02.3f %02.3f",i_actual,i_actual_monitor);
#else
      sprintf(display_string,"%04d %04d",i_control_raw_last,i_raw_monitor);
#endif
      lcd_send_string(display_string);
      // move to second line
      lcd_second_line();
#ifndef DEBUG_RAW
      sprintf(display_string,"%02.3f %02.3f",v_actual,v_actual_monitor);
#else
      sprintf(display_string,"%04d %04d",v_control_raw_last,v_raw_monitor);
#endif
      lcd_send_string(display_string);
      display_update=0;
    }
    // a short delay to help with debouncing and other display issues
    _delay_ms(1);
    loop_count+=1;
  }
}
