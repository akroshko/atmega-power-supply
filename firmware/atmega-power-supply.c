#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <avr/io.h>
#define F_CPU 16000000
#include <util/delay.h>

// account for different brands of rotary encoders
#define V_DIRECTION 1
#define I_DIRECTION 0

// PWM pins
#define P_PWM_V          0b00000010 // PB1
#define P_PWM_I          0b00000100 // PB2

// rotary encoder pins
#define P_ROT_V_CLK      0b00000100 // PC2
#define P_ROT_V_CLK_N    0b11111011 // PC2 NOT
#define P_ROT_V_CLK_S    2          // PC2 SHIFT
#define P_ROT_V_DT       0b00001000 // PC3
#define P_ROT_V_DT_N     0b11110111 // PC3 NOT
#define P_ROT_V_DT_S     3          // PC3 SHIFT
#define P_ROT_I_CLK      0b00010000 // PC4
#define P_ROT_I_CLK_N    0b11101111 // PC4 NOT
#define P_ROT_I_CLK_S    4          // PC4 SHIFT
#define P_ROT_I_DT       0b00100000 // PC5
#define P_ROT_I_DT_N     0b11011111 // PC5 NOT
#define P_ROT_I_DT_S     5          // PC5 SHIFT

#define LCD_RS   0b00000001 // PD0
#define LCD_E    0b00000010 // PD1
#define LCD_DB4  0b00010000 // PD4
#define LCD_DB5  0b00100000 // PD5
#define LCD_DB6  0b01000000 // PD6
#define LCD_DB7  0b10000000 // PD7

#define PORT_DIRECTION_MASK 0b11110011
#define NDB_MASK            0b00001111
#define UPPERMASK           0b11110000
#define LOWERSHIFT          4

#define INITIAL_I 0
#define INITIAL_V 0

int current_time;
int last_display = -9999;
volatile int i_counter = INITIAL_I;
int i_counter_old = INITIAL_I;
volatile int v_counter = INITIAL_V;
int v_counter_old = INITIAL_V;
volatile int i_current_state;
volatile int i_current_state_2;
volatile int v_current_state;
volatile int v_current_state_2;

volatile int i_last_state;
volatile int v_last_state;

int i_full_scale=256;
int v_full_scale=1023;


int last_i_display=-1;
int last_v_display=-1;

float i_actual=-1.0;
float v_actual=-1.0;

char display_string[16];

void lcd_send_instruction(unsigned char instruction) {
  PORTD = (PORTD & NDB_MASK) | (instruction & UPPERMASK);
  _delay_us(1);
  PORTD &= ~LCD_RS;
  PORTD |=  LCD_E;
  _delay_us(1);
  PORTD &= ~LCD_E;
  _delay_us(200);
  PORTD = (PORTD & NDB_MASK) | (instruction << LOWERSHIFT);
  _delay_us(1);
  PORTD |=  LCD_E;
  _delay_us(1);
  PORTD &= ~LCD_E;
  _delay_ms(2);
}

void lcd_send_data(unsigned char data) {
  PORTD = (PORTD & NDB_MASK) | (data & UPPERMASK);
  _delay_us(1);
  PORTD |=  LCD_RS;
  PORTD |=  LCD_E;
  _delay_us(1);
  PORTD &= ~LCD_E;
  _delay_us(200);
  PORTD = (PORTD & NDB_MASK) | (data << LOWERSHIFT);
  _delay_us(1);
  PORTD |=  LCD_E;
  _delay_us(1);
  PORTD &= ~LCD_E;
  _delay_us(100);
}

void lcd_send_string(char *str) {
  for (int i=0; str[i]!=0; i++) {
    if (str[i] == ' ') {
      lcd_send_data(0xFE);
    } else {
      lcd_send_data(str[i]);
    }
  }
}

void v_count () {
  v_current_state=(PINC & P_ROT_V_CLK) >> P_ROT_V_CLK_S;
  v_current_state_2=(PINC & P_ROT_V_DT) >> P_ROT_V_DT_S;
  if(v_current_state != v_last_state) {
    if(v_current_state_2 != v_current_state) {
#if V_DIRECTION == 0
      if (v_counter < v_full_scale) {
        v_counter++;
      }
    } else {
      if (v_counter > 0) {
        v_counter--;
      }
#else
      if (v_counter > 0) {
        v_counter--;
      }
    } else {
      if (v_counter < v_full_scale) {
        v_counter++;
      }
#endif
    }
    OCR1A=v_counter;
    v_last_state = v_current_state;
  }
}

void i_count () {
  i_current_state=(PINC & P_ROT_I_CLK) >> P_ROT_I_CLK_S;
  i_current_state_2=(PINC & P_ROT_I_DT) >> P_ROT_I_DT_S;
  if(i_current_state != i_last_state) {
    if(i_current_state_2 != i_current_state) {
#if I_DIRECTION == 0
      if (i_counter < i_full_scale) {
        i_counter++;
      }
    } else {
      if (i_counter > 0) {
        i_counter--;
      }
#else
      if (i_counter > 0) {
        i_counter--;
      }
    } else {
      if (i_counter < i_full_scale) {
        i_counter++;
      }
#endif
    }
    OCR1B=i_counter;
    i_last_state = i_current_state;
  }
}

float pwm_to_current_limit (int pwm) {
        float fraction = 5.0 * ((float)pwm/(float)1024);
        float current_limit=(fraction*5.0/10.0);
        return current_limit;
}

float pwm_to_voltage (int pwm) {
        float fraction_voltage = 5.0 * ((float)pwm/(float)1024);
        float regulated_voltage=(2.0*fraction_voltage)+1.25;
        return regulated_voltage;
}

int main(void) {
  // TODO: setup
  DDRD |= PORT_DIRECTION_MASK;
  _delay_ms(50);
  // set to 4 bit mode
  lcd_send_instruction(0x02);
  // set to 2 lines and appropriate font
  lcd_send_instruction(0x28);
  // cursor and clear
  lcd_send_instruction(0x0c);
  lcd_send_instruction(0x06);
  lcd_send_instruction(0x01);
  _delay_ms(2);
  lcd_send_instruction(0x80);
  _delay_ms(2);
  lcd_send_string("Starting...");
  _delay_ms(3000);
  // next line
  lcd_send_instruction(0xC0);
  // this is a unique test string
  /* lcd_send_string("1234-5678-90"); */
  /* _delay_ms(3000); */
  // test out floating point
  float test_float=5.123;
  sprintf(display_string, "%.4f", test_float);
  lcd_send_string(display_string);
  _delay_ms(3000);
  // set up the pins
  DDRB  |= P_PWM_V;
  DDRB  |= P_PWM_I;
  DDRC  &= P_ROT_V_CLK_N;
  PORTC |= P_ROT_V_CLK;
  DDRC  &= P_ROT_V_DT_N;
  PORTC |= P_ROT_V_DT;
  DDRC  &= P_ROT_I_CLK_N;
  PORTC |= P_ROT_I_CLK;
  DDRC  &= P_ROT_I_DT_N;
  PORTC |= P_ROT_I_DT;
  // initial states
  v_last_state = (PINC & P_ROT_V_CLK) >> P_ROT_V_CLK_S;
  i_last_state = (PINC & P_ROT_I_CLK) >> P_ROT_I_CLK_S;
  // should set up as 10 bit phase correct PWM
  TCCR1A=_BV(COM1A1) | _BV(COM1B1) | _BV(WGM11) | _BV(WGM10);
  TCCR1B=_BV(CS10) | _BV(WGM12) | _BV(CS10);
  OCR1A=INITIAL_V;
  OCR1B=INITIAL_I;

  // finally clear out the LCD display
  lcd_send_instruction(0x01);
  lcd_send_instruction(0x80);
  // add the I and V
  lcd_send_string("I: ");
  lcd_send_instruction(0xC0);
  lcd_send_string("V: ");
  // move to first line
  lcd_send_instruction(0x83);
  i_actual=pwm_to_current_limit(INITIAL_I);
  sprintf(display_string,"%02.3fA %04d",i_actual,INITIAL_I);
  lcd_send_string(display_string);
  // move to second line
  lcd_send_instruction(0xC3);

  v_actual=pwm_to_voltage(INITIAL_V);
  sprintf(display_string,"%02.3fV %04d",v_actual,INITIAL_V);
  lcd_send_string(display_string);
  // the main loop
  while (1) {
    // let's count with the rotary encoder
    v_count();
    i_count();
    if (last_v_display != v_counter || last_i_display != i_counter) {
      // move to first line
      lcd_send_instruction(0x83);
      i_actual=pwm_to_current_limit(i_counter);
      sprintf(display_string,"%02.3fA %04d",i_actual,i_counter);
      lcd_send_string(display_string);
      // move to second line
      lcd_send_instruction(0xC3);
      v_actual=pwm_to_voltage(v_counter);
      sprintf(display_string,"%02.3fV %04d",v_actual,v_counter);
      lcd_send_string(display_string);
      last_v_display=v_counter;
      last_i_display=i_counter;
      _delay_ms(1);
    }
  }
}
