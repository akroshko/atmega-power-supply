// includes configurables

////////////////////////////////////////////////////////////////////////////////
// Software values to adjust behaviour
//
// These might need to be adjusted based on swapping out different
// components
////////////////////////////////////////////////////////////////////////////////

// account for different brands of rotary encoders
#define V_DIRECTION 1
#define I_DIRECTION 0

// values for PWM
const int i_full_scale=256;
const int v_full_scale=1023;

#define INITIAL_I 0
#define INITIAL_V 0

// values for the monitor
const float VREF_VALUE=2.048;

const float V_SCALE=0.2;

const float I_SCALE_MONITOR=3.0;
const float I_SCALE_DIFF=10.0;
const float I_SENSE_R=0.25;

const uint16_t MONITOR_RAW_MAX=1024;

////////////////////////////////////////////////////////////////////////////////
// Arbitrary constants that can be used to tweak performance
////////////////////////////////////////////////////////////////////////////////

#define LCD_DELAY_US           1
#define LCD_DELAY_SEND_US    100
#define ADC_POST_DELAY_US     16

 // do analog to digital every 50 loops
#define AD_LOOP_SKIP          50

////////////////////////////////////////////////////////////////////////////////
// Constants that will generally only change with the changing the circuit itself
// _S indicates shift need for to access appropriate bit
// P_ designates the "pin", which actually maps to the individual pin of a partuclar port
//
// _ROT indicates rotary encoding
// _V indicates the rotary encoder that controls voltage
// _I indicates the rotary encoder that controls current
// _CLK indicates the rotary encoder clock pin
// _DT indicates the rotary encoder ??? pin
//
// LCD_ indicates the LCD

// _RS _E _DB4 _DB5 _DB6 _DB7 are all constants that are specific to
// pins on the LCD
////////////////////////////////////////////////////////////////////////////////

// PWM pins
const uint8_t P_PWM_V=(_BV(1)); // PB1
const uint8_t P_PWM_I=(_BV(2)); // PB2

// rotary encoder pins
#define P_ROT_V_CLK_S 2                         // PC2 SHIFT
const uint8_t P_ROT_V_CLK=(_BV(P_ROT_V_CLK_S)); // PC2
#define P_ROT_V_DT_S 3                          // PC3 SHIFT
const uint8_t P_ROT_V_DT=(_BV(P_ROT_V_DT_S));   // PC3
#define P_ROT_I_CLK_S 4                         // PC4 SHIFT
const uint8_t P_ROT_I_CLK=(_BV(P_ROT_I_CLK_S)); // PC4
#define P_ROT_I_DT_S 5                          // PC5 SHIFT
const uint8_t P_ROT_I_DT=(_BV(P_ROT_I_DT_S));   // PC5

// LCD pins
#define LCD_RS_S 0                      // PD0 SHIFT
const uint8_t LCD_RS=(_BV(LCD_RS_S));   // PD0
#define LCD_E_S  1                      // PD1 SHIFT
const uint8_t LCD_E=(_BV(LCD_E_S));     // PD1
#define LCD_DB4_S 4                     // PD4 SHIFT
const uint8_t LCD_DB4=(_BV(LCD_DB4_S)); // PD4
#define LCD_DB5_S 5                     // PD5 SHIFT
const uint8_t LCD_DB5=(_BV(LCD_DB5_S)); // PD5
#define LCD_DB6_S 6                     // PD6 SHIFT
const uint8_t LCD_DB6=(_BV(LCD_DB6_S)); // PD6
#define LCD_DB7_S 7                     // PD7 SHIFT
const uint8_t LCD_DB7=(_BV(LCD_DB7_S)); // PD7

const uint8_t LCD_PORT_D_DIRECTION_MASK=_BV(LCD_RS_S)|_BV(LCD_E_S)| \
                                        _BV(LCD_DB4_S)|_BV(LCD_DB5_S)| \
                                        _BV(LCD_DB6_S)|_BV(LCD_DB7_S);
// LCD commands
#define LCD_COMMAND_4bit_mode  0x02
#define LCD_COMMAND_2line_mode 0x28
// TODO: fill out more LCD commands after datasheet review
