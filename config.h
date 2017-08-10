#define SSID_SIZE 32
#define PASS_SIZE 64
/*==============================================================================
 * Network definitions
 *==============================================================================
 * Local Wifi definitions
 *============================================================================*/
//#define LOCAL_IP_ADDRESS    192, 168, 0, 50
//#define LOCAL_PORT          3030
#define SUBNET_MASK         255, 255, 255, 0
#define GATEWAY_IP_ADDRESS  192, 168, 0, 1
/*==============================================================================
 *Remote Server definitions
 *============================================================================*/

#define SERVER_IP_ADDRESS    192, 168, 0, 6
#define MQTT_SERVER_IP_ADDRESS   "192.168.0.6"
#define MQTT_PORT         1883
#define SERVER_PORT         3004


/*==============================================================================
 * Local Wifi credentials
 *============================================================================*/
char ssid[SSID_SIZE+1] = "riodejaneiro";
#define WIFI_WPA_SECURITY

#ifdef WIFI_WPA_SECURITY
char password[PASS_SIZE+1] = "87654312";
#endif  //WIFI_WPA_SECURITY

#define CLIENT_ID   "58f3ac46866064c6189ec932"
//#define UUID        "6dffcf5a-8be1-42dc-822d-e4aafa40223d"
#define UUID        ""
//#define TOKEN       "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJqdGkiOiIzOWRhOTNjZi01YmMxLTRmZWYtYWE5Mi02MDkxODhkNGU2ZDMifQ.2fOWOPw8DsNkssI9OQUAuJ0TzBFaOxq_fIbD02AIfew"
#define TOKEN        ""


#define MAX_PROFILES                20

#define OFFLINE_MESSAGE "topic is offline"
#define ONLINE_MESSAGE "topic is online"
#define MQTT_QOS_1 0x1
#define MQTT_QOS_0 0x0
#define MQTT_CONN_WILLRETAIN 0x1

// the minimum interval for sampling analog input
#define MINIMUM_SAMPLING_INTERVAL   1
#define MAX_CONN_ATTEMPTS           20  // [500 ms] -> 10 s

#define STRING_DATA             0x72 // a string message with 14-bits per char
#define STRING_ERROR            0x73 // a string message with 14-bits per char
#define DIGITAL_MESSAGE         0x90 // send data for a digital pin
#define ANALOG_MESSAGE          0xE0 // send data for an analog pin (or PWM)
#define REPORT_ANALOG           0xC0 // enable analog input by pin #
#define REPORT_DIGITAL          0xD0 // enable digital input by port pair
#define REPORT_STATE            0xD0 // enable digital input by port pair
//
#define SET_PIN_MODE            0xF4 // set a pin to INPUT/OUTPUT/PWM/etc
#define SET_DIGITAL_PIN_VALUE   0xF5 // set value of an individual digital pin
//
#define SYSTEM_RESET            0xFF // reset from MIDI

//
// extended command set using sysex (0-127/0x00-0x7F)
/* 0x00-0x0F reserved for user-defined commands */

// pin modes
#define PIN_MODE_INPUT          0x00 // same as INPUT defined in Arduino.h
#define PIN_MODE_OUTPUT         0x01 // same as OUTPUT defined in Arduino.h
#define PIN_MODE_ANALOG         0x02 // analog pin in analogInput mode
#define PIN_MODE_PWM            0x03 // digital pin in PWM output mode
#define PIN_MODE_SERVO          0x04 // digital pin in Servo output mode
#define PIN_MODE_SHIFT          0x05 // shiftIn/shiftOut mode
#define PIN_MODE_I2C            0x06 // pin included in I2C setup
#define PIN_MODE_ONEWIRE        0x07 // pin configured for 1-wire
#define PIN_MODE_STEPPER        0x08 // pin configured for stepper motor
#define PIN_MODE_ENCODER        0x09 // pin configured for rotary encoders
#define PIN_MODE_SERIAL         0x0A // pin configured for serial communication
#define PIN_MODE_PULLUP         0x0B // enable internal pull-up resistor for pin
#define PIN_MODE_IGNORE         0x7F // pin configured to be ignored by digitalWrite and capabilityResponse
#define TOTAL_PIN_MODES         13

#define PIN_STATE_PWM           0x02 // analog pin in analogInput mode
#define PIN_STATE_SERVO         0x02 // analog pin in analogInput mode


#define ANALOG                  0x02 // same as PIN_MODE_ANALOG
#define PWM                     0x03 // same as PIN_MODE_PWM
#define SERVO                   0x04 // same as PIN_MODE_SERVO
#define SHIFT                   0x05 // same as PIN_MODE_SHIFT
#define I2C                     0x06 // same as PIN_MODE_I2C
#define ONEWIRE                 0x07 // same as PIN_MODE_ONEWIRE
#define STEPPER                 0x08 // same as PIN_MODE_STEPPER
#define ENCODER                 0x09 // same as PIN_MODE_ENCODER
#define IGNORE                  0x7F // same as PIN_MODE_IGNORE




/*==============================================================================
 * PIN IGNORE MACROS (don't change anything here)
 *============================================================================*/

#if defined(WIFI_101) && !defined(ARDUINO_SAMD_MKR1000)
// ignore SPI pins, pin 5 (reset WiFi101 shield), pin 7 (WiFi handshake) and pin 10 (WiFi SS)
// also don't ignore SS pin if it's not pin 10. Not needed for Arduino MKR1000.
#define IS_IGNORE_PIN(p)  ((p) == 10 || (IS_PIN_SPI(p) && (p) != SS) || (p) == 5 || (p) == 7)

#elif defined(ARDUINO_WIFI_SHIELD) && defined(__AVR_ATmega32U4__)
// ignore SPI pins, pin 4 (SS for SD-Card on WiFi-shield), pin 7 (WiFi handshake) and pin 10 (WiFi SS)
// On Leonardo, pin 24 maps to D4 and pin 28 maps to D10
#define IS_IGNORE_PIN(p)  ((IS_PIN_SPI(p) || (p) == 4) || (p) == 7 || (p) == 10 || (p) == 24 || (p) == 28)

#elif defined(ARDUINO_WIFI_SHIELD)
// ignore SPI pins, pin 4 (SS for SD-Card on WiFi-shield), pin 7 (WiFi handshake) and pin 10 (WiFi SS)
#define IS_IGNORE_PIN(p)  ((IS_PIN_SPI(p) || (p) == 4) || (p) == 7 || (p) == 10)

#elif defined(ESP8266_WIFI) && defined(SERIAL_DEBUG)
#define IS_IGNORE_PIN(p)  ((p) == 1)

#endif
