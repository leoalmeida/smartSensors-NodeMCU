
/*==============================================================================
 * WIFI CONFIGURATION
 *
 * You must configure your particular hardware. Follow the steps below.
 *============================================================================*/

// Uncomment / comment the appropriate set of includes for your hardware (OPTION A, B or C)
// Arduino MKR1000 or ESP8266 are enabled by default if compiling for either of those boards.

/*
 * OPTION A: Configure for Arduino MKR1000 or Arduino WiFi Shield 101
 */
//#define WIFI_101

//do not modify the following 11 lines
#if defined(ARDUINO_SAMD_MKR1000) && !defined(WIFI_101)
// automatically include if compiling for MRK1000
#define WIFI_101
#endif
#ifdef WIFI_101
#include <WiFi101.h>
#include "utility/WiFiClientStream.h"
#include "utility/WiFiServerStream.h"
  #define WIFI_LIB_INCLUDED
#endif

/*
 * OPTION B: Configure for legacy Arduino WiFi shield
 */
//#define ARDUINO_WIFI_SHIELD

//do not modify the following 10 lines
#ifdef ARDUINO_WIFI_SHIELD
#include <WiFi.h>
#include "utility/WiFiClientStream.h"
#include "utility/WiFiServerStream.h"
  #ifdef WIFI_LIB_INCLUDED
  #define MULTIPLE_WIFI_LIB_INCLUDES
  #else
  #define WIFI_LIB_INCLUDED
  #endif
#endif

/*
 * OPTION C: Configure for ESP8266
 *
 */
//do not modify the following 14 lines
#ifdef ESP8266
// automatically include if compiling for ESP8266
#define ESP8266_WIFI
#endif
#ifdef ESP8266_WIFI
#include <ESP8266WiFi.h>
#include "utility/WiFiClientStream.h"
#include "utility/WiFiServerStream.h"
  #ifdef WIFI_LIB_INCLUDED
  #define MULTIPLE_WIFI_LIB_INCLUDES
  #else
  #define WIFI_LIB_INCLUDED
  #endif
#endif

/*==============================================================================
 * CONFIGURATION ERROR CHECK (don't change anything here)
 *============================================================================*/

#ifdef MULTIPLE_WIFI_LIB_INCLUDES
#error "you may not define more than one wifi device type in wifiConfig.h."
#endif

#ifndef WIFI_LIB_INCLUDED
#error "you must define a wifi device type in wifiConfig.h."
#endif

#if ((defined(WIFI_NO_SECURITY) && (defined(WIFI_WEP_SECURITY) || defined(WIFI_WPA_SECURITY))) || (defined(WIFI_WEP_SECURITY) && defined(WIFI_WPA_SECURITY)))
#error "you may not define more than one security type at the same time in wifiConfig.h."
#endif  //WIFI_* security define check

#if !(defined(WIFI_NO_SECURITY) || defined(WIFI_WEP_SECURITY) || defined(WIFI_WPA_SECURITY))
#error "you must define a wifi security type in wifiConfig.h."
#endif  //WIFI_* security define check

#if (defined(ESP8266_WIFI) && !(defined(WIFI_NO_SECURITY) || (defined(WIFI_WPA_SECURITY))))
#error "you must choose between WIFI_NO_SECURITY and WIFI_WPA_SECURITY"
#endif

/*==============================================================================
 * WIFI STREAM (don't change anything here)
 *============================================================================*/


//#ifdef SERVER_IP_ADDRESS
//  WiFiClientStream stream(IPAddress(SERVER_IP_ADDRESS), MQTT_PORT);
//#else
//  WiFiServerStream stream(MQTT_PORT);
//#endif
/* network */
#ifdef LOCAL_IP_ADDRESS
IPAddress local_ip(LOCAL_IP_ADDRESS);
#endif
#ifdef SUBNET_MASK
IPAddress subnet(SUBNET_MASK);
#endif
#ifdef GATEWAY_IP_ADDRESS
IPAddress gateway(GATEWAY_IP_ADDRESS);
#endif

