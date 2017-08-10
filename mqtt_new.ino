#include <Servo.h>
#include <Wire.h>
#include "Boards.h"  /* Hardware Abstraction Layer + Wiring/Arduino */
#include "config.h"
#include "LinkedList.h"
#include "wifiConfig.h"
#include <mqtt_client.h>

#include <ArduinoJson.h>

extern "C" {
#include "user_interface.h"
}

os_timer_t myTimer;

bool tickOccured;

// create MQTT object and callback functions
String pub_topic(CLIENT_ID);
String will_topic(CLIENT_ID "/offline");
String action_topic(CLIENT_ID "/action");

void myDataCb(String& topic, String& data);
void myPublishedCb();
void myDisconnectedCb();
void myConnectedCb();
void myTimeoutCb();

MQTT myMqtt(CLIENT_ID, MQTT_SERVER_IP_ADDRESS, MQTT_PORT, UUID, TOKEN);

//===============================================
//void timerCallback(void *pArg);
//void user_init(void);
//void parseCommand(byte command, byte pin, const char *string);
//void parseCommand(byte command, byte pin, int inputData);
void outputPort(byte, byte, byte);
void checkDigitalInputs(void);
void reportAnalog(void);
void reportAnalog(byte, int);
void reportDigital(byte, int);

void digitalWriteCallback(byte, int);
void writeStateAnalog(byte, int);
void writeStateDigital(byte, int);

void writePinMode(byte, int);

void processInput(String);
void printWifiStatus();
void initTransport();

void attachProfile(String , boolean, boolean);
boolean detachProfile(String);
byte getPinMode(byte);
void setPinMode(byte, int);
int getPinState(byte);
void setPinState(byte, int);
void setPinValue(byte, int);
void systemReset(void);
void ignorePins();
void enableI2CPins();
void disableI2CPins();
void reset();
boolean elapsed();
void setSamplingInterval(int interval);
//===============================================

/* analog inputs */
int analogInputsToReport = 0;      // bitwise array to store pin reporting

/* digital input ports */
int digitalInputsToReport = 0;
byte reportPINs[TOTAL_PORTS];       // 1 = report this port, 0 = silence
byte previousPINs[TOTAL_PORTS];     // previous 8 bits sent

/* pins configuration */
byte pinConfig[TOTAL_PORTS];         // configuration of every pin
byte portConfigInputs[TOTAL_PORTS]; // each bit: 1 = pin in INPUT, 0 = anything else
int pinState[TOTAL_PINS];           // any value that has been written

/* timer variables */
unsigned long currentMillis;        // store the current value from millis()
unsigned long previousMillis;       // for comparison with currentMillis
unsigned int samplingInterval; // how often to sample analog inputs (in ms)
boolean mqttConnected=false;

long lastAction = 0;
//std::vector<String> profiles;
LinkedList<profile_t> profiles = LinkedList<profile_t>();
int profileCount = 0;

Servo servos[MAX_SERVOS];
byte servoPinMap[TOTAL_PINS];
byte detachedServos[MAX_SERVOS];
byte detachedServoCount = 0;
byte servoCount = 0;
boolean isResetting;
int connectionAttempts = 0;

bool streamConnected = false;


// start of timerCallback
void timerCallback(void *pArg) {

      tickOccured = true;

} // End of timerCallback

void user_init(void) {
  os_timer_setfn(&myTimer, timerCallback, NULL);
  os_timer_arm(&myTimer, 1000, true);
}// End of user_init

/// Commmands
void parseCommand(byte command, byte pin, int inputData)
{
  Serial.println(pin);
  Serial.println(inputData);
  Serial.println(command);
  switch (command) {
    case ANALOG_MESSAGE:
          writeStateAnalog(pin, inputData);
          break;
    case DIGITAL_MESSAGE:
          digitalWriteCallback(pin, inputData);
          break;
    case SET_PIN_MODE:
    {
          Serial.println("set pin");
          setPinMode(pin, inputData);
          break;
    }
    case SET_DIGITAL_PIN_VALUE:
          writeStateDigital(pin, inputData);
          break;
    case REPORT_STATE:
      {
          Serial.println("MESSAGE");
          String buffer = prepareResponse(pin, String(inputData), "message");
          // publish value to topic
          Serial.print("msg: ");
          Serial.println(buffer);
          myMqtt.publish(pub_topic, buffer, MQTT_QOS_1, MQTT_CONN_WILLRETAIN);
          break;
      }
    case SYSTEM_RESET:
          systemReset();
          break;
  }
}
void parseCommand(byte command, byte pin, const char *string)
{
  switch (command) {
    case STRING_ERROR:
    {
      Serial.print("error");
      String buffer = prepareResponse(pin, string, "error");
      // publish value to topic
      boolean result = myMqtt.publish(pub_topic, buffer, MQTT_QOS_1, MQTT_CONN_WILLRETAIN);
      break;
    }
    case STRING_DATA:
    {
      Serial.print("message");
      String buffer = prepareResponse(pin, string, "message");
      // publish value to topic
      boolean result = myMqtt.publish(pub_topic, buffer, MQTT_QOS_1, MQTT_CONN_WILLRETAIN);
      break;
    }
  }
}

// main functions
void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println();
  Serial.println("");
  Serial.println("--------------------------");
  Serial.println("ESP8266 Sensor controller");
  Serial.println("--------------------------");
  
  systemReset();
  ignorePins();

  yield();

  initTransport();

  delay(10);
}

//
void loop() {
  byte pin, analogPin, digitalPin;
  int value;
  //String topic(CLIENT_ID);
  if (tickOccured == true){
      Serial.println("Tick Occurred");
      tickOccured = false;
      Serial.print("MQTT Status: ");
      Serial.println(myMqtt.getState());
      Serial.println(myMqtt.getClient().connect_info.client_id);


  //if (elapsed()) {

      //Serial.println("Reading");
      /* ANALOGREAD - do all analogReads() at the configured sampling interval */
      reportAnalog();
      checkDigitalInputs();
  }

  yield();
  //stream.maintain();
  //delay(1000);
}

/*
 *
 */
void myConnectedCb()
{
  Serial.println("connected to MQTT server");
  mqttConnected=true;
  //attachProfile("58f3ac46866064c6189ec947", true, true);

  Serial.println("connected");

  // Once connected, publish an announcement...
  myMqtt.publish(will_topic, ONLINE_MESSAGE, MQTT_QOS_1, MQTT_CONN_WILLRETAIN);
  // ... and resubscribe
  Serial.println("published will");
  myMqtt.subscribe(action_topic);
  Serial.println("subscribed action");
}

void myDisconnectedCb()
{
  mqttConnected=false;
  Serial.println("disconnected. trying to reconnect...");
  delay(500);
  myMqtt.connect();
}

void myPublishedCb()
{
  Serial.println("published.");
}

void myTimeoutCb()
{
  Serial.println("timeout.");
}

void myDataCb(String& topic, String& data)
{

  Serial.print(topic);
  Serial.print(": ");
  Serial.println(data);

  processInputData(data);
}

void checkDigitalInputs(void)
{
  /* Using non-looping code allows constants to be given to readPort().
   * The compiler will apply substantial optimizations if the inputs
   * to readPort() are compile-time constants. */
  /*for (int port=0;port<TOTAL_PINS;port++){
    Serial.print("report pin: ");
    Serial.print(port);
    Serial.print("[");
    Serial.print(reportPINs[port]);
    Serial.print("][");
    Serial.print(portConfigInputs[port]);
    Serial.println("]");
   Serial.print(".");
   //if (reportPINs[port])
      outputPort(port, readPort(port, portConfigInputs[port]), false);
  }*/

  if (TOTAL_PORTS > 0 && reportPINs[0]) outputPort(0, readPort(0, portConfigInputs[0]), false);
  if (TOTAL_PORTS > 1 && reportPINs[1]) outputPort(1, readPort(1, portConfigInputs[1]), false);
  if (TOTAL_PORTS > 2 && reportPINs[2]) outputPort(2, readPort(2, portConfigInputs[2]), false);
  if (TOTAL_PORTS > 3 && reportPINs[3]) outputPort(3, readPort(3, portConfigInputs[3]), false);
  if (TOTAL_PORTS > 4 && reportPINs[4]) outputPort(4, readPort(4, portConfigInputs[4]), false);
  if (TOTAL_PORTS > 5 && reportPINs[5]) outputPort(5, readPort(5, portConfigInputs[5]), false);
  if (TOTAL_PORTS > 6 && reportPINs[6]) outputPort(6, readPort(6, portConfigInputs[6]), false);
  if (TOTAL_PORTS > 7 && reportPINs[7]) outputPort(7, readPort(7, portConfigInputs[7]), false);
  if (TOTAL_PORTS > 8 && reportPINs[8]) outputPort(8, readPort(8, portConfigInputs[8]), false);
  if (TOTAL_PORTS > 9 && reportPINs[9]) outputPort(9, readPort(9, portConfigInputs[9]), false);
  if (TOTAL_PORTS > 10 && reportPINs[10]) outputPort(10, readPort(10, portConfigInputs[10]), false);
  if (TOTAL_PORTS > 11 && reportPINs[11]) outputPort(11, readPort(11, portConfigInputs[11]), false);
  if (TOTAL_PORTS > 12 && reportPINs[12]) outputPort(12, readPort(12, portConfigInputs[12]), false);
  if (TOTAL_PORTS > 13 && reportPINs[13]) outputPort(13, readPort(13, portConfigInputs[13]), false);
  if (TOTAL_PORTS > 14 && reportPINs[14]) outputPort(14, readPort(14, portConfigInputs[14]), false);
  if (TOTAL_PORTS > 15 && reportPINs[15]) outputPort(15, readPort(15, portConfigInputs[15]), false);

}

void reportDigital(byte port, int value)
{

  Serial.print("report pin: ");
  Serial.print(port);
  Serial.print("[");
    Serial.print(reportPINs[port]);
    Serial.print("]");
  if (port < TOTAL_PINS) {
    //reportPINs[port] = (byte)value;
    if (value) outputPort(port, readPort(port, portConfigInputs[port]), true);
  }
}

void outputPort(byte portNumber, byte portValue, byte forceSend)
{
  //Serial.print("outputPort[");
  //Serial.print(portNumber);
  //Serial.println("]");
  // pins not configured as INPUT are cleared to zeros
  portValue = portValue & portConfigInputs[portNumber];
  // only send if the value is different than previously sent
  if (forceSend || previousPINs[portNumber] != portValue) {
    previousPINs[portNumber] = portValue;
    //parseCommand(SET_DIGITAL_PIN_VALUE, (pin & 0x7F), (readvalue != 0));
    if (myMqtt.isConnected()) {
      parseCommand(REPORT_STATE, reportPINs[(portNumber & 0xF)], portValue);
    }else{
      Serial.println("MQTT Server not connected!!!");
      Serial.print("Digital Reading: ");
      Serial.println(portValue);
    }
  }
}

void reportAnalog()
{
  byte pin, analogPin;
  /* ANALOGREAD - do all analogReads() at the configured sampling interval */
  for (pin = 0; pin < TOTAL_PINS; pin++) {
    if (IS_PIN_ANALOG(pin) && getPinMode(pin) == PIN_MODE_ANALOG) {
      analogPin = PIN_TO_ANALOG(pin);
      if (analogInputsToReport & (1 << analogPin)) {
        uint16_t readvalue = analogRead(analogPin);
        if ( (0xF >= pin) && (0x3FFF >= readvalue) ) {
          if (myMqtt.isConnected()) {
            parseCommand(REPORT_STATE, pin, readvalue);
          } else{
            Serial.println("MQTT Server not connected!!!");
            Serial.print("Analog Reading: ");
            Serial.println(readvalue);
          }
        }
      }
    }
  }
}

void reportAnalog(byte analogPin, int value)
{
  if (analogPin < TOTAL_ANALOG_PINS) {
    if (value == 0) {
      analogInputsToReport = analogInputsToReport & ~ (1 << analogPin);
    } else {
      analogInputsToReport = analogInputsToReport | (1 << analogPin);
      // prevent during system reset or all analog pin values will be reported
      // which may report noise for unconnected analog pins
      if (!isResetting) {
        // Send pin value immediately. This is helpful when connected via
        // ethernet, wi-fi or bluetooth so pin states can be known upon
        // reconnecting.
        uint16_t readvalue = analogRead(analogPin);
        if (myMqtt.isConnected()) {
            parseCommand(REPORT_STATE, analogPin, readvalue);
          } else{
            Serial.println("MQTT Server not connected!!!");
            Serial.print("Analog Reading: ");
            Serial.println(readvalue);
          }
      }
    }
  }
}

String prepareResponse(byte pin, String readvalue, const char *msgCategory) {
  StaticJsonBuffer<500> jsonBuffer;
  JsonObject& jsonObj = jsonBuffer.createObject();
  jsonObj["type"] = "update";
  jsonObj["category"] = msgCategory;
  jsonObj["sync"] = millis();
  jsonObj["access"] = "public";
  jsonObj["data"] = jsonBuffer.createObject();
  jsonObj["data"]["pin"] = pin;
  jsonObj["data"]["updatedValue"] = readvalue;

  String msgval("Atualização de Valor: ");
  msgval+=readvalue;
  jsonObj["data"]["message"] = msgval;
  jsonObj["data"]["profile"] = CLIENT_ID;
  int len = jsonObj.measureLength();

  String buffer;
  jsonObj.printTo(buffer);
  return buffer;
}

boolean verifyValidMessage(JsonObject& jsonObj){
  if (jsonObj["type"]!="action") return false;


  long sync = jsonObj["sync"];
  Serial.println(sync);
  if (sync<lastAction) return false;
  lastAction = sync;

  String category = jsonObj["category"];
  if (category=="command"){
    String msgprofile = jsonObj["data"]["profile"];
    /*for(String profile : profiles) {
      if (msgprofile == profile) {
        return true;
      }
    }*/
    if (profiles.find(msgprofile)) {
       return true;
    }
  }else if (category =="profiles") return true;

  return false;
}

/**
 * Read the action from the input message. If the value is not = -1, pass it on to parseCommand(byte)
 */
void processInputData(String data)
{
  DynamicJsonBuffer jsonBuffer;
  JsonObject& jsonmsg = jsonBuffer.parseObject(data);
  // Test if parsing succeeds.
  if (!jsonmsg.success()){
    Serial.println("parseObject() failed");
    return;
  }

  if (!verifyValidMessage(jsonmsg)){
    Serial.println("Invalid Message");
    return;
  }
Serial.println("Processing");
  if (jsonmsg["category"] == "command"){
    byte msgpin = jsonmsg["data"]["pin"];
    if (jsonmsg["data"]["commands"].is<JsonArray&>()){
      JsonObject& arrCommands =  jsonmsg["data"]["commands"];
      for (auto commandItem : arrCommands)
      {
        //Serial.println(it->key);
        //JsonObject& msgCommand = it->value
        int command = commandItem.value["command"];
        int msgmode = commandItem.value["mode"];
        parseCommand(command, msgpin, msgmode);
      }
    }
  }else if (jsonmsg["category"] == "profiles"){
    int index = 0;
    if (jsonmsg["data"].is<JsonArray&>()){
      JsonObject& arrProfiles =  jsonmsg["data"];
      for (auto profile : arrProfiles){
        Serial.println(profile.value["profile"].as<String>());
        //attachProfile(profile.value["profile"].as<String>(), profile.value["publish"], profile.value["subscribe"]);
        index++;
      }
    }
  }
}

byte getPinMode(byte pin)
{
  return pinConfig[pin];
}

void setPinMode(byte pin, int mode){
  if (pinConfig[pin] == PIN_MODE_IGNORE)
    return;

  Serial.print("is pin: ");
  Serial.println(IS_PIN_DIGITAL(pin));
  Serial.print("is mode: ");
    Serial.println(mode);
  if (IS_PIN_DIGITAL(pin)) {
    if (mode == INPUT || mode == PIN_MODE_PULLUP) {
      portConfigInputs[pin / 8] |= (1 << (pin & 7));
      reportPINs[pin / 8] = pin;
    } else {
      portConfigInputs[pin / 8] &= ~(1 << (pin & 7));
      reportPINs[pin / 8] = pin;
    }
    Serial.print("portConfigInputs: ");
    Serial.println(portConfigInputs[pin / 8]);
    if (mode != PIN_MODE_SERVO) {
      if (servoPinMap[pin] < MAX_SERVOS && servos[servoPinMap[pin]].attached()) {
        detachServo(pin);
      }
    }
  }

  setPinState(pin, 0);

  switch (mode) {
    case PIN_MODE_ANALOG:
      if (IS_PIN_ANALOG(pin)) {
        if (IS_PIN_DIGITAL(pin)) {
          pinMode(PIN_TO_DIGITAL(pin), PIN_MODE_INPUT);    // disable output driver
#if ARDUINO <= 100
          // deprecated since Arduino 1.0.1 - TODO: drop support in Firmata 2.6
          digitalWrite(PIN_TO_DIGITAL(pin), LOW); // disable internal pull-ups
#endif
        } else {
          if (IS_PIN_PWM(pin)) {
            pinMode(PIN_TO_PWM(pin), PIN_MODE_INPUT);    // disable output driver
          }
        }
        pinConfig[pin] = PIN_MODE_ANALOG;
        reportAnalog(PIN_TO_ANALOG(pin), true); // turn on/off reporting
      }
      break;
    case PIN_MODE_INPUT:
      if (IS_PIN_DIGITAL(pin)) {
        pinMode(PIN_TO_DIGITAL(pin), PIN_MODE_INPUT);    // disable output driver
#if ARDUINO <= 100
        // deprecated since Arduino 1.0.1 - TODO: drop support in Firmata 2.6
        digitalWrite(PIN_TO_DIGITAL(pin), LOW); // disable internal pull-ups
#endif

        Serial.print("pin: ");
        Serial.println(PIN_TO_DIGITAL(pin));
        pinConfig[PIN_TO_DIGITAL(pin)] = PIN_MODE_INPUT;
        reportDigital(pin / 8, true);
      }
      break;
    case PIN_MODE_PULLUP:
      if (IS_PIN_DIGITAL(pin)) {
        pinMode(PIN_TO_DIGITAL(pin), INPUT_PULLUP);
        pinConfig[pin] = PIN_MODE_PULLUP;
        setPinState(pin, 1);
        reportDigital(PIN_TO_DIGITAL(pin), true);
      }
      break;
    case PIN_MODE_OUTPUT:
      if (IS_PIN_DIGITAL(pin)) {
        if (getPinMode(pin) == PIN_MODE_PWM) {
          // Disable PWM if pin mode was previously set to PWM.
          digitalWrite(PIN_TO_DIGITAL(pin), LOW);
        }
        pinMode(PIN_TO_DIGITAL(pin), PIN_MODE_OUTPUT);
        pinConfig[pin] = PIN_MODE_OUTPUT;
      }
      break;
    case PIN_MODE_PWM:
      if (IS_PIN_PWM(pin)) {
        pinMode(PIN_TO_PWM(pin), PIN_MODE_OUTPUT);
        analogWrite(PIN_TO_PWM(pin), 0);
        pinConfig[pin] = PIN_MODE_PWM;
      }
      break;
    case PIN_MODE_SERVO:
      if (IS_PIN_DIGITAL(pin)) {
        pinConfig[pin] = PIN_MODE_SERVO;
        if (servoPinMap[pin] == 255 || !servos[servoPinMap[pin]].attached()) {
          // pass -1 for min and max pulse values to use default values set
          // by Servo library
          attachServo(pin, -1, -1);
        }
      }
      break;
    case PIN_MODE_I2C:
      if (IS_PIN_I2C(pin)) {
        // mark the pin as i2c
        // the user must call I2C_CONFIG to enable I2C for a device
        pinConfig[pin] = PIN_MODE_I2C;
      }
      break;
    default:
      parseCommand(STRING_ERROR, pin, "Unknown pin mode");
  }

}

int getPinState(byte pin)
{
  return pinState[pin];
}

void setPinState(byte pin, int state)
{
  pinState[pin] = state;
}

void setPinValue(byte pin, int value)
{
  if (pin < TOTAL_PINS && IS_PIN_DIGITAL(pin)) {
    if (getPinMode(pin) == PIN_MODE_OUTPUT) {
      digitalWrite(PIN_TO_DIGITAL(pin), value);
    }
  }
}

void systemReset(void)
{
  isResetting = true;
  tickOccured = false;
  user_init();

  for (byte i = 0; i < TOTAL_PINS; i++) {
    reportPINs[i] = false;    // by default, reporting off
    portConfigInputs[i] = 0;  // until activated
    previousPINs[i] = 0;
  }

  for (byte i = 0; i < TOTAL_PINS; i++) {
    // need to ignore pins 1 and 3 when using an ESP8266 board
    if (i == 1 || i == 3) continue;

    if (IS_PIN_ANALOG(i)) {
      // turns off pull-up, configures everything
      setPinMode(i, PIN_MODE_ANALOG);
    } else if (IS_PIN_DIGITAL(i)) {
      // sets the output to 0, configures portConfigInputs
      setPinMode(i, PIN_MODE_OUTPUT);
    }
    // by default, do not report any analog inputs
    servoPinMap[i] = 255;
  }

  analogInputsToReport = 0;
  detachedServoCount = 0;
  servoCount = 0;
  reset();

  isResetting = false;
}

void ignorePins()
{
  #ifdef IS_IGNORE_PIN
  for (byte i = 0; i < TOTAL_PINS; i++) {
    if (IS_IGNORE_PIN(i)) {
      setPinMode(i, PIN_MODE_IGNORE);
    }
  }
#endif
}

/* utility functions */
void wireWrite(byte data)
{
#if ARDUINO >= 100
  Wire.write((byte)data);
#else
  Wire.send(data);
#endif
}

byte wireRead(void)
{
#if ARDUINO >= 100
  return Wire.read();
#else
  return Wire.receive();
#endif
}

void attachProfile(String profile, boolean publish, boolean subscribe)
{
  Serial.println(profile);
  profiles.push_back(profile, publish, subscribe);
  Serial.println(profiles.length());

}

boolean detachProfile(String detachedProfile)
{
  if (profiles.empty()) return false;
  int count = 0;
  if (profiles.pull_item(detachedProfile)) {
      return true;
  }
  return false;
}

void attachServo(byte pin, int minPulse, int maxPulse)
{
  if (servoCount < MAX_SERVOS) {
    // reuse indexes of detached servos until all have been reallocated
    if (detachedServoCount > 0) {
      servoPinMap[pin] = detachedServos[detachedServoCount - 1];
      if (detachedServoCount > 0) detachedServoCount--;
    } else {
      servoPinMap[pin] = servoCount;
      servoCount++;
    }
    if (minPulse > 0 && maxPulse > 0) {
      servos[servoPinMap[pin]].attach(PIN_TO_DIGITAL(pin), minPulse, maxPulse);
    } else {
      servos[servoPinMap[pin]].attach(PIN_TO_DIGITAL(pin));
    }
  } else {
    Serial.println("Max servos attached");
  }
}

void detachServo(byte pin)
{
  servos[servoPinMap[pin]].detach();
  // if we're detaching the last servo, decrement the count
  // otherwise store the index of the detached servo
  if (servoPinMap[pin] == servoCount && servoCount > 0) {
    servoCount--;
  } else if (servoCount > 0) {
    // keep track of detached servos because we want to reuse their indexes
    // before incrementing the count of attached servos
    detachedServoCount++;
    detachedServos[detachedServoCount - 1] = servoPinMap[pin];
  }

  servoPinMap[pin] = 255;
}

void printWifiStatus() {
  Serial.println("");
  if (WiFi.status() != WL_CONNECTED) {
    Serial.print( "WiFi connection failed. Status value: " );
    Serial.println( WiFi.status() );
    Serial.println("Only serial interface will be available to setup AP settings");
  } else {
    Serial.println("WiFi connected");
    Serial.print( "SSID: " );
    Serial.println( WiFi.SSID() );

    // print your WiFi shield's IP address:
    Serial.print( "IP Address: " );
    IPAddress ip = WiFi.localIP();
    Serial.println( ip );

    // print the received signal strength:
    Serial.print( "signal strength (RSSI): " );
    long rssi = WiFi.RSSI();
    Serial.print( rssi );
    Serial.println( " dBm" );
  }
  Serial.flush();
}

void hostConnectionCallback(byte state)
{
  switch (state) {
    case HOST_CONNECTION_CONNECTED:
      Serial.println( "TCP connection established" );
      break;
    case HOST_CONNECTION_DISCONNECTED:
      Serial.println( "TCP connection disconnected" );
      break;
  }
}

void initTransport()
{
  Serial.print( "StandardFirmataWiFi will attempt a WiFi connection " );
#if defined(WIFI_101)
  Serial.printLN( "using the WiFi 101 library." );
#elif defined(ARDUINO_WIFI_SHIELD)
  Serial.println( "using the legacy WiFi library." );
#elif defined(ESP8266_WIFI)
  Serial.println( "using the ESP8266 WiFi library." );
#elif defined(HUZZAH_WIFI)
  Serial.println( "using the HUZZAH WiFi library." );
#endif  //defined(WIFI_101)

/*#ifdef LOCAL_IP_ADDRESS
  Serial.print( "Using static IP: " );
  Serial.println( local_ip );
#if defined(ESP8266_WIFI) || (defined(SUBNET_MASK) && defined(GATEWAY_IP_ADDRESS))
  //WiFi.config(local_ip, gateway, subnet);
  stream.config( local_ip , gateway, subnet );
#else
  //WiFi.config(local_ip);
  stream.config( local_ip );
#endif

#else
  Serial.println( "IP will be requested from DHCP ..." );
#endif

  stream.attach(hostConnectionCallback);
*/
  WiFi.begin(ssid, password);

/*
#if defined(WIFI_WPA_SECURITY)
  Serial.print( "Attempting to connect to WPA SSID: " );
  Serial.println(ssid);
  stream.begin(ssid, password);
#else                          //OPEN network
  Serial.print( "Attempting to connect to open SSID: " );
  Serial.println(ssid);
  stream.begin(ssid);
#endif //defined(WIFI_WPA_SECURITY)
*/
  Serial.println( "WiFi setup done" );

  while (WiFi.status() != WL_CONNECTED && ++connectionAttempts <= MAX_CONN_ATTEMPTS) {
    delay(500);
    Serial.print(".");
  }

  printWifiStatus();

  // Start MQTT Server connection

  // setup callbacks
  Serial.println("setting MQTT server config");
  myMqtt.onConnected(myConnectedCb);
  myMqtt.onDisconnected(myDisconnectedCb);
  myMqtt.onPublished(myPublishedCb);
  myMqtt.onTimeout(myTimeoutCb);
  myMqtt.onData(myDataCb);
  //myMqtt.initLWT(will_topic.c_str(), OFFLINE_MESSAGE, MQTT_QOS_1, MQTT_CONN_WILLRETAIN);
  myMqtt.connect();
  
  delay(10);
}
boolean elapsed()
{
  currentMillis = millis();
  /*
  Serial.print(currentMillis);
  Serial.print(" - ");
  Serial.print(previousMillis);
  Serial.print(" - ");
  Serial.println(samplingInterval);
  */
  if (currentMillis - previousMillis > samplingInterval) {
    previousMillis += samplingInterval;
    if (currentMillis - previousMillis > samplingInterval)
      previousMillis = currentMillis - samplingInterval;
    return true;
  }
  return false;
}

void reset()
{
  previousMillis = millis();
  samplingInterval = 19;
}

void setSamplingInterval(int interval)
{
  samplingInterval = interval;
}
void digitalWriteCallback(byte port, int state)
{
  byte pin, lastPin, pinValue, mask = 1, pinWriteMask = 0;

  if (port < TOTAL_PINS) {
    // create a mask of the pins on this port that are writable.
    lastPin = port * 8 + 8;
    if (lastPin > TOTAL_PINS) lastPin = TOTAL_PINS;
    for (pin = port * 8; pin < lastPin; pin++) {
      // do not disturb non-digital pins (eg, Rx & Tx)
      if (IS_PIN_DIGITAL(pin)) {
        // do not touch pins in PWM, ANALOG, SERVO or other modes
        if (getPinMode(pin) == OUTPUT || getPinMode(pin) == PIN_MODE_INPUT) {
          pinValue = ((byte)state & mask) ? 1 : 0;
          if (getPinMode(pin) == PIN_MODE_OUTPUT) {
            pinWriteMask |= mask;
          } else if (getPinMode(pin) == INPUT && pinValue == 1 && getPinState(pin) != 1) {
            // only handle INPUT here for backwards compatibility
#if ARDUINO > 100
            pinMode(pin, INPUT_PULLUP);
#else
            // only write to the INPUT pin to enable pullups if Arduino v1.0.0 or earlier
            pinWriteMask |= mask;
#endif
          }
          setPinState(pin, pinValue);
        }
      }
      mask = mask << 1;
    }
    writePort(port, (byte)state, pinWriteMask);
  }
}

void writeStateDigital(byte pin, int value)
{
  Serial.print("Pin: ");
  Serial.println(pin);
  Serial.print("value: ");
  Serial.println(value);
  if (pin < TOTAL_PINS && IS_PIN_DIGITAL(pin)) {
    if (getPinMode(pin) == PIN_MODE_OUTPUT) {
      setPinState(pin, value);
      digitalWrite(PIN_TO_DIGITAL(pin), value);
    }
  }
}

void writeStateAnalog(byte pin, int state)
{
  if (pin < TOTAL_PINS) {
    switch (getPinMode(pin)) {
      case PIN_MODE_SERVO:
        if (IS_PIN_DIGITAL(pin))
          servos[servoPinMap[pin]].write(state);
        setPinState(pin, state);
        break;
      case PIN_MODE_PWM:
        if (IS_PIN_PWM(pin))
          analogWrite(PIN_TO_PWM(pin), state);
        setPinState(pin, state);
        break;
    }
  }
}
