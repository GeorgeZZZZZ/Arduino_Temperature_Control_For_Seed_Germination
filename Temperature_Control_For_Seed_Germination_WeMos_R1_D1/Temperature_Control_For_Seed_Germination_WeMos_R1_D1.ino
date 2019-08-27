#include <ESP8266WiFi.h>
#include <OneWire.h> 
#include <DallasTemperature.h>

//#define HOSTNAME "ControlBoard" // useless
#define MAX_SRV_CLIENTS 1
#define MAX_Buff_Length 5
//#define LED_BUILTIN 2 // pin 2 for led

/***********************************************
 * 2019.07.13 A template of TCP server Station 
 * 
 * use bool UseNetWorkFunciton to start net work function, if this is false then neither softAp or TCP server going to work
 * use bool UsingSoftAP to start ap mode or connect to a wifi
 * use bool tcpMessageDebug to debug message format
 * the message format should be (header)content;(header)content;[CR][LF]
 * 
 * use method HeaderAnalysis() to recognize header label
 * use method ContentAnalysis() to read command after header
 * use method CommandApplyAtEnd() to give command after message end
 * use method CleanCommandTemp() to clean command cache after apply commands
 *************************************************/

/**********************************************
 * using board WeMos R1 D1
 * using DS18B20
 * use float TargetTemperature to set target temperature
 * use float TemperatureDifference to set temperature difference
 * 
 * 
 *********************************************/
enum TemperatureError{
  No_Temperature_Error
  , Lost_sensor
  , Heat_Cool_Reversed
  , No_Heat_Cool_AtAll
  , Lost_Fan
};
TemperatureError CurrentTemperatureError = No_Temperature_Error;
TemperatureError OldTemperatureError = CurrentTemperatureError;

enum ThermalChamberControlOrder{
  Chamber_Off
  , Chamber_Cooling
  , Chamber_Heating
};

typedef struct
{
  String result;
  bool isEnd;
  bool err;
}ProcessContentResult;
ProcessContentResult PCResult;

typedef struct
{
  byte result;
  bool isEnd;
  bool err;
}ProcessEndingResult;
ProcessEndingResult PEResult;

// is not going to use net working function for debug run just turn off this value 
bool UseNetWorkFunciton = false;
// if using soft ap then set this value to true, otherwrise set to false
bool UsingSoftAP = false;

const char* ssid = "ThermalChamberControlStation";
const char* password = "qwertyui";

IPAddress local_IP(50,0,0,50); 
IPAddress gateway(50,0,0,1);
IPAddress subnet(255,255,255,0);

WiFiServer server(23);
WiFiClient serverClients[MAX_SRV_CLIENTS];
/*
const char* testServerIP = "10.0.0.100";
int testPort = 23;
WiFiClient client;
bool bConnected = false;
*/

// for state machine, 'const' make the variable read only
const uint8_t sm_pi_begin = -10;
uint8_t sm_pi_state = sm_pi_begin; //  sate machine for process incomming data
const uint8_t sm_pi_headerAnalyse = 0; // state machine processing incoming data, analyse the header
const uint8_t sm_pi_clearBeforeGoBack = 1; // state machine processing incoming data, analyse the header
const uint8_t sm_pi_error = 5;
const uint8_t sm_pi_contentAnalyse = 10; // analyse content after header
const uint8_t sm_pi_workingFine = 11;
const uint8_t sm_pi_fullEnding = 20;
const uint8_t sm_pi_EndingStage_0 = 21;
const uint8_t sm_pi_EndingStage_1 = 22;


char pi_buff[MAX_Buff_Length];
uint8_t pi_bufferCount = 0;

// timer cache
bool TempBehaveMonitor_TimerRollover = false;
unsigned long TempBehaveMonitor_TimerCache = 0;

bool TempSensorFrequency_TimerRollover = false;
unsigned long TempSensorFrequency_TimerCache = 0;

bool serialMessageTimerRollover = false;
unsigned long SerialMessagePrintTimerCache = 0;
bool ledOffTimerRollover = false;
unsigned long ledOffTimerCache = 0;  // use for led control
bool ledOnTimerRollover = false;
unsigned long ledOnTimerCache = 0;
/////******

/// led time control, can be change by current status
unsigned int LEDoffResetTime = 1900; // the time for led to turn off
unsigned int LEDonResetTime = 100;  // the time for led to turn on
bool LEDCycleDone = false; // a one machine cycle flag indicate led has finished one turn of On and Off 
//

///* put pin definition here 
uint8_t TemperatureReadPin = 13; // D7, WeMos R1 D1
uint8_t ThermalChamberCoolPin_Positive = 5; // D3, WeMos R1 D1
uint8_t ThermalChamberCoolPin_Negative = 4; // D4, WeMos R1 D1
uint8_t ThermalChamberHeatPin_Positive = 14; // D5, WeMos R1 D1
uint8_t ThermalChamberHeatPin_Negative = 12; // D6, WeMos R1 D1
//*/

bool valueInDefultFlag = 0;  // set this value in 1 if machine restart

/********************************************************************/
// DS18B20 read pin
#define ONE_WIRE_BUS TemperatureReadPin
/********************************************************************/
// Setup a oneWire instance to communicate with any OneWire devices  
// (not just Maxim/Dallas temperature ICs) 
OneWire oneWire(ONE_WIRE_BUS); 
/********************************************************************/
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature TemperatureSensor(&oneWire);
/********************************************************************/ 
float CurrentTemperature = -300;
float TargetTemperature = 23;
float TemperatureDifference = 1;  // 12305 is tooooo weak
bool TurnOn = true;  // debug now, always on, should be false and control by tcp command



// debug mode define
bool tcpMessageDebug = true;

//-----------------

void setup() {
  // initiallize values
  
  valueInDefultFlag = 1;
  // end of initiallize value
  
  Serial.begin(115200); // initiallize serial for debug
  //Serial.setDebugOutput(true);  // dont know why when this enabled it will constantly give can not connect to wifi
  Serial.println(""); // print nothing but lf
  
  if (UseNetWorkFunciton)
  {  
    Serial.print("Initialize network...");
    if (UsingSoftAP)
    {
      UseSoftAP();
    }
    else
    {
      ConnectToExistentWIFI();
    }

    Serial.print("Initialize TCP server...");
    server.begin(); // start to run TCP server
    server.setNoDelay(true);
    uint8_t serverStatus = server.status();
    if (serverStatus == 1) Serial.println("Done.");
    else
    {
      Serial.println("Fail.");
      Serial.print("server statue: ");
      Serial.println(serverStatus);
    }
  }
  else
  {// if is not using wifi function at all
    Serial.println("Skip network and TCP server initialization.");
    WiFi.mode(WIFI_OFF);  // turn off wifi mode
    // maybe deep sleep is better? did not test, but this works for me
  }

  Serial.println("Initialize Pin and LED...");
  
  pinMode(LED_BUILTIN, OUTPUT); // initiallize LED
  digitalWrite(LED_BUILTIN, HIGH);

  pinMode(ThermalChamberCoolPin_Positive, OUTPUT); // initialize pin out is necessary, althought don't initialize still can use pwm function but that require pwm out put low and high to work
  pinMode(ThermalChamberCoolPin_Negative, OUTPUT); // if no initialization and directly use pwm to control will not get correct value
  pinMode(ThermalChamberHeatPin_Positive, OUTPUT);
  pinMode(ThermalChamberHeatPin_Negative, OUTPUT);

  Serial.println("Initializing Temperature Sensor...");
  TemperatureSensor.begin(); 

  Serial.println("All initialization Done. Start Running...");
}

void loop() {

  LEDManager(); // manage LED
  
  if (UseNetWorkFunciton)
  {
    TCPClientsManagement(); // manage clients and process incoming messages
  }

  TemperatureReading();

  CentralControl();

  MessageToBePrintToSerialConstantly();  
  //DebugAndTest ();
}

void TCPClientsManagement ()
{
  uint8_t i;  // "uint8_t" its shorthand for: a type of unsigned integer of length 8 bits
  
  // if client is connected in
  if (server.hasClient())
  {
    // Looking for new client connection
    for(i = 0; i < MAX_SRV_CLIENTS; i++)
    {
      if (!serverClients[i] || !serverClients[i].connected())
      {
        if(serverClients[i]) serverClients[i].stop();
        serverClients[i] = server.available();
        Serial.print("New client: "); Serial.println(i);
        continue;
      }
    }
    WiFiClient serverClient = server.available();
    serverClient.stop();  // why immediately close client?
  }

  // check process incoming message, look if its correct format
  // correct format, should be: (Header)Content;[CR][LF]
  for(i = 0; i < MAX_SRV_CLIENTS; i++)
  {
    if (serverClients[i] && serverClients[i].connected())
    {
      Socket_Communication (i); // monitor cache for incoming data and process it 
    }
  }
}

void ConnectToExistentWIFI()
{
  WiFi.config(local_IP, gateway, subnet); // static IP address
  WiFi.hostname("MyESPName");

  Serial.print("Start to connect to wifi: ");
  Serial.println(ssid);
  //WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password); // start connect router
  
  Serial.println("\nConnecting to router.");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println(WiFi.status() == WL_CONNECTED? "Ready" : "Failed!");
}

void UseSoftAP()
{
  Serial.print("Setting soft-AP configuration ... ");
  Serial.println(WiFi.softAPConfig(local_IP, gateway, subnet) ? "Ready" : "Failed!");
  Serial.print("Setting soft-AP: '");
  Serial.print(ssid);
  Serial.print("'...");
  boolean result = WiFi.softAP(ssid, password);
  if(result == true)
  {
    Serial.println("Ready");
  }
  else
  {
    Serial.println("Failed!");
  }

  Serial.print("Soft-AP IP address = ");
  Serial.println(WiFi.softAPIP());
}

void LEDManager ()
{
  LEDCycleDone = false; // reset flag inidcate new led on/off cycle has started
  digitalWrite(LED_BUILTIN, HIGH); // turn off LED
  if (ledOffTimerCache == 0) ledOffTimerCache = millis(); // start countting led turnning off time
  if (millis() - ledOffTimerCache < 0)  // rollover preventive, if rollover
  { 
    ledOffTimerCache = 4294967295 - ledOffTimerCache; // calculate already passed time
    ledOffTimerRollover = true;
  }
  if (TimeCounter (ledOffTimerCache, LEDoffResetTime, ledOffTimerRollover))  // off time reached
  {
    digitalWrite(LED_BUILTIN, LOW); // turn on LED
    if (ledOnTimerCache == 0)  ledOnTimerCache = millis();  // start countting led turnning on time
    if (millis() - ledOnTimerCache < 0)
    { 
      ledOnTimerCache = 4294967295 - ledOnTimerCache;
      ledOnTimerRollover = true;
    }
    if (TimeCounter (ledOnTimerCache, LEDonResetTime, ledOnTimerRollover))  // on time reached
    {
      LEDCycleDone = true;  // one led on/off cycle finished 
      ledOnTimerCache = ledOffTimerCache = 0; // reset timer
      ledOnTimerRollover = ledOffTimerRollover = false;
    }
  }
}

void Socket_Communication (int _clientNum)
{
  while(serverClients[_clientNum].available()) // loop when there are data coming
  {
    char _num;
    switch (sm_pi_state)
    {
      case sm_pi_begin:
        _num = serverClients[_clientNum].read();
        if (_num == '(')  sm_pi_state = sm_pi_headerAnalyse;
        else if (_num == '[') sm_pi_state = sm_pi_fullEnding;
        else
        {
          sm_pi_state = sm_pi_error;  // uncorrenct format data
          Serial.print("PI State Machine error: sm_pi_state");
        }
        //Serial.print("state change: "); // keep this code for future debug
        //Serial.println(sm_pi_state);
      break;
      
      case sm_pi_headerAnalyse: // analyse the header to understander what command comes
        ProcessIncoming_ContentExtract (_clientNum);
        if (PCResult.isEnd && !PEResult.isEnd && !PCResult.err) // when tacking header should not receive ending mark
        {
          // extract header from incoming message
          if (HeaderAnalysis(PCResult.result) != sm_pi_error) sm_pi_state = sm_pi_contentAnalyse;
          else sm_pi_state = sm_pi_error;
        }
        else if (PCResult.err || PEResult.isEnd)
        {
          sm_pi_state = sm_pi_error;
          Serial.println("PI State Machine error: sm_pi_headerAnalyse - 01.");
        }
      break;

      case sm_pi_contentAnalyse: // analyse the content after header to understander what is the command
        ProcessIncoming_ContentExtract (_clientNum);
        if (PCResult.isEnd && !PEResult.isEnd && !PCResult.err) // when tacking header should not receive ending mark
        {
          // extract content from incoming message then go back to beginning for next command or ending
          if (ContentAnalysis(PCResult.result) != sm_pi_error) sm_pi_state = sm_pi_clearBeforeGoBack;
          else sm_pi_state = sm_pi_error;
        }
        else if (PCResult.err || PEResult.isEnd)
        {
          sm_pi_state = sm_pi_error;
          Serial.println("PI State Machine error: sm_pi_contentAnalyse - 01.");
        }
      break;
      
      case sm_pi_clearBeforeGoBack: // use to reset something
        CleanPISturct ();  //reset structure
        sm_pi_state = sm_pi_begin;
      break;
      
      case sm_pi_error: // if is not correct format or other error
        serverClients[_clientNum].flush();  // flush cache
        CleanCommandTemp (); // reset motor command temp
        CleanPISturct ();  //reset structures
        sm_pi_state = sm_pi_begin;
        Serial.println("PI State Machine error: Error occured, Format not correct.");
      break;
      
      case sm_pi_fullEnding:
        ProcessIncoming_ContentExtract (_clientNum);
        if (PEResult.isEnd && !PCResult.isEnd && !PEResult.err)
        {
          if (PEResult.result == 13)  sm_pi_state = sm_pi_EndingStage_0;// if char is CR
          else
          {
            sm_pi_state = sm_pi_error;
            Serial.println("PI State Machine error: sm_pi_fullEnding_1.");
          }
        }
        else if (PEResult.err || PCResult.isEnd)
        {
          sm_pi_state = sm_pi_error;
          Serial.println("PI State Machine error: sm_pi_fullEnding_0.");
        }
      break;

      case sm_pi_EndingStage_0:
        _num = serverClients[_clientNum].read();
        if (_num == '[') sm_pi_state = sm_pi_EndingStage_1;
        else
        {
          sm_pi_state = sm_pi_error;  // uncorrenct format data
          Serial.print("PI State Machine error: sm_pi_EndingStage_0");
        }
      break;
      
      case sm_pi_EndingStage_1:
        ProcessIncoming_ContentExtract (_clientNum);
        if (PEResult.isEnd && !PCResult.isEnd && !PEResult.err)
        {
          if (PEResult.result == 10) // if char is LF
          {

            /*
            Serial.println("Good, get end");
            */
            CommandApplyAtEnd ();
            CleanCommandTemp (); // reset motor command temp
            sm_pi_state = sm_pi_clearBeforeGoBack;  // prepare go back to beginning
          }
          else
          {
            sm_pi_state = sm_pi_error;
            Serial.println("PI State Machine error: sm_pi_EndingStage_1_1.");
          }
        }
        else if (PEResult.err || PCResult.isEnd)
        {
          sm_pi_state = sm_pi_error;
          Serial.println("PI State Machine error: sm_pi_EndingStage_1_0.");
        }
      break;
    }
  }
  if (PCResult.err || PEResult.err)
  {
    CleanPISturct ();  //reset structure
    Serial.println("PI outside error: structure err..");
  }
}

void ProcessIncoming_ContentExtract (int _clientNum) // extract data from incoming message before break mark
{
  ProcessContentResult* _cResult = &PCResult;
  ProcessEndingResult* _eResult = &PEResult;
  
  char _num;
  while (serverClients[_clientNum].available()) // loop until package finish
  {
    _num = serverClients[_clientNum].read();
    if (_num == ')' ||  _num == ';')
    {
      _cResult->result = String (pi_buff);
      _cResult->isEnd = true;
      CleanPIBuff (); // got reuslt, can reset buffer
      return;
    }
    else if (_num == ']')
    {
      _eResult->result = pi_buff[pi_bufferCount - 1];
      _eResult->isEnd = true;
      CleanPIBuff (); // got reuslt, can reset buffer
      return;
    }
    pi_buff[pi_bufferCount] = _num;
    pi_bufferCount ++;
    if (pi_bufferCount > 5)
    {
      _cResult->err = true; // if content get more then 5 letter
      CleanPIBuff ();
      return;
    }
  }
  /*
  _cResult->err = true; // content result and this command should not be execute
  _eResult->err = true; // content result and this command should not be execute
  CleanPIBuff ();
  Serial.println("PI content error: Error occured, content process unexpected failed");
  */
}

void CleanCommandTemp ()
{
  //leftMotorTemp = rightMotorTemp = midMotorTemp = speedTemp = -999;  // clean temp stroge valuable
}

void CleanPISturct () 
{
  PCResult = ProcessContentResult ();  //reset structure
  PEResult = ProcessEndingResult ();  //reset structure
}

void CleanPIBuff ()
{
  memset(pi_buff, 0, sizeof(pi_buff));  // zero the buffer
  pi_bufferCount = 0;
}

int Convert_String_to_Int (String _str)
{
  char _takeNum [_str.length() + 1]; // don't know why toCharArray will not return full lenght number if array length exactly same as string length. May be first position of array has been occupied by other usage, so add 1 more position.
  _str.toCharArray (_takeNum, sizeof (_takeNum)); // string to array
  
  return atoi(_takeNum);  // array to int
}

uint8_t HeaderAnalysis (String _str)
{
  if (!tcpMessageDebug)
  {
    if (_str == "SP")  return sm_pi_workingFine;
    else
    {
      return sm_pi_error;  // uncorrenct format data
      Serial.println("PI State Machine error: sm_pi_headerAnalyse - 02.");
    }
  }
  else
  {
    Serial.println("Is in Header analysis!!");
    return sm_pi_workingFine;
  }
}

uint8_t ContentAnalysis (String _str)
{
  if (!tcpMessageDebug)
  {
    if (_str == "SP")  return sm_pi_workingFine;
    else
    {
      return sm_pi_error;  // uncorrenct format data
      Serial.println("PI State Machine error: sm_pi_contentAnalyse - 02.");
    }
  }
  else
  {
    Serial.println("Is in content analysis!!");
    return sm_pi_workingFine;
  }
}

void CommandApplyAtEnd ()
{
  if (!tcpMessageDebug)
  {
  }
  else
  {
    Serial.println("Is ending and appling commands!!");
  }
}

void MessageToBePrintToSerialConstantly()
{
  if (SerialMessagePrintTimerCache == 0)  SerialMessagePrintTimerCache = millis();  // start countting led turnning on time
  if (millis() - SerialMessagePrintTimerCache < 0)
  { 
    SerialMessagePrintTimerCache = 4294967295 - SerialMessagePrintTimerCache;
    serialMessageTimerRollover = true;
  }
  if (TimeCounter (SerialMessagePrintTimerCache, 3000, serialMessageTimerRollover))  // sent message every 3s
  {
    SerialMessagePrintTimerCache = 0; // reset timer
    serialMessageTimerRollover = false;

    // write message below
    if (UseNetWorkFunciton)
    {
      if (UsingSoftAP)
      {// monitor how many connecter
        Serial.printf("Stations connected = %d\n", WiFi.softAPgetStationNum());
      }
      else
      {

      }
    }

    if (OldTemperatureError != CurrentTemperatureError)
    {
      OldTemperatureError = CurrentTemperatureError;
      Serial.print("Temperature Error Code Changed, New Error: ");
      Serial.println(OldTemperatureError);
    }

    switch (CurrentTemperatureError)
    {
    case No_Temperature_Error:
      // no error
      break;
    case Lost_sensor:
      Serial.println("Lost sensor, check connection plz!!");
      break;
    case Heat_Cool_Reversed:
      Serial.println("Heating become Cooling or Cooling become Heating, check connection plz!!!");
      break;
    case No_Heat_Cool_AtAll:
      Serial.println("Not Cooling or Heating at all, check connection plz!!");
      break;
    case Lost_Fan:
      /* */
      break;
    default:
      break;
    }
    
    // write message above
  }
}

byte TemperatureReadFrequency = 5; // 5 for 5s
bool okToReadTemp = true;
void TemperatureReading ()
{
  if (okToReadTemp)
  {
    okToReadTemp = false; 
    // call sensors.requestTemperatures() to issue a global temperature 
    // request to all devices on the bus 
    /********************************************************************/
    Serial.print("Requesting temperatures..."); 
    TemperatureSensor.requestTemperatures(); // Send the command to get temperature readings 
    Serial.println("DONE"); 
    /********************************************************************/
    CurrentTemperature = TemperatureSensor.getTempCByIndex(0);
    Serial.print("Temperature is: "); 
    Serial.println(CurrentTemperature); // Why "byIndex"?  
    // You can have more than one DS18B20 on the same bus.  
    // 0 refers to the first IC on the wire 
  }
  else
  {
    // a timer to control sensor read frequency
    unsigned int frequencyTime = TemperatureReadFrequency * 1000;

    if (TempSensorFrequency_TimerCache == 0) TempSensorFrequency_TimerCache = millis(); // start countting led turnning off time
    if (millis() - TempSensorFrequency_TimerCache < 0)  // rollover preventive, if rollover
    { 
      TempSensorFrequency_TimerCache = 4294967295 - TempSensorFrequency_TimerCache; // calculate already passed time
      TempSensorFrequency_TimerRollover = true;
    }
    if (TimeCounter (TempSensorFrequency_TimerCache, frequencyTime, TempSensorFrequency_TimerRollover))  // off time reached
    {
      okToReadTemp = true;
      TempSensorFrequency_TimerCache = 0;
      TempSensorFrequency_TimerRollover = false;
    }
  }

}
bool turnOn_Internal = false;
bool temperatureDone = false;
bool temperatureUp = false;
void CentralControl ()
{
  if (CurrentTemperature < -126) CurrentTemperatureError = Lost_sensor; // lost connection of the sensor
  else if (CurrentTemperatureError == Lost_sensor)
  {
    LEDoffResetTime = 1900; // change led behavior
    LEDonResetTime = 100;
    CurrentTemperatureError = No_Temperature_Error;
  }

  if (CurrentTemperatureError != No_Temperature_Error)
  { // if any error
    LEDoffResetTime = 2000; // change led behavior
    LEDonResetTime = 2000;
    CC_Emergency_Behavour(); 
    return;
  }

  if (TurnOn && !turnOn_Internal) 
  { // only run once at start
    turnOn_Internal = true;
    if (CurrentTemperature > TargetTemperature) temperatureUp = false; // if current temp is higher than target then start cooling
    else temperatureUp = true; // else start heating
  }
  else if (!TurnOn) turnOn_Internal = false;

  
  if (turnOn_Internal)
  {
    bool turnOn_Monitoring_Temperature = false;
    if (!temperatureUp) // if is cooling
    { 
      if (!temperatureDone)  // if not yet reach target temp
      {
        turnOn_Monitoring_Temperature = true;
        ThermalChamberControl(true, Chamber_Cooling); // start cooling
        if (CurrentTemperature < TargetTemperature) // if reach tar temp
        {
          temperatureDone = true;  // cooling done
        }
      }
      else if (CurrentTemperature >= TargetTemperature + TemperatureDifference) // if cooling done and temp rise higher than target and diff
      {
        temperatureDone = false; // start cooling again
      }
      else CC_Emergency_Behavour();
      
    }
    else if(temperatureUp) // if is heating
    {
      if (!temperatureDone)
      {
        turnOn_Monitoring_Temperature = true;
        ThermalChamberControl(true, Chamber_Heating);
        if (CurrentTemperature > TargetTemperature)
        {
          temperatureDone = true;
        }
      }
      else if (CurrentTemperature <= TargetTemperature - TemperatureDifference)
      {
        temperatureDone = false;
      }
      else CC_Emergency_Behavour();
    }
    TempBehaveMonitor(turnOn_Monitoring_Temperature, temperatureUp);
  }
}

bool TemperatureBehaveMonitorStartFlag = false;
float MonitorTempReference = -300;
float TemperatureBehaveTolerance = 3;
// monitor temperature change behavior, return ture if behave matched
// check evey one min to renew temperature reference
void TempBehaveMonitor(bool _enable, bool _temperatureUp)
{
  if (_enable)
  {
    if (!TemperatureBehaveMonitorStartFlag)
    {
      MonitorTempReference = CurrentTemperature;
      TemperatureBehaveMonitorStartFlag = true;
    }

    if (_temperatureUp)
    {// if is heating
      if (MonitorTempReference - TemperatureBehaveTolerance >= CurrentTemperature)
      {
        CurrentTemperatureError =  Heat_Cool_Reversed;
        return;
      }
    }
    else
    {// if is cooling
      if (MonitorTempReference + TemperatureBehaveTolerance <= CurrentTemperature)
      {
        CurrentTemperatureError =  Heat_Cool_Reversed;
        return;
      }
    }

    if (TempBehaveMonitor_TimerCache == 0) TempBehaveMonitor_TimerCache = millis(); // start countting led turnning off time
    if (millis() - TempBehaveMonitor_TimerCache < 0)  // rollover preventive, if rollover
    { 
      TempBehaveMonitor_TimerCache = 4294967295 - TempBehaveMonitor_TimerCache; // calculate already passed time
      TempBehaveMonitor_TimerRollover = true;
    }
    if (TimeCounter (TempBehaveMonitor_TimerCache, 60000, TempBehaveMonitor_TimerRollover))  // off time reached
    {
      TempBehaveMonitor_TimerCache = 0;
      TempBehaveMonitor_TimerRollover = false;
      if (_temperatureUp)
      {// if is heating
        if (MonitorTempReference < CurrentTemperature) MonitorTempReference = CurrentTemperature; // renew temperature reference
      }
      else
      {// if is cooling
        if (MonitorTempReference > CurrentTemperature) MonitorTempReference = CurrentTemperature;
      }
    }
  }
  else
  {
    TempBehaveMonitor_TimerCache = 0;
    TemperatureBehaveMonitorStartFlag = TempBehaveMonitor_TimerRollover = false;
  }
  
}

void CC_Emergency_Behavour()
{
  ThermalChamberControl(false, Chamber_Off);  // turn off
  TempBehaveMonitor(false,false); // reset monitor
}

bool ChamberControl_TimerRollover = false;
unsigned long ChamberControl_TimerCache = 0;
ThermalChamberControlOrder oldOrder = Chamber_Off;
ThermalChamberControlOrder appliedOder = Chamber_Off;
bool newOrder = false;
// this function using GPIO to control relay
// currentily only using one relay to turn on and turn off, heat and cool is control by manually
void ThermalChamberControl (bool _enable, ThermalChamberControlOrder _order)
{
  if (!_enable) reset_ThermalChamberControl();
  else
  {
    if (_order != oldOrder)
    {
      oldOrder = Chamber_Off;
      if (_order != Chamber_Off)newOrder = true;
    }
  }
  
  switch (oldOrder)
  {
  case Chamber_Off:
    digitalWrite(ThermalChamberCoolPin_Positive,LOW);
    digitalWrite(ThermalChamberCoolPin_Negative,LOW);
    digitalWrite(ThermalChamberHeatPin_Positive,LOW);
    digitalWrite(ThermalChamberHeatPin_Negative,LOW);
    break;
  case Chamber_Cooling:
    digitalWrite(ThermalChamberHeatPin_Positive,LOW);
    digitalWrite(ThermalChamberHeatPin_Negative,LOW);
    digitalWrite(ThermalChamberCoolPin_Positive,HIGH);
    digitalWrite(ThermalChamberCoolPin_Negative,HIGH);
    break;
  case Chamber_Heating:
    digitalWrite(ThermalChamberHeatPin_Positive,LOW);
    digitalWrite(ThermalChamberHeatPin_Negative,LOW);
    digitalWrite(ThermalChamberCoolPin_Positive,HIGH);
    digitalWrite(ThermalChamberCoolPin_Negative,HIGH);
    //digitalWrite(ThermalChamberHeatPin_Positive,HIGH);
    //digitalWrite(ThermalChamberHeatPin_Positive,HIGH);
    //digitalWrite(ThermalChamberCoolPin_Positive,LOW);
    //digitalWrite(ThermalChamberCoolPin_Negative,LOW);
    break;
  
  default:
    Serial.println("Wrong Command Send to Thermal Chamber Control method!!!");
    break;
  }
  if (newOrder)
  {
    unsigned int ChamberControlSwitchTime = 2000;

    if (ChamberControl_TimerCache == 0) ChamberControl_TimerCache = millis(); // start countting led turnning off time
    if (millis() - ChamberControl_TimerCache < 0)  // rollover preventive, if rollover
    { 
      ChamberControl_TimerCache = 4294967295 - ChamberControl_TimerCache; // calculate already passed time
      ChamberControl_TimerRollover = true;
    }
    if (TimeCounter (ChamberControl_TimerCache, ChamberControlSwitchTime, ChamberControl_TimerRollover))  // off time reached
    {
      reset_ThermalChamberControl();
      oldOrder = _order;
    }
  }
  if (appliedOder != oldOrder)
  {
    appliedOder = oldOrder;
    Serial.print("New Chamber order commit: ");
    Serial.println(appliedOder);
  }
}

void reset_ThermalChamberControl()
{
  newOrder = ChamberControl_TimerRollover = false;
  ChamberControl_TimerCache = 0;
  oldOrder = Chamber_Off;
}

void DebugAndTest ()
{
}

// use to calculate how long past after previous time
// can not handle rollover inside, so better reset every 50 days or do it outside
bool TimeCounter (unsigned long _startTime, unsigned int _targetTime, bool _rollover)
{
  unsigned int _compare = 0;
  if (!_rollover) _compare = (unsigned int)(millis() - _startTime);
  else _compare = (unsigned int)(millis() + _startTime);
  if (_compare >= _targetTime)
  {
    return true;
  }
  else
  {
    return false;
  }

}
