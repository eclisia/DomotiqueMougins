#include "../Commands/UPD.h"


#include "../../ESPEasy_common.h"

#include "../Commands/Common.h"
#include "../ESPEasyCore/ESPEasyNetwork.h"
#include "../Globals/NetworkState.h"
#include "../Globals/Settings.h"
#include "../Helpers/Misc.h"
#include "../Helpers/Network.h"
#include "../Helpers/Networking.h"
#include "../Helpers/StringConverter.h"
#include "../Helpers/StringParser.h"

String Command_UDP_Test(struct EventStruct *event, const char *Line)
{
  for (byte x = 0; x < event->Par2; x++)
  {
    String eventName = "Test ";
    eventName += x;
    SendUDPCommand(event->Par1, eventName.c_str(), eventName.length());
  }
  return return_command_success();
}

String Command_UDP_Port(struct EventStruct *event, const char *Line)
{
  return Command_GetORSetBool(event, F("UDPPort:"),
                              Line,
                              (bool *)&Settings.UDPPort,
                              1);
}

String Command_UPD_SendTo(struct EventStruct *event, const char *Line)
{
  int destUnit = parseCommandArgumentInt(Line, 1);
  if ((destUnit > 0) && (destUnit < 255))
  {
    String eventName = tolerantParseStringKeepCase(Line, 3);
    SendUDPCommand(destUnit, eventName.c_str(), eventName.length());
  }
  return return_command_success();
}

String Command_UDP_SendToUPD(struct EventStruct *event, const char *Line)
{
  if (NetworkConnected()) {
    String ip      = parseString(Line, 2);
    int port    = parseCommandArgumentInt(Line, 2);

    if (port < 0 || port > 65535) return return_command_failed();
    // FIXME TD-er: This command is not using the tolerance setting
    // tolerantParseStringKeepCase(Line, 4);
    String message = parseStringToEndKeepCase(Line, 4);
    IPAddress UDP_IP;

    if (UDP_IP.fromString(ip)) {
      portUDP.beginPacket(UDP_IP, port);
      #if defined(ESP8266)
      portUDP.write(message.c_str(),            message.length());
      #endif // if defined(ESP8266)
      #if defined(ESP32)
      portUDP.write((uint8_t *)message.c_str(), message.length());
      #endif // if defined(ESP32)
      portUDP.endPacket();
    }
    return return_command_success();
  }
  return return_not_connected();
}

#include "../Commands/System.h"

#include "../../ESPEasy_common.h"


#include "../Commands/Common.h"

#include "../Globals/Settings.h"

#include "../Helpers/DeepSleep.h"
#include "../Helpers/Misc.h"
#include "../Helpers/Scheduler.h"

String Command_System_NoSleep(struct EventStruct *event, const char* Line)
{
	if (event->Par1 > 0)
		Settings.deepSleep_wakeTime = event->Par1; // set deep Sleep awake time
	else Settings.deepSleep_wakeTime = 0;
	return return_command_success();
}

String Command_System_deepSleep(struct EventStruct *event, const char* Line)
{
	if (event->Par1 >= 0) {
		deepSleepStart(event->Par1); // call the second part of the function to avoid check and enable one-shot operation
	}
	return return_command_success();
}

String Command_System_Reboot(struct EventStruct *event, const char* Line)
{
	pinMode(0, INPUT);
	pinMode(2, INPUT);
	pinMode(15, INPUT);
	reboot(ESPEasy_Scheduler::IntendedRebootReason_e::CommandReboot);
	return return_command_success();
}


#include "../../ESPEasy_common.h"
#include "../Globals/MQTT.h"

#ifdef USES_MQTT



#include "../Commands/Common.h"
#include "../Commands/MQTT.h"

#include "../ESPEasyCore/Controller.h"
#include "../ESPEasyCore/ESPEasy_Log.h"

#include "../Globals/CPlugins.h"
#include "../Globals/ESPEasy_Scheduler.h"
#include "../Globals/Settings.h"

#include "../Helpers/ESPEasy_Storage.h"
#include "../Helpers/PeriodicalActions.h"
#include "../Helpers/Scheduler.h"
#include "../Helpers/StringConverter.h"


String Command_MQTT_Publish(struct EventStruct *event, const char *Line)
{
  // ToDo TD-er: Not sure about this function, but at least it sends to an existing MQTTclient
  controllerIndex_t enabledMqttController = firstEnabledMQTT_ControllerIndex();

  if (!validControllerIndex(enabledMqttController)) {
    return F("No MQTT controller enabled");
  }

  // Command structure:  Publish,<topic>,<value>
  String topic = parseStringKeepCase(Line, 2);
  String value = tolerantParseStringKeepCase(Line, 3);
  addLog(LOG_LEVEL_DEBUG, String(F("Publish: ")) + topic + value);

  if ((topic.length() > 0) && (value.length() > 0)) {

    bool mqtt_retainFlag;
    {
      // Place the ControllerSettings in a scope to free the memory as soon as we got all relevant information.
      MakeControllerSettings(ControllerSettings);
      if (!AllocatedControllerSettings()) {
        String error = F("MQTT : Cannot publish, out of RAM");
        addLog(LOG_LEVEL_ERROR, error);
        return error;
      }

      LoadControllerSettings(enabledMqttController, ControllerSettings);
      mqtt_retainFlag = ControllerSettings.mqtt_retainFlag();
    }


    // @giig1967g: if payload starts with '=' then treat it as a Formula and evaluate accordingly
    // The evaluated value is already present in event->Par2
    // FIXME TD-er: Is the evaluated value always present in event->Par2 ?
    // Should it already be evaluated, or should we evaluate it now?

    bool success = false;
    if (value[0] != '=') {
      success = MQTTpublish(enabledMqttController, topic.c_str(), value.c_str(), mqtt_retainFlag);
    }
    else {
      success = MQTTpublish(enabledMqttController, topic.c_str(), String(event->Par2).c_str(), mqtt_retainFlag);
    }
    if (success) {
      return return_command_success();
    }
  }
  return return_command_failed();
}


boolean MQTTsubscribe(controllerIndex_t controller_idx, const char* topic, boolean retained)
{
  if (MQTTclient.subscribe(topic)) {
    Scheduler.setIntervalTimerOverride(ESPEasy_Scheduler::IntervalTimer_e::TIMER_MQTT, 10); // Make sure the MQTT is being processed as soon as possible.
    String log = F("Subscribed to: ");  log += topic;
    addLog(LOG_LEVEL_INFO, log);
    return true;
  }
  addLog(LOG_LEVEL_ERROR, F("MQTT : subscribe failed"));
  return false;
}

String Command_MQTT_Subscribe(struct EventStruct *event, const char* Line)
{
  if (MQTTclient.connected() ) {
    // ToDo TD-er: Not sure about this function, but at least it sends to an existing MQTTclient
    controllerIndex_t enabledMqttController = firstEnabledMQTT_ControllerIndex();
    if (validControllerIndex(enabledMqttController)) {
      bool mqtt_retainFlag;
      {
        // Place the ControllerSettings in a scope to free the memory as soon as we got all relevant information.
        MakeControllerSettings(ControllerSettings);
        if (!AllocatedControllerSettings()) {
          String error = F("MQTT : Cannot subscribe, out of RAM");
          addLog(LOG_LEVEL_ERROR, error);
          return error;
        }
        LoadControllerSettings(event->ControllerIndex, ControllerSettings);
        mqtt_retainFlag = ControllerSettings.mqtt_retainFlag();
      }

      String eventName = Line;
      String topic = eventName.substring(10);
      if (!MQTTsubscribe(enabledMqttController, topic.c_str(), mqtt_retainFlag))
         return_command_failed();
      return_command_success();
    }
    return F("No MQTT controller enabled");
  }
  return return_not_connected();
}


#endif // ifdef USES_MQTT

#include "../Commands/Timer.h"




#include "../../ESPEasy_common.h"


#include "../Commands/Common.h"

#include "../ESPEasyCore/ESPEasy_Log.h"
#include "../ESPEasyCore/ESPEasyRules.h"

#include "../Globals/ESPEasy_Scheduler.h"

#include "../Helpers/ESPEasy_time_calc.h"
#include "../Helpers/Misc.h"
#include "../Helpers/Scheduler.h"

String command_setRulesTimer(int msecFromNow, int timerIndex, int recurringCount) {
  if (msecFromNow < 0)
  {
    addLog(LOG_LEVEL_ERROR, F("TIMER: time must be positive"));
  } else {
    // start new timer when msecFromNow > 0
    // Clear timer when msecFromNow == 0
    if (Scheduler.setRulesTimer(msecFromNow, timerIndex, recurringCount))
    { 
      return return_command_success();
    }
  }
  return return_command_failed();
}

String Command_Timer_Set(struct EventStruct *event, const char *Line)
{
  return command_setRulesTimer(
    event->Par2 * 1000, // msec from now
    event->Par1,        // timer index
    0                   // recurringCount
    );
}

String Command_Timer_Set_ms (struct EventStruct *event, const char* Line)
{
  return command_setRulesTimer(
    event->Par2, // interval
    event->Par1, // timer index
    0            // recurringCount
    );
}

String Command_Loop_Timer_Set (struct EventStruct *event, const char* Line)
{
  int recurringCount = event->Par3;
  if (recurringCount == 0) {
    // if the optional 3rd parameter is not given, set it to "run always"
    recurringCount = -1;
  }
  return command_setRulesTimer(
    event->Par2 * 1000, // msec from now
    event->Par1,        // timer index
    recurringCount
    );
}

String Command_Loop_Timer_Set_ms (struct EventStruct *event, const char* Line)
{
  int recurringCount = event->Par3;
  if (recurringCount == 0) {
    // if the optional 3rd parameter is not given, set it to "run always"
    recurringCount = -1;
  }
  return command_setRulesTimer(
    event->Par2, // interval
    event->Par1, // timer index
    recurringCount
    );
}

String Command_Timer_Pause(struct EventStruct *event, const char *Line)
{
  if (Scheduler.pause_rules_timer(event->Par1)) {
    String eventName = F("Rules#TimerPause=");
    eventName += event->Par1;
    rulesProcessing(eventName); // TD-er: Process right now
    return return_command_success();
  }
  return return_command_failed();
}

String Command_Timer_Resume(struct EventStruct *event, const char *Line)
{
  if (Scheduler.resume_rules_timer(event->Par1)) {
    String eventName = F("Rules#TimerResume=");
    eventName += event->Par1;
    rulesProcessing(eventName); // TD-er: Process right now
    return return_command_success();
  }
  return return_command_failed();
}

String Command_Delay(struct EventStruct *event, const char *Line)
{
  delayBackground(event->Par1);
  return return_command_success();
}

#include "../Commands/Networks.h"

#include "../../ESPEasy_common.h"
#include "../Commands/Common.h"
#include "../Globals/Settings.h"
#include "../WebServer/AccessControl.h"


#ifdef HAS_ETHERNET
#include "ETH.h"
#endif

String Command_AccessInfo_Ls(struct EventStruct *event, const char* Line)
{
  String result = F("Allowed IP range : ");
  result += describeAllowedIPrange();
  return return_result(event, result);
}

String Command_AccessInfo_Clear (struct EventStruct *event, const char* Line)
{
  clearAccessBlock();
  return Command_AccessInfo_Ls(event, Line);
}

String Command_DNS (struct EventStruct *event, const char* Line)
{
  return Command_GetORSetIP(event, F("DNS:"), Line, Settings.DNS,WiFi.dnsIP(0),1);
}

String Command_Gateway (struct EventStruct *event, const char* Line)
{
  return Command_GetORSetIP(event, F("Gateway:"), Line, Settings.Gateway,WiFi.gatewayIP(),1);
}

String Command_IP (struct EventStruct *event, const char* Line)
{
  return Command_GetORSetIP(event, F("IP:"), Line, Settings.IP,WiFi.localIP(),1);
}

String Command_Subnet (struct EventStruct *event, const char* Line)
{
  return Command_GetORSetIP(event, F("Subnet:"), Line, Settings.Subnet,WiFi.subnetMask(),1);
}

#ifdef HAS_ETHERNET
String Command_ETH_Phy_Addr (struct EventStruct *event, const char* Line)
{
  return Command_GetORSetUint8_t(event, F("ETH_Phy_Addr:"), Line, (uint8_t*)&Settings.ETH_Phy_Addr,1);
}

String Command_ETH_Pin_mdc (struct EventStruct *event, const char* Line)
{
  return Command_GetORSetInt8_t(event, F("ETH_Pin_mdc:"), Line, (int8_t*)&Settings.ETH_Pin_mdc,1);
}

String Command_ETH_Pin_mdio (struct EventStruct *event, const char* Line)
{
  return Command_GetORSetInt8_t(event, F("ETH_Pin_mdio:"), Line, (int8_t*)&Settings.ETH_Pin_mdio,1);
}

String Command_ETH_Pin_power (struct EventStruct *event, const char* Line)
{
  return Command_GetORSetInt8_t(event, F("ETH_Pin_power:"), Line, (int8_t*)&Settings.ETH_Pin_power,1);
}

String Command_ETH_Phy_Type (struct EventStruct *event, const char* Line)
{
  return Command_GetORSetInt8_t(event, F("ETH_Phy_Type:"), Line, (int8_t*)&Settings.ETH_Phy_Type,1);
}

String Command_ETH_Clock_Mode (struct EventStruct *event, const char* Line)
{
  return Command_GetORSetUint8_t(event, F("ETH_Clock_Mode:"), Line, (uint8_t*)&Settings.ETH_Clock_Mode,1);
}

String Command_ETH_IP (struct EventStruct *event, const char* Line)
{
  return Command_GetORSetIP(event, F("ETH_IP:"), Line, Settings.ETH_IP,ETH.localIP(),1);
}

String Command_ETH_Gateway (struct EventStruct *event, const char* Line)
{
  return Command_GetORSetIP(event, F("ETH_Gateway:"), Line, Settings.ETH_Gateway,ETH.gatewayIP(),1);
}

String Command_ETH_Subnet (struct EventStruct *event, const char* Line)
{
  return Command_GetORSetIP(event, F("ETH_Subnet:"), Line, Settings.ETH_Subnet,ETH.subnetMask(),1);
}

String Command_ETH_DNS (struct EventStruct *event, const char* Line)
{
  return Command_GetORSetIP(event, F("ETH_DNS:"), Line, Settings.ETH_DNS,ETH.dnsIP(),1);
}

String Command_ETH_Wifi_Mode (struct EventStruct *event, const char* Line)
{
  return Command_GetORSetUint8_t(event, F("NetworkMedium:"), Line, (uint8_t*)&Settings.NetworkMedium,1);
}

#endif

#include "../Commands/GPIO.h"


#include "../../ESPEasy-Globals.h"
#include "../../ESPEasy_common.h"
#include "../../ESPEasy_fdwdecl.h"
#include "../Commands/Common.h"
#include "../DataStructs/PinMode.h"
#include "../ESPEasyCore/Controller.h"
#include "../ESPEasyCore/ESPEasyGPIO.h"
#include "../ESPEasyCore/ESPEasy_Log.h"
#include "../Globals/ESPEasy_Scheduler.h"
#include "../Globals/GlobalMapPortStatus.h"
#include "../Helpers/Audio.h"
#include "../Helpers/Hardware.h"
#include "../Helpers/StringConverter.h"
#include "../Helpers/PortStatus.h"


//predeclaration of functions used in this module
void createAndSetPortStatus_Mode_State(uint32_t key, byte newMode, int8_t newState);
bool getPluginIDAndPrefix(char selection, pluginID_t &pluginID, String &logPrefix);
void logErrorGpioOffline(const String& prefix, int port);
void logErrorGpioOutOfRange(const String& prefix, int port, const char* Line = nullptr);
void logErrorGpioNotOutput(const String& prefix, int port);

String Command_GPIO_Monitor(struct EventStruct *event, const char* Line)
{
  String logPrefix;
  pluginID_t pluginID = INVALID_PLUGIN_ID;
  //parseString(Line, 2).charAt(0)='g':gpio; ='p':pcf; ='m':mcp
  bool success = getPluginIDAndPrefix(parseString(Line, 2).charAt(0), pluginID, logPrefix);
  if (success && checkValidPortRange(pluginID, event->Par2))
  {
    const uint32_t key = createKey(pluginID, event->Par2); // WARNING: 'monitor' uses Par2 instead of Par1
    //if (!existPortStatus(key)) globalMapPortStatus[key].mode=PIN_MODE_OUTPUT;
    addMonitorToPort(key);

    int8_t state;
    //giig1967g: Comment next 3 lines to receive an EVENT just after calling the monitor command
    GPIO_Read(pluginID, event->Par2, state);
    globalMapPortStatus[key].state = state;
    if (state == -1) globalMapPortStatus[key].mode=PIN_MODE_OFFLINE;

    String log = logPrefix + String(F(" port #")) + String(event->Par2) + String(F(": added to monitor list."));
    addLog(LOG_LEVEL_INFO, log);
    String dummy;
    SendStatusOnlyIfNeeded(event->Source, SEARCH_PIN_STATE, key, dummy, 0);

    return return_command_success();
  } else {
    logErrorGpioOutOfRange(logPrefix,event->Par2, Line);
    return return_command_failed();
  }
}

String Command_GPIO_UnMonitor(struct EventStruct *event, const char* Line)
{
  String logPrefix;
  pluginID_t pluginID = INVALID_PLUGIN_ID;
  //parseString(Line, 2).charAt(0)='g':gpio; ='p':pcf; ='m':mcp
  bool success = getPluginIDAndPrefix(parseString(Line, 2).charAt(0), pluginID, logPrefix);

  if (success && checkValidPortRange(pluginID, event->Par2))
  {
    const uint32_t key = createKey(pluginID, event->Par2); // WARNING: 'monitor' uses Par2 instead of Par1
    String dummy;
    SendStatusOnlyIfNeeded(event->Source, SEARCH_PIN_STATE, key, dummy, 0);

    removeMonitorFromPort(key);
    String log = logPrefix + String(F(" port #")) + String(event->Par2) + String(F(": removed from monitor list."));
    addLog(LOG_LEVEL_INFO, log);

    return return_command_success();
  } else {
    logErrorGpioOutOfRange(logPrefix,event->Par2, Line);
    return return_command_failed();
  }
}

String Command_GPIO_LongPulse(struct EventStruct *event, const char* Line)
{
  event->Par3 = event->Par3 * 1000;
  return Command_GPIO_LongPulse_Ms( event, Line);
}

String Command_GPIO_LongPulse_Ms(struct EventStruct *event, const char* Line)
{
  String logPrefix;// = ;
  pluginID_t pluginID=INVALID_PLUGIN_ID;
  //Line[0]='l':longpulse; ='p':pcflongpulse; ='m':mcplongpulse
  bool success = getPluginIDAndPrefix(Line[0], pluginID, logPrefix);
  if (success && checkValidPortRange(pluginID, event->Par1))
  {
    const uint32_t key = createKey(pluginID,event->Par1);
    createAndSetPortStatus_Mode_State(key,PIN_MODE_OUTPUT,event->Par2);
    GPIO_Write(pluginID, event->Par1, event->Par2);

    Scheduler.setGPIOTimer(event->Par3, pluginID, event->Par1, !event->Par2);

    String log = logPrefix + String(F(" : port ")) + String(event->Par1);
    log += String(F(". Pulse set for ")) + String(event->Par3)+String(F(" ms"));
    addLog(LOG_LEVEL_INFO, log);
    SendStatusOnlyIfNeeded(event->Source, SEARCH_PIN_STATE, key, log, 0);

    return return_command_success();
  } else {
    logErrorGpioOutOfRange(logPrefix,event->Par1, Line);
    return return_command_failed();
  }
}

String Command_GPIO_Status(struct EventStruct *event, const char* Line)
{
	bool success = true;
  bool sendStatusFlag;
  byte pluginID;

  switch (tolower(parseString(Line, 2).charAt(0)))
  {
    case 'g': //gpio
      pluginID=PLUGIN_GPIO;
	    sendStatusFlag = true;
      break;
    case 'm': //mcp
      pluginID=PLUGIN_MCP;
	    sendStatusFlag = GPIO_MCP_Read(event->Par2)==-1;
      break;
    case 'p': //pcf
      pluginID=PLUGIN_PCF;
	    sendStatusFlag = GPIO_PCF_Read(event->Par2)==-1;
      break;
    default:
      success=false;
  }

  if (success && checkValidPortRange(pluginID, event->Par2))
  {
    const uint32_t key = createKey(pluginID, event->Par2); // WARNING: 'status' uses Par2 instead of Par1
	  String dummy;
	  SendStatusOnlyIfNeeded(event->Source, sendStatusFlag, key, dummy, 0);
    return return_command_success();
  } else {
    return return_command_failed();
  }
}

String Command_GPIO_PWM(struct EventStruct *event, const char *Line)
{
  // Par1: GPIO
  // Par2: Duty Cycle
  // Par3: Fade duration
  // Par4: Frequency

  // For now, we only support the internal GPIO pins.
  String logPrefix = F("GPIO");
  uint32_t frequency = event->Par4;
  uint32_t key = 0;
  if (set_Gpio_PWM(event->Par1, event->Par2, event->Par3, frequency, key)) {
    String log = F("PWM  : GPIO: ");
    log += event->Par1;
    log += F(" duty: ");
    log += event->Par2;

    if (event->Par3 != 0) {
      log += F(" Fade: ");
      log += event->Par3;
      log += F(" ms");
    }
    if (event->Par4 != 0) {
      log += F(" f: ");
      log += frequency;
      log += F(" Hz");
    }
    addLog(LOG_LEVEL_INFO, log);
    SendStatusOnlyIfNeeded(event->Source, SEARCH_PIN_STATE, key, log, 0);

    // SendStatus(event->Source, getPinStateJSON(SEARCH_PIN_STATE, pluginID, event->Par1, log, 0));

    return return_command_success();
  } 
  logErrorGpioOutOfRange(logPrefix, event->Par1, Line);
  return return_command_failed();
}

String Command_GPIO_Tone(struct EventStruct *event, const char* Line)
{
  // play a tone on pin par1, with frequency par2 and duration in msec par3.
  unsigned long duration = event->Par3;
  bool mustScheduleToneOff = false;
  if (duration > 50) {
    duration = 0;
    mustScheduleToneOff = true;
  }
  if (tone_espEasy(event->Par1, event->Par2, duration)) {
    if (mustScheduleToneOff) {
      // For now, we only support the internal GPIO pins.
      byte   pluginID  = PLUGIN_GPIO;
      Scheduler.setGPIOTimer(event->Par3, pluginID, event->Par1, 0);
    }
    return return_command_success();
  }
  return return_command_failed();
}


String Command_GPIO_RTTTL(struct EventStruct *event, const char* Line)
{
  #ifdef USE_RTTTL
  // FIXME: Absolutely no error checking in play_rtttl, until then keep it only in testing
  // play a tune via a RTTTL string, look at https://www.letscontrolit.com/forum/viewtopic.php?f=4&t=343&hilit=speaker&start=10 for
  // more info.

  String melody = parseStringToEndKeepCase(Line, 2);
  melody.replace('-', '#');
  if (loglevelActiveFor(LOG_LEVEL_INFO)) {
    String log = F("RTTTL : pin: ");
    log += event->Par1;
    log += F(" melody: ");
    log += melody;
    addLog(LOG_LEVEL_INFO, log);
  }
  if (play_rtttl(event->Par1, melody.c_str())) {
    return return_command_success();
  }
  #else 
  addLog(LOG_LEVEL_ERROR, F("RTTTL : command not included in build"));
  #endif
  return return_command_failed();
}

String Command_GPIO_Pulse(struct EventStruct *event, const char* Line)
{
  String logPrefix;
  bool success = false;
  byte pluginID=INVALID_PLUGIN_ID;
  switch (tolower(Line[0]))
  {
    case 'p': // pulse or pcfpulse
      if (tolower(Line[1])=='u') { //pulse
        pluginID=PLUGIN_GPIO;
        logPrefix=String(F("GPIO"));
        success=true;
      } else if (tolower(Line[1])=='c'){ //pcfpulse
        pluginID=PLUGIN_PCF;
        logPrefix=String(F("PCF"));
        success=true;
      }
      break;
    case 'm': //mcp
      pluginID=PLUGIN_MCP;
      logPrefix=String(F("MCP"));
      success=true;
      break;
  }

  if (success && checkValidPortRange(pluginID, event->Par1))
  {
    const uint32_t key = createKey(pluginID,event->Par1);

    createAndSetPortStatus_Mode_State(key,PIN_MODE_OUTPUT,event->Par2);
    GPIO_Write(pluginID, event->Par1, event->Par2);

    delay(event->Par3);

    createAndSetPortStatus_Mode_State(key,PIN_MODE_OUTPUT,!event->Par2);
    GPIO_Write(pluginID, event->Par1, !event->Par2);

    String log = logPrefix + String(F(" : port ")) + String(event->Par1);
    log += String(F(". Pulse set for ")) + String(event->Par3)+String(F(" ms"));
    addLog(LOG_LEVEL_INFO, log);
    SendStatusOnlyIfNeeded(event->Source, SEARCH_PIN_STATE, key, log, 0);

    return return_command_success();
  } else {
    logErrorGpioOutOfRange(logPrefix,event->Par1, Line);
    return return_command_failed();
  }
}

String Command_GPIO_Toggle(struct EventStruct *event, const char* Line)
{
  String logPrefix;
  pluginID_t pluginID=INVALID_PLUGIN_ID;
  //Line[0]='g':gpiotoggle; ='p':pcfgpiotoggle; ='m':mcpgpiotoggle
  bool success = getPluginIDAndPrefix(Line[0], pluginID, logPrefix);
  if (success && checkValidPortRange(pluginID, event->Par1))
  {
    const uint32_t key = createKey(pluginID,event->Par1);
    // WARNING: operator [] creates an entry in the map if key does not exist
    // So the next command should be part of each command:
    byte mode;
    int8_t state;

    auto it = globalMapPortStatus.find(key);
    if (it != globalMapPortStatus.end()) {
      mode  = it->second.mode;
      state = it->second.state;
    } else {
      GPIO_Read(pluginID, event->Par1, state);
      mode = (state==-1)?PIN_MODE_OFFLINE:PIN_MODE_OUTPUT;
    }

    switch (mode) {
      case PIN_MODE_OUTPUT:
      case PIN_MODE_UNDEFINED:
        {
          createAndSetPortStatus_Mode_State(key,PIN_MODE_OUTPUT,!state);
          GPIO_Write(pluginID, event->Par1, !state);

          String log = logPrefix + String(F(" toggle: port#")) + String(event->Par1) + String(F(": set to ")) + String(!state);
          addLog(LOG_LEVEL_ERROR, log);
      	  SendStatusOnlyIfNeeded(event->Source, SEARCH_PIN_STATE, key, log, 0);

          return return_command_success();
        }
        break;
      case PIN_MODE_OFFLINE:
        logErrorGpioOffline(logPrefix,event->Par1);
        return return_command_failed();
        break;
      default:
        logErrorGpioNotOutput(logPrefix,event->Par1);
        return return_command_failed();
        break;
    }
  } else {
    logErrorGpioOutOfRange(logPrefix,event->Par1, Line);
    return return_command_failed();
  }
}

String Command_GPIO(struct EventStruct *event, const char* Line)
{
  String logPrefix;// = new char;
  pluginID_t pluginID=INVALID_PLUGIN_ID;
  //Line[0]='g':gpio; ='p':pcfgpio; ='m':mcpgpio
  bool success = getPluginIDAndPrefix(Line[0], pluginID, logPrefix);
  if (success && checkValidPortRange(pluginID, event->Par1))
  {
	  int8_t state=0;
	  byte mode;

	  if (event->Par2 == 2) { //INPUT
		  mode = PIN_MODE_INPUT_PULLUP;
      switch (pluginID) {
        case PLUGIN_GPIO:
          setInternalGPIOPullupMode(event->Par1);
          state = GPIO_Read_Switch_State(event->Par1, PIN_MODE_INPUT_PULLUP);
          break;
        case PLUGIN_MCP:
        case PLUGIN_PCF:
          // PCF8574/MCP specific: only can read 0/low state, so we must send 1
          state = 1;
          break;
		  }
    } else { // OUTPUT
      mode=PIN_MODE_OUTPUT;
      state=(event->Par2==0)?0:1;
    }

    const uint32_t key = createKey(pluginID,event->Par1);

    if (globalMapPortStatus[key].mode != PIN_MODE_OFFLINE)
    {
      int8_t currentState;
      GPIO_Read(pluginID, event->Par1, currentState);
      if (currentState==-1) {
        mode=PIN_MODE_OFFLINE;
        state = -1;
      }

      createAndSetPortStatus_Mode_State(key,mode,state);
      GPIO_Write(pluginID,event->Par1,state,mode);

  		String log = logPrefix + String(F(" : port#")) + String(event->Par1) + String(F(": set to ")) + String(state);
  		addLog(LOG_LEVEL_INFO, log);
  		SendStatusOnlyIfNeeded(event->Source, SEARCH_PIN_STATE, key, log, 0);
  		return return_command_success();
  	} else {
      logErrorGpioOffline(logPrefix,event->Par1);
      return return_command_failed();
    }
  } else {
    logErrorGpioOutOfRange(logPrefix,event->Par1, Line);
    return return_command_failed();
  }
}

void logErrorGpio(const String& prefix, int port, const String& description)
{
  if (port >= 0) {
    addLog(LOG_LEVEL_ERROR, prefix + String(F(" : port#")) + String(port) + description);
  }
}

void logErrorGpioOffline(const String& prefix, int port)
{
  logErrorGpio(prefix, port, F(" is offline."));
}

void logErrorGpioOutOfRange(const String& prefix, int port, const char* Line)
{
  logErrorGpio(prefix, port, F(" is out of range"));
  if (port >= 0) {
    if (Line != nullptr) {
      addLog(LOG_LEVEL_DEBUG, Line);
    }
  }
}

void logErrorGpioNotOutput(const String& prefix, int port)
{
  logErrorGpio(prefix, port, F(" is not an output port"));
}

void createAndSetPortStatus_Mode_State(uint32_t key, byte newMode, int8_t newState)
{
  // WARNING: operator [] creates an entry in the map if key does not exist

  // If it doesn't exist, it is now created.
  globalMapPortStatus[key].mode     = newMode;
  auto it = globalMapPortStatus.find(key);
  if (it != globalMapPortStatus.end()) {
    // Should always be true, as it would be created if it didn't exist.
    it->second.command  = 1; //set to 1 in order to display the status in the PinStatus page
    //only force events if state has changed
    if (it->second.state != newState) {
      it->second.state        = newState;
      it->second.output       = newState;
      it->second.forceEvent   = 1;
      it->second.forceMonitor = 1;
    }
  }
}

bool getPluginIDAndPrefix(char selection, pluginID_t &pluginID, String &logPrefix)
{
  bool success = true;
  switch(tolower(selection))
  {
    case 'g': //gpio
    case 'l': //longpulse (gpio)
      pluginID=PLUGIN_GPIO;
      logPrefix=F("GPIO");
      break;
    case 'm': //mcp & mcplongpulse
      pluginID=PLUGIN_MCP;
      logPrefix=F("MCP");
      break;
    case 'p': //pcf & pcflongpulse
      pluginID=PLUGIN_PCF;
      logPrefix=F("PCF");
      break;
    default:
      logPrefix=F("PluginID out of range. Error");
      success=false;
  }
  return success;
}

/*
bool getPluginIDAndPrefixAndType(char selection, pluginID_t &pluginID, String &logPrefix, byte &gpioTimerType)
{
  bool success = true;
  switch(tolower(selection))
  {
    case 'l': //longpulse (gpio)
      pluginID =PLUGIN_GPIO;
      logPrefix=F("GPIO");
      gpioTimerType = GPIO_TYPE_INTERNAL;
      break;
    case 'm': //mcplongpulse (mcp)
      pluginID =PLUGIN_MCP;
      logPrefix=F("MCP");
      gpioTimerType = GPIO_TYPE_MCP;
      break;
    case 'p': //pcflongpulse (pcf)
      pluginID =PLUGIN_PCF;
      logPrefix=F("PCF");
      gpioTimerType = GPIO_TYPE_PCF;
      break;
    default:
      logPrefix=F("PluginID out of range. Error");
      success=false;
  }
  return success;
}
*/

#include "../Commands/wd.h"


#include "../Commands/Common.h"

#include "../DataStructs/ESPEasy_EventStruct.h"

#include "../ESPEasyCore/Serial.h"

#include "../Helpers/StringConverter.h"


String Command_WD_Config(EventStruct *event, const char* Line)
{
  Wire.beginTransmission(event->Par1);  // address
  Wire.write(event->Par2);              // command
  Wire.write(event->Par3);              // data
  Wire.endTransmission();
  return return_command_success();
}

String Command_WD_Read(EventStruct *event, const char* Line)
{
  Wire.beginTransmission(event->Par1);  // address
  Wire.write(0x83);                     // command to set pointer
  Wire.write(event->Par2);              // pointer value
  Wire.endTransmission();
  if ( Wire.requestFrom(static_cast<uint8_t>(event->Par1), static_cast<uint8_t>(1)) == 1 )
  {
    byte value = Wire.read();
    serialPrintln();
    String result = F("I2C Read address ");
    result += formatToHex(event->Par1);
    result += F(" Value ");
    result += formatToHex(value);
    return return_result(event, result);
  }
  return return_command_success();
}

#include "../Commands/Settings.h"

#include "../../ESPEasy_common.h"

#include "../Commands/Common.h"

#include "../ESPEasyCore/ESPEasyNetwork.h"
#include "../ESPEasyCore/Serial.h"

#include "../Globals/SecuritySettings.h"
#include "../Globals/Settings.h"

#include "../Helpers/ESPEasy_FactoryDefault.h"
#include "../Helpers/ESPEasy_Storage.h"
#include "../Helpers/Memory.h"
#include "../Helpers/Misc.h"
#include "../Helpers/StringConverter.h"


String Command_Settings_Build(struct EventStruct *event, const char* Line)
{
	if (HasArgv(Line, 2)) {
		Settings.Build = event->Par1;
	} else {
		serialPrintln();
		String result = F("Build:");
		result += Settings.Build;
    return return_result(event, result);
	}
	return return_command_success();
}

String Command_Settings_Unit(struct EventStruct *event, const char* Line)
{
	if (HasArgv(Line, 2)) {
		Settings.Unit = event->Par1;
	}else  {
		serialPrintln();
		String result = F("Unit:");
		result += Settings.Unit;
    return return_result(event, result);
	}
	return return_command_success();
}

String Command_Settings_Name(struct EventStruct *event, const char* Line)
{
	return Command_GetORSetString(event, F("Name:"),
				      Line,
				      Settings.Name,
				      sizeof(Settings.Name),
				      1);
}

String Command_Settings_Password(struct EventStruct *event, const char* Line)
{
	return Command_GetORSetString(event, F("Password:"),
				      Line,
				      SecuritySettings.Password,
				      sizeof(SecuritySettings.Password),
				      1
				      );
}

String Command_Settings_Save(struct EventStruct *event, const char* Line)
{
	SaveSettings();
	return return_command_success();
}

String Command_Settings_Load(struct EventStruct *event, const char* Line)
{
	LoadSettings();
	return return_command_success();
}

String Command_Settings_Print(struct EventStruct *event, const char* Line)
{
	serialPrintln();

	serialPrintln(F("System Info"));
	serialPrint(F("  IP Address    : ")); serialPrintln(NetworkLocalIP().toString());
	serialPrint(F("  Build         : ")); serialPrintln(String((int)BUILD));
	serialPrint(F("  Name          : ")); serialPrintln(Settings.Name);
	serialPrint(F("  Unit          : ")); serialPrintln(String((int)Settings.Unit));
	serialPrint(F("  WifiSSID      : ")); serialPrintln(SecuritySettings.WifiSSID);
	serialPrint(F("  WifiKey       : ")); serialPrintln(SecuritySettings.WifiKey);
	serialPrint(F("  WifiSSID2     : ")); serialPrintln(SecuritySettings.WifiSSID2);
	serialPrint(F("  WifiKey2      : ")); serialPrintln(SecuritySettings.WifiKey2);
	serialPrint(F("  Free mem      : ")); serialPrintln(String(FreeMem()));
	return return_see_serial(event);
}

String Command_Settings_Reset(struct EventStruct *event, const char* Line)
{
	ResetFactory();
	reboot(ESPEasy_Scheduler::IntendedRebootReason_e::ResetFactoryCommand);
	return return_command_success();
}

#include "../Commands/Servo.h"

#include "../Commands/Common.h"
#include "../Commands/GPIO.h"
#include "../DataStructs/EventStructCommandWrapper.h"
#include "../DataStructs/PinMode.h"
#include "../DataStructs/PortStatusStruct.h"
#include "../ESPEasyCore/Controller.h"
#include "../ESPEasyCore/ESPEasyGPIO.h"
#include "../ESPEasyCore/ESPEasy_Log.h"
#include "../Globals/GlobalMapPortStatus.h"
#include "../Helpers/Hardware.h"
#include "../Helpers/PortStatus.h"

// Needed also here for PlatformIO's library finder as the .h file 
// is in a directory which is excluded in the src_filter
#ifdef USE_SERVO
# ifdef ESP32
#  include <Servo.h>
# endif // ifdef ESP32
#endif

#ifdef USE_SERVO
ServoPinMap_t ServoPinMap;
#endif // ifdef USE_SERVO

String Command_Servo(struct EventStruct *event, const char *Line)
{
#ifdef USE_SERVO

  // GPIO number is stored inside event->Par2 instead of event->Par1 as in all the other commands
  // So needs to reload the tempPortStruct.

  // FIXME TD-er: For now only fixed to "P001" even when it is for internal GPIO pins
  pluginID_t pluginID = PLUGIN_GPIO;

  // Par1: Servo ID (obsolete/unused since 2020/11/22)
  // Par2: GPIO pin
  // Par3: angle 0...180 degree
  if (checkValidPortRange(pluginID, event->Par2)) {
    portStatusStruct tempStatus;
    const uint32_t   key = createKey(pluginID, event->Par2); // WARNING: 'servo' uses Par2 instead of Par1
    // WARNING: operator [] creates an entry in the map if key does not exist
    // So the next command should be part of each command:
    tempStatus = globalMapPortStatus[key];

    String log = F("Servo : GPIO ");
    log += event->Par2;

    // SPECIAL CASE TO ALLOW SERVO TO BE DETATTCHED AND SAVE POWER.
    if (event->Par3 >= 9000) {
      auto it = ServoPinMap.find(event->Par2);

      if (it != ServoPinMap.end()) {
        it->second.detach();
        # ifdef ESP32
          detachLedChannel(event->Par2);
        # endif // ifdef ESP32
        ServoPinMap.erase(it);
      }

      // Set parameters to make sure the port status will be removed.
      tempStatus.task    = 0;
      tempStatus.monitor = 0;
      tempStatus.command = 0;
      savePortStatus(key, tempStatus);
      log += F(" Servo detached");
      addLog(LOG_LEVEL_INFO, log);
      return return_command_success();

    }
    # ifdef ESP32
      // Must keep track of used channels or else cause conflicts with PWM
      int8_t ledChannel = attachLedChannel(event->Par2);
      ServoPinMap[event->Par2].attach(event->Par2, ledChannel);
    # else // ifdef ESP32
      ServoPinMap[event->Par2].attach(event->Par2);
    # endif // ifdef ESP32
    ServoPinMap[event->Par2].write(event->Par3);

    tempStatus.command   = 1; // set to 1 in order to display the status in the PinStatus page
    tempStatus.state     = 1;
    tempStatus.output    = 1;
    tempStatus.dutyCycle = event->Par3;

    // setPinState(PLUGIN_ID_001, event->Par2, PIN_MODE_SERVO, event->Par3);
    tempStatus.mode = PIN_MODE_SERVO;
    savePortStatus(key, tempStatus);
    log += F(" Servo set to ");
    log += event->Par3;
    addLog(LOG_LEVEL_INFO, log);
    SendStatusOnlyIfNeeded(event->Source, SEARCH_PIN_STATE, key, log, 0);

    // SendStatus(event->Source, getPinStateJSON(SEARCH_PIN_STATE, PLUGIN_ID_001, event->Par2, log, 0));
    return return_command_success();
  }
    #else // ifdef USE_SERVO
  addLog(LOG_LEVEL_ERROR, F("USE_SERVO not included in build"));
    #endif // USE_SERVO
  return return_command_failed();
}

#include "../Commands/Common.h"

#include <ctype.h>
#include <IPAddress.h>

#include "../../ESPEasy_common.h"

#include "../DataStructs/ESPEasy_EventStruct.h"
#include "../DataTypes/EventValueSource.h"

#include "../ESPEasyCore/ESPEasyWifi.h"
#include "../ESPEasyCore/Serial.h"

#include "../Helpers/Numerical.h"
#include "../Helpers/StringConverter.h"


// Simple function to return "Ok", to avoid flash string duplication in the firmware.
String return_command_success()
{
  return F("\nOk");
}

String return_command_failed()
{
  return F("\nFailed");
}

String return_incorrect_nr_arguments()
{
  return F("Too many arguments, try using quotes!");
}

String return_incorrect_source()
{
  return F("Command not allowed from this source!");
}

String return_not_connected()
{
  return F("Not connected to WiFi");
}

String return_result(struct EventStruct *event, const String& result)
{
  serialPrintln(result);

  if (event->Source == EventValueSource::Enum::VALUE_SOURCE_SERIAL) {
    return return_command_success();
  }
  return result;
}

String return_see_serial(struct EventStruct *event)
{
  if (event->Source == EventValueSource::Enum::VALUE_SOURCE_SERIAL) {
    return return_command_success();
  }
  return F("Output sent to serial");
}

String Command_GetORSetIP(struct EventStruct *event,
                          const String      & targetDescription,
                          const char         *Line,
                          byte               *IP,
                          const IPAddress   & dhcpIP,
                          int                 arg)
{
  bool hasArgument = false;
  {
    // Check if command is valid. Leave in separate scope to delete the TmpStr1
    String TmpStr1;

    if (GetArgv(Line, TmpStr1, arg + 1)) {
      hasArgument = true;

      if (!str2ip(TmpStr1, IP)) {
        String result = F("Invalid parameter: ");
        result += TmpStr1;
        return return_result(event, result);
      }
    }
  }

  if (!hasArgument) {
    serialPrintln();
    String result = targetDescription;

    if (useStaticIP()) {
      result += formatIP(IP);
    } else {
      result += formatIP(dhcpIP);
      result += F("(DHCP)");
    }
    return return_result(event, result);
  }
  return return_command_success();
}

String Command_GetORSetString(struct EventStruct *event,
                              const String      & targetDescription,
                              const char         *Line,
                              char               *target,
                              size_t              len,
                              int                 arg
                              )
{
  bool hasArgument = false;
  {
    // Check if command is valid. Leave in separate scope to delete the TmpStr1
    String TmpStr1;

    if (GetArgv(Line, TmpStr1, arg + 1)) {
      hasArgument = true;

      if (TmpStr1.length() > len) {
        String result = targetDescription;
        result += F(" is too large. max size is ");
        result += len;
        serialPrintln();
        return return_result(event, result);
      }
      strcpy(target, TmpStr1.c_str());
    }
  }

  if (hasArgument) {
    serialPrintln();
    String result = targetDescription;
    result += target;
    return return_result(event, result);
  }
  return return_command_success();
}

String Command_GetORSetBool(struct EventStruct *event,
                            const String      & targetDescription,
                            const char         *Line,
                            bool               *value,
                            int                 arg)
{
  bool hasArgument = false;
  {
    // Check if command is valid. Leave in separate scope to delete the TmpStr1
    String TmpStr1;

    if (GetArgv(Line, TmpStr1, arg + 1)) {
      hasArgument = true;
      TmpStr1.toLowerCase();

      if (isInt(TmpStr1)) {
        *value = atoi(TmpStr1.c_str()) > 0;
      }
      else if (strcmp_P(PSTR("on"), TmpStr1.c_str()) == 0) { *value = true; }
      else if (strcmp_P(PSTR("true"), TmpStr1.c_str()) == 0) { *value = true; }
      else if (strcmp_P(PSTR("off"), TmpStr1.c_str()) == 0) { *value = false; }
      else if (strcmp_P(PSTR("false"), TmpStr1.c_str()) == 0) { *value = false; }
    }
  }

  if (hasArgument) {
    String result = targetDescription;
    result += boolToString(*value);
    return return_result(event, result);
  }
  return return_command_success();
}

String Command_GetORSetUint8_t(struct EventStruct *event,
                            const String      & targetDescription,
                            const char         *Line,
                            uint8_t            *value,
                            int                 arg)
{
  bool hasArgument = false;
  {
    // Check if command is valid. Leave in separate scope to delete the TmpStr1
    String TmpStr1;

    if (GetArgv(Line, TmpStr1, arg + 1)) {
      hasArgument = true;
      TmpStr1.toLowerCase();

      if (isInt(TmpStr1)) {
        *value = (uint8_t)atoi(TmpStr1.c_str());
      }
      else if (strcmp_P(PSTR("WIFI"), TmpStr1.c_str()) == 0) { *value = 0; }
      else if (strcmp_P(PSTR("ETHERNET"), TmpStr1.c_str()) == 0) { *value = 1; }
    }
  }

  if (hasArgument) {
    String result = targetDescription;
    result += *value;
    return return_result(event, result);
  }
  return return_command_success();
}

String Command_GetORSetInt8_t(struct EventStruct *event,
                            const String      & targetDescription,
                            const char         *Line,
                            int8_t             *value,
                            int                 arg)
{
  bool hasArgument = false;
  {
    // Check if command is valid. Leave in separate scope to delete the TmpStr1
    String TmpStr1;

    if (GetArgv(Line, TmpStr1, arg + 1)) {
      hasArgument = true;
      TmpStr1.toLowerCase();

      if (isInt(TmpStr1)) {
        *value = (int8_t)atoi(TmpStr1.c_str());
      }
    }
  }

  if (hasArgument) {
    String result = targetDescription;
    result += *value;
    return return_result(event, result);
  }
  return return_command_success();
}

#include "../Commands/Rules.h"


#include "../../ESPEasy_common.h"


#include "../Commands/Common.h"

#include "../DataTypes/EventValueSource.h"

#include "../ESPEasyCore/Controller.h"
#include "../ESPEasyCore/ESPEasyRules.h"

#include "../Globals/EventQueue.h"
#include "../Globals/Settings.h"

#include "../Helpers/Misc.h"
#include "../Helpers/Rules_calculate.h"
#include "../Helpers/StringConverter.h"

String Command_Rules_Execute(struct EventStruct *event, const char *Line)
{
  String filename;

  if (GetArgv(Line, filename, 2)) {
    String event;
    rulesProcessingFile(filename, event);
  }
  return return_command_success();
}

String Command_Rules_UseRules(struct EventStruct *event, const char *Line)
{
  return Command_GetORSetBool(event, F("Rules:"),
                              Line,
                              (bool *)&Settings.UseRules,
                              1);
}

String Command_Rules_Async_Events(struct EventStruct *event, const char *Line)
{
  String eventName = parseStringToEndKeepCase(Line, 2);
  eventName.replace('$', '#');

  if (Settings.UseRules) {
    eventQueue.add(eventName);
  }
  return return_command_success();
}


String Command_Rules_Events(struct EventStruct *event, const char *Line)
{
  String eventName = parseStringToEndKeepCase(Line, 2);
  eventName.replace('$', '#');

  if (Settings.UseRules) {
    const bool executeImmediately = 
        SourceNeedsStatusUpdate(event->Source) ||
        event->Source == EventValueSource::Enum::VALUE_SOURCE_RULES;
    if (executeImmediately) {
      rulesProcessing(eventName); // TD-er: Process right now 
    } else {
      eventQueue.add(eventName);
    }
  }
  return return_command_success();
}

String Command_Rules_Let(struct EventStruct *event, const char *Line)
{
  String TmpStr1;

  if (GetArgv(Line, TmpStr1, 3)) {
    float result = 0.0f;
    Calculate(TmpStr1.c_str(), &result);
    customFloatVar[event->Par1 - 1] = result;
  }
  return return_command_success();
}

#include "../Commands/InternalCommands.h"

#include "../../ESPEasy_common.h"

#include "../../_Plugin_Helper.h"
#include "../Globals/Settings.h"

#ifdef USES_BLYNK
# include "../Commands/Blynk.h"
# include "../Commands/Blynk_c015.h"
#endif // ifdef USES_BLYNK

#include "../Commands/Common.h"
#include "../Commands/Controller.h"
#include "../Commands/Diagnostic.h"
#include "../Commands/GPIO.h"
#include "../Commands/HTTP.h"
#include "../Commands/i2c.h"

#ifdef USES_MQTT
# include "../Commands/MQTT.h"
#endif // USES_MQTT

#include "../Commands/Networks.h"
#include "../Commands/Notifications.h"
#include "../Commands/RTC.h"
#include "../Commands/Rules.h"
#include "../Commands/SDCARD.h"
#include "../Commands/Settings.h"
#include "../Commands/Servo.h"
#include "../Commands/System.h"
#include "../Commands/Tasks.h"
#include "../Commands/Time.h"
#include "../Commands/Timer.h"
#include "../Commands/UPD.h"
#include "../Commands/wd.h"
#include "../Commands/WiFi.h"

#include "../ESPEasyCore/ESPEasy_Log.h"

#include "../Helpers/Misc.h"
#include "../Helpers/StringConverter.h"
#include "../Helpers/StringParser.h"


bool checkNrArguments(const char *cmd, const char *Line, int nrArguments) {
  if (nrArguments < 0) { return true; }

  // 0 arguments means argument on pos1 is valid (the command) and argpos 2 should not be there.
  if (HasArgv(Line, nrArguments + 2)) {
    if (loglevelActiveFor(LOG_LEVEL_ERROR)) {
      String log;
      log.reserve(128);
      log += F("Too many arguments: cmd=");
      log += cmd;

      if (nrArguments < 1) {
        log += Line;
      } else {
        // Check for one more argument than allowed, since we apparently have one.
        bool done = false;
        int  i    = 1;

        while (!done) {
          String parameter;

          if (i == nrArguments) {
            parameter = tolerantParseStringKeepCase(Line, i + 1);
          } else {
            parameter = parseStringKeepCase(Line, i + 1);
          }
          done = parameter.length() == 0;

          if (!done) {
            if (i <= nrArguments) {
              if (Settings.TolerantLastArgParse() && (i == nrArguments)) {
                log += F(" (fixed)");
              }
              log += F(" Arg");
            } else {
              log += F(" ExtraArg");
            }
            log += String(i);
            log += '=';
            log += parameter;
          }
          ++i;
        }
      }
      log += F(" lineLength=");
      log += strlen(Line);
      addLog(LOG_LEVEL_ERROR, log);
      log  = F("Line: _");
      log += Line;
      log += '_';
      addLog(LOG_LEVEL_ERROR, log);

      if (!Settings.TolerantLastArgParse()) {
        log = F("Command not executed!");
      } else {
        log = F("Command executed, but may fail.");
      }
      log += F(" See: https://github.com/letscontrolit/ESPEasy/issues/2724");
      addLog(LOG_LEVEL_ERROR, log);
    }

    if (Settings.TolerantLastArgParse()) {
      return true;
    }
    return false;
  }
  return true;
}

bool checkSourceFlags(EventValueSource::Enum source, EventValueSourceGroup::Enum group) {
  if (EventValueSource::partOfGroup(source, group)) {
    return true;
  }
  addLog(LOG_LEVEL_ERROR, return_incorrect_source());
  return false;
}

command_case_data::command_case_data(const char *cmd, struct EventStruct *event, const char *line) :
  cmd(cmd), event(event), line(line)
{
  cmd_lc = cmd;
  cmd_lc.toLowerCase();
}


bool do_command_case(command_case_data         & data,
                     const String              & cmd_test,
                     command_function            pFunc,
                     int                         nrArguments,
                     EventValueSourceGroup::Enum group)
{
  // The data struct is re-used on each attempt to process an internal command.
  // Re-initialize the only two members that may have been altered by a previous call.
  data.retval = false;
  data.status = "";
  if (!data.cmd_lc.equals(cmd_test)) {
    return false;
  }
  if (!checkSourceFlags(data.event->Source, group)) {
    data.status = return_incorrect_source();
    return false;
  } 
  // FIXME TD-er: Do not check nr arguments from MQTT source.
  // See https://github.com/letscontrolit/ESPEasy/issues/3344
  // C005 does recreate command partly from topic and published message
  // e.g. ESP_Easy/Bathroom_pir_env/GPIO/14 with data 0 or 1
  // This only allows for 2 parameters, but some commands need more arguments (default to "0")
  const bool mustCheckNrArguments = data.event->Source != EventValueSource::Enum::VALUE_SOURCE_MQTT;
  if (mustCheckNrArguments) {
    if (!checkNrArguments(data.cmd, data.line, nrArguments)) {
      data.status = return_incorrect_nr_arguments();
      data.retval = false;
      return true; // Command is handled
    }
  } 
  data.status = pFunc(data.event, data.line);
  data.retval = true;
  return true; // Command is handled
}

bool executeInternalCommand(command_case_data & data)
{
  // Simple macro to match command to function call.

  // EventValueSourceGroup::Enum::ALL
  #define COMMAND_CASE_A(S, C, NARGS) \
  if (do_command_case(data, F(S), &C, NARGS, EventValueSourceGroup::Enum::ALL)) { return data.retval; }

  // EventValueSourceGroup::Enum::RESTRICTED
  #define COMMAND_CASE_R(S, C, NARGS) \
  if (do_command_case(data, F(S), &C, NARGS, EventValueSourceGroup::Enum::RESTRICTED)) { return data.retval; }

  // FIXME TD-er: Should we execute command when number of arguments is wrong?

  // FIXME TD-er: must determine nr arguments where NARGS is set to -1

  switch (data.cmd_lc[0]) {
    case 'a': {
      COMMAND_CASE_A("accessinfo", Command_AccessInfo_Ls,       0); // Network Command
      COMMAND_CASE_A("asyncevent", Command_Rules_Async_Events, -1); // Rule.h
      break;
    }
    case 'b': {
    #ifndef BUILD_NO_DIAGNOSTIC_COMMANDS
      COMMAND_CASE_R("background", Command_Background, 1); // Diagnostic.h
    #endif // ifndef BUILD_NO_DIAGNOSTIC_COMMANDS
    #ifdef USES_C012
      COMMAND_CASE_A("blynkget", Command_Blynk_Get, -1);
    #endif // ifdef USES_C012
    #ifdef USES_C015
      COMMAND_CASE_R("blynkset", Command_Blynk_Set, -1);
    #endif // ifdef USES_C015
      COMMAND_CASE_A("build", Command_Settings_Build, 1);      // Settings.h
      break;
    }
    case 'c': {
      COMMAND_CASE_R( "clearaccessblock", Command_AccessInfo_Clear,   0); // Network Command
      COMMAND_CASE_R(      "clearrtcram", Command_RTC_Clear,          0); // RTC.h
      COMMAND_CASE_R(           "config", Command_Task_RemoteConfig, -1); // Tasks.h
      COMMAND_CASE_R("controllerdisable", Command_Controller_Disable, 1); // Controller.h
      COMMAND_CASE_R( "controllerenable", Command_Controller_Enable,  1); // Controller.h

      break;
    }
    case 'd': {
      COMMAND_CASE_R( "datetime", Command_DateTime,         2); // Time.h
      COMMAND_CASE_R(    "debug", Command_Debug,            1); // Diagnostic.h
      COMMAND_CASE_R("deepsleep", Command_System_deepSleep, 1); // System.h
      COMMAND_CASE_R(    "delay", Command_Delay,            1); // Timers.h
      COMMAND_CASE_R(      "dns", Command_DNS,              1); // Network Command
      COMMAND_CASE_R(      "dst", Command_DST,              1); // Time.h
      break;
    }
    case 'e': {
    #ifdef HAS_ETHERNET
      COMMAND_CASE_R(   "ethphyadr", Command_ETH_Phy_Addr,   1); // Network Command
      COMMAND_CASE_R(   "ethpinmdc", Command_ETH_Pin_mdc,    1); // Network Command
      COMMAND_CASE_R(  "ethpinmdio", Command_ETH_Pin_mdio,   1); // Network Command
      COMMAND_CASE_R( "ethpinpower", Command_ETH_Pin_power,  1); // Network Command
      COMMAND_CASE_R(  "ethphytype", Command_ETH_Phy_Type,   1); // Network Command
      COMMAND_CASE_R("ethclockmode", Command_ETH_Clock_Mode, 1); // Network Command
      COMMAND_CASE_R(       "ethip", Command_ETH_IP,         1); // Network Command
      COMMAND_CASE_R(  "ethgateway", Command_ETH_Gateway,    1); // Network Command
      COMMAND_CASE_R(   "ethsubnet", Command_ETH_Subnet,     1); // Network Command  
      COMMAND_CASE_R(      "ethdns", Command_ETH_DNS,        1); // Network Command
      COMMAND_CASE_R( "ethwifimode", Command_ETH_Wifi_Mode,  1); // Network Command
    #endif // HAS_ETHERNET
      COMMAND_CASE_R("erasesdkwifi", Command_WiFi_Erase,     0); // WiFi.h
      COMMAND_CASE_A(       "event", Command_Rules_Events,  -1); // Rule.h
      COMMAND_CASE_A("executerules", Command_Rules_Execute, -1); // Rule.h
      break;
    }
    case 'g': {
      COMMAND_CASE_R("gateway", Command_Gateway, 1);        // Network Command
      COMMAND_CASE_A(      "gpio", Command_GPIO,        2); // Gpio.h
      COMMAND_CASE_A("gpiotoggle", Command_GPIO_Toggle, 1); // Gpio.h
      break;
    }
    case 'i': {
      COMMAND_CASE_R("i2cscanner", Command_i2c_Scanner, -1); // i2c.h
      COMMAND_CASE_R(        "ip", Command_IP,           1); // Network Command
      break;
    }
    case 'j': {
      #ifndef BUILD_NO_DIAGNOSTIC_COMMANDS
      COMMAND_CASE_A("jsonportstatus", Command_JSONPortStatus, -1); // Diagnostic.h
      #endif // ifndef BUILD_NO_DIAGNOSTIC_COMMANDS
      break;
    }
    case 'l': {
      COMMAND_CASE_A(          "let", Command_Rules_Let,     2); // Rules.h
      COMMAND_CASE_A(         "load", Command_Settings_Load, 0); // Settings.h
      COMMAND_CASE_A(     "logentry", Command_logentry,      1); // Diagnostic.h
      COMMAND_CASE_A(   "looptimerset", Command_Loop_Timer_Set,    3); // Timers.h
      COMMAND_CASE_A("looptimerset_ms", Command_Loop_Timer_Set_ms, 3); // Timers.h
      COMMAND_CASE_A(    "longpulse", Command_GPIO_LongPulse,   3);    // GPIO.h
      COMMAND_CASE_A( "longpulse_ms", Command_GPIO_LongPulse_Ms,3);    // GPIO.h
    #ifndef BUILD_NO_DIAGNOSTIC_COMMANDS
      COMMAND_CASE_A("logportstatus", Command_logPortStatus,    0); // Diagnostic.h
      COMMAND_CASE_A(       "lowmem", Command_Lowmem,           0); // Diagnostic.h
    #endif // ifndef BUILD_NO_DIAGNOSTIC_COMMANDS
      break;
    }
    case 'm': {
      if (data.cmd_lc[1] == 'c') {
        COMMAND_CASE_A(        "mcpgpio", Command_GPIO,              2); // Gpio.h
        COMMAND_CASE_A(  "mcpgpiotoggle", Command_GPIO_Toggle,       1); // Gpio.h
        COMMAND_CASE_A(   "mcplongpulse", Command_GPIO_LongPulse,    3); // GPIO.h
        COMMAND_CASE_A("mcplongpulse_ms", Command_GPIO_LongPulse_Ms, 3); // GPIO.h
        COMMAND_CASE_A(       "mcppulse", Command_GPIO_Pulse,        3); // GPIO.h
      }
      COMMAND_CASE_A(      "monitor", Command_GPIO_Monitor,   2); // GPIO.h
    #ifndef BUILD_NO_DIAGNOSTIC_COMMANDS
      COMMAND_CASE_A(       "malloc", Command_Malloc,         1);        // Diagnostic.h
      COMMAND_CASE_A(      "meminfo", Command_MemInfo,        0);        // Diagnostic.h
      COMMAND_CASE_A("meminfodetail", Command_MemInfo_detail, 0);        // Diagnostic.h
    #endif // ifndef BUILD_NO_DIAGNOSTIC_COMMANDS

      break;
    }
    case 'n': {
      COMMAND_CASE_R(   "name", Command_Settings_Name,        1); // Settings.h
      COMMAND_CASE_R("nosleep", Command_System_NoSleep,       1); // System.h
      COMMAND_CASE_R( "notify", Command_Notifications_Notify, 2); // Notifications.h
      COMMAND_CASE_R("ntphost", Command_NTPHost,              1); // Time.h
      break;
    }
    case 'p': {
      if (data.cmd_lc[1] == 'c') {
        COMMAND_CASE_A(        "pcfgpio", Command_GPIO,              2); // Gpio.h
        COMMAND_CASE_A(  "pcfgpiotoggle", Command_GPIO_Toggle,       1); // Gpio.h
        COMMAND_CASE_A(   "pcflongpulse", Command_GPIO_LongPulse,    3); // GPIO.h
        COMMAND_CASE_A("pcflongpulse_ms", Command_GPIO_LongPulse_Ms, 3); // GPIO.h
        COMMAND_CASE_A(       "pcfpulse", Command_GPIO_Pulse,        3); // GPIO.h
      }
      COMMAND_CASE_R("password", Command_Settings_Password, 1);          // Settings.h
      COMMAND_CASE_A(   "pulse", Command_GPIO_Pulse,        3); // GPIO.h
#ifdef USES_MQTT
      COMMAND_CASE_A("publish", Command_MQTT_Publish, 2);                // MQTT.h
#endif // USES_MQTT
      COMMAND_CASE_A(    "pwm", Command_GPIO_PWM,        4); // GPIO.h
      break;
    }
    case 'r': {
      COMMAND_CASE_A("reboot", Command_System_Reboot, 0);                              // System.h
      COMMAND_CASE_R("reset", Command_Settings_Reset, 0);                              // Settings.h
      COMMAND_CASE_A("resetflashwritecounter", Command_RTC_resetFlashWriteCounter, 0); // RTC.h
      COMMAND_CASE_A(               "restart", Command_System_Reboot,              0); // System.h
      COMMAND_CASE_A(                 "rtttl", Command_GPIO_RTTTL,                -1); // GPIO.h
      COMMAND_CASE_A(                 "rules", Command_Rules_UseRules,             1); // Rule.h
      break;
    }
    case 's': {
      COMMAND_CASE_R(    "save", Command_Settings_Save, 0); // Settings.h
    #ifdef FEATURE_SD
      COMMAND_CASE_R(  "sdcard", Command_SD_LS,         0); // SDCARDS.h
      COMMAND_CASE_R("sdremove", Command_SD_Remove,     1); // SDCARDS.h
    #endif // ifdef FEATURE_SD

      if (data.cmd_lc[1] == 'e') {
        COMMAND_CASE_A(    "sendto", Command_UPD_SendTo,      2); // UDP.h    // FIXME TD-er: These send commands, can we determine the nr
                                                                  // of
                                                                  // arguments?
        COMMAND_CASE_A("sendtohttp", Command_HTTP_SendToHTTP, 3); // HTTP.h
        COMMAND_CASE_A( "sendtoudp", Command_UDP_SendToUPD,   3); // UDP.h
    #ifndef BUILD_NO_DIAGNOSTIC_COMMANDS
        COMMAND_CASE_R("serialfloat", Command_SerialFloat,    0); // Diagnostic.h
    #endif // ifndef BUILD_NO_DIAGNOSTIC_COMMANDS
        COMMAND_CASE_R(   "settings", Command_Settings_Print, 0); // Settings.h
        COMMAND_CASE_A(      "servo", Command_Servo,          3); // Servo.h
      }
      COMMAND_CASE_A("status", Command_GPIO_Status,          2); // GPIO.h
      COMMAND_CASE_R("subnet", Command_Subnet, 1);                // Network Command
    #ifdef USES_MQTT
      COMMAND_CASE_A("subscribe", Command_MQTT_Subscribe, 1);     // MQTT.h
    #endif // USES_MQTT
    #ifndef BUILD_NO_DIAGNOSTIC_COMMANDS
      COMMAND_CASE_A(  "sysload", Command_SysLoad,        0);     // Diagnostic.h
    #endif // ifndef BUILD_NO_DIAGNOSTIC_COMMANDS
      break;
    }
    case 't': {
      if (data.cmd_lc[1] == 'a') {
        COMMAND_CASE_R(   "taskclear", Command_Task_Clear,    1);             // Tasks.h
        COMMAND_CASE_R("taskclearall", Command_Task_ClearAll, 0);             // Tasks.h
        COMMAND_CASE_R( "taskdisable", Command_Task_Disable,  1);             // Tasks.h
        COMMAND_CASE_R(  "taskenable", Command_Task_Enable,   1);             // Tasks.h
        COMMAND_CASE_A(           "taskrun", Command_Task_Run,            1); // Tasks.h
        COMMAND_CASE_A(      "taskvalueset", Command_Task_ValueSet,       3); // Tasks.h
        COMMAND_CASE_A(   "taskvaluetoggle", Command_Task_ValueToggle,    2); // Tasks.h
        COMMAND_CASE_A("taskvaluesetandrun", Command_Task_ValueSetAndRun, 3); // Tasks.h
      } else if (data.cmd_lc[1] == 'i') {
        COMMAND_CASE_A( "timerpause", Command_Timer_Pause,  1);               // Timers.h
        COMMAND_CASE_A("timerresume", Command_Timer_Resume, 1);               // Timers.h
        COMMAND_CASE_A(   "timerset", Command_Timer_Set,    2);               // Timers.h
        COMMAND_CASE_A("timerset_ms", Command_Timer_Set_ms, 2); // Timers.h
        COMMAND_CASE_R("timezone", Command_TimeZone, 1);                      // Time.h
      }
      COMMAND_CASE_A(      "tone", Command_GPIO_Tone, 3); // GPIO.h
      break;
    }
    case 'u': {
      COMMAND_CASE_R("udpport", Command_UDP_Port,      1);    // UDP.h
      COMMAND_CASE_R("udptest", Command_UDP_Test,      2);    // UDP.h
      COMMAND_CASE_R(   "unit", Command_Settings_Unit, 1);    // Settings.h
      COMMAND_CASE_A("unmonitor", Command_GPIO_UnMonitor, 2); // GPIO.h
      COMMAND_CASE_R("usentp", Command_useNTP, 1);            // Time.h
      break;
    }
    case 'w': {
      COMMAND_CASE_R("wdconfig", Command_WD_Config, 3);               // WD.h
      COMMAND_CASE_R(  "wdread", Command_WD_Read,   2);               // WD.h

      if (data.cmd_lc[1] == 'i') {
        COMMAND_CASE_R(    "wifiapmode", Command_Wifi_APMode,     0); // WiFi.h
        COMMAND_CASE_A(   "wificonnect", Command_Wifi_Connect,    0); // WiFi.h
        COMMAND_CASE_A("wifidisconnect", Command_Wifi_Disconnect, 0); // WiFi.h
        COMMAND_CASE_R(       "wifikey", Command_Wifi_Key,        1); // WiFi.h
        COMMAND_CASE_R(      "wifikey2", Command_Wifi_Key2,       1); // WiFi.h
        COMMAND_CASE_R(      "wifimode", Command_Wifi_Mode,       1); // WiFi.h
        COMMAND_CASE_R(      "wifiscan", Command_Wifi_Scan,       0); // WiFi.h
        COMMAND_CASE_R(      "wifissid", Command_Wifi_SSID,       1); // WiFi.h
        COMMAND_CASE_R(     "wifissid2", Command_Wifi_SSID2,      1); // WiFi.h
        COMMAND_CASE_R(   "wifistamode", Command_Wifi_STAMode,    0); // WiFi.h
      }
      break;
    }
    default:
      break;
  }

  #undef COMMAND_CASE_R
  #undef COMMAND_CASE_A
  return false;
}

// Execute command which may be plugin or internal commands
bool ExecuteCommand_all(EventValueSource::Enum source, const char *Line)
{
  return ExecuteCommand(INVALID_TASK_INDEX, source, Line, true, true, false);
}

bool ExecuteCommand_all_config(EventValueSource::Enum source, const char *Line)
{
  return ExecuteCommand(INVALID_TASK_INDEX, source, Line, true, true, true);
}

bool ExecuteCommand_plugin_config(EventValueSource::Enum source, const char *Line)
{
  return ExecuteCommand(INVALID_TASK_INDEX, source, Line, true, false, true);
}

bool ExecuteCommand_all_config_eventOnly(EventValueSource::Enum source, const char *Line)
{
  bool tryInternal = false;
  {
    String cmd;

    if (GetArgv(Line, cmd, 1)) {
      tryInternal = cmd.equalsIgnoreCase(F("event"));
    }
  }

  return ExecuteCommand(INVALID_TASK_INDEX, source, Line, true, tryInternal, true);
}

bool ExecuteCommand_internal(EventValueSource::Enum source, const char *Line)
{
  return ExecuteCommand(INVALID_TASK_INDEX, source, Line, false, true, false);
}

bool ExecuteCommand_plugin(EventValueSource::Enum source, const char *Line)
{
  return ExecuteCommand(INVALID_TASK_INDEX, source, Line, true, false, false);
}

bool ExecuteCommand_plugin(taskIndex_t taskIndex, EventValueSource::Enum source, const char *Line)
{
  return ExecuteCommand(taskIndex, source, Line, true, false, false);
}

bool ExecuteCommand(taskIndex_t            taskIndex,
                    EventValueSource::Enum source,
                    const char            *Line,
                    bool                   tryPlugin,
                    bool                   tryInternal,
                    bool                   tryRemoteConfig)
{
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("ExecuteCommand"));
  #endif
  String cmd;

  if (!GetArgv(Line, cmd, 1)) {
    SendStatus(source, return_command_failed());
    return false;
  }

  if (tryInternal) {
    // Small optimization for events, which happen frequently
    // FIXME TD-er: Make quick check to see if a command is an internal command, so we don't need to try all
    if (cmd.equalsIgnoreCase(F("event"))) {
      tryPlugin       = false;
      tryRemoteConfig = false;
    }
  }

  // FIXME TD-er: Not sure what happens now, but TaskIndex cannot always be set here
  // since commands can originate from anywhere.
  struct EventStruct TempEvent;
  TempEvent.setTaskIndex(taskIndex);
  checkDeviceVTypeForTask(&TempEvent);
  TempEvent.Source = source;

  String action(Line);
  action = parseTemplate(action); // parseTemplate before executing the command

  // Split the arguments into Par1...5 of the event.
  // Do not split it in executeInternalCommand, since that one will be called from the scheduler with pre-set events.
  // FIXME TD-er: Why call this for all commands? The CalculateParam function is quite heavy.
  parseCommandString(&TempEvent, action);

  // FIXME TD-er: This part seems a bit strange.
  // It can't schedule a call to PLUGIN_WRITE.
  // Maybe ExecuteCommand can be scheduled?
  delay(0);

  if (loglevelActiveFor(LOG_LEVEL_DEBUG)) {
    String log = F("Command: ");
    log += cmd;
    addLog(LOG_LEVEL_DEBUG, log);
#ifndef BUILD_NO_DEBUG
    addLog(LOG_LEVEL_DEBUG, Line); // for debug purposes add the whole line.
    String parameters;
    parameters.reserve(64);
    parameters += F("Par1: ");
    parameters += TempEvent.Par1;
    parameters += F(" Par2: ");
    parameters += TempEvent.Par2;
    parameters += F(" Par3: ");
    parameters += TempEvent.Par3;
    parameters += F(" Par4: ");
    parameters += TempEvent.Par4;
    parameters += F(" Par5: ");
    parameters += TempEvent.Par5;
    addLog(LOG_LEVEL_DEBUG, parameters);
#endif // ifndef BUILD_NO_DEBUG
  }


  if (tryInternal) {
    command_case_data data(cmd.c_str(), &TempEvent, action.c_str());
    bool   handled = executeInternalCommand(data);

    if (data.status.length() > 0) {
      delay(0);
      SendStatus(source, data.status);
      delay(0);
    }

    if (handled) {
      return true;
    }
  }

  if (tryPlugin) {
    // Use a tmp string to call PLUGIN_WRITE, since PluginCall may inadvertenly
    // alter the string.
    String tmpAction(action);
    bool   handled = PluginCall(PLUGIN_WRITE, &TempEvent, tmpAction);
    
    #ifndef BUILD_NO_DEBUG
    if (!tmpAction.equals(action)) {
      if (loglevelActiveFor(LOG_LEVEL_ERROR)) {
        String log = F("PLUGIN_WRITE altered the string: ");
        log += action;
        log += F(" to: ");
        log += tmpAction;
        addLog(LOG_LEVEL_ERROR, log);
      }
    }
    #endif

    if (handled) {
      SendStatus(source, return_command_success());
      return true;
    }
  }

  if (tryRemoteConfig) {
    if (remoteConfig(&TempEvent, action)) {
      SendStatus(source, return_command_success());
      return true;
    }
  }
  String errorUnknown = F("Command unknown: ");
  errorUnknown += action;
  addLog(LOG_LEVEL_INFO, errorUnknown);
  SendStatus(source, errorUnknown);
  delay(0);
  return false;
}

#include "../Commands/Tasks.h"



#include "../../ESPEasy_common.h"


#include "../Commands/Common.h"

#include "../ESPEasyCore/Controller.h"
#include "../ESPEasyCore/Serial.h"

#include "../Helpers/Misc.h"
#include "../Helpers/Rules_calculate.h"
#include "../Helpers/StringConverter.h"

//      taskIndex = (event->Par1 - 1);   Par1 is here for 1 ... TASKS_MAX
//	varNr = event->Par2 - 1;
bool validTaskVars(struct EventStruct *event, taskIndex_t& taskIndex, unsigned int& varNr)
{
  if (event == nullptr) { return false; }
  if (event->Par1 <= 0) { return false; }
  taskIndex_t tmp_taskIndex = static_cast<taskIndex_t>(event->Par1 - 1);
  varNr     = 0;

  if (event->Par2 > 0) {
    varNr = event->Par2 - 1;
  }

  if (!validTaskIndex(tmp_taskIndex)) { return false; }

  if (varNr >= VARS_PER_TASK) { return false; }

  taskIndex = tmp_taskIndex;

  return true;
}

String Command_Task_Clear(struct EventStruct *event, const char *Line)
{
  taskIndex_t  taskIndex;
  unsigned int varNr;

  if (!validTaskVars(event, taskIndex, varNr)) { return return_command_failed(); }

  taskClear(taskIndex, true);
  return return_command_success();
}

String Command_Task_ClearAll(struct EventStruct *event, const char *Line)
{
  for (taskIndex_t t = 0; t < TASKS_MAX; t++) {
    taskClear(t, false);
  }
  return return_command_success();
}

String Command_Task_EnableDisable(struct EventStruct *event, bool enable)
{
  taskIndex_t  taskIndex;
  unsigned int varNr;
  String dummy;

  if (validTaskVars(event, taskIndex, varNr)) {
    // This is a command so no guarantee the taskIndex is correct in the event
    event->setTaskIndex(taskIndex);
    if (setTaskEnableStatus(event, enable)) {
      return return_command_success();
    }
  }
  return return_command_failed();
}

String Command_Task_Disable(struct EventStruct *event, const char *Line)
{
  return Command_Task_EnableDisable(event, false);
}

String Command_Task_Enable(struct EventStruct *event, const char *Line)
{
  return Command_Task_EnableDisable(event, true);
}

String Command_Task_ValueSet(struct EventStruct *event, const char *Line)
{
  String TmpStr1;
  taskIndex_t  taskIndex;
  unsigned int varNr;

  if (!validTaskVars(event, taskIndex, varNr)) { return return_command_failed(); }
  unsigned int uservarIndex = (VARS_PER_TASK * taskIndex) + varNr;

  if (GetArgv(Line, TmpStr1, 4)) {
    // Perform calculation with float result.
    float result = 0;
    Calculate(TmpStr1.c_str(), &result);

    // FIXME TD-er: The return code of Calculate is not used.
    UserVar[uservarIndex] = result;
  } else  {
    // TODO: Get Task description and var name
    serialPrintln(String(UserVar[uservarIndex]));
  }
  return return_command_success();
}

String Command_Task_ValueToggle(struct EventStruct *event, const char *Line)
{
  taskIndex_t  taskIndex;
  unsigned int varNr;

  if (!validTaskVars(event, taskIndex, varNr)) { return return_command_failed(); }
  unsigned int uservarIndex = (VARS_PER_TASK * taskIndex) + varNr;
  const int    result       = round(UserVar[uservarIndex]);

  if ((result == 0) || (result == 1)) {
    UserVar[uservarIndex] = (result == 0) ? 1.0f : 0.0f;
  }
  return return_command_success();
}

String Command_Task_ValueSetAndRun(struct EventStruct *event, const char *Line)
{
  String TmpStr1;

  if (GetArgv(Line, TmpStr1, 4)) {
    taskIndex_t  taskIndex;
    unsigned int varNr;

    if (!validTaskVars(event, taskIndex, varNr)) { return return_command_failed(); }
    unsigned int uservarIndex = (VARS_PER_TASK * taskIndex) + varNr;

    float result = 0;
    Calculate(TmpStr1.c_str(), &result);
    UserVar[uservarIndex] = result;
    SensorSendTask(taskIndex);
  }
  return return_command_success();
}

String Command_Task_Run(struct EventStruct *event, const char *Line)
{
  taskIndex_t  taskIndex;
  unsigned int varNr;

  if (!validTaskVars(event, taskIndex, varNr)) { return return_command_failed(); }

  SensorSendTask(taskIndex);
  return return_command_success();
}

String Command_Task_RemoteConfig(struct EventStruct *event, const char *Line)
{
  struct EventStruct TempEvent(event->TaskIndex);
  String request = Line;

  // FIXME TD-er: Should we call ExecuteCommand here? The command is not parsed like any other call.
  remoteConfig(&TempEvent, request);
  return return_command_success();
}

#include "../Commands/SDCARD.h"

#include "../../ESPEasy_common.h"
#include "../Commands/Common.h"
#include "../ESPEasyCore/Serial.h"
#include "../Globals/Settings.h"




#ifdef FEATURE_SD

#include <SD.h>


void printDirectory(File dir, int numTabs)
{
  while (true) {
    File entry = dir.openNextFile();

    if (!entry) {
      // no more files
      break;
    }

    for (uint8_t i = 0; i < numTabs; i++) {
      serialPrint("\t");
    }
    serialPrint(entry.name());

    if (entry.isDirectory()) {
      serialPrintln("/");
      printDirectory(entry, numTabs + 1);
    } else {
      // files have sizes, directories do not
      serialPrint("\t\t");
      serialPrintln(String(entry.size(), DEC));
    }
    entry.close();
  }
}


String Command_SD_LS(struct EventStruct *event, const char* Line)
{
  File root = SD.open("/");
  root.rewindDirectory();
  printDirectory(root, 0);
  root.close();
  return return_see_serial(event);
}

String Command_SD_Remove(struct EventStruct *event, const char* Line)
{
  // FIXME TD-er: This one is not using parseString* function
  String fname = Line;
  fname = fname.substring(9);
  String result = F("Removing:");
  result += fname.c_str();
  SD.remove((char*)fname.c_str());
  return return_result(event, result);
}
#endif

#include "../Commands/RTC.h"

#include "../../ESPEasy_common.h"


#include "../Commands/Common.h"

#include "../DataStructs/RTCStruct.h"

#include "../Globals/RTC.h"

#include "../Helpers/ESPEasyRTC.h"


String Command_RTC_Clear(struct EventStruct *event, const char* Line)
{
	initRTC();
	return return_command_success();
}

String Command_RTC_resetFlashWriteCounter(struct EventStruct *event, const char* Line)
{
	RTC.flashDayCounter = 0;
	return return_command_success();
}

#include "../Commands/Time.h"


#include "../../ESPEasy_common.h"


#include "../Commands/Common.h"

#include "../ESPEasyCore/Serial.h"

#include "../Globals/ESPEasy_time.h"
#include "../Globals/Settings.h"

#include "../Helpers/ESPEasy_time_calc.h"
#include "../Helpers/StringConverter.h"


String Command_NTPHost(struct EventStruct *event, const char *Line)
{
  return Command_GetORSetString(event, F("NTPHost:"),
                                Line,
                                Settings.NTPHost,
                                sizeof(Settings.NTPHost),
                                1);
}

String Command_useNTP(struct EventStruct *event, const char *Line)
{
  if (HasArgv(Line, 2)) {
    Settings.UseNTP = event->Par1;
  } else {
    serialPrintln();
    String result = F("UseNTP:");
    result += boolToString(Settings.UseNTP);
    return return_result(event, result);
  }
  return return_command_success();
}

String Command_TimeZone(struct EventStruct *event, const char *Line)
{
  if (HasArgv(Line, 2)) {
    Settings.TimeZone = event->Par1;
  } else {
    serialPrintln();
    String result = F("TimeZone:");
    result += Settings.TimeZone;
    return return_result(event, result);
  }
  return return_command_success();
}

String Command_DST(struct EventStruct *event, const char *Line)
{
  if (HasArgv(Line, 2)) {
    Settings.DST = event->Par1;
  } else  {
    serialPrintln();
    String result = F("DST:");
    result += boolToString(Settings.DST);
    return return_result(event, result);
  }
  return return_command_success();
}

String Command_DateTime(struct EventStruct *event, const char *Line)
{
  String TmpStr1;

  if (GetArgv(Line, TmpStr1, 2)) {
    struct tm tm;
    int yr, mnth, d;
    sscanf(TmpStr1.c_str(), "%4d-%2d-%2d", &yr, &mnth, &d);
    tm.tm_year = yr - 1970;
    tm.tm_mon  = mnth;
    tm.tm_mday = d;

    if (GetArgv(Line, TmpStr1, 3)) {
      int h, m, s;
      sscanf(TmpStr1.c_str(), "%2d:%2d:%2d", &h, &m, &s);
      tm.tm_hour = h;
      tm.tm_min  = m;
      tm.tm_sec  = s;
    } else {
      tm.tm_hour = 0;
      tm.tm_min  = 0;
      tm.tm_sec  = 0;
    }

    node_time.sysTime    = makeTime(tm);
    node_time.timeSource = Manual_set;
  } else  {
    // serialPrintln();
    String result = F("Datetime:");
    result += node_time.getDateTimeString('-', ':', ' ');
    return return_result(event, result);
  }
  return return_command_success();
}

#include "../Commands/Controller.h"


#include "../../ESPEasy_common.h"


#include "../Commands/Common.h"

#include "../DataStructs/ESPEasy_EventStruct.h"

#include "../DataTypes/ControllerIndex.h"

#include "../Globals/CPlugins.h"

#include "../Helpers/Misc.h"

//      controllerIndex = (event->Par1 - 1);   Par1 is here for 1 ... CONTROLLER_MAX
bool validControllerVar(struct EventStruct *event, controllerIndex_t& controllerIndex)
{
  if (event->Par1 <= 0) { return false; }
  controllerIndex = static_cast<controllerIndex_t>(event->Par1 - 1);
  return validControllerIndex(controllerIndex);
}

String Command_Controller_Disable(struct EventStruct *event, const char *Line)
{
  controllerIndex_t controllerIndex;

  if (validControllerVar(event, controllerIndex) && setControllerEnableStatus(controllerIndex, false)) {
    return return_command_success();
  }
  return return_command_failed();
}

String Command_Controller_Enable(struct EventStruct *event, const char *Line)
{
  controllerIndex_t controllerIndex;

  if (validControllerVar(event, controllerIndex) && setControllerEnableStatus(controllerIndex, true)) {
    return return_command_success();
  }
  return return_command_failed();
}

#include "../Commands/Notifications.h"

#include "../Commands/Common.h"

#include "../../ESPEasy_common.h"
#include "../DataTypes/ESPEasy_plugin_functions.h"
#include "../Globals/ESPEasy_Scheduler.h"
#include "../Globals/Settings.h"
#include "../Globals/NPlugins.h"
#include "../Helpers/StringConverter.h"


String Command_Notifications_Notify(struct EventStruct *event, const char* Line)
{
	String message = "";
	GetArgv(Line, message, 3);

	if (event->Par1 > 0) {
		int index = event->Par1 - 1;
		if (Settings.NotificationEnabled[index] && Settings.Notification[index] != 0) {
			nprotocolIndex_t NotificationProtocolIndex = getNProtocolIndex(Settings.Notification[index]);
			if (validNProtocolIndex(NotificationProtocolIndex )) {
				struct EventStruct TempEvent(event->TaskIndex);
				// TempEvent.NotificationProtocolIndex = NotificationProtocolIndex;
				TempEvent.NotificationIndex = index;
				TempEvent.String1 = message;
				Scheduler.schedule_notification_event_timer(NotificationProtocolIndex, NPlugin::Function::NPLUGIN_NOTIFY, &TempEvent);
			}
		}
	}
	return return_command_success();
}

#include "../Commands/Diagnostic.h"

/*
 #include "Common.h"
 #include "../../ESPEasy_common.h"
 
 #include "../DataStructs/ESPEasy_EventStruct.h"
 */

#include "../../ESPEasy_fdwdecl.h"

#include "../Commands/Common.h"

#include "../DataStructs/PortStatusStruct.h"

#include "../DataTypes/SettingsType.h"

#include "../ESPEasyCore/ESPEasy_Log.h"
#include "../ESPEasyCore/Serial.h"

#include "../Globals/Device.h"
#include "../Globals/ExtraTaskSettings.h"
#include "../Globals/GlobalMapPortStatus.h"
#include "../Globals/SecuritySettings.h"
#include "../Globals/Settings.h"
#include "../Globals/Statistics.h"

#include "../Helpers/Convert.h"
#include "../Helpers/ESPEasy_Storage.h"
#include "../Helpers/ESPEasy_time_calc.h"
#include "../Helpers/Misc.h"
#include "../Helpers/PortStatus.h"
#include "../Helpers/StringConverter.h"
#include "../Helpers/StringParser.h"

#include <map>
#include <stdint.h>


#ifndef BUILD_MINIMAL_OTA
bool showSettingsFileLayout = false;
#endif // ifndef BUILD_MINIMAL_OTA

#ifndef BUILD_NO_DIAGNOSTIC_COMMANDS
String Command_Lowmem(struct EventStruct *event, const char *Line)
{
  String result;

  result += lowestRAM;
  result += F(" : ");
  result += lowestRAMfunction;
  return return_result(event, result);
}

String Command_Malloc(struct EventStruct *event, const char *Line)
{
  char *ramtest;
  int size = parseCommandArgumentInt(Line, 1);

  ramtest = (char *)malloc(size);

  if (ramtest == nullptr) { return return_command_failed(); }
  free(ramtest);
  return return_command_success();
}

String Command_SysLoad(struct EventStruct *event, const char *Line)
{
  String result = toString(getCPUload(), 2);

  result += F("% (LC=");
  result += getLoopCountPerSec();
  result += ')';
  return return_result(event, result);
}

String Command_SerialFloat(struct EventStruct *event, const char *Line)
{
  pinMode(1, INPUT);
  pinMode(3, INPUT);
  delay(60000);
  return return_command_success();
}

String Command_MemInfo(struct EventStruct *event, const char *Line)
{
  serialPrint(F("SecurityStruct         | "));
  serialPrintln(String(sizeof(SecuritySettings)));
  serialPrint(F("SettingsStruct         | "));
  serialPrintln(String(sizeof(Settings)));
  serialPrint(F("ExtraTaskSettingsStruct| "));
  serialPrintln(String(sizeof(ExtraTaskSettings)));
  serialPrint(F("DeviceStruct           | "));
  serialPrintln(String(Device.size()));
  return return_see_serial(event);
}

String Command_MemInfo_detail(struct EventStruct *event, const char *Line)
{
#ifndef BUILD_MINIMAL_OTA
  showSettingsFileLayout = true;
  Command_MemInfo(event, Line);

  for (int st = 0; st < static_cast<int>(SettingsType::Enum::SettingsType_MAX); ++st) {
    SettingsType::SettingsType::Enum settingsType = static_cast<SettingsType::Enum>(st);
    int max_index, offset, max_size;
    int struct_size = 0;
    serialPrintln();
    serialPrint(SettingsType::getSettingsTypeString(settingsType));
    serialPrintln(F(" | start | end | max_size | struct_size"));
    serialPrintln(F("--- | --- | --- | --- | ---"));
    SettingsType::getSettingsParameters(settingsType, 0, max_index, offset, max_size, struct_size);

    for (int i = 0; i < max_index; ++i) {
      SettingsType::getSettingsParameters(settingsType, i, offset, max_size);
      serialPrint(String(i));
      serialPrint("|");
      serialPrint(String(offset));
      serialPrint("|");
      serialPrint(String(offset + max_size - 1));
      serialPrint("|");
      serialPrint(String(max_size));
      serialPrint("|");
      serialPrintln(String(struct_size));
    }
  }
  return return_see_serial(event);
  #else
  return return_command_failed();
  #endif // ifndef BUILD_MINIMAL_OTA
}

String Command_Background(struct EventStruct *event, const char *Line)
{
  unsigned long timer = millis() + parseCommandArgumentInt(Line, 1);

  serialPrintln(F("start"));

  while (!timeOutReached(timer)) {
    backgroundtasks();
  }
  serialPrintln(F("end"));
  return return_see_serial(event);
}
#endif // BUILD_NO_DIAGNOSTIC_COMMANDS

String Command_Debug(struct EventStruct *event, const char *Line)
{
  if (HasArgv(Line, 2)) {
    setLogLevelFor(LOG_TO_SERIAL, parseCommandArgumentInt(Line, 1));
  } else  {
    serialPrintln();
    serialPrint(F("Serial debug level: "));
    serialPrintln(String(Settings.SerialLogLevel));
  }
  return return_see_serial(event);
}

String Command_logentry(struct EventStruct *event, const char *Line)
{
  // FIXME TD-er: Add an extra optional parameter to set log level.
  addLog(LOG_LEVEL_INFO, tolerantParseStringKeepCase(Line, 2));
  return return_command_success();
}

#ifndef BUILD_NO_DIAGNOSTIC_COMMANDS
String Command_JSONPortStatus(struct EventStruct *event, const char *Line)
{
  addLog(LOG_LEVEL_INFO, F("JSON Port Status: Command not implemented yet."));
  return return_command_success();
}

void createLogPortStatus(std::map<uint32_t, portStatusStruct>::iterator it)
{  
  String log = F("PortStatus detail: ");

  log += F("Port=");
  log += getPortFromKey(it->first);
  log += F(" State=");
  log += it->second.state;
  log += F(" Output=");
  log += it->second.output;
  log += F(" Mode=");
  log += it->second.mode;
  log += F(" Task=");
  log += it->second.task;
  log += F(" Monitor=");
  log += it->second.monitor;
  log += F(" Command=");
  log += it->second.command;
  log += F(" Init=");
  log += it->second.init;
  log += F(" PreviousTask=");
  log += it->second.previousTask;
  addLog(LOG_LEVEL_INFO, log);
}

void debugPortStatus(std::map<uint32_t, portStatusStruct>::iterator it)
{
  createLogPortStatus(it);
}

void logPortStatus(const String& from) {
  String log;

  log  = F("PortStatus structure: Called from=");
  log += from;
  log += F(" Count=");
  log += globalMapPortStatus.size();
  addLog(LOG_LEVEL_INFO, log);

  for (std::map<uint32_t, portStatusStruct>::iterator it = globalMapPortStatus.begin(); it != globalMapPortStatus.end(); ++it) {
    debugPortStatus(it);
  }
}

String Command_logPortStatus(struct EventStruct *event, const char *Line)
{
  logPortStatus("Rules");
  return return_command_success();
}
#endif // BUILD_NO_DIAGNOSTIC_COMMANDS

#include "../Commands/Blynk.h"

#include "../../ESPEasy_fdwdecl.h"
#include "../Commands/Common.h"
#include "../DataStructs/ESPEasy_EventStruct.h"
#include "../ESPEasyCore/ESPEasy_Log.h"
#include "../Globals/Protocol.h"
#include "../Globals/Settings.h"
#include "../Helpers/_CPlugin_Helper.h"
#include "../Helpers/ESPEasy_Storage.h"
#include "../Helpers/ESPEasy_time_calc.h"


#ifdef USES_C012

controllerIndex_t firstEnabledBlynk_ControllerIndex() {
  for (controllerIndex_t i = 0; i < CONTROLLER_MAX; ++i) {
    protocolIndex_t ProtocolIndex = getProtocolIndex_from_ControllerIndex(i);

    if (validProtocolIndex(ProtocolIndex)) {
      if ((Protocol[ProtocolIndex].Number == 12) && Settings.ControllerEnabled[i]) {
        return i;
      }
    }
  }
  return INVALID_CONTROLLER_INDEX;
}

String Command_Blynk_Get(struct EventStruct *event, const char *Line)
{
  controllerIndex_t first_enabled_blynk_controller = firstEnabledBlynk_ControllerIndex();

  if (!validControllerIndex(first_enabled_blynk_controller)) {
    return F("Controller not enabled");
  } else {
    // FIXME TD-er: This one is not using parseString* function
    String strLine = Line;
    strLine = strLine.substring(9);
    int index = strLine.indexOf(',');

    if (index > 0)
    {
      int index           = strLine.lastIndexOf(',');
      String blynkcommand = strLine.substring(index + 1);
      float  value        = 0;

      if (Blynk_get(blynkcommand, first_enabled_blynk_controller, &value))
      {
        UserVar[(VARS_PER_TASK * (event->Par1 - 1)) + event->Par2 - 1] = value;
      }
      else {
        return F("Error getting data");
      }
    }
    else
    {
      if (!Blynk_get(strLine, first_enabled_blynk_controller, nullptr))
      {
        return F("Error getting data");
      }
    }
  }
  return return_command_success();
}

bool Blynk_get(const String& command, controllerIndex_t controllerIndex, float *data)
{
  bool MustCheckReply = false;
  String hostname, pass;
  unsigned int ClientTimeout = 0;
  WiFiClient client;

  {
    // Place ControllerSettings in its own scope, as it is quite big.
    MakeControllerSettings(ControllerSettings);
    if (!AllocatedControllerSettings()) {
      addLog(LOG_LEVEL_ERROR, F("Blynk : Cannot run GET, out of RAM"));
      return false;
    }

    LoadControllerSettings(controllerIndex, ControllerSettings);
    MustCheckReply = ControllerSettings.MustCheckReply;
    hostname = ControllerSettings.getHost();
    pass = getControllerPass(controllerIndex, ControllerSettings);
    ClientTimeout = ControllerSettings.ClientTimeout;

    if (pass.length() == 0) {
      addLog(LOG_LEVEL_ERROR, F("Blynk : No password set"));
      return false;
    }

    if (!try_connect_host(/* CPLUGIN_ID_012 */ 12, client, ControllerSettings)) {
      return false;
    }
  }

  // We now create a URI for the request
  {
    // Place this stack allocated array in its own scope, as it is quite big.
    char request[300] = { 0 };
    sprintf_P(request,
              PSTR("GET /%s/%s HTTP/1.1\r\n Host: %s \r\n Connection: close\r\n\r\n"),
              pass.c_str(),
              command.c_str(),
              hostname.c_str());
    addLog(LOG_LEVEL_DEBUG, request);
    client.print(request);
  }
  bool success = !MustCheckReply;

  if (MustCheckReply || data) {
    unsigned long timer = millis() + 200;

    while (!client_available(client) && !timeOutReached(timer)) {
      delay(1);
    }

    char log[80] = { 0 };
    timer = millis() + 1500;

    // Read all the lines of the reply from server and log them
    while (client_available(client) && !success && !timeOutReached(timer)) {
      String line;
      safeReadStringUntil(client, line, '\n');
      addLog(LOG_LEVEL_DEBUG_MORE, line);

      // success ?
      if (line.substring(0, 15) == F("HTTP/1.1 200 OK")) {
        strcpy_P(log, PSTR("HTTP : Success"));

        if (!data) { success = true; }
      }
      else if (line.substring(0, 24) == F("HTTP/1.1 400 Bad Request")) {
        strcpy_P(log, PSTR("HTTP : Unauthorized"));
      }
      else if (line.substring(0, 25) == F("HTTP/1.1 401 Unauthorized")) {
        strcpy_P(log, PSTR("HTTP : Unauthorized"));
      }
      addLog(LOG_LEVEL_DEBUG, log);

      // data only
      if (data && line.startsWith("["))
      {
        String strValue = line;
        byte   pos      = strValue.indexOf('"', 2);
        strValue = strValue.substring(2, pos);
        strValue.trim();
        float value = strValue.toFloat();
        *data   = value;
        success = true;

        char value_char[5] = { 0 };
        strValue.toCharArray(value_char, 5);
        sprintf_P(log, PSTR("Blynk get - %s => %s"), command.c_str(), value_char);
        addLog(LOG_LEVEL_DEBUG, log);
      }
      delay(0);
    }
  }
  addLog(LOG_LEVEL_DEBUG, F("HTTP : closing connection (012)"));

  client.flush();
  client.stop();

  // important - backgroundtasks - free mem
  unsigned long timer = millis() + ClientTimeout;

  while (!timeOutReached(timer)) {
    backgroundtasks();
  }

  return success;
}

#endif // ifdef USES_C012

#include "../Commands/HTTP.h"

#include "../../ESPEasy_common.h"

#include "../Commands/Common.h"

#include "../DataStructs/ControllerSettingsStruct.h"
#include "../DataStructs/SettingsStruct.h"

#include "../ESPEasyCore/ESPEasy_Log.h"
#include "../ESPEasyCore/ESPEasyNetwork.h"

#include "../Globals/Settings.h"

#include "../Helpers/_CPlugin_Helper.h"
#include "../Helpers/Misc.h"
#include "../Helpers/Networking.h"
#include "../Helpers/StringParser.h"


String Command_HTTP_SendToHTTP(struct EventStruct *event, const char* Line)
{
	if (NetworkConnected()) {
		String host = parseString(Line, 2);
		const int port = parseCommandArgumentInt(Line, 2);
		if (loglevelActiveFor(LOG_LEVEL_DEBUG)) {
			String log = F("SendToHTTP: Host: ");
			log += host;
			log += F(" port: ");
			log += port;
			addLog(LOG_LEVEL_DEBUG, log);
		}
		if (port < 0 || port > 65535) return return_command_failed();
		// FIXME TD-er: This is not using the tolerant settings option.
    // String path = tolerantParseStringKeepCase(Line, 4);
		String path = parseStringToEndKeepCase(Line, 4);
#ifndef BUILD_NO_DEBUG
		if (loglevelActiveFor(LOG_LEVEL_DEBUG)) {
			String log = F("SendToHTTP: Path: ");
			log += path;
			addLog(LOG_LEVEL_DEBUG, log);
		}
#endif
		WiFiClient client;
		client.setTimeout(CONTROLLER_CLIENTTIMEOUT_MAX);
		const bool connected = connectClient(client, host.c_str(), port);
		if (connected) {
			String hostportString = host;
			if (port != 0 && port != 80) {
				hostportString += ':';
				hostportString += port;
			}
			String request = do_create_http_request(hostportString, F("GET"), path);
#ifndef BUILD_NO_DEBUG
			addLog(LOG_LEVEL_DEBUG, request);
#endif
            bool mustCheckAck = Settings.SendToHttp_ack();
			send_via_http(F("Command_HTTP_SendToHTTP"), client, request, mustCheckAck);
			return return_command_success();
		}
		addLog(LOG_LEVEL_ERROR, F("SendToHTTP connection failed"));
	} else {
		addLog(LOG_LEVEL_ERROR, F("SendToHTTP Not connected to network"));
	}
	return return_command_failed();
}

#include "../Commands/WiFi.h"

#include "../../ESPEasy_common.h"

#include "../Commands/Common.h"

#include "../ESPEasyCore/ESPEasyWifi.h"
#include "../ESPEasyCore/Serial.h"

#include "../Globals/ESPEasyWiFiEvent.h"
#include "../Globals/Settings.h"
#include "../Globals/ESPEasyWiFiEvent.h"

#include "../Helpers/StringConverter.h"


#define WIFI_MODE_MAX (WiFiMode_t)4


String Command_Wifi_SSID(struct EventStruct *event, const char *Line)
{
  return Command_GetORSetString(event, F("Wifi SSID:"),
                                Line,
                                SecuritySettings.WifiSSID,
                                sizeof(SecuritySettings.WifiSSID),
                                1);
}

String Command_Wifi_Key(struct EventStruct *event, const char *Line)
{
  return Command_GetORSetString(event, F("Wifi Key:"),
                                Line,
                                SecuritySettings.WifiKey,
                                sizeof(SecuritySettings.WifiKey),
                                1);
}

String Command_Wifi_SSID2(struct EventStruct *event, const char *Line)
{
  return Command_GetORSetString(event, F("Wifi2 SSID:"),
                                Line,
                                SecuritySettings.WifiSSID2,
                                sizeof(SecuritySettings.WifiSSID2),
                                1);
}

String Command_Wifi_Key2(struct EventStruct *event, const char *Line)
{
  return Command_GetORSetString(event, F("Wifi2 Key:"),
                                Line,
                                SecuritySettings.WifiKey2,
                                sizeof(SecuritySettings.WifiKey2),
                                1);
}

String Command_Wifi_Scan(struct EventStruct *event, const char *Line)
{
  WifiScan();
  return return_command_success();
}

String Command_Wifi_Connect(struct EventStruct *event, const char *Line)
{
  WiFiEventData.wifiConnectAttemptNeeded = true;
  return return_command_success();
}

String Command_Wifi_Disconnect(struct EventStruct *event, const char *Line)
{
  WifiDisconnect();
  return return_command_success();
}

String Command_Wifi_APMode(struct EventStruct *event, const char *Line)
{
  setAP(true);
  return return_command_success();
}

String Command_Wifi_STAMode(struct EventStruct *event, const char *Line)
{
  setSTA(true);
  return return_command_success();
}

String Command_Wifi_Mode(struct EventStruct *event, const char *Line)
{
  String TmpStr1;

  if (GetArgv(Line, TmpStr1, 2)) {
    WiFiMode_t mode = WIFI_MODE_MAX;

    if (event->Par1 > 0 && event->Par1 < WIFI_MODE_MAX) {
      mode = static_cast<WiFiMode_t>(event->Par1 - 1);
    } else {
      TmpStr1.toLowerCase();

      if (strcmp_P(TmpStr1.c_str(), PSTR("off")) == 0) { mode = WIFI_OFF; }
      else if (strcmp_P(TmpStr1.c_str(), PSTR("sta")) == 0) { mode = WIFI_STA; }
      else if (strcmp_P(TmpStr1.c_str(), PSTR("ap")) == 0) { mode = WIFI_AP; }
      else if (strcmp_P(TmpStr1.c_str(), PSTR("ap+sta")) == 0) { mode = WIFI_AP_STA; }
    }

    if ((mode >= WIFI_OFF) && (mode < WIFI_MODE_MAX)) {
      setWifiMode(mode);
    } else {
      serialPrintln();
      return return_result(event, F("Wifi Mode: invalid arguments"));
    }
  } else {
    serialPrintln();
    String result = F("WiFi Mode:");
    result += getWifiModeString(WiFi.getMode());
    return return_result(event, result);
  }
  return return_command_success();
}

// FIXME: TD-er This is not an erase, but actually storing the current settings
// in the wifi settings of the core library
String Command_WiFi_Erase(struct EventStruct *event, const char *Line)
{
  WiFi.persistent(true);  // use SDK storage of SSID/WPA parameters
  WifiDisconnect();       // this will store empty ssid/wpa into sdk storage
  WiFi.persistent(false); // Do not use SDK storage of SSID/WPA parameters
  return return_command_success();
}

#include "../Commands/i2c.h"

#include "../Commands/Common.h"
#include "../ESPEasyCore/Serial.h"

#include "../Globals/I2Cdev.h"

#include "../../ESPEasy_common.h"

String Command_i2c_Scanner(struct EventStruct *event, const char* Line)
{
	byte error, address;
	for (address = 1; address <= 127; address++) {
		Wire.beginTransmission(address);
		error = Wire.endTransmission();
		if (error == 0) {
			serialPrint(F("I2C  : Found 0x"));
			serialPrintln(String(address, HEX));
		}else if (error == 4) {
			serialPrint(F("I2C  : Error at 0x"));
			serialPrintln(String(address, HEX));
		}
	}
	return return_see_serial(event);
}

