#include "../WebServer/404.h"

#include "../WebServer/WebServer.h"
#include "../WebServer/LoadFromFS.h"
#include "../WebServer/Rules.h"

#include "../Globals/Services.h"

#include "../Globals/ESPEasyWiFiEvent.h"

// ********************************************************************************
// Web Interface handle other requests
// ********************************************************************************
void handleNotFound() {
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("handleNotFound"));
  #endif

  if (WiFiEventData.wifiSetup)
  {
    web_server.send(200, F("text/html"), F("<meta HTTP-EQUIV='REFRESH' content='0; url=/setup'>"));
    return;
  }

  if (!isLoggedIn()) { return; }

#ifdef WEBSERVER_RULES
  if (handle_rules_edit(web_server.uri())) { return; }
#endif

  if (loadFromFS(true, web_server.uri())) { return; }

  if (loadFromFS(false, web_server.uri())) { return; }
  String message = F("URI: ");
  message += web_server.uri();
  message += F("\nMethod: ");
  message += (web_server.method() == HTTP_GET) ? F("GET") : F("POST");
  message += F("\nArguments: ");
  message += web_server.args();
  message += "\n";

  for (uint8_t i = 0; i < web_server.args(); i++) {
    message += F(" NAME:");
    message += web_server.argName(i);
    message += F("\n VALUE:");
    message += web_server.arg(i);
    message += '\n';
  }
  web_server.send(404, F("text/plain"), message);
}

#include "../WebServer/AccessControl.h"

#include "../ESPEasyCore/ESPEasy_Log.h"
#include "../ESPEasyCore/ESPEasyNetwork.h"
#include "../ESPEasyCore/ESPEasyWifi.h"

#include "../Globals/SecuritySettings.h"
#include "../Globals/Services.h"

#include "../Helpers/Networking.h"
#include "../Helpers/StringConverter.h"

#include "../WebServer/Markup.h"

// ********************************************************************************
// Allowed IP range check
// ********************************************************************************

boolean ipLessEqual(const IPAddress& ip, const IPAddress& high)
{
  unsigned long u_ip = ((unsigned long )ip[0] << 24) | ((unsigned long )ip[1] << 16) | ((unsigned long )ip[2] << 8) | ip[3];
  unsigned long u_high = ((unsigned long )high[0] << 24) | ((unsigned long )high[1] << 16) | ((unsigned long )high[2] << 8) | high[3];
  return u_ip <= u_high;
}

boolean ipInRange(const IPAddress& ip, const IPAddress& low, const IPAddress& high)
{
  return ipLessEqual(low, ip) && ipLessEqual(ip, high);
}

String describeAllowedIPrange() {
  String reply;

  switch (SecuritySettings.IPblockLevel) {
    case ALL_ALLOWED:
      reply +=  F("All Allowed");
      break;
    default:
    {
      IPAddress low, high;
      getIPallowedRange(low, high);
      reply +=  formatIP(low);
      reply +=  F(" - ");
      reply +=  formatIP(high);
    }
  }
  return reply;
}

bool getIPallowedRange(IPAddress& low, IPAddress& high)
{
  switch (SecuritySettings.IPblockLevel) {
    case LOCAL_SUBNET_ALLOWED:

      if (WifiIsAP(WiFi.getMode())) {
        // WiFi is active as accesspoint, do not check.
        return false;
      }
      return getSubnetRange(low, high);
    case ONLY_IP_RANGE_ALLOWED:
      low  = SecuritySettings.AllowedIPrangeLow;
      high = SecuritySettings.AllowedIPrangeHigh;
      break;
    default:
      low  = IPAddress(0, 0, 0, 0);
      high = IPAddress(255, 255, 255, 255);
      return false;
  }
  return true;
}

bool clientIPinSubnet() {
  IPAddress low, high;

  if (!getSubnetRange(low, high)) {
    // Could not determine subnet.
    return false;
  }
  WiFiClient client(web_server.client());
  return ipInRange(client.remoteIP(), low, high);
}

boolean clientIPallowed()
{
  // TD-er Must implement "safe time after boot"
  IPAddress low, high;

  if (!getIPallowedRange(low, high))
  {
    // No subnet range determined, cannot filter on IP
    return true;
  }
  WiFiClient client(web_server.client());

  if (ipInRange(client.remoteIP(), low, high)) {
    return true;
  }

  if (WifiIsAP(WiFi.getMode())) {
    // @TD-er Fixme: Should match subnet of SoftAP.
    return true;
  }
  String response = F("IP blocked: ");
  response += formatIP(client.remoteIP());
  web_server.send(403, F("text/html"), response);

  if (loglevelActiveFor(LOG_LEVEL_ERROR)) {
    response += F(" Allowed: ");
    response += formatIP(low);
    response += F(" - ");
    response += formatIP(high);
    addLog(LOG_LEVEL_ERROR, response);
  }
  return false;
}

void clearAccessBlock()
{
  SecuritySettings.IPblockLevel = ALL_ALLOWED;
}

// ********************************************************************************
// Add a IP Access Control select dropdown list
// ********************************************************************************
void addIPaccessControlSelect(const String& name, int choice)
{
  String options[3] = { F("Allow All"), F("Allow Local Subnet"), F("Allow IP range") };

  addSelector(name, 3, options, NULL, NULL, choice);
}

#include "../WebServer/AdvancedConfigPage.h"

#include "../WebServer/HTML_wrappers.h"
#include "../WebServer/Markup.h"
#include "../WebServer/Markup_Buttons.h"
#include "../WebServer/Markup_Forms.h"
#include "../WebServer/WebServer.h"

#include "../Globals/ESPEasy_time.h"
#include "../Globals/Settings.h"
#include "../Globals/TimeZone.h"

#include "../Helpers/ESPEasy_Storage.h"
#include "../Helpers/StringConverter.h"


#ifdef WEBSERVER_ADVANCED


// ********************************************************************************
// Web Interface config page
// ********************************************************************************
void handle_advanced() {
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("handle_advanced"));
  #endif

  if (!isLoggedIn()) { return; }
  navMenuIndex = MENU_INDEX_TOOLS;
  TXBuffer.startStream();
  sendHeadandTail_stdtemplate();

  String edit       = web_server.arg(F("edit"));

  if (edit.length() != 0)
  {
//    Settings.MessageDelay_unused = getFormItemInt(F("messagedelay"));
    Settings.IP_Octet     = web_server.arg(F("ip")).toInt();
    strncpy_webserver_arg(Settings.NTPHost, F("ntphost"));
    Settings.TimeZone = getFormItemInt(F("timezone"));
    TimeChangeRule dst_start(getFormItemInt(F("dststartweek")), getFormItemInt(F("dststartdow")), getFormItemInt(F("dststartmonth")), getFormItemInt(F("dststarthour")), Settings.TimeZone);

    if (dst_start.isValid()) { Settings.DST_Start = dst_start.toFlashStoredValue(); }
    TimeChangeRule dst_end(getFormItemInt(F("dstendweek")), getFormItemInt(F("dstenddow")), getFormItemInt(F("dstendmonth")), getFormItemInt(F("dstendhour")), Settings.TimeZone);

    if (dst_end.isValid()) { Settings.DST_End = dst_end.toFlashStoredValue(); }
    webArg2ip(F("syslogip"), Settings.Syslog_IP);
    Settings.WebserverPort = getFormItemInt(F("webport"));
    Settings.UDPPort = getFormItemInt(F("udpport"));

    Settings.SyslogFacility = getFormItemInt(F("syslogfacility"));
    Settings.SyslogPort     = getFormItemInt(F("syslogport"));
    Settings.UseSerial      = isFormItemChecked(F("useserial"));
    setLogLevelFor(LOG_TO_SYSLOG, getFormItemInt(getInternalLabel(LabelType::SYSLOG_LOG_LEVEL)));
    setLogLevelFor(LOG_TO_SERIAL, getFormItemInt(getInternalLabel(LabelType::SERIAL_LOG_LEVEL)));
    setLogLevelFor(LOG_TO_WEBLOG, getFormItemInt(getInternalLabel(LabelType::WEB_LOG_LEVEL)));
#ifdef FEATURE_SD
    setLogLevelFor(LOG_TO_SDCARD, getFormItemInt(getInternalLabel(LabelType::SD_LOG_LEVEL)));
#endif // ifdef FEATURE_SD
    Settings.UseValueLogger              = isFormItemChecked(F("valuelogger"));
    Settings.BaudRate                    = getFormItemInt(F("baudrate"));
    Settings.UseNTP                      = isFormItemChecked(F("usentp"));
    Settings.DST                         = isFormItemChecked(F("dst"));
    Settings.WDI2CAddress                = getFormItemInt(F("wdi2caddress"));
    #ifdef USES_SSDP
    Settings.UseSSDP                     = isFormItemChecked(F("usessdp"));
    #endif // USES_SSDP
    Settings.WireClockStretchLimit       = getFormItemInt(F("wireclockstretchlimit"));
    Settings.UseRules                    = isFormItemChecked(F("userules"));
    Settings.ConnectionFailuresThreshold = getFormItemInt(F("cft"));
    Settings.ArduinoOTAEnable            = isFormItemChecked(F("arduinootaenable"));
    Settings.UseRTOSMultitasking         = isFormItemChecked(F("usertosmultitasking"));

    // MQTT settings now moved to the controller settings.
//    Settings.MQTTRetainFlag_unused              = isFormItemChecked(F("mqttretainflag"));
//    Settings.MQTTUseUnitNameAsClientId   = isFormItemChecked(F("mqttuseunitnameasclientid"));
//    Settings.uniqueMQTTclientIdReconnect(isFormItemChecked(F("uniquemqttclientidreconnect")));
    Settings.Latitude  = getFormItemFloat(F("latitude"));
    Settings.Longitude = getFormItemFloat(F("longitude"));
    #ifdef WEBSERVER_NEW_RULES
    Settings.OldRulesEngine(isFormItemChecked(F("oldrulesengine")));
    #endif // WEBSERVER_NEW_RULES
    Settings.TolerantLastArgParse(isFormItemChecked(F("tolerantargparse")));
    Settings.SendToHttp_ack(isFormItemChecked(F("sendtohttp_ack")));
    Settings.ForceWiFi_bg_mode(isFormItemChecked(getInternalLabel(LabelType::FORCE_WIFI_BG)));
    Settings.WiFiRestart_connection_lost(isFormItemChecked(getInternalLabel(LabelType::RESTART_WIFI_LOST_CONN)));
    Settings.EcoPowerMode(isFormItemChecked(getInternalLabel(LabelType::CPU_ECO_MODE)));
    Settings.WifiNoneSleep(isFormItemChecked(getInternalLabel(LabelType::FORCE_WIFI_NOSLEEP)));
#ifdef SUPPORT_ARP
    Settings.gratuitousARP(isFormItemChecked(getInternalLabel(LabelType::PERIODICAL_GRAT_ARP)));
#endif // ifdef SUPPORT_ARP

    addHtmlError(SaveSettings());

    if (node_time.systemTimePresent()) {
      node_time.initTime();
    }
  }

  addHtml(F("<form  method='post'>"));
  html_table_class_normal();

  addFormHeader(F("Advanced Settings"), F("RTDTools/Tools.html#advanced"));

  addFormSubHeader(F("Rules Settings"));

  addFormCheckBox(F("Rules"),      F("userules"),       Settings.UseRules);
  #ifdef WEBSERVER_NEW_RULES
  addFormCheckBox(F("Old Engine"), F("oldrulesengine"), Settings.OldRulesEngine());
  #endif // WEBSERVER_NEW_RULES
  addFormCheckBox(F("Tolerant last parameter"), F("tolerantargparse"), Settings.TolerantLastArgParse());
  addFormNote(F("Perform less strict parsing on last argument of some commands (e.g. publish and sendToHttp)"));
  addFormCheckBox(F("SendToHTTP wait for ack"), F("sendtohttp_ack"), Settings.SendToHttp_ack());

  /*
  // MQTT settings now moved to the controller settings.
  addFormSubHeader(F("Controller Settings"));

  addFormNumericBox(F("Message Interval"), F("messagedelay"), Settings.MessageDelay_unused, 0, INT_MAX);
  addUnit(F("ms"));

  addFormCheckBox(F("MQTT Retain Msg"), F("mqttretainflag"), Settings.MQTTRetainFlag_unused);
  addFormCheckBox(F("MQTT use unit name as ClientId"),    F("mqttuseunitnameasclientid"),   Settings.MQTTUseUnitNameAsClientId);
  addFormCheckBox(F("MQTT change ClientId at reconnect"), F("uniquemqttclientidreconnect"), Settings.uniqueMQTTclientIdReconnect_unused());
*/

  addFormSubHeader(F("NTP Settings"));

  addFormCheckBox(F("Use NTP"), F("usentp"), Settings.UseNTP);
  addFormTextBox(F("NTP Hostname"), F("ntphost"), Settings.NTPHost, 63);

  addFormSubHeader(F("DST Settings"));
  addFormDstSelect(true,  Settings.DST_Start);
  addFormDstSelect(false, Settings.DST_End);
  addFormCheckBox(F("DST"), F("dst"), Settings.DST);

  addFormSubHeader(F("Location Settings"));
  addFormNumericBox(F("Timezone Offset (UTC +)"), F("timezone"), Settings.TimeZone, -720, 840); // UTC-12H ... UTC+14h
  addUnit(F("minutes"));
  addFormFloatNumberBox(F("Latitude"), F("latitude"), Settings.Latitude, -90.0f, 90.0f);
  addUnit(F("&deg;"));
  addFormFloatNumberBox(F("Longitude"), F("longitude"), Settings.Longitude, -180.0f, 180.0f);
  addUnit(F("&deg;"));
  addFormNote(F("Longitude and Latitude are used to compute sunrise and sunset"));

  addFormSubHeader(F("Log Settings"));

  addFormIPBox(F("Syslog IP"), F("syslogip"), Settings.Syslog_IP);
  addFormNumericBox(F("Syslog UDP port"), F("syslogport"), Settings.SyslogPort, 0, 65535);

  addFormLogLevelSelect(getLabel(LabelType::SYSLOG_LOG_LEVEL), getInternalLabel(LabelType::SYSLOG_LOG_LEVEL), Settings.SyslogLevel);
  addFormLogFacilitySelect(F("Syslog Facility"), F("syslogfacility"), Settings.SyslogFacility);
  addFormLogLevelSelect(getLabel(LabelType::SERIAL_LOG_LEVEL), getInternalLabel(LabelType::SERIAL_LOG_LEVEL), Settings.SerialLogLevel);
  addFormLogLevelSelect(getLabel(LabelType::WEB_LOG_LEVEL),    getInternalLabel(LabelType::WEB_LOG_LEVEL),    Settings.WebLogLevel);

#ifdef FEATURE_SD
  addFormLogLevelSelect(getLabel(LabelType::SD_LOG_LEVEL),     getInternalLabel(LabelType::SD_LOG_LEVEL),     Settings.SDLogLevel);

  addFormCheckBox(F("SD Card Value Logger"), F("valuelogger"), Settings.UseValueLogger);
#endif // ifdef FEATURE_SD


  addFormSubHeader(F("Serial Settings"));

  addFormCheckBox(F("Enable Serial port"), F("useserial"), Settings.UseSerial);
  addFormNumericBox(F("Baud Rate"), F("baudrate"), Settings.BaudRate, 0, 1000000);


  addFormSubHeader(F("Inter-ESPEasy Network"));

  addFormNumericBox(F("UDP port"), F("udpport"), Settings.UDPPort, 0, 65535);

  // TODO sort settings in groups or move to other pages/groups
  addFormSubHeader(F("Special and Experimental Settings"));

  addFormNumericBox(F("Webserver port"), F("webport"), Settings.WebserverPort, 0, 65535);
  addFormNote(F("Requires reboot to activate"));

  addFormNumericBox(F("Fixed IP Octet"), F("ip"),           Settings.IP_Octet,     0, 255);

  addFormNumericBox(F("WD I2C Address"), F("wdi2caddress"), Settings.WDI2CAddress, 0, 127);
  addHtml(F(" (decimal)"));

  addFormNumericBox(F("I2C ClockStretchLimit"), F("wireclockstretchlimit"), Settings.WireClockStretchLimit); // TODO define limits
  #if defined(FEATURE_ARDUINO_OTA)
  addFormCheckBox(F("Enable Arduino OTA"), F("arduinootaenable"), Settings.ArduinoOTAEnable);
  #endif // if defined(FEATURE_ARDUINO_OTA)
  #if defined(ESP32)
  addFormCheckBox_disabled(F("Enable RTOS Multitasking"), F("usertosmultitasking"), Settings.UseRTOSMultitasking);
  #endif // if defined(ESP32)

  #ifdef USES_SSDP
  addFormCheckBox_disabled(F("Use SSDP"), F("usessdp"), Settings.UseSSDP);
  #endif // ifdef USES_SSDP

  addFormNumericBox(getLabel(LabelType::CONNECTION_FAIL_THRESH), F("cft"), Settings.ConnectionFailuresThreshold, 0, 100);
#ifdef ESP8266
  addFormCheckBox(LabelType::FORCE_WIFI_BG, Settings.ForceWiFi_bg_mode());
#endif // ifdef ESP8266
#ifdef ESP32

  // Disabled for now, since it is not working properly.
  addFormCheckBox_disabled(LabelType::FORCE_WIFI_BG, Settings.ForceWiFi_bg_mode());
#endif // ifdef ESP32

  addFormCheckBox(LabelType::RESTART_WIFI_LOST_CONN, Settings.WiFiRestart_connection_lost());
#ifdef ESP8266
  addFormCheckBox(LabelType::FORCE_WIFI_NOSLEEP,     Settings.WifiNoneSleep());
#endif // ifdef ESP8266
  addFormNote(F("Change WiFi sleep settings requires reboot to activate"));
#ifdef SUPPORT_ARP
  addFormCheckBox(LabelType::PERIODICAL_GRAT_ARP, Settings.gratuitousARP());
#endif // ifdef SUPPORT_ARP
  addFormCheckBox(LabelType::CPU_ECO_MODE,        Settings.EcoPowerMode());
  addFormNote(F("Node may miss receiving packets with Eco mode enabled"));
  addFormSeparator(2);

  html_TR_TD();
  html_TD();
  addSubmitButton();
  addHtml(F("<input type='hidden' name='edit' value='1'>"));
  html_end_table();
  html_end_form();
  sendHeadandTail_stdtemplate(true);
  TXBuffer.endStream();
}

void addFormDstSelect(bool isStart, uint16_t choice) {
  uint16_t tmpstart(choice);
  uint16_t tmpend(choice);

  if (!TimeChangeRule(choice, 0).isValid()) {
    time_zone.getDefaultDst_flash_values(tmpstart, tmpend);
  }
  TimeChangeRule rule(isStart ? tmpstart : tmpend, 0);
  {
    String weeklabel = isStart ? F("Start (week, dow, month)")  : F("End (week, dow, month)");
    String weekid  = isStart ? F("dststartweek")  : F("dstendweek");
    String week[5]       = { F("Last"), F("1st"), F("2nd"), F("3rd"), F("4th") };
    int    weekValues[5] = { 0, 1, 2, 3, 4 };

    addRowLabel(weeklabel);
    addSelector(weekid, 5, week, weekValues, NULL, rule.week);
  }
  html_BR();
  {
    String dowid   = isStart ? F("dststartdow")   : F("dstenddow");
    String dow[7]        = { F("Sun"), F("Mon"), F("Tue"), F("Wed"), F("Thu"), F("Fri"), F("Sat") };
    int    dowValues[7]  = { 1, 2, 3, 4, 5, 6, 7 };

    addSelector(dowid, 7, dow, dowValues, NULL, rule.dow);
  }
  html_BR();
  {
    String monthid = isStart ? F("dststartmonth") : F("dstendmonth");
    String month[12]     = { F("Jan"), F("Feb"), F("Mar"), F("Apr"), F("May"), F("Jun"), F("Jul"), F("Aug"), F("Sep"), F("Oct"), F("Nov"), F(
                             "Dec") };
    int    monthValues[12] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12 };

    addSelector(monthid, 12, month, monthValues, NULL, rule.month);
  }
  {
    String hourid  = isStart ? F("dststarthour")  : F("dstendhour");
    String hourlabel = isStart ? F("Start (localtime, e.g. 2h&rarr;3h)")  : F("End (localtime, e.g. 3h&rarr;2h)");

    addFormNumericBox(hourlabel, hourid, rule.hour, 0, 23);
    addUnit(isStart ? F("hour &#x21b7;") : F("hour &#x21b6;"));
  }
}

void addFormLogLevelSelect(const String& label, const String& id, int choice)
{
  addRowLabel(label);
  addLogLevelSelect(id, choice);
}

void addLogLevelSelect(const String& name, int choice)
{
  String options[LOG_LEVEL_NRELEMENTS + 1];
  int    optionValues[LOG_LEVEL_NRELEMENTS + 1] = { 0 };

  options[0]      = getLogLevelDisplayString(0);

  for (int i = 0; i < LOG_LEVEL_NRELEMENTS; ++i) {
    options[i + 1] = getLogLevelDisplayStringFromIndex(i, optionValues[i + 1]);
  }
  addSelector(name, LOG_LEVEL_NRELEMENTS + 1, options, optionValues, NULL, choice);
}

void addFormLogFacilitySelect(const String& label, const String& id, int choice)
{
  addRowLabel(label);
  addLogFacilitySelect(id, choice);
}

void addLogFacilitySelect(const String& name, int choice)
{
  String options[12] =
  { F("Kernel"), F("User"),   F("Daemon"),   F("Message"), F("Local0"),  F("Local1"),
    F("Local2"), F("Local3"), F("Local4"),   F("Local5"),  F("Local6"),  F("Local7") };
  int optionValues[12] = { 0, 1, 3, 5, 16, 17, 18, 19, 20, 21, 22, 23 };

  addSelector(name, 12, options, optionValues, NULL, choice);
}

#endif // ifdef WEBSERVER_ADVANCED

#include "../WebServer/CacheControllerPages.h"

#ifdef USES_C016

#include "../WebServer/WebServer.h"
#include "../WebServer/AccessControl.h"
#include "../WebServer/HTML_wrappers.h"
#include "../WebServer/JSON.h"
#include "../CustomBuild/ESPEasyLimits.h"
#include "../DataStructs/DeviceStruct.h"
#include "../DataTypes/TaskIndex.h"
#include "../Globals/C016_ControllerCache.h"
#include "../Globals/ExtraTaskSettings.h"
#include "../Helpers/ESPEasy_Storage.h"


// ********************************************************************************
// URLs needed for C016_CacheController
// to help dump the content of the binary log files
// ********************************************************************************
void handle_dumpcache() {
  if (!isLoggedIn()) { return; }

  C016_startCSVdump();
  unsigned long timestamp;
  byte  controller_idx;
  byte  TaskIndex;
  Sensor_VType  sensorType;
  byte  valueCount;
  float val1;
  float val2;
  float val3;
  float val4;

  TXBuffer.startStream();
  addHtml(F("UNIX timestamp;contr. idx;sensortype;taskindex;value count"));

  for (taskIndex_t i = 0; i < TASKS_MAX; ++i) {
    LoadTaskSettings(i);

    for (int j = 0; j < VARS_PER_TASK; ++j) {
      addHtml(";");
      addHtml(ExtraTaskSettings.TaskDeviceName);
      addHtml("#");
      addHtml(ExtraTaskSettings.TaskDeviceValueNames[j]);
    }
  }
  html_BR();
  float csv_values[VARS_PER_TASK * TASKS_MAX];

  for (int i = 0; i < VARS_PER_TASK * TASKS_MAX; ++i) {
    csv_values[i] = 0.0f;
  }

  while (C016_getCSVline(timestamp, controller_idx, TaskIndex, sensorType,
                         valueCount, val1, val2, val3, val4)) {
    {
      String html;
      html.reserve(64);
      html += timestamp;
      html += ';';
      html += controller_idx;
      html += ';';
      html += static_cast<byte>(sensorType);
      html += ';';
      html += TaskIndex;
      html += ';';
      html += valueCount;
      addHtml(html);
    }
    int valindex = TaskIndex * VARS_PER_TASK;
    csv_values[valindex++] = val1;
    csv_values[valindex++] = val2;
    csv_values[valindex++] = val3;
    csv_values[valindex++] = val4;

    for (int i = 0; i < VARS_PER_TASK * TASKS_MAX; ++i) {
      String html;
      html.reserve(12);
      html += ';';

      if (csv_values[i] == 0.0f) {
        html += '0';
      } else {
        html += String(csv_values[i], 6);
      }
      addHtml(html);
    }
    html_BR();
    delay(0);
  }
  TXBuffer.endStream();
}

void handle_cache_json() {
  if (!isLoggedIn()) { return; }

  TXBuffer.startJsonStream();
  addHtml(F("{\"columns\": ["));

  //     addHtml(F("UNIX timestamp;contr. idx;sensortype;taskindex;value count"));
  stream_to_json_value(F("UNIX timestamp"));
  addHtml(",");
  stream_to_json_value(F("UTC timestamp"));
  addHtml(",");
  stream_to_json_value(F("task index"));

  for (taskIndex_t i = 0; i < TASKS_MAX; ++i) {
    LoadTaskSettings(i);

    for (int j = 0; j < VARS_PER_TASK; ++j) {
      String label = ExtraTaskSettings.TaskDeviceName;
      label += '#';
      label += ExtraTaskSettings.TaskDeviceValueNames[j];
      addHtml(",");
      stream_to_json_value(label);
    }
  }
  addHtml(F("],\n"));
  C016_startCSVdump();
  addHtml(F("\"files\": ["));
  bool islast = false;
  int  filenr = 0;

  while (!islast) {
    String currentFile = C016_getCacheFileName(islast);

    if (currentFile.length() > 0) {
      if (filenr != 0) {
        addHtml(",");
      }
      stream_to_json_value(currentFile);
      ++filenr;
    }
  }
  addHtml(F("],\n"));
  stream_last_json_object_value(F("nrfiles"), String(filenr));
  addHtml(F("\n"));
  TXBuffer.endStream();
}

void handle_cache_csv() {
  if (!isLoggedIn()) { return; }
}

#endif // ifdef USES_C016

#include "../WebServer/ConfigPage.h"

#ifdef WEBSERVER_CONFIG

#include "../WebServer/HTML_wrappers.h"
#include "../WebServer/AccessControl.h"
#include "../WebServer/Markup.h"
#include "../WebServer/Markup_Buttons.h"
#include "../WebServer/Markup_Forms.h"
#include "../WebServer/WebServer.h"

#include "../ESPEasyCore/Controller.h"

#include "../Globals/MQTT.h"
#include "../Globals/SecuritySettings.h"
#include "../Globals/Settings.h"

#include "../Helpers/DeepSleep.h"
#include "../Helpers/ESPEasy_Storage.h"
#include "../Helpers/Networking.h"
#include "../Helpers/StringConverter.h"



// ********************************************************************************
// Web Interface config page
// ********************************************************************************
void handle_config() {
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("handle_config"));
  #endif

  if (!isLoggedIn()) { return; }

  navMenuIndex = MENU_INDEX_CONFIG;
  TXBuffer.startStream();
  sendHeadandTail_stdtemplate(_HEAD);

  if (web_server.args() != 0)
  {
    String name = web_server.arg(F("name"));
    name.trim();

    Settings.Delay              = getFormItemInt(F("delay"), Settings.Delay);
    Settings.deepSleep_wakeTime = getFormItemInt(F("awaketime"), Settings.deepSleep_wakeTime);
    Settings.Unit = getFormItemInt(F("unit"), Settings.Unit);

    // String apkey = web_server.arg(F("apkey"));
    String ssid = web_server.arg(F("ssid"));

    if (strcmp(Settings.Name, name.c_str()) != 0) {
      addLog(LOG_LEVEL_INFO, F("Unit Name changed."));

      if (CPluginCall(CPlugin::Function::CPLUGIN_GOT_INVALID, 0)) { // inform controllers that the old name will be invalid from now on.
#ifdef USES_MQTT
        MQTTDisconnect();                                           // disconnect form MQTT Server if invalid message was sent succesfull.
#endif // USES_MQTT
      }
#ifdef USES_MQTT
      MQTTclient_should_reconnect = true;
#endif // USES_MQTT
    }

    // Unit name
    safe_strncpy(Settings.Name, name.c_str(), sizeof(Settings.Name));
    Settings.appendUnitToHostname(isFormItemChecked(F("appendunittohostname")));

    // Password
    copyFormPassword(F("password"), SecuritySettings.Password, sizeof(SecuritySettings.Password));

    // SSID 1
    safe_strncpy(SecuritySettings.WifiSSID, ssid.c_str(), sizeof(SecuritySettings.WifiSSID));
    copyFormPassword(F("key"), SecuritySettings.WifiKey, sizeof(SecuritySettings.WifiKey));

    // SSID 2
    strncpy_webserver_arg(SecuritySettings.WifiSSID2, F("ssid2"));
    copyFormPassword(F("key2"),  SecuritySettings.WifiKey2,  sizeof(SecuritySettings.WifiKey2));

    // Access point password.
    copyFormPassword(F("apkey"), SecuritySettings.WifiAPKey, sizeof(SecuritySettings.WifiAPKey));


    // TD-er Read access control from form.
    SecuritySettings.IPblockLevel = getFormItemInt(F("ipblocklevel"));

    switch (SecuritySettings.IPblockLevel) {
      case LOCAL_SUBNET_ALLOWED:
      {
        IPAddress low, high;
        getSubnetRange(low, high);

        for (byte i = 0; i < 4; ++i) {
          SecuritySettings.AllowedIPrangeLow[i]  = low[i];
          SecuritySettings.AllowedIPrangeHigh[i] = high[i];
        }
        break;
      }
      case ONLY_IP_RANGE_ALLOWED:
      case ALL_ALLOWED:

        webArg2ip(F("iprangelow"),  SecuritySettings.AllowedIPrangeLow);
        webArg2ip(F("iprangehigh"), SecuritySettings.AllowedIPrangeHigh);
        break;
    }

    Settings.deepSleepOnFail = isFormItemChecked(F("deepsleeponfail"));
    webArg2ip(F("espip"),      Settings.IP);
    webArg2ip(F("espgateway"), Settings.Gateway);
    webArg2ip(F("espsubnet"),  Settings.Subnet);
    webArg2ip(F("espdns"),     Settings.DNS);
#ifdef HAS_ETHERNET
    webArg2ip(F("espethip"),      Settings.ETH_IP);
    webArg2ip(F("espethgateway"), Settings.ETH_Gateway);
    webArg2ip(F("espethsubnet"),  Settings.ETH_Subnet);
    webArg2ip(F("espethdns"),     Settings.ETH_DNS);
#endif
    addHtmlError(SaveSettings());
  }

  html_add_form();
  html_table_class_normal();

  addFormHeader(F("Main Settings"));

  Settings.Name[25]             = 0;
  SecuritySettings.Password[25] = 0;
  addFormTextBox(F("Unit Name"), F("name"), Settings.Name, 25);
  addFormNumericBox(F("Unit Number"), F("unit"), Settings.Unit, 0, UNIT_NUMBER_MAX);
  addFormCheckBox(F("Append Unit Number to hostname"), F("appendunittohostname"), Settings.appendUnitToHostname());
  addFormPasswordBox(F("Admin Password"), F("password"), SecuritySettings.Password, 25);

  addFormSubHeader(F("Wifi Settings"));

  addFormTextBox(getLabel(LabelType::SSID), F("ssid"), SecuritySettings.WifiSSID, 31);
  addFormPasswordBox(F("WPA Key"), F("key"), SecuritySettings.WifiKey, 63);
  addFormTextBox(F("Fallback SSID"), F("ssid2"), SecuritySettings.WifiSSID2, 31);
  addFormPasswordBox(F("Fallback WPA Key"), F("key2"), SecuritySettings.WifiKey2, 63);
  addFormSeparator(2);
  addFormPasswordBox(F("WPA AP Mode Key"), F("apkey"), SecuritySettings.WifiAPKey, 63);

  // TD-er add IP access box F("ipblocklevel")
  addFormSubHeader(F("Client IP filtering"));
  {
    IPAddress low, high;
    getIPallowedRange(low, high);
    byte iplow[4];
    byte iphigh[4];

    for (byte i = 0; i < 4; ++i) {
      iplow[i]  = low[i];
      iphigh[i] = high[i];
    }
    addFormIPaccessControlSelect(F("Client IP block level"), F("ipblocklevel"), SecuritySettings.IPblockLevel);
    addFormIPBox(F("Access IP lower range"), F("iprangelow"),  iplow);
    addFormIPBox(F("Access IP upper range"), F("iprangehigh"), iphigh);
  }

  addFormSubHeader(F("WiFi IP Settings"));

  addFormIPBox(F("ESP WiFi IP"),         F("espip"),      Settings.IP);
  addFormIPBox(F("ESP WiFi Gateway"),    F("espgateway"), Settings.Gateway);
  addFormIPBox(F("ESP WiFi Subnetmask"), F("espsubnet"),  Settings.Subnet);
  addFormIPBox(F("ESP WiFi DNS"),        F("espdns"),     Settings.DNS);
  addFormNote(F("Leave empty for DHCP"));

#ifdef HAS_ETHERNET
  addFormSubHeader(F("Ethernet IP Settings"));

  addFormIPBox(F("ESP Ethernet IP"),         F("espethip"),      Settings.ETH_IP);
  addFormIPBox(F("ESP Ethernet Gateway"),    F("espethgateway"), Settings.ETH_Gateway);
  addFormIPBox(F("ESP Ethernet Subnetmask"), F("espethsubnet"),  Settings.ETH_Subnet);
  addFormIPBox(F("ESP Ethernet DNS"),        F("espethdns"),     Settings.ETH_DNS);
  addFormNote(F("Leave empty for DHCP"));
#endif


  addFormSubHeader(F("Sleep Mode"));

  addFormNumericBox(F("Sleep awake time"), F("awaketime"), Settings.deepSleep_wakeTime, 0, 255);
  addUnit(F("sec"));
  addHelpButton(F("SleepMode"));
  addFormNote(F("0 = Sleep Disabled, else time awake from sleep"));

  int dsmax = getDeepSleepMax();
  addFormNumericBox(F("Sleep time"), F("delay"), Settings.Delay, 0, dsmax); // limited by hardware
  {
    String maxSleeptimeUnit = F("sec (max: ");
    maxSleeptimeUnit += String(dsmax);
    maxSleeptimeUnit += ')';
    addUnit(maxSleeptimeUnit);
  }

  addFormCheckBox(F("Sleep on connection failure"), F("deepsleeponfail"), Settings.deepSleepOnFail);

  addFormSeparator(2);

  html_TR_TD();
  html_TD();
  addSubmitButton();
  html_end_table();
  html_end_form();

  sendHeadandTail_stdtemplate(_TAIL);
  TXBuffer.endStream();
}

#endif // ifdef WEBSERVER_CONFIG

#include "../WebServer/ControllerPage.h"


#ifdef WEBSERVER_CONTROLLERS

#include "../WebServer/WebServer.h"
#include "../WebServer/HTML_wrappers.h"
#include "../WebServer/Markup.h"
#include "../WebServer/Markup_Buttons.h"
#include "../WebServer/Markup_Forms.h"

#include "../DataStructs/ESPEasy_EventStruct.h"

#include "../ESPEasyCore/Controller.h"

#include "../Globals/CPlugins.h"
#include "../Globals/Protocol.h"
#include "../Globals/Settings.h"

#include "../Helpers/_CPlugin_Helper_webform.h"
#include "../Helpers/_Plugin_SensorTypeHelper.h"
#include "../Helpers/ESPEasy_Storage.h"
#include "../Helpers/StringConverter.h"



// ********************************************************************************
// Web Interface controller page
// ********************************************************************************
void handle_controllers() {
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("handle_controllers"));
  #endif

  if (!isLoggedIn()) { return; }
  navMenuIndex = MENU_INDEX_CONTROLLERS;
  TXBuffer.startStream();
  sendHeadandTail_stdtemplate(_HEAD);

  byte controllerindex     = getFormItemInt(F("index"), 0);
  boolean controllerNotSet = controllerindex == 0;
  --controllerindex; // Index in URL is starting from 1, but starting from 0 in the array.

  const int protocol = getFormItemInt(F("protocol"), -1);

  // submitted data
  if ((protocol != -1) && !controllerNotSet)
  {
    bool mustInit = false;
    bool mustCallCpluginSave = false;
    {
      // Place in a scope to free ControllerSettings memory ASAP
      MakeControllerSettings(ControllerSettings);
      if (!AllocatedControllerSettings()) {
        addHtmlError(F("Not enough free memory to save settings"));
      } else {
        if (Settings.Protocol[controllerindex] != protocol)
        {
          // Protocol has changed.
          Settings.Protocol[controllerindex] = protocol;

          // there is a protocol selected?
          if (protocol != 0)
          {
            mustInit = true;
            handle_controllers_clearLoadDefaults(controllerindex, ControllerSettings);
          }
        }

        // subitted same protocol
        else
        {
          // there is a protocol selected
          if (protocol != 0)
          {
            mustInit = true;
            handle_controllers_CopySubmittedSettings(controllerindex, ControllerSettings);
            mustCallCpluginSave = true;
          }
        }
        addHtmlError(SaveControllerSettings(controllerindex, ControllerSettings));
      }
    }
    if (mustCallCpluginSave) {
      // Call CPLUGIN_WEBFORM_SAVE after destructing ControllerSettings object to reduce RAM usage.
      // Controller plugin almost only deals with custom controller settings.
      // Even if they need to save things to the ControllerSettings, then the changes must 
      // already be saved first as the CPluginCall does not have the ControllerSettings as argument.
      handle_controllers_CopySubmittedSettings_CPluginCall(controllerindex);
    }
    addHtmlError(SaveSettings());

    if (mustInit) {
      // Init controller plugin using the new settings.
      protocolIndex_t ProtocolIndex = getProtocolIndex_from_ControllerIndex(controllerindex);

      if (validProtocolIndex(ProtocolIndex)) {
        struct EventStruct TempEvent;
        TempEvent.ControllerIndex = controllerindex;
        String dummy;
        CPlugin::Function cfunction = Settings.ControllerEnabled[controllerindex] ? CPlugin::Function::CPLUGIN_INIT : CPlugin::Function::CPLUGIN_EXIT;
        CPluginCall(ProtocolIndex, cfunction, &TempEvent, dummy);
      }
    }
  }

  html_add_form();

  if (controllerNotSet)
  {
    handle_controllers_ShowAllControllersTable();
  }
  else
  {
    handle_controllers_ControllerSettingsPage(controllerindex);
  }

  sendHeadandTail_stdtemplate(_TAIL);
  TXBuffer.endStream();
}

// ********************************************************************************
// Selected controller has changed.
// Clear all Controller settings and load some defaults
// ********************************************************************************
void handle_controllers_clearLoadDefaults(byte controllerindex, ControllerSettingsStruct& ControllerSettings)
{
  // Protocol has changed and it was not an empty one.
  // reset (some) default-settings
  protocolIndex_t ProtocolIndex = getProtocolIndex(Settings.Protocol[controllerindex]);

  if (!validProtocolIndex(ProtocolIndex)) {
    return;
  }

  ControllerSettings.reset();
  ControllerSettings.Port = Protocol[ProtocolIndex].defaultPort;

  // Load some templates from the controller.
  struct EventStruct TempEvent;

  if (Protocol[ProtocolIndex].usesTemplate) {
    String dummy;
    CPluginCall(ProtocolIndex, CPlugin::Function::CPLUGIN_PROTOCOL_TEMPLATE, &TempEvent, dummy);
  }
  safe_strncpy(ControllerSettings.Subscribe,            TempEvent.String1.c_str(), sizeof(ControllerSettings.Subscribe));
  safe_strncpy(ControllerSettings.Publish,              TempEvent.String2.c_str(), sizeof(ControllerSettings.Publish));
  safe_strncpy(ControllerSettings.MQTTLwtTopic,         TempEvent.String3.c_str(), sizeof(ControllerSettings.MQTTLwtTopic));
  safe_strncpy(ControllerSettings.LWTMessageConnect,    TempEvent.String4.c_str(), sizeof(ControllerSettings.LWTMessageConnect));
  safe_strncpy(ControllerSettings.LWTMessageDisconnect, TempEvent.String5.c_str(), sizeof(ControllerSettings.LWTMessageDisconnect));

  // NOTE: do not enable controller by default, give user a change to enter sensible values first
  Settings.ControllerEnabled[controllerindex] = false;

  // not resetted to default (for convenience)
  // SecuritySettings.ControllerUser[controllerindex]
  // SecuritySettings.ControllerPassword[controllerindex]

  ClearCustomControllerSettings(controllerindex);
}

// ********************************************************************************
// Collect all submitted form data and store in the ControllerSettings
// ********************************************************************************
void handle_controllers_CopySubmittedSettings(byte controllerindex, ControllerSettingsStruct& ControllerSettings)
{
  // copy all settings to controller settings struct
  for (int parameterIdx = 0; parameterIdx <= ControllerSettingsStruct::CONTROLLER_ENABLED; ++parameterIdx) {
    ControllerSettingsStruct::VarType varType = static_cast<ControllerSettingsStruct::VarType>(parameterIdx);
    saveControllerParameterForm(ControllerSettings, controllerindex, varType);
  }
}

void handle_controllers_CopySubmittedSettings_CPluginCall(byte controllerindex) {
  protocolIndex_t ProtocolIndex = getProtocolIndex_from_ControllerIndex(controllerindex);

  if (validProtocolIndex(ProtocolIndex)) {
    struct EventStruct TempEvent;
    TempEvent.ControllerIndex = controllerindex;

    // Call controller plugin to save CustomControllerSettings
    String dummy;
    CPluginCall(ProtocolIndex, CPlugin::Function::CPLUGIN_WEBFORM_SAVE, &TempEvent, dummy);
  }
}

// ********************************************************************************
// Show table with all selected controllers
// ********************************************************************************
void handle_controllers_ShowAllControllersTable()
{
  html_table_class_multirow();
  html_TR();
  html_table_header("",           70);
  html_table_header("Nr",         50);
  html_table_header(F("Enabled"), 100);
  html_table_header(F("Protocol"));
  html_table_header("Host");
  html_table_header("Port");

  MakeControllerSettings(ControllerSettings);
  if (AllocatedControllerSettings()) {
    for (controllerIndex_t x = 0; x < CONTROLLER_MAX; x++)
    {
      const bool cplugin_set = Settings.Protocol[x] != INVALID_C_PLUGIN_ID;


      LoadControllerSettings(x, ControllerSettings);
      html_TR_TD();

      if (cplugin_set && !supportedCPluginID(Settings.Protocol[x])) {
        html_add_button_prefix(F("red"), true);
      } else {
        html_add_button_prefix();
      }
      {
        String html;
        html.reserve(32);
        html += F("controllers?index=");
        html += x + 1;
        html += F("'>");

        if (cplugin_set) {
          html += F("Edit");
        } else {
          html += F("Add");
        }
        html += F("</a><TD>");
        html += getControllerSymbol(x);
        addHtml(html);
      }
      html_TD();

      if (cplugin_set)
      {
        addEnabled(Settings.ControllerEnabled[x]);

        html_TD();
        addHtml(getCPluginNameFromCPluginID(Settings.Protocol[x]));
        html_TD();
        {
          const protocolIndex_t ProtocolIndex = getProtocolIndex_from_ControllerIndex(x);
          String hostDescription;
          CPluginCall(ProtocolIndex, CPlugin::Function::CPLUGIN_WEBFORM_SHOW_HOST_CONFIG, 0, hostDescription);

          if (hostDescription.length() != 0) {
            addHtml(hostDescription);
          } else {
            addHtml(ControllerSettings.getHost());
          }
        }

        html_TD();
        addHtml(String(ControllerSettings.Port));
      }
      else {
        html_TD(3);
      }
    }
  }
  html_end_table();
  html_end_form();
}

// ********************************************************************************
// Show the controller settings page
// ********************************************************************************
void handle_controllers_ControllerSettingsPage(controllerIndex_t controllerindex)
{
  if (!validControllerIndex(controllerindex)) {
    return;
  }

  // Show controller settings page
  html_table_class_normal();
  addFormHeader(F("Controller Settings"));
  addRowLabel(F("Protocol"));
  byte choice = Settings.Protocol[controllerindex];
  addSelector_Head_reloadOnChange(F("protocol"));
  addSelector_Item(F("- Standalone -"), 0, false, false, "");

  for (byte x = 0; x <= protocolCount; x++)
  {
    boolean disabled = false; // !((controllerindex == 0) || !Protocol[x].usesMQTT);
    addSelector_Item(getCPluginNameFromProtocolIndex(x),
                     Protocol[x].Number,
                     choice == Protocol[x].Number,
                     disabled,
                     "");
  }
  addSelector_Foot();

  addHelpButton(F("EasyProtocols"));

  const protocolIndex_t ProtocolIndex = getProtocolIndex_from_ControllerIndex(controllerindex);

  if (Settings.Protocol[controllerindex])
  { 
    {
      MakeControllerSettings(ControllerSettings);
      if (!AllocatedControllerSettings()) {
        addHtmlError(F("Out of memory, cannot load page"));
      } else {
        LoadControllerSettings(controllerindex, ControllerSettings);

        if (!Protocol[ProtocolIndex].Custom)
        {
          if (Protocol[ProtocolIndex].usesHost) {
            addControllerParameterForm(ControllerSettings, controllerindex, ControllerSettingsStruct::CONTROLLER_USE_DNS);

            if (ControllerSettings.UseDNS)
            {
              addControllerParameterForm(ControllerSettings, controllerindex, ControllerSettingsStruct::CONTROLLER_HOSTNAME);
            }
            else
            {
              addControllerParameterForm(ControllerSettings, controllerindex, ControllerSettingsStruct::CONTROLLER_IP);
            }
          }
          addControllerParameterForm(ControllerSettings, controllerindex, ControllerSettingsStruct::CONTROLLER_PORT);

          if (Protocol[ProtocolIndex].usesQueue) {
            addTableSeparator(F("Controller Queue"), 2, 3);
            addControllerParameterForm(ControllerSettings, controllerindex, ControllerSettingsStruct::CONTROLLER_MIN_SEND_INTERVAL);
            addControllerParameterForm(ControllerSettings, controllerindex, ControllerSettingsStruct::CONTROLLER_MAX_QUEUE_DEPTH);
            addControllerParameterForm(ControllerSettings, controllerindex, ControllerSettingsStruct::CONTROLLER_MAX_RETRIES);
            addControllerParameterForm(ControllerSettings, controllerindex, ControllerSettingsStruct::CONTROLLER_FULL_QUEUE_ACTION);
          }

          if (Protocol[ProtocolIndex].usesCheckReply) {
            addControllerParameterForm(ControllerSettings, controllerindex, ControllerSettingsStruct::CONTROLLER_CHECK_REPLY);
          }

          if (Protocol[ProtocolIndex].usesTimeout) {
            addControllerParameterForm(ControllerSettings, controllerindex, ControllerSettingsStruct::CONTROLLER_TIMEOUT);
          }

          if (Protocol[ProtocolIndex].usesSampleSets) {
            addControllerParameterForm(ControllerSettings, controllerindex, ControllerSettingsStruct::CONTROLLER_SAMPLE_SET_INITIATOR);
          }

          if (Protocol[ProtocolIndex].useCredentials()) {
            addTableSeparator(F("Credentials"), 2, 3);
          }

          if (Protocol[ProtocolIndex].useExtendedCredentials()) {
            addControllerParameterForm(ControllerSettings, controllerindex, ControllerSettingsStruct::CONTROLLER_USE_EXTENDED_CREDENTIALS);
          }

          if (Protocol[ProtocolIndex].usesAccount)
          {
            addControllerParameterForm(ControllerSettings, controllerindex, ControllerSettingsStruct::CONTROLLER_USER);
          }

          if (Protocol[ProtocolIndex].usesPassword)
          {
            addControllerParameterForm(ControllerSettings, controllerindex, ControllerSettingsStruct::CONTROLLER_PASS);
          }
          #ifdef USES_MQTT
          if (Protocol[ProtocolIndex].usesMQTT) {
            addTableSeparator(F("MQTT"), 2, 3);

            addControllerParameterForm(ControllerSettings, controllerindex, ControllerSettingsStruct::CONTROLLER_CLIENT_ID);
            addControllerParameterForm(ControllerSettings, controllerindex, ControllerSettingsStruct::CONTROLLER_UNIQUE_CLIENT_ID_RECONNECT);        
            addRowLabel(F("Current Client ID"));
            addHtml(getMQTTclientID(ControllerSettings));
            addFormNote(F("Updated on load of this page"));
            addControllerParameterForm(ControllerSettings, controllerindex, ControllerSettingsStruct::CONTROLLER_RETAINFLAG);
          }
          #endif // USES_MQTT


          if (Protocol[ProtocolIndex].usesTemplate || Protocol[ProtocolIndex].usesMQTT)
          {
            addControllerParameterForm(ControllerSettings, controllerindex, ControllerSettingsStruct::CONTROLLER_SUBSCRIBE);
            addControllerParameterForm(ControllerSettings, controllerindex, ControllerSettingsStruct::CONTROLLER_PUBLISH);
          }
          #ifdef USES_MQTT
          if (Protocol[ProtocolIndex].usesMQTT)
          {
            addControllerParameterForm(ControllerSettings, controllerindex, ControllerSettingsStruct::CONTROLLER_LWT_TOPIC);
            addControllerParameterForm(ControllerSettings, controllerindex, ControllerSettingsStruct::CONTROLLER_LWT_CONNECT_MESSAGE);
            addControllerParameterForm(ControllerSettings, controllerindex, ControllerSettingsStruct::CONTROLLER_LWT_DISCONNECT_MESSAGE);
            addControllerParameterForm(ControllerSettings, controllerindex, ControllerSettingsStruct::CONTROLLER_SEND_LWT);
            addControllerParameterForm(ControllerSettings, controllerindex, ControllerSettingsStruct::CONTROLLER_WILL_RETAIN);
            addControllerParameterForm(ControllerSettings, controllerindex, ControllerSettingsStruct::CONTROLLER_CLEAN_SESSION);
          }
          #endif // USES_MQTT
        }
      }
      // End of scope for ControllerSettings, destruct it to save memory.
    }
    {
      // Load controller specific settings
      struct EventStruct TempEvent;
      TempEvent.ControllerIndex = controllerindex;

      String webformLoadString;
      CPluginCall(ProtocolIndex, CPlugin::Function::CPLUGIN_WEBFORM_LOAD, &TempEvent, webformLoadString);

      if (webformLoadString.length() > 0) {
        addHtmlError(F("Bug in CPlugin::Function::CPLUGIN_WEBFORM_LOAD, should not append to string, use addHtml() instead"));
      }
    }
    // Separate enabled checkbox as it doesn't need to use the ControllerSettings.
    // So ControllerSettings object can be destructed before controller specific settings are loaded.
    addControllerEnabledForm(controllerindex);
  }

  addFormSeparator(2);
  html_TR_TD();
  html_TD();
  addButton(F("controllers"), F("Close"));
  addSubmitButton();
  html_end_table();
  html_end_form();
}

#endif // ifdef WEBSERVER_CONTROLLERS

#include "../WebServer/ControlPage.h"


#ifdef WEBSERVER_CONTROL


# include "../WebServer/HTML_wrappers.h"
# include "../Helpers/WebServer_commandHelper.h"

# include "../../ESPEasy-Globals.h"


// ********************************************************************************
// Web Interface control page (no password!)
// ********************************************************************************
void handle_control() {
  # ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("handle_control"));
  # endif // ifndef BUILD_NO_RAM_TRACKER

  String webrequest            = web_server.arg(F("cmd"));
  HandledWebCommand_result res = handle_command_from_web(EventValueSource::Enum::VALUE_SOURCE_HTTP, webrequest);

  switch (res) {
    case HandledWebCommand_result::IP_not_allowed:
    case HandledWebCommand_result::NoCommand:
      return;
    case HandledWebCommand_result::CommandHandled:
    case HandledWebCommand_result::Unknown_or_restricted_command:
      break;
  }

  if (printToWebJSON) { // it may be set in PLUGIN_WRITE (SendStatus)
    TXBuffer.startJsonStream();
    addHtml(printWebString);
  } else {
    TXBuffer.startStream();
    addEncodedHtml(printWebString);
  }

  TXBuffer.endStream();

  printWebString = "";
  printToWeb     = false;
  printToWebJSON = false;
}

#endif // ifdef WEBSERVER_CONTROL

#include "../WebServer/CustomPage.h"

#include "../WebServer/WebServer.h"
#include "../WebServer/AccessControl.h"
#include "../WebServer/HTML_wrappers.h"
#include "../WebServer/Markup.h"
#include "../WebServer/Markup_Forms.h"

#include "../Commands/InternalCommands.h"
#include "../Globals/ExtraTaskSettings.h"
#include "../Globals/Nodes.h"
#include "../Globals/Device.h"
#include "../Globals/Plugins.h"
#include "../Globals/Settings.h"

#include "../Helpers/ESPEasy_Storage.h"
#include "../Helpers/StringParser.h"

#include "../../_Plugin_Helper.h"

#ifdef WEBSERVER_CUSTOM

// ********************************************************************************
// Web Interface custom page handler
// ********************************************************************************
boolean handle_custom(String path) {
  // path is a deepcopy, since it will be changed.
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("handle_custom"));
  #endif

  if (!clientIPallowed()) { return false; }

#ifdef ESP8266
  // For ESP32 remove the leading slash
  path = path.substring(1);
#endif

  // create a dynamic custom page, parsing task values into [<taskname>#<taskvalue>] placeholders and parsing %xx% system variables
  fs::File   dataFile      = tryOpenFile(path.c_str(), "r");
#ifdef ESP8266
  const bool dashboardPage = path.startsWith(F("dashboard"));
#endif
#ifdef ESP32
  const bool dashboardPage = path.startsWith(F("/dashboard"));
#endif

  if (!dataFile && !dashboardPage) {
    return false;    // unknown file that does not exist...
  }

  if (dashboardPage) // for the dashboard page, create a default unit dropdown selector
  {
    // handle page redirects to other unit's as requested by the unit dropdown selector
    byte unit    = getFormItemInt(F("unit"));
    byte btnunit = getFormItemInt(F("btnunit"));

    if (!unit) { unit = btnunit; // unit element prevails, if not used then set to btnunit
    }

    if (unit && (unit != Settings.Unit))
    {
      NodesMap::iterator it = Nodes.find(unit);

      if (it != Nodes.end()) {
        TXBuffer.startStream();
        sendHeadandTail(F("TmplDsh"), _HEAD);
        addHtml(F("<meta http-equiv=\"refresh\" content=\"0; URL=http://"));
        addHtml(it->second.ip.toString());
        addHtml(F("/dashboard.esp\">"));
        sendHeadandTail(F("TmplDsh"), _TAIL);
        TXBuffer.endStream();
        return true;
      }
    }

    TXBuffer.startStream();
    sendHeadandTail(F("TmplDsh"), _HEAD);
    html_add_autosubmit_form();
    html_add_form();

    // create unit selector dropdown
    addSelector_Head_reloadOnChange(F("unit"));
    byte choice = Settings.Unit;

    for (NodesMap::iterator it = Nodes.begin(); it != Nodes.end(); ++it)
    {
      if ((it->second.ip[0] != 0) || (it->first == Settings.Unit))
      {
        String name = String(it->first) + F(" - ");

        if (it->first != Settings.Unit) {
          name += it->second.nodeName;
        }
        else {
          name += Settings.Name;
        }
        addSelector_Item(name, it->first, choice == it->first, false, "");
      }
    }
    addSelector_Foot();

    // create <> navigation buttons
    byte prev = Settings.Unit;
    byte next = Settings.Unit;
    NodesMap::iterator it;

    for (byte x = Settings.Unit - 1; x > 0; x--) {
      it = Nodes.find(x);

      if (it != Nodes.end()) {
        if (it->second.ip[0] != 0) { prev = x; break; }
      }
    }

    for (byte x = Settings.Unit + 1; x < UNIT_NUMBER_MAX; x++) {
      it = Nodes.find(x);

      if (it != Nodes.end()) {
        if (it->second.ip[0] != 0) { next = x; break; }
      }
    }

    html_add_button_prefix();
    addHtml(path);
    addHtml(F("?btnunit="));
    addHtml(String(prev));
    addHtml(F("'>&lt;</a>"));
    html_add_button_prefix();
    addHtml(path);
    addHtml(F("?btnunit="));
    addHtml(String(next));
    addHtml(F("'>&gt;</a>"));
  }

  // handle commands from a custom page
  String webrequest = web_server.arg(F("cmd"));

  if (webrequest.length() > 0) {
    ExecuteCommand_all_config(EventValueSource::Enum::VALUE_SOURCE_HTTP, webrequest.c_str());

    // handle some update processes first, before returning page update...
    String dummy;
    PluginCall(PLUGIN_TEN_PER_SECOND, 0, dummy);
  }


  if (dataFile)
  {
    String page = "";
    page.reserve(dataFile.size());

    while (dataFile.available()) {
      page += ((char)dataFile.read());
    }

    addHtml(parseTemplate(page));
    dataFile.close();
  }
  else // if the requestef file does not exist, create a default action in case the page is named "dashboard*"
  {
    if (dashboardPage)
    {
      // if the custom page does not exist, create a basic task value overview page in case of dashboard request...
      addHtml(F(
                "<meta name='viewport' content='width=width=device-width, initial-scale=1'><STYLE>* {font-family:sans-serif; font-size:16pt;}.button {margin:4px; padding:4px 16px; background-color:#07D; color:#FFF; text-decoration:none; border-radius:4px}</STYLE>"));
      html_table_class_normal();

      for (taskIndex_t x = 0; x < TASKS_MAX; x++)
      {
        if (validPluginID_fullcheck(Settings.TaskDeviceNumber[x]))
        {
          const deviceIndex_t DeviceIndex = getDeviceIndex_from_TaskIndex(x);

          if (validDeviceIndex(DeviceIndex)) {
            LoadTaskSettings(x);
            html_TR_TD();
            addHtml(ExtraTaskSettings.TaskDeviceName);

            const byte valueCount = getValueCountForTask(x);

            for (byte varNr = 0; varNr < VARS_PER_TASK; varNr++)
            {
              if ((varNr < valueCount) &&
                  (ExtraTaskSettings.TaskDeviceValueNames[varNr][0] != 0))
              {
                if (varNr > 0) {
                  html_TR_TD();
                }
                html_TD();
                addHtml(ExtraTaskSettings.TaskDeviceValueNames[varNr]);
                html_TD();
                addHtml(formatUserVarNoCheck(x, varNr));
              }
            }
          }
        }
      }
    }
  }
  sendHeadandTail(F("TmplDsh"), _TAIL);
  TXBuffer.endStream();
  return true;
}

#endif

#include "../WebServer/DevicesPage.h"

#ifdef WEBSERVER_DEVICES

# include "../WebServer/WebServer.h"
# include "../WebServer/HTML_wrappers.h"
# include "../WebServer/Markup.h"
# include "../WebServer/Markup_Buttons.h"
# include "../WebServer/Markup_Forms.h"

# include "../Globals/CPlugins.h"
# include "../Globals/Device.h"
# include "../Globals/ExtraTaskSettings.h"
# include "../Globals/Nodes.h"
# include "../Globals/Plugins.h"
# include "../Globals/Protocol.h"

# include "../Static/WebStaticData.h"

# include "../Helpers/_Plugin_SensorTypeHelper.h"
# include "../Helpers/_Plugin_Helper_serial.h"
# include "../Helpers/ESPEasy_Storage.h"
# include "../Helpers/Hardware.h"
# include "../Helpers/StringConverter.h"
# include "../Helpers/StringGenerator_GPIO.h"

# include "../../_Plugin_Helper.h"

# include <ESPeasySerial.h>


void handle_devices() {
  # ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("handle_devices"));
  # endif // ifndef BUILD_NO_RAM_TRACKER

  if (!isLoggedIn()) { return; }
  navMenuIndex = MENU_INDEX_DEVICES;
  TXBuffer.startStream();
  sendHeadandTail_stdtemplate(_HEAD);


  // char tmpString[41];


  // String taskindex = web_server.arg(F("index"));

  pluginID_t taskdevicenumber;

  if (web_server.hasArg(F("del"))) {
    taskdevicenumber = 0;
  }
  else {
    taskdevicenumber = getFormItemInt(F("TDNUM"), 0);
  }


  // String taskdeviceid[CONTROLLER_MAX];
  // String taskdevicepin1 = web_server.arg(F("taskdevicepin1"));   // "taskdevicepin*" should not be changed because it is uses by plugins
  // and expected to be saved by this code
  // String taskdevicepin2 = web_server.arg(F("taskdevicepin2"));
  // String taskdevicepin3 = web_server.arg(F("taskdevicepin3"));
  // String taskdevicepin1pullup = web_server.arg(F("TDPPU"));
  // String taskdevicepin1inversed = web_server.arg(F("TDPI"));
  // String taskdevicename = web_server.arg(F("TDN"));
  // String taskdeviceport = web_server.arg(F("TDP"));
  // String taskdeviceformula[VARS_PER_TASK];
  // String taskdevicevaluename[VARS_PER_TASK];
  // String taskdevicevaluedecimals[VARS_PER_TASK];
  // String taskdevicesenddata[CONTROLLER_MAX];
  // String taskdeviceglobalsync = web_server.arg(F("TDGS"));
  // String taskdeviceenabled = web_server.arg(F("TDE"));

  // for (byte varNr = 0; varNr < VARS_PER_TASK; varNr++)
  // {
  //   char argc[25];
  //   String arg = F("TDF");
  //   arg += varNr + 1;
  //   arg.toCharArray(argc, 25);
  //   taskdeviceformula[varNr] = web_server.arg(argc);
  //
  //   arg = F("TDVN");
  //   arg += varNr + 1;
  //   arg.toCharArray(argc, 25);
  //   taskdevicevaluename[varNr] = web_server.arg(argc);
  //
  //   arg = F("TDVD");
  //   arg += varNr + 1;
  //   arg.toCharArray(argc, 25);
  //   taskdevicevaluedecimals[varNr] = web_server.arg(argc);
  // }

  // for (controllerIndex_t controllerNr = 0; controllerNr < CONTROLLER_MAX; controllerNr++)
  // {
  //   char argc[25];
  //   String arg = F("TDID");
  //   arg += controllerNr + 1;
  //   arg.toCharArray(argc, 25);
  //   taskdeviceid[controllerNr] = web_server.arg(argc);
  //
  //   arg = F("TDSD");
  //   arg += controllerNr + 1;
  //   arg.toCharArray(argc, 25);
  //   taskdevicesenddata[controllerNr] = web_server.arg(argc);
  // }

  byte page = getFormItemInt(F("page"), 0);

  if (page == 0) {
    page = 1;
  }
  byte setpage = getFormItemInt(F("setpage"), 0);

  if (setpage > 0)
  {
    if (setpage <= (TASKS_MAX / TASKS_PER_PAGE)) {
      page = setpage;
    }
    else {
      page = TASKS_MAX / TASKS_PER_PAGE;
    }
  }
  const int edit = getFormItemInt(F("edit"), 0);

  // taskIndex in the URL is 1 ... TASKS_MAX
  // For use in other functions, set it to 0 ... (TASKS_MAX - 1)
  taskIndex_t taskIndex       = getFormItemInt(F("index"), 0);
  boolean     taskIndexNotSet = taskIndex == 0;

  if (!taskIndexNotSet) {
    --taskIndex;
    LoadTaskSettings(taskIndex); // Make sure ExtraTaskSettings are up-to-date
  }

  // FIXME TD-er: Might have to clear any caches here.
  if ((edit != 0) && !taskIndexNotSet) // when form submitted
  {
    if (Settings.TaskDeviceNumber[taskIndex] != taskdevicenumber)
    {
      // change of device: cleanup old device and reset default settings
      setTaskDevice_to_TaskIndex(taskdevicenumber, taskIndex);
    }
    else if (taskdevicenumber != 0) // save settings
    {
      handle_devices_CopySubmittedSettings(taskIndex, taskdevicenumber);
    }

    if (taskdevicenumber != 0) {
      // Task index has a task device number, so it makes sense to save.
      // N.B. When calling delete, the settings were already saved.
      addHtmlError(SaveTaskSettings(taskIndex));
      addHtmlError(SaveSettings());

      struct EventStruct TempEvent(taskIndex);
      String dummy;

      if (Settings.TaskDeviceEnabled[taskIndex]) {
        PluginCall(PLUGIN_INIT, &TempEvent, dummy);
      } else {
        PluginCall(PLUGIN_EXIT, &TempEvent, dummy);
      }
    }
  }

  // show all tasks as table
  if (taskIndexNotSet)
  {
    handle_devicess_ShowAllTasksTable(page);
  }

  // Show edit form if a specific entry is chosen with the edit button
  else
  {
    handle_devices_TaskSettingsPage(taskIndex, page);
  }

  # ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("handle_devices"));
  # endif // ifndef BUILD_NO_RAM_TRACKER
# ifndef BUILD_NO_DEBUG

  if (loglevelActiveFor(LOG_LEVEL_DEBUG_DEV)) {
    String log = F("DEBUG: String size:");
    log += String(TXBuffer.sentBytes);
    addLog(LOG_LEVEL_DEBUG_DEV, log);
  }
# endif // ifndef BUILD_NO_DEBUG
  sendHeadandTail_stdtemplate(_TAIL);
  TXBuffer.endStream();
}

// ********************************************************************************
// Add a device select dropdown list
// TODO TD-er: Add JavaScript filter:
//             https://www.w3schools.com/howto/howto_js_filter_dropdown.asp
// ********************************************************************************
void addDeviceSelect(const String& name,  int choice)
{
  String deviceName;

  addSelector_Head_reloadOnChange(name);
  addSelector_Item(F("- None -"), 0, false, false, "");

  for (byte x = 0; x <= deviceCount; x++)
  {
    const deviceIndex_t deviceIndex = DeviceIndex_sorted[x];

    if (validDeviceIndex(deviceIndex)) {
      const pluginID_t pluginID = DeviceIndex_to_Plugin_id[deviceIndex];

      if (validPluginID(pluginID)) {
        deviceName = getPluginNameFromDeviceIndex(deviceIndex);


  # ifdef PLUGIN_BUILD_DEV
        String plugin = "P";

        if (pluginID < 10) { plugin += '0'; }

        if (pluginID < 100) { plugin += '0'; }
        plugin    += pluginID;
        plugin    += F(" - ");
        deviceName = plugin + deviceName;
  # endif // ifdef PLUGIN_BUILD_DEV

        addSelector_Item(deviceName,
                         Device[deviceIndex].Number,
                         choice == Device[deviceIndex].Number,
                         false,
                         "");
      }
    }
  }
  addSelector_Foot();
}

// ********************************************************************************
// Collect all submitted form data and store the task settings
// ********************************************************************************
void handle_devices_CopySubmittedSettings(taskIndex_t taskIndex, pluginID_t taskdevicenumber)
{
  if (!validTaskIndex(taskIndex)) { return; }
  const deviceIndex_t DeviceIndex = getDeviceIndex(taskdevicenumber);

  if (!validDeviceIndex(DeviceIndex)) { return; }

  unsigned long taskdevicetimer = getFormItemInt(F("TDT"), 0);

  Settings.TaskDeviceNumber[taskIndex] = taskdevicenumber;


  uint8_t flags = 0;

  if (Device[DeviceIndex].Type == DEVICE_TYPE_I2C) {
    bitWrite(flags, 0, isFormItemChecked(F("taskdeviceflags0")));
  }
# ifdef FEATURE_I2CMULTIPLEXER

  if ((Device[DeviceIndex].Type == DEVICE_TYPE_I2C) && isI2CMultiplexerEnabled()) {
    int multipleMuxPortsOption = getFormItemInt(F("taskdeviceflags1"), 0);
    bitWrite(flags, 1, multipleMuxPortsOption == 1);

    if (multipleMuxPortsOption == 1) {
      uint8_t selectedPorts = 0;

      for (uint8_t x = 0; x < I2CMultiplexerMaxChannels(); x++) {
        String id = F("taskdeviceflag1ch");
        id += String(x);
        bitWrite(selectedPorts, x, isFormItemChecked(id));
      }
      Settings.I2C_Multiplexer_Channel[taskIndex] = selectedPorts;
    } else {
      Settings.I2C_Multiplexer_Channel[taskIndex] = getFormItemInt(F("taskdevicei2cmuxport"), 0);
    }
  }
# endif // ifdef FEATURE_I2CMULTIPLEXER

  if (Device[DeviceIndex].Type == DEVICE_TYPE_I2C) {
    Settings.I2C_Flags[taskIndex] = flags;
  }

  struct EventStruct TempEvent(taskIndex);

  ExtraTaskSettings.clear();
  ExtraTaskSettings.TaskIndex = taskIndex;

  // Save selected output type.
  switch (Device[DeviceIndex].OutputDataType) {
    case Output_Data_type_t::Default:
    {
      String dummy;
      PluginCall(PLUGIN_GET_DEVICEVALUENAMES, &TempEvent, dummy);
      break;
    }
    case Output_Data_type_t::Simple:
    case Output_Data_type_t::All:
    {
      int pconfigIndex = checkDeviceVTypeForTask(&TempEvent);

      if ((pconfigIndex >= 0) && (pconfigIndex < PLUGIN_CONFIGVAR_MAX)) {
        Sensor_VType VType = static_cast<Sensor_VType>(getFormItemInt(PCONFIG_LABEL(pconfigIndex), 0));
        Settings.TaskDevicePluginConfig[taskIndex][pconfigIndex] = static_cast<int>(VType);
        ExtraTaskSettings.clearUnusedValueNames(getValueCountFromSensorType(VType));

        // nr output values has changed, generate new variable names
        String oldNames[VARS_PER_TASK];
        byte   oldNrDec[VARS_PER_TASK];

        for (byte i = 0; i < VARS_PER_TASK; ++i) {
          oldNames[i] = ExtraTaskSettings.TaskDeviceValueNames[i];
          oldNrDec[i] = ExtraTaskSettings.TaskDeviceValueDecimals[i];
        }

        String dummy;
        PluginCall(PLUGIN_GET_DEVICEVALUENAMES, &TempEvent, dummy);

        // Restore the settings that were already set by the user
        for (byte i = 0; i < VARS_PER_TASK; ++i) {
          if (oldNames[i].length() != 0) {
            safe_strncpy(ExtraTaskSettings.TaskDeviceValueNames[i], oldNames[i], sizeof(ExtraTaskSettings.TaskDeviceValueNames[i]));
            ExtraTaskSettings.TaskDeviceValueDecimals[i] = oldNrDec[i];
          }
        }
      }
      break;
    }
  }

  int pin1 = -1;
  int pin2 = -1;
  int pin3 = -1;
  update_whenset_FormItemInt(F("taskdevicepin1"), pin1);
  update_whenset_FormItemInt(F("taskdevicepin2"), pin2);
  update_whenset_FormItemInt(F("taskdevicepin3"), pin3);
  setBasicTaskValues(taskIndex, taskdevicetimer,
                     isFormItemChecked(F("TDE")), web_server.arg(F("TDN")),
                     pin1, pin2, pin3);
  Settings.TaskDevicePort[taskIndex] = getFormItemInt(F("TDP"), 0);
  update_whenset_FormItemInt(F("remoteFeed"), Settings.TaskDeviceDataFeed[taskIndex]);

  for (controllerIndex_t controllerNr = 0; controllerNr < CONTROLLER_MAX; controllerNr++)
  {
    Settings.TaskDeviceID[controllerNr][taskIndex]       = getFormItemInt(String(F("TDID")) + (controllerNr + 1));
    Settings.TaskDeviceSendData[controllerNr][taskIndex] = isFormItemChecked(String(F("TDSD")) + (controllerNr + 1));
  }

  if (Device[DeviceIndex].PullUpOption) {
    Settings.TaskDevicePin1PullUp[taskIndex] = isFormItemChecked(F("TDPPU"));
  }

  if (Device[DeviceIndex].InverseLogicOption) {
    Settings.TaskDevicePin1Inversed[taskIndex] = isFormItemChecked(F("TDPI"));
  }

  if ((Device[DeviceIndex].Type == DEVICE_TYPE_SERIAL) ||
      (Device[DeviceIndex].Type == DEVICE_TYPE_SERIAL_PLUS1))
  {
    #ifdef PLUGIN_USES_SERIAL
    serialHelper_webformSave(&TempEvent);
    #else
    addLog(LOG_LEVEL_ERROR,  F("PLUGIN_USES_SERIAL not defined"));
    #endif

  }

  const byte valueCount = getValueCountForTask(taskIndex);

  for (byte varNr = 0; varNr < valueCount; varNr++)
  {
    strncpy_webserver_arg(ExtraTaskSettings.TaskDeviceFormula[varNr], String(F("TDF")) + (varNr + 1));
    update_whenset_FormItemInt(String(F("TDVD")) + (varNr + 1), ExtraTaskSettings.TaskDeviceValueDecimals[varNr]);
    strncpy_webserver_arg(ExtraTaskSettings.TaskDeviceValueNames[varNr], String(F("TDVN")) + (varNr + 1));
  }

  // allow the plugin to save plugin-specific form settings.
  {
    String dummy;
    PluginCall(PLUGIN_WEBFORM_SAVE, &TempEvent, dummy);
  }

  // notify controllers: CPlugin::Function::CPLUGIN_TASK_CHANGE_NOTIFICATION
  for (controllerIndex_t x = 0; x < CONTROLLER_MAX; x++)
  {
    TempEvent.ControllerIndex = x;

    if (Settings.TaskDeviceSendData[TempEvent.ControllerIndex][TempEvent.TaskIndex] &&
        Settings.ControllerEnabled[TempEvent.ControllerIndex] && Settings.Protocol[TempEvent.ControllerIndex])
    {
      protocolIndex_t ProtocolIndex = getProtocolIndex_from_ControllerIndex(TempEvent.ControllerIndex);
      String dummy;
      CPluginCall(ProtocolIndex, CPlugin::Function::CPLUGIN_TASK_CHANGE_NOTIFICATION, &TempEvent, dummy);
    }
  }
}

// ********************************************************************************
// Show table with all selected Tasks/Devices
// ********************************************************************************
void handle_devicess_ShowAllTasksTable(byte page)
{
  serve_JS(JSfiles_e::UpdateSensorValuesDevicePage);
  html_table_class_multirow();
  html_TR();
  html_table_header("", 70);

  if (TASKS_MAX != TASKS_PER_PAGE)
  {
    html_add_button_prefix();

    {
      String html;
      html.reserve(30);

      html += F("devices?setpage=");

      if (page > 1) {
        html += page - 1;
      }
      else {
        html += page;
      }
      html += F("'>&lt;</a>");
      addHtml(html);
    }
    html_add_button_prefix();
    {
      String html;
      html.reserve(30);

      html += F("devices?setpage=");

      if (page < (TASKS_MAX / TASKS_PER_PAGE)) {
        html += page + 1;
      }
      else {
        html += page;
      }
      html += F("'>&gt;</a>");
      addHtml(html);
    }
  }

  html_table_header(F("Task"),    50);
  html_table_header(F("Enabled"), 100);
  html_table_header(F("Device"));
  html_table_header(F("Name"));
  html_table_header(F("Port"));
  html_table_header(F("Ctr (IDX)"), 100);
  html_table_header(F("GPIO"));
  html_table_header(F("Values"));

  String deviceName;

  for (taskIndex_t x = (page - 1) * TASKS_PER_PAGE; x < ((page) * TASKS_PER_PAGE) && validTaskIndex(x); x++)
  {
    const deviceIndex_t DeviceIndex = getDeviceIndex_from_TaskIndex(x);
    const bool pluginID_set         = INVALID_PLUGIN_ID != Settings.TaskDeviceNumber[x];

    html_TR_TD();

    if (pluginID_set && !supportedPluginID(Settings.TaskDeviceNumber[x])) {
      html_add_button_prefix(F("red"), true);
    } else {
      html_add_button_prefix();
    }
    {
      String html;
      html.reserve(30);

      html += F("devices?index=");
      html += x + 1;
      html += F("&page=");
      html += page;
      html += F("'>");

      if (pluginID_set) {
        html += F("Edit");
      } else {
        html += F("Add");
      }
      html += F("</a><TD>");
      html += x + 1;
      addHtml(html);
      html_TD();
    }

    // Show table of all configured tasks
    // A task may also refer to a non supported plugin.
    // This will be shown as not supported.
    // Editing a task which has a non supported plugin will present the same as when assigning a new plugin to a task.
    if (pluginID_set)
    {
      LoadTaskSettings(x);
      int8_t spi_gpios[3] { -1, -1, -1 };
      struct EventStruct TempEvent(x);
      addEnabled(Settings.TaskDeviceEnabled[x]  && validDeviceIndex(DeviceIndex));

      html_TD();
      addHtml(getPluginNameFromPluginID(Settings.TaskDeviceNumber[x]));
      html_TD();
      addHtml(ExtraTaskSettings.TaskDeviceName);
      html_TD();

      if (validDeviceIndex(DeviceIndex)) {
        if (Settings.TaskDeviceDataFeed[x] != 0) {
          // Show originating node number
          const byte remoteUnit = Settings.TaskDeviceDataFeed[x];
          format_originating_node(remoteUnit);
        } else {
          String portDescr;

          if (PluginCall(PLUGIN_WEBFORM_SHOW_CONFIG, &TempEvent, portDescr)) {
            addHtml(portDescr);
          } else {
            switch (Device[DeviceIndex].Type) {
              case DEVICE_TYPE_I2C:
                format_I2C_port_description(x);
                break;
              case DEVICE_TYPE_SPI:
              case DEVICE_TYPE_SPI2:
              case DEVICE_TYPE_SPI3:
              {
                format_SPI_port_description(spi_gpios);
                break;
              }
              case DEVICE_TYPE_SERIAL:
              case DEVICE_TYPE_SERIAL_PLUS1:
                #ifdef PLUGIN_USES_SERIAL
                addHtml(serialHelper_getSerialTypeLabel(&TempEvent));
                #else
                addHtml(F("PLUGIN_USES_SERIAL not defined"));
                #endif

                break;

              default:

                // Plugin has no custom port formatting, show default one.
                if (Device[DeviceIndex].Ports != 0)
                {
                  addHtml(formatToHex_decimal(Settings.TaskDevicePort[x]));
                }
                break;
            }
          }
        }
      }

      html_TD();

      if (validDeviceIndex(DeviceIndex)) {
        if (Device[DeviceIndex].SendDataOption)
        {
          boolean doBR = false;

          for (controllerIndex_t controllerNr = 0; controllerNr < CONTROLLER_MAX; controllerNr++)
          {
            if (Settings.TaskDeviceSendData[controllerNr][x])
            {
              if (doBR) {
                html_BR();
              }
              addHtml(getControllerSymbol(controllerNr));
              protocolIndex_t ProtocolIndex = getProtocolIndex_from_ControllerIndex(controllerNr);

              if (validProtocolIndex(ProtocolIndex)) {
                if (Protocol[ProtocolIndex].usesID && (Settings.Protocol[controllerNr] != 0))
                {
                  String html;
                  html.reserve(16);
                  html += " (";
                  html += Settings.TaskDeviceID[controllerNr][x];
                  html += ')';

                  if (Settings.TaskDeviceID[controllerNr][x] == 0) {
                    html += ' ';
                    html += F(HTML_SYMBOL_WARNING);
                  }
                  addHtml(html);
                }
                doBR = true;
              }
            }
          }
        }
      }

      html_TD();

      if (validDeviceIndex(DeviceIndex)) {
        if (Settings.TaskDeviceDataFeed[x] == 0)
        {
          bool showpin1 = false;
          bool showpin2 = false;
          bool showpin3 = false;

          switch (Device[DeviceIndex].Type) {
            case DEVICE_TYPE_I2C:
            {
              format_I2C_pin_description();
              break;
            }
            case DEVICE_TYPE_SPI3:
              showpin3 = true;

            // Fall Through
            case DEVICE_TYPE_SPI2:
              showpin2 = true;

            // Fall Through
            case DEVICE_TYPE_SPI:
              format_SPI_pin_description(spi_gpios, x);
              break;
            case DEVICE_TYPE_ANALOG:
            {
              # ifdef ESP8266
                #  if FEATURE_ADC_VCC
              addHtml(F("ADC (VDD)"));
                #  else // if FEATURE_ADC_VCC
              addHtml(F("ADC (TOUT)"));
                #  endif // if FEATURE_ADC_VCC
              # endif // ifdef ESP8266
              # ifdef ESP32
              showpin1 = true;
              addHtml(formatGpioName_ADC(Settings.TaskDevicePin1[x]));
              html_BR();
              # endif // ifdef ESP32

              break;
            }
            case DEVICE_TYPE_SERIAL_PLUS1:
              showpin3 = true;

            // fallthrough
            case DEVICE_TYPE_SERIAL:
            {
              #ifdef PLUGIN_USES_SERIAL
              addHtml(serialHelper_getGpioDescription(static_cast<ESPEasySerialPort>(Settings.TaskDevicePort[x]), Settings.TaskDevicePin1[x],
                                                      Settings.TaskDevicePin2[x], F("<BR>")));
              #else
              addHtml(F("PLUGIN_USES_SERIAL not defined"));
              #endif

              if (showpin3) {
                html_BR();
              }
              break;
            }
            default:
              showpin1 = true;
              showpin2 = true;
              showpin3 = true;
              break;
          }

          if ((Settings.TaskDevicePin1[x] != -1) && showpin1)
          {
            String html = formatGpioLabel(Settings.TaskDevicePin1[x], false);

            if ((spi_gpios[0] == Settings.TaskDevicePin1[x])
                || (spi_gpios[1] == Settings.TaskDevicePin1[x])
                || (spi_gpios[2] == Settings.TaskDevicePin1[x])
                || (Settings.Pin_i2c_sda == Settings.TaskDevicePin1[x])
                || (Settings.Pin_i2c_scl == Settings.TaskDevicePin1[x])) {
              html += ' ';
              html += F(HTML_SYMBOL_WARNING);
            }
            addHtml(html);
          }

          if ((Settings.TaskDevicePin2[x] != -1) && showpin2)
          {
            html_BR();
            String html = formatGpioLabel(Settings.TaskDevicePin2[x], false);

            if ((spi_gpios[0] == Settings.TaskDevicePin2[x])
                || (spi_gpios[1] == Settings.TaskDevicePin2[x])
                || (spi_gpios[2] == Settings.TaskDevicePin2[x])
                || (Settings.Pin_i2c_sda == Settings.TaskDevicePin2[x])
                || (Settings.Pin_i2c_scl == Settings.TaskDevicePin2[x])) {
              html += ' ';
              html += F(HTML_SYMBOL_WARNING);
            }
            addHtml(html);
          }

          if ((Settings.TaskDevicePin3[x] != -1) && showpin3)
          {
            html_BR();
            String html = formatGpioLabel(Settings.TaskDevicePin3[x], false);

            if ((spi_gpios[0] == Settings.TaskDevicePin3[x])
                || (spi_gpios[1] == Settings.TaskDevicePin3[x])
                || (spi_gpios[2] == Settings.TaskDevicePin3[x])
                || (Settings.Pin_i2c_sda == Settings.TaskDevicePin3[x])
                || (Settings.Pin_i2c_scl == Settings.TaskDevicePin3[x])) {
              html += ' ';
              html += F(HTML_SYMBOL_WARNING);
            }
            addHtml(html);
          }
        }
      }

      html_TD();

      if (validDeviceIndex(DeviceIndex)) {
        String customValuesString;
        const bool customValues = PluginCall(PLUGIN_WEBFORM_SHOW_VALUES, &TempEvent, customValuesString);

        if (!customValues)
        {
          const byte valueCount = getValueCountForTask(x);

          for (byte varNr = 0; varNr < valueCount; varNr++)
          {
            if (validPluginID_fullcheck(Settings.TaskDeviceNumber[x]))
            {
              addHtml(pluginWebformShowValue(x, varNr, ExtraTaskSettings.TaskDeviceValueNames[varNr], formatUserVarNoCheck(x, varNr)));
            }
          }
        }
      }
    }
    else {
      html_TD(6);
    }
  } // next
  html_end_table();
  html_end_form();
}

void format_originating_node(byte remoteUnit) {
  addHtml(F("Unit "));
  addHtml(String(remoteUnit));

  if (remoteUnit != 255) {
    NodesMap::iterator it = Nodes.find(remoteUnit);

    if (it != Nodes.end()) {
      addHtml(F(" - "));
      addHtml(it->second.nodeName);
    } else {
      addHtml(F(" - Not Seen recently"));
    }
  }
}

void format_I2C_port_description(taskIndex_t x)
{
  addHtml(F("I2C"));
# ifdef FEATURE_I2CMULTIPLEXER

  if (isI2CMultiplexerEnabled() && I2CMultiplexerPortSelectedForTask(x)) {
    String mux;

    if (bitRead(Settings.I2C_Flags[x], I2C_FLAGS_MUX_MULTICHANNEL)) { // Multi-channel
      mux = F("<BR>Multiplexer channel(s)");
      uint8_t b = 0;                                                  // For adding lineBreaks

      for (uint8_t c = 0; c < I2CMultiplexerMaxChannels(); c++) {
        if (bitRead(Settings.I2C_Multiplexer_Channel[x], c)) {
          mux += b == 0 ? F("<BR>") : F(", ");
          b++;
          mux += String(c);
        }
      }
    } else { // Single channel
      mux  = F("<BR>Multiplexer channel ");
      mux += String(Settings.I2C_Multiplexer_Channel[x]);
    }
    addHtml(mux);
  }
# endif // ifdef FEATURE_I2CMULTIPLEXER
}

void format_SPI_port_description(int8_t spi_gpios[3])
{
  if (Settings.InitSPI == 0) {
    addHtml(F("SPI (Not enabled)"));
  } else {
    # ifdef ESP32

    switch (Settings.InitSPI) {
      case 1:
      {
        addHtml(F("VSPI"));
        spi_gpios[0] = 18; spi_gpios[1] = 19; spi_gpios[2] = 23;
        break;
      }
      case 2:
      {
        addHtml(F("HSPI"));
        spi_gpios[0] = 14; spi_gpios[1] = 12; spi_gpios[2] = 13;
        break;
      }
    }
    # endif // ifdef ESP32
    # ifdef ESP8266
    addHtml(F("SPI"));
    spi_gpios[0] = 14; spi_gpios[1] = 12; spi_gpios[2] = 13;
    # endif // ifdef ESP8266
  }
}

void format_I2C_pin_description()
{
  String html;

  html.reserve(20);
  html += F("SDA: ");
  html += formatGpioLabel(Settings.Pin_i2c_sda, false);
  html += F("<BR>SCL: ");
  html += formatGpioLabel(Settings.Pin_i2c_scl, false);

  addHtml(html);
}

void format_SPI_pin_description(int8_t spi_gpios[3], taskIndex_t x)
{
  if (Settings.InitSPI != 0) {
    for (int i = 0; i < 3; ++i) {
      switch (i) {
        case 0:  addHtml(F("CLK: ")); break;
        case 1:  addHtml(F("MISO: ")); break;
        case 2:  addHtml(F("MOSI: ")); break;
      }
      addHtml(formatGpioLabel(spi_gpios[i], false));
      html_BR();
    }
    addHtml(F("CS: "));
    addHtml(formatGpioLabel(Settings.TaskDevicePin1[x], false));
  }
}

// ********************************************************************************
// Show the task settings page
// ********************************************************************************
void handle_devices_TaskSettingsPage(taskIndex_t taskIndex, byte page)
{
  if (!validTaskIndex(taskIndex)) { return; }
  const deviceIndex_t DeviceIndex = getDeviceIndex_from_TaskIndex(taskIndex);

  LoadTaskSettings(taskIndex);

  html_add_form();
  html_table_class_normal();
  addFormHeader(F("Task Settings"));


  addHtml(F("<TR><TD style='width:150px;' align='left'>Device:<TD>"));

  // no (supported) device selected, this effectively checks for validDeviceIndex
  if (!supportedPluginID(Settings.TaskDeviceNumber[taskIndex]))
  {
    // takes lots of memory/time so call this only when needed.
    addDeviceSelect(F("TDNUM"), Settings.TaskDeviceNumber[taskIndex]); // ="taskdevicenumber"
  }

  // device selected
  else
  {
    // remember selected device number
    addHtml(F("<input "));
    addHtmlAttribute(F("type"),  F("hidden"));
    addHtmlAttribute(F("name"),  F("TDNUM"));
    addHtmlAttribute(F("value"), Settings.TaskDeviceNumber[taskIndex]);
    addHtml('>');

    // show selected device name and delete button
    addHtml(getPluginNameFromDeviceIndex(DeviceIndex));

    addHelpButton(String(F("Plugin")) + Settings.TaskDeviceNumber[taskIndex]);
    addRTDPluginButton(Settings.TaskDeviceNumber[taskIndex]);


    if ((Device[DeviceIndex].Number == 3) && (taskIndex >= 4)) // Number == 3 = PulseCounter Plugin
    {
      // FIXME TD-er: Make a PLUGIN_WEBFORM_SHOW_TASKCONFIG_WARNING
      addFormNote(F("This plugin is only supported on task 1-4 for now"));
    }

    addFormTextBox(F("Name"), F("TDN"), ExtraTaskSettings.TaskDeviceName, NAME_FORMULA_LENGTH_MAX); // ="taskdevicename"

    addFormCheckBox(F("Enabled"), F("TDE"), Settings.TaskDeviceEnabled[taskIndex]);                 // ="taskdeviceenabled"

    // section: Sensor / Actuator
    if (!Device[DeviceIndex].Custom && (Settings.TaskDeviceDataFeed[taskIndex] == 0) &&
        ((Device[DeviceIndex].Ports != 0) ||
         (Device[DeviceIndex].PullUpOption) ||
         (Device[DeviceIndex].InverseLogicOption) ||
         (Device[DeviceIndex].connectedToGPIOpins())))
    {
      addFormSubHeader((Device[DeviceIndex].SendDataOption) ? F("Sensor") : F("Actuator"));

      if (Device[DeviceIndex].Ports != 0) {
        addFormNumericBox(F("Port"), F("TDP"), Settings.TaskDevicePort[taskIndex]); // ="taskdeviceport"
      }

      devicePage_show_pin_config(taskIndex, DeviceIndex);
    }

    switch (Device[DeviceIndex].Type) {
      case DEVICE_TYPE_SERIAL:
      case DEVICE_TYPE_SERIAL_PLUS1:
      {
        #ifdef PLUGIN_USES_SERIAL
        devicePage_show_serial_config(taskIndex);
        #else
        addHtml(F("PLUGIN_USES_SERIAL not defined"));
        #endif

        break;
      }

      case DEVICE_TYPE_I2C:
      {
        devicePage_show_I2C_config(taskIndex);

        // FIXME TD-er: Why do we need this only for I2C devices?
        addFormSubHeader(F("Device settings"));
        break;
      }

      default: break;
    }

    devicePage_show_output_data_type(taskIndex, DeviceIndex);


    // add plugins content
    if (Settings.TaskDeviceDataFeed[taskIndex] == 0) { // only show additional config for local connected sensors
      String webformLoadString;
      struct EventStruct TempEvent(taskIndex);
      PluginCall(PLUGIN_WEBFORM_LOAD, &TempEvent, webformLoadString);

      if (webformLoadString.length() > 0) {
        String errorMessage;
        PluginCall(PLUGIN_GET_DEVICENAME, &TempEvent, errorMessage);
        errorMessage += F(": Bug in PLUGIN_WEBFORM_LOAD, should not append to string, use addHtml() instead");
        addHtmlError(errorMessage);
      }
    }
    else {
      // Show remote feed information.
      addFormSubHeader(F("Data Source"));
      byte remoteUnit = Settings.TaskDeviceDataFeed[taskIndex];
      addFormNumericBox(F("Remote Unit"), F("RemoteUnit"), remoteUnit, 0, 255);

      if (remoteUnit != 255) {
        NodesMap::iterator it = Nodes.find(remoteUnit);

        if (it != Nodes.end()) {
          addUnit(it->second.nodeName);
        } else {
          addUnit(F("Unknown Unit Name"));
        }
      }
      addFormNote(F("0 = disable remote feed, 255 = broadcast")); // FIXME TD-er: Must verify if broadcast can be set.
    }


    // section: Data Acquisition
    devicePage_show_controller_config(taskIndex, DeviceIndex);

    addFormSeparator(2);

    devicePage_show_interval_config(taskIndex, DeviceIndex);

    devicePage_show_task_values(taskIndex, DeviceIndex);
  }

  addFormSeparator(4);

  html_TR_TD();
  addHtml(F("<TD colspan='3'>"));
  html_add_button_prefix();
  {
    String html;
    html.reserve(32);

    html += F("devices?setpage=");
    html += page;
    html += F("'>Close</a>");
    addHtml(html);
  }
  addSubmitButton();
  addHtml(F("<input type='hidden' name='edit' value='1'>"));
  addHtml(F("<input type='hidden' name='page' value='1'>"));

  // if user selected a device, add the delete button
  if (validPluginID_fullcheck(Settings.TaskDeviceNumber[taskIndex])) {
    addSubmitButton(F("Delete"), F("del"));
  }

  html_end_table();
  html_end_form();
}

void devicePage_show_pin_config(taskIndex_t taskIndex, deviceIndex_t DeviceIndex)
{
  if (Device[DeviceIndex].PullUpOption)
  {
    addFormCheckBox(F("Internal PullUp"), F("TDPPU"), Settings.TaskDevicePin1PullUp[taskIndex]); // ="taskdevicepin1pullup"
      # if defined(ESP8266)

    if ((Settings.TaskDevicePin1[taskIndex] == 16) || (Settings.TaskDevicePin2[taskIndex] == 16) ||
        (Settings.TaskDevicePin3[taskIndex] == 16)) {
      addFormNote(F("PullDown for GPIO-16 (D0)"));
    }
      # endif // if defined(ESP8266)
  }

  if (Device[DeviceIndex].InverseLogicOption)
  {
    addFormCheckBox(F("Inversed Logic"), F("TDPI"), Settings.TaskDevicePin1Inversed[taskIndex]); // ="taskdevicepin1inversed"
    addFormNote(F("Will go into effect on next input change."));
  }

  if (((Device[DeviceIndex].Type == DEVICE_TYPE_SPI)
       || (Device[DeviceIndex].Type == DEVICE_TYPE_SPI2)
       || (Device[DeviceIndex].Type == DEVICE_TYPE_SPI3))
      && (Settings.InitSPI == 0)) {
    addFormNote(F("SPI Interface is not configured yet (Hardware page)."));
  }

  if ((Device[DeviceIndex].Type == DEVICE_TYPE_I2C)
      && ((Settings.Pin_i2c_sda == -1)
          || (Settings.Pin_i2c_scl == -1)
          || (Settings.I2C_clockSpeed == 0))) {
    addFormNote(F("I2C Interface is not configured yet (Hardware page)."));
  }

  if (Device[DeviceIndex].connectedToGPIOpins()) {
    // get descriptive GPIO-names from plugin
    struct EventStruct TempEvent(taskIndex);

    TempEvent.String1 = F("1st GPIO");
    TempEvent.String2 = F("2nd GPIO");
    TempEvent.String3 = F("3rd GPIO");
    String dummy;
    PluginCall(PLUGIN_GET_DEVICEGPIONAMES, &TempEvent, dummy);

    if (Device[DeviceIndex].usesTaskDevicePin(1)) {
      addFormPinSelect(TempEvent.String1, F("taskdevicepin1"), Settings.TaskDevicePin1[taskIndex]);
    }

    if (Device[DeviceIndex].usesTaskDevicePin(2)) {
      addFormPinSelect(TempEvent.String2, F("taskdevicepin2"), Settings.TaskDevicePin2[taskIndex]);
    }

    if (Device[DeviceIndex].usesTaskDevicePin(3)) {
      addFormPinSelect(TempEvent.String3, F("taskdevicepin3"), Settings.TaskDevicePin3[taskIndex]);
    }
  }
}

void devicePage_show_serial_config(taskIndex_t taskIndex)
{
  struct EventStruct TempEvent(taskIndex);

  serialHelper_webformLoad(&TempEvent);
  String webformLoadString;

  PluginCall(PLUGIN_WEBFORM_SHOW_SERIAL_PARAMS, &TempEvent, webformLoadString);
}

void devicePage_show_I2C_config(taskIndex_t taskIndex)
{
  struct EventStruct TempEvent(taskIndex);

  addFormSubHeader(F("I2C options"));
  String dummy;

  PluginCall(PLUGIN_WEBFORM_SHOW_I2C_PARAMS, &TempEvent, dummy);
  addFormCheckBox(F("Force Slow I2C speed"), F("taskdeviceflags0"), bitRead(Settings.I2C_Flags[taskIndex], I2C_FLAGS_SLOW_SPEED));

# ifdef FEATURE_I2CMULTIPLEXER

  // Show selector for an I2C multiplexer port if a multiplexer is configured
  if (isI2CMultiplexerEnabled()) {
    bool multipleMuxPorts = bitRead(Settings.I2C_Flags[taskIndex], I2C_FLAGS_MUX_MULTICHANNEL);
    {
      String i2c_mux_channels[2];
      int    i2c_mux_channelOptions[2];
      int    i2c_mux_channelCount = 1;
      i2c_mux_channels[0]       = F("Single channel");
      i2c_mux_channelOptions[0] = 0;

      if (Settings.I2C_Multiplexer_Type == I2C_MULTIPLEXER_PCA9540) {
        multipleMuxPorts = false; // force off
      } else {
        i2c_mux_channels[1]       = F("Multiple channels");
        i2c_mux_channelOptions[1] = 1;
        i2c_mux_channelCount++;
      }
      addFormSelector(F("Multiplexer channels"),
                      F("taskdeviceflags1"),
                      i2c_mux_channelCount,
                      i2c_mux_channels,
                      i2c_mux_channelOptions,
                      multipleMuxPorts ? 1 : 0,
                      true);
    }

    if (multipleMuxPorts) {
      addRowLabel(F("Select connections"), F(""));
      html_table(F(""), false); // Sub-table
      html_table_header(F("Channel"));
      html_table_header(F("Enable"));
      html_table_header(F("Channel"));
      html_table_header(F("Enable"));

      for (uint8_t x = 0; x < I2CMultiplexerMaxChannels(); x++) {
        String label = F("Channel ");
        label += String(x);
        String id = F("taskdeviceflag1ch");
        id += String(x);

        if (x % 2 == 0) { html_TR(); } // Start a new row for every 2 channels
        html_TD();
        addHtml(label);
        html_TD();
        addCheckBox(id, bitRead(Settings.I2C_Multiplexer_Channel[taskIndex], x), false);
      }
      html_end_table();
    } else {
      int taskDeviceI2CMuxPort = Settings.I2C_Multiplexer_Channel[taskIndex];
      String  i2c_mux_portoptions[9];
      int     i2c_mux_portchoices[9];
      uint8_t mux_opt = 0;
      i2c_mux_portoptions[mux_opt] = F("(Not connected via multiplexer)");
      i2c_mux_portchoices[mux_opt] = -1;
      uint8_t mux_max = I2CMultiplexerMaxChannels();

      for (int8_t x = 0; x < mux_max; x++) {
        mux_opt++;
        i2c_mux_portoptions[mux_opt]  = F("Channel ");
        i2c_mux_portoptions[mux_opt] += String(x);

        i2c_mux_portchoices[mux_opt] = x;
      }

      if (taskDeviceI2CMuxPort >= mux_max) { taskDeviceI2CMuxPort = -1; } // Reset if out of range
      addFormSelector(F("Connected to"),
                      F("taskdevicei2cmuxport"),
                      mux_opt + 1,
                      i2c_mux_portoptions,
                      i2c_mux_portchoices,
                      taskDeviceI2CMuxPort);
    }
  }
# endif // ifdef FEATURE_I2CMULTIPLEXER
}

void devicePage_show_output_data_type(taskIndex_t taskIndex, deviceIndex_t DeviceIndex)
{
  struct EventStruct TempEvent(taskIndex);

  int pconfigIndex = checkDeviceVTypeForTask(&TempEvent);

  switch (Device[DeviceIndex].OutputDataType) {
    case Output_Data_type_t::Default:
      break;
    case Output_Data_type_t::Simple:

      if (pconfigIndex >= 0) {
        sensorTypeHelper_webformLoad_simple(&TempEvent, pconfigIndex);
      }
      break;
    case Output_Data_type_t::All:
    {
      if (pconfigIndex >= 0) {
        sensorTypeHelper_webformLoad_allTypes(&TempEvent, pconfigIndex);
      }
      break;
    }
  }
}

void devicePage_show_controller_config(taskIndex_t taskIndex, deviceIndex_t DeviceIndex)
{
  if (Device[DeviceIndex].SendDataOption)
  {
    addFormSubHeader(F("Data Acquisition"));

    for (controllerIndex_t controllerNr = 0; controllerNr < CONTROLLER_MAX; controllerNr++)
    {
      if (Settings.Protocol[controllerNr] != 0)
      {
        String id = F("TDSD"); // ="taskdevicesenddata"
        id += controllerNr + 1;

        html_TR_TD();
        addHtml(F("Send to Controller "));
        addHtml(getControllerSymbol(controllerNr));
        html_TD();
        addCheckBox(id, Settings.TaskDeviceSendData[controllerNr][taskIndex]);

        protocolIndex_t ProtocolIndex = getProtocolIndex_from_ControllerIndex(controllerNr);

        if (validProtocolIndex(ProtocolIndex)) {
          if (Protocol[ProtocolIndex].usesID && (Settings.Protocol[controllerNr] != 0))
          {
            addRowLabel(F("IDX"));
            id  = F("TDID"); // ="taskdeviceid"
            id += controllerNr + 1;
            addNumericBox(id, Settings.TaskDeviceID[controllerNr][taskIndex], 0, DOMOTICZ_MAX_IDX);
          }
        }
      }
    }
  }
}

void devicePage_show_interval_config(taskIndex_t taskIndex, deviceIndex_t DeviceIndex)
{
  if (Device[DeviceIndex].TimerOption)
  {
    // FIXME: shoudn't the max be ULONG_MAX because Settings.TaskDeviceTimer is an unsigned long? addFormNumericBox only supports ints
    // for min and max specification
    addFormNumericBox(F("Interval"), F("TDT"), Settings.TaskDeviceTimer[taskIndex], 0, 65535); // ="taskdevicetimer"
    addUnit(F("sec"));

    if (Device[DeviceIndex].TimerOptional) {
      addHtml(F(" (Optional for this Device)"));
    }
  }
}

void devicePage_show_task_values(taskIndex_t taskIndex, deviceIndex_t DeviceIndex)
{
  // section: Values
  const byte valueCount = getValueCountForTask(taskIndex);

  if (!Device[DeviceIndex].Custom && (valueCount > 0))
  {
    addFormSubHeader(F("Values"));
    html_end_table();
    html_table_class_normal();

    // table header
    addHtml(F("<TR><TH style='width:30px;' align='center'>#"));
    html_table_header("Name");

    if (Device[DeviceIndex].FormulaOption)
    {
      html_table_header(F("Formula"), F("EasyFormula"), 0);
    }

    if (Device[DeviceIndex].configurableDecimals())
    {
      html_table_header(F("Decimals"), 30);
    }

    // table body
    for (byte varNr = 0; varNr < valueCount; varNr++)
    {
      html_TR_TD();
      addHtml(String(varNr + 1));
      html_TD();
      String id = F("TDVN"); // ="taskdevicevaluename"
      id += (varNr + 1);
      addTextBox(id, ExtraTaskSettings.TaskDeviceValueNames[varNr], NAME_FORMULA_LENGTH_MAX);

      if (Device[DeviceIndex].FormulaOption)
      {
        html_TD();
        String id = F("TDF"); // ="taskdeviceformula"
        id += (varNr + 1);
        addTextBox(id, ExtraTaskSettings.TaskDeviceFormula[varNr], NAME_FORMULA_LENGTH_MAX);
      }

      if (Device[DeviceIndex].configurableDecimals())
      {
        html_TD();
        String id = F("TDVD"); // ="taskdevicevaluedecimals"
        id += (varNr + 1);
        addNumericBox(id, ExtraTaskSettings.TaskDeviceValueDecimals[varNr], 0, 6);
      }
    }
  }
}

#endif // ifdef WEBSERVER_DEVICES

#include "../WebServer/DownloadPage.h"

#ifdef WEBSERVER_DOWNLOAD

#include "../WebServer/WebServer.h"
#include "../Globals/ESPEasy_time.h"
#include "../Globals/Settings.h"
#include "../Helpers/ESPEasy_Storage.h"


// ********************************************************************************
// Web Interface download page
// ********************************************************************************
void handle_download()
{
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("handle_download"));
  #endif

  if (!isLoggedIn()) { return; }
  navMenuIndex = MENU_INDEX_TOOLS;

  //  TXBuffer.startStream();
  //  sendHeadandTail_stdtemplate();


  fs::File dataFile = tryOpenFile(F(FILE_CONFIG), "r");

  if (!dataFile) {
    return;
  }

  String str = F("attachment; filename=config_");
  str += Settings.Name;
  str += "_U";
  str += Settings.Unit;
  str += F("_Build");
  str += BUILD;
  str += '_';

  if (node_time.systemTimePresent())
  {
    str += node_time.getDateTimeString('\0', '\0', '\0');
  }
  str += F(".dat");

  web_server.sendHeader(F("Content-Disposition"), str);
  web_server.streamFile(dataFile, F("application/octet-stream"));
  dataFile.close();
}

#endif // ifdef WEBSERVER_DOWNLOAD

#include "../WebServer/FactoryResetPage.h"


#ifdef WEBSERVER_FACTORY_RESET

#include "../WebServer/WebServer.h"
#include "../WebServer/HTML_wrappers.h"
#include "../WebServer/Markup.h"
#include "../WebServer/Markup_Buttons.h"
#include "../WebServer/Markup_Forms.h"

#include "../DataTypes/DeviceModel.h"

#include "../Globals/ResetFactoryDefaultPref.h"

#include "../Helpers/ESPEasy_FactoryDefault.h"
#include "../Helpers/ESPEasy_Storage.h"
#include "../Helpers/Hardware.h"

// ********************************************************************************
// Web Interface Factory Reset
// ********************************************************************************
void handle_factoryreset() {
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("handle_factoryreset"));
  #endif

  if (!isLoggedIn()) { return; }
  navMenuIndex = MENU_INDEX_TOOLS;
  TXBuffer.startStream();
  sendHeadandTail_stdtemplate(_HEAD);
  html_add_form();
  html_table_class_normal();
  html_TR();
  addFormHeader(F("Factory Reset"));

  if (web_server.hasArg("fdm")) {
    DeviceModel model = static_cast<DeviceModel>(getFormItemInt("fdm"));

    if (modelMatchingFlashSize(model)) {
      setFactoryDefault(model);
    }
  }

  if (web_server.hasArg(F("savepref"))) {
    // User choose a pre-defined config and wants to save it as the new default.
    ResetFactoryDefaultPreference.keepUnitName(isFormItemChecked("kun"));
    ResetFactoryDefaultPreference.keepWiFi(isFormItemChecked("kw"));
    ResetFactoryDefaultPreference.keepNetwork(isFormItemChecked("knet"));
    ResetFactoryDefaultPreference.keepNTP(isFormItemChecked("kntp"));
    ResetFactoryDefaultPreference.keepLogSettings(isFormItemChecked("klog"));
    applyFactoryDefaultPref();
    addHtmlError(SaveSettings());
  }

  if (web_server.hasArg(F("performfactoryreset"))) {
    // User confirmed to really perform the reset.
    applyFactoryDefaultPref();

    // No need to call SaveSettings(); ResetFactory() will save the new settings.
    ResetFactory();
  } else {
    // Nothing chosen yet, show options.
    addTableSeparator(F("Settings to keep"), 2, 3);

    addRowLabel(F("Keep Unit/Name"));
    addCheckBox("kun", ResetFactoryDefaultPreference.keepUnitName());

    addRowLabel(F("Keep WiFi config"));
    addCheckBox("kw", ResetFactoryDefaultPreference.keepWiFi());

    addRowLabel(F("Keep Network config"));
    addCheckBox("knet", ResetFactoryDefaultPreference.keepNetwork());

    addRowLabel(F("Keep NTP/DST config"));
    addCheckBox("kntp", ResetFactoryDefaultPreference.keepNTP());

    addRowLabel(F("Keep log config"));
    addCheckBox("klog", ResetFactoryDefaultPreference.keepLogSettings());

    addTableSeparator(F("Pre-defined configurations"), 2, 3);
    addRowLabel(F("Pre-defined config"));
    addPreDefinedConfigSelector();


    html_TR_TD();
    html_TD();
    addSubmitButton(F("Save Preferences"), F("savepref"));


    html_TR_TD_height(30);

    addTableSeparator(F("Immediate full reset"), 2, 3);
    addRowLabel(F("Erase settings files"));
    addSubmitButton(F("Factory Reset"), F("performfactoryreset"), F("red"));
  }

  html_end_table();
  html_end_form();
  sendHeadandTail_stdtemplate(_TAIL);
  TXBuffer.endStream();
}

// ********************************************************************************
// Create pre-defined config selector
// ********************************************************************************
void addPreDefinedConfigSelector() {
  DeviceModel active_model = ResetFactoryDefaultPreference.getDeviceModel();

  addSelector_Head_reloadOnChange("fdm");

  for (byte x = 0; x < DeviceModel_MAX; ++x) {
    DeviceModel model = static_cast<DeviceModel>(x);
    if (modelMatchingFlashSize(model)) {
      addSelector_Item(
        getDeviceModelString(model),
        x,
        model == active_model,
        false,
        ""
        );
    }
  }
  addSelector_Foot();
}

#ifdef WEBSERVER_NEW_UI
void handle_factoryreset_json() {
  if (!isLoggedIn()) { return; }
  TXBuffer.startJsonStream();
  addHtml("{");

  if (web_server.hasArg("fdm")) {
    DeviceModel model = static_cast<DeviceModel>(getFormItemInt("fdm"));

    if (modelMatchingFlashSize(model)) {
      setFactoryDefault(model);
    }
  }

  if (web_server.hasArg("kun")) {
    ResetFactoryDefaultPreference.keepUnitName(isFormItemChecked("kun"));
  }

  if (web_server.hasArg("kw")) {
    ResetFactoryDefaultPreference.keepWiFi(isFormItemChecked("kw"));
  }

  if (web_server.hasArg("knet")) {
    ResetFactoryDefaultPreference.keepNetwork(isFormItemChecked("knet"));
  }

  if (web_server.hasArg("kntp")) {
    ResetFactoryDefaultPreference.keepNTP(isFormItemChecked("kntp"));
  }

  if (web_server.hasArg("klog")) {
    ResetFactoryDefaultPreference.keepLogSettings(isFormItemChecked("klog"));
  }
  String error;
  bool   performReset = false;
  bool   savePref     = false;

  if (web_server.hasArg(F("savepref"))) {
    // User choose a pre-defined config and wants to save it as the new default.
    savePref = true;
  }

  if (web_server.hasArg(F("performfactoryreset"))) {
    // User confirmed to really perform the reset.
    performReset = true;
    savePref     = true;
  } else {
    error = F("no reset");
  }

  if (savePref) {
    applyFactoryDefaultPref();
    error = SaveSettings();
  }

  if (error.length() == 0) {
    error = F("ok");
  }

  stream_last_json_object_value(F("status"), error);
  addHtml("}");
  TXBuffer.endStream();

  if (performReset) {
    ResetFactory();
  }
}

#endif // WEBSERVER_NEW_UI

#endif // ifdef WEBSERVER_FACTORY_RESET


#include "../WebServer/Favicon.h"

#include "../WebServer/WebServer.h"
#include "../WebServer/404.h"

#include "../Globals/RamTracker.h"


#include "../Static/WebStaticData.h"

void handle_favicon() {
  #ifdef WEBSERVER_FAVICON
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("handle_favicon"));
  #endif
  web_server.send_P(200, PSTR("image/x-icon"), favicon_8b_ico, favicon_8b_ico_len);
  #else // ifdef WEBSERVER_FAVICON
  handleNotFound();
  #endif // ifdef WEBSERVER_FAVICON
}

#include "../WebServer/FileList.h"

#include "../WebServer/WebServer.h"
#include "../WebServer/HTML_wrappers.h"
#include "../WebServer/AccessControl.h"
#include "../WebServer/Markup.h"
#include "../WebServer/Markup_Buttons.h"

#include "../ESPEasyCore/ESPEasyRules.h"

#include "../Helpers/ESPEasy_Storage.h"



#ifdef USES_C016
#include "../Globals/C016_ControllerCache.h"
#endif


#ifdef WEBSERVER_NEW_UI

// ********************************************************************************
// Web Interface file list
// ********************************************************************************
void handle_filelist_json() {
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("handle_filelist"));
  #endif

  if (!clientIPallowed()) { return; }
  navMenuIndex = MENU_INDEX_TOOLS;
  TXBuffer.startJsonStream();

  String fdelete = web_server.arg(F("delete"));

  if (tryDeleteFile(fdelete)) {
    # if defined(ESP32)

    // flashCount();
    # endif // if defined(ESP32)
    # if defined(ESP8266)
    checkRuleSets();
    # endif // if defined(ESP8266)
  }

  const int pageSize = 25;
  int startIdx       = 0;

  String fstart = web_server.arg(F("start"));

  if (fstart.length() > 0)
  {
    startIdx = atoi(fstart.c_str());
  }
  int endIdx = startIdx + pageSize - 1;

  addHtml("[{");
  bool firstentry = true;
  # if defined(ESP32)
  File root  = ESPEASY_FS.open("/");
  File file  = root.openNextFile();
  int  count = -1;

  while (file and count < endIdx)
  {
    if (!file.isDirectory()) {
      ++count;

      if (count >= startIdx)
      {
        if (firstentry) {
          firstentry = false;
        } else {
          addHtml(",{");
        }
        stream_next_json_object_value(F("fileName"), String(file.name()));
        stream_next_json_object_value(F("index"),    String(startIdx));
        stream_last_json_object_value(F("size"), String(file.size()));
      }
    }
    file = root.openNextFile();
  }
  # endif // if defined(ESP32)
  # if defined(ESP8266)
  fs::Dir dir = ESPEASY_FS.openDir("");

  int count = -1;

  while (dir.next())
  {
    ++count;

    if (count < startIdx)
    {
      continue;
    }

    if (firstentry) {
      firstentry = false;
    } else {
      addHtml(",{");
    }

    stream_next_json_object_value(F("fileName"), String(dir.fileName()));

    fs::File f = dir.openFile("r");

    if (f) {
      stream_next_json_object_value(F("size"), String(f.size()));
      f.close();
    }

    stream_last_json_object_value(F("index"), String(startIdx));

    if (count >= endIdx)
    {
      break;
    }
  }

  if (firstentry) {
    addHtml("}");
  }

  # endif // if defined(ESP8266)
  addHtml("]");
  TXBuffer.endStream();
}

#endif // WEBSERVER_NEW_UI

#ifdef WEBSERVER_FILELIST
void handle_filelist() {
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("handle_filelist"));
  #endif

  if (!clientIPallowed()) { return; }
  navMenuIndex = MENU_INDEX_TOOLS;
  TXBuffer.startStream();
  sendHeadandTail_stdtemplate();

  String fdelete = web_server.arg(F("delete"));

  if (tryDeleteFile(fdelete))
  {
    checkRuleSets();
  }
  # ifdef USES_C016

  if (web_server.hasArg(F("delcache"))) {
    while (C016_deleteOldestCacheBlock()) {
      delay(1);
    }

    while (GarbageCollection()) {
      delay(1);
    }
  }
  # endif // ifdef USES_C016
  const int pageSize = 25;
  int startIdx       = 0;
  String fstart      = web_server.arg(F("start"));

  if (fstart.length() > 0)
  {
    startIdx = atoi(fstart.c_str());
  }
  int endIdx = startIdx + pageSize - 1;
  html_table_class_multirow();
  html_table_header("",        50);
  html_table_header(F("Filename"));
  html_table_header(F("Size"), 80);
  int count = -1;

  bool moreFilesPresent  = false;
  bool cacheFilesPresent = false;

# if defined(ESP8266)

  fs::Dir dir = ESPEASY_FS.openDir("");

  while (dir.next() && count < endIdx)
  {
    ++count;

    if (count >= startIdx)
    {
      int filesize = -1;
      fs::File f   = dir.openFile("r");

      if (f) {
        filesize = f.size();
      }

      if (!cacheFilesPresent && (getCacheFileCountFromFilename(dir.fileName()) != -1))
      {
        cacheFilesPresent = true;
      }
      handle_filelist_add_file(dir.fileName(), filesize, startIdx);
    }
  }
  moreFilesPresent = dir.next();
# endif // if defined(ESP8266)
# if defined(ESP32)
  File root = ESPEASY_FS.open("/");
  File file = root.openNextFile();

  while (file && count < endIdx)
  {
    if (!file.isDirectory()) {
      ++count;

      if (count >= startIdx)
      {
        if (!cacheFilesPresent && (getCacheFileCountFromFilename(file.name()) != -1))
        {
          cacheFilesPresent = true;
        }
        handle_filelist_add_file(file.name(), file.size(), startIdx);
      }
    }
    file = root.openNextFile();
  }
  moreFilesPresent = file;
# endif // if defined(ESP32)

  int start_prev = -1;

  if (startIdx > 0)
  {
    start_prev = startIdx < pageSize ? 0 : startIdx - pageSize;
  }
  int start_next = -1;

  if ((count >= endIdx) && moreFilesPresent) {
    start_next = endIdx + 1;
  }
  handle_filelist_buttons(start_prev, start_next, cacheFilesPresent);
}

void handle_filelist_add_file(const String& filename, int filesize, int startIdx) {
  html_TR_TD();

  if ((filename != F(FILE_CONFIG)) && (filename != F(FILE_SECURITY)) && (filename != F(FILE_NOTIFICATION)))
  {
    html_add_button_prefix();
    addHtml(F("filelist?delete="));
    addHtml(filename);

    if (startIdx > 0)
    {
      addHtml(F("&start="));
      addHtml(String(startIdx));
    }
    addHtml(F("'>Del</a>"));
  }
  {
    String html;
    html.reserve(30 + 2 * filename.length());

    html += F("<TD><a href=\"");
    html += filename;
    html += "\">";
    html += filename;
    html += F("</a><TD>");

    if (filesize >= 0) {
      html += String(filesize);
    }
    addHtml(html);
  }
}

void handle_filelist_buttons(int start_prev, int start_next, bool cacheFilesPresent) {
  html_end_table();
  html_end_form();
  html_BR();
  addButton(F("/upload"), F("Upload"));

  if (start_prev >= 0)
  {
    html_add_button_prefix();
    String html;
    html.reserve(36);
    html += F("/filelist?start=");
    html += start_prev;
    html += F("'>Previous</a>");
    addHtml(html);
  }

  if (start_next >= 0)
  {
    html_add_button_prefix();
    String html;
    html.reserve(36);

    html += F("/filelist?start=");
    html += start_next;
    html += F("'>Next</a>");
    addHtml(html);
  }

  if (cacheFilesPresent) {
    html_add_button_prefix(F("red"), true);
    addHtml(F("filelist?delcache'>Delete Cache Files</a>"));
  }
  addHtml(F("<BR><BR>"));
  sendHeadandTail_stdtemplate(true);
  TXBuffer.endStream();
}

#endif // ifdef WEBSERVER_FILELIST

// ********************************************************************************
// Web Interface SD card file and directory list
// ********************************************************************************
#ifdef FEATURE_SD
void handle_SDfilelist() {
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("handle_SDfilelist"));
  #endif

  if (!clientIPallowed()) { return; }
  navMenuIndex = MENU_INDEX_TOOLS;
  TXBuffer.startStream();
  sendHeadandTail_stdtemplate();


  String fdelete       = "";
  String ddelete       = "";
  String change_to_dir = "";
  String current_dir   = "";
  String parent_dir    = "";

  for (uint8_t i = 0; i < web_server.args(); i++) {
    if (web_server.argName(i) == F("delete"))
    {
      fdelete = web_server.arg(i);
    }

    if (web_server.argName(i) == F("deletedir"))
    {
      ddelete = web_server.arg(i);
    }

    if (web_server.argName(i) == F("chgto"))
    {
      change_to_dir = web_server.arg(i);
    }
  }

  if (fdelete.length() > 0)
  {
    SD.remove((char *)fdelete.c_str());
  }

  if (ddelete.length() > 0)
  {
    SD.rmdir((char *)ddelete.c_str());
  }

  if (change_to_dir.length() > 0)
  {
    current_dir = change_to_dir;
  }
  else
  {
    current_dir = "/";
  }

  File root = SD.open(current_dir.c_str());
  root.rewindDirectory();
  File entry = root.openNextFile();
  parent_dir = current_dir;

  if (!current_dir.equals("/"))
  {
    /* calculate the position to remove
       /
       / current_dir = /dir1/dir2/   =>   parent_dir = /dir1/
       /                     ^ position to remove, second last index of "/" + 1
       /
       / current_dir = /dir1/   =>   parent_dir = /
       /                ^ position to remove, second last index of "/" + 1
     */
    parent_dir.remove(parent_dir.lastIndexOf("/", parent_dir.lastIndexOf("/") - 1) + 1);
  }


  String subheader = "SD Card: " + current_dir;
  addFormSubHeader(subheader);
  html_BR();
  html_table_class_multirow();
  html_table_header("", 50);
  html_table_header("Name");
  html_table_header("Size");
  html_TR_TD();
  {
    String html;
    html.reserve(50 + parent_dir.length());
    html += F("<TD><a href=\"SDfilelist?chgto=");
    html += parent_dir;
    html += F("\">..");
    html += F("</a><TD>");
    addHtml(html);
  }

  while (entry)
  {
    html_TR_TD();
    size_t entrynameLength = strlen(entry.name());
    if (entry.isDirectory())
    {
      char SDcardChildDir[80];

      // take a look in the directory for entries
      String child_dir = current_dir + entry.name();
      child_dir.toCharArray(SDcardChildDir, child_dir.length() + 1);
      File child         = SD.open(SDcardChildDir);
      File dir_has_entry = child.openNextFile();

      // when the directory is empty, display the button to delete them
      if (!dir_has_entry)
      {
        addHtml(F("<a class='button link' onclick=\"return confirm('Delete this directory?')\" href=\"SDfilelist?deletedir="));
        String html;
        html.reserve(20 + 2 * current_dir.length() + entrynameLength);
        html += current_dir;
        html += entry.name();
        html += '/';
        html += F("&chgto=");
        html += current_dir;
        html += F("\">Del</a>");
        addHtml(html);
      }
      {
        String html;
        html.reserve(48 + current_dir.length() + 2 * entrynameLength);

        html += F("<TD><a href=\"SDfilelist?chgto=");
        html += current_dir;
        html += entry.name();
        html += '/';
        html += "\">";
        html += entry.name();
        html += F("</a><TD>");
        html += F("dir");
        addHtml(html);
      }
      dir_has_entry.close();
    }
    else
    {

      if ((entry.name() != String(F(FILE_CONFIG)).c_str()) && (entry.name() != String(F(FILE_SECURITY)).c_str()))
      {
        addHtml(F("<a class='button link' onclick=\"return confirm('Delete this file?')\" href=\"SDfilelist?delete="));
        String html;
        html.reserve(20 + 2 * current_dir.length() + entrynameLength);

        html += current_dir;
        html += entry.name();
        html += F("&chgto=");
        html += current_dir;
        html += F("\">Del</a>");
        addHtml(html);
      }
      {
        String html;
        html.reserve(48 + current_dir.length() + 2 * entrynameLength);
        html += F("<TD><a href=\"");
        html += current_dir;
        html += entry.name();
        html += "\">";
        html += entry.name();
        html += F("</a><TD>");
        html += entry.size();
        addHtml(html);
      }
    }
    entry.close();
    entry = root.openNextFile();
  }
  root.close();
  html_end_table();
  html_end_form();

  // addHtml(F("<BR><a class='button link' href=\"/upload\">Upload</a>"));
  sendHeadandTail_stdtemplate(true);
  TXBuffer.endStream();
}

#endif // ifdef FEATURE_SD

#include "../WebServer/HardwarePage.h"

#include "../WebServer/WebServer.h"
#include "../WebServer/HTML_wrappers.h"
#include "../WebServer/Markup.h"
#include "../WebServer/Markup_Buttons.h"
#include "../WebServer/Markup_Forms.h"

#include "../DataStructs/DeviceStruct.h"

#include "../Globals/Settings.h"

#include "../Helpers/ESPEasy_Storage.h"
#include "../Helpers/Hardware.h"
#include "../Helpers/StringConverter.h"
#include "../Helpers/StringGenerator_GPIO.h"

#ifdef WEBSERVER_HARDWARE

// ********************************************************************************
// Web Interface hardware page
// ********************************************************************************
void handle_hardware() {
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("handle_hardware"));
  #endif

  if (!isLoggedIn()) { return; }
  navMenuIndex = MENU_INDEX_HARDWARE;
  TXBuffer.startStream();
  sendHeadandTail_stdtemplate(_HEAD);

  if (isFormItem(F("psda")))
  {
    Settings.Pin_status_led           = getFormItemInt(F("pled"));
    Settings.Pin_status_led_Inversed  = isFormItemChecked(F("pledi"));
    Settings.Pin_Reset                = getFormItemInt(F("pres"));
    Settings.Pin_i2c_sda              = getFormItemInt(F("psda"));
    Settings.Pin_i2c_scl              = getFormItemInt(F("pscl"));
    Settings.I2C_clockSpeed           = getFormItemInt(F("pi2csp"), DEFAULT_I2C_CLOCK_SPEED);
    Settings.I2C_clockSpeed_Slow      = getFormItemInt(F("pi2cspslow"), DEFAULT_I2C_CLOCK_SPEED_SLOW);
#ifdef FEATURE_I2CMULTIPLEXER
    Settings.I2C_Multiplexer_Type     = getFormItemInt(F("pi2cmuxtype"));
    if (Settings.I2C_Multiplexer_Type != I2C_MULTIPLEXER_NONE) {
      Settings.I2C_Multiplexer_Addr   = getFormItemInt(F("pi2cmuxaddr"));
    } else {
      Settings.I2C_Multiplexer_Addr   = -1;
    }
    Settings.I2C_Multiplexer_ResetPin = getFormItemInt(F("pi2cmuxreset"));
#endif
    #ifdef ESP32
      Settings.InitSPI                = getFormItemInt(F("initspi"), 0);
    #else //for ESP8266 we keep the old UI
      Settings.InitSPI                = isFormItemChecked(F("initspi")); // SPI Init
    #endif
    Settings.Pin_sd_cs                = getFormItemInt(F("sd"));
#ifdef HAS_ETHERNET
    Settings.ETH_Phy_Addr             = getFormItemInt(F("ethphy"));
    Settings.ETH_Pin_mdc              = getFormItemInt(F("ethmdc"));
    Settings.ETH_Pin_mdio             = getFormItemInt(F("ethmdio"));
    Settings.ETH_Pin_power            = getFormItemInt(F("ethpower"));
    Settings.ETH_Phy_Type             = static_cast<EthPhyType_t>(getFormItemInt(F("ethtype")));
    Settings.ETH_Clock_Mode           = static_cast<EthClockMode_t>(getFormItemInt(F("ethclock")));
    Settings.NetworkMedium            = static_cast<NetworkMedium_t>(getFormItemInt(F("ethwifi")));
#endif
    int gpio = 0;

    while (gpio <= MAX_GPIO) {
      if (Settings.UseSerial && ((gpio == 1) || (gpio == 3))) {
        // do not add the pin state select for these pins.
      } else {
        int  pinnr = -1;
        bool input, output, warning;

        if (getGpioInfo(gpio, pinnr, input, output, warning)) {
          String int_pinlabel = "p";
          int_pinlabel       += gpio;
          Settings.setPinBootState(gpio, static_cast<PinBootState>(getFormItemInt(int_pinlabel)));
        }
      }
      ++gpio;
    }
    String error = SaveSettings();
    addHtmlError(error);
    if (error.length() == 0) {
      // Apply I2C settings.
      initI2C();
    }
  }

  addHtml(F("<form  method='post'>"));
  html_table_class_normal();
  addFormHeader(F("Hardware Settings"), F("ESPEasy#Hardware_page"), F("Hardware/Hardware.html"));

  addFormSubHeader(F("Wifi Status LED"));
  addFormPinSelect(formatGpioName_output("LED"), "pled", Settings.Pin_status_led);
  addFormCheckBox(F("Inversed LED"), F("pledi"), Settings.Pin_status_led_Inversed);
  addFormNote(F("Use &rsquo;GPIO-2 (D4)&rsquo; with &rsquo;Inversed&rsquo; checked for onboard LED"));

  addFormSubHeader(F("Reset Pin"));
  addFormPinSelect(formatGpioName_input(F("Switch")), "pres", Settings.Pin_Reset);
  addFormNote(F("Press about 10s for factory reset"));

  addFormSubHeader(F("I2C Interface"));
  addFormPinSelectI2C(formatGpioName_bidirectional("SDA"), F("psda"), Settings.Pin_i2c_sda);
  addFormPinSelectI2C(formatGpioName_output("SCL"),        F("pscl"), Settings.Pin_i2c_scl);
  addFormNumericBox(F("Clock Speed"), F("pi2csp"), Settings.I2C_clockSpeed, 100, 3400000);
  addUnit(F("Hz"));
  addFormNote(F("Use 100 kHz for old I2C devices, 400 kHz is max for most."));
  addFormNumericBox(F("Slow device Clock Speed"), F("pi2cspslow"), Settings.I2C_clockSpeed_Slow, 100, 3400000);
  addUnit(F("Hz"));
#ifdef FEATURE_I2CMULTIPLEXER
  addFormSubHeader(F("I2C Multiplexer"));
  // Select the type of multiplexer to use
  {
    String i2c_muxtype_options[5];
    int    i2c_muxtype_choices[5];
    i2c_muxtype_options[0] = F("- None -");
    i2c_muxtype_choices[0] = -1;
    i2c_muxtype_options[1] = F("TCA9548a - 8 channel");
    i2c_muxtype_choices[1] = I2C_MULTIPLEXER_TCA9548A;
    i2c_muxtype_options[2] = F("TCA9546a - 4 channel");
    i2c_muxtype_choices[2] = I2C_MULTIPLEXER_TCA9546A;
    i2c_muxtype_options[3] = F("TCA9543a - 2 channel");
    i2c_muxtype_choices[3] = I2C_MULTIPLEXER_TCA9543A;
    i2c_muxtype_options[4] = F("PCA9540 - 2 channel (experimental)");
    i2c_muxtype_choices[4] = I2C_MULTIPLEXER_PCA9540;
    addFormSelector(F("I2C Multiplexer type"), F("pi2cmuxtype"), 5, i2c_muxtype_options, i2c_muxtype_choices, Settings.I2C_Multiplexer_Type);
  }
  // Select the I2C address for a port multiplexer
  {
    String  i2c_mux_options[9];
    int     i2c_mux_choices[9];
    uint8_t mux_opt = 0;
    i2c_mux_options[mux_opt] = F("- None -");
    i2c_mux_choices[mux_opt] = I2C_MULTIPLEXER_NONE;
    for (int8_t x = 0; x < 8; x++) {
      mux_opt++;
      i2c_mux_options[mux_opt] = formatToHex_decimal(0x70 + x);
      if (x == 0) { // PCA9540 has a fixed address 0f 0x70
        i2c_mux_options[mux_opt] += F(" [TCA9543a/6a/8a, PCA9540]");
      } else if (x < 4) {
        i2c_mux_options[mux_opt] += F(" [TCA9543a/6a/8a]");
      } else {
        i2c_mux_options[mux_opt] += F(" [TCA9546a/8a]");
      }
      i2c_mux_choices[mux_opt] = 0x70 + x;
    }
    addFormSelector(F("I2C Multiplexer address"), F("pi2cmuxaddr"), mux_opt + 1, i2c_mux_options, i2c_mux_choices, Settings.I2C_Multiplexer_Addr);
  }
  addFormPinSelect(formatGpioName_output_optional("Reset"), F("pi2cmuxreset"), Settings.I2C_Multiplexer_ResetPin);
  addFormNote(F("Will be pulled low to force a reset. Reset is not available on PCA9540."));
#endif

  // SPI Init
  addFormSubHeader(F("SPI Interface"));
  #ifdef ESP32
    String spi_options[3] = { F("Disabled"), F("VSPI: CLK=GPIO-18, MISO=GPIO-19, MOSI=GPIO-23"), F("HSPI: CLK=GPIO-14, MISO=GPIO-12, MOSI=GPIO-13")};
    addFormSelector(F("Init SPI"), F("initspi"), 3, spi_options, NULL, Settings.InitSPI);
    addFormNote(F("Changing SPI settings requires to manualy restart"));
  #else //for ESP8266 we keep the existing UI
  addFormCheckBox(F("Init SPI"), F("initspi"), Settings.InitSPI>0);
  addFormNote(F("CLK=GPIO-14 (D5), MISO=GPIO-12 (D6), MOSI=GPIO-13 (D7)"));
  #endif
  addFormNote(F("Chip Select (CS) config must be done in the plugin"));
  
#ifdef FEATURE_SD
  addFormPinSelect(formatGpioName_output("SD Card CS"), "sd", Settings.Pin_sd_cs);
#endif // ifdef FEATURE_SD
  
#ifdef HAS_ETHERNET
  addFormSubHeader(F("Ethernet"));
  addRowLabel_tr_id(F("Preferred network medium"), "ethwifi");
  String ethWifiOptions[2] = {
    toString(NetworkMedium_t::WIFI), 
    toString(NetworkMedium_t::Ethernet) 
    };
  addSelector("ethwifi", 2, ethWifiOptions, NULL, NULL, static_cast<int>(Settings.NetworkMedium), false, true);
  addFormNote(F("Change Switch between WiFi and Ethernet requires reboot to activate"));
  addRowLabel_tr_id(F("Ethernet PHY type"), "ethtype");
  String ethPhyTypes[2] = { 
    toString(EthPhyType_t::LAN8710), 
    toString(EthPhyType_t::TLK110) };
  addSelector("ethtype", 2, ethPhyTypes, NULL, NULL, static_cast<int>(Settings.ETH_Phy_Type), false, true);
  addFormNumericBox(F("Ethernet PHY Address"), "ethphy", Settings.ETH_Phy_Addr, 0, 255);
  addFormNote(F("I&sup2;C-address of Ethernet PHY (0 or 1 for LAN8720, 31 for TLK110)"));
  addFormPinSelect(formatGpioName_output("Ethernet MDC pin"), "ethmdc", Settings.ETH_Pin_mdc);
  addFormPinSelect(formatGpioName_input("Ethernet MIO pin"), "ethmdio", Settings.ETH_Pin_mdio);
  addFormPinSelect(formatGpioName_output("Ethernet Power pin"), "ethpower", Settings.ETH_Pin_power);
  addRowLabel_tr_id(F("Ethernet Clock"), "ethclock");
  String ethClockOptions[4] = { 
    toString(EthClockMode_t::Ext_crystal_osc),
    toString(EthClockMode_t::Int_50MHz_GPIO_0),
    toString(EthClockMode_t::Int_50MHz_GPIO_16),
    toString(EthClockMode_t::Int_50MHz_GPIO_17_inv)
     };
  addSelector("ethclock", 4, ethClockOptions, NULL, NULL, static_cast<int>(Settings.ETH_Clock_Mode), false, true);
#endif // ifdef HAS_ETHERNET

  addFormSubHeader(F("GPIO boot states"));

  for (int gpio = 0; gpio <= MAX_GPIO; ++gpio) {
    addFormPinStateSelect(gpio, static_cast<int>(Settings.getPinBootState(gpio)));
  }
  addFormSeparator(2);

  html_TR_TD();
  html_TD();
  addSubmitButton();
  html_TR_TD();
  html_end_table();
  html_end_form();

  sendHeadandTail_stdtemplate(_TAIL);
  TXBuffer.endStream();
}

#endif // ifdef WEBSERVER_HARDWARE

#include "../WebServer/HTML_wrappers.h"

#include "../Static/WebStaticData.h"

#include "../WebServer/Markup.h"

#include "../Helpers/StringConverter.h"


// ********************************************************************************
// HTML string re-use to keep the executable smaller
// Flash strings are not checked for duplication.
// ********************************************************************************
void wrap_html_tag(const String& tag, const String& text) {
  String html;

  html.reserve(6 + (2 * tag.length()) + text.length());

  html += '<';
  html += tag;
  html += '>';
  html += text;
  html += "</";
  html += tag;
  html += '>';
  addHtml(html);
}

void html_B(const String& text) {
  wrap_html_tag("b", text);
}

void html_I(const String& text) {
  wrap_html_tag("i", text);
}

void html_U(const String& text) {
  wrap_html_tag("u", text);
}

void html_TR_TD_highlight() {
  addHtml(F("<TR class=\"highlight\">"));
  html_TD();
}

void html_TR_TD() {
  html_TR();
  html_TD();
}

void html_BR() {
  addHtml(F("<BR>"));
}

void html_TR() {
  addHtml(F("<TR>"));
}

void html_TR_TD_height(int height) {
  html_TR();

  String html;

  html.reserve(20);

  html += F("<TD HEIGHT=\"");
  html += height;
  html += "\">";
  addHtml(html);
}

void html_TD() {
  html_TD(1);
}

void html_TD(int td_cnt) {
  for (int i = 0; i < td_cnt; ++i) {
    addHtml(F("<TD>"));
  }
}

int copyTextCounter = 0;

void html_reset_copyTextCounter() {
  copyTextCounter = 0;
}

void html_copyText_TD() {
  ++copyTextCounter;

  String html;

  html.reserve(24);

  html += F("<TD id='copyText_");
  html += copyTextCounter;
  html += "'>";
  addHtml(html);
}

// Add some recognizable token to show which parts will be copied.
void html_copyText_marker() {
  addHtml(F("&#x022C4;")); //   &diam; &diamond; &Diamond; &#x022C4; &#8900;
}

void html_add_estimate_symbol() {
  addHtml(F(" &#8793; ")); //   &#8793;  &#x2259;  &wedgeq;
}

void html_table_class_normal() {
  html_table(F("normal"));
}

void html_table_class_multirow() {
  html_table(F("multirow"), true);
}

void html_table_class_multirow_noborder() {
  html_table(F("multirow"), false);
}

void html_table(const String& tableclass) {
  html_table(tableclass, false);
}

void html_table(const String& tableclass, bool boxed) {
  addHtml(F("<table "));
  addHtmlAttribute(F("class"), tableclass);

  if (boxed) {
    addHtmlAttribute(F("border"), F("1px"));
    addHtmlAttribute(F("frame"),  F("box"));
    addHtmlAttribute(F("rules"),  F("all"));
  }
  addHtml('>');
}

void html_table_header(const String& label) {
  html_table_header(label, 0);
}

void html_table_header(const String& label, int width) {
  html_table_header(label, F(""), F(""), width);
}

void html_table_header(const String& label, const String& helpButton, int width) {
  html_table_header(label, helpButton, F(""), width);
}

void html_table_header(const String& label, const String& helpButton, const String& rtdHelpButton, int width) {
  String html;

  html.reserve(25 + label.length());

  html += F("<TH");

  if (width > 0) {
    html += F(" style='width:");
    html += String(width);
    html += F("px;'");
  }
  html += '>';
  html += label;
  addHtml(html);

  if (helpButton.length() > 0) {
    addHelpButton(helpButton);
  }

  if (rtdHelpButton.length() > 0) {
    addRTDHelpButton(rtdHelpButton);
  }
  addHtml(F("</TH>"));
}

void html_end_table() {
  addHtml(F("</table>"));
}

void html_end_form() {
  addHtml(F("</form>"));
}

void html_add_button_prefix() {
  html_add_button_prefix("", true);
}

void html_add_button_prefix(const String& classes, bool enabled) {
  addHtml(F(" <a class='button link"));

  if (classes.length() > 0) {
    String html;
    html.reserve(classes.length() + 1);
    html += ' ';
    html += classes;
    addHtml(html);
  }

  if (!enabled) {
    addDisabled();
  }
  addHtml("'");

  if (!enabled) {
    addDisabled();
  }
  addHtml(F(" href='"));
}

void html_add_wide_button_prefix() {
  html_add_wide_button_prefix("", true);
}

void html_add_wide_button_prefix(const String& classes, bool enabled) {
  String wide_classes;

  wide_classes.reserve(classes.length() + 5);
  wide_classes  = F("wide ");
  wide_classes += classes;
  html_add_button_prefix(wide_classes, enabled);
}

void html_add_form() {
  addHtml(F("<form name='frmselect' method='post'>"));
}

void html_add_autosubmit_form() {
  addHtml(F("<script><!--\n"
            "function dept_onchange(frmselect) {frmselect.submit();}"
            "\n//--></script>"));
}

void html_add_script(const String& script, bool defer) {
  html_add_script(defer);
  addHtml(script);
  html_add_script_end();
}

void html_add_script(bool defer) {
  addHtml(F("<script"));

  if (defer) {
    addHtml(F(" defer"));
  }
  addHtml(F(" type='text/JavaScript'>"));
}

void html_add_script_end() {
  addHtml(F("</script>"));
}

// if there is an error-string, add it to the html code with correct formatting
void addHtmlError(const String& error) {
  if (error.length() > 0)
  {
    String html;
    html.reserve(20);
    html += F("<div class=\"");

    if (error.startsWith(F("Warn"))) {
      html += F("warning");
    } else {
      html += F("alert");
    }
    addHtml(html);
    addHtml(F("\"><span class=\"closebtn\" onclick=\"this.parentElement.style.display='none';\">&times;</span>"));
    addHtml(error);
    addHtml(F("</div>"));
  }
  else
  {}
}

void addHtml(char html) {
  TXBuffer += html;
}

void addHtml(const String& html) {
  TXBuffer += html;
}

void addEncodedHtml(const String& html) {
  // FIXME TD-er: What about the function htmlStrongEscape ??
  String copy(html);

  htmlEscape(copy);
  addHtml(copy);
}

void addHtmlAttribute(const String& label, int value) {
  addHtml(' ');
  addHtml(label);
  addHtml('=');
  addHtml(String(value));
  addHtml(' ');
}

void addHtmlAttribute(const String& label, const String& value) {
  addHtml(' ');
  addHtml(label);
  addHtml(F("='"));
  addEncodedHtml(value);
  addHtml(F("' "));
}

void addDisabled() {
  addHtml(F(" disabled"));
}

void addHtmlLink(const String& htmlclass, const String& url, const String& label) {
  addHtml(F(" <a "));
  addHtmlAttribute(F("class"),  htmlclass);
  addHtmlAttribute(F("href"),   url);
  addHtmlAttribute(F("target"), F("_blank"));
  addHtml('>');
  addHtml(label);
  addHtml(F("</a>"));
}

void addEnabled(boolean enabled)
{
  String html;

  html.reserve(40);
  html += F("<span class='enabled ");

  if (enabled) {
    html += F("on'>&#10004;");
  }
  else {
    html += F("off'>&#10060;");
  }
  html += F("</span>");
  addHtml(html);
}

#include "../WebServer/I2C_Scanner.h"

#include "../WebServer/WebServer.h"
#include "../WebServer/AccessControl.h"
#include "../WebServer/HTML_wrappers.h"

#include "../Globals/Settings.h"

#include "../Helpers/Hardware.h"
#include "../Helpers/StringConverter.h"


#ifdef WEBSERVER_I2C_SCANNER

#ifdef WEBSERVER_NEW_UI

// ********************************************************************************
// Web Interface I2C scanner
// ********************************************************************************

int scanI2CbusForDevices_json( // Utility function for scanning the I2C bus for valid devices, with JSON output
        int8_t muxAddr
      , int8_t channel
      , int nDevices
#ifdef FEATURE_I2CMULTIPLEXER
      , i2c_addresses_t &excludeDevices
#endif
) {
  byte error, address;

  for (address = 1; address <= 127; address++)
  {
#ifdef FEATURE_I2CMULTIPLEXER
    bool skipCheck = false;
    if (channel != -1 && excludeDevices.size() > address) {
      skipCheck = excludeDevices[address];
    }
    if (!skipCheck) { // Ignore I2C multiplexer and addresses to exclude when scanning its channels
#endif
      Wire.beginTransmission(address);
      error = Wire.endTransmission();
      delay(1);

      if ((error == 0) || (error == 4))
      {
        json_open();
        json_prop(F("addr"), String(formatToHex(address)));
#ifdef FEATURE_I2CMULTIPLEXER
        if (muxAddr != -1) {
          if (channel == -1){
            json_prop(F("I2Cbus"), F("Standard I2C bus"));
            excludeDevices[address] = true;
          } else {
            String i2cChannel = F("Multiplexer channel ");
            i2cChannel += String(channel);
            json_prop(F("I2Cbus"), i2cChannel);
          }
        }
#endif
        json_number(F("status"), String(error));

        if (error == 4) {
          json_prop(F("error"), F("Unknown error at address "));
        } else {
          String description = getKnownI2Cdevice(address);

          if (description.length() > 0) {
            json_open(true, F("known devices"));
            int pos = 0;

            while (pos >= 0) {
              int newpos = description.indexOf(',', pos);

              if (pos != 0) {
                addHtml(",");
              }

              if (newpos == -1) {
                json_quote_val(description.substring(pos));
              } else {
                json_quote_val(description.substring(pos, newpos));
              }
              pos = newpos;

              if (newpos != -1) {
                ++pos;
              }
            }
            json_close(true);
          }
          nDevices++;
        }
        json_close();
        addHtml("\n");
      }
#ifdef FEATURE_I2CMULTIPLEXER
    }
#endif
  }
  return nDevices;
}

void handle_i2cscanner_json() {
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("handle_i2cscanner"));
  #endif

  if (!isLoggedIn()) { return; }
  navMenuIndex = MENU_INDEX_TOOLS;
  TXBuffer.startJsonStream();
  json_init();
  json_open(true);

  int  nDevices = 0;

  I2CSelectClockSpeed(true);    // Always scan in low speed to also find old/slow devices
#ifdef FEATURE_I2CMULTIPLEXER
  i2c_addresses_t mainBusDevices;
  mainBusDevices.resize(128);
  for (int i = 0; i < 128; i++) {
    mainBusDevices[i] = false;
  }
  nDevices = scanI2CbusForDevices_json(Settings.I2C_Multiplexer_Addr, -1, nDevices, mainBusDevices); // Channel -1 = standard I2C bus
#else
  nDevices = scanI2CbusForDevices_json(-1, -1, nDevices); // Standard scan
#endif

#ifdef FEATURE_I2CMULTIPLEXER
  if (isI2CMultiplexerEnabled()) {
    uint8_t mux_max = I2CMultiplexerMaxChannels();
    for (int8_t channel = 0; channel < mux_max; channel++) {
      I2CMultiplexerSelect(channel);
      nDevices += scanI2CbusForDevices_json(Settings.I2C_Multiplexer_Addr, channel, nDevices, mainBusDevices); // Specific channels
    }
    I2CMultiplexerOff();
  }
#endif
  I2CSelectClockSpeed(false); // Reset bus to standard speed
  
  json_close(true);
  TXBuffer.endStream();
}
#endif // WEBSERVER_NEW_UI


String getKnownI2Cdevice(byte address) {
  String result;
  #ifndef LIMIT_BUILD_SIZE

  switch (address)
  {
    case 0x20:
    case 0x21:
    case 0x22:
    case 0x25:
    case 0x26:
    case 0x27:
      result =  F("PCF8574,MCP23017,LCD");
      break;
    case 0x23:
      result =  F("PCF8574,MCP23017,LCD,BH1750");
      break;
    case 0x24:
      result =  F("PCF8574,MCP23017,LCD,PN532");
      break;
    case 0x29:
      result =  F("TSL2561");
      break;
    case 0x38:
    case 0x3A:
    case 0x3B:
    case 0x3E:
    case 0x3F:
      result =  F("PCF8574A");
      break;
    case 0x39:
      result =  F("PCF8574A,TSL2561,APDS9960");
      break;
    case 0x3C:
    case 0x3D:
      result =  F("PCF8574A,OLED");
      break;
    case 0x40:
      result =  F("SI7021,HTU21D,INA219,PCA9685,HDC1080");
      break;
    case 0x41:
    case 0x42:
    case 0x43:
      result =  F("INA219");
      break;
    case 0x44:
    case 0x45:
      result =  F("SHT30/31/35");
      break;
    case 0x48:
    case 0x4A:
    case 0x4B:
      result =  F("PCF8591,ADS1115,LM75A");
      break;
    case 0x49:
      result =  F("PCF8591,ADS1115,TSL2561,LM75A");
      break;
    case 0x4C:
    case 0x4E:
    case 0x4F:
      result =  F("PCF8591,LM75A");
      break;
    case 0x4D:
      result =  F("PCF8591,MCP3221,LM75A");
      break;
    case 0x58:
      result =  F("SGP30");
      break;
    case 0x5A:
      result =  F("MLX90614,MPR121,CCS811");
      break;
    case 0x5B:
      result =  F("MPR121,CCS811");
      break;
    case 0x5C:
      result =  F("DHT12,AM2320,BH1750,MPR121");
      break;
    case 0x5D:
      result =  F("MPR121");
      break;
    case 0x60:
      result =  F("Adafruit Motorshield v2,SI1145");
      break;
    case 0x70:
      result =  F("Adafruit Motorshield v2 (Catchall),HT16K33,TCA9543a/6a/8a I2C multiplexer,PCA9540 I2C multiplexer");
      break;
    case 0x71:
    case 0x72:
    case 0x73:
      result =  F("HT16K33,TCA9543a/6a/8a I2C multiplexer");
      break;
    case 0x74:
    case 0x75:
      result =  F("HT16K33,TCA9546a/8a I2C multiplexer");
      break;
    case 0x76:
      result =  F("BMP280,BME280,BME680,MS5607,MS5611,HT16K33,TCA9546a/8a I2C multiplexer");
      break;
    case 0x77:
      result =  F("BMP085,BMP180,BMP280,BME280,BME680,MS5607,MS5611,HT16K33,TCA9546a/8a I2C multiplexer");
      break;
    case 0x7f:
      result =  F("Arduino PME");
      break;
  }
  #endif // LIMIT_BUILD_SIZE
  return result;
}

int scanI2CbusForDevices( // Utility function for scanning the I2C bus for valid devices, with HTML table output
        int8_t muxAddr
      , int8_t channel
      , int nDevices
#ifdef FEATURE_I2CMULTIPLEXER
      , i2c_addresses_t &excludeDevices
#endif
) {
  byte error, address;

  for (address = 1; address <= 127; address++)
  {
#ifdef FEATURE_I2CMULTIPLEXER
    bool skipCheck = false;
    if (channel != -1 && excludeDevices.size() > address) {
      skipCheck = excludeDevices[address];
    }
    if (!skipCheck) { // Ignore I2C multiplexer and addresses to exclude when scanning its channels
#endif
      Wire.beginTransmission(address);
      error = Wire.endTransmission();
      delay(1);

      if (error == 0)
      {
        html_TR_TD();
#ifdef FEATURE_I2CMULTIPLEXER
        if (muxAddr != -1) {
          if (channel == -1){
            addHtml(F("Standard I2C bus"));
            excludeDevices[address] = true;
          } else {
            String i2cChannel = F("Multiplexer channel ");
            i2cChannel += String(channel);
            addHtml(i2cChannel);
          }
          html_TD();
        }
#endif
        addHtml(formatToHex(address));
        html_TD();
        String description = getKnownI2Cdevice(address);

        if (description.length() > 0) {
          description.replace(",", "<BR>");
          addHtml(description);
        }
        nDevices++;
      }
      else if (error == 4)
      {
        html_TR_TD();
        addHtml(F("Unknown error at address "));
        addHtml(formatToHex(address));
      }
#ifdef FEATURE_I2CMULTIPLEXER
    }
#endif
  }
  return nDevices;
}

// FIXME TD-er: Query all included plugins for their supported addresses (return name of plugin)
void handle_i2cscanner() {
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("handle_i2cscanner"));
  #endif

  if (!isLoggedIn()) { return; }
  navMenuIndex = MENU_INDEX_TOOLS;
  TXBuffer.startStream();
  sendHeadandTail_stdtemplate(_HEAD);

  html_table_class_multirow();
#ifdef FEATURE_I2CMULTIPLEXER
  if (isI2CMultiplexerEnabled()) {
    html_table_header(F("I2C bus"));
  }
#endif
  html_table_header(F("I2C Addresses in use"));
  html_table_header(F("Supported devices"));

  int  nDevices = 0;
  I2CSelectClockSpeed(true);  // Scan bus using low speed
#ifdef FEATURE_I2CMULTIPLEXER
  i2c_addresses_t mainBusDevices;
  mainBusDevices.resize(128);
  for (int i = 0; i < 128; i++) {
    mainBusDevices[i] = false;
  }
  nDevices = scanI2CbusForDevices(Settings.I2C_Multiplexer_Addr, -1, nDevices, mainBusDevices); // Channel -1 = standard I2C bus
#else
  nDevices = scanI2CbusForDevices(-1, -1, nDevices); // Standard scan
#endif

#ifdef FEATURE_I2CMULTIPLEXER
  if (isI2CMultiplexerEnabled()) {
    uint8_t mux_max = I2CMultiplexerMaxChannels();
    for (int8_t channel = 0; channel < mux_max; channel++) {
      I2CMultiplexerSelect(channel);
      nDevices += scanI2CbusForDevices(Settings.I2C_Multiplexer_Addr, channel, nDevices, mainBusDevices);
    }
    I2CMultiplexerOff();
  }
#endif
  I2CSelectClockSpeed(false);   // By default the bus is in standard speed

  if (nDevices == 0) {
    addHtml(F("<TR>No I2C devices found"));
  }

  html_end_table();
  sendHeadandTail_stdtemplate(_TAIL);
  TXBuffer.endStream();
}
#endif // WEBSERVER_I2C_SCANNER

#include "../WebServer/JSON.h"

#include "../WebServer/WebServer.h"
#include "../WebServer/JSON.h"
#include "../WebServer/Markup_Forms.h"

#include "../Globals/Nodes.h"
#include "../Globals/Device.h"
#include "../Globals/Plugins.h"

#include "../Helpers/ESPEasyStatistics.h"
#include "../Helpers/ESPEasy_Storage.h"
#include "../Helpers/Hardware.h"
#include "../Helpers/Numerical.h"
#include "../Helpers/StringConverter.h"
#include "../Helpers/StringProvider.h"

#include "../../_Plugin_Helper.h"
#include "../../ESPEasy-Globals.h"



// ********************************************************************************
// Web Interface get CSV value from task
// ********************************************************************************
void handle_csvval()
{
  String htmlData;
  htmlData.reserve(25); // Reserve for error message
  const int printHeader = getFormItemInt(F("header"), 1);
  bool printHeaderValid = true;
  if (printHeader != 1 && printHeader != 0)
  {
    htmlData = F("ERROR: Header not valid!\n");
    printHeaderValid = false;
  }

  const taskIndex_t taskNr    = getFormItemInt(F("tasknr"), INVALID_TASK_INDEX);
  const bool taskValid = validTaskIndex(taskNr);
  if (!taskValid)
  {
    htmlData = F("ERROR: TaskNr not valid!\n");
  }

  const int INVALID_VALUE_NUM = INVALID_TASKVAR_INDEX + 1;
  const taskVarIndex_t valNr    = getFormItemInt(F("valnr"), INVALID_VALUE_NUM);
  bool valueNumberValid = true;
  if (valNr != INVALID_VALUE_NUM && !validTaskVarIndex(valNr))
  {
    htmlData = F("ERROR: ValueId not valid!\n");
    valueNumberValid = false;
  }

  TXBuffer.startJsonStream();
  if (taskValid && valueNumberValid && printHeaderValid)
  {
    const deviceIndex_t DeviceIndex = getDeviceIndex_from_TaskIndex(taskNr);

    if (validDeviceIndex(DeviceIndex))
    {
      LoadTaskSettings(taskNr);
      const byte taskValCount = getValueCountForTask(taskNr);
      uint16_t stringReserveSize = (valNr == INVALID_VALUE_NUM ? 1 : taskValCount) * 24;
      htmlData.reserve(stringReserveSize);

      if (printHeader)
      {
        for (byte x = 0; x < taskValCount; x++)
        {
          if (valNr == INVALID_VALUE_NUM || valNr == x)
          {
            htmlData += String(ExtraTaskSettings.TaskDeviceValueNames[x]);
            if (x != taskValCount - 1)
            {
              htmlData += ';';
            }
          }
        }
        htmlData += '\n';
        addHtml(htmlData);
        htmlData = "";
      }

      for (byte x = 0; x < taskValCount; x++)
      {
        if (valNr == INVALID_VALUE_NUM || valNr == x)
        {
          htmlData += formatUserVarNoCheck(taskNr, x);
          if (x != taskValCount - 1)
          {
            htmlData += ';';
          }
        }
      }
      htmlData += '\n';
    }
  }
  addHtml(htmlData);
  TXBuffer.endStream();
}
// ********************************************************************************
// Web Interface JSON page (no password!)
// ********************************************************************************
void handle_json()
{
  const taskIndex_t taskNr    = getFormItemInt(F("tasknr"), INVALID_TASK_INDEX);
  const bool showSpecificTask = validTaskIndex(taskNr);
  bool showSystem             = true;
  bool showWifi               = true;
  #ifdef HAS_ETHERNET
  bool showEthernet           = true;
  #endif
  bool showDataAcquisition    = true;
  bool showTaskDetails        = true;
  bool showNodes              = true;
  {
    String view = web_server.arg("view");

    if (view.length() != 0) {
      if (view == F("sensorupdate")) {
        showSystem          = false;
        showWifi            = false;
        #ifdef HAS_ETHERNET
        showEthernet        = false;
        #endif
        showDataAcquisition = false;
        showTaskDetails     = false;
        showNodes           = false;
      }
    }
  }

  TXBuffer.startJsonStream();

  if (!showSpecificTask)
  {
    addHtml("{");

    if (showSystem) {
      addHtml(F("\"System\":{\n"));
      stream_next_json_object_value(LabelType::BUILD_DESC);
      stream_next_json_object_value(LabelType::GIT_BUILD);
      stream_next_json_object_value(LabelType::SYSTEM_LIBRARIES);
      stream_next_json_object_value(LabelType::PLUGIN_COUNT);
      stream_next_json_object_value(LabelType::PLUGIN_DESCRIPTION);
      stream_next_json_object_value(LabelType::LOCAL_TIME);
      stream_next_json_object_value(LabelType::UNIT_NR);
      stream_next_json_object_value(LabelType::UNIT_NAME);
      stream_next_json_object_value(LabelType::UPTIME);
      stream_next_json_object_value(LabelType::BOOT_TYPE);
      stream_next_json_object_value(LabelType::RESET_REASON);

      if (wdcounter > 0)
      {
        stream_next_json_object_value(LabelType::LOAD_PCT);
        stream_next_json_object_value(LabelType::LOOP_COUNT);
      }
      stream_next_json_object_value(LabelType::CPU_ECO_MODE);

      #ifdef CORE_POST_2_5_0
      stream_next_json_object_value(LabelType::HEAP_MAX_FREE_BLOCK);
      stream_next_json_object_value(LabelType::HEAP_FRAGMENTATION);
      #endif // ifdef CORE_POST_2_5_0
      stream_last_json_object_value(LabelType::FREE_MEM);
      addHtml(F(",\n"));
    }

    if (showWifi) {
      addHtml(F("\"WiFi\":{\n"));
      #if defined(ESP8266)
      stream_next_json_object_value(LabelType::HOST_NAME);
      #endif // if defined(ESP8266)
      #ifdef FEATURE_MDNS
      stream_next_json_object_value(LabelType::M_DNS);
      #endif // ifdef FEATURE_MDNS
      stream_next_json_object_value(LabelType::IP_CONFIG);
      stream_next_json_object_value(LabelType::IP_ADDRESS);
      stream_next_json_object_value(LabelType::IP_SUBNET);
      stream_next_json_object_value(LabelType::GATEWAY);
      stream_next_json_object_value(LabelType::STA_MAC);
      stream_next_json_object_value(LabelType::DNS_1);
      stream_next_json_object_value(LabelType::DNS_2);
      stream_next_json_object_value(LabelType::SSID);
      stream_next_json_object_value(LabelType::BSSID);
      stream_next_json_object_value(LabelType::CHANNEL);
      stream_next_json_object_value(LabelType::CONNECTED_MSEC);
      stream_next_json_object_value(LabelType::LAST_DISCONNECT_REASON);
      stream_next_json_object_value(LabelType::LAST_DISC_REASON_STR);
      stream_next_json_object_value(LabelType::NUMBER_RECONNECTS);
      stream_next_json_object_value(LabelType::FORCE_WIFI_BG);
      stream_next_json_object_value(LabelType::RESTART_WIFI_LOST_CONN);
#ifdef ESP8266
      stream_next_json_object_value(LabelType::FORCE_WIFI_NOSLEEP);
#endif // ifdef ESP8266
#ifdef SUPPORT_ARP
      stream_next_json_object_value(LabelType::PERIODICAL_GRAT_ARP);
#endif // ifdef SUPPORT_ARP
      stream_next_json_object_value(LabelType::CONNECTION_FAIL_THRESH);
      stream_last_json_object_value(LabelType::WIFI_RSSI);
      // TODO: PKR: Add ETH Objects
      addHtml(F(",\n"));
    }

    #ifdef HAS_ETHERNET
    if (showEthernet) {
      addHtml(F("\"Ethernet\":{\n"));
      stream_next_json_object_value(LabelType::ETH_WIFI_MODE);
      stream_next_json_object_value(LabelType::ETH_CONNECTED);
      stream_next_json_object_value(LabelType::ETH_DUPLEX);
      stream_next_json_object_value(LabelType::ETH_SPEED);
      stream_next_json_object_value(LabelType::ETH_STATE);
      stream_last_json_object_value(LabelType::ETH_SPEED_STATE);
      addHtml(F(",\n"));
    }
    #endif

    if (showNodes) {
      bool comma_between = false;

      for (NodesMap::iterator it = Nodes.begin(); it != Nodes.end(); ++it)
      {
        if (it->second.ip[0] != 0)
        {
          if (comma_between) {
            addHtml(",");
          } else {
            comma_between = true;
            addHtml(F("\"nodes\":[\n")); // open json array if >0 nodes
          }

          addHtml("{");
          stream_next_json_object_value(F("nr"), String(it->first));
          stream_next_json_object_value(F("name"),
                                        (it->first != Settings.Unit) ? it->second.nodeName : Settings.Name);

          if (it->second.build) {
            stream_next_json_object_value(F("build"), String(it->second.build));
          }

          if (it->second.nodeType) {
            String platform = getNodeTypeDisplayString(it->second.nodeType);

            if (platform.length() > 0) {
              stream_next_json_object_value(F("platform"), platform);
            }
          }
          stream_next_json_object_value(F("ip"), it->second.ip.toString());
          stream_last_json_object_value(F("age"), String(it->second.age));
        } // if node info exists
      }   // for loop

      if (comma_between) {
        addHtml(F("],\n")); // close array if >0 nodes
      }
    }
  }

  taskIndex_t firstTaskIndex = 0;
  taskIndex_t lastTaskIndex  = TASKS_MAX - 1;

  if (showSpecificTask)
  {
    firstTaskIndex = taskNr - 1;
    lastTaskIndex  = taskNr - 1;
  }
  taskIndex_t lastActiveTaskIndex = 0;

  for (taskIndex_t TaskIndex = firstTaskIndex; TaskIndex <= lastTaskIndex; TaskIndex++) {
    if (validPluginID_fullcheck(Settings.TaskDeviceNumber[TaskIndex])) {
      lastActiveTaskIndex = TaskIndex;
    }
  }

  if (!showSpecificTask) {
    addHtml(F("\"Sensors\":[\n"));
  }

  // Keep track of the lowest reported TTL and use that as refresh interval.
  unsigned long lowest_ttl_json = 60;

  for (taskIndex_t TaskIndex = firstTaskIndex; TaskIndex <= lastActiveTaskIndex && validTaskIndex(TaskIndex); TaskIndex++)
  {
    const deviceIndex_t DeviceIndex = getDeviceIndex_from_TaskIndex(TaskIndex);

    if (validDeviceIndex(DeviceIndex))
    {
      const unsigned long taskInterval = Settings.TaskDeviceTimer[TaskIndex];
      LoadTaskSettings(TaskIndex);
      addHtml(F("{\n"));

      unsigned long ttl_json = 60; // Default value

      // For simplicity, do the optional values first.
      const byte valueCount = getValueCountForTask(TaskIndex);
      if (valueCount != 0) {
        if ((taskInterval > 0) && Settings.TaskDeviceEnabled[TaskIndex]) {
          ttl_json = taskInterval;

          if (ttl_json < lowest_ttl_json) {
            lowest_ttl_json = ttl_json;
          }
        }
        addHtml(F("\"TaskValues\": [\n"));

        for (byte x = 0; x < valueCount; x++)
        {
          addHtml("{");
          stream_next_json_object_value(F("ValueNumber"), String(x + 1));
          stream_next_json_object_value(F("Name"),        String(ExtraTaskSettings.TaskDeviceValueNames[x]));
          stream_next_json_object_value(F("NrDecimals"),  String(ExtraTaskSettings.TaskDeviceValueDecimals[x]));
          stream_last_json_object_value(F("Value"), formatUserVarNoCheck(TaskIndex, x));

          if (x < (valueCount - 1)) {
            addHtml(F(",\n"));
          }
        }
        addHtml(F("],\n"));
      }

      if (showSpecificTask) {
        stream_next_json_object_value(F("TTL"), String(ttl_json * 1000));
      }

      if (showDataAcquisition) {
        addHtml(F("\"DataAcquisition\": [\n"));

        for (controllerIndex_t x = 0; x < CONTROLLER_MAX; x++)
        {
          addHtml("{");
          stream_next_json_object_value(F("Controller"), String(x + 1));
          stream_next_json_object_value(F("IDX"),        String(Settings.TaskDeviceID[x][TaskIndex]));
          stream_last_json_object_value(F("Enabled"), jsonBool(Settings.TaskDeviceSendData[x][TaskIndex]));

          if (x < (CONTROLLER_MAX - 1)) {
            addHtml(F(",\n"));
          }
        }
        addHtml(F("],\n"));
      }

      if (showTaskDetails) {
        stream_next_json_object_value(F("TaskInterval"),     String(taskInterval));
        stream_next_json_object_value(F("Type"),             getPluginNameFromDeviceIndex(DeviceIndex));
        stream_next_json_object_value(F("TaskName"),         String(ExtraTaskSettings.TaskDeviceName));
        stream_next_json_object_value(F("TaskDeviceNumber"), String(Settings.TaskDeviceNumber[TaskIndex]));
#ifdef FEATURE_I2CMULTIPLEXER
        if (Device[DeviceIndex].Type == DEVICE_TYPE_I2C && isI2CMultiplexerEnabled()) {
          int8_t channel = Settings.I2C_Multiplexer_Channel[TaskIndex];
          if (bitRead(Settings.I2C_Flags[TaskIndex], I2C_FLAGS_MUX_MULTICHANNEL)) {
            addHtml(F("\"I2CBus\" : ["));
            uint8_t b = 0;
            for (uint8_t c = 0; c < I2CMultiplexerMaxChannels(); c++) {
              if (bitRead(channel, c)) {
                if (b > 0) { addHtml(F(",\n")); }
                b++;
                String i2cChannel = F("\"Multiplexer channel ");
                i2cChannel += String(c);
                i2cChannel += F("\"");
                addHtml(i2cChannel);
              }
            }
            addHtml(F("],\n"));
          } else {
            if (channel == -1){
              stream_next_json_object_value(F("I2Cbus"),       F("Standard I2C bus"));
            } else {
              String i2cChannel = F("Multiplexer channel ");
              i2cChannel += String(channel);
              stream_next_json_object_value(F("I2Cbus"),       i2cChannel);
            }
          }
        }
#endif
      }
      stream_next_json_object_value(F("TaskEnabled"), jsonBool(Settings.TaskDeviceEnabled[TaskIndex]));
      stream_last_json_object_value(F("TaskNumber"), String(TaskIndex + 1));

      if (TaskIndex != lastActiveTaskIndex) {
        addHtml(",");
      }
      addHtml("\n");
    }
  }

  if (!showSpecificTask) {
    addHtml(F("],\n"));
    stream_last_json_object_value(F("TTL"), String(lowest_ttl_json * 1000));
  }

  TXBuffer.endStream();
}

// ********************************************************************************
// JSON formatted timing statistics
// ********************************************************************************

#ifdef WEBSERVER_NEW_UI
void handle_timingstats_json() {
  TXBuffer.startJsonStream();
  json_init();
  json_open();
  # ifdef USES_TIMING_STATS
  jsonStatistics(false);
  # endif // ifdef USES_TIMING_STATS
  json_close();
  TXBuffer.endStream();
}

#endif // WEBSERVER_NEW_UI

#ifdef WEBSERVER_NEW_UI
void handle_nodes_list_json() {
  if (!isLoggedIn()) { return; }
  TXBuffer.startJsonStream();
  json_init();
  json_open(true);

  for (NodesMap::iterator it = Nodes.begin(); it != Nodes.end(); ++it)
  {
    if (it->second.ip[0] != 0)
    {
      json_open();
      bool isThisUnit = it->first == Settings.Unit;

      if (isThisUnit) {
        json_number(F("thisunit"), String(1));
      }

      json_number(F("first"), String(it->first));
      json_prop(F("name"), isThisUnit ? Settings.Name : it->second.nodeName);

      if (it->second.build) { json_prop(F("build"), String(it->second.build)); }
      json_prop(F("type"), getNodeTypeDisplayString(it->second.nodeType));
      json_prop(F("ip"),   it->second.ip.toString());
      json_number(F("age"), String(it->second.age));
      json_close();
    }
  }
  json_close(true);
  TXBuffer.endStream();
}

void handle_buildinfo() {
  if (!isLoggedIn()) { return; }
  TXBuffer.startJsonStream();
  json_init();
  json_open();
  {
    json_open(true, F("plugins"));

    for (deviceIndex_t x = 0; x <= deviceCount; x++) {
      if (validPluginID(DeviceIndex_to_Plugin_id[x])) {
        json_open();
        json_number(F("id"), String(DeviceIndex_to_Plugin_id[x]));
        json_prop(F("name"), getPluginNameFromDeviceIndex(x));
        json_close();
      }
    }
    json_close(true);
  }
  {
    json_open(true, F("controllers"));

    for (protocolIndex_t x = 0; x < CPLUGIN_MAX; x++) {
      if (getCPluginID_from_ProtocolIndex(x) != INVALID_C_PLUGIN_ID) {
        json_open();
        json_number(F("id"), String(x + 1));
        json_prop(F("name"), getCPluginNameFromProtocolIndex(x));
        json_close();
      }
    }
    json_close(true);
  }
  {
    json_open(true, F("notifications"));

    for (byte x = 0; x < NPLUGIN_MAX; x++) {
      if (validNPluginID(NPlugin_id[x])) {
        json_open();
        json_number(F("id"), String(x + 1));
        json_prop(F("name"), getNPluginNameFromNotifierIndex(x));
        json_close();
      }
    }
    json_close(true);
  }
  json_prop(LabelType::BUILD_DESC);
  json_prop(LabelType::GIT_BUILD);
  json_prop(LabelType::SYSTEM_LIBRARIES);
  json_prop(LabelType::PLUGIN_COUNT);
  json_prop(LabelType::PLUGIN_DESCRIPTION);
  json_close();
  TXBuffer.endStream();
}

#endif // WEBSERVER_NEW_UI


/*********************************************************************************************\
   Streaming versions directly to TXBuffer
\*********************************************************************************************/
void stream_to_json_value(const String& value) {
  if ((value.length() == 0) || !isFloat(value)) {
    String html;
    html.reserve(value.length() + 2);
    html += '\"';
    html += value;
    html += '\"';
    addHtml(html);
  } else {
    addHtml(value);
  }
}

void stream_to_json_object_value(const String& object, const String& value) {
  String html;

  html.reserve(object.length() + 4);

  html += '\"';
  html += object;
  html += "\":";
  addHtml(html);
  stream_to_json_value(value);
}

String jsonBool(bool value) {
  return boolToString(value);
}

// Add JSON formatted data directly to the TXbuffer, including a trailing comma.
void stream_next_json_object_value(const String& object, const String& value) {
  addHtml(to_json_object_value(object, value));
  addHtml(F(",\n"));
}

// Add JSON formatted data directly to the TXbuffer, including a closing '}'
void stream_last_json_object_value(const String& object, const String& value) {
  addHtml(to_json_object_value(object, value));
  addHtml(F("\n}"));
}

void stream_next_json_object_value(LabelType::Enum label) {
  stream_next_json_object_value(getLabel(label), getValue(label));
}

void stream_last_json_object_value(LabelType::Enum label) {
  stream_last_json_object_value(getLabel(label), getValue(label));
}

#include "../WebServer/LoadFromFS.h"

#include "../WebServer/WebServer.h"
#include "../WebServer/CustomPage.h"
#include "../Globals/RamTracker.h"
#include "../Helpers/ESPEasy_Storage.h"
#include "../Helpers/Network.h"

#ifdef FEATURE_SD
#include <SD.h>
#endif

// ********************************************************************************
// Web Interface server web file from FS
// ********************************************************************************
bool loadFromFS(boolean spiffs, String path) {
  // path is a deepcopy, since it will be changed here.
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("loadFromFS"));
  #endif

  if (!isLoggedIn()) { return false; }

  statusLED(true);

  String dataType = F("text/plain");

  if (!path.startsWith(F("/"))) {
    path = String(F("/")) + path;
  }

  if (path.endsWith(F("/"))) { path += F("index.htm"); }

  if (path.endsWith(F(".src"))) { path = path.substring(0, path.lastIndexOf(".")); }
  else if (path.endsWith(F(".htm")) || path.endsWith(F(".html")) || path.endsWith(F(".htm.gz")) || path.endsWith(F(".html.gz"))) { dataType = F("text/html"); }
  else if (path.endsWith(F(".css")) || path.endsWith(F(".css.gz"))) { dataType = F("text/css"); }
  else if (path.endsWith(F(".js")) || path.endsWith(F(".js.gz"))) { dataType = F("application/javascript"); }
  else if (path.endsWith(F(".png")) || path.endsWith(F(".png.gz"))) { dataType = F("image/png"); }
  else if (path.endsWith(F(".gif")) || path.endsWith(F(".gif.gz"))) { dataType = F("image/gif"); }
  else if (path.endsWith(F(".jpg")) || path.endsWith(F(".jpg.gz"))) { dataType = F("image/jpeg"); }
  else if (path.endsWith(F(".ico"))) { dataType = F("image/x-icon"); }
  else if (path.endsWith(F(".svg"))) { dataType = F("image/svg+xml"); }
  else if (path.endsWith(F(".json"))) { dataType = F("application/json"); }
  else if (path.endsWith(F(".txt")) ||
           path.endsWith(F(".dat"))) { dataType = F("application/octet-stream"); }
#ifdef WEBSERVER_CUSTOM
  else if (path.endsWith(F(".esp"))) { return handle_custom(path); }
#endif

#ifndef BUILD_NO_DEBUG

  if (loglevelActiveFor(LOG_LEVEL_DEBUG)) {
    String log = F("HTML : Request file ");
    log += path;
    addLog(LOG_LEVEL_DEBUG, log);
  }
#endif // ifndef BUILD_NO_DEBUG

#if !defined(ESP32)
  path = path.substring(1);
#endif // if !defined(ESP32)

  if (spiffs)
  {
    if (!fileExists(path)) {
      return false;
    }
    fs::File dataFile = tryOpenFile(path.c_str(), "r");

    if (!dataFile) {
      return false;
    }

    // prevent reloading stuff on every click
    web_server.sendHeader(F("Cache-Control"), F("max-age=3600, public"));
    web_server.sendHeader(F("Vary"),          "*");
    web_server.sendHeader(F("ETag"),          F("\"2.0.0\""));

    if (path.endsWith(F(".dat"))) {
      web_server.sendHeader(F("Content-Disposition"), F("attachment;"));
    }

    web_server.streamFile(dataFile, dataType);
    dataFile.close();
  }
  else
  {
#ifdef FEATURE_SD
    File dataFile = SD.open(path.c_str());

    if (!dataFile) {
      return false;
    }

    if (path.endsWith(F(".DAT"))) {
      web_server.sendHeader(F("Content-Disposition"), F("attachment;"));
    }
    web_server.streamFile(dataFile, dataType);
    dataFile.close();
#else // ifdef FEATURE_SD

    // File from SD requested, but no SD support.
    return false;
#endif // ifdef FEATURE_SD
  }
  statusLED(true);
  return true;
}

#include "../WebServer/Log.h"

#include "../WebServer/WebServer.h"
#include "../WebServer/404.h"
#include "../WebServer/HTML_wrappers.h"
#include "../WebServer/JSON.h"
#include "../WebServer/Markup.h"
#include "../WebServer/Markup_Buttons.h"

#include "../DataStructs/LogStruct.h"

#include "../Globals/Logging.h"
#include "../Globals/Settings.h"
#include "../Helpers/ESPEasy_time_calc.h"
#include "../Static/WebStaticData.h"

// ********************************************************************************
// Web Interface log page
// ********************************************************************************
void handle_log() {
  if (!isLoggedIn()) { return; }
  navMenuIndex = MENU_INDEX_TOOLS;

  TXBuffer.startStream();
  sendHeadandTail_stdtemplate(_HEAD);
  html_table_class_normal();

  #ifdef WEBSERVER_LOG
  addHtml(F("<TR><TH id=\"headline\" align=\"left\">Log"));
  addCopyButton(F("copyText"), "", F("Copy log to clipboard"));
  addHtml(F("</TR></table><div  id='current_loglevel' style='font-weight: bold;'>Logging: </div><div class='logviewer' id='copyText_1'></div>"));
  addHtml(F("Autoscroll: "));
  addCheckBox(F("autoscroll"), true);
  addHtml(F("<BR></body>"));

  serve_JS(JSfiles_e::FetchAndParseLog);

  #else // ifdef WEBSERVER_LOG
  addHtml(F("Not included in build"));
  #endif // ifdef WEBSERVER_LOG
  sendHeadandTail_stdtemplate(_TAIL);
  TXBuffer.endStream();
}

// ********************************************************************************
// Web Interface JSON log page
// ********************************************************************************
void handle_log_JSON() {
  if (!isLoggedIn()) { return; }
  #ifdef WEBSERVER_LOG
  TXBuffer.startJsonStream();
  String webrequest = web_server.arg(F("view"));
  addHtml(F("{\"Log\": {"));

  if (webrequest == F("legend")) {
    addHtml(F("\"Legend\": ["));

    for (byte i = 0; i < LOG_LEVEL_NRELEMENTS; ++i) {
      if (i != 0) {
        addHtml(",");
      }
      addHtml("{");
      int loglevel;
      stream_next_json_object_value(F("label"), getLogLevelDisplayStringFromIndex(i, loglevel));
      stream_last_json_object_value(F("loglevel"), String(loglevel));
    }
    addHtml(F("],\n"));
  }
  addHtml(F("\"Entries\": ["));
  bool logLinesAvailable       = true;
  int  nrEntries               = 0;
  unsigned long firstTimeStamp = 0;
  unsigned long lastTimeStamp  = 0;

  while (logLinesAvailable) {
    String reply = Logging.get_logjson_formatted(logLinesAvailable, lastTimeStamp);

    if (reply.length() > 0) {
      addHtml(reply);

      if (nrEntries == 0) {
        firstTimeStamp = lastTimeStamp;
      }
      ++nrEntries;
    }

    // Do we need to do something here and maybe limit number of lines at once?
  }
  addHtml(F("],\n"));
  long logTimeSpan       = timeDiff(firstTimeStamp, lastTimeStamp);
  long refreshSuggestion = 1000;
  long newOptimum        = 1000;

  if ((nrEntries > 2) && (logTimeSpan > 1)) {
    // May need to lower the TTL for refresh when time needed
    // to fill half the log is lower than current TTL
    newOptimum = logTimeSpan * (LOG_STRUCT_MESSAGE_LINES / 2);
    newOptimum = newOptimum / (nrEntries - 1);
  }

  if (newOptimum < refreshSuggestion) { refreshSuggestion = newOptimum; }

  if (refreshSuggestion < 100) {
    // Reload times no lower than 100 msec.
    refreshSuggestion = 100;
  }
  stream_next_json_object_value(F("TTL"),                 String(refreshSuggestion));
  stream_next_json_object_value(F("timeHalfBuffer"),      String(newOptimum));
  stream_next_json_object_value(F("nrEntries"),           String(nrEntries));
  stream_next_json_object_value(F("SettingsWebLogLevel"), String(Settings.WebLogLevel));
  stream_last_json_object_value(F("logTimeSpan"), String(logTimeSpan));
  addHtml(F("}\n"));
  TXBuffer.endStream();
  updateLogLevelCache();

  #else // ifdef WEBSERVER_LOG
  handleNotFound();
  #endif // ifdef WEBSERVER_LOG
}

#include "../WebServer/Login.h"


#include "../WebServer/WebServer.h"
#include "../WebServer/AccessControl.h"
#include "../WebServer/HTML_wrappers.h"
#include "../WebServer/Markup.h"
#include "../WebServer/Markup_Buttons.h"

#include "../ESPEasyCore/ESPEasyRules.h"

#include "../Globals/SecuritySettings.h"
#include "../Globals/Settings.h"

#include "../../ESPEasy-Globals.h"


// ********************************************************************************
// Web Interface login page
// ********************************************************************************
void handle_login() {
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("handle_login"));
  #endif // ifndef BUILD_NO_RAM_TRACKER

  if (!clientIPallowed()) { return; }
  TXBuffer.startStream();
  sendHeadandTail_stdtemplate(_HEAD);

  String webrequest = web_server.arg(F("password"));
  addHtml(F("<form method='post'>"));
  html_table_class_normal();
  addHtml(F("<TR><TD>Password<TD>"));
  addHtml(F("<input "));
  addHtmlAttribute(F("class"), F("wide"));
  addHtmlAttribute(F("type"),  F("password"));
  addHtmlAttribute(F("name"),  F("password"));
  addHtmlAttribute(F("value"), webrequest);
  addHtml('>');
  html_TR_TD();
  html_TD();
  addSubmitButton();
  html_TR_TD();
  html_end_table();
  html_end_form();

  if (webrequest.length() != 0)
  {
    char command[80];
    command[0] = 0;
    webrequest.toCharArray(command, 80);

    // compare with stored password and set timer if there's a match
    if ((strcasecmp(command, SecuritySettings.Password) == 0) || (SecuritySettings.Password[0] == 0))
    {
      WebLoggedIn      = true;
      WebLoggedInTimer = 0;
      addHtml(F("<script>window.location = '.'</script>"));
    }
    else
    {
      addHtml(F("Invalid password!"));

      if (Settings.UseRules)
      {
        String event = F("Login#Failed");

        // TD-er: Do not add to the eventQueue, but execute right now.
        rulesProcessing(event);
      }
    }
  }

  sendHeadandTail_stdtemplate(_TAIL);
  TXBuffer.endStream();
  printWebString = "";
  printToWeb     = false;
}


#include "../WebServer/Markup.h"

#include "../WebServer/HTML_wrappers.h"

#include "../Globals/Settings.h"

#include "../Helpers/Hardware.h"
#include "../Helpers/StringGenerator_GPIO.h"

// ********************************************************************************
// Add Selector
// ********************************************************************************
void addSelector(const String& id,
                 int           optionCount,
                 const String  options[],
                 const int     indices[],
                 const String  attr[],
                 int           selectedIndex)
{
  addSelector(id, optionCount, options, indices, attr, selectedIndex, false, true, F("wide"));
}

void addSelector(const String& id,
                 int           optionCount,
                 const String  options[],
                 const int     indices[],
                 const String  attr[],
                 int           selectedIndex,
                 boolean       reloadonchange,
                 bool          enabled)
{
  addSelector(id, optionCount, options, indices, attr, selectedIndex, reloadonchange, enabled, F("wide"));
}

void addSelector(const String& id,
                 int           optionCount,
                 const String  options[],
                 const int     indices[],
                 const String  attr[],
                 int           selectedIndex,
                 boolean       reloadonchange,
                 bool          enabled,
                 const String& classname)
{
  // FIXME TD-er Change boolean to disabled
  if (reloadonchange)
  {
    addSelector_Head_reloadOnChange(id, classname, !enabled);
  } else {
    do_addSelector_Head(id, classname, "", !enabled);
  }
  addSelector_options(optionCount, options, indices, attr, selectedIndex);
  addSelector_Foot();
}

void addSelector_options(int optionCount, const String options[], const int indices[], const String attr[], int selectedIndex)
{
  int index;

  for (byte x = 0; x < optionCount; x++)
  {
    if (indices) {
      index = indices[x];
    }
    else {
      index = x;
    }
    String attr_str;

    if (attr)
    {
      attr_str = attr[x];
    }
    addSelector_Item(options[x], index, selectedIndex == index, false, attr_str);
  }
}

void addSelector_Head(const String& id) {
  do_addSelector_Head(id, F("wide"), "", false);
}

void addSelector_Head_reloadOnChange(const String& id) {
  addSelector_Head_reloadOnChange(id, F("wide"), false);
}

void addSelector_Head_reloadOnChange(const String& id, const String& classname, bool disabled) {
  do_addSelector_Head(id, classname, F("return dept_onchange(frmselect)"), disabled);
}

void addSelector_Head_reloadOnChange(const String& id, const String& classname, const String& onChangeCall, bool disabled) {
  do_addSelector_Head(id, classname, onChangeCall, disabled);
}

void do_addSelector_Head(const String& id, const String& classname, const String& onChangeCall, const bool& disabled)
{
  addHtml(F("<select "));
  addHtmlAttribute(F("class"), classname);
  addHtmlAttribute(F("name"),  id);
  addHtmlAttribute(F("id"),    id);

  if (disabled) {
    addDisabled();
  }

  if (onChangeCall.length() > 0) {
    addHtmlAttribute(F("onchange"), onChangeCall);
  }
  addHtml('>');
}

void addSelector_Item(const String& option, int index, boolean selected, boolean disabled, const String& attr)
{
  addHtml(F("<option "));
  addHtmlAttribute(F("value"), index);

  if (selected) {
    addHtml(F(" selected"));
  }

  if (disabled) {
    addDisabled();
  }

  if (attr.length() > 0)
  {
    addHtml(' ');
    addHtml(attr);
  }
  addHtml('>');
  addHtml(option);
  addHtml(F("</option>"));
}

void addSelector_Foot()
{
  addHtml(F("</select>"));
}

void addUnit(const String& unit)
{
  String html;

  html += F(" [");
  html += unit;
  html += "]";
  addHtml(html);
}

void addRowLabel_tr_id(const String& label, const String& id)
{
  String tr_id = F("tr_");

  tr_id += id;
  addRowLabel(label, tr_id);
}

void addRowLabel(const String& label, const String& id)
{
  if (id.length() > 0) {
    String html;
    html += F("<TR id='");
    html += id;
    html += F("'><TD>");
    addHtml(html);
  } else {
    html_TR_TD();
  }

  if (label.length() != 0) {
    String html;
    html += label;
    html += ':';
    addHtml(html);
  }
  html_TD();
}

// Add a row label and mark it with copy markers to copy it to clipboard.
void addRowLabel_copy(const String& label) {
  addHtml(F("<TR>"));
  html_copyText_TD();
  String html;

  html += label;
  html += ':';
  addHtml(html);
  html_copyText_marker();
  html_copyText_TD();
}

void addRowLabel(LabelType::Enum label) {
  addRowLabel(getLabel(label));
}

void addRowLabelValue(LabelType::Enum label) {
  addRowLabel(getLabel(label));
  addHtml(getValue(label));
}

void addRowLabelValue_copy(LabelType::Enum label) {
  addRowLabel_copy(getLabel(label));
  addHtml(getValue(label));
}

// ********************************************************************************
// Add a header
// ********************************************************************************
void addTableSeparator(const String& label, int colspan, int h_size, const String& helpButton) {
  {
    String html;
    html.reserve(32 + label.length());
    html += F("<TR><TD colspan=");
    html += colspan;
    html += "><H";
    html += h_size;
    html += '>';
    html += label;
    addHtml(html);
  }

  if (helpButton.length() > 0) {
    addHelpButton(helpButton);
  }
  {
    String html;
    html.reserve(16);
    html += "</H";
    html += h_size;
    html += F("></TD></TR>");
    addHtml(html);
  }
}

void addFormHeader(const String& header, const String& helpButton) {
  addFormHeader(header, helpButton, F(""));
}

void addFormHeader(const String& header, const String& helpButton, const String& rtdHelpButton)
{
  html_TR();
  html_table_header(header, helpButton, rtdHelpButton, 225);
  html_table_header("");
}

// ********************************************************************************
// Add a sub header
// ********************************************************************************
void addFormSubHeader(const String& header)
{
  addTableSeparator(header, 2, 3);
}

// ********************************************************************************
// Add a checkbox
// ********************************************************************************
void addCheckBox(const String& id, boolean checked, bool disabled)
{
  addHtml(F("<label class='container'>&nbsp;"));
  addHtml(F("<input "));
  addHtmlAttribute(F("type"), F("checkbox"));
  addHtmlAttribute(F("id"),   id);
  addHtmlAttribute(F("name"), id);

  if (checked) {
    addHtml(F(" checked"));
  }

  if (disabled) { addDisabled(); }
  addHtml(F("><span class='checkmark"));

  if (disabled) { addDisabled(); }
  addHtml(F("'></span></label>"));
}

// ********************************************************************************
// Add a numeric box
// ********************************************************************************
void addNumericBox(const String& id, int value, int min, int max)
{
  addHtml(F("<input "));
  addHtmlAttribute(F("class"), F("widenumber"));
  addHtmlAttribute(F("type"),  F("number"));
  addHtmlAttribute(F("name"),  id);

  if (value < min) {
    value = min;
  }

  if (value > max) {
    value = max;
  }

  if (min != INT_MIN)
  {
    addHtmlAttribute(F("min"), min);
  }

  if (max != INT_MAX)
  {
    addHtmlAttribute(F("max"), max);
  }
  addHtmlAttribute(F("value"), value);
  addHtml('>');
}

void addFloatNumberBox(const String& id, float value, float min, float max)
{
  String html;

  html.reserve(64 + id.length());

  html += F("<input type='number' name='");
  html += id;
  html += '\'';
  html += F(" min=");
  html += String(min, 6);
  html += F(" max=");
  html += String(max, 6);
  html += F(" step=0.000001");

  html += F(" style='width:7em;' value=");
  html += String(value, 6);
  html += '>';

  addHtml(html);
}

// ********************************************************************************
// Add Textbox
// ********************************************************************************
void addTextBox(const String& id, const String&  value, int maxlength, bool readonly, bool required, const String& pattern)
{
  addTextBox(id, value, maxlength, readonly, required, pattern, F("wide"));
}

void addTextBox(const String& id,
                const String& value,
                int           maxlength,
                bool          readonly,
                bool          required,
                const String& pattern,
                const String& classname)
{
  addHtml(F("<input "));
  addHtmlAttribute(F("class"),     classname);
  addHtmlAttribute(F("type"),      F("text"));
  addHtmlAttribute(F("name"),      id);
  addHtmlAttribute(F("maxlength"), maxlength);
  addHtmlAttribute(F("value"),     value);

  if (readonly) {
    addHtml(F(" readonly "));
  }

  if (required) {
    addHtml(F(" required "));
  }

  if (pattern.length() > 0) {
    addHtmlAttribute(F("pattern"), pattern);
  }
  addHtml('>');
}

// ********************************************************************************
// Add Textarea
// ********************************************************************************
void addTextArea(const String& id, const String& value, int maxlength, int rows, int columns, bool readonly, bool required)
{
  addHtml(F("<textarea "));
  addHtmlAttribute(F("class"),     F("wide"));
  addHtmlAttribute(F("type"),      F("text"));
  addHtmlAttribute(F("name"),      id);
  addHtmlAttribute(F("maxlength"), maxlength);
  addHtmlAttribute(F("rows"),      rows);
  addHtmlAttribute(F("cols"),      columns);

  if (readonly) {
    addHtml(F(" readonly "));
  }

  if (required) {
    addHtml(F(" required "));
  }
  addHtml('>');
  addHtml(value);
  addHtml(F("</textarea>"));
}

// ********************************************************************************
// Add Help Buttons
// ********************************************************************************

// adds a Help Button with points to the the given Wiki Subpage
// If url starts with "RTD", it will be considered as a Read-the-docs link
void addHelpButton(const String& url) {
  if (url.startsWith("RTD")) {
    addRTDHelpButton(url.substring(3));
  } else {
    addHelpButton(url, false);
  }
}

void addRTDHelpButton(const String& url)
{
  addHelpButton(url, true);
}

void addHelpButton(const String& url, bool isRTD)
{
  addHtmlLink(
    F("button help"),
    makeDocLink(url, isRTD),
    isRTD ? F("&#8505;") : F("&#10068;"));
}

void addRTDPluginButton(pluginID_t taskDeviceNumber) {
  String url;

  url.reserve(16);
  url = F("Plugin/P");

  if (taskDeviceNumber < 100) { url += '0'; }

  if (taskDeviceNumber < 10) { url += '0'; }
  url += String(taskDeviceNumber);
  url += F(".html");
  addRTDHelpButton(url);

  switch (taskDeviceNumber) {
    case 76:
    case 77:
      addHtmlLink(
        F("button help"),
        makeDocLink(F("Reference/Safety.html"), true),
        F("&#9889;")); // High voltage sign
      break;
  }
}

String makeDocLink(const String& url, bool isRTD) {
  String result;

  if (!url.startsWith(F("http"))) {
    if (isRTD) {
      result += F("https://espeasy.readthedocs.io/en/latest/");
    } else {
      result += F("http://www.letscontrolit.com/wiki/index.php/");
    }
  }
  result += url;
  return result;
}

void addPinSelect(boolean forI2C, const String& id,  int choice)
{
  #ifdef ESP32
    # define NR_ITEMS_PIN_DROPDOWN  35 // 34 GPIO + 1
  #else // ifdef ESP32
    # define NR_ITEMS_PIN_DROPDOWN  14 // 13 GPIO + 1
  #endif // ifdef ESP32

  String *gpio_labels  = new String[NR_ITEMS_PIN_DROPDOWN];
  int    *gpio_numbers = new int[NR_ITEMS_PIN_DROPDOWN];

  // At i == 0 && gpio == -1, add the "- None -" option first
  int i    = 0;
  int gpio = -1;

  while (i < NR_ITEMS_PIN_DROPDOWN && gpio <= MAX_GPIO) {
    int  pinnr = -1;
    bool input, output, warning;

    if (getGpioInfo(gpio, pinnr, input, output, warning) || (i == 0)) {
      gpio_labels[i]  = createGPIO_label(gpio, pinnr, input, output, warning);
      gpio_numbers[i] = gpio;
      ++i;
    }
    ++gpio;
  }
  renderHTMLForPinSelect(gpio_labels, gpio_numbers, forI2C, id, choice, NR_ITEMS_PIN_DROPDOWN);
  delete[] gpio_numbers;
  delete[] gpio_labels;
  #undef NR_ITEMS_PIN_DROPDOWN
}

#ifdef ESP32
void addADC_PinSelect(bool touchOnly, const String& id,  int choice)
{
  int NR_ITEMS_PIN_DROPDOWN = touchOnly ? 10 : 19;
  String *gpio_labels       = new String[NR_ITEMS_PIN_DROPDOWN];
  int    *gpio_numbers      = new int[NR_ITEMS_PIN_DROPDOWN];

  // At i == 0 && gpio == -1, add the "Hall Effect" option first
  int i    = 0;
  int gpio = -1;

  while (i < NR_ITEMS_PIN_DROPDOWN && gpio <= MAX_GPIO) {
    int  pinnr = -1;
    bool input, output, warning;

    if (touchOnly) {
      // For touch only list, sort based on touch number
      // Default sort is on GPIO number.
      gpio = touchPinToGpio(i);
    }

    if (getGpioInfo(gpio, pinnr, input, output, warning) || (i == 0)) {
      int adc, ch, t;

      if (getADC_gpio_info(gpio, adc, ch, t)) {
        if (!touchOnly || (t >= 0)) {
          gpio_labels[i] = formatGpioName_ADC(gpio);

          if (adc != 0) {
            gpio_labels[i] += F(" / ");
            gpio_labels[i] += createGPIO_label(gpio, pinnr, input, output, warning);
          }
          gpio_numbers[i] = gpio;
          ++i;
        }
      }
    }
    ++gpio;
  }
  bool forI2C = false;

  renderHTMLForPinSelect(gpio_labels, gpio_numbers, forI2C, id, choice, i);
  delete[] gpio_numbers;
  delete[] gpio_labels;
}

#endif // ifdef ESP32


// ********************************************************************************
// Helper function actually rendering dropdown list for addPinSelect()
// ********************************************************************************
void renderHTMLForPinSelect(String options[], int optionValues[], boolean forI2C, const String& id,  int choice, int count) {
  addSelector_Head(id);

  for (byte x = 0; x < count; x++)
  {
    boolean disabled = false;

    if (optionValues[x] != -1) // empty selection can never be disabled...
    {
      if (!forI2C && ((optionValues[x] == Settings.Pin_i2c_sda) || (optionValues[x] == Settings.Pin_i2c_scl))) {
        disabled = true;
      }

      if (Settings.UseSerial && ((optionValues[x] == 1) || (optionValues[x] == 3))) {
        disabled = true;
      }

      if (Settings.InitSPI != 0) {
        #ifdef ESP32

        switch (Settings.InitSPI)
        {
          case 1:
            disabled = (optionValues[x] == 18 || optionValues[x] == 19 || optionValues[x] == 23);
            break;
          case 2:
            disabled = (optionValues[x] == 14 || optionValues[x] == 12 || optionValues[x] == 13);
            break;
        }
        #else // #ifdef ESP32
        disabled = (optionValues[x] == 14 || optionValues[x] == 12 || optionValues[x] == 13);
        #endif // ifdef ESP32
      }
    }
    addSelector_Item(options[x],
                     optionValues[x],
                     choice == optionValues[x],
                     disabled,
                     "");
  }
  addSelector_Foot();
}

#include "../WebServer/Markup_Buttons.h"

#include "../WebServer/common.h"
#include "../WebServer/HTML_wrappers.h"

#include "../Static/WebStaticData.h"


void addButton(const String& url, const String& label) {
  addButton(url, label, "");
}

void addButton(const String& url, const String& label, const String& classes) {
  addButton(url, label, classes, true);
}

void addButton(const String& url, const String& label, const String& classes, bool enabled)
{
  html_add_button_prefix(classes, enabled);
  String html;

  html.reserve(8 + url.length() + label.length());
  html += url;
  html += "'>";
  html += label;
  html += F("</a>");
  addHtml(html);
}

void addButtonWithSvg(const String& url, const String& label)
{
  addButtonWithSvg(url, label, "", false);
}

void addButtonWithSvg(const String& url, const String& label, const String& svgPath, bool needConfirm) {
  addHtml(F("<a "));
  addHtmlAttribute(F("class"), F("button link"));
  addHtmlAttribute(F("href"),  url);
  #ifndef BUILD_MINIMAL_OTA
  bool hasSVG = svgPath.length() > 0;

  if (hasSVG)
  {
    addHtmlAttribute(F("alt"), label);
  }
  #endif // ifndef BUILD_MINIMAL_OTA

  if (needConfirm) {
    addHtmlAttribute(F("onclick"), F("return confirm(\"Are you sure?\")"));
  }
  addHtml('>');

  #ifndef BUILD_MINIMAL_OTA

  if (hasSVG) {
    addHtml(F("<svg width='24' height='24' viewBox='-1 -1 26 26' style='position: relative; top: 5px;'>"));
    addHtml(svgPath);
    addHtml(F("</svg>"));
  } else
  #endif // ifndef BUILD_MINIMAL_OTA
  {
    addHtml(label);
  }
  addHtml(F("</a>"));
}

void addSaveButton(const String& url, const String& label)
{
#ifdef BUILD_MINIMAL_OTA
  addButtonWithSvg(url, label
                   , ""
                   , false);
#else // ifdef BUILD_MINIMAL_OTA
  addButtonWithSvg(url,
                   label
                   ,
                   F(
                     "<path d='M19 12v7H5v-7H3v7c0 1.1.9 2 2 2h14c1.1 0 2-.9 2-2v-7h-2zm-6 .67l2.59-2.58L17 11.5l-5 5-5-5 1.41-1.41L11 12.67V3h2v9.67z'  stroke='white' fill='white' ></path>")
                   ,
                   false);
#endif // ifdef BUILD_MINIMAL_OTA
}

void addDeleteButton(const String& url, const String& label)
{
#ifdef BUILD_MINIMAL_OTA
  addButtonWithSvg(url, label
                   , ""
                   , true);
#else // ifdef BUILD_MINIMAL_OTA
  addButtonWithSvg(url,
                   label
                   ,
                   F(
                     "<path fill='none' d='M0 0h24v24H0V0z'></path><path d='M6 19c0 1.1.9 2 2 2h8c1.1 0 2-.9 2-2V7H6v12zM8 9h8v10H8V9zm7.5-5l-1-1h-5l-1 1H5v2h14V4h-3.5z' stroke='white' fill='white' ></path>")
                   ,
                   true);
#endif // ifdef BUILD_MINIMAL_OTA
}

void addWideButton(const String& url, const String& label) {
  addWideButton(url, label, "", true);
}

void addWideButton(const String& url, const String& label, const String& classes) {
  addWideButton(url, label, classes, true);
}

void addWideButton(const String& url, const String& label, const String& classes, bool enabled)
{
  html_add_wide_button_prefix(classes, enabled);
  String html;

  html.reserve(8 + url.length() + label.length());
  html += url;
  html += "'>";
  html += label;
  html += F("</a>");
  addHtml(html);
}

void addSubmitButton()
{
  addSubmitButton(F("Submit"), "");
}

// add submit button with different label and name
void addSubmitButton(const String& value, const String& name) {
  addSubmitButton(value, name, "");
}

void addSubmitButton(const String& value, const String& name, const String& classes)
{
  addHtml(F("<input "));
  {
    String fullClasses;
    fullClasses.reserve(12 + classes.length());
    fullClasses = F("button link");

    if (classes.length() == 0) {
      fullClasses += ' ';
      fullClasses += classes;
    }
    addHtmlAttribute(F("class"), fullClasses);
  }
  addHtmlAttribute(F("type"),  F("submit"));
  addHtmlAttribute(F("value"), value);

  if (name.length() > 0) {
    addHtmlAttribute(F("name"), name);
  }
  addHtmlAttribute(F("onclick"), F("toasting()"));
  addHtml(F("/><div id='toastmessage'></div>"));
}

// add copy to clipboard button
void addCopyButton(const String& value, const String& delimiter, const String& name)
{
  TXBuffer += jsClipboardCopyPart1;
  addHtml(value);
  TXBuffer += jsClipboardCopyPart2;
  addHtml(delimiter);
  TXBuffer += jsClipboardCopyPart3;

  // Fix HTML
  addHtml(F("<button "));
  addHtmlAttribute(F("class"),   F("button link"));
  addHtmlAttribute(F("onclick"), F("setClipboard()"));
  addHtml('>');
  addHtml(name);
  addHtml(" (");
  html_copyText_marker();
  addHtml(F(")</button>"));
}

#include "../WebServer/Markup_Forms.h"

#include "../WebServer/WebServer.h"
#include "../WebServer/AccessControl.h"
#include "../WebServer/Markup.h"
#include "../WebServer/HTML_wrappers.h"

#include "../Globals/Settings.h"

#include "../Helpers/Hardware.h"
#include "../Helpers/Numerical.h"
#include "../Helpers/StringConverter.h"
#include "../Helpers/StringGenerator_GPIO.h"

// ********************************************************************************
// Add a separator as row start
// ********************************************************************************
void addFormSeparator(int clspan)
{
  String html;

  html.reserve(40);
  html += F("<TR><TD colspan='");
  html += clspan;
  html += F("'><hr>");
  addHtml(html);
}

// ********************************************************************************
// Add a note as row start
// ********************************************************************************
void addFormNote(const String& text, const String& id)
{
  addRowLabel_tr_id("", id);
  String html;

  html.reserve(40 + text.length());
  html += F("<div class='note'>Note: ");
  html += text;
  html += F("</div>");
  addHtml(html);
}

// ********************************************************************************
// Create Forms
// ********************************************************************************


// ********************************************************************************
// Add a checkbox Form
// ********************************************************************************

void addFormCheckBox_disabled(const String& label, const String& id, boolean checked) {
  addFormCheckBox(label, id, checked, true);
}

void addFormCheckBox(const String& label, const String& id, boolean checked, bool disabled)
{
  addRowLabel_tr_id(label, id);
  addCheckBox(id, checked, disabled);
}

void addFormCheckBox(LabelType::Enum label, boolean checked, bool disabled) {
  addFormCheckBox(getLabel(label), getInternalLabel(label), checked, disabled);
}

void addFormCheckBox_disabled(LabelType::Enum label, boolean checked) {
  addFormCheckBox(label, checked, true);
}

// ********************************************************************************
// Add a Numeric Box form
// ********************************************************************************
void addFormNumericBox(const String& label, const String& id, int value, int min, int max)
{
  addRowLabel_tr_id(label, id);
  addNumericBox(id, value, min, max);
}

void addFormFloatNumberBox(const String& label, const String& id, float value, float min, float max)
{
  addRowLabel_tr_id(label, id);
  addFloatNumberBox(id, value, min, max);
}

// ********************************************************************************
// Add a task selector form
// ********************************************************************************
void addTaskSelectBox(const String& label, const String& id, taskIndex_t choice)
{
  addRowLabel_tr_id(label, id);
  addTaskSelect(id, choice);
}

// ********************************************************************************
// Add a Text Box form
// ********************************************************************************
void addFormTextBox(const String& label,
                    const String& id,
                    const String& value,
                    int           maxlength,
                    bool          readonly,
                    bool          required,
                    const String& pattern)
{
  addRowLabel_tr_id(label, id);
  addTextBox(id, value, maxlength, readonly, required, pattern);
}

void addFormTextArea(const String& label,
                     const String& id,
                     const String& value,
                     int           maxlength,
                     int           rows,
                     int           columns,
                     bool          readonly,
                     bool          required)
{
  addRowLabel_tr_id(label, id);
  addTextArea(id, value, maxlength, rows, columns, readonly, required);
}

// ********************************************************************************
// Add a Password Box form
// ********************************************************************************

void addFormPasswordBox(const String& label, const String& id, const String& password, int maxlength)
{
  addRowLabel_tr_id(label, id);

  addHtml(F("<input "));
  addHtmlAttribute(F("class"),     F("wide"));
  addHtmlAttribute(F("type"),      F("password"));
  addHtmlAttribute(F("name"),      id);
  addHtmlAttribute(F("maxlength"), maxlength);
  addHtmlAttribute(F("value"),     (password.length() == 0) ? F("") : F("*****"));
  addHtml('>');
}

bool getFormPassword(const String& id, String& password)
{
  password = web_server.arg(id);
  return !password.equals(F("*****"));
}

// ********************************************************************************
// Add a IP Box form
// ********************************************************************************

void addFormIPBox(const String& label, const String& id, const byte ip[4])
{
  bool empty_IP = (ip[0] == 0 && ip[1] == 0 && ip[2] == 0 && ip[3] == 0);

  addRowLabel_tr_id(label, id);

  addHtml(F("<input "));
  addHtmlAttribute(F("class"), F("wide"));
  addHtmlAttribute(F("type"),  F("text"));
  addHtmlAttribute(F("name"),  id);
  addHtmlAttribute(F("value"), (empty_IP) ? F("") : formatIP(ip));
  addHtml('>');
}

// ********************************************************************************
// Add a IP Access Control select dropdown list
// ********************************************************************************
void addFormIPaccessControlSelect(const String& label, const String& id, int choice)
{
  addRowLabel_tr_id(label, id);
  addIPaccessControlSelect(id, choice);
}

// ********************************************************************************
// Add a selector form
// ********************************************************************************

void addFormPinSelect(const String& label, const String& id, int choice)
{
  addRowLabel_tr_id(label, id);
  addPinSelect(false, id, choice);
}

void addFormPinSelectI2C(const String& label, const String& id, int choice)
{
  addRowLabel_tr_id(label, id);
  addPinSelect(true, id, choice);
}

void addFormSelectorI2C(const String& id, int addressCount, const int addresses[], int selectedIndex)
{
  addRowLabel_tr_id(F("I2C Address"), id);
  do_addSelector_Head(id, "", "", false);

  for (byte x = 0; x < addressCount; x++)
  {
    String option = formatToHex_decimal(addresses[x]);

    if (x == 0) {
      option += F(" - (default)");
    }
    addSelector_Item(option, addresses[x], addresses[x] == selectedIndex, false, "");
  }
  addSelector_Foot();
}

void addFormSelector(const String& label, const String& id, int optionCount, const String options[], const int indices[], int selectedIndex)
{
  addFormSelector(label, id, optionCount, options, indices, NULL, selectedIndex, false);
}

void addFormSelector(const String& label,
                     const String& id,
                     int           optionCount,
                     const String  options[],
                     const int     indices[],
                     int           selectedIndex,
                     bool          reloadonchange)
{
  addFormSelector(label, id, optionCount, options, indices, NULL, selectedIndex, reloadonchange);
}

void addFormSelector(const String& label,
                     const String& id,
                     int           optionCount,
                     const String  options[],
                     const int     indices[],
                     const String  attr[],
                     int           selectedIndex,
                     boolean       reloadonchange)
{
  addRowLabel_tr_id(label, id);
  addSelector(id, optionCount, options, indices, attr, selectedIndex, reloadonchange, true);
}

void addFormSelector_script(const String& label,
                            const String& id,
                            int           optionCount,
                            const String  options[],
                            const int     indices[],
                            const String  attr[],
                            int           selectedIndex,
                            const String& onChangeCall)
{
  addRowLabel_tr_id(label, id);
  do_addSelector_Head(id, "", onChangeCall, false);
  addSelector_options(optionCount, options, indices, attr, selectedIndex);
  addSelector_Foot();
}

// ********************************************************************************
// Add a GPIO pin select dropdown list
// ********************************************************************************
void addFormPinStateSelect(int gpio, int choice)
{
  bool enabled = true;

  if (Settings.UseSerial && ((gpio == 1) || (gpio == 3))) {
    // do not add the pin state select for these pins.
    enabled = false;
  }
  int  pinnr = -1;
  bool input, output, warning;

  if (getGpioInfo(gpio, pinnr, input, output, warning)) {
    String label;
    label.reserve(32);
    label  = F("Pin mode ");
    label += createGPIO_label(gpio, pinnr, input, output, warning);
    String id = "p";
    id += gpio;

    addRowLabel_tr_id(label, id);
    bool hasPullUp, hasPullDown;
    getGpioPullResistor(gpio, hasPullUp, hasPullDown);
    int nr_options = 0;
    String options[6];
    int    option_val[6];
    options[nr_options]    = F("Default");
    option_val[nr_options] = static_cast<int>(PinBootState::Default_state);
    ++nr_options;

    if (output) {
      options[nr_options]    = F("Output Low");
      option_val[nr_options] = static_cast<int>(PinBootState::Output_low);
      ++nr_options;
      options[nr_options]    = F("Output High");
      option_val[nr_options] = static_cast<int>(PinBootState::Output_high);
      ++nr_options;
    }

    if (input) {
      if (hasPullUp) {
        options[nr_options]    = F("Input pullup");
        option_val[nr_options] = static_cast<int>(PinBootState::Input_pullup);
        ++nr_options;
      }

      if (hasPullDown) {
        options[nr_options]    = F("Input pulldown");
        option_val[nr_options] = static_cast<int>(PinBootState::Input_pulldown);
        ++nr_options;
      }

      if (!hasPullUp && !hasPullDown) {
        options[nr_options]    = F("Input");
        option_val[nr_options] = static_cast<int>(PinBootState::Input);
        ++nr_options;
      }
    }
    addSelector(id, nr_options, options, option_val, NULL, choice, false, enabled);
  }
}

// ********************************************************************************
// Retrieve return values from form/checkbox.
// ********************************************************************************


int getFormItemInt(const String& key, int defaultValue) {
  int value = defaultValue;

  getCheckWebserverArg_int(key, value);
  return value;
}

bool getCheckWebserverArg_int(const String& key, int& value) {
  String valueStr = web_server.arg(key);

  if (!isInt(valueStr)) { return false; }
  value = valueStr.toInt();
  return true;
}

bool update_whenset_FormItemInt(const String& key, int& value) {
  int tmpVal;

  if (getCheckWebserverArg_int(key, tmpVal)) {
    value = tmpVal;
    return true;
  }
  return false;
}

bool update_whenset_FormItemInt(const String& key, byte& value) {
  int tmpVal;

  if (getCheckWebserverArg_int(key, tmpVal)) {
    value = tmpVal;
    return true;
  }
  return false;
}

// Note: Checkbox values will not appear in POST Form data if unchecked.
// So if webserver does not have an argument for a checkbox form, it means it should be considered unchecked.
bool isFormItemChecked(const String& id)
{
  return web_server.arg(id) == F("on");
}

int getFormItemInt(const String& id)
{
  return getFormItemInt(id, 0);
}

float getFormItemFloat(const String& id)
{
  String val = web_server.arg(id);

  if (!isFloat(val)) { return 0.0f; }
  return val.toFloat();
}

bool isFormItem(const String& id)
{
  return web_server.arg(id).length() != 0;
}

void copyFormPassword(const String& id, char *pPassword, int maxlength)
{
  String password;

  if (getFormPassword(id, password)) {
    safe_strncpy(pPassword, password.c_str(), maxlength);
  }
}

#include "../WebServer/NotificationPage.h"

#include "../WebServer/WebServer.h"
#include "../WebServer/HTML_wrappers.h"
#include "../WebServer/Markup.h"
#include "../WebServer/Markup_Buttons.h"
#include "../WebServer/Markup_Forms.h"

#include "../DataStructs/ESPEasy_EventStruct.h"
#include "../DataStructs/NotificationSettingsStruct.h"

#include "../Helpers/ESPEasy_Storage.h"

#include "../Globals/ESPEasy_Scheduler.h"
#include "../Globals/NPlugins.h"
#include "../Globals/Settings.h"



// ********************************************************************************
// Web Interface notifcations page
// ********************************************************************************

#ifdef USES_NOTIFIER

#include "../Globals/NPlugins.h"


void handle_notifications() {
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("handle_notifications"));
  #endif

  if (!isLoggedIn()) { return; }
  navMenuIndex = MENU_INDEX_NOTIFICATIONS;
  TXBuffer.startStream();
  sendHeadandTail_stdtemplate(_HEAD);

  struct EventStruct TempEvent;

  // char tmpString[64];


  byte notificationindex          = getFormItemInt(F("index"), 0);
  boolean notificationindexNotSet = notificationindex == 0;
  --notificationindex;

  const int notification = getFormItemInt(F("notification"), -1);

  if ((notification != -1) && !notificationindexNotSet)
  {
    MakeNotificationSettings(NotificationSettings);

    if (Settings.Notification[notificationindex] != notification)
    {
      Settings.Notification[notificationindex] = notification;
    }
    else
    {
      if (Settings.Notification != 0)
      {
        nprotocolIndex_t NotificationProtocolIndex = getNProtocolIndex_from_NotifierIndex(notificationindex);

        if (validNProtocolIndex(NotificationProtocolIndex)) {
          String dummyString;
          NPlugin_ptr[NotificationProtocolIndex](NPlugin::Function::NPLUGIN_WEBFORM_SAVE, 0, dummyString);
        }
        NotificationSettings.Port                       = getFormItemInt(F("port"), 0);
        NotificationSettings.Pin1                       = getFormItemInt(F("pin1"), 0);
        NotificationSettings.Pin2                       = getFormItemInt(F("pin2"), 0);
        Settings.NotificationEnabled[notificationindex] = isFormItemChecked(F("notificationenabled"));
        strncpy_webserver_arg(NotificationSettings.Domain,   F("domain"));
        strncpy_webserver_arg(NotificationSettings.Server,   F("server"));
        strncpy_webserver_arg(NotificationSettings.Sender,   F("sender"));
        strncpy_webserver_arg(NotificationSettings.Receiver, F("receiver"));
        strncpy_webserver_arg(NotificationSettings.Subject,  F("subject"));
        strncpy_webserver_arg(NotificationSettings.User,     F("user"));
        strncpy_webserver_arg(NotificationSettings.Pass,     F("pass"));
        strncpy_webserver_arg(NotificationSettings.Body,     F("body"));
      }
    }

    // Save the settings.
    addHtmlError(SaveNotificationSettings(notificationindex, (byte *)&NotificationSettings, sizeof(NotificationSettingsStruct)));
    addHtmlError(SaveSettings());

    if (web_server.hasArg(F("test"))) {
      // Perform tests with the settings in the form.
      nprotocolIndex_t NotificationProtocolIndex = getNProtocolIndex_from_NotifierIndex(notificationindex);

      if (validNProtocolIndex(NotificationProtocolIndex))
      {
        // TempEvent.NotificationProtocolIndex = NotificationProtocolIndex;
        TempEvent.NotificationIndex = notificationindex;
        Scheduler.schedule_notification_event_timer(NotificationProtocolIndex, NPlugin::Function::NPLUGIN_NOTIFY, &TempEvent);
      }
    }
  }

  html_add_form();

  if (notificationindexNotSet)
  {
    html_table_class_multirow();
    html_TR();
    html_table_header("",           70);
    html_table_header("Nr",         50);
    html_table_header(F("Enabled"), 100);
    html_table_header(F("Service"));
    html_table_header(F("Server"));
    html_table_header("Port");

    MakeNotificationSettings(NotificationSettings);

    for (byte x = 0; x < NOTIFICATION_MAX; x++)
    {
      LoadNotificationSettings(x, (byte *)&NotificationSettings, sizeof(NotificationSettingsStruct));
      NotificationSettings.validate();
      html_TR_TD();
      html_add_button_prefix();
      addHtml(F("notifications?index="));
      addHtml(String(x + 1));
      addHtml(F("'>Edit</a>"));
      html_TD();
      addHtml(String(x + 1));
      html_TD();

      if (Settings.Notification[x] != 0)
      {
        addEnabled(Settings.NotificationEnabled[x]);

        html_TD();
        byte   NotificationProtocolIndex = getNProtocolIndex(Settings.Notification[x]);
        String NotificationName          = F("(plugin not found?)");

        if (validNProtocolIndex(NotificationProtocolIndex))
        {
          NPlugin_ptr[NotificationProtocolIndex](NPlugin::Function::NPLUGIN_GET_DEVICENAME, 0, NotificationName);
        }
        addHtml(NotificationName);
        html_TD();
        addHtml(NotificationSettings.Server);
        html_TD();
        addHtml(String(NotificationSettings.Port));
      }
      else {
        html_TD(3);
      }
    }
    html_end_table();
    html_end_form();
  }
  else
  {
    html_table_class_normal();
    addFormHeader(F("Notification Settings"));
    addRowLabel(F("Notification"));
    byte choice = Settings.Notification[notificationindex];
    addSelector_Head_reloadOnChange(F("notification"));
    addSelector_Item(F("- None -"), 0, false, false, "");

    for (byte x = 0; x <= notificationCount; x++)
    {
      String NotificationName = "";
      NPlugin_ptr[x](NPlugin::Function::NPLUGIN_GET_DEVICENAME, 0, NotificationName);
      addSelector_Item(NotificationName,
                       Notification[x].Number,
                       choice == Notification[x].Number,
                       false,
                       "");
    }
    addSelector_Foot();

    addHelpButton(F("EasyNotifications"));

    if (Settings.Notification[notificationindex])
    {
      MakeNotificationSettings(NotificationSettings);
      LoadNotificationSettings(notificationindex, (byte *)&NotificationSettings, sizeof(NotificationSettingsStruct));
      NotificationSettings.validate();

      nprotocolIndex_t NotificationProtocolIndex = getNProtocolIndex_from_NotifierIndex(notificationindex);

      if (validNProtocolIndex(NotificationProtocolIndex))
      {
        if (Notification[NotificationProtocolIndex].usesMessaging)
        {
          addFormTextBox(F("Domain"), F("domain"), NotificationSettings.Domain, sizeof(NotificationSettings.Domain) - 1);
          addFormTextBox(F("Server"), F("server"), NotificationSettings.Server, sizeof(NotificationSettings.Server) - 1);
          addFormNumericBox(F("Port"), F("port"), NotificationSettings.Port, 1, 65535);

          addFormTextBox(F("Sender"),   F("sender"),   NotificationSettings.Sender,   sizeof(NotificationSettings.Sender) - 1);
          addFormTextBox(F("Receiver"), F("receiver"), NotificationSettings.Receiver, sizeof(NotificationSettings.Receiver) - 1);
          addFormTextBox(F("Subject"),  F("subject"),  NotificationSettings.Subject,  sizeof(NotificationSettings.Subject) - 1);

          addFormTextBox(F("User"),     F("user"),     NotificationSettings.User,     sizeof(NotificationSettings.User) - 1);
          addFormTextBox(F("Pass"),     F("pass"),     NotificationSettings.Pass,     sizeof(NotificationSettings.Pass) - 1);

          addRowLabel(F("Body"));
          addHtml(F("<textarea name='body' rows='20' size=512 wrap='off'>"));
          addHtml(NotificationSettings.Body);
          addHtml(F("</textarea>"));
        }

        if (Notification[NotificationProtocolIndex].usesGPIO > 0)
        {
          addRowLabel(F("1st GPIO"));
          addPinSelect(false, "pin1", NotificationSettings.Pin1);
        }

        addRowLabel(F("Enabled"));
        addCheckBox(F("notificationenabled"), Settings.NotificationEnabled[notificationindex]);

        TempEvent.NotificationIndex = notificationindex;
        String webformLoadString;
        NPlugin_ptr[NotificationProtocolIndex](NPlugin::Function::NPLUGIN_WEBFORM_LOAD, &TempEvent, webformLoadString);

        if (webformLoadString.length() > 0) {
          addHtmlError(F("Bug in NPlugin::Function::NPLUGIN_WEBFORM_LOAD, should not append to string, use addHtml() instead"));
        }
      }
    }

    addFormSeparator(2);

    html_TR_TD();
    html_TD();
    addButton(F("notifications"), F("Close"));
    addSubmitButton();
    addSubmitButton(F("Test"), F("test"));
    html_end_table();
    html_end_form();
  }
  sendHeadandTail_stdtemplate(_TAIL);
  TXBuffer.endStream();
}

#endif // USES_NOTIFIER

#include "../WebServer/PinStates.h"

#include "../WebServer/WebServer.h"
#include "../WebServer/HTML_wrappers.h"

#include "../DataStructs/PinMode.h"
#include "../Globals/GlobalMapPortStatus.h"
#include "../Helpers/PortStatus.h"

#ifdef WEBSERVER_NEW_UI


// ********************************************************************************
// Web Interface pin state list
// ********************************************************************************
void handle_pinstates_json() {
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("handle_pinstates"));
  #endif

  if (!isLoggedIn()) { return; }
  navMenuIndex = MENU_INDEX_TOOLS;
  TXBuffer.startJsonStream();

  bool first = true;
  addHtml(F("["));

  for (std::map<uint32_t, portStatusStruct>::iterator it = globalMapPortStatus.begin(); it != globalMapPortStatus.end(); ++it)
  {
    if (!first) {
      addHtml(",");
    } else {
      first = false;
    }
    addHtml("{");


    const uint16_t plugin = getPluginFromKey(it->first);
    const uint16_t port   = getPortFromKey(it->first);

    stream_next_json_object_value(F("plugin"),  String(plugin));
    stream_next_json_object_value(F("port"),    String(port));
    stream_next_json_object_value(F("state"),   String(it->second.state));
    stream_next_json_object_value(F("task"),    String(it->second.task));
    stream_next_json_object_value(F("monitor"), String(it->second.monitor));
    stream_next_json_object_value(F("command"), String(it->second.command));
    stream_last_json_object_value(F("init"), String(it->second.init));
  }

  addHtml(F("]"));

  TXBuffer.endStream();
}

#endif // WEBSERVER_NEW_UI

#ifdef WEBSERVER_PINSTATES

void handle_pinstates() {
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("handle_pinstates"));
  #endif

  if (!isLoggedIn()) { return; }
  navMenuIndex = MENU_INDEX_TOOLS;
  TXBuffer.startStream();
  sendHeadandTail_stdtemplate(_HEAD);

  // addFormSubHeader(F("Pin state table<TR>"));

  html_table_class_multirow();
  html_TR();
  html_table_header(F("Plugin"), F("Official_plugin_list"), 0);
  html_table_header("GPIO");
  html_table_header("Mode");
  html_table_header(F("Value/State"));
  html_table_header(F("Task"));
  html_table_header(F("Monitor"));
  html_table_header(F("Command"));
  html_table_header("Init");

  for (std::map<uint32_t, portStatusStruct>::iterator it = globalMapPortStatus.begin(); it != globalMapPortStatus.end(); ++it)
  {
    html_TR_TD();
    addHtml("P");
    const uint16_t plugin = getPluginFromKey(it->first);
    const uint16_t port   = getPortFromKey(it->first);

    if (plugin < 100)
    {
      addHtml("0");
    }

    if (plugin < 10)
    {
      addHtml("0");
    }
    addHtml(String(plugin));
    html_TD();
    addHtml(String(port));
    html_TD();
    addHtml(getPinModeString(it->second.mode));
    html_TD();
    addHtml(String(it->second.getValue()));
    html_TD();
    addHtml(String(it->second.task));
    html_TD();
    addHtml(String(it->second.monitor));
    html_TD();
    addHtml(String(it->second.command));
    html_TD();
    addHtml(String(it->second.init));
  }

  html_end_table();
  sendHeadandTail_stdtemplate(_TAIL);
  TXBuffer.endStream();
}

#endif // ifdef WEBSERVER_PINSTATES

#include "../WebServer/RootPage.h"


#ifdef WEBSERVER_ROOT

#include "../WebServer/WebServer.h"
#include "../WebServer/HTML_wrappers.h"
#include "../WebServer/LoadFromFS.h"
#include "../WebServer/Markup.h"
#include "../WebServer/Markup_Buttons.h"

#include "../Commands/InternalCommands.h"
#include "../ESPEasyCore/ESPEasyNetwork.h"
#include "../Globals/ESPEasy_time.h"
#include "../Globals/ESPEasyWiFiEvent.h"
#include "../Globals/MainLoopCommand.h"
#include "../Globals/NetworkState.h"
#include "../Globals/Nodes.h"
#include "../Globals/Settings.h"
#include "../Globals/Statistics.h"
#include "../Helpers/ESPEasy_Storage.h"
#include "../Helpers/Memory.h"
#include "../Helpers/WebServer_commandHelper.h"

#include "../../ESPEasy_fdwdecl.h"
#include "../../ESPEasy-Globals.h"

// ********************************************************************************
// Web Interface root page
// ********************************************************************************
void handle_root() {
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("handle_root"));
  #endif

  // if Wifi setup, launch setup wizard
  if (WiFiEventData.wifiSetup)
  {
    web_server.send(200, F("text/html"), F("<meta HTTP-EQUIV='REFRESH' content='0; url=/setup'>"));
    return;
  }

  if (!isLoggedIn()) { return; }
  navMenuIndex = 0;

  // if index.htm exists on FS serve that one (first check if gziped version exists)
  if (loadFromFS(true, F("/index.htm.gz"))) { return; }
  #ifdef FEATURE_SD
  if (loadFromFS(false, F("/index.htm.gz"))) { return; }
  #endif

  if (loadFromFS(true, F("/index.htm"))) { return; }
  #ifdef FEATURE_SD
  if (loadFromFS(false, F("/index.htm"))) { return; }
  #endif

  TXBuffer.startStream();
  String  sCommand  = web_server.arg(F("cmd"));
  boolean rebootCmd = strcasecmp_P(sCommand.c_str(), PSTR("reboot")) == 0;
  sendHeadandTail_stdtemplate(_HEAD, rebootCmd);

  int freeMem = ESP.getFreeHeap();

  // TODO: move this to handle_tools, from where it is actually called?

  // have to disconnect or reboot from within the main loop
  // because the webconnection is still active at this point
  // disconnect here could result into a crash/reboot...
  if (strcasecmp_P(sCommand.c_str(), PSTR("wifidisconnect")) == 0)
  {
    addLog(LOG_LEVEL_INFO, F("WIFI : Disconnecting..."));
    cmd_within_mainloop = CMD_WIFI_DISCONNECT;
    addHtml(F("OK"));
  } else if (strcasecmp_P(sCommand.c_str(), PSTR("reboot")) == 0)
  {
    addLog(LOG_LEVEL_INFO, F("     : Rebooting..."));
    cmd_within_mainloop = CMD_REBOOT;
    addHtml(F("OK"));
  } else if (strcasecmp_P(sCommand.c_str(), PSTR("reset")) == 0)
  {
    addLog(LOG_LEVEL_INFO, F("     : factory reset..."));
    cmd_within_mainloop = CMD_REBOOT;
    addHtml(F(
              "OK. Please wait > 1 min and connect to Acces point.<BR><BR>PW=configesp<BR>URL=<a href='http://192.168.4.1'>192.168.4.1</a>"));
    TXBuffer.endStream();
    ExecuteCommand_internal(EventValueSource::Enum::VALUE_SOURCE_HTTP, sCommand.c_str());
    return;
  } else {
    handle_command_from_web(EventValueSource::Enum::VALUE_SOURCE_HTTP, sCommand);
    printToWeb     = false;
    printToWebJSON = false;

    addHtml(F("<form>"));
    html_table_class_normal();
    addFormHeader(F("System Info"));

    addRowLabelValue(LabelType::UNIT_NR);
    addRowLabelValue(LabelType::GIT_BUILD);
    addRowLabel(LabelType::LOCAL_TIME);

    if (node_time.systemTimePresent())
    {
      addHtml(getValue(LabelType::LOCAL_TIME));
    }
    else {
      addHtml(F("<font color='red'>No system time source</font>"));
    }

    addRowLabel(LabelType::UPTIME);
    {
      addHtml(getExtendedValue(LabelType::UPTIME));
    }
    addRowLabel(LabelType::LOAD_PCT);

    if (wdcounter > 0)
    {
      String html;
      html.reserve(32);
      html += getCPUload();
      html += F("% (LC=");
      html += getLoopCountPerSec();
      html += ')';
      addHtml(html);
    }
    {
      addRowLabel(LabelType::FREE_MEM);
      String html;
      html.reserve(64);
      html += freeMem;
      #ifndef BUILD_NO_RAM_TRACKER
      html += " (";
      html += lowestRAM;
      html += F(" - ");
      html += lowestRAMfunction;
      html += ')';
      #endif
      addHtml(html);
    }
    {
      addRowLabel(LabelType::FREE_STACK);
      String html;
      html.reserve(64);
      html += String(getCurrentFreeStack());
      #ifndef BUILD_NO_RAM_TRACKER
      html += " (";
      html += String(lowestFreeStack);
      html += F(" - ");
      html += String(lowestFreeStackfunction);
      html += ')';
      #endif
      addHtml(html);
    }

    addRowLabelValue(LabelType::IP_ADDRESS);
    addRowLabel(LabelType::WIFI_RSSI);

    if (NetworkConnected())
    {
      String html;
      html.reserve(32);
      html += String(WiFi.RSSI());
      html += F(" dB (");
      html += WiFi.SSID();
      html += ')';
      addHtml(html);
    }

#ifdef HAS_ETHERNET
    addRowLabelValue(LabelType::ETH_WIFI_MODE);
    if(active_network_medium == NetworkMedium_t::Ethernet) {
      addRowLabelValue(LabelType::ETH_SPEED_STATE);
      addRowLabelValue(LabelType::ETH_IP_ADDRESS);
    }
#endif

    #ifdef FEATURE_MDNS
    {
      addRowLabel(LabelType::M_DNS);
      String html;
      html.reserve(64);
      html += F("<a href='http://");
      html += getValue(LabelType::M_DNS);
      html += F("'>");
      html += getValue(LabelType::M_DNS);
      html += F("</a>");
      addHtml(html);
    }
    #endif // ifdef FEATURE_MDNS
    html_TR_TD();
    html_TD();
    addButton(F("sysinfo"), F("More info"));

    if (printWebString.length() > 0)
    {
      html_BR();
      html_BR();
      addFormHeader(F("Command Argument"));
      addRowLabel(F("Command"));
      addHtml(sCommand);

      addHtml(F("<TR><TD colspan='2'>Command Output<BR><textarea readonly rows='10' wrap='on'>"));
      addHtml(printWebString);
      addHtml(F("</textarea>"));
      printWebString = "";
    }
    html_end_table();

    html_BR();
    html_BR();
    html_table_class_multirow_noborder();
    html_TR();
    html_table_header(F("Node List"));
    html_table_header("Name");
    html_table_header(getLabel(LabelType::BUILD_DESC));
    html_table_header("Type");
    html_table_header("IP", 160); // Should fit "255.255.255.255"
    html_table_header("Age");

    for (NodesMap::iterator it = Nodes.begin(); it != Nodes.end(); ++it)
    {
      if (it->second.ip[0] != 0)
      {
        bool isThisUnit = it->first == Settings.Unit;

        if (isThisUnit) {
          html_TR_TD_highlight();
        }
        else {
          html_TR_TD();
        }

        addHtml(F("Unit "));
        addHtml(String(it->first));
        html_TD();

        if (isThisUnit) {
          addHtml(Settings.Name);
        }
        else {
          addHtml(it->second.nodeName);
        }
        html_TD();

        if (it->second.build) {
          addHtml(String(it->second.build));
        }
        html_TD();
        addHtml(getNodeTypeDisplayString(it->second.nodeType));
        html_TD();
        html_add_wide_button_prefix();
        {
          String html;
          html.reserve(64);

          html += F("http://");
          html += it->second.ip.toString();
          uint16_t port = it->second.webgui_portnumber;
          if (port !=0 && port != 80) {
            html += ':';
            html += String(port);
          }
          html += "'>";
          html += it->second.ip.toString();
          html += "</a>";
          addHtml(html);
        }
        html_TD();
        addHtml(String(it->second.age));
      }
    }

    html_end_table();
    html_end_form();

    printWebString = "";
    printToWeb     = false;
    sendHeadandTail_stdtemplate(_TAIL);
  }
  TXBuffer.endStream();
}

#endif // ifdef WEBSERVER_ROOT

// #define WEBSERVER_RULES_DEBUG

#include "../WebServer/Rules.h"

#include "../WebServer/WebServer.h"
#include "../WebServer/AccessControl.h"
#include "../WebServer/HTML_wrappers.h"
#include "../WebServer/Markup.h"
#include "../WebServer/Markup_Buttons.h"
#include "../WebServer/Markup_Forms.h"

#include "../ESPEasyCore/ESPEasyRules.h"

#include "../Globals/Settings.h"
#include "../Helpers/ESPEasy_Storage.h"
#include "../Helpers/StringConverter.h"
#include "../Static/WebStaticData.h"

#include <FS.h>

#ifdef WEBSERVER_RULES

// ********************************************************************************
// Web Interface rules page
// ********************************************************************************
void handle_rules() {
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("handle_rules"));
  #endif

  if (!isLoggedIn() || !Settings.UseRules) { return; }
  navMenuIndex = MENU_INDEX_RULES;
  const byte rulesSet = getFormItemInt(F("set"), 1);

  #if defined(ESP8266)
  String fileName = F("rules");
  #endif // if defined(ESP8266)
  #if defined(ESP32)
  String fileName = F("/rules");
  #endif // if defined(ESP32)
  fileName += rulesSet;
  fileName += F(".txt");

  String error;

  // Make sure file exists
  if (!fileExists(fileName))
  {
    if (loglevelActiveFor(LOG_LEVEL_INFO)) {
      String log = F("Rules : Create new file: ");
      log += fileName;
      addLog(LOG_LEVEL_INFO, log);
    }
    fs::File f = tryOpenFile(fileName, "w");

    if (f) { f.close(); }
  }

  TXBuffer.startStream();
  sendHeadandTail_stdtemplate();
  addHtmlError(error);

  html_table_class_normal();
  html_TR();
  html_table_header(F("Rules"));

  html_TR_TD();
  addHtml(F("<form name = 'frmselect'>"));
  {
    // Place combo box in its own scope to release these arrays as soon as possible
    byte   choice = rulesSet;
    String options[RULESETS_MAX];
    int    optionValues[RULESETS_MAX];

    for (byte x = 0; x < RULESETS_MAX; x++)
    {
      options[x]      = F("Rules Set ");
      options[x]     += x + 1;
      optionValues[x] = x + 1;
    }

    addSelector(F("set"), RULESETS_MAX, options, optionValues, NULL, choice, true, true);
    addHelpButton(F("Tutorial_Rules"));
    addRTDHelpButton(F("Rules/Rules.html"));
  }

  html_TR_TD();
  Rule_showRuleTextArea(fileName);

  html_TR_TD();
  html_end_form();
  addHtml(F("<button id='save_button' class='button' onClick='saveRulesFile()'>Save</button>"));
  addHtml(F("<div id='toastmessage'>Saved!</div>"));

  addButton(fileName, F("Download to file"));
  html_end_table();

  serve_JS(JSfiles_e::SaveRulesFile);

  sendHeadandTail_stdtemplate(true);
  TXBuffer.endStream();

  checkRuleSets();
}

// ********************************************************************************
// Web Interface rules page  (NEW)
// ********************************************************************************
void handle_rules_new() {
  if (!isLoggedIn() || !Settings.UseRules) { return; }

  if (!clientIPallowed()) { return; }

  if (Settings.OldRulesEngine())
  {
    handle_rules();
    return;
  }
  #ifdef WEBSERVER_NEW_RULES
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("handle_rules"));
  #endif
  navMenuIndex = 5;
  TXBuffer.startStream();
  sendHeadandTail(F("TmplStd"), _HEAD);

  // define macro
  #if defined(ESP8266)
  String rootPath = F("rules");
  #endif // if defined(ESP8266)
  #if defined(ESP32)

  String rootPath = F("/rules");
  #endif // if defined(ESP32)

  // Pagionation of rules list
  const int rulesListPageSize = 25;
  int startIdx                = 0;

  String fstart = web_server.arg(F("start"));

  if (fstart.length() > 0)
  {
    startIdx = atoi(fstart.c_str());
  }
  int endIdx = startIdx + rulesListPageSize - 1;

  // Build table header
  html_table_class_multirow();
  html_TR();
  html_table_header(F("Event Name"));
  html_table_header(F("Filename"));
  html_table_header(F("Size"));
  addHtml(F("<TH>Actions"));
  addSaveButton(F("/rules/backup"), F("Backup"));
  addHtml(F("</TH></TR>"));

  // class StreamingBuffer buffer = TXBuffer;

  // Build table detail
  int count = -1;
  HandlerFileInfo renderDetail = [/*&buffer,*/ &count, endIdx](fileInfo fi) 
  {
#ifdef WEBSERVER_RULES_DEBUG
    Serial.print(F("Start generation of: "));
    Serial.println(fi.Name);
#endif // ifdef WEBSERVER_RULES_DEBUG

    if (fi.isDirectory)
    {
      html_TR_TD();
    }
    else
    {
      count++;
      addHtml(F("<TR><TD style='text-align:right'>"));
    }

    // Event Name
    addHtml(FileNameToEvent(fi.Name));

    if (fi.isDirectory)
    {
      addHtml(F("</TD><TD></TD><TD></TD><TD>"));
      addSaveButton(String(F("/rules/backup?directory=")) + URLEncode(fi.Name.c_str())
                    , F("Backup")
                    );
    }
    else
    {
      String encodedPath =  URLEncode((fi.Name + F(".txt")).c_str());

      // File Name
      {
        String html;
        html.reserve(128);

        html += F("</TD><TD><a href='");
        html += fi.Name;
        html += F(".txt");
        html += "'>";
        html += fi.Name;
        html += F(".txt");
        html += F("</a></TD>");

        // File size
        html += F("<TD>");
        html += fi.Size;
        html += F("</TD>");
        addHtml(html);
      }

      // Actions
      html_TD();
      addSaveButton(String(F("/rules/backup?fileName=")) + encodedPath
                    , F("Backup")
                    );

      addDeleteButton(String(F("/rules/delete?fileName=")) + encodedPath
                      , F("Delete")
                      );
    }
    addHtml(F("</TD></TR>"));
#ifdef WEBSERVER_RULES_DEBUG
    Serial.print(F("End generation of: "));
    Serial.println(fi.Name);
#endif // ifdef WEBSERVER_RULES_DEBUG

    return count < endIdx;
  };


  bool hasMore = EnumerateFileAndDirectory(rootPath
                                           , startIdx
                                           , renderDetail);
  html_TR_TD();
  addButton(F("/rules/add"), F("Add"));
  addHtml(F("</TD><TD></TD><TD></TD><TD></TD></TR>"));
  addHtml(F("</table>"));

  if (startIdx > 0)
  {
    int showIdx = startIdx - rulesListPageSize;

    if (showIdx < 0) { showIdx = 0; }
    addButton(String(F("/rules?start=")) + String(showIdx)
              , F("Previous"));
  }

  if (hasMore && (count >= endIdx))
  {
    addButton(String(F("/rules?start=")) + String(endIdx + 1)
              , F("Next"));
  }

  // TXBuffer += F("<BR><BR>");
  sendHeadandTail(F("TmplStd"), _TAIL);
  TXBuffer.endStream();
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("handle_rules"));
  #endif
  #endif // WEBSERVER_NEW_RULES
}

void handle_rules_backup() {
  if (Settings.OldRulesEngine())
  {
    Goto_Rules_Root();
    return;
  }
  #ifdef WEBSERVER_NEW_RULES
  #ifdef WEBSERVER_RULES_DEBUG
  Serial.println(F("handle rules backup"));
  #endif // ifdef WEBSERVER_RULES_DEBUG

  if (!isLoggedIn() || !Settings.UseRules) { return; }

  if (!clientIPallowed()) { return; }
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("handle_rules_backup"));
  #endif
  String directory = web_server.arg(F("directory"));
  String fileName  = web_server.arg(F("fileName"));
  String error;

  if (directory.length() > 0)
  {
    HandlerFileInfo downloadHandler = [&error](fileInfo fi) {
                                        if (!fi.isDirectory)
                                        {
                                          if (!Rule_Download(fi.Name))
                                          {
                                            error += String(F("Invalid path: ")) + fi.Name;
                                          }
                                        }
                                        return true;
                                      };
    EnumerateFileAndDirectory(directory
                              , 0
                              , downloadHandler);
  }
  else if (fileName.length() > 0) {
    fileName = fileName.substring(0, fileName.length() - 4);

    if (!Rule_Download(fileName))
    {
      error = String(F("Invalid path: ")) + fileName;
    }
  }
  else
  {
    error = F("Invalid parameters");
  }

  if (error.length() > 0) {
    TXBuffer.startStream();
    sendHeadandTail(F("TmplMsg"), _HEAD);
    addHtmlError(error);
    TXBuffer.endStream();
  }
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("handle_rules_backup"));
  #endif
  #endif // WEBSERVER_NEW_RULES
}

void handle_rules_delete() {
  if (!isLoggedIn() || !Settings.UseRules) { return; }

  if (!clientIPallowed()) { return; }

  if (Settings.OldRulesEngine())
  {
    Goto_Rules_Root();
    return;
  }
  #ifdef WEBSERVER_NEW_RULES
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("handle_rules_delete"));
  #endif
  String fileName = web_server.arg(F("fileName"));
  fileName = fileName.substring(0, fileName.length() - 4);
  bool removed = false;
  #ifdef WEBSERVER_RULES_DEBUG
  Serial.println(F("handle_rules_delete"));
  Serial.print(F("File name: "));
  Serial.println(fileName);
  #endif // ifdef WEBSERVER_RULES_DEBUG

  if (fileName.length() > 0)
  {
    removed = tryDeleteFile(fileName);
  }

  if (removed)
  {
    web_server.sendHeader(F("Location"), F("/rules"), true);
    web_server.send(302, F("text/plain"), F(""));
  }
  else
  {
    String error = String(F("Delete rule Invalid path: ")) + fileName;
    addLog(LOG_LEVEL_ERROR, error);
    TXBuffer.startStream();
    sendHeadandTail(F("TmplMsg"), _HEAD);
    addHtmlError(error);
    sendHeadandTail(F("TmplMsg"), _TAIL);
    TXBuffer.endStream();
  }
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("handle_rules_delete"));
  #endif
  #endif // WEBSERVER_NEW_RULES
}

bool handle_rules_edit(const String& originalUri)
{
  return handle_rules_edit(originalUri, false);
}

bool handle_rules_edit(String originalUri, bool isAddNew) {
  // originalUri is passed via deepcopy, since it will be converted to lower case.
  if (!isLoggedIn() || !Settings.UseRules) { return false; }
  originalUri.toLowerCase();
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("handle_rules_edit"));
  #endif
  bool handle = false;

  #ifdef WEBSERVER_RULES_DEBUG
  Serial.println(originalUri);
  Serial.println(F("handle_rules_edit"));
  #endif // ifdef WEBSERVER_RULES_DEBUG

  if (isAddNew || (originalUri.startsWith(F("/rules/"))
                   && originalUri.endsWith(F(".txt")))) {
    if (Settings.OldRulesEngine())
    {
      Goto_Rules_Root();
      return true;
    }
    #ifdef WEBSERVER_NEW_RULES

    String eventName;
    String fileName;
    bool   isOverwrite = false;
    bool   isNew       = false;
    String error;


    if (isAddNew)
    {
      eventName = web_server.arg(F("eventName"));
      fileName += EventToFileName(eventName);
    }
    else
    {
        #if defined(ESP8266)
      fileName = F("rules/");
        #endif // if defined(ESP8266)
        #if defined(ESP32)
      fileName = F("/rules/");
        #endif // if defined(ESP32)
      fileName += originalUri.substring(7, originalUri.length() - 4);
      eventName = FileNameToEvent(fileName);
    }
      #ifdef WEBSERVER_RULES_DEBUG
    Serial.print(F("File name: "));
    Serial.println(fileName);
      #endif // ifdef WEBSERVER_RULES_DEBUG
    bool isEdit = fileExists(fileName);

    if (web_server.args() > 0)
    {
      const String& rules = web_server.arg(F("rules"));
      isNew = web_server.arg(F("IsNew")) == F("yes");

      // Overwrite verification
      if (isEdit && isNew) {
        error = String(F("There is another rule with the same name: "))
                + fileName;
        addLog(LOG_LEVEL_ERROR, error);
        isAddNew    = true;
        isOverwrite = true;
      }
      else if (!web_server.hasArg(F("rules")))
      {
        error = F("Data was not saved, rules argument missing or corrupted");
        addLog(LOG_LEVEL_ERROR, error);
      }

      // Check rules size
      else if (rules.length() > RULES_MAX_SIZE)
      {
        error = String(F("Data was not saved, exceeds web editor limit! "))
                + fileName;
        addLog(LOG_LEVEL_ERROR, error);
      }

      // Passed all checks, write file
      else
      {
        fs::File f = tryOpenFile(fileName, "w");

        if (f)
        {
          addLog(LOG_LEVEL_INFO, String(F(" Write to file: ")) + fileName);
          f.print(rules);
          f.close();
        }

        if (isAddNew) {
          web_server.sendHeader(F("Location"), F("/rules"), true);
          web_server.send(302, F("text/plain"), F(""));
          return true;
        }
      }
    }
    navMenuIndex = 5;
    TXBuffer.startStream();
    sendHeadandTail(F("TmplStd"));

    if (error.length() > 0) {
      addHtmlError(error);
    }
    addHtml(F("<form name = 'editRule' method = 'post'><table class='normal'><TR><TH align='left' colspan='2'>Edit Rule"));

    // hidden field to check Overwrite
    addHtml(F("<input "));
    addHtmlAttribute(F("type"), F("hidden"));
    addHtmlAttribute(F("id"), F("IsNew"));
    addHtmlAttribute(F("name"), F("IsNew"));
    addHtmlAttribute(F("value"), isAddNew ? F("yes") : F("no"));
    addHtml('>');

    bool isReadOnly = !isOverwrite && ((isEdit && !isAddNew && !isNew) || (isAddNew && isNew));
      #ifdef WEBSERVER_RULES_DEBUG
    Serial.print(F("Is Overwrite: "));
    Serial.println(isOverwrite);
    Serial.print(F("Is edit: "));
    Serial.println(isEdit);
    Serial.print(F("Is addnew: "));
    Serial.println(isAddNew);
    Serial.print(F("Is New: "));
    Serial.println(isNew);
    Serial.print(F("Is Read Only: "));
    Serial.println(isReadOnly);
      #endif // ifdef WEBSERVER_RULES_DEBUG

    addFormTextBox(F("Event name")            // Label
                   , F("eventName")           // field name
                   , eventName                // field value
                   , RULE_MAX_FILENAME_LENGTH // max length
                   , isReadOnly               // is readonly
                   , isAddNew                 // required
                   , F("[A-Za-z]+#.+")        // validation pattern
                   );
    addHelpButton(F("Tutorial_Rules"));

    // load form data from flash
    addHtml(F("<TR><TD colspan='2'>"));

    Rule_showRuleTextArea(fileName);

    addFormSeparator(2);
    html_TR_TD();
    addSubmitButton();

    addHtml(F("</table></form>"));

    sendHeadandTail(F("TmplStd"), true);
    TXBuffer.endStream();
    #endif // WEBSERVER_NEW_RULES
  }
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("handle_rules_edit2"));
  #endif
  return handle;
}

void Rule_showRuleTextArea(const String& fileName) {
  // Read rules from file and stream directly into the textarea

  size_t size = 0;

  addHtml(F("<textarea id='rules' name='rules' rows='30' wrap='off'>"));
  size = streamFile_htmlEscape(fileName);
  addHtml(F("</textarea>"));

  html_TR_TD();
  {
    String html;
    html.reserve(64);

    html += F("Current size: <span id='size'>");
    html += size;
    html += F("</span> characters (Max ");
    html += RULES_MAX_SIZE;
    html += F(")");
    addHtml(html);
  }

  if (size > RULES_MAX_SIZE) {
    addHtml(F("<span style=\"color:red\">Filesize exceeds web editor limit!</span>"));
  }
}

bool Rule_Download(const String& path)
{
  #ifdef WEBSERVER_RULES_DEBUG
  Serial.print(F("Rule_Download path: "));
  Serial.println(path);
  #endif // ifdef WEBSERVER_RULES_DEBUG
  fs::File dataFile = tryOpenFile(path, "r");

  if (!dataFile)
  {
    addLog(LOG_LEVEL_ERROR, String(F("Invalid path: ")) + path);
    return false;
  }
  String filename = path + String(F(".txt"));
  filename.replace(RULE_FILE_SEPARAROR, '_');
  String str = String(F("attachment; filename=")) + filename;
  web_server.sendHeader(F("Content-Disposition"), str);
  web_server.sendHeader(F("Cache-Control"),       F("max-age=3600, public"));
  web_server.sendHeader(F("Vary"),                "*");
  web_server.sendHeader(F("ETag"),                F("\"2.0.0\""));

  web_server.streamFile(dataFile, F("application/octet-stream"));
  dataFile.close();
  return true;
}

void Goto_Rules_Root() {
  web_server.sendHeader(F("Location"), F("/rules"), true);
  web_server.send(302, F("text/plain"), F(""));
}

bool EnumerateFileAndDirectory(String          & rootPath
                               , int             skip
                               , HandlerFileInfo handler)
{
  bool hasMore = false;
  int  count   = 0;
  bool next    = true;

  #ifdef ESP8266
  fs::Dir dir = ESPEASY_FS.openDir(rootPath);
  Serial.print(F("Enumerate files of "));
  Serial.println(rootPath);

  while (next && dir.next()) {
    // Skip files
    if (count++ < skip) {
      continue;
    }

    // Workaround for skipped unwanted files
    if (!dir.fileName().startsWith(rootPath + "/")) {
      continue;
    }
    fileInfo fi;
    fi.Name = dir.fileName() /*.substring(rootPath.length())*/;
    fs::File f = dir.openFile("r");
    fi.Size = f.size();
    f.close();
    next = handler(fi);
  }
  hasMore = dir.next();
  #endif // ifdef ESP8266
  #ifdef ESP32
  File root = ESPEASY_FS.open(rootPath);

  if (root)
  {
    File file = root.openNextFile();

    while (next && file) {
      if (count >= skip) {
        fileInfo fi;
        fi.Name        = file.name();
        fi.Size        = file.size();
        fi.isDirectory = file.isDirectory();
        next           = handler(fi);
      }

      if (!file.isDirectory()) {
        ++count;
      }
      file = root.openNextFile();
    }
    hasMore = file;
  }
  else
  {
    addLog(LOG_LEVEL_ERROR, F("Invalid root."));
  }
  #endif // ifdef ESP32
  return hasMore;
}

#endif // ifdef WEBSERVER_RULES

#include "../WebServer/SettingsArchive.h"

#ifdef USE_SETTINGS_ARCHIVE

#include "../WebServer/WebServer.h"
#include "../WebServer/HTML_wrappers.h"
#include "../WebServer/Markup.h"
#include "../WebServer/Markup_Buttons.h"
#include "../WebServer/Markup_Forms.h"

#include "../Globals/ResetFactoryDefaultPref.h"
#include "../Helpers/ESPEasy_FactoryDefault.h"
#include "../Helpers/ESPEasy_Storage.h"
#include "../Helpers/Misc.h"
#include "../Helpers/Networking.h"


// ********************************************************************************
// Web Interface to manage archived settings
// ********************************************************************************
void handle_settingsarchive() {
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("handle_settingsarchive"));
  #endif

  if (!isLoggedIn()) { return; }
  navMenuIndex = MENU_INDEX_TOOLS;
  TXBuffer.startStream();
  sendHeadandTail_stdtemplate(_HEAD);
  html_add_form();
  html_table_class_normal();
  html_TR();
  addFormHeader(F("Settings Archive"));

  if (web_server.hasArg(F("savepref")) || web_server.hasArg(F("download"))) {
    // User choose a pre-defined config and wants to save it as the new default.
    storeDownloadFiletypeCheckbox(FileType::CONFIG_DAT);
    storeDownloadFiletypeCheckbox(FileType::SECURITY_DAT);
    storeDownloadFiletypeCheckbox(FileType::NOTIFICATION_DAT);

    for (int i = 0; i < 4; ++i) {
      storeDownloadFiletypeCheckbox(FileType::RULES_TXT, i);
    }

    ResetFactoryDefaultPreference.deleteFirst(isFormItemChecked("del"));

    applyFactoryDefaultPref();
    addHtmlError(SaveSettings());
  }

  bool showOptions = true;

  if (web_server.hasArg(F("download"))) {
    // Try downloading files.
    String url  = web_server.arg(F("url"));
    String user = web_server.arg(F("user"));
    String pass = web_server.arg(F("pass"));

    addTableSeparator(F("Download result"), 2, 3);
    bool somethingDownloaded = false;

    if (tryDownloadFileType(url, user, pass, FileType::CONFIG_DAT)) { somethingDownloaded = true; }

    if (tryDownloadFileType(url, user, pass, FileType::SECURITY_DAT)) { somethingDownloaded = true; }

    if (tryDownloadFileType(url, user, pass, FileType::NOTIFICATION_DAT)) { somethingDownloaded = true; }

    for (int i = 0; i < 4; ++i) {
      if (tryDownloadFileType(url, user, pass, FileType::RULES_TXT, i)) { somethingDownloaded = true; }
    }

    if (somethingDownloaded) {
      showOptions = false;
      html_TR_TD();
      html_TD();
      addSubmitButton(F("Reboot"), F("reboot"), F("red"));
      addFormNote(F("If settings files are updated you MUST reboot first!"));
    }
  } else if (web_server.hasArg(F("reboot"))) {
    showOptions = false;
    reboot(ESPEasy_Scheduler::IntendedRebootReason_e::RestoreSettings);
  }

  if (showOptions) {
    // Nothing chosen yet, show options.

    addTableSeparator(F("Files to Download"), 2, 3);
    addDownloadFiletypeCheckbox(FileType::CONFIG_DAT);
    addDownloadFiletypeCheckbox(FileType::SECURITY_DAT);
    addDownloadFiletypeCheckbox(FileType::NOTIFICATION_DAT);

    for (int i = 0; i < 4; ++i) {
      addDownloadFiletypeCheckbox(FileType::RULES_TXT, i);
    }

    addTableSeparator(F("Download Settings"), 2, 3);

    addRowLabel(F("Delete First"));
    addCheckBox("del", ResetFactoryDefaultPreference.deleteFirst());
    addFormNote(F("Needed on filesystem with not enough free space. Use with care!"));


    html_TR_TD();
    html_TD();
    addSubmitButton(F("Save Preferences"), F("savepref"));

    addTableSeparator(F("Archive Location"), 2, 3);

    addFormTextBox(F("URL with settings"), F("url"), web_server.arg(F("url")), 256);
    addFormNote(F("Only HTTP supported. Do not include filename"));
    addFormTextBox(F("User"), F("user"), web_server.arg(F("user")), 64);
    addFormPasswordBox(F("Pass"), F("pass"), web_server.arg(F("pass")), 64);
    addFormNote(F("URL, user and pass will not be stored"));

    addRowLabel(F("Try download files"));
    addSubmitButton(F("Download"), F("download"), F("red"));
  }

  html_end_table();
  html_end_form();
  sendHeadandTail_stdtemplate(_TAIL);
  TXBuffer.endStream();
}

// ********************************************************************************
// download filetype selectors
// ********************************************************************************
bool getDownloadFiletypeChecked(FileType::Enum filetype, unsigned int filenr) {
  bool isChecked = false;

  switch (filetype) {
    case FileType::CONFIG_DAT: isChecked       = ResetFactoryDefaultPreference.fetchConfigDat(); break;
    case FileType::SECURITY_DAT: isChecked     = ResetFactoryDefaultPreference.fetchSecurityDat(); break;
    case FileType::NOTIFICATION_DAT: isChecked = ResetFactoryDefaultPreference.fetchNotificationDat(); break;
    case FileType::RULES_TXT: isChecked        = ResetFactoryDefaultPreference.fetchRulesTXT(filenr); break;
  }
  return isChecked;
}

void addDownloadFiletypeCheckbox(FileType::Enum filetype, unsigned int filenr) {
  String filetype_str = getFileName(filetype, filenr);
  String label        = F("Fetch ");

  label += filetype_str;
  addRowLabel(label);
  addCheckBox(filetype_str, getDownloadFiletypeChecked(filetype, filenr));
}

void storeDownloadFiletypeCheckbox(FileType::Enum filetype, unsigned int filenr) {
  String filetype_str = getFileName(filetype, filenr);
  bool   isChecked    = isFormItemChecked(filetype_str);

  switch (filetype) {
    case FileType::CONFIG_DAT: ResetFactoryDefaultPreference.fetchConfigDat(isChecked); break;
    case FileType::SECURITY_DAT: ResetFactoryDefaultPreference.fetchSecurityDat(isChecked); break;
    case FileType::NOTIFICATION_DAT: ResetFactoryDefaultPreference.fetchNotificationDat(isChecked); break;
    case FileType::RULES_TXT: { ResetFactoryDefaultPreference.fetchRulesTXT(filenr, isChecked); break; }
  }
}

bool tryDownloadFileType(const String& url, const String& user, const String& pass, FileType::Enum filetype, unsigned int filenr) {
  if (!getDownloadFiletypeChecked(filetype, filenr)) {
    // Not selected, so not downloaded
    return false;
  }
  String filename = getFileName(filetype, filenr);
  bool   res      = false;
  String error;

  addRowLabel(filename);

  if (ResetFactoryDefaultPreference.deleteFirst()) {
    if (!fileExists(filename) || tryDeleteFile(filename)) {
      res = downloadFile(url + filename, filename, user, pass, error);
    } else {
      error = F("Could not delete existing file");
    }
  } else {
    String tmpfile = filename;
    tmpfile += F(".tmp");
    res      = downloadFile(url + filename, tmpfile, user, pass, error);

    if (res) {
      String filename_bak = filename;
      filename_bak += F("_bak");

      if (fileExists(filename) && !tryRenameFile(filename, filename_bak)) {
        res   = false;
        error = F("Could not rename to _bak");
      } else {
        // File does not exist (anymore)
        if (!tryRenameFile(tmpfile, filename)) {
          res   = false;
          error = F("Could not rename tmp file");

          if (tryRenameFile(filename_bak, filename)) {
            error += F("... reverted");
          } else {
            error += F(" Not reverted!");
          }
        }
      }
    }
  }

  if (!res) {
    addHtml(error);
  }  else {
    addHtml(F("Success"));
  }
  return res;
}

#endif // ifdef USE_SETTINGS_ARCHIVE

#include "../WebServer/SetupPage.h"


#ifdef WEBSERVER_SETUP

# include "../WebServer/WebServer.h"
# include "../WebServer/AccessControl.h"
# include "../WebServer/HTML_wrappers.h"
# include "../WebServer/Markup.h"
# include "../WebServer/Markup_Buttons.h"
# include "../WebServer/Markup_Forms.h"
# include "../WebServer/SysInfoPage.h"

# include "../ESPEasyCore/ESPEasyNetwork.h"
# include "../ESPEasyCore/ESPEasyWifi.h"


# include "../Globals/ESPEasyWiFiEvent.h"
# include "../Globals/SecuritySettings.h"

# include "../Helpers/Networking.h"
# include "../Helpers/ESPEasy_Storage.h"
# include "../Helpers/StringConverter.h"


// ********************************************************************************
// Web Interface Setup Wizard
// ********************************************************************************

# define HANDLE_SETUP_SCAN_STAGE       0
# define HANDLE_SETUP_CONNECTING_STAGE 1

void handle_setup() {
  # ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("handle_setup"));
  # endif // ifndef BUILD_NO_RAM_TRACKER

  // Do not check client IP range allowed.
  TXBuffer.startStream();

  if (!NetworkConnected())
  {
    sendHeadandTail(F("TmplAP"));
    static byte status       = 0;
    static byte refreshCount = 0;
    String ssid              = web_server.arg(F("ssid"));
    String other             = web_server.arg(F("other"));
    String password          = web_server.arg(F("pass"));

    if (other.length() != 0)
    {
      ssid = other;
    }

    // if ssid config not set and params are both provided
    if ((status == 0) && (ssid.length() != 0) /*&& strcasecmp(SecuritySettings.WifiSSID, "ssid") == 0 */)
    {
      safe_strncpy(SecuritySettings.WifiKey,  password.c_str(), sizeof(SecuritySettings.WifiKey));
      safe_strncpy(SecuritySettings.WifiSSID, ssid.c_str(),     sizeof(SecuritySettings.WifiSSID));
      WiFiEventData.wifiSetupConnect         = true;
      WiFiEventData.wifiConnectAttemptNeeded = true;

      if (loglevelActiveFor(LOG_LEVEL_INFO)) {
        String reconnectlog = F("WIFI : Credentials Changed, retry connection. SSID: ");
        reconnectlog += ssid;
        addLog(LOG_LEVEL_INFO, reconnectlog);
      }
      status       = HANDLE_SETUP_CONNECTING_STAGE;
      refreshCount = 0;
    }
    html_BR();
    wrap_html_tag(F("h1"), F("Wifi Setup wizard"));
    html_add_form();

    switch (status) {
      case HANDLE_SETUP_SCAN_STAGE:
      {
        // first step, scan and show access points within reach...
        handle_setup_scan_and_show(ssid, other, password);
        break;
      }
      case  HANDLE_SETUP_CONNECTING_STAGE:
      {
        if (!handle_setup_connectingStage(refreshCount)) {
          status = HANDLE_SETUP_SCAN_STAGE;
        }
        ++refreshCount;
        break;
      }
    }
    html_end_form();
    sendHeadandTail(F("TmplAP"), true);
  } else {
    // Connect Success
    handle_setup_finish();
  }

  TXBuffer.endStream();
  delay(10);
}

void handle_setup_scan_and_show(const String& ssid, const String& other, const String& password) {
  if (WiFi.scanComplete() <= 0) {
    WiFiMode_t cur_wifimode = WiFi.getMode();
    WifiScan(false, false);
    setWifiMode(cur_wifimode);
  }

  const int8_t scanCompleteStatus = WiFi.scanComplete();

  if (scanCompleteStatus <= 0) {
    addHtml(F("No Access Points found"));
  }
  else
  {
    html_table_class_multirow();
    html_TR();
    html_table_header(F("Pick"), 50);
    html_table_header(F("Network info"));
    html_table_header(F("RSSI"), 50);

    for (int i = 0; i < scanCompleteStatus; ++i)
    {
      html_TR_TD();
      addHtml(F("<label class='container2'>"));
      addHtml(F("<input type='radio' name='ssid' value='"));
      {
        String escapeBuffer = WiFi.SSID(i);
        htmlStrongEscape(escapeBuffer);
        addHtml(escapeBuffer);
      }
      addHtml("'");

      if (WiFi.SSID(i) == ssid) {
        addHtml(F(" checked "));
      }
      addHtml(F("><span class='dotmark'></span></label><TD>"));
      int32_t rssi = 0;
      addHtml(formatScanResult(i, "<BR>", rssi));
      html_TD();
      getWiFi_RSSI_icon(rssi, 45);
    }
    html_end_table();
  }

  html_BR();
  addHtml(F("<label "));
  addHtmlAttribute(F("class"), F("container2"));
  addHtml('>');
  addHtml(F("other SSID:"));
  addHtml(F("<input "));
  addHtmlAttribute(F("type"),  F("radio"));
  addHtmlAttribute(F("name"),  F("ssid"));
  addHtmlAttribute(F("id"),    F("other_ssid"));
  addHtmlAttribute(F("value"), F("other"));
  addHtml('>');
  addHtml(F("<span class='dotmark'></span></label>"));

  addHtml(F("<input "));
  addHtmlAttribute(F("class"), F("wide"));
  addHtmlAttribute(F("type"),  F("text"));
  addHtmlAttribute(F("name"),  F("other"));
  addHtmlAttribute(F("value"), other);
  addHtml('>');
  html_BR();
  html_BR();

  addFormSeparator(2);

  html_BR();

  addHtml(F("Password:"));
  html_BR();
  addHtml(F("<input "));
  addHtmlAttribute(F("class"), F("wide"));
  addHtmlAttribute(F("type"),  F("text"));
  addHtmlAttribute(F("name"),  F("pass"));
  addHtmlAttribute(F("value"), password);
  addHtml('>');
  html_BR();
  html_BR();

  addSubmitButton(F("Connect"), "");
}

bool handle_setup_connectingStage(byte refreshCount) {
  if (refreshCount > 0)
  {
    //      safe_strncpy(SecuritySettings.WifiSSID, "ssid", sizeof(SecuritySettings.WifiSSID));
    //      SecuritySettings.WifiKey[0] = 0;
    addButton(F("/setup"), F("Back to Setup"));
    html_BR();
    WiFiEventData.wifiSetupConnect = false;
    return false;
  }
  int wait = WIFI_RECONNECT_WAIT / 1000;

  if (refreshCount != 0) {
    wait = 3;
  }
  addHtml(F("Please wait for <h1 id='countdown'>20..</h1>"));
  addHtml(F("<script type='text/JavaScript'>"));
  addHtml(F("function timedRefresh(timeoutPeriod) {"));
  addHtml(F("var timer = setInterval(function() {"));
  addHtml(F("if (timeoutPeriod > 0) {"));
  addHtml(F("timeoutPeriod -= 1;"));
  addHtml(F("document.getElementById('countdown').innerHTML = timeoutPeriod + '..' + '<br />';"));
  addHtml(F("} else {"));
  addHtml(F("clearInterval(timer);"));
  addHtml(F("window.location.href = window.location.href;"));
  addHtml(F("};"));
  addHtml(F("}, 1000);"));
  addHtml(F("};"));
  addHtml(F("timedRefresh("));
  addHtml(String(wait));
  addHtml(F(");"));
  html_add_script_end();
  addHtml(F("seconds while trying to connect"));
  return true;
}

void handle_setup_finish() {
  navMenuIndex = MENU_INDEX_TOOLS;
  sendHeadandTail_stdtemplate(_HEAD);
  addHtmlError(SaveSettings());
  html_add_form();
  html_table_class_normal();
  html_TR();

  addFormHeader(F("WiFi Setup Complete"));

  handle_sysinfo_Network();

  addFormSeparator(2);

  html_TR_TD();
  html_TD();

  if (!clientIPinSubnet()) {
    String host = formatIP(NetworkLocalIP());
    String url  = F("http://");
    url += host;
    url += F("/config");
    addButton(url, host);
  }
  html_end_table();
  html_end_form();

  WiFiEventData.wifiSetup = false;
  sendHeadandTail_stdtemplate(_TAIL);
}

#endif // ifdef WEBSERVER_SETUP

#include "../WebServer/SysInfoPage.h"

#include "../WebServer/WebServer.h"
#include "../WebServer/HTML_wrappers.h"
#include "../WebServer/Markup.h"
#include "../WebServer/Markup_Buttons.h"

#include "../../ESPEasy_fdwdecl.h"
#include "../../ESPEasy-Globals.h"

#include "../Commands/Diagnostic.h"

#include "../DataStructs/RTCStruct.h"

#include "../ESPEasyCore/ESPEasyNetwork.h"

#include "../Globals/CRCValues.h"
#include "../Globals/ESPEasy_time.h"
#include "../Globals/NetworkState.h"
#include "../Globals/RTC.h"

#include "../Helpers/CompiletimeDefines.h"
#include "../Helpers/ESPEasyStatistics.h"
#include "../Helpers/ESPEasy_Storage.h"
#include "../Helpers/Hardware.h"
#include "../Helpers/Memory.h"
#include "../Helpers/OTA.h"
#include "../Helpers/StringConverter.h"
#include "../Helpers/StringGenerator_GPIO.h"
#include "../Helpers/StringGenerator_System.h"

#include "../Static/WebStaticData.h"

#ifdef ESP32
# include <esp_partition.h>
#endif // ifdef ESP32


#ifdef WEBSERVER_NEW_UI

// ********************************************************************************
// Web Interface sysinfo page
// ********************************************************************************
void handle_sysinfo_json() {
  # ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("handle_sysinfo"));
  # endif // ifndef BUILD_NO_RAM_TRACKER

  if (!isLoggedIn()) { return; }
  TXBuffer.startJsonStream();
  json_init();
  json_open();
  json_open(false, F("general"));
  json_number(F("unit"), String(Settings.Unit));
  json_prop(F("time"),   node_time.getDateTimeString('-', ':', ' '));
  json_prop(F("uptime"), getExtendedValue(LabelType::UPTIME));
  json_number(F("cpu_load"),   String(getCPUload()));
  json_number(F("loop_count"), String(getLoopCountPerSec()));
  json_close();

  int freeMem = ESP.getFreeHeap();
  json_open(false, F("mem"));
  json_number(F("free"),    String(freeMem));
  json_number(F("low_ram"), String(
  # ifndef BUILD_NO_RAM_TRACKER
                lowestRAM
  # else // ifndef BUILD_NO_RAM_TRACKER
                0
  # endif // ifndef BUILD_NO_RAM_TRACKER
                ));
  json_prop(F("low_ram_fn"),
  # ifndef BUILD_NO_RAM_TRACKER
            lowestRAMfunction
  # else // ifndef BUILD_NO_RAM_TRACKER
            0
  # endif // ifndef BUILD_NO_RAM_TRACKER
            );
  json_number(F("stack"),     String(getCurrentFreeStack()));
  json_number(F("low_stack"), String(
  # ifndef BUILD_NO_RAM_TRACKER
                lowestFreeStack
  # else // ifndef BUILD_NO_RAM_TRACKER
                0
  # endif // ifndef BUILD_NO_RAM_TRACKER
                ));
  json_prop(F("low_stack_fn"),
  # ifndef BUILD_NO_RAM_TRACKER
            lowestFreeStackfunction
  # else // ifndef BUILD_NO_RAM_TRACKER
            0
  # endif // ifndef BUILD_NO_RAM_TRACKER
            );
  json_close();
  json_open(false, F("boot"));
  json_prop(F("last_cause"), getLastBootCauseString());
  json_number(F("counter"), String(RTC.bootCounter));
  json_prop(F("reset_reason"), getResetReasonString());
  json_close();
  json_open(false, F("wifi"));

  # if defined(ESP8266)
  byte PHYmode = wifi_get_phy_mode();
  # endif // if defined(ESP8266)
  # if defined(ESP32)
  byte PHYmode = 3; // wifi_get_phy_mode();
  # endif // if defined(ESP32)

  switch (PHYmode)
  {
    case 1:
      json_prop(F("type"), F("802.11B"));
      break;
    case 2:
      json_prop(F("type"), F("802.11G"));
      break;
    case 3:
      json_prop(F("type"), F("802.11N"));
      break;
  }
  json_number(F("rssi"), String(WiFi.RSSI()));
  json_prop(F("dhcp"),          useStaticIP() ? getLabel(LabelType::IP_CONFIG_STATIC) : getLabel(LabelType::IP_CONFIG_DYNAMIC));
  json_prop(F("ip"),            getValue(LabelType::IP_ADDRESS));
  json_prop(F("subnet"),        getValue(LabelType::IP_SUBNET));
  json_prop(F("gw"),            getValue(LabelType::GATEWAY));
  json_prop(F("dns1"),          getValue(LabelType::DNS_1));
  json_prop(F("dns2"),          getValue(LabelType::DNS_2));
  json_prop(F("allowed_range"), describeAllowedIPrange());
  json_prop(F("sta_mac"),       getValue(LabelType::STA_MAC));
  json_prop(F("ap_mac"),        getValue(LabelType::AP_MAC));
  json_prop(F("ssid"),          getValue(LabelType::SSID));
  json_prop(F("bssid"),         getValue(LabelType::BSSID));
  json_number(F("channel"),     getValue(LabelType::CHANNEL));
  json_prop(F("connected"),     getValue(LabelType::CONNECTED));
  json_prop(F("ldr"),           getValue(LabelType::LAST_DISC_REASON_STR));
  json_number(F("reconnects"),  getValue(LabelType::NUMBER_RECONNECTS));
  json_close();

# ifdef HAS_ETHERNET
  json_open(false, F("ethernet"));
  json_prop(F("ethwifimode"),   getValue(LabelType::ETH_WIFI_MODE));
  json_prop(F("ethconnected"),  getValue(LabelType::ETH_CONNECTED));
  json_prop(F("ethduplex"),     getValue(LabelType::ETH_DUPLEX));
  json_prop(F("ethspeed"),      getValue(LabelType::ETH_SPEED));
  json_prop(F("ethstate"),      getValue(LabelType::ETH_STATE));
  json_prop(F("ethspeedstate"), getValue(LabelType::ETH_SPEED_STATE));
  json_close();
# endif // ifdef HAS_ETHERNET

  json_open(false, F("firmware"));
  json_prop(F("build"),          String(BUILD));
  json_prop(F("notes"),          F(BUILD_NOTES));
  json_prop(F("libraries"),      getSystemLibraryString());
  json_prop(F("git_version"),    F(BUILD_GIT));
  json_prop(F("plugins"),        getPluginDescriptionString());
  json_prop(F("md5"),            String(CRCValues.compileTimeMD5[0], HEX));
  json_number(F("md5_check"),    String(CRCValues.checkPassed()));
  json_prop(F("build_time"),     get_build_time());
  json_prop(F("filename"),       getValue(LabelType::BINARY_FILENAME));
  json_prop(F("build_platform"), getValue(LabelType::BUILD_PLATFORM));
  json_prop(F("git_head"),       getValue(LabelType::GIT_HEAD));
  json_close();

  json_open(false, F("esp"));
  json_prop(F("chip_id"), getValue(LabelType::ESP_CHIP_ID));
  json_number(F("cpu"),   getValue(LabelType::ESP_CHIP_FREQ));

  # ifdef ARDUINO_BOARD
  json_prop(F("board"), ARDUINO_BOARD);
  # endif // ifdef ARDUINO_BOARD
  json_close();
  json_open(false, F("storage"));

  # if defined(ESP8266)
  uint32_t flashChipId = ESP.getFlashChipId();

  // Set to HEX may be something like 0x1640E0.
  // Where manufacturer is 0xE0 and device is 0x4016.
  json_number(F("chip_id"), String(flashChipId));

  if (flashChipVendorPuya())
  {
    if (puyaSupport()) {
      json_prop(F("vendor"), F("puya, supported"));
    } else {
      json_prop(F("vendor"), F("puya, error"));
    }
  }
  uint32_t flashDevice = (flashChipId & 0xFF00) | ((flashChipId >> 16) & 0xFF);
  json_number(F("device"),    String(flashDevice));
  # endif // if defined(ESP8266)
  json_number(F("real_size"), String(getFlashRealSizeInBytes() / 1024));
  json_number(F("ide_size"),  String(ESP.getFlashChipSize() / 1024));

  // Please check what is supported for the ESP32
  # if defined(ESP8266)
  json_number(F("flash_speed"), String(ESP.getFlashChipSpeed() / 1000000));

  FlashMode_t ideMode = ESP.getFlashChipMode();

  switch (ideMode) {
    case FM_QIO:   json_prop(F("mode"), F("QIO"));  break;
    case FM_QOUT:  json_prop(F("mode"), F("QOUT")); break;
    case FM_DIO:   json_prop(F("mode"), F("DIO"));  break;
    case FM_DOUT:  json_prop(F("mode"), F("DOUT")); break;
    default:
      json_prop(F("mode"), getUnknownString()); break;
  }
  # endif // if defined(ESP8266)

  json_number(F("writes"),        String(RTC.flashDayCounter));
  json_number(F("flash_counter"), String(RTC.flashCounter));
  json_number(F("sketch_size"),   String(ESP.getSketchSize() / 1024));
  json_number(F("sketch_free"),   String(ESP.getFreeSketchSpace() / 1024));

  json_number(F("spiffs_size"),   String(SpiffsTotalBytes() / 1024));
  json_number(F("spiffs_free"),   String(SpiffsFreeSpace() / 1024));
  json_close();
  json_close();

  TXBuffer.endStream();
}

#endif // WEBSERVER_NEW_UI

#ifdef WEBSERVER_SYSINFO

void handle_sysinfo() {
  # ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("handle_sysinfo"));
  # endif // ifndef BUILD_NO_RAM_TRACKER

  if (!isLoggedIn()) { return; }
  navMenuIndex = MENU_INDEX_TOOLS;
  html_reset_copyTextCounter();
  TXBuffer.startStream();
  sendHeadandTail_stdtemplate();

  addHtml(printWebString);
  addHtml(F("<form>"));

  // the table header
  html_table_class_normal();


  # ifdef WEBSERVER_GITHUB_COPY

  // Not using addFormHeader() to get the copy button on the same header line as 2nd column
  html_TR();
  html_table_header(F("System Info"), 225);
  addHtml(F("<TH>")); // Needed to get the copy button on the same header line.
  addCopyButton(F("copyText"), F("\\n"), F("Copy info to clipboard"));

  TXBuffer += githublogo;
  serve_JS(JSfiles_e::GitHubClipboard);

  # else // ifdef WEBSERVER_GITHUB_COPY
  addFormHeader(F("System Info"));

  # endif // ifdef WEBSERVER_GITHUB_COPY

  handle_sysinfo_basicInfo();

  handle_sysinfo_memory();

  handle_sysinfo_Network();

# ifdef HAS_ETHERNET
  handle_sysinfo_Ethernet();
# endif // ifdef HAS_ETHERNET

  handle_sysinfo_WiFiSettings();

  handle_sysinfo_Firmware();

  handle_sysinfo_SystemStatus();

  handle_sysinfo_ESP_Board();

  handle_sysinfo_Storage();


  html_end_table();
  html_end_form();
  sendHeadandTail_stdtemplate(true);
  TXBuffer.endStream();
}

void handle_sysinfo_basicInfo() {
  addRowLabelValue(LabelType::UNIT_NR);

  if (node_time.systemTimePresent())
  {
    addRowLabelValue(LabelType::LOCAL_TIME);
  }

  addRowLabel(LabelType::UPTIME);
  {
    addHtml(getExtendedValue(LabelType::UPTIME));
  }

  addRowLabel(LabelType::LOAD_PCT);

  if (wdcounter > 0)
  {
    String html;
    html.reserve(32);
    html += getCPUload();
    html += F("% (LC=");
    html += getLoopCountPerSec();
    html += ')';
    addHtml(html);
  }
  addRowLabelValue(LabelType::CPU_ECO_MODE);


  addRowLabel(F("Boot"));
  {
    String html;
    html.reserve(64);

    html += getLastBootCauseString();
    html += " (";
    html += RTC.bootCounter;
    html += ')';
    addHtml(html);
  }
  addRowLabelValue(LabelType::RESET_REASON);
  addRowLabelValue(LabelType::LAST_TASK_BEFORE_REBOOT);
  addRowLabelValue(LabelType::SW_WD_COUNT);
}

void handle_sysinfo_memory() {
  addTableSeparator(F("Memory"), 2, 3);

# ifdef ESP32
  addRowLabelValue(LabelType::HEAP_SIZE);
  addRowLabelValue(LabelType::HEAP_MIN_FREE);
# endif // ifdef ESP32

  int freeMem = ESP.getFreeHeap();
  addRowLabel(LabelType::FREE_MEM);
  {
    String html;
    html.reserve(64);

    html += freeMem;
# ifndef BUILD_NO_RAM_TRACKER
    html += " (";
    html += lowestRAM;
    html += F(" - ");
    html += lowestRAMfunction;
    html += ')';
# endif // ifndef BUILD_NO_RAM_TRACKER
    addHtml(html);
  }
# if defined(CORE_POST_2_5_0) || defined(ESP32)
  addRowLabelValue(LabelType::HEAP_MAX_FREE_BLOCK);
# endif // if defined(CORE_POST_2_5_0) || defined(ESP32)
# if defined(CORE_POST_2_5_0)
  addRowLabelValue(LabelType::HEAP_FRAGMENTATION);
  addHtml("%");
# endif // ifdef CORE_POST_2_5_0


  addRowLabel(LabelType::FREE_STACK);
  {
    String html;
    html.reserve(64);
    html += getCurrentFreeStack();
# ifndef BUILD_NO_RAM_TRACKER
    html += " (";
    html += lowestFreeStack;
    html += F(" - ");
    html += lowestFreeStackfunction;
    html += ')';
# endif // ifndef BUILD_NO_RAM_TRACKER
    addHtml(html);
  }

# ifdef ESP32

  if (ESP.getPsramSize() > 0) {
    addRowLabelValue(LabelType::PSRAM_SIZE);
    addRowLabelValue(LabelType::PSRAM_FREE);
    addRowLabelValue(LabelType::PSRAM_MIN_FREE);
    addRowLabelValue(LabelType::PSRAM_MAX_FREE_BLOCK);
  }
# endif // ifdef ESP32
}

# ifdef HAS_ETHERNET
void handle_sysinfo_Ethernet() {
  if (active_network_medium == NetworkMedium_t::Ethernet) {
    addTableSeparator(F("Ethernet"), 2, 3);
    addRowLabelValue(LabelType::ETH_STATE);
    addRowLabelValue(LabelType::ETH_SPEED);
    addRowLabelValue(LabelType::ETH_DUPLEX);
    addRowLabelValue(LabelType::ETH_MAC);
    addRowLabelValue(LabelType::ETH_IP_ADDRESS_SUBNET);
    addRowLabelValue(LabelType::ETH_IP_GATEWAY);
    addRowLabelValue(LabelType::ETH_IP_DNS);
  }
}

# endif // ifdef HAS_ETHERNET

void handle_sysinfo_Network() {
  addTableSeparator(F("Network"), 2, 3, F("Wifi"));

  # ifdef HAS_ETHERNET
  addRowLabelValue(LabelType::ETH_WIFI_MODE);
  # endif // ifdef HAS_ETHERNET


  if (
    # ifdef HAS_ETHERNET
    active_network_medium == NetworkMedium_t::WIFI &&
    # endif // ifdef HAS_ETHERNET
    NetworkConnected())
  {
    addRowLabel(F("Wifi"));
    # if defined(ESP8266)
    byte PHYmode = wifi_get_phy_mode();
    # endif // if defined(ESP8266)
    # if defined(ESP32)
    byte PHYmode = 3; // wifi_get_phy_mode();
    # endif // if defined(ESP32)

    {
      String html;
      html.reserve(64);

      switch (PHYmode)
      {
        case 1:
          html += F("802.11B");
          break;
        case 2:
          html += F("802.11G");
          break;
        case 3:
          html += F("802.11N");
          break;
      }
      html += F(" (RSSI ");
      html += WiFi.RSSI();
      html += F(" dB)");
      addHtml(html);
    }
  }
  addRowLabelValue(LabelType::IP_CONFIG);
  addRowLabelValue(LabelType::IP_ADDRESS_SUBNET);
  addRowLabelValue(LabelType::GATEWAY);
  addRowLabelValue(LabelType::CLIENT_IP);
  addRowLabelValue(LabelType::DNS);
  addRowLabelValue(LabelType::ALLOWED_IP_RANGE);
  addRowLabelValue(LabelType::STA_MAC);
  addRowLabelValue(LabelType::AP_MAC);

  addRowLabel(LabelType::SSID);
  {
    String html;
    html.reserve(64);

    html += WiFi.SSID();
    html += " (";
    html += WiFi.BSSIDstr();
    html += ')';
    addHtml(html);
  }

  addRowLabelValue(LabelType::CHANNEL);
  addRowLabelValue(LabelType::CONNECTED);
  addRowLabel(LabelType::LAST_DISCONNECT_REASON);
  addHtml(getValue(LabelType::LAST_DISC_REASON_STR));
  addRowLabelValue(LabelType::NUMBER_RECONNECTS);
}

void handle_sysinfo_WiFiSettings() {
  addTableSeparator(F("WiFi Settings"), 2, 3);
  addRowLabelValue(LabelType::FORCE_WIFI_BG);
  addRowLabelValue(LabelType::RESTART_WIFI_LOST_CONN);
# ifdef ESP8266
  addRowLabelValue(LabelType::FORCE_WIFI_NOSLEEP);
# endif // ifdef ESP8266
# ifdef SUPPORT_ARP
  addRowLabelValue(LabelType::PERIODICAL_GRAT_ARP);
# endif // ifdef SUPPORT_ARP
  addRowLabelValue(LabelType::CONNECTION_FAIL_THRESH);
}

void handle_sysinfo_Firmware() {
  addTableSeparator(F("Firmware"), 2, 3);

  addRowLabelValue_copy(LabelType::BUILD_DESC);
  addHtml(" ");
  addHtml(F(BUILD_NOTES));

  addRowLabelValue_copy(LabelType::SYSTEM_LIBRARIES);
  addRowLabelValue_copy(LabelType::GIT_BUILD);
  addRowLabelValue_copy(LabelType::PLUGIN_COUNT);
  addHtml(" ");
  addHtml(getPluginDescriptionString());

  addRowLabel(F("Build Origin"));
  addHtml(get_build_origin());
  addRowLabelValue_copy(LabelType::BUILD_TIME);
  addRowLabelValue_copy(LabelType::BINARY_FILENAME);
  addRowLabelValue_copy(LabelType::BUILD_PLATFORM);
  addRowLabelValue_copy(LabelType::GIT_HEAD);
}

void handle_sysinfo_SystemStatus() {
  addTableSeparator(F("System Status"), 2, 3);

  // Actual Loglevel
  addRowLabelValue(LabelType::SYSLOG_LOG_LEVEL);
  addRowLabelValue(LabelType::SERIAL_LOG_LEVEL);
  addRowLabelValue(LabelType::WEB_LOG_LEVEL);
    # ifdef FEATURE_SD
  addRowLabelValue(LabelType::SD_LOG_LEVEL);
    # endif // ifdef FEATURE_SD
}

void handle_sysinfo_ESP_Board() {
  addTableSeparator(F("ESP Board"), 2, 3);


  addRowLabel(LabelType::ESP_CHIP_ID);
  {
    String html;
    html.reserve(32);
    html += getChipId();
    html += F(" (0x");
    String espChipId(getChipId(), HEX);
    espChipId.toUpperCase();
    html += espChipId;
    html += ')';
    addHtml(html);
  }

  addRowLabel(LabelType::ESP_CHIP_FREQ);
  addHtml(String(ESP.getCpuFreqMHz()));
  addHtml(F(" MHz"));

  addRowLabelValue(LabelType::ESP_CHIP_MODEL);

  # if defined(ESP32)
  addRowLabelValue(LabelType::ESP_CHIP_REVISION);
  # endif // if defined(ESP32)
  addRowLabelValue(LabelType::ESP_CHIP_CORES);

  # ifdef ARDUINO_BOARD
  addRowLabel(LabelType::ESP_BOARD_NAME);
  addHtml(ARDUINO_BOARD);
  # endif // ifdef ARDUINO_BOARD
}

void handle_sysinfo_Storage() {
  addTableSeparator(F("Storage"), 2, 3);

  uint32_t flashChipId = getFlashChipId();

  if (flashChipId != 0) {
    addRowLabel(LabelType::FLASH_CHIP_ID);


    // Set to HEX may be something like 0x1640E0.
    // Where manufacturer is 0xE0 and device is 0x4016.
    addHtml(F("Vendor: "));
    addHtml(formatToHex(flashChipId & 0xFF));

    if (flashChipVendorPuya())
    {
      addHtml(F(" (PUYA"));

      if (puyaSupport()) {
        addHtml(F(", supported"));
      } else {
        addHtml(F(HTML_SYMBOL_WARNING));
      }
      addHtml(")");
    }
    addHtml(F(" Device: "));
    uint32_t flashDevice = (flashChipId & 0xFF00) | ((flashChipId >> 16) & 0xFF);
    addHtml(formatToHex(flashDevice));
  }
  uint32_t realSize = getFlashRealSizeInBytes();
  uint32_t ideSize  = ESP.getFlashChipSize();

  addRowLabel(LabelType::FLASH_CHIP_REAL_SIZE);
  addHtml(String(realSize / 1024));
  addHtml(F(" kB"));

  addRowLabel(LabelType::FLASH_IDE_SIZE);
  addHtml(String(ideSize / 1024));
  addHtml(F(" kB"));

  // Please check what is supported for the ESP32
  # if defined(ESP8266)
  addRowLabel(LabelType::FLASH_IDE_SPEED);
  addHtml(String(ESP.getFlashChipSpeed() / 1000000));
  addHtml(F(" MHz"));

  FlashMode_t ideMode = ESP.getFlashChipMode();
  addRowLabel(LabelType::FLASH_IDE_MODE);
  {
    String html;

    switch (ideMode) {
      case FM_QIO:   html += F("QIO");  break;
      case FM_QOUT:  html += F("QOUT"); break;
      case FM_DIO:   html += F("DIO");  break;
      case FM_DOUT:  html += F("DOUT"); break;
      default:
        html += getUnknownString(); break;
    }
    addHtml(html);
  }
  # endif // if defined(ESP8266)

  addRowLabel(LabelType::FLASH_WRITE_COUNT);
  {
    String html;
    html.reserve(32);
    html += RTC.flashDayCounter;
    html += F(" daily / ");
    html += RTC.flashCounter;
    html += F(" boot");
    addHtml(html);
  }

  {
    // FIXME TD-er: Must also add this for ESP32.
    addRowLabel(LabelType::SKETCH_SIZE);
    {
      String html;
      html.reserve(32);
      html += ESP.getSketchSize() / 1024;
      html += F(" kB (");
      html += ESP.getFreeSketchSpace() / 1024;
      html += F(" kB free)");
      addHtml(html);
    }

    uint32_t maxSketchSize;
    bool     use2step;
    # if defined(ESP8266)
    bool otaEnabled =
    # endif // if defined(ESP8266)
    OTA_possible(maxSketchSize, use2step);
    addRowLabel(LabelType::MAX_OTA_SKETCH_SIZE);
    {
      String html;
      html.reserve(32);

      html += maxSketchSize / 1024;
      html += F(" kB (");
      html += maxSketchSize;
      html += F(" bytes)");
      addHtml(html);
    }

    # if defined(ESP8266)
    addRowLabel(LabelType::OTA_POSSIBLE);
    addHtml(boolToString(otaEnabled));

    addRowLabel(LabelType::OTA_2STEP);
    addHtml(boolToString(use2step));
    # endif // if defined(ESP8266)
  }

  addRowLabel(LabelType::FS_SIZE);
  {
    String html;
    html.reserve(32);

    html += SpiffsTotalBytes() / 1024;
    html += F(" kB (");
    html += SpiffsFreeSpace() / 1024;
    html += F(" kB free)");
    addHtml(html);
  }
  #ifndef LIMIT_BUILD_SIZE
  addRowLabel(F("Page size"));
  addHtml(String(SpiffsPagesize()));

  addRowLabel(F("Block size"));
  addHtml(String(SpiffsBlocksize()));

  addRowLabel(F("Number of blocks"));
  addHtml(String(SpiffsTotalBytes() / SpiffsBlocksize()));

  {
  # if defined(ESP8266)
    fs::FSInfo fs_info;
    ESPEASY_FS.info(fs_info);
    addRowLabel(F("Maximum open files"));
    addHtml(String(fs_info.maxOpenFiles));

    addRowLabel(F("Maximum path length"));
    addHtml(String(fs_info.maxPathLength));

  # endif // if defined(ESP8266)
  }
  #endif

# ifndef BUILD_MINIMAL_OTA

  if (showSettingsFileLayout) {
    addTableSeparator(F("Settings Files"), 2, 3);
    html_TR_TD();
    addHtml(F("Layout Settings File"));
    html_TD();
    getConfig_dat_file_layout();
    html_TR_TD();
    html_TD();
    addHtml(F("(offset / size per item / index)"));

    for (int st = 0; st < static_cast<int>(SettingsType::Enum::SettingsType_MAX); ++st) {
      SettingsType::Enum settingsType = static_cast<SettingsType::Enum>(st);
      html_TR_TD();
      addHtml(SettingsType::getSettingsTypeString(settingsType));
      html_BR();
      addHtml(SettingsType::getSettingsFileName(settingsType));
      html_TD();
      getStorageTableSVG(settingsType);
    }
  }
# endif // ifndef BUILD_MINIMAL_OTA

  # ifdef ESP32
  addTableSeparator(F("Partitions"), 2, 3,
                    F("https://dl.espressif.com/doc/esp-idf/latest/api-guides/partition-tables.html"));

  addRowLabel(F("Data Partition Table"));

  //   TXBuffer += getPartitionTableHeader(F(" - "), F("<BR>"));
  //   TXBuffer += getPartitionTable(ESP_PARTITION_TYPE_DATA, F(" - "), F("<BR>"));
  getPartitionTableSVG(ESP_PARTITION_TYPE_DATA, 0x5856e6);

  addRowLabel(F("App Partition Table"));

  //   TXBuffer += getPartitionTableHeader(F(" - "), F("<BR>"));
  //   TXBuffer += getPartitionTable(ESP_PARTITION_TYPE_APP , F(" - "), F("<BR>"));
  getPartitionTableSVG(ESP_PARTITION_TYPE_APP, 0xab56e6);
  # endif // ifdef ESP32
}

#endif    // ifdef WEBSERVER_SYSINFO

#include "../WebServer/SysVarPage.h"

#include "../WebServer/WebServer.h"
#include "../WebServer/AccessControl.h"
#include "../WebServer/Markup.h"
#include "../WebServer/Markup_Forms.h"
#include "../WebServer/HTML_wrappers.h"

#include "../Helpers/StringConverter.h"
#include "../Helpers/SystemVariables.h"

#ifdef WEBSERVER_SYSVARS


// ********************************************************************************
// Web Interface sysvars showing all system vars and their value.
// ********************************************************************************
void addSysVar_enum_html(SystemVariables::Enum enumval) {
  addSysVar_html(SystemVariables::toString(enumval));
}


void handle_sysvars() {
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("handle_sysvars"));
  #endif

  if (!isLoggedIn()) { return; }
  TXBuffer.startStream();
  sendHeadandTail_stdtemplate();

  html_BR();
  addHtml(F("<p>This page may load slow.<BR>Do not load too often, since it may affect performance of the node.</p>"));
  html_BR();

  // the table header
  html_table_class_normal();
  html_TR();
  html_table_header(F("System Variables"));
  html_table_header(F("Normal"));
  html_table_header(F("URL encoded"), F("ESPEasy_System_Variables"), 0);

  addTableSeparator(F("Constants"), 3, 3);
  addSysVar_enum_html(SystemVariables::CR);
  addSysVar_enum_html(SystemVariables::LF);
  addSysVar_enum_html(SystemVariables::SPACE);
  addSysVar_enum_html(SystemVariables::S_CR);
  addSysVar_enum_html(SystemVariables::S_LF);

  addTableSeparator(F("Network"), 3, 3);
  addSysVar_enum_html(SystemVariables::MAC);
#if defined(ESP8266)
  addSysVar_enum_html(SystemVariables::MAC_INT);
#endif // if defined(ESP8266)
  addSysVar_enum_html(SystemVariables::IP);
  addSysVar_enum_html(SystemVariables::IP4);
  addSysVar_enum_html(SystemVariables::SUBNET);
  addSysVar_enum_html(SystemVariables::GATEWAY);
  addSysVar_enum_html(SystemVariables::DNS);
  addSysVar_enum_html(SystemVariables::RSSI);
  addSysVar_enum_html(SystemVariables::SSID);
  addSysVar_enum_html(SystemVariables::BSSID);
  addSysVar_enum_html(SystemVariables::WI_CH);

#ifdef HAS_ETHERNET
  addTableSeparator(F("Ethernet"), 3, 3);
  addSysVar_enum_html(SystemVariables::ETHWIFIMODE);
  addSysVar_enum_html(SystemVariables::ETHCONNECTED);
  addSysVar_enum_html(SystemVariables::ETHDUPLEX);
  addSysVar_enum_html(SystemVariables::ETHSPEED);
  addSysVar_enum_html(SystemVariables::ETHSTATE);
  addSysVar_enum_html(SystemVariables::ETHSPEEDSTATE);
 #endif

  addTableSeparator(F("System"), 3, 3);
  addSysVar_enum_html(SystemVariables::UNIT_sysvar);
  addSysVar_enum_html(SystemVariables::SYSLOAD);
  addSysVar_enum_html(SystemVariables::SYSHEAP);
  addSysVar_enum_html(SystemVariables::SYSSTACK);
  addSysVar_enum_html(SystemVariables::SYSNAME);
#if FEATURE_ADC_VCC
  addSysVar_enum_html(SystemVariables::VCC);
#endif // if FEATURE_ADC_VCC

  addTableSeparator(F("System status"), 3, 3);

  addSysVar_enum_html(SystemVariables::ISWIFI);
  addSysVar_enum_html(SystemVariables::ISNTP);
  addSysVar_enum_html(SystemVariables::ISMQTT);
#ifdef USES_P037
  addSysVar_enum_html(SystemVariables::ISMQTTIMP);
#endif // USES_P037

  addTableSeparator(F("Time"), 3, 3);
  addSysVar_enum_html(SystemVariables::LCLTIME);
  addSysVar_enum_html(SystemVariables::LCLTIME_AM);
  addSysVar_enum_html(SystemVariables::SYSTM_HM);
  addSysVar_enum_html(SystemVariables::SYSTM_HM_AM);
  addSysVar_enum_html(SystemVariables::SYSTIME);
  addSysVar_enum_html(SystemVariables::SYSTIME_AM);
  addSysVar_enum_html(SystemVariables::SYSBUILD_DATE);
  addSysVar_enum_html(SystemVariables::SYSBUILD_TIME);
  addSysVar_enum_html(SystemVariables::SYSBUILD_FILENAME);
  addSysVar_enum_html(SystemVariables::SYSBUILD_DESCR);
  addSysVar_enum_html(SystemVariables::SYSBUILD_GIT);
  
  addTableSeparator(F("System"), 3, 3);
  addSysVar_html(F("%sysyear%  // %sysyear_0%"));
  addSysVar_html(F("%sysyears%"));
  addSysVar_html(F("%sysmonth% // %sysmonth_0%"));
  addSysVar_html(F("%sysday%   // %sysday_0%"));
  addSysVar_html(F("%syshour%  // %syshour_0%"));
  addSysVar_html(F("%sysmin%   // %sysmin_0%"));
  addSysVar_html(F("%syssec%   // %syssec_0%"));
  addSysVar_enum_html(SystemVariables::SYSSEC_D);
  addSysVar_enum_html(SystemVariables::SYSWEEKDAY);
  addSysVar_enum_html(SystemVariables::SYSWEEKDAY_S);
  addTableSeparator(F("System"), 3, 3);
  addSysVar_enum_html(SystemVariables::UPTIME);
  addSysVar_enum_html(SystemVariables::UNIXTIME);
  addSysVar_enum_html(SystemVariables::UNIXDAY);
  addSysVar_enum_html(SystemVariables::UNIXDAY_SEC);
  addSysVar_html(F("%sunset%"));
  addSysVar_html(F("%sunset-1h%"));
  addSysVar_html(F("%sunrise%"));
  addSysVar_html(F("%sunrise+10m%"));

  addTableSeparator(F("Custom Variables"), 3, 3);

  for (byte i = 0; i < CUSTOM_VARS_MAX; ++i) {
    addSysVar_html("%v" + toString(i + 1, 0) + '%');
  }
#ifndef BUILD_NO_SPECIAL_CHARACTERS_STRINGCONVERTER
  addTableSeparator(F("Special Characters"), 3, 2);
  addTableSeparator(F("Degree"),             3, 3);
  addSysVar_html(F("{D}"));
  addSysVar_html(F("&deg;"));

  addTableSeparator(F("Angle quotes"), 3, 3);
  addSysVar_html(F("{<<}"));
  addSysVar_html(F("&laquo;"));
  addFormSeparator(3);
  addSysVar_html(F("{>>}"));
  addSysVar_html(F("&raquo;"));
  addTableSeparator(F("Greek letter Mu"), 3, 3);
  addSysVar_html(F("{u}"));
  addSysVar_html(F("&micro;"));
  addTableSeparator(F("Currency"), 3, 3);
  addSysVar_html(F("{E}"));
  addSysVar_html(F("&euro;"));
  addFormSeparator(3);
  addSysVar_html(F("{Y}"));
  addSysVar_html(F("&yen;"));
  addFormSeparator(3);
  addSysVar_html(F("{P}"));
  addSysVar_html(F("&pound;"));
  addFormSeparator(3);
  addSysVar_html(F("{c}"));
  addSysVar_html(F("&cent;"));

  addTableSeparator(F("Math symbols"), 3, 3);
  addSysVar_html(F("{^1}"));
  addSysVar_html(F("&sup1;"));
  addFormSeparator(3);
  addSysVar_html(F("{^2}"));
  addSysVar_html(F("&sup2;"));
  addFormSeparator(3);
  addSysVar_html(F("{^3}"));
  addSysVar_html(F("&sup3;"));
  addFormSeparator(3);
  addSysVar_html(F("{1_4}"));
  addSysVar_html(F("&frac14;"));
  addFormSeparator(3);
  addSysVar_html(F("{1_2}"));
  addSysVar_html(F("&frac12;"));
  addFormSeparator(3);
  addSysVar_html(F("{3_4}"));
  addSysVar_html(F("&frac34;"));
  addFormSeparator(3);
  addSysVar_html(F("{+-}"));
  addSysVar_html(F("&plusmn;"));
  addFormSeparator(3);
  addSysVar_html(F("{x}"));
  addSysVar_html(F("&times;"));
  addFormSeparator(3);
  addSysVar_html(F("{..}"));
  addSysVar_html(F("&divide;"));
#endif
  addTableSeparator(F("Standard Conversions"), 3, 2);

  addSysVar_html(F("Wind Dir.:    %c_w_dir%(123.4)"));
  addSysVar_html(F("{D}C to {D}F: %c_c2f%(20.4)"));
  addSysVar_html(F("m/s to Bft:   %c_ms2Bft%(5.1)"));
  addSysVar_html(F("Dew point(T,H): %c_dew_th%(18.6,67)"));
  addFormSeparator(3);
  addSysVar_html(F("cm to imperial: %c_cm2imp%(190)"));
  addSysVar_html(F("mm to imperial: %c_mm2imp%(1900)"));
  addFormSeparator(3);
  addSysVar_html(F("Mins to days: %c_m2day%(1900)"));
  addSysVar_html(F("Mins to dh:   %c_m2dh%(1900)"));
  addSysVar_html(F("Mins to dhm:  %c_m2dhm%(1900)"));
  addSysVar_html(F("Secs to dhms: %c_s2dhms%(100000)"));
  addFormSeparator(3);
  addSysVar_html(F("To HEX: %c_2hex%(100000)"));
  addFormSeparator(3);
  addSysVar_html(F("Unit to IP: %c_u2ip%(%unit%, 2)"));

  html_end_table();
  html_end_form();
  sendHeadandTail_stdtemplate(true);
  TXBuffer.endStream();
}


void addSysVar_html(const String& input) {
  html_TR_TD();
  {
    String html;
    html.reserve(24 + input.length());
    html += F("<pre>"); // Make monospaced (<tt> tag?)
    html += F("<xmp>"); // Make sure HTML code is escaped. Tag depricated??
    html += input;
    html += F("</xmp>");
    html += F("</pre>");
    addHtml(html);
  }
  html_TD();
  String replacement(input);                // Make deepcopy for replacement
  parseSystemVariables(replacement, false); // Not URL encoded
  parseStandardConversions(replacement, false);
  addHtml(replacement);
  html_TD();
  replacement = input;
  parseSystemVariables(replacement, true); // URL encoded
  parseStandardConversions(replacement, true);
  addHtml(replacement);
  delay(0);
}

#endif // WEBSERVER_SYSVARS

#include "../WebServer/TimingStats.h"

#include "../WebServer/WebServer.h"
#include "../WebServer/HTML_wrappers.h"
#include "../WebServer/Markup.h"
#include "../WebServer/Markup_Forms.h"

#include "../DataTypes/ESPEasy_plugin_functions.h"

#include "../Globals/ESPEasy_time.h"
#include "../Globals/Protocol.h"
#include "../Globals/RamTracker.h"

#if defined(WEBSERVER_TIMINGSTATS) && defined(USES_TIMING_STATS)
#include "../Globals/Device.h"


#define TIMING_STATS_THRESHOLD 100000

void handle_timingstats() {
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("handle_timingstats"));
  #endif
  navMenuIndex = MENU_INDEX_TOOLS;
  TXBuffer.startStream();
  sendHeadandTail_stdtemplate(_HEAD);
  html_table_class_multirow();
  html_TR();
  html_table_header(F("Description"));
  html_table_header(F("Function"));
  html_table_header(F("#calls"));
  html_table_header(F("call/sec"));
  html_table_header(F("duty (%)"));
  html_table_header(F("min (ms)"));
  html_table_header(F("Avg (ms)"));
  html_table_header(F("max (ms)"));

  long timeSinceLastReset = stream_timing_statistics(true);
  html_end_table();

  html_table_class_normal();
  const float timespan = timeSinceLastReset / 1000.0f;
  addFormHeader(F("Statistics"));
  addRowLabel(F("Start Period"));
  struct tm startPeriod = node_time.addSeconds(node_time.tm, -1.0 * timespan, false);
  addHtml(ESPEasy_time::getDateTimeString(startPeriod, '-', ':', ' ', false));
  addRowLabelValue(LabelType::LOCAL_TIME);
  addRowLabel(F("Time span"));
  addHtml(String(timespan));
  addHtml(F(" sec"));
  addRowLabel(F("*"));
  addHtml(F("Duty cycle based on average < 1 msec is highly unreliable"));
  html_end_table();

  sendHeadandTail_stdtemplate(_TAIL);
  TXBuffer.endStream();
}

// ********************************************************************************
// HTML table formatted timing statistics
// ********************************************************************************
void format_using_threshhold(unsigned long value) {
  float value_msec = value / 1000.0f;

  if (value > TIMING_STATS_THRESHOLD) {
    html_B(String(value_msec, 3));
  } else {
    addHtml(String(value_msec, 3));
  }
}

void stream_html_timing_stats(const TimingStats& stats, long timeSinceLastReset) {
  unsigned long minVal, maxVal;
  const unsigned int  c = stats.getMinMax(minVal, maxVal);

  html_TD();
  addHtml(String(c));
  html_TD();
  const float call_per_sec = static_cast<float>(c) / static_cast<float>(timeSinceLastReset) * 1000.0f;
  const float avg = stats.getAvg();
  addHtml(String(call_per_sec, 2));
  html_TD();
  {
    const float duty = (call_per_sec * avg / 10000.0f);
    String duty_str = String(duty, 2);
    if (avg < 1000) {
      // Unreliable as average is below 1 msec
      duty_str += '*';
      html_I(duty_str);
    } else if (duty > 10.0f) {
      // Over 10% of the time
      html_B(duty_str);
    } else {
      addHtml(duty_str);
    }
  }

  html_TD();
  format_using_threshhold(minVal);
  html_TD();
  format_using_threshhold(avg);
  html_TD();
  format_using_threshhold(maxVal);
}

long stream_timing_statistics(bool clearStats) {
  long timeSinceLastReset = timePassedSince(timingstats_last_reset);

  for (auto& x: pluginStats) {
    if (!x.second.isEmpty()) {
      const deviceIndex_t deviceIndex = static_cast<deviceIndex_t>(x.first / 256);

      if (validDeviceIndex(deviceIndex)) {
        if (x.second.thresholdExceeded(TIMING_STATS_THRESHOLD)) {
          html_TR_TD_highlight();
        } else {
          html_TR_TD();
        }
        {
          String html;
          html.reserve(64);
          html += F("P_");
          html += Device[deviceIndex].Number;
          html += '_';
          html += getPluginNameFromDeviceIndex(deviceIndex);
          addHtml(html);
        }
        html_TD();
        addHtml(getPluginFunctionName(x.first % 256));
        stream_html_timing_stats(x.second, timeSinceLastReset);
      }

      if (clearStats) { x.second.reset(); }
    }
  }

  for (auto& x: controllerStats) {
    if (!x.second.isEmpty()) {
      const int ProtocolIndex = x.first / 256;

      if (x.second.thresholdExceeded(TIMING_STATS_THRESHOLD)) {
        html_TR_TD_highlight();
      } else {
        html_TR_TD();
      }
      {
        String html;
        html.reserve(64);

        html += F("C_");
        html += Protocol[ProtocolIndex].Number;
        html += '_';
        html += getCPluginNameFromProtocolIndex(ProtocolIndex);
        addHtml(html);
      }
      html_TD();
      addHtml(getCPluginCFunctionName(static_cast<CPlugin::Function>(x.first % 256)));
      stream_html_timing_stats(x.second, timeSinceLastReset);

      if (clearStats) { x.second.reset(); }
    }
  }

  for (auto& x: miscStats) {
    if (!x.second.isEmpty()) {
      if (x.second.thresholdExceeded(TIMING_STATS_THRESHOLD)) {
        html_TR_TD_highlight();
      } else {
        html_TR_TD();
      }
      addHtml(getMiscStatsName(x.first));
      html_TD();
      stream_html_timing_stats(x.second, timeSinceLastReset);

      if (clearStats) { x.second.reset(); }
    }
  }

  if (clearStats) {
    timingstats_last_reset = millis();
  }
  return timeSinceLastReset;
}

#endif // WEBSERVER_TIMINGSTATS

#include "../WebServer/ToolsPage.h"

#include "../WebServer/WebServer.h"
#include "../WebServer/HTML_wrappers.h"
#include "../WebServer/Markup.h"
#include "../WebServer/Markup_Buttons.h"
#include "../WebServer/Markup_Forms.h"

#include "../Helpers/OTA.h"

#include "../../ESPEasy-Globals.h"




#ifdef WEBSERVER_TOOLS

#include "../Commands/InternalCommands.h"
#include "../Helpers/WebServer_commandHelper.h"

// ********************************************************************************
// Web Interface Tools page
// ********************************************************************************
void handle_tools() {
  if (!isLoggedIn()) { return; }
  navMenuIndex = MENU_INDEX_TOOLS;
  TXBuffer.startStream();
  sendHeadandTail_stdtemplate(_HEAD);

  String webrequest = web_server.arg(F("cmd"));
  handle_command_from_web(EventValueSource::Enum::VALUE_SOURCE_WEB_FRONTEND, webrequest);
  printToWeb     = false;
  printToWebJSON = false;

  addHtml(F("<form>"));
  html_table_class_normal();

  addFormHeader(F("Tools"));

  addFormSubHeader(F("Command"));
  html_TR_TD();
  addHtml(F("<TR><TD colspan='2'>"));
  addHtml(F("<input "));
  addHtmlAttribute(F("style"), F("width: 98%"));
  addHtmlAttribute(F("type"), F("text"));
  addHtmlAttribute(F("name"), F("cmd"));
  addHtmlAttribute(F("value"), webrequest);
  addHtml('>');
  
  html_TR_TD();
  addSubmitButton();
  addHelpButton(F("ESPEasy_Command_Reference"));
  addRTDHelpButton(F("Reference/Command.html"));
  html_TR_TD();

  if (printWebString.length() > 0)
  {
    addHtml(F("<TR><TD colspan='2'>Command Output<BR><textarea readonly rows='10' wrap='on'>"));
    addHtml(printWebString);
    addHtml(F("</textarea>"));
    printWebString = "";
  }


  addFormSubHeader(F("System"));

  addWideButtonPlusDescription(F("/?cmd=reboot"), F("Reboot"),           F("Reboots ESP"));

  # ifdef WEBSERVER_LOG
  addWideButtonPlusDescription(F("log"),          F("Log"),              F("Open log output"));
  # endif // ifdef WEBSERVER_LOG

  #ifdef WEBSERVER_SYSINFO
  addWideButtonPlusDescription(F("sysinfo"),      F("Info"),             F("Open system info page"));
  #endif

  #ifdef WEBSERVER_ADVANCED
  addWideButtonPlusDescription(F("advanced"),     F("Advanced"),         F("Open advanced settings"));
  #endif

  addWideButtonPlusDescription(F("json"),         F("Show JSON"),        F("Open JSON output"));

  # ifdef WEBSERVER_TIMINGSTATS
  addWideButtonPlusDescription(F("timingstats"),  F("Timing stats"),     F("Open timing statistics of system"));
  # endif // WEBSERVER_TIMINGSTATS

  #ifdef WEBSERVER_PINSTATES
  addWideButtonPlusDescription(F("pinstates"),    F("Pin state buffer"), F("Show Pin state buffer"));
  #endif

  # ifdef WEBSERVER_SYSVARS
  addWideButtonPlusDescription(F("sysvars"),      F("System Variables"), F("Show all system variables and conversions"));
  # endif // ifdef WEBSERVER_SYSVARS

  addFormSubHeader(F("Wifi"));

  addWideButtonPlusDescription(F("/?cmd=wificonnect"),    F("Connect"),    F("Connects to known Wifi network"));
  addWideButtonPlusDescription(F("/?cmd=wifidisconnect"), F("Disconnect"), F("Disconnect from wifi network"));

  #ifdef WEBSERVER_WIFI_SCANNER
  addWideButtonPlusDescription(F("wifiscanner"),          F("Scan"),       F("Scan for wifi networks"));
  #endif // ifdef WEBSERVER_WIFI_SCANNER

  #ifdef WEBSERVER_I2C_SCANNER
  addFormSubHeader(F("Interfaces"));

  addWideButtonPlusDescription(F("i2cscanner"), F("I2C Scan"), F("Scan for I2C devices"));
  #endif // ifdef WEBSERVER_I2C_SCANNER

  addFormSubHeader(F("Settings"));

  addWideButtonPlusDescription(F("upload"), F("Load"), F("Loads a settings file"));
  addFormNote(F("(File MUST be renamed to \"config.dat\" before upload!)"));
  addWideButtonPlusDescription(F("download"), F("Save"), F("Saves a settings file"));

#ifdef WEBSERVER_NEW_UI
  # if defined(ESP8266)

  if ((SpiffsFreeSpace() / 1024) > 50) {
    html_TR_TD();
    addHtml(F(
              "<script>function downloadUI() { fetch('https://raw.githubusercontent.com/letscontrolit/espeasy_ui/master/build/index.htm.gz').then(r=>r.arrayBuffer()).then(r => {var f=new FormData();f.append('file', new File([new Blob([new Uint8Array(r)])], 'index.htm.gz'));f.append('edit', 1);fetch('/upload',{method:'POST',body:f}).then(() => {window.location.href='/';});}); }</script>"));
    addHtml(F("<a class=\"button link wide\" onclick=\"downloadUI()\">Download new UI</a>"));
    addHtml(F("</TD><TD>Download new UI(alpha)</TD></TR>"));
  }
  # endif // if defined(ESP8266)
#endif     // WEBSERVER_NEW_UI

#if defined(ESP8266) || defined(ESP32)
  {
    # ifndef NO_HTTP_UPDATER
    {
      uint32_t maxSketchSize;
      bool     use2step;
      bool     otaEnabled = OTA_possible(maxSketchSize, use2step);
      addFormSubHeader(F("Firmware"));
      html_TR_TD_height(30);
      addWideButton(F("update"), F("Update Firmware"), "", otaEnabled);
      addHelpButton(F("EasyOTA"));
      html_TD();
      addHtml(F("Load a new firmware"));

      if (otaEnabled) {
        if (use2step) {
          addHtml(F(" <b>WARNING</b> only use 2-step OTA update."));
        }
      } else {
        addHtml(F(" <b>WARNING</b> OTA not possible."));
      }
      addHtml(F(" Max sketch size: "));
      addHtml(String(maxSketchSize / 1024));
      addHtml(F(" kB ("));
      addHtml(String(maxSketchSize));
      addHtml(F(" bytes)"));
    }
    # endif // ifndef NO_HTTP_UPDATER
  }
#endif      // if defined(ESP8266)

  addFormSubHeader(F("Filesystem"));

  addWideButtonPlusDescription(F("filelist"),         F("File browser"),     F("Show files on internal flash file system"));
  addWideButtonPlusDescription(F("/factoryreset"),    F("Factory Reset"),    F("Select pre-defined configuration or full erase of settings"));
  #ifdef USE_SETTINGS_ARCHIVE
  addWideButtonPlusDescription(F("/settingsarchive"), F("Settings Archive"), F("Download settings from some archive"));
  #endif // ifdef USE_SETTINGS_ARCHIVE
#ifdef FEATURE_SD
  addWideButtonPlusDescription(F("SDfilelist"),       F("SD Card"),          F("Show files on SD-Card"));
#endif   // ifdef FEATURE_SD

  html_end_table();
  html_end_form();
  sendHeadandTail_stdtemplate(_TAIL);
  TXBuffer.endStream();
  printWebString = "";
  printToWeb     = false;
}

// ********************************************************************************
// Web Interface debug page
// ********************************************************************************
void addWideButtonPlusDescription(const String& url, const String& buttonText, const String& description)
{
  html_TR_TD_height(30);
  addWideButton(url, buttonText);
  html_TD();
  addHtml(description);
}

#endif // ifdef WEBSERVER_TOOLS

#include "../WebServer/UploadPage.h"

#include "../WebServer/WebServer.h"
#include "../WebServer/AccessControl.h"
#include "../WebServer/HTML_wrappers.h"

#include "../Globals/Cache.h"
#include "../Helpers/ESPEasy_Storage.h"

#include "../../ESPEasy-Globals.h"


#ifdef WEBSERVER_UPLOAD

// ********************************************************************************
// Web Interface upload page
// ********************************************************************************
uploadResult_e uploadResult = uploadResult_e::UploadStarted;

void handle_upload() {
  if (!isLoggedIn()) { return; }
  navMenuIndex = MENU_INDEX_TOOLS;
  TXBuffer.startStream();
  sendHeadandTail_stdtemplate();

  addHtml(F(
            "<form enctype='multipart/form-data' method='post'><p>Upload settings file:<br><input type='file' name='datafile' size='40'></p><div><input class='button link' type='submit' value='Upload'></div><input type='hidden' name='edit' value='1'></form>"));
  sendHeadandTail_stdtemplate(true);
  TXBuffer.endStream();
  printWebString = "";
  printToWeb     = false;
}

// ********************************************************************************
// Web Interface upload page
// ********************************************************************************
void handle_upload_post() {
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("handle_upload_post"));
  #endif

  if (!isLoggedIn()) { return; }

  navMenuIndex = MENU_INDEX_TOOLS;
  TXBuffer.startStream();
  sendHeadandTail_stdtemplate();

  switch (uploadResult) {
    case uploadResult_e::Success:
      addHtml(F("Upload OK!<BR>You may need to reboot to apply all settings..."));
      clearAllCaches();
      LoadSettings();
      break;
    case uploadResult_e::InvalidFile:
      addHtml(F("<font color=\"red\">Upload file invalid!</font>"));
      break;
    case uploadResult_e::NoFilename:
      addHtml(F("<font color=\"red\">No filename!</font>"));
      break;
    case uploadResult_e::UploadStarted:
      break;
  }

  addHtml(F("Upload finished"));
  sendHeadandTail_stdtemplate(true);
  TXBuffer.endStream();
  printWebString = "";
  printToWeb     = false;
}

#ifdef WEBSERVER_NEW_UI
void handle_upload_json() {
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("handle_upload_post"));
  #endif
  uint8_t result = static_cast<int>(uploadResult);

  if (!isLoggedIn()) { result = 255; }

  TXBuffer.startJsonStream();
  addHtml("{");
  stream_next_json_object_value(F("status"), String(result));
  addHtml("}");

  TXBuffer.endStream();
}

#endif // WEBSERVER_NEW_UI

// ********************************************************************************
// Web Interface upload handler
// ********************************************************************************
fs::File uploadFile;
void handleFileUpload() {
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("handleFileUpload"));
  #endif

  if (!isLoggedIn()) { return; }

  static boolean valid = false;

  HTTPUpload& upload = web_server.upload();

  if (upload.filename.c_str()[0] == 0)
  {
    uploadResult = uploadResult_e::NoFilename;
    return;
  }

  if (upload.status == UPLOAD_FILE_START)
  {
    if (loglevelActiveFor(LOG_LEVEL_INFO)) {
      String log = F("Upload: START, filename: ");
      log += upload.filename;
      addLog(LOG_LEVEL_INFO, log);
    }
    valid        = false;
    uploadResult = uploadResult_e::UploadStarted;
  }
  else if (upload.status == UPLOAD_FILE_WRITE)
  {
    // first data block, if this is the config file, check PID/Version
    if (upload.totalSize == 0)
    {
      if (strcasecmp(upload.filename.c_str(), FILE_CONFIG) == 0)
      {
        struct TempStruct {
          unsigned long PID;
          int           Version;
        } Temp;

        for (unsigned int x = 0; x < sizeof(struct TempStruct); x++)
        {
          byte b = upload.buf[x];
          memcpy((byte *)&Temp + x, &b, 1);
        }

        if ((Temp.Version == VERSION) && (Temp.PID == ESP_PROJECT_PID)) {
          valid = true;
        }
      }
      else
      {
        // other files are always valid...
        valid = true;
      }

      if (valid)
      {
        String filename;
#if defined(ESP32)
        filename += '/';
#endif // if defined(ESP32)
        filename += upload.filename;

        // once we're safe, remove file and create empty one...
        tryDeleteFile(filename);
        uploadFile = tryOpenFile(filename.c_str(), "w");

        // dont count manual uploads: flashCount();
      }
    }

    if (uploadFile) { uploadFile.write(upload.buf, upload.currentSize); }

    if (loglevelActiveFor(LOG_LEVEL_INFO)) {
      String log = F("Upload: WRITE, Bytes: ");
      log += upload.currentSize;
      addLog(LOG_LEVEL_INFO, log);
    }
  }
  else if (upload.status == UPLOAD_FILE_END)
  {
    if (uploadFile) { uploadFile.close(); }

    if (loglevelActiveFor(LOG_LEVEL_INFO)) {
      String log = F("Upload: END, Size: ");
      log += upload.totalSize;
      addLog(LOG_LEVEL_INFO, log);
    }
  }

  if (valid) {
    uploadResult = uploadResult_e::Success;
  }
  else {
    uploadResult = uploadResult_e::InvalidFile;
  }
}

#endif // ifdef WEBSERVER_UPLOAD

#include "../WebServer/WebServer.h"

#include "../WebServer/404.h"
#include "../WebServer/AccessControl.h"
#include "../WebServer/AdvancedConfigPage.h"
#include "../WebServer/CacheControllerPages.h"
#include "../WebServer/ConfigPage.h"
#include "../WebServer/ControlPage.h"
#include "../WebServer/ControllerPage.h"
#include "../WebServer/CustomPage.h"
#include "../WebServer/DevicesPage.h"
#include "../WebServer/DownloadPage.h"
#include "../WebServer/FactoryResetPage.h"
#include "../WebServer/Favicon.h"
#include "../WebServer/FileList.h"
#include "../WebServer/HTML_wrappers.h"
#include "../WebServer/HardwarePage.h"
#include "../WebServer/I2C_Scanner.h"
#include "../WebServer/JSON.h"
#include "../WebServer/LoadFromFS.h"
#include "../WebServer/Log.h"
#include "../WebServer/Login.h"
#include "../WebServer/Markup.h"
#include "../WebServer/Markup_Buttons.h"
#include "../WebServer/Markup_Forms.h"
#include "../WebServer/NotificationPage.h"
#include "../WebServer/PinStates.h"
#include "../WebServer/RootPage.h"
#include "../WebServer/Rules.h"
#include "../WebServer/SettingsArchive.h"
#include "../WebServer/SetupPage.h"
#include "../WebServer/SysInfoPage.h"
#include "../WebServer/SysVarPage.h"
#include "../WebServer/TimingStats.h"
#include "../WebServer/ToolsPage.h"
#include "../WebServer/UploadPage.h"
#include "../WebServer/WiFiScanner.h"


#include "../../ESPEasy-Globals.h"
#include "../../_Plugin_Helper.h"

#include "../DataStructs/TimingStats.h"

#include "../DataTypes/SettingsType.h"

#include "../ESPEasyCore/ESPEasyNetwork.h"
#include "../ESPEasyCore/ESPEasyWifi.h"

#include "../Globals/CPlugins.h"
#include "../Globals/Device.h"
#include "../Globals/ExtraTaskSettings.h"
#include "../Globals/NetworkState.h"
#include "../Globals/Protocol.h"
#include "../Globals/SecuritySettings.h"

#include "../Helpers/ESPEasy_Storage.h"
#include "../Helpers/Hardware.h"
#include "../Helpers/MDNS_Helper.h"
#include "../Helpers/Networking.h"
#include "../Helpers/OTA.h"
#include "../Helpers/StringConverter.h"

#include "../Static/WebStaticData.h"


void sendHeadandTail(const String& tmplName, boolean Tail, boolean rebooting) {
  // This function is called twice per serving a web page.
  // So it must keep track of the timer longer than the scope of this function.
  // Therefore use a local static variable.
  #ifdef USES_TIMING_STATS
  static unsigned statisticsTimerStart = 0;

  if (!Tail) {
    statisticsTimerStart = micros();
  }
  #endif // ifdef USES_TIMING_STATS

  String pageTemplate = "";
  String fileName     = tmplName;

  fileName += F(".htm");
  fs::File f = tryOpenFile(fileName, "r");

  if (f) {
    pageTemplate.reserve(f.size());

    while (f.available()) { pageTemplate += (char)f.read(); }
    f.close();
  } else {
    // TODO TD-er: Should send data directly to TXBuffer instead of using large strings.
    getWebPageTemplateDefault(tmplName, pageTemplate);
  }
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("sendWebPage"));
  #endif // ifndef BUILD_NO_RAM_TRACKER

  // web activity timer
  lastWeb = millis();

  if (Tail) {
    addHtml(pageTemplate.substring(
              11 +                                      // Size of "{{content}}"
              pageTemplate.indexOf(F("{{content}}")))); // advance beyond content key
  } else {
    int indexStart = 0;
    int indexEnd   = 0;
    int readPos    = 0; // Position of data sent to TXBuffer
    String varName;     // , varValue;
    String meta;

    if (rebooting) {
      meta = F("<meta http-equiv='refresh' content='10 url=/'>");
    }

    while ((indexStart = pageTemplate.indexOf(F("{{"), indexStart)) >= 0) {
      addHtml(pageTemplate.substring(readPos, indexStart));
      readPos = indexStart;

      if ((indexEnd = pageTemplate.indexOf(F("}}"), indexStart)) > 0) {
        varName    = pageTemplate.substring(indexStart + 2, indexEnd);
        indexStart = indexEnd + 2;
        readPos    = indexEnd + 2;
        varName.toLowerCase();

        if (varName == F("content")) { // is var == page content?
          break;                       // send first part of result only
        } else if (varName == F("error")) {
          getErrorNotifications();
        }
        else if (varName == F("meta")) {
          addHtml(meta);
        }
        else {
          getWebPageTemplateVar(varName);
        }
      } else { // no closing "}}"
        // eat "{{"
        readPos    += 2;
        indexStart += 2;
      }
    }
  }

  if (shouldReboot) {
    // we only add this here as a seperate chunk to prevent using too much memory at once
    serve_JS(JSfiles_e::Reboot);
  }
  STOP_TIMER(HANDLE_SERVING_WEBPAGE);
}

void sendHeadandTail_stdtemplate(boolean Tail, boolean rebooting) {
  sendHeadandTail(F("TmplStd"), Tail, rebooting);

  if (!Tail) {
    if (!clientIPinSubnet() && WifiIsAP(WiFi.getMode()) && (WiFi.softAPgetStationNum() > 0)) {
      addHtmlError(F("Warning: Connected via AP"));
    }

    #ifndef BUILD_NO_DEBUG
    if (loglevelActiveFor(LOG_LEVEL_INFO)) {
      const int nrArgs = web_server.args();

      if (nrArgs > 0) {
        String log = F(" Webserver args:");

        for (int i = 0; i < nrArgs; ++i) {
          log += ' ';
          log += i;
          log += F(": '");
          log += web_server.argName(i);
          log += F("' length: ");
          log += web_server.arg(i).length();
        }
        addLog(LOG_LEVEL_INFO, log);
      }
    }
    #endif
  }
}

size_t streamFile_htmlEscape(const String& fileName)
{
  fs::File f    = tryOpenFile(fileName, "r");
  size_t   size = 0;

  if (f)
  {
    String escaped;

    while (f.available())
    {
      char c = (char)f.read();

      if (htmlEscapeChar(c, escaped)) {
        addHtml(escaped);
      } else {
        addHtml(String(c));
      }
      ++size;
    }
    f.close();
  }
  return size;
}

// ********************************************************************************
// Web Interface init
// ********************************************************************************
// #include "core_version.h"


void WebServerInit()
{
  if (webserver_init) { return; }
  webserver_init = true;

  // Prepare webserver pages
  #ifdef WEBSERVER_ROOT
  web_server.on("/",               handle_root);
  #endif // ifdef WEBSERVER_ROOT
  #ifdef WEBSERVER_ADVANCED
  web_server.on(F("/advanced"),    handle_advanced);
  #endif // ifdef WEBSERVER_ADVANCED
  #ifdef WEBSERVER_CONFIG
  web_server.on(F("/config"),      handle_config);
  #endif // ifdef WEBSERVER_CONFIG
  #ifdef WEBSERVER_CONTROL
  web_server.on(F("/control"),     handle_control);
  #endif // ifdef WEBSERVER_CONTROL
  #ifdef WEBSERVER_CONTROLLERS
  web_server.on(F("/controllers"), handle_controllers);
  #endif // ifdef WEBSERVER_CONTROLLERS
  #ifdef WEBSERVER_DEVICES
  web_server.on(F("/devices"),     handle_devices);
  #endif // ifdef WEBSERVER_DEVICES
  #ifdef WEBSERVER_DOWNLOAD
  web_server.on(F("/download"),    handle_download);
  #endif // ifdef WEBSERVER_DOWNLOAD

#ifdef USES_C016

  // web_server.on(F("/dumpcache"),     handle_dumpcache);  // C016 specific entrie
  web_server.on(F("/cache_json"), handle_cache_json); // C016 specific entrie
  web_server.on(F("/cache_csv"),  handle_cache_csv);  // C016 specific entrie
#endif // USES_C016

  #ifdef WEBSERVER_FACTORY_RESET
  web_server.on(F("/factoryreset"),    handle_factoryreset);
  #endif // ifdef WEBSERVER_FACTORY_RESET
  #ifdef USE_SETTINGS_ARCHIVE
  web_server.on(F("/settingsarchive"), handle_settingsarchive);
  #endif // ifdef USE_SETTINGS_ARCHIVE
  web_server.on(F("/favicon.ico"),     handle_favicon);
  #ifdef WEBSERVER_FILELIST
  web_server.on(F("/filelist"),        handle_filelist);
  #endif // ifdef WEBSERVER_FILELIST
  #ifdef WEBSERVER_HARDWARE
  web_server.on(F("/hardware"),        handle_hardware);
  #endif // ifdef WEBSERVER_HARDWARE
  #ifdef WEBSERVER_I2C_SCANNER
  web_server.on(F("/i2cscanner"),      handle_i2cscanner);
  #endif // ifdef WEBSERVER_I2C_SCANNER
  web_server.on(F("/json"),            handle_json); // Also part of WEBSERVER_NEW_UI
  web_server.on(F("/csv"),             handle_csvval);
  web_server.on(F("/log"),             handle_log);
  web_server.on(F("/login"),           handle_login);
  web_server.on(F("/logjson"),         handle_log_JSON); // Also part of WEBSERVER_NEW_UI
#ifdef USES_NOTIFIER
  web_server.on(F("/notifications"),   handle_notifications);
#endif // ifdef USES_NOTIFIER
  #ifdef WEBSERVER_PINSTATES
  web_server.on(F("/pinstates"),       handle_pinstates);
  #endif // ifdef WEBSERVER_PINSTATES
  #ifdef WEBSERVER_RULES
  web_server.on(F("/rules"),           handle_rules_new);
  web_server.on(F("/rules/"),          Goto_Rules_Root);
  #ifdef WEBSERVER_NEW_RULES
  web_server.on(F("/rules/add"),       []()
  {
    handle_rules_edit(web_server.uri(), true);
  });
  web_server.on(F("/rules/backup"), handle_rules_backup);
  web_server.on(F("/rules/delete"), handle_rules_delete);
  #endif // WEBSERVER_NEW_RULES
  #endif // WEBSERVER_RULES
#ifdef FEATURE_SD
  web_server.on(F("/SDfilelist"),   handle_SDfilelist);
#endif   // ifdef FEATURE_SD
#ifdef WEBSERVER_SETUP
  web_server.on(F("/setup"),        handle_setup);
#endif // ifdef WEBSERVER_SETUP
#ifdef WEBSERVER_SYSINFO
  web_server.on(F("/sysinfo"),      handle_sysinfo);
#endif // ifdef WEBSERVER_SYSINFO
#ifdef WEBSERVER_SYSVARS
  web_server.on(F("/sysvars"),      handle_sysvars);
#endif // WEBSERVER_SYSVARS
#ifdef WEBSERVER_TIMINGSTATS
  web_server.on(F("/timingstats"),  handle_timingstats);
#endif // WEBSERVER_TIMINGSTATS
#ifdef WEBSERVER_TOOLS
  web_server.on(F("/tools"),        handle_tools);
#endif // ifdef WEBSERVER_TOOLS
#ifdef WEBSERVER_UPLOAD
  web_server.on(F("/upload"),       HTTP_GET,  handle_upload);
  web_server.on(F("/upload"),       HTTP_POST, handle_upload_post, handleFileUpload);
#endif // ifdef WEBSERVER_UPLOAD
#ifdef WEBSERVER_WIFI_SCANNER
  web_server.on(F("/wifiscanner"),  handle_wifiscanner);
#endif // ifdef WEBSERVER_WIFI_SCANNER

#ifdef WEBSERVER_NEW_UI
  web_server.on(F("/buildinfo"),         handle_buildinfo); // Also part of WEBSERVER_NEW_UI
  web_server.on(F("/factoryreset_json"), handle_factoryreset_json);
  web_server.on(F("/filelist_json"),     handle_filelist_json);
  web_server.on(F("/i2cscanner_json"),   handle_i2cscanner_json);
  web_server.on(F("/node_list_json"),    handle_nodes_list_json);
  web_server.on(F("/pinstates_json"),    handle_pinstates_json);
  web_server.on(F("/sysinfo_json"),      handle_sysinfo_json);
  web_server.on(F("/timingstats_json"),  handle_timingstats_json);
  web_server.on(F("/upload_json"),       HTTP_POST, handle_upload_json, handleFileUpload);
  web_server.on(F("/wifiscanner_json"),  handle_wifiscanner_json);
#endif // WEBSERVER_NEW_UI

  web_server.onNotFound(handleNotFound);

  #if defined(ESP8266) || defined(ESP32)
  {
    # ifndef NO_HTTP_UPDATER
    uint32_t maxSketchSize;
    bool     use2step;

    if (OTA_possible(maxSketchSize, use2step)) {
      httpUpdater.setup(&web_server);
    }
    # endif // ifndef NO_HTTP_UPDATER
  }
  #endif    // if defined(ESP8266)

  #if defined(ESP8266)

  # ifdef USES_SSDP

  if (Settings.UseSSDP)
  {
    web_server.on(F("/ssdp.xml"), HTTP_GET, []() {
      WiFiClient client(web_server.client());
      client.setTimeout(CONTROLLER_CLIENTTIMEOUT_DFLT);
      SSDP_schema(client);
    });
    SSDP_begin();
  }
  # endif // USES_SSDP
  #endif  // if defined(ESP8266)
}

void setWebserverRunning(bool state) {
  if (webserverRunning == state) {
    return;
  }

  if (state) {
    WebServerInit();
    web_server.begin(Settings.WebserverPort);
    addLog(LOG_LEVEL_INFO, F("Webserver: start"));
  } else {
    web_server.stop();
    addLog(LOG_LEVEL_INFO, F("Webserver: stop"));
  }
  webserverRunning = state;
  set_mDNS(); // Uses webserverRunning state.
}

void getWebPageTemplateDefault(const String& tmplName, String& tmpl)
{
  tmpl.reserve(576);
  const bool addJS   = true;
  const bool addMeta = true;

  if (tmplName == F("TmplAP"))
  {
    getWebPageTemplateDefaultHead(tmpl, !addMeta, !addJS);
    tmpl += F("<body>"
              "<header class='apheader'>"
              "<h1>Welcome to ESP Easy Mega AP</h1>"
              "</header>");
    getWebPageTemplateDefaultContentSection(tmpl);
    getWebPageTemplateDefaultFooter(tmpl);
  }
  else if (tmplName == F("TmplMsg"))
  {
    getWebPageTemplateDefaultHead(tmpl, !addMeta, !addJS);
    tmpl += F("<body>");
    getWebPageTemplateDefaultHeader(tmpl, F("{{name}}"), false);
    getWebPageTemplateDefaultContentSection(tmpl);
    getWebPageTemplateDefaultFooter(tmpl);
  }
  else if (tmplName == F("TmplDsh"))
  {
    getWebPageTemplateDefaultHead(tmpl, !addMeta, addJS);
    tmpl += F(
      "<body>"
      "{{content}}"
      "</body></html>"
      );
  }
  else // all other template names e.g. TmplStd
  {
    getWebPageTemplateDefaultHead(tmpl, addMeta, addJS);
    tmpl += F("<body class='bodymenu'>"
              "<span class='message' id='rbtmsg'></span>");
    getWebPageTemplateDefaultHeader(tmpl, F("{{name}} {{logo}}"), true);
    getWebPageTemplateDefaultContentSection(tmpl);
    getWebPageTemplateDefaultFooter(tmpl);
  }
}

void getWebPageTemplateDefaultHead(String& tmpl, bool addMeta, bool addJS) {
  tmpl += F("<!DOCTYPE html><html lang='en'>"
            "<head>"
            "<meta charset='utf-8'/>"
            "<meta name='viewport' content='width=device-width, initial-scale=1.0'>"
            "<title>{{name}}</title>");

  if (addMeta) { tmpl += F("{{meta}}"); }

  if (addJS) { tmpl += F("{{js}}"); }

  tmpl += F("{{css}}"
            "</head>");
}

void getWebPageTemplateDefaultHeader(String& tmpl, const String& title, bool addMenu) {
  tmpl += F("<header class='headermenu'>"
            "<h1>ESP Easy Mega: ");
  tmpl += title;
  tmpl += F("</h1><BR>");

  if (addMenu) { tmpl += F("{{menu}}"); }
  tmpl += F("</header>");
}

void getWebPageTemplateDefaultContentSection(String& tmpl) {
  tmpl += F("<section>"
            "<span class='message error'>"
            "{{error}}"
            "</span>"
            "{{content}}"
            "</section>"
            );
}

void getWebPageTemplateDefaultFooter(String& tmpl) {
  tmpl += F("<footer>"
            "<br>"
            "<h6>Powered by <a href='http://www.letscontrolit.com' style='font-size: 15px; text-decoration: none'>Let's Control It</a> community</h6>"
            "</footer>"
            "</body></html>"
            );
}

void getErrorNotifications() {
  // Check number of MQTT controllers active.
  int nrMQTTenabled = 0;

  for (controllerIndex_t x = 0; x < CONTROLLER_MAX; x++) {
    if (Settings.Protocol[x] != 0) {
      protocolIndex_t ProtocolIndex = getProtocolIndex_from_ControllerIndex(x);

      if (validProtocolIndex(ProtocolIndex) && Settings.ControllerEnabled[x] && Protocol[ProtocolIndex].usesMQTT) {
        ++nrMQTTenabled;
      }
    }
  }

  if (nrMQTTenabled > 1) {
    // Add warning, only one MQTT protocol should be used.
    addHtmlError(F("Only one MQTT controller should be active."));
  }

  // Check checksum of stored settings.
}

byte navMenuIndex = MENU_INDEX_MAIN;

// See https://github.com/letscontrolit/ESPEasy/issues/1650
String getGpMenuIcon(byte index) {
  switch (index) {
    case MENU_INDEX_MAIN: return F("&#8962;");
    case MENU_INDEX_CONFIG: return F("&#9881;");
    case MENU_INDEX_CONTROLLERS: return F("&#128172;");
    case MENU_INDEX_HARDWARE: return F("&#128204;");
    case MENU_INDEX_DEVICES: return F("&#128268;");
    case MENU_INDEX_RULES: return F("&#10740;");
    case MENU_INDEX_NOTIFICATIONS: return F("&#9993;");
    case MENU_INDEX_TOOLS: return F("&#128295;");
  }
  return "";
}

String getGpMenuLabel(byte index) {
  switch (index) {
    case MENU_INDEX_MAIN: return F("Main");
    case MENU_INDEX_CONFIG: return F("Config");
    case MENU_INDEX_CONTROLLERS: return F("Controllers");
    case MENU_INDEX_HARDWARE: return F("Hardware");
    case MENU_INDEX_DEVICES: return F("Devices");
    case MENU_INDEX_RULES: return F("Rules");
    case MENU_INDEX_NOTIFICATIONS: return F("Notifications");
    case MENU_INDEX_TOOLS: return F("Tools");
  }
  return "";
}

String getGpMenuURL(byte index) {
  switch (index) {
    case MENU_INDEX_MAIN: return F("/");
    case MENU_INDEX_CONFIG: return F("/config");
    case MENU_INDEX_CONTROLLERS: return F("/controllers");
    case MENU_INDEX_HARDWARE: return F("/hardware");
    case MENU_INDEX_DEVICES: return F("/devices");
    case MENU_INDEX_RULES: return F("/rules");
    case MENU_INDEX_NOTIFICATIONS: return F("/notifications");
    case MENU_INDEX_TOOLS: return F("/tools");
  }
  return "";
}

void getWebPageTemplateVar(const String& varName)
{
  // serialPrint(varName); serialPrint(" : free: "); serialPrint(ESP.getFreeHeap());   serialPrint("var len before:  "); serialPrint
  // (varValue.length()) ;serialPrint("after:  ");
  // varValue = "";

  if (varName == F("name"))
  {
    addHtml(Settings.Name);
  }

  else if (varName == F("unit"))
  {
    addHtml(String(Settings.Unit));
  }

  else if (varName == F("menu"))
  {
    addHtml(F("<div class='menubar'>"));

    for (byte i = 0; i < 8; i++)
    {
      if ((i == MENU_INDEX_RULES) && !Settings.UseRules) { // hide rules menu item
        continue;
      }
#ifndef USES_NOTIFIER

      if (i == MENU_INDEX_NOTIFICATIONS) { // hide notifications menu item
        continue;
      }
#endif // ifndef USES_NOTIFIER

      addHtml(F("<a "));

      addHtmlAttribute(F("class"), (i == navMenuIndex) ? F("menu active") : F("menu"));
      addHtmlAttribute(F("href"), getGpMenuURL(i));
      addHtml('>');
      addHtml(getGpMenuIcon(i));
      addHtml(F("<span class='showmenulabel'>"));
      addHtml(getGpMenuLabel(i));
      addHtml(F("</span></a>"));
    }

    addHtml(F("</div>"));
  }

  else if (varName == F("logo"))
  {
    if (fileExists(F("esp.png")))
    {
      addHtml(F("<img src=\"esp.png\" width=48 height=48 align=right>"));
    }
  }

  else if (varName == F("css"))
  {
    serve_favicon();
    serve_CSS();
  }


  else if (varName == F("js"))
  {
    html_add_autosubmit_form();

    // FIXME TD-er: Can only call this after tag has been set for the file.
    //serve_JS(JSfiles_e::Toasting);
    html_add_script(false);
    TXBuffer += jsToastMessageBegin;

    // we can push custom messages here in future releases...
    addHtml(F("Submitted"));
    TXBuffer += jsToastMessageEnd;

    html_add_script_end();
  }

  else if (varName == F("error"))
  {
    // print last error - not implemented yet
  }

  else if (varName == F("debug"))
  {
    // print debug messages - not implemented yet
  }

  else
  {
    #ifndef BUILD_NO_DEBUG
    if (loglevelActiveFor(LOG_LEVEL_ERROR)) {
      String log = F("Templ: Unknown Var : ");
      log += varName;
      addLog(LOG_LEVEL_ERROR, log);
    }
    #endif

    // no return string - eat var name
  }
}

void writeDefaultCSS(void)
{
  return; // TODO

#ifndef BUILD_NO_DEBUG

  if (!fileExists(F("esp.css")))
  {
    fs::File f = tryOpenFile(F("esp.css"), "w");

    if (f)
    {
      String defaultCSS;
      defaultCSS = PGMT(DATA_ESPEASY_DEFAULT_MIN_CSS);

      if (loglevelActiveFor(LOG_LEVEL_INFO)) {
        String log = F("CSS  : Writing default CSS file to FS (");
        log += defaultCSS.length();
        log += F(" bytes)");
        addLog(LOG_LEVEL_INFO, log);
      }
      f.write((const unsigned char *)defaultCSS.c_str(), defaultCSS.length()); // note: content must be in RAM - a write of F("XXX") does
                                                                               // not work
      f.close();
    }
  }
#endif // ifndef BUILD_NO_DEBUG
}

// ********************************************************************************
// Functions to stream JSON directly to TXBuffer
// FIXME TD-er: replace stream_xxx_json_object* into this code.
// N.B. handling of numerical values differs (string vs. no string)
// ********************************************************************************

int8_t level     = 0;
int8_t lastLevel = -1;

void json_quote_name(const String& val) {
  String html;

  html.reserve(4 + val.length());

  if (lastLevel == level) {
    html += ",";
  }

  if (val.length() > 0) {
    html += '\"';
    html += val;
    html += '\"';
    html += ':';
  }
  addHtml(html);
}

void json_quote_val(const String& val) {
  String html;

  html.reserve(4 + val.length());
  html += '\"';
  html += val;
  html += '\"';
  addHtml(html);
}

void json_open() {
  json_open(false, String());
}

void json_open(bool arr) {
  json_open(arr, String());
}

void json_open(bool arr, const String& name) {
  json_quote_name(name);
  addHtml(arr ? "[" : "{");
  lastLevel = level;
  level++;
}

void json_init() {
  level     = 0;
  lastLevel = -1;
}

void json_close() {
  json_close(false);
}

void json_close(bool arr) {
  addHtml(arr ? "]" : "}");
  level--;
  lastLevel = level;
}

void json_number(const String& name, const String& value) {
  json_quote_name(name);
  json_quote_val(value);
  lastLevel = level;
}

void json_prop(const String& name, const String& value) {
  json_quote_name(name);
  json_quote_val(value);
  lastLevel = level;
}

void json_prop(LabelType::Enum label) {
  json_prop(getInternalLabel(label, '-'), getValue(label));
}

// ********************************************************************************
// Add a task select dropdown list
// This allows to select a task index based on the existing tasks.
// ********************************************************************************
void addTaskSelect(const String& name,  taskIndex_t choice)
{
  String deviceName;

  addHtml(F("<select "));
  addHtmlAttribute(F("id"),       F("selectwidth"));
  addHtmlAttribute(F("name"),     name);
  addHtmlAttribute(F("onchange"), F("return dept_onchange(frmselect)"));
  addHtml('>');

  for (taskIndex_t x = 0; x < TASKS_MAX; x++)
  {
    const deviceIndex_t DeviceIndex = getDeviceIndex_from_TaskIndex(x);
    deviceName = getPluginNameFromDeviceIndex(DeviceIndex);
    LoadTaskSettings(x);

    {
      String html;
      html.reserve(32);

      html += F("<option value='");
      html += x;
      html += '\'';

      if (choice == x) {
        html += F(" selected");
      }
      addHtml(html);
    }

    if (!validPluginID_fullcheck(Settings.TaskDeviceNumber[x])) {
      addDisabled();
    }
    {
      String html;
      html.reserve(96);

      html += '>';
      html += x + 1;
      html += F(" - ");
      html += deviceName;
      html += F(" - ");
      html += ExtraTaskSettings.TaskDeviceName;
      html += F("</option>");
      addHtml(html);
    }
  }
}

// ********************************************************************************
// Add a Value select dropdown list, based on TaskIndex
// This allows to select a task value, based on the existing tasks.
// ********************************************************************************
void addTaskValueSelect(const String& name, int choice, taskIndex_t TaskIndex)
{
  if (!validTaskIndex(TaskIndex)) { return; }
  const deviceIndex_t DeviceIndex = getDeviceIndex_from_TaskIndex(TaskIndex);

  if (!validDeviceIndex(DeviceIndex)) { return; }

  addHtml(F("<select "));
  addHtmlAttribute(F("id"),   F("selectwidth"));
  addHtmlAttribute(F("name"), name);
  addHtml('>');

  LoadTaskSettings(TaskIndex);
  const byte valueCount = getValueCountForTask(TaskIndex);

  for (byte x = 0; x < valueCount; x++)
  {
    String html;
    html.reserve(96);
    html += F("<option value='");
    html += x;
    html += '\'';

    if (choice == x) {
      html += F(" selected");
    }
    html += '>';
    html += ExtraTaskSettings.TaskDeviceValueNames[x];
    html += F("</option>");
    addHtml(html);
  }
}

// ********************************************************************************
// Login state check
// ********************************************************************************
boolean isLoggedIn()
{
  String www_username = F(DEFAULT_ADMIN_USERNAME);

  if (!clientIPallowed()) { return false; }

  if (SecuritySettings.Password[0] == 0) { return true; }

  if (!web_server.authenticate(www_username.c_str(), SecuritySettings.Password))

  // Basic Auth Method with Custom realm and Failure Response
  // return server.requestAuthentication(BASIC_AUTH, www_realm, authFailResponse);
  // Digest Auth Method with realm="Login Required" and empty Failure Response
  // return server.requestAuthentication(DIGEST_AUTH);
  // Digest Auth Method with Custom realm and empty Failure Response
  // return server.requestAuthentication(DIGEST_AUTH, www_realm);
  // Digest Auth Method with Custom realm and Failure Response
  {
#ifdef CORE_PRE_2_5_0

    // See https://github.com/esp8266/Arduino/issues/4717
    HTTPAuthMethod mode = BASIC_AUTH;
#else // ifdef CORE_PRE_2_5_0
    HTTPAuthMethod mode = DIGEST_AUTH;
#endif // ifdef CORE_PRE_2_5_0
    String message = F("Login Required (default user: ");
    message += www_username;
    message += ')';
    web_server.requestAuthentication(mode, message.c_str());
    return false;
  }
  return true;
}

String getControllerSymbol(byte index)
{
  String ret = F("<p style='font-size:20px'>&#");

  ret += 10102 + index;
  ret += F(";</p>");
  return ret;
}

/*
   String getValueSymbol(byte index)
   {
   String ret = F("&#");
   ret += 10112 + index;
   ret += ';';
   return ret;
   }
 */
void addSVG_param(const String& key, float value) {
  String value_str = String(value, 2);

  addSVG_param(key, value_str);
}

void addSVG_param(const String& key, const String& value) {
  String html;

  html.reserve(8 + key.length() + value.length());
  html += ' ';
  html += key;
  html += '=';
  html += '\"';
  html += value;
  html += '\"';
  addHtml(html);
}

void createSvgRect_noStroke(unsigned int fillColor, float xoffset, float yoffset, float width, float height, float rx, float ry) {
  createSvgRect(fillColor, fillColor, xoffset, yoffset, width, height, 0, rx, ry);
}

void createSvgRect(unsigned int fillColor,
                   unsigned int strokeColor,
                   float        xoffset,
                   float        yoffset,
                   float        width,
                   float        height,
                   float        strokeWidth,
                   float        rx,
                   float        ry) {
  addHtml(F("<rect"));
  addSVG_param(F("fill"), formatToHex(fillColor, F("#")));

  if (strokeWidth != 0) {
    addSVG_param(F("stroke"),       formatToHex(strokeColor, F("#")));
    addSVG_param(F("stroke-width"), strokeWidth);
  }
  addSVG_param("x",         xoffset);
  addSVG_param("y",         yoffset);
  addSVG_param(F("width"),  width);
  addSVG_param(F("height"), height);
  addSVG_param(F("rx"),     rx);
  addSVG_param(F("ry"),     ry);
  addHtml(F("/>"));
}

void createSvgHorRectPath(unsigned int color, int xoffset, int yoffset, int size, int height, int range, float SVG_BAR_WIDTH) {
  float width = SVG_BAR_WIDTH * size / range;

  if (width < 2) { width = 2; }
  String html;

  html.reserve(96);
  html += formatToHex(color, F("<path fill=\"#"));
  html += F("\" d=\"M");
  html += toString(SVG_BAR_WIDTH * xoffset / range, 2);
  html += ' ';
  html += yoffset;
  html += 'h';
  html += toString(width, 2);
  html += 'v';
  html += height;
  html += 'H';
  html += toString(SVG_BAR_WIDTH * xoffset / range, 2);
  html += F("z\"/>\n");
  addHtml(html);
}

void createSvgTextElement(const String& text, float textXoffset, float textYoffset) {
  addHtml(F("<text style=\"line-height:1.25\" x=\""));
  addHtml(toString(textXoffset, 2));
  addHtml(F("\" y=\""));
  addHtml(toString(textYoffset, 2));
  addHtml(F("\" stroke-width=\".3\" font-family=\"sans-serif\" font-size=\"8\" letter-spacing=\"0\" word-spacing=\"0\">\n"));
  addHtml(F("<tspan x=\""));
  addHtml(toString(textXoffset, 2));
  addHtml(F("\" y=\""));
  addHtml(toString(textYoffset, 2));
  addHtml("\">");
  addHtml(text);
  addHtml(F("</tspan>\n</text>"));
}

#define SVG_BAR_HEIGHT 16
#define SVG_BAR_WIDTH 400

void write_SVG_image_header(int width, int height) {
  write_SVG_image_header(width, height, false);
}

void write_SVG_image_header(int width, int height, bool useViewbox) {
  String html;

  html.reserve(128);
  html += F("<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"");
  html += width;
  html += F("\" height=\"");
  html += height;
  html += F("\" version=\"1.1\"");

  if (useViewbox) {
    html += F(" viewBox=\"0 0 100 100\"");
  }
  html += '>';
  addHtml(html);
}

/*
   void getESPeasyLogo(int width_pixels) {
   write_SVG_image_header(width_pixels, width_pixels, true);
   addHtml(F("<g transform=\"translate(-33.686 -7.8142)\"><rect x=\"49\" y=\"23.1\" width=\"69.3\" height=\"69.3\" fill=\"#2c72da\"
      stroke=\"#2c72da\"
      stroke-linecap=\"round\"stroke-linejoin=\"round\" stroke-width=\"30.7\"/><g transform=\"matrix(3.3092 0 0 3.3092 -77.788
         -248.96)\"><path d=\"m37.4 89 7.5-7.5M37.4 96.5l15-15M37.4 96.5l15-15M37.4 104l22.5-22.5M44.9 104l15-15\"
      fill=\"none\"stroke=\"#fff\" stroke-linecap=\"round\" stroke-width=\"2.6\"/><circle cx=\"58\" cy=\"102.1\" r=\"3\"
         fill=\"#fff\"/></g></g></svg>");
   }
 */
void getWiFi_RSSI_icon(int rssi, int width_pixels)
{
  const int nbars_filled = (rssi + 100) / 8;
  int nbars              = 5;
  int white_between_bar  = (static_cast<float>(width_pixels) / nbars) * 0.2;

  if (white_between_bar < 1) { white_between_bar = 1; }
  const int barWidth   = (width_pixels - (nbars - 1) * white_between_bar) / nbars;
  int svg_width_pixels = nbars * barWidth + (nbars - 1) * white_between_bar;

  write_SVG_image_header(svg_width_pixels, svg_width_pixels, true);
  float scale               = 100 / svg_width_pixels;
  const int bar_height_step = 100 / nbars;

  for (int i = 0; i < nbars; ++i) {
    unsigned int color = i < nbars_filled ? 0x0 : 0xa1a1a1; // Black/Grey
    int barHeight      = (i + 1) * bar_height_step;
    createSvgRect_noStroke(color, i * (barWidth + white_between_bar) * scale, 100 - barHeight, barWidth, barHeight, 0, 0);
  }
  addHtml(F("</svg>\n"));
}

#ifndef BUILD_MINIMAL_OTA
void getConfig_dat_file_layout() {
  const int shiftY  = 2;
  float     yOffset = shiftY;

  write_SVG_image_header(SVG_BAR_WIDTH + 250, SVG_BAR_HEIGHT + shiftY);

  int max_index, offset, max_size;
  int struct_size = 0;

  // background
  const uint32_t realSize = SettingsType::getFileSize(SettingsType::Enum::TaskSettings_Type);

  createSvgHorRectPath(0xcdcdcd, 0, yOffset, realSize, SVG_BAR_HEIGHT - 2, realSize, SVG_BAR_WIDTH);

  for (int st = 0; st < static_cast<int>(SettingsType::Enum::SettingsType_MAX); ++st) {
    SettingsType::Enum settingsType = static_cast<SettingsType::Enum>(st);

    if (SettingsType::getSettingsFile(settingsType) == SettingsType::SettingsFileEnum::FILE_CONFIG_type) {
      unsigned int color = SettingsType::getSVGcolor(settingsType);
      SettingsType::getSettingsParameters(settingsType, 0, max_index, offset, max_size, struct_size);

      for (int i = 0; i < max_index; ++i) {
        SettingsType::getSettingsParameters(settingsType, i, offset, max_size);

        // Struct position
        createSvgHorRectPath(color, offset, yOffset, max_size, SVG_BAR_HEIGHT - 2, realSize, SVG_BAR_WIDTH);
      }
    }
  }

  // Text labels
  float textXoffset = SVG_BAR_WIDTH + 2;
  float textYoffset = yOffset + 0.9 * SVG_BAR_HEIGHT;

  createSvgTextElement(SettingsType::getSettingsFileName(SettingsType::Enum::TaskSettings_Type), textXoffset, textYoffset);
  addHtml(F("</svg>\n"));
}

void getStorageTableSVG(SettingsType::Enum settingsType) {
  uint32_t realSize   = SettingsType::getFileSize(settingsType);
  unsigned int color  = SettingsType::getSVGcolor(settingsType);
  const int    shiftY = 2;

  int max_index, offset, max_size;
  int struct_size = 0;

  SettingsType::getSettingsParameters(settingsType, 0, max_index, offset, max_size, struct_size);

  if (max_index == 0) { return; }

  // One more to add bar indicating struct size vs. reserved space.
  write_SVG_image_header(SVG_BAR_WIDTH + 250, (max_index + 1) * SVG_BAR_HEIGHT + shiftY);
  float yOffset = shiftY;

  for (int i = 0; i < max_index; ++i) {
    SettingsType::getSettingsParameters(settingsType, i, offset, max_size);

    // background
    createSvgHorRectPath(0xcdcdcd, 0,      yOffset, realSize, SVG_BAR_HEIGHT - 2, realSize, SVG_BAR_WIDTH);

    // Struct position
    createSvgHorRectPath(color,    offset, yOffset, max_size, SVG_BAR_HEIGHT - 2, realSize, SVG_BAR_WIDTH);

    // Text labels
    float textXoffset = SVG_BAR_WIDTH + 2;
    float textYoffset = yOffset + 0.9 * SVG_BAR_HEIGHT;
    createSvgTextElement(formatHumanReadable(offset, 1024),   textXoffset, textYoffset);
    textXoffset = SVG_BAR_WIDTH + 60;
    createSvgTextElement(formatHumanReadable(max_size, 1024), textXoffset, textYoffset);
    textXoffset = SVG_BAR_WIDTH + 130;
    createSvgTextElement(String(i),                           textXoffset, textYoffset);
    yOffset += SVG_BAR_HEIGHT;
  }

  // usage
  createSvgHorRectPath(0xcdcdcd, 0, yOffset, max_size, SVG_BAR_HEIGHT - 2, max_size, SVG_BAR_WIDTH);

  // Struct size (used part of the reserved space)
  if (struct_size != 0) {
    createSvgHorRectPath(color, 0, yOffset, struct_size, SVG_BAR_HEIGHT - 2, max_size, SVG_BAR_WIDTH);
  }

  // Text labels
  float textXoffset = SVG_BAR_WIDTH + 2;
  float textYoffset = yOffset + 0.9 * SVG_BAR_HEIGHT;

  if (struct_size != 0) {
    String text = formatHumanReadable(struct_size, 1024);
    text += '/';
    text += formatHumanReadable(max_size, 1024);
    text += F(" per item");
    createSvgTextElement(text, textXoffset, textYoffset);
  } else {
    createSvgTextElement(F("Variable size"), textXoffset, textYoffset);
  }
  addHtml(F("</svg>\n"));
}

#endif // ifndef BUILD_MINIMAL_OTA

#ifdef ESP32

# include <esp_partition.h>

int getPartionCount(byte pType) {
  esp_partition_type_t partitionType       = static_cast<esp_partition_type_t>(pType);
  esp_partition_iterator_t _mypartiterator = esp_partition_find(partitionType, ESP_PARTITION_SUBTYPE_ANY, NULL);
  int nrPartitions                         = 0;

  if (_mypartiterator) {
    do {
      ++nrPartitions;
    } while ((_mypartiterator = esp_partition_next(_mypartiterator)) != NULL);
  }
  esp_partition_iterator_release(_mypartiterator);
  return nrPartitions;
}

void getPartitionTableSVG(byte pType, unsigned int partitionColor) {
  int nrPartitions = getPartionCount(pType);

  if (nrPartitions == 0) { return; }
  const int shiftY = 2;

  uint32_t realSize                      = getFlashRealSizeInBytes();
  esp_partition_type_t     partitionType = static_cast<esp_partition_type_t>(pType);
  const esp_partition_t   *_mypart;
  esp_partition_iterator_t _mypartiterator = esp_partition_find(partitionType, ESP_PARTITION_SUBTYPE_ANY, NULL);

  write_SVG_image_header(SVG_BAR_WIDTH + 250, nrPartitions * SVG_BAR_HEIGHT + shiftY);
  float yOffset = shiftY;

  if (_mypartiterator) {
    do {
      _mypart = esp_partition_get(_mypartiterator);
      createSvgHorRectPath(0xcdcdcd,       0,                yOffset, realSize,      SVG_BAR_HEIGHT - 2, realSize, SVG_BAR_WIDTH);
      createSvgHorRectPath(partitionColor, _mypart->address, yOffset, _mypart->size, SVG_BAR_HEIGHT - 2, realSize, SVG_BAR_WIDTH);
      float textXoffset = SVG_BAR_WIDTH + 2;
      float textYoffset = yOffset + 0.9 * SVG_BAR_HEIGHT;
      createSvgTextElement(formatHumanReadable(_mypart->size, 1024),          textXoffset, textYoffset);
      textXoffset = SVG_BAR_WIDTH + 60;
      createSvgTextElement(_mypart->label,                                    textXoffset, textYoffset);
      textXoffset = SVG_BAR_WIDTH + 130;
      createSvgTextElement(getPartitionType(_mypart->type, _mypart->subtype), textXoffset, textYoffset);
      yOffset += SVG_BAR_HEIGHT;
    } while ((_mypartiterator = esp_partition_next(_mypartiterator)) != NULL);
  }
  addHtml(F("</svg>\n"));
  esp_partition_iterator_release(_mypartiterator);
}

#endif // ifdef ESP32

bool webArg2ip(const String& arg, byte *IP) {
  return str2ip(web_server.arg(arg), IP);
}
#include "../WebServer/WiFiScanner.h"

#include "../WebServer/WebServer.h"
#include "../WebServer/AccessControl.h"
#include "../WebServer/HTML_wrappers.h"

#include "../ESPEasyCore/ESPEasyWifi.h"


#ifdef WEBSERVER_NEW_UI

// ********************************************************************************
// Web Interface Wifi scanner
// ********************************************************************************
void handle_wifiscanner_json() {
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("handle_wifiscanner"));
  #endif

  if (!isLoggedIn()) { return; }
  navMenuIndex = MENU_INDEX_TOOLS;
  TXBuffer.startJsonStream();
  addHtml("[{");
  bool firstentry = true;
  int  n          = WiFi.scanNetworks(false, true);

  for (int i = 0; i < n; ++i)
  {
    if (firstentry) { firstentry = false; }
    else { addHtml(",{"); }
    String authType;

    switch (WiFi.encryptionType(i)) {
    # ifdef ESP32
      case WIFI_AUTH_OPEN: authType            = F("open"); break;
      case WIFI_AUTH_WEP:  authType            = F("WEP"); break;
      case WIFI_AUTH_WPA_PSK: authType         = F("WPA/PSK"); break;
      case WIFI_AUTH_WPA2_PSK: authType        = F("WPA2/PSK"); break;
      case WIFI_AUTH_WPA_WPA2_PSK: authType    = F("WPA/WPA2/PSK"); break;
      case WIFI_AUTH_WPA2_ENTERPRISE: authType = F("WPA2 Enterprise"); break;
    # else // ifdef ESP32
      case ENC_TYPE_WEP:  authType = F("WEP"); break;
      case ENC_TYPE_TKIP: authType = F("WPA/PSK"); break;
      case ENC_TYPE_CCMP: authType = F("WPA2/PSK"); break;
      case ENC_TYPE_NONE: authType = F("open"); break;
      case ENC_TYPE_AUTO: authType = F("WPA/WPA2/PSK"); break;
    # endif // ifdef ESP32
      default:
        break;
    }

    if (authType.length() > 0) {
      stream_next_json_object_value(F("auth"), authType);
    }
    stream_next_json_object_value(getLabel(LabelType::SSID),      WiFi.SSID(i));
    stream_next_json_object_value(getLabel(LabelType::BSSID),     WiFi.BSSIDstr(i));
    stream_next_json_object_value(getLabel(LabelType::CHANNEL),   String(WiFi.channel(i)));
    stream_last_json_object_value(getLabel(LabelType::WIFI_RSSI), String(WiFi.RSSI(i)));
  }
  if (firstentry) {
    addHtml("}");
  }
  addHtml("]");
  TXBuffer.endStream();
}

#endif // WEBSERVER_NEW_UI

#ifdef WEBSERVER_WIFI_SCANNER

void handle_wifiscanner() {
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("handle_wifiscanner"));
  #endif

  if (!isLoggedIn()) { return; }

  WiFiMode_t cur_wifimode = WiFi.getMode();
  WifiScan(false, false);
  setWifiMode(cur_wifimode);

  navMenuIndex = MENU_INDEX_TOOLS;
  TXBuffer.startStream();
  sendHeadandTail_stdtemplate(_HEAD);
  html_table_class_multirow();
  html_TR();
  html_table_header(getLabel(LabelType::SSID));
  html_table_header(getLabel(LabelType::BSSID));
  html_table_header(F("Network info"));
  html_table_header(F("RSSI"), 50);

  const int8_t scanCompleteStatus = WiFi.scanComplete();

  if (scanCompleteStatus <= 0) {
    addHtml(F("No Access Points found"));
  }
  else
  {
    for (int i = 0; i < scanCompleteStatus; ++i)
    {
      html_TR_TD();
      int32_t rssi = 0;
      addHtml(formatScanResult(i, "<TD>", rssi));
      html_TD();
      getWiFi_RSSI_icon(rssi, 45);
    }
  }

  html_end_table();
  sendHeadandTail_stdtemplate(_TAIL);
  TXBuffer.endStream();
}

#endif // ifdef WEBSERVER_WIFI_SCANNER

