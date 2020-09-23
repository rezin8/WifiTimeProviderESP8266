/*
   ESP8266 Webserver for the Arduino Nixie Clock Firmware V1
    - Starts the ESP8266 as an access point and provides a web interface to configure and store WiFi credentials.
    - Allows the time server to be defined and stored

   Program with following settings (status line / IDE):

    Board: Generic ESP8266 Module,
    Crystal Frequency: 26MHz,
    Flash: 80MHz,
    CPU: 160MHz,
    Flash Mode: QIO,
    Upload speed: 115200,
    Flash size: 1M (64k SPIFFS),
    Reset method: ck, Disabled, none
    Erase Flash: All flash contents,
    Builtin LED: 1 (1 for ESP-01)

   Go to http://192.168.4.1 in a web browser connected to this access point to see it
*/

/*
 * 15Mar20 gml
 * Added changes from forums. Removed redunant SPIFFS call, upgraded JSON functions
 * to ArduinoJSON 6. Changed GPIO Pins to work with Aideepen Adapter Module.
 * 
 * 9Feb19 mjs
 * Added ntp and timezone support
 * ntp is preferred if available. If a blank ntp server is configured ("none"), 
 * or the NTP server is unreachable, it will fall back to the configured 
 * (non-ntp) time server.
 * 
 * .3 Initial release
 * .4 Bug fix and added OTA web browser update ability for 1 MB+ flash devices
 *    note that a lot of ESP-8266/ESP-01 modules only come with 512K. Be sure to 
 *    set the size correctly in the IDE Tools menu. It looks like Ian sources 1 MB 
 *    ones, so they should work with OTA.
 * .5 Added tweaks and basic security for pages which can cause harmful changes
 *    (web: admin/setup, update: admin/update)
 *    now syncs clock to NTP within ~25 ms.
 *    bumped EEPROM size to 1024 for future needs.
 * .6 Changed some defaults, more tweaks. Added user/pass hint when in softAP mode.
 *    Added some bounds checking, support for V1 clocks running i2c protocol v54 (untested).
 * .7 Try to sync NTP immediately after changing time server settings. Some refactoring of I2C
 *    clock reading stuff to try and get V1 compatibility working. Now get a compiler warning 
 *    about "uint8_t available = Wire.requestFrom((int)preferredI2CSlaveAddress, 1);" which is
 *    counterintuitive, but causes no harm (C++ experts welcome).
 * .8 Gave /setvalue a 5 second default timeout. Refactored globals. Better I2C scan detail. Changed 
 *    NTP sync to 5 ms. Added Utility page. 7Feb19
 * .9 Show NTP sync status. Revamped status page. Always compile for OTA, enable at run
 *    time based on sketch size and available space. 
 *.10 Fixed time zone save bug.    
 *
*/

#include <FS.h>
#include <DNSServer.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPClient.h>
#include <Wire.h>

#include <WiFiManager.h>          // https://github.com/tzapu/WiFiManager  (0.15.0)
#include <ArduinoJson.h>          // https://github.com/bblanchon/ArduinoJson (6.14.1)
#include <ezTime.h>               // https://github.com/ropg/ezTime (0.8.3)

// Other parts of the code, broken out for clarity
#include "I2CDefs.h"
  
#define SOFTWARE_VERSION "v357gml.10"
#define SERIAL_NUMBER "Rev3 xp001"

#define BUILD_NOTES "Includes DST change and timeNeedsSync fixes for ezTime 0.7.2 library \
                     </br>test line"

/* This will compile in OTA support - need at least 1 MB for OTA updates.
   we check available space at runtime before allowing it.
*/
#include <ESP8266HTTPUpdateServer.h>
ESP8266HTTPUpdateServer httpUpdater;
const char* update_path = "/update";

// security stuff
#define WEB_USERNAME "admin"
#define WEB_PASSWORD "setup"
#define AP_SSID "NixieTimeModule"   
#define AP_PSK  "SetMeUp!"                                // to add password
const char* update_username = "admin";
const char* update_password = "update";

// time stuff
#define DEFAULT_TIME_SERVER_URL_1 "http://time-zone-server.scapp.io/getTime/America/Detroit"
#define DEFAULT_TIME_SERVER_URL_2 "http://its.internet-box.ch/getTime/America/Detroit"

#define DEFAULT_NTP_TZ "EST5EDT,M3.2.0,M11.1.0"
#define DEFAULT_NTP_TZ "EST5EDT,M3.2.0,M11.1.0"   //POSIX format
#define DEFAULT_NTP_INTERVAL 62                 // seconds, best if not a multiple of 60
#define DEFAULT_NTP_SERVER "ntp.gledet.com"
#define MIN_NTP_INTERVAL 31                      // seconds
#define MAX_NTP_INTERVAL 64999                    // ~18 hours

#define DEBUG_ON                                  // DEBUG_[ON|OFF] enable debugging output (no blue LED on ESP-01)

#define EZT_DBG_LEVEL DEBUG                       // ezTime - NONE, ERROR, INFO, DEBUG

int blinkMode = 2; // 0 = Connected, short flash, 1 = connected, connected, no time server, 2 = not connected

// ----------------------------------------------------------------------------------------------------
// ------------------------------------------  Globals (mostly)  --------------------------------------
// ----------------------------------------------------------------------------------------------------
const char *ap_ssid = AP_SSID;
const char *ap_password = AP_PSK;
const char* web_username = WEB_USERNAME;
const char* web_password = WEB_PASSWORD;

boolean blueLedState = true;
// used for flashing the blue LED
int blinkOnTime = 1000;
int blinkTopTime = 2000;
unsigned long lastMillis = 0;

// Timer for how often we send the I2C data
long lastI2CUpdateTime = 0;
byte preferredI2CSlaveAddress = 0xFF;
byte preferredAddressFoundBy = 0; // 0 = not found, 1 = found by default, 2 = found by ping

String timeServerURL = DEFAULT_TIME_SERVER_URL_1;

ADC_MODE(ADC_VCC);

// Clock config
byte configHourMode;
byte configBlankLead;
byte configScrollback;
byte configSuppressACP;
byte configDateFormat;
byte configDayBlanking;
byte configBlankFrom;
byte configBlankTo;
byte configFadeSteps;
byte configScrollSteps;
byte configBacklightMode;
byte configRedCnl;
byte configGreenCnl;
byte configBlueCnl;
byte configCycleSpeed;
byte configUseFade;
byte configUseLDR;
byte configBlankMode;
byte configSlotsMode;
unsigned int configPirTimeout;
unsigned int configMinDim;
const char *serialNumber = SERIAL_NUMBER;

String ntpServer;
String ntpTZ;
String tzName;
String tzPosix;
unsigned short int ntpInterval;
boolean wlanConnected = false;
boolean largeFlash = false;

enum timeSources {none, NTP, TimeServer};
timeSources timeSource ; // 0=none, 1=ntp, 2=timeserver

enum i2cProtocols {unKnown, v2v62, v1v54};
i2cProtocols i2cProtocol ;

ESP8266WebServer server(80);
Timezone myTz;

WiFiManager wifiManager;

//gets called when WiFiManager enters configuration mode
void configModeCallback (WiFiManager *myWiFiManager) {
  debugMsg("Entered config mode");
  debugMsg(formatIPAsString(WiFi.softAPIP()));
  //if you used auto generated SSID, print it
  debugMsg(myWiFiManager->getConfigPortalSSID());
}

// ----------------------------------------------------------------------------------------------------
// ----------------------------------------------  Set up  --------------------------------------------
// ----------------------------------------------------------------------------------------------------
void setup()
{  
#ifdef DEBUG_ON
  setupDebug();
  wifiManager.setDebugOutput(true);
#else
  pinMode(LED_BUILTIN, OUTPUT);
  wifiManager.setDebugOutput(false);
#endif

  if (setOTAUpdates()) {
    debugMsg("  --> Set OTA Updates on");
  } else {
    debugMsg("  --> Set OTA Updates OFF");
  }
  //set callback that gets called when connecting to previous WiFi fails, and enters Access Point mode
  wifiManager.setAPCallback(configModeCallback);
  
  wifiManager.setConfigPortalTimeout(60);

  boolean connected = false;
  while (!connected) {
    #ifndef DEBUG
      for (int i = 0 ; i < 5 ; i++) {
      setBlueLED(false);
      delay(100);
      setBlueLED(true);
      delay(100);
    }
    #endif
    connected = wifiManager.autoConnect("NixieClockTimeModule","SetMeUp!");
  }

  #ifdef DEBUG_ON
    debugMsg("Connected!");
  #else  
    for (int i = 0 ; i < 3 ; i++) {
      setBlueLED(false);
      delay(50);
      setBlueLED(true);
      delay(500);
    }
  #endif

  /*
  * Locations of configs
  * ---
  * SSID 0-31 (32) -> WiFi Mnanager
  * Password 32-95 (64) -> WiFi Mnanager
  * Timeserver 96-351 (256) -> SPIFFS
  * ntpServer 352-415 (64) -> SPIFFS
  * ntpTZ 416-469 (64) -> SPIFFS
  * ntpInterval 470-471 (2) -> SPIFFS
  */

  if(getConfigfromSpiffs()) {
    debugMsg("Recovered Config from SPIFFS");
    myTz.setPosix(ntpTZ);
  } else {
    debugMsg("Setting default configs");
    resetConfigs();
  }
  
  IPAddress apIP = WiFi.softAPIP();
  IPAddress myIP = WiFi.localIP();
  debugMsg("AP IP address: " + formatIPAsString(apIP));
  debugMsg("IP address: " + formatIPAsString(myIP));

  Wire.begin(0, 2); // SDA = 0, SCL = 2 for standard install 
//  Wire.begin(1, 3); // SDA/TX = 1, SCL/RX = 3 for add-on board
  debugMsg("I2C master started");
  
  /* Set page handler functions */
  server.on("/",            rootPageHandler);  // no auth here

  server.on("/time", []() {
    if (!server.authenticate(web_username, web_password)) {
      return server.requestAuthentication();
    }
    return timeServerPageHandler();
  });

  server.on("/reset", []() {
    if (!server.authenticate(web_username, web_password)) {
      return server.requestAuthentication();
    }
    return resetPageHandler();
  });

  server.on("/resetWiFi", []() {
    if (!server.authenticate(web_username, web_password)) {
      return server.requestAuthentication();
    }
    return resetWiFiPageHandler();
  });

  server.on("/updatetime",   updateTimePageHandler); // no auth here
    
  server.on("/clockconfig", []() {
    if (!server.authenticate(web_username, web_password)) {
      return server.requestAuthentication();
    }
    return clockConfigPageHandler();
  });

  server.on("/setvalue", []() {
    if (!server.authenticate(web_username, web_password)) {
      return server.requestAuthentication();
    }
    return setDisplayValuePageHandler();
  });

  server.on("/wlan_config", []() { //added because this page was missing ******************************************
    if (!server.authenticate(web_username, web_password)) {
      return server.requestAuthentication();
    }
    return setDisplayValuePageHandler();
  });

  server.on("/utility", []() {
  if (!server.authenticate(web_username, web_password)) {
    return server.requestAuthentication();
  }
    return utilityPageHandler();
  });

  server.on("/local.css",   localCSSHandler); // no auth here
  server.onNotFound(handleNotFound);          // no auth here

  scanI2CBus();


  // Allow the IP to be displayed on the clock
  // this allows the user to know the address to log into
  // We need to send a time update and the IP
  sendIPAddressToI2C(WiFi.localIP());
  sendTimeToI2C("2018,01,01,0,0,0");
  
  server.begin();
  debugMsg("HTTP server started");
}

// ----------------------------------------------------------------------------------------------------
// --------------------------------------------- Main Loop --------------------------------------------
// ----------------------------------------------------------------------------------------------------
void loop() {
  server.handleClient();                            // for web server
  events();                                         // for NTP
  if (WiFi.status() == WL_CONNECTED) {
    wlanConnected = true;
    if (lastMillis > millis()) {                    // rollover
      lastI2CUpdateTime = 0;
    }

    if (((millis() - lastI2CUpdateTime) > 60000) || // See if it is time to update the Clock
        (lastI2CUpdateTime==0)) {

      // Try to recover the current time
      String timeStr = getTimeFromTimeZoneServer();

      // Send the time to the I2C client, but only if there was no error
      // note that NTP is tried first, but fails over to the TZ time server,
      // so we still get this error if we're not getting any external time.
      if (!timeStr.startsWith("ERROR:")) {
        sendTimeToI2C(timeStr);

        blinkMode = 0;

        // all OK, flash 10 millisecond per second
        debugMsg("Normal time serve mode");
      } else {
        // connected, but time server not found, flash middle speed
        blinkMode = 1;
        debugMsg("Connected, but no time server found");
      }

      // Allow the IP to be displayed on the clock
      sendIPAddressToI2C(WiFi.localIP());
      lastI2CUpdateTime = millis();
    }
  } else {
    // offline, flash fast
    blinkMode = 2;
  }


  switch (blinkMode) {
    case 0:
      {
        blinkOnTime = 5;
        blinkTopTime = 1000;
        break;
      }
    case 1:
      {
        blinkOnTime = 250;
        blinkTopTime = 1000;
        break;
      }
    case 2:
      {
        blinkOnTime = 250;
        blinkTopTime = 500;
        break;
      }
  }

  unsigned long nowMillis = millis();
  if ((nowMillis - lastMillis) > blinkTopTime) {
    lastMillis = nowMillis;
  }

  setBlueLED((nowMillis - lastMillis) < blinkOnTime);
}

// ----------------------------------------------------------------------------------------------------
// ------------------------------------------- Page Handlers ------------------------------------------
// ----------------------------------------------------------------------------------------------------

/**
   Root page for the webserver
*/
void rootPageHandler()
{
  String response_message = getHTMLHead();
  String timeSourceString;
  String currentTime;
  
  response_message += getNavBar();

  // Status table
  response_message += getTableHead2Col("Current Status", "Name", "Value");

  if ( wlanConnected )
  {
//    if ( wlanConnected ) {updateNTP();}  // force immediate update
    switch( timeSource ) {
      case NTP:          timeSourceString = "NTP";
                         currentTime = myTz.dateTime(RFC850);
                         break;
      case TimeServer:   timeSourceString = "Timeserver";
                         currentTime = getTimeFromTimeZoneServer();
                         break;
      default:           timeSourceString = "none";
    }
    response_message += getTableRow2Col("Current time source", timeSourceString);
    response_message += getTableRow2Col("Current time", currentTime);
    response_message += getTableRow2Col("NTP server", ntpServer);
    switch( timeStatus() ) {
      case timeNotSet:     timeSourceString = "No sync yet"; break;
      case timeNeedsSync:  timeSourceString = "No contact for polling interval + 1 hour"; break;
      case timeSet:        timeSourceString = "Last poll valid"; break;
      default:             timeSourceString = "Unknown"; 
    }
    response_message += getTableRow2Col("NTP status", timeSourceString);
    response_message += getTableRow2Col("Time server URL", timeServerURL);
    IPAddress ip = WiFi.localIP();
    response_message += getTableRow2Col("WLAN IP", formatIPAsString(ip));
    response_message += getTableRow2Col("WLAN MAC", WiFi.macAddress());
    response_message += getTableRow2Col("WLAN SSID", WiFi.SSID());
    response_message += getTableRow2Col("Time server URL", timeServerURL);
    response_message += getTableRow2Col("Time according to server", getTimeFromTimeZoneServer());
  }
  else
  {
    response_message += getTableRow2Col("name/password to configure<br/>WiFi (WLAN) connection",String(web_username)+"/"+String(web_password));
    IPAddress softapip = WiFi.softAPIP();
    response_message += getTableRow2Col("AP IP", formatIPAsString(softapip));
    response_message += getTableRow2Col("AP MAC", WiFi.softAPmacAddress());
  }

  String lastUpdateString = String(1 + (60000 - (millis() - lastI2CUpdateTime))/1000) + "s";
  lastUpdateString += " (<a href=\"/updatetime\">do full update now</a>)";
  response_message += getTableRow2Col("Next clock update in", lastUpdateString);

  // Make the uptime readable
  long upSecs = millis() / 1000;
  long upDays = upSecs / 86400;
  long upHours = (upSecs - (upDays * 86400)) / 3600;
  long upMins = (upSecs - (upDays * 86400) - (upHours * 3600)) / 60;
  upSecs = upSecs - (upDays * 86400) - (upHours * 3600) - (upMins * 60);
  String uptimeString = ""; uptimeString += upDays; uptimeString += " days, "; uptimeString += upHours, uptimeString += " hours, "; uptimeString += upMins; uptimeString += " mins, "; uptimeString += upSecs; uptimeString += " secs";

  response_message += getTableRow2Col("Uptime", uptimeString);

  response_message += getTableRow2Col("Time last update", lastUpdateString);

  response_message += getTableRow2Col("Version", SOFTWARE_VERSION);
  response_message += getTableRow2Col("Serial Number", serialNumber);

  // Scan I2C bus
  for (int idx = 0 ; idx < 128 ; idx++) { 
    Wire.beginTransmission(idx);
    int error = Wire.endTransmission();
    String slaveMsg = "Found I2C slave ";
    if (error == 0) {
      if (idx == 87) {
        slaveMsg += "(24xx EEPROM on RTC)";
      } else if (idx == 104) {
        slaveMsg += "(DS3231 RTC)";
      } else if (idx == 105) {
        slaveMsg += "(Nixie clock)";
      } else {
        slaveMsg += "(unknown)";
      }
      if (idx == preferredI2CSlaveAddress) {
        if (preferredAddressFoundBy == 1) {
          slaveMsg += " (default)";
        } else if (preferredAddressFoundBy == 2) {
          slaveMsg += " (ping, preferred)";
        }
      }
      response_message += getTableRow2Col(slaveMsg,idx);
    }
  }
  response_message += getTableFoot();

  float voltaje = (float)ESP.getVcc()/(float)1024;
  voltaje -= 0.01f;  // by default reads high
  char dtostrfbuffer[15];
  dtostrf(voltaje,7, 2, dtostrfbuffer);
  String vccString = String(dtostrfbuffer);

  // ESP8266 Info table
  response_message += getTableHead2Col("Platform Information", "Name", "Value");
  response_message += getTableRow2Col("Sketch compiled", dateTime(compileTime(),"l, d-M-y G:i:s ~U~T~C"));
  response_message += getTableRow2Col("Build notes",BUILD_NOTES);
  response_message += getTableRow2Col("Sketch size", ESP.getSketchSize());
  String ota = String(ESP.getFreeSketchSpace());
  if ( largeFlash ) { ota += " (OTA update capable)"; }
  response_message += getTableRow2Col("Free sketch size", ota);
  response_message += getTableRow2Col("Free heap", ESP.getFreeHeap());
  response_message += getTableRow2Col("Boot version", ESP.getBootVersion());
  response_message += getTableRow2Col("CPU Freqency (MHz)", ESP.getCpuFreqMHz());
  response_message += getTableRow2Col("SDK version", ESP.getSdkVersion());
  response_message += getTableRow2Col("Chip ID", ESP.getChipId());
  response_message += getTableRow2Col("Flash Chip ID", String(ESP.getFlashChipId(), HEX));
  response_message += getTableRow2Col("Flash size", ESP.getFlashChipRealSize());
  response_message += getTableRow2Col("Last reset reason", ESP.getResetReason());
  response_message += getTableRow2Col("Vcc", vccString);
  response_message += getTableFoot();

  response_message += getHTMLFoot();

  server.send(200, "text/html", response_message);
}

// ===================================================================================================================
// ===================================================================================================================

/**
   Get the local time from the time server, and modify the time server URL if needed
*/
void timeServerPageHandler() {
  String serverArg = "";
  // Check if there are any GET parameters, if there are, we are configuring
  if (server.hasArg("timeserverurl"))
  {
    serverArg = String(server.arg("timeserverurl").c_str());
    if (serverArg.length() > 4 && serverArg.length() < 256) {
      timeServerURL = server.arg("timeserverurl").c_str();
      // storeTimeServerURLInEEPROM(timeServerURL);
    }
  }

  if (server.hasArg("ntpserver"))
  {
    serverArg = String(server.arg("ntpserver").c_str());
    if ( serverArg.length() > 4 && serverArg.length() < 64 ) {
      ntpServer = server.arg("ntpserver").c_str();
      saveConfigToSpiffs();
      setServer(ntpServer);
      waitForSync(15);                                      // try to sync with new server
    } 
    else {
      setInterval(0);
      setServer("none");
      ntpServer = "none";
      saveConfigToSpiffs();
    }
  }

 if (server.hasArg("ntptz"))
  {
    serverArg = String(server.arg("ntptz").c_str());
    if ( serverArg.length() > 4 && serverArg.length() < 64 ) {
      lastI2CUpdateTime = 0;                               // force an update to the clock
      ntpTZ = server.arg("ntptz").c_str();
      saveConfigToSpiffs();
      myTz.setPosix(ntpTZ);
    }
  }

 if (server.hasArg("ntpinterval"))
  {
    if (strlen(server.arg("ntpinterval").c_str()) > 2) {
      unsigned int value = atoi(server.arg("ntpinterval").c_str());
      if ( value >= MIN_NTP_INTERVAL && value <= MAX_NTP_INTERVAL ) {
        ntpInterval = value;
        saveConfigToSpiffs();
        setInterval(ntpInterval);
      }  
    }  
  }

 if (server.hasArg("tzname"))
  {
    serverArg = String(server.arg("tzname").c_str());
    if ( serverArg.length() > 3 && serverArg.length() < 64 ) {
      tzName = server.arg("tzname").c_str();
      if (myTz.setLocation(tzName)) {  // also sets TZ string, so we restore after
        tzPosix = myTz.getPosix();
        myTz.setPosix(ntpTZ);          // restore the existing string
      }
    }
  }

  String response_message = getHTMLHead();
  response_message += getNavBar();

  // form header
  response_message += getFormHead("Configure time servers");

  // only fill in the value we have if it looks realistic
  if ((timeServerURL.length() < 10) || (timeServerURL.length() > 250)) {
    debugMsg("Got a bad URL for the TZS, resetting");
    timeServerURL = DEFAULT_TIME_SERVER_URL_1;
  } else {
    // save the timeserver URL
    saveConfigToSpiffs();
  }

  response_message += getTextInputWide("NTP server<br/>(name or IP)", "ntpserver", ntpServer, false);
  String foo = "NTP polling interval<br/> ("+String(MIN_NTP_INTERVAL)+"-"+String(MAX_NTP_INTERVAL)+" seconds)";
  response_message += getTextInputWide(foo, "ntpinterval", String(ntpInterval), false);
  response_message += getTextInputWide("POSIX TZ<br/>(see below)", "ntptz", ntpTZ, false);
  response_message += getTextInputWide("Time service URL<br/>(backup to NTP)", "timeserverurl", timeServerURL, false);
  response_message += getSubmitButton("Set");
  response_message += "<br/><hr/>";
  response_message += "You can try a POSIX lookup by entering an ";
  response_message += "<a href=\"https://en.wikipedia.org/wiki/List_of_tz_database_time_zones\" target=\"_blank\">";
  response_message += "Olson name</a> (like America/Detroit) here, and then copy/paste it to the field above. ";
  // response_message += "TZ changes may take a minute to appear on the clock."
  response_message += getTextInputWide("Olson name", "tzname", tzName, false);
  response_message += getTextInputWide("POSIX TZ string:", "tzposix", tzPosix, true);
  response_message += getSubmitButton("Lookup");
  response_message += "<br/><hr/>";
  response_message += "<a href=\"http://www.gnu.org/software/libc/manual/html_node/TZ-Variable.html\" target=\"_blank\">";
  response_message += "POSIX time zone string reference,</a> e.g. US Eastern is <b>EST5EDT,M3.2.0,M11.1.0</b>. ";
  response_message += "<br/><hr/>";
  response_message += "The NTP poll time should <i>not</i> be set to a multiple of 60 seconds. That will help spread ";
  response_message += "out the load on the NTP servers. 7201 seconds is good, 7200 is not. If using an ntp.org pool ";
  response_message += "server, polls should happen no more often than every 1/2 hour (1800 seconds) to comply with ";
  response_message += "their terms of service. Start by setting the polling interval to a high value then observe ";
  response_message += "how well the clock stays in sync. If it gets more than a second out of sync during that period, ";
  response_message += "lower it. NTP providers will thank you. Regardless of the polling period, the clock will ";
  response_message += "be sent a time update every 60 seconds.";
  response_message += getFormFoot();
  response_message += getHTMLFoot();

  server.send(200, "text/html", response_message);
}

// ===================================================================================================================
// ===================================================================================================================

/**
   Get the local time from the time server, and send it via I2C right now
*/
void updateTimePageHandler()
{
  updateNTP();  // force an immediate poll
  String timeString = getTimeFromTimeZoneServer();

  String response_message = getHTMLHead();
  response_message += getNavBar();


  if (timeString.substring(1,6) == "ERROR:") {
    response_message += "<div class=\"container\" role=\"main\"><h3 class=\"sub-header\">Send time to I2C right now</h3>";
    response_message += "<div class=\"alert alert-danger fade in\"><strong>Error!</strong> Could not recover the time from time server. ";
    response_message += timeString;
    response_message += "</div></div>";
  } else {
    boolean result = sendTimeToI2C(timeString);

    response_message += "<div class=\"container\" role=\"main\"><h3 class=\"sub-header\">Send time to I2C right now</h3>";
    if (result) {
      response_message += "<div class=\"alert alert-success fade in\"><strong>Success!</strong>Time received, update sent.</div></div>";
      lastI2CUpdateTime = millis();
    } else {
      response_message += "<div class=\"alert alert-danger fade in\"><strong>Error!</strong> Update was not sent.</div></div>";
    }
  }
  response_message += "<script> var timer = setTimeout(function() {window.location='/'}, 3000);</script>";
  response_message += getHTMLFoot();

  server.send(200, "text/html", response_message);
}

// ===================================================================================================================
// ===================================================================================================================

/**
   Reset the EEPROM and stored values
*/
void resetPageHandler() {
  wifiManager.resetSettings();

  String response_message = getHTMLHead();
  response_message += getNavBar();
  response_message += "<div class=\"container\" role=\"main\"><h3 class=\"sub-header\">Send time to I2C right now</h3>";
  response_message += "<div class=\"alert alert-success fade in\"><strong>Success!</strong> Reset done.</div>";
  response_message += "<div class=\"alert alert-success fade in\">Attempting reboot, but power cycle if needed. ";
  response_message += "You will then need to connect to the <b>"+String(ap_ssid)+"</b> SSID and open ";
  response_message += "<b>http://192.168.4.1</b> in a web browser to reconfigure.</div></div>";
  response_message += getHTMLFoot();
  resetConfigs();
  server.send(200, "text/html", response_message);
  delay(1000);    // wait to deliver response
  ESP.restart();  // should reboot
}

/**
   Reset the ESP card
*/
void resetWiFiPageHandler() {
  String response_message = getHTMLHead();
  response_message += getNavBar();
  response_message += "<div class=\"alert alert-success fade in\"><strong>Attempting a restart.</strong>";
  response_message += "<script> var timer = setTimeout(function() {window.location='/'}, 10000);</script>";  
  response_message += getHTMLFoot();
  server.send(200, "text/html", response_message);
  delay(1000);    // wait to deliver response
  ESP.restart();  // should reboot
}

// ===================================================================================================================
// ===================================================================================================================

/**
   Page for the clock configuration.
*/
void clockConfigPageHandler()
{
  // -----------------------------------------------------------------------------

  if (server.hasArg("12h24hMode"))
  {
    debugMsg("Got 24h mode param: " + server.arg("12h24hMode"));
    if ((server.arg("12h24hMode") == "24h") && (configHourMode)) {
      debugMsg("I2C --> Set 24h mode");
      setClockOption12H24H(true);
    }

    if ((server.arg("12h24hMode") == "12h")  && (!configHourMode)) {
      debugMsg("I2C --> Set 12h mode");
      setClockOption12H24H(false);
    }
  }

  // -----------------------------------------------------------------------------

  if (server.hasArg("blankLeading"))
  {
    debugMsg("Got blankLeading param: " + server.arg("blankLeading"));
    if ((server.arg("blankLeading") == "blank") && (!configBlankLead)) {
      debugMsg("I2C --> Set blank leading zero");
      setClockOptionBlankLeadingZero(false);
    }

    if ((server.arg("blankLeading") == "show") && (configBlankLead)) {
      debugMsg("I2C --> Set show leading zero");
      setClockOptionBlankLeadingZero(true);
    }
  }

  // -----------------------------------------------------------------------------

  if (server.hasArg("useScrollback"))
  {
    debugMsg("Got useScrollback param: " + server.arg("useScrollback"));
    if ((server.arg("useScrollback") == "on") && (!configScrollback)) {
      debugMsg("I2C --> Set scrollback on");
      setClockOptionScrollback(false);
    }

    if ((server.arg("useScrollback") == "off") && (configScrollback)) {
      debugMsg("I2C --> Set scrollback off");
      setClockOptionScrollback(true);
    }
  }

  // -----------------------------------------------------------------------------

  if (server.hasArg("suppressACP"))
  {
    debugMsg("Got suppressACP param: " + server.arg("suppressACP"));
    if ((server.arg("suppressACP") == "on") && (!configSuppressACP)) {
      debugMsg("I2C --> Set suppressACP on");
      setClockOptionSuppressACP(false);
    }

    if ((server.arg("suppressACP") == "off") && (configSuppressACP)) {
      debugMsg("I2C --> Set suppressACP off");
      setClockOptionSuppressACP(true);
    }
  }

  // -----------------------------------------------------------------------------

  if (server.hasArg("useFade"))
  {
    debugMsg("Got useFade param: " + server.arg("useFade"));
    if ((server.arg("useFade") == "on") && (!configUseFade)) {
      debugMsg("I2C --> Set useFade on");
      setClockOptionUseFade(false);
    }

    if ((server.arg("useFade") == "off") && (configUseFade)) {
      debugMsg("I2C --> Set useFade off");
      setClockOptionUseFade(true);
    }
  }

  // -----------------------------------------------------------------------------

  if (server.hasArg("useLDR"))
  {
    debugMsg("Got useLDR param: " + server.arg("useLDR"));
    if ((server.arg("useLDR") == "on") && (!configUseLDR)) {
      debugMsg("I2C --> Set useLDR on");
      setClockOptionUseLDR(false);
    }

    if ((server.arg("useLDR") == "off") && (configUseLDR)) {
      debugMsg("I2C --> Set useLDR off");
      setClockOptionUseLDR(true);
    }
  }

  // -----------------------------------------------------------------------------

  if (server.hasArg("dateFormat")) {
    debugMsg("Got dateFormat param: " + server.arg("dateFormat"));
    byte newDateFormat = atoi(server.arg("dateFormat").c_str());
    if (newDateFormat != configDateFormat) {
      setClockOptionDateFormat(newDateFormat);
      debugMsg("I2C --> Set dateFormat: " + newDateFormat);
    }
  }

  // -----------------------------------------------------------------------------

  if (server.hasArg("dayBlanking")) {
    debugMsg("Got dayBlanking param: " + server.arg("dayBlanking"));
    byte newDayBlanking = atoi(server.arg("dayBlanking").c_str());
    if (newDayBlanking != configDayBlanking) {
      setClockOptionDayBlanking(newDayBlanking);
      debugMsg("I2C --> Set dayBlanking: " + newDayBlanking);
    }
  }

  // -----------------------------------------------------------------------------

  if (server.hasArg("blankFrom")) {
    debugMsg("Got blankFrom param: " + server.arg("blankFrom"));
    byte newBlankFrom = atoi(server.arg("blankFrom").c_str());
    if (newBlankFrom != configBlankFrom) {
      setClockOptionBlankFrom(newBlankFrom);
      debugMsg("I2C --> Set blankFrom: " + newBlankFrom);
    }
  }

  // -----------------------------------------------------------------------------

  if (server.hasArg("blankTo")) {
    debugMsg("Got blankTo param: " + server.arg("blankTo"));
    byte newBlankTo = atoi(server.arg("blankTo").c_str());
    if (newBlankTo != configBlankTo) {
      setClockOptionBlankTo(newBlankTo);
      debugMsg("I2C --> Set blankTo: " + newBlankTo);
    }
  }

  // -----------------------------------------------------------------------------

  if (server.hasArg("fadeSteps")) {
    debugMsg("Got fadeSteps param: " + server.arg("fadeSteps"));
    byte newFadeSteps = atoi(server.arg("fadeSteps").c_str());
    if (newFadeSteps != configFadeSteps) {
      setClockOptionFadeSteps(newFadeSteps);
      debugMsg("I2C --> Set fadeSteps: " + newFadeSteps);
    }
  }

  // -----------------------------------------------------------------------------

  if (server.hasArg("scrollSteps")) {
    debugMsg("Got scrollSteps param: " + server.arg("scrollSteps"));
    byte newScrollSteps = atoi(server.arg("scrollSteps").c_str());
    if (newScrollSteps != configScrollSteps) {
      setClockOptionScrollSteps(newScrollSteps);
      debugMsg("I2C --> Set fadeSteps: " + newScrollSteps);
    }
  }

  // -----------------------------------------------------------------------------

  if (server.hasArg("backLight")) {
    debugMsg("Got backLight param: " + server.arg("backLight"));
    byte newBacklight = atoi(server.arg("backLight").c_str());
    if (newBacklight != configBacklightMode) {
      setClockOptionBacklightMode(newBacklight);
      debugMsg("I2C --> Set backLight: " + newBacklight);
    }
  }

  // -----------------------------------------------------------------------------

  if (server.hasArg("redCnl")) {
    debugMsg("Got redCnl param: " + server.arg("redCnl"));
    byte newRedCnl = atoi(server.arg("redCnl").c_str());
    if (newRedCnl != configRedCnl) {
      setClockOptionRedCnl(newRedCnl);
      debugMsg("I2C --> Set redCnl: " + newRedCnl);
    }
  }

  // -----------------------------------------------------------------------------

  if (server.hasArg("grnCnl")) {
    debugMsg("Got grnCnl param: " + server.arg("grnCnl"));
    byte newGreenCnl = atoi(server.arg("grnCnl").c_str());
    if (newGreenCnl != configGreenCnl) {
      setClockOptionGrnCnl(newGreenCnl);
      debugMsg("I2C --> Set grnCnl: " + newGreenCnl);
    }
  }

  // -----------------------------------------------------------------------------

  if (server.hasArg("bluCnl")) {
    debugMsg("Got bluCnl param: " + server.arg("bluCnl"));
    byte newBlueCnl = atoi(server.arg("bluCnl").c_str());
    if (newBlueCnl != configBlueCnl) {
      setClockOptionBluCnl(newBlueCnl);
      debugMsg("I2C --> Set bluCnl: " + newBlueCnl);
    }
  }

  // -----------------------------------------------------------------------------

  if (server.hasArg("cycleSpeed")) {
    debugMsg("Got cycleSpeed param: " + server.arg("cycleSpeed"));
    byte newCycleSpeed = atoi(server.arg("cycleSpeed").c_str());
    if (newCycleSpeed != configCycleSpeed) {
      setClockOptionCycleSpeed(newCycleSpeed);
      debugMsg("I2C --> Set cycleSpeed: " + newCycleSpeed);
    }
  }

  // -----------------------------------------------------------------------------

  if (server.hasArg("blankMode")) {
    debugMsg("Got blankMode param: " + server.arg("blankMode"));
    byte newBlankMode = atoi(server.arg("blankMode").c_str());
    if (newBlankMode != configBlankMode) {
      setClockOptionBlankMode(newBlankMode);
      debugMsg("I2C --> Set blankMode: " + newBlankMode);
    }
  }

  // -----------------------------------------------------------------------------

  if (server.hasArg("slotsMode")) {
    debugMsg("Got slotsMode param: " + server.arg("slotsMode"));
    byte newSlotsMode = atoi(server.arg("slotsMode").c_str());
    if (newSlotsMode != configSlotsMode) {
      setClockOptionSlotsMode(newSlotsMode);
      debugMsg("I2C --> Set slotsMode: " + newSlotsMode);
    }
  }

  // -----------------------------------------------------------------------------
  if ( i2cProtocol == v2v62 ) {
    if (server.hasArg("pirTimeout")) {
      debugMsg("Got pirTimeout param: " + server.arg("pirTimeout"));
      int newPirTimeOut = atoi(server.arg("pirTimeout").c_str());
      if (newPirTimeOut != configPirTimeout) {
        setClockOptionPirTimeout(newPirTimeOut);
        debugMsg("I2C --> Set pirTimeout: " + newPirTimeOut);
      }
    }
  }
  // -----------------------------------------------------------------------------

  if (server.hasArg("minDim")) {
    debugMsg("Got minDim param: " + server.arg("minDim"));
    int newMinDim = atoi(server.arg("minDim").c_str());
    if (newMinDim != configMinDim) {
      setClockOptionMinDim(newMinDim);
      debugMsg("I2C --> Set minDim: " + newMinDim);
    }
  }

  // -----------------------------------------------------------------------------

  // Get the options, put the result into variables called "config*"
  getClockOptionsFromI2C();

  String response_message = getHTMLHead();
  response_message += getNavBar();

  // form header
  response_message += getFormHead("Set Configuration");
  String proto = "";
  switch(i2cProtocol) {
    case v1v54:   proto = "NixieFirmwareV1, I2C v54";
                  break;
    case v2v62:   proto = "NixieFirmwareV2, I2C v62";
                  break;
    default:      proto = "<strong>Clock not found!</strong>";
  }  
  proto += "<hr/>";
  response_message += getTableRow2Col("Communicating with: ", proto);
  
  // 12/24 mode
  response_message += getRadioGroupHeader("12H/24H mode:");
  if (configHourMode == 0) {
    response_message += getRadioButton("12h24hMode", " 12H", "12h", false);
    response_message += getRadioButton("12h24hMode", " 24H", "24h", true);
  } else {
    response_message += getRadioButton("12h24hMode", " 12H", "12h", true);
    response_message += getRadioButton("12h24hMode", " 24H", "24h", false);
  }
  response_message += getRadioGroupFooter();

  // blank leading
  response_message += getRadioGroupHeader("Blank leading zero:");
  if (configBlankLead) {
    response_message += getRadioButton("blankLeading", "Blank", "blank", true);
    response_message += getRadioButton("blankLeading", "Show", "show", false);
  } else {
    response_message += getRadioButton("blankLeading", "Blank", "blank", false);
    response_message += getRadioButton("blankLeading", "Show", "show", true);
  }
  response_message += getRadioGroupFooter();

  // Scrollback
  response_message += getRadioGroupHeader("Scrollback effect:");
  if (configScrollback) {
    response_message += getRadioButton("useScrollback", "On", "on", true);
    response_message += getRadioButton("useScrollback", "Off", "off", false);
  } else {
    response_message += getRadioButton("useScrollback", "On", "on", false);
    response_message += getRadioButton("useScrollback", "Off", "off", true);
  }
  response_message += getRadioGroupFooter();

  // Scroll steps
  boolean scrollStepsDisabled = (configScrollback == 0);
  response_message += getNumberInput("Scroll steps:", "scrollSteps", 1, 40, configScrollSteps, scrollStepsDisabled);

  // fade
  response_message += getRadioGroupHeader("Fade effect:");
  if (configUseFade) {
    response_message += getRadioButton("useFade", "On", "on", true);
    response_message += getRadioButton("useFade", "Off", "off", false);
  } else {
    response_message += getRadioButton("useFade", "On", "on", false);
    response_message += getRadioButton("useFade", "Off", "off", true);
  }
  response_message += getRadioGroupFooter();

  // Fade steps
  boolean fadeStepsDisabled = (configUseFade == 0);
  response_message += getNumberInput("Fade steps:", "fadeSteps", 20, 200, configFadeSteps, fadeStepsDisabled);

  // Suppress ACP
  response_message += getRadioGroupHeader("Suppress ACP when dimmed:");
  if (configSuppressACP) {
    response_message += getRadioButton("suppressACP", "On", "on", true);
    response_message += getRadioButton("suppressACP", "Off", "off", false);
  } else {
    response_message += getRadioButton("suppressACP", "On", "on", false);
    response_message += getRadioButton("suppressACP", "Off", "off", true);
  }
  response_message += getRadioGroupFooter();
  //response_message += getCheckBox("suppressACP", "on", "Suppress ACP when fully dimmed", (configSuppressACP == 1));

  // LDR
  response_message += getRadioGroupHeader("Use LDR:");
  if (configUseLDR) {
    response_message += getRadioButton("useLDR", "On", "on", true);
    response_message += getRadioButton("useLDR", "Off", "off", false);
  } else {
    response_message += getRadioButton("useLDR", "On", "on", false);
    response_message += getRadioButton("useLDR", "Off", "off", true);
  }
  response_message += getRadioGroupFooter();
  //response_message += getCheckBox("useLDR", "on", "Use LDR for dimming", (useLDR == 1));

  // Date format
  response_message += getDropDownHeader("Date format:", "dateFormat", false);
  response_message += getDropDownOption("0", "YY-MM-DD", (configDateFormat == 0));
  response_message += getDropDownOption("1", "MM-DD-YY", (configDateFormat == 1));
  response_message += getDropDownOption("2", "DD-MM-YY", (configDateFormat == 2));
  response_message += getDropDownFooter();

  // Day blanking
  response_message += getDropDownHeader("Day blanking:", "dayBlanking", true);
  response_message += getDropDownOption("0", "Never blank", (configDayBlanking == 0));
  response_message += getDropDownOption("1", "Blank all day on weekends", (configDayBlanking == 1));
  response_message += getDropDownOption("2", "Blank all day on week days", (configDayBlanking == 2));
  response_message += getDropDownOption("3", "Blank always", (configDayBlanking == 3));
  response_message += getDropDownOption("4", "Blank during selected hours every day", (configDayBlanking == 4));
  response_message += getDropDownOption("5", "Blank during selected hours on week days and all day on weekends", (configDayBlanking == 5));
  response_message += getDropDownOption("6", "Blank during selected hours on weekends and all day on week days", (configDayBlanking == 6));
  response_message += getDropDownOption("7", "Blank during selected hours on weekends only", (configDayBlanking == 7));
  response_message += getDropDownOption("8", "Blank during selected hours on week days only", (configDayBlanking == 8));
  response_message += getDropDownFooter();

  // Blank Mode
  response_message += getDropDownHeader("Blank Mode:", "blankMode", true);
  response_message += getDropDownOption("0", "Blank tubes only", (configBlankMode == 0));
  response_message += getDropDownOption("1", "Blank LEDs only", (configBlankMode == 1));
  response_message += getDropDownOption("2", "Blank tubes and LEDs", (configBlankMode == 2));
  response_message += getDropDownFooter();
  
  boolean hoursDisabled = (configDayBlanking < 4);

  // Blank hours from
  response_message += getNumberInput("Blank from:", "blankFrom", 0, 23, configBlankFrom, hoursDisabled);

  // Blank hours to
  response_message += getNumberInput("Blank to:", "blankTo", 0, 23, configBlankTo, hoursDisabled);

  // Back light
  response_message += getDropDownHeader("Back light:", "backLight", true);
  response_message += getDropDownOption("0", "Fixed RGB backlight, no dimming", (configBacklightMode == 0));
  response_message += getDropDownOption("1", "Pulsing RGB backlight, no dimming", (configBacklightMode == 1));
  response_message += getDropDownOption("2", "Cycling RGB backlight, no dimming", (configBacklightMode == 2));
  response_message += getDropDownOption("3", "Fixed RGB backlight, dims with ambient light", (configBacklightMode == 3));
  response_message += getDropDownOption("4", "Pulsing RGB backlight, dims with ambient light", (configBacklightMode == 4));
  response_message += getDropDownOption("5", "Cycling RGB backlight, dims with ambient light", (configBacklightMode == 5));
  if ( i2cProtocol == v2v62 ) {
    response_message += getDropDownOption("6", "'Colourtime' backlight", (configBacklightMode == 6));
    response_message += getDropDownOption("7", "'Colourtime' backlight, dims with ambient light", (configBacklightMode == 7));
  }
  response_message += getDropDownFooter();

  boolean cycleEnabled = !((configBacklightMode == 2) || (configBacklightMode == 5));
  boolean colourChannelEnabled = !((configBacklightMode == 0) || (configBacklightMode == 1) || (configBacklightMode == 3) || (configBacklightMode == 4));

  // RGB channels
  response_message += getNumberInput("Red intensity:", "redCnl", 0, 15, configRedCnl, colourChannelEnabled);
  response_message += getNumberInput("Green intensity:", "grnCnl", 0, 15, configGreenCnl, colourChannelEnabled);
  response_message += getNumberInput("Blue intensity:", "bluCnl", 0, 15, configBlueCnl, colourChannelEnabled);

  // Cycle speed
  response_message += getNumberInput("Backlight Cycle Speed:", "cycleSpeed", 2, 64, configCycleSpeed, cycleEnabled);

  // Slots Mode
  response_message += getDropDownHeader("Date Slots:", "slotsMode", true);
  response_message += getDropDownOption("0", "Don't use slots mode", (configSlotsMode == 0));
  response_message += getDropDownOption("1", "Scroll In, Scramble Out", (configSlotsMode == 1));
  response_message += getDropDownFooter();

  if ( i2cProtocol == v2v62 ) {
    // PIR timeout
    response_message += getNumberInput("PIR timeout:", "pirTimeout", 60, 3600, configPirTimeout, false);
  }
  
  // Min Dim
  response_message += getNumberInput("Min Dim:", "minDim", 100, 500, configMinDim, false);

  // form footer
  response_message += getSubmitButton("Set");

  response_message += "</form></div>";

  // all done
  response_message += getHTMLFoot();

  server.send(200, "text/html", response_message);
}

// ===================================================================================================================
// ===================================================================================================================

/**
   Utility functions
*/
void utilityPageHandler()
{
  String response_message = getHTMLHead();
  response_message += getNavBar();

  response_message += "<hr><li><a href=\"/resetWiFi\">Restart WiFi module</a></li>";
  response_message += "<hr><li><a href=\"/reset\">Clear WiFi config and restart module</a></li>";
  if ( largeFlash ) {
  response_message += "<hr><li><a href=\"/update\">Update firmware</a></li>";
  }

  response_message += getHTMLFoot();
  server.send(200, "text/html", response_message);
}



/**
   Send a value to the clock for display
*/
void setDisplayValuePageHandler() {
  String resultMessage;
  byte timeValue = 0x05;
  if (server.hasArg("time"))
  {
    debugMsg("Got time");
    debugMsg(server.arg("time"));
    unsigned int timeToSend = atoi(server.arg("time").c_str());
    if (timeToSend > 255) 
    {
      timeValue = 0xff;
    } else {
      timeValue = timeToSend;
    }
  };

  if (server.hasArg("value"))
  {
    debugMsg("Got server arg value: " + server.arg("value"));
    long valueToSend = atol(server.arg("value").c_str());
    byte val2 = hex2bcd(valueToSend % 100);
    valueToSend = valueToSend / 100;
    byte val1 = hex2bcd(valueToSend % 100);
    valueToSend = valueToSend / 100;
    byte val0 = hex2bcd(valueToSend % 100);

    if (sendValueToI2C(val0,val1,val2,timeValue)) {
      resultMessage = "Value sent!";
    } else {
      resultMessage = "Error sending value!";
    }
  } else {
    resultMessage = "No value to send";
  }

  if (server.hasArg("format"))
  {
    debugMsg("Got format");
    debugMsg(server.arg("format"));

    long formatToSend = atol(server.arg("format").c_str());
    byte val2 = hex2bcd(formatToSend % 100);
    formatToSend = formatToSend / 100;
    byte val1 = hex2bcd(formatToSend % 100);
    formatToSend = formatToSend / 100;
    byte val0 = hex2bcd(formatToSend % 100);
    sendValueFormatToI2C(val0,val1,val2);
  };

  String response_message = getHTMLHead();
  response_message += getNavBar();

  response_message += "<div class=\"container\" role=\"main\"><h3 class=\"sub-header\">Send value to I2C</h3>";
  response_message += "<div class=\"alert alert-success fade in\"><strong>";
  response_message += resultMessage;
  response_message += "</strong></div></div>";

  response_message += getHTMLFoot();

  server.send(200, "text/html", response_message);
}

unsigned char hex2bcd (unsigned char x)
{
    unsigned char y;
    y = (x / 10) << 4;
    y = y | (x % 10);
    return (y);
}

// ===================================================================================================================
// ===================================================================================================================

/* Called if requested page is not found */
void handleNotFound()
{
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";

  for (uint8_t i = 0; i < server.args(); i++)
  {
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }

  server.send(404, "text/plain", message);
}

// ===================================================================================================================
// ===================================================================================================================

/* Called if we need to have a local CSS */
void localCSSHandler()
{
  String message = ".navbar,.table{margin-bottom:20px}.nav>li,.nav>li>a,article,aside,details,figcaption,figure,footer,header,hgroup,main,menu,nav,section,summary{display:block}.btn,.form-control,.navbar-toggle{background-image:none}.table,label{max-width:100%}.sub-header{padding-bottom:10px;border-bottom:1px solid #eee}.h3,h3{font-size:24px}.table{width:100%}table{background-color:transparent;border-spacing:0;border-collapse:collapse}.table-striped>tbody>tr:nth-of-type(2n+1){background-color:#f9f9f9}.table>caption+thead>tr:first-child>td,.table>caption+thead>tr:first-child>th,.table>colgroup+thead>tr:first-child>td,.table>colgroup+thead>tr:first-child>th,.table>thead:first-child>tr:first-child>td,.table>thead:first-child>tr:first-child>th{border-top:0}.table>thead>tr>th{border-bottom:2px solid #ddd}.table>tbody>tr>td,.table>tbody>tr>th,.table>tfoot>tr>td,.table>tfoot>tr>th,.table>thead>tr>td,.table>thead>tr>th{padding:8px;line-height:1.42857143;vertical-align:top;border-top:1px solid #ddd}th{text-align:left}td,th{padding:0}.navbar>.container .navbar-brand,.navbar>.container-fluid .navbar-brand{margin-left:-15px}.container,.container-fluid{padding-right:15px;padding-left:15px;margin-right:auto;margin-left:auto}.navbar-inverse .navbar-brand{color:#9d9d9d}.navbar-brand{float:left;height:50px;padding:15px;font-size:18px;line-height:20px}a{color:#337ab7;text-decoration:none;background-color:transparent}.navbar-fixed-top{border:0;top:0;border-width:0 0 1px}.navbar-inverse{background-color:#222;border-color:#080808}.navbar-fixed-bottom,.navbar-fixed-top{border-radius:0;position:fixed;right:0;left:0;z-index:1030}.nav>li,.nav>li>a,.navbar,.navbar-toggle{position:relative}.navbar{border-radius:4px;min-height:50px;border:1px solid transparent}.container{width:750px}.navbar-right{float:right!important;margin-right:-15px}.navbar-nav{float:left;margin:7.5px -15px}.nav{padding-left:0;margin-bottom:0;list-style:none}.navbar-nav>li{float:left}.navbar-inverse .navbar-nav>li>a{color:#9d9d9d}.navbar-nav>li>a{padding-top:10px;padding-bottom:10px;line-height:20px}.nav>li>a{padding:10px 15px}.navbar-inverse .navbar-toggle{border-color:#333}.navbar-toggle{display:none;float:right;padding:9px 10px;margin-top:8px;margin-right:15px;margin-bottom:8px;background-color:transparent;border:1px solid transparent;border-radius:4px}button,select{text-transform:none}button{overflow:visible}button,html input[type=button],input[type=reset],input[type=submit]{-webkit-appearance:button;cursor:pointer}.btn-primary{color:#fff;background-color:#337ab7;border-color:#2e6da4}.btn{display:inline-block;padding:6px 12px;margin-bottom:0;font-size:14px;font-weight:400;line-height:1.42857143;text-align:center;white-space:nowrap;vertical-align:middle;-ms-touch-action:manipulation;touch-action:manipulation;cursor:pointer;-webkit-user-select:none;-moz-user-select:none;-ms-user-select:none;user-select:none;border:1px solid transparent;border-radius:4px}button,input,select,textarea{font-family:inherit;font-size:inherit;line-height:inherit}input{line-height:normal}button,input,optgroup,select,textarea{margin:0;font:inherit;color:inherit}.form-control,body{font-size:14px;line-height:1.42857143}.form-horizontal .form-group{margin-right:-15px;margin-left:-15px}.form-group{margin-bottom:15px}.form-horizontal .control-label{padding-top:7px;margin-bottom:0;text-align:right}.form-control{display:block;width:100%;height:34px;padding:6px 12px;color:#555;background-color:#fff;border:1px solid #ccc;border-radius:4px;-webkit-box-shadow:inset 0 1px 1px rgba(0,0,0,.075);box-shadow:inset 0 1px 1px rgba(0,0,0,.075);-webkit-transition:border-color ease-in-out .15s,-webkit-box-shadow ease-in-out .15s;-o-transition:border-color ease-in-out .15s,box-shadow ease-in-out .15s;transition:border-color ease-in-out .15s,box-shadow ease-in-out .15s}.col-xs-8{width:66.66666667%}.col-xs-3{width:25%}.col-xs-1,.col-xs-10,.col-xs-11,.col-xs-12,.col-xs-2,.col-xs-3,.col-xs-4,.col-xs-5,.col-xs-6,.col-xs-7,.col-xs-8,.col-xs-9{float:left}.col-lg-1,.col-lg-10,.col-lg-11,.col-lg-12,.col-lg-2,.col-lg-3,.col-lg-4,.col-lg-5,.col-lg-6,.col-lg-7,.col-lg-8,.col-lg-9,.col-md-1,.col-md-10,.col-md-11,.col-md-12,.col-md-2,.col-md-3,.col-md-4,.col-md-5,.col-md-6,.col-md-7,.col-md-8,.col-md-9,.col-sm-1,.col-sm-10,.col-sm-11,.col-sm-12,.col-sm-2,.col-sm-3,.col-sm-4,.col-sm-5,.col-sm-6,.col-sm-7,.col-sm-8,.col-sm-9,.col-xs-1,.col-xs-10,.col-xs-11,.col-xs-12,.col-xs-2,.col-xs-3,.col-xs-4,.col-xs-5,.col-xs-6,.col-xs-7,.col-xs-8,.col-xs-9{position:relative;min-height:1px;padding-right:15px;padding-left:15px}label{display:inline-block;margin-bottom:5px;font-weight:700}*{-webkit-box-sizing:border-box;-moz-box-sizing:border-box;box-sizing:border-box}body{font-family:\"Helvetica Neue\",Helvetica,Arial,sans-serif;color:#333}html{font-size:10px;font-family:sans-serif;-webkit-text-size-adjust:100%}";

  server.send(200, "text/css", message);
}

// ----------------------------------------------------------------------------------------------------
// ----------------------------------------- Network handling -----------------------------------------
// ----------------------------------------------------------------------------------------------------

/**
   Get the local time from the time zone server. Return the error description prefixed by "ERROR:" if something went wrong.
   Uses the global variable timeServerURL.
*/
String getTimeFromTimeZoneServer() {
  String payload = "";
  timeSource = none;
  if ( wlanConnected && timeStatus() == timeSet && ntpServer != "none" ) {   // try NTP first
    while ( ms() > 5 ) {                               // exit when we're within 25 ms of actual time
      yield();
    }
    payload = myTz.dateTime(TIME_NOW,"Y,n,j,G,i,s");     // format is 2019,2,1,10,16,46 
    timeSource = NTP;
  } else {                                              // otherwise, use the http time service
    HTTPClient http;
    http.begin(timeServerURL);
    String espId = "";espId += ESP.getChipId();
    http.addHeader("ESP",espId);
    http.addHeader("ClientID",serialNumber);

    int httpCode = http.GET();

    // file found at server
    if (httpCode == HTTP_CODE_OK) {
      payload = http.getString();
      timeSource = TimeServer;
    } else {
      debugMsg("[HTTP] GET... failed, error: " + http.errorToString(httpCode));
      if (httpCode > 0) {
        // RFC error codes don't have a string mapping
        payload = "ERROR: " + String(httpCode);
      } else {
        // ESP error codes have a string mapping
        payload = "ERROR: " + String(httpCode) + " ("+ http.errorToString(httpCode) + ")";
      }
    }
  http.end();
  }
  return payload;
}


// ----------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------
// ----------------------------------------- Utility functions ----------------------------------------
// ----------------------------------------------------------------------------------------------------

void setBlueLED(boolean newState) {
#ifndef DEBUG
  if (newState) {
    digitalWrite(LED_BUILTIN, LOW);
  } else {
    digitalWrite(LED_BUILTIN, HIGH);
  }
#endif
}

/**
 * Split a string based on a separator, get the element given by index
 */
String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length() - 1;
  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

/**
   Split a string based on a separator, get the element given by index, return an integer value
*/
int getIntValue(String data, char separator, int index) {
  String result = getValue(data, separator, index);
  return atoi(result.c_str());
}

void debugMsg(String msg) {
  #ifdef DEBUG_ON
  Serial.println(msg);
  delay(10); //will this let string get sent?
  #endif
}

void debugMsgContinue(String msg) {
  #ifdef DEBUG_ON
  Serial.print(msg);
  #endif
}

void setupDebug() {
#ifdef DEBUG_ON
  Serial.begin(115200);
  debugMsg("Starting debug session");
  setDebug(EZT_DBG_LEVEL, Serial); // ezTime
#else
  setDebug(NONE); // ezTime
#endif
}

String formatIPAsString(IPAddress ip) {
  return String(ip[0]) + '.' + String(ip[1]) + '.' + String(ip[2]) + '.' + String(ip[3]);
}

boolean setOTAUpdates() {
  if ( ESP.getSketchSize() + 4096 < ESP.getFreeSketchSpace() ) { largeFlash = true; }
  
  if ( largeFlash ) {
    httpUpdater.setup(&server, update_path, update_username, update_password);
    return true;
  }

  return false;
}

// ----------------------------------------------------------------------------------------------------
// ------------------------------------------- I2C functions ------------------------------------------
// ----------------------------------------------------------------------------------------------------

/**
 * Send the time to the I2C slave. If the transmission went OK, return true, otherwise false.
 */
boolean sendTimeToI2C(String timeString) {

  int year = getIntValue(timeString, ',', 0);
  byte month = getIntValue(timeString, ',', 1);
  byte day = getIntValue(timeString, ',', 2);
  byte hour = getIntValue(timeString, ',', 3);
  byte minute = getIntValue(timeString, ',', 4);
  byte sec = getIntValue(timeString, ',', 5);

  byte yearAdjusted = (year - 2000);

  debugMsg("Sending time to I2C: " + timeString);

  Wire.beginTransmission(preferredI2CSlaveAddress);
  Wire.write(I2C_TIME_UPDATE); // Command
  Wire.write(yearAdjusted);
  Wire.write(month);
  Wire.write(day);
  Wire.write(hour);
  Wire.write(minute);
  Wire.write(sec);
  int error = Wire.endTransmission();
  return (error == 0);
}

/**
   Get the options from the I2C slave. If the transmission went OK, return true, otherwise false.
*/
boolean getClockOptionsFromI2C() {
  byte receivedByte = 0;
  uint8_t available = Wire.requestFrom((uint8_t)preferredI2CSlaveAddress, (uint8_t)1);  // just ask for i2c version byte

  debugMsg("I2C <-- Received bytes (expecting 1): " + available);
  if (available == 1 ) {
    receivedByte = Wire.read();                                           // get the protocol number
    debugMsg("I2C <-- Got protocol header: " + receivedByte);
    if (receivedByte != I2C_PROTOCOL_NUMBER_V1 && receivedByte != I2C_PROTOCOL_NUMBER_V2 ) {
      debugMsg("I2C Protocol ERROR! Expected header " + String(I2C_PROTOCOL_NUMBER_V1) + " or " + String(I2C_PROTOCOL_NUMBER_V2) + ", but got: " + receivedByte);
      return false;
    } else if ( receivedByte == I2C_PROTOCOL_NUMBER_V1 ) {
      i2cProtocol = v1v54;
    } else if ( receivedByte == I2C_PROTOCOL_NUMBER_V2 ) {
      i2cProtocol = v2v62;
    } else {
      return false;     // didn't get either protocol number
    }
  } else {
    return false;       // didn't get 1 byte as expected
  }
    // now get the data
    byte dataLen = 0;
    if ( i2cProtocol == v1v54 ) { dataLen = I2C_DATA_SIZE_V1; }
    if ( i2cProtocol == v2v62 ) { dataLen = I2C_DATA_SIZE_V2; }
    available = Wire.requestFrom((uint8_t)preferredI2CSlaveAddress, (uint8_t)dataLen ); 
    debugMsg("I2C <-- Received bytes (expecting " + String(dataLen) +  "): " + available);
    if (available == dataLen ) {
      receivedByte = Wire.read();                                              // throw away i2c version, we already know it

      receivedByte = Wire.read();
      debugMsg("I2C <-- Got hour mode: " + receivedByte);
      configHourMode = receivedByte;

      receivedByte = Wire.read();
      debugMsg("I2C <-- Got blank lead: " + receivedByte);
      configBlankLead = receivedByte;

      receivedByte = Wire.read();
      debugMsg("I2C <-- Got scrollback: " + receivedByte);
      configScrollback = receivedByte;

      receivedByte = Wire.read();
      debugMsg("I2C <-- Got suppress ACP: " + receivedByte);
      configSuppressACP = receivedByte;

      receivedByte = Wire.read();
      debugMsg("I2C <-- Got useFade: " + receivedByte);
      configUseFade = receivedByte;

      receivedByte = Wire.read();
      debugMsg("I2C <-- Got date format: " + receivedByte);
      configDateFormat = receivedByte;

      receivedByte = Wire.read();
      debugMsg("I2C <-- Got day blanking: " + receivedByte);
      configDayBlanking = receivedByte;

      receivedByte = Wire.read();
      debugMsg("I2C <-- Got blank hour start: " + receivedByte);
      configBlankFrom = receivedByte;

      receivedByte = Wire.read();
      debugMsg("I2C <-- Got blank hour end: " + receivedByte);
      configBlankTo = receivedByte;

      receivedByte = Wire.read();
      debugMsg("I2C <-- Got fade steps: " + receivedByte);
      configFadeSteps = receivedByte;

      receivedByte = Wire.read();
      debugMsg("I2C <-- Got scroll steps: " + receivedByte);
      configScrollSteps = receivedByte;

      receivedByte = Wire.read();
      debugMsg("I2C <-- Got backlight mode: " + receivedByte);
      configBacklightMode = receivedByte;

      receivedByte = Wire.read();
      debugMsg("I2C <-- Got red channel: " + receivedByte);
      configRedCnl = receivedByte;

      receivedByte = Wire.read();
      debugMsg("I2C <-- Got green channel: " + receivedByte);
      configGreenCnl = receivedByte;

      receivedByte = Wire.read();
      debugMsg("I2C <-- Got blue channel: " + receivedByte);
      configBlueCnl = receivedByte;

      receivedByte = Wire.read();
      debugMsg("I2C <-- Got cycle speed: " + receivedByte);
      configCycleSpeed = receivedByte;

      receivedByte = Wire.read();
      debugMsg("I2C <-- Got useLDR: " + receivedByte);
      configUseLDR = receivedByte;

      receivedByte = Wire.read();
      debugMsg("I2C <-- Got blankMode: " + receivedByte);
      configBlankMode = receivedByte;

      receivedByte = Wire.read();
      debugMsg("I2C <-- Got slotsMode: " + receivedByte);
      configSlotsMode = receivedByte;
  
      if ( i2cProtocol == v2v62 ) {                                  // only for V2
        receivedByte = Wire.read();
        debugMsg("I2C <-- Got pirTimeOut Hi: " + receivedByte);
        int pirTO = receivedByte * 256;

        receivedByte = Wire.read();
        debugMsg("I2C <-- Got pirTimeOut Lo: " + receivedByte);
        pirTO += receivedByte;
        debugMsg("I2C <-- Got pirTimeOut combined: " + pirTO);
        configPirTimeout = pirTO;
      }

      receivedByte = Wire.read();
      debugMsg("I2C <-- Got minDim Hi: " + receivedByte);
      int minDimNew = receivedByte * 256;

      receivedByte = Wire.read();
      debugMsg("I2C <-- Got minDim Lo: " + receivedByte);
      minDimNew += receivedByte;
      debugMsg("I2C <-- Got minDim combined: " + minDimNew);
      configMinDim = minDimNew;
    } else {
      // didn't get the right number of bytes
      debugMsg("I2C <-- Got wrong number of bytes, expected " + String(I2C_DATA_SIZE_V1) + " or " + String(I2C_DATA_SIZE_V2) + ", got: " + available);
    }

  int error = Wire.endTransmission();
  return (error == 0);
}

boolean sendIPAddressToI2C(IPAddress ip) {  
  debugMsg("Sending IP Address to I2C: " + formatIPAsString(ip));

  Wire.beginTransmission(preferredI2CSlaveAddress);
  Wire.write(I2C_SHOW_IP_ADDR); // Command
  Wire.write(ip[0]);
  Wire.write(ip[1]);
  Wire.write(ip[2]);
  Wire.write(ip[3]);
  
  int error = Wire.endTransmission();
  return (error == 0);
}

boolean sendValueToI2C(byte digit01, byte digit23, byte digit45, byte timeValue) {
  debugMsg("Sending Arbitrary value to I2C: " + String(digit01) + "," + String(digit23) + "," + String(digit45));

  Wire.beginTransmission(preferredI2CSlaveAddress);
  Wire.write(I2C_SHOW_VALUE); // Command
  Wire.write(digit01);
  Wire.write(digit23);
  Wire.write(digit45);
  Wire.write(timeValue);
  
  int error = Wire.endTransmission();
  return (error == 0);
}

boolean sendValueFormatToI2C(byte digit01, byte digit23, byte digit45) {  
  debugMsg("Sending Arbitrary value format to I2C: " + String(digit01) + "," + String(digit23) + "," + String(digit45));

  Wire.beginTransmission(preferredI2CSlaveAddress);
  Wire.write(I2C_SHOW_VALUE_FORMAT); // Command
  Wire.write(digit01);
  Wire.write(digit23);
  Wire.write(digit45);
  
  int error = Wire.endTransmission();
  return (error == 0);
}

boolean setClockOption12H24H(boolean newMode) {
  return setClockOptionBoolean(I2C_SET_OPTION_12_24, newMode);
}

boolean setClockOptionBlankLeadingZero(boolean newMode) {
  return setClockOptionBoolean(I2C_SET_OPTION_BLANK_LEAD, newMode);
}

boolean setClockOptionScrollback(boolean newMode) {
  return setClockOptionBoolean(I2C_SET_OPTION_SCROLLBACK, newMode);
}

boolean setClockOptionSuppressACP(boolean newMode) {
  return setClockOptionBoolean(I2C_SET_OPTION_SUPPRESS_ACP, newMode);
}

boolean setClockOptionUseFade(boolean newMode) {
  return setClockOptionBoolean(I2C_SET_OPTION_FADE, newMode);
}

boolean setClockOptionUseLDR(boolean newMode) {
  return setClockOptionBoolean(I2C_SET_OPTION_USE_LDR, newMode);
}

boolean setClockOptionDateFormat(byte newMode) {
  return setClockOptionByte(I2C_SET_OPTION_DATE_FORMAT, newMode);
}

boolean setClockOptionDayBlanking(byte newMode) {
  return setClockOptionByte(I2C_SET_OPTION_DAY_BLANKING, newMode);
}

boolean setClockOptionBlankFrom(byte newMode) {
  return setClockOptionByte(I2C_SET_OPTION_BLANK_START, newMode);
}

boolean setClockOptionBlankTo(byte newMode) {
  return setClockOptionByte(I2C_SET_OPTION_BLANK_END, newMode);
}

boolean setClockOptionFadeSteps(byte newMode) {
  return setClockOptionByte(I2C_SET_OPTION_FADE_STEPS, newMode);
}

boolean setClockOptionScrollSteps(byte newMode) {
  return setClockOptionByte(I2C_SET_OPTION_SCROLL_STEPS, newMode);
}

boolean setClockOptionBacklightMode(byte newMode) {
  return setClockOptionByte(I2C_SET_OPTION_BACKLIGHT_MODE, newMode);
}

boolean setClockOptionRedCnl(byte newMode) {
  return setClockOptionByte(I2C_SET_OPTION_RED_CHANNEL, newMode);
}

boolean setClockOptionGrnCnl(byte newMode) {
  return setClockOptionByte(I2C_SET_OPTION_GREEN_CHANNEL, newMode);
}

boolean setClockOptionBluCnl(byte newMode) {
  return setClockOptionByte(I2C_SET_OPTION_BLUE_CHANNEL, newMode);
}

boolean setClockOptionCycleSpeed(byte newMode) {
  return setClockOptionByte(I2C_SET_OPTION_CYCLE_SPEED, newMode);
}

boolean setClockOptionBlankMode(byte newMode) {
  return setClockOptionByte(I2C_SET_OPTION_BLANK_MODE, newMode);
}

boolean setClockOptionSlotsMode(byte newMode) {
  return setClockOptionByte(I2C_SET_OPTION_SLOTS_MODE, newMode);
}

boolean setClockOptionPirTimeout(int newMode) {
  return setClockOptionInt(I2C_SET_OPTION_PIR_TIMEOUT, newMode);
}


boolean setClockOptionMinDim(int newMode) {
  if ( i2cProtocol == v2v62 ) {
    return setClockOptionInt(I2C_SET_OPTION_MIN_DIM_V2, newMode);
  } else if ( i2cProtocol == v1v54 ) {
    return setClockOptionInt(I2C_SET_OPTION_MIN_DIM_V1, newMode);
  }  
}

/**
   Send the options from the I2C slave. If the transmission went OK, return true, otherwise false.
*/
boolean setClockOptionBoolean(byte option, boolean newMode) {
  debugMsg("I2C --> setting boolean option: " + String(option) + " with value: " + String(newMode));

  Wire.beginTransmission(preferredI2CSlaveAddress);
  Wire.write(option);
  byte newOption;
  if (newMode) {
    newOption = 0;
  } else {
    newOption = 1;
  }
  Wire.write(newOption);
  int error = Wire.endTransmission();
  delay(10);
  return (error == 0);
}

/**
   Send the options from the I2C slave. If the transmission went OK, return true, otherwise false.
*/
boolean setClockOptionByte(byte option, byte newMode) {
  debugMsg("I2C --> setting byte option: " + String(option) + " with value: " + String(newMode));

  Wire.beginTransmission(preferredI2CSlaveAddress);
  Wire.write(option);
  Wire.write(newMode);
  int error = Wire.endTransmission();
  delay(10);
  return (error == 0);
}

/**
   Send the options from the I2C slave. If the transmission went OK, return true, otherwise false.
*/
boolean setClockOptionInt(byte option, int newMode) {
  byte loByte = newMode % 256;
  byte hiByte = newMode / 256; 
  debugMsg("I2C --> setting int option: " + String(option) + " with value: " + String(newMode));

  Wire.beginTransmission(preferredI2CSlaveAddress);
  Wire.write(option);
  Wire.write(hiByte);
  Wire.write(loByte);
  int error = Wire.endTransmission();
  delay(10);
  return (error == 0);
}

boolean scanI2CBus() {
  debugMsg("Scanning I2C bus");
  byte pingAnsweredFrom = 0xff;
  for (int idx = 0 ; idx < 128 ; idx++)
  {
    Wire.beginTransmission(idx);
    int error = Wire.endTransmission();
    if (error == 0) {
      debugMsg("Received a response from " + String(idx));
      preferredI2CSlaveAddress = idx;
      preferredAddressFoundBy = 1;
      if (getClockOptionsFromI2C()) {
        debugMsg("Received a ping answer from " + String(idx));
        pingAnsweredFrom = idx;
      }
    }
  }

  // if we got a ping answer, then we must use that
  if (pingAnsweredFrom != 0xff) {
    preferredI2CSlaveAddress = pingAnsweredFrom;
    preferredAddressFoundBy = 2;
  }
  debugMsg("Scanning I2C bus done");
}

// ----------------------------------------------------------------------------------------------------
// ------------------------------------------- HTML functions -----------------------------------------
// ----------------------------------------------------------------------------------------------------

String getHTMLHead() {
  String header = "<!DOCTYPE html><html><head>";

  if (WiFi.status() == WL_CONNECTED) {
    header += "<link href=\"https://maxcdn.bootstrapcdn.com/bootstrap/3.3.6/css/bootstrap.min.css\" rel=\"stylesheet\" integrity=\"sha384-1q8mTJOASx8j1Au+a5WDVnPi2lkFfwwEAa8hDDdjZlpLegxhjVME1fgjWPGmkzs7\" crossorigin=\"anonymous\">";
    header += "<link href=\"http://www.open-rate.com/wl.css\" rel=\"stylesheet\" type=\"text/css\">";
    header += "<script src=\"http://code.jquery.com/jquery-1.12.3.min.js\" integrity=\"sha256-aaODHAgvwQW1bFOGXMeX+pC4PZIPsvn2h1sArYOhgXQ=\" crossorigin=\"anonymous\"></script>";
    header += "<script src=\"https://maxcdn.bootstrapcdn.com/bootstrap/3.3.6/js/bootstrap.min.js\" integrity=\"sha384-0mSbJDEHialfmuBBQP6A4Qrprq5OVfW37PRR3j5ELqxss1yVqOtnepnHVP9aJ7xS\" crossorigin=\"anonymous\"></script>";
  } else {
    header += "<link href=\"local.css\" rel=\"stylesheet\">";
  }
  header += "<title>Arduino Nixie Clock Time Module</title></head>";
  header += "<body>";
  return header;
}

/**
   Get the bootstrap top row navbar, including the Bootstrap links
*/
String getNavBar() {
  String navbar = "<nav class=\"navbar navbar-inverse navbar-fixed-top\">";
  navbar += "<div class=\"container-fluid\"><div class=\"navbar-header\"><button type=\"button\" class=\"navbar-toggle collapsed\" data-toggle=\"collapse\" data-target=\"#navbar\" aria-expanded=\"false\" aria-controls=\"navbar\">";
  navbar += "<span class=\"sr-only\">Toggle navigation</span><span class=\"icon-bar\"></span><span class=\"icon-bar\"></span><span class=\"icon-bar\"></span></button>";
  navbar += "<a class=\"navbar-brand\" href=\"https://www.nixieclock.biz/Store.html\">Arduino Nixie Clock Time Module</a></div><div id=\"navbar\" class=\"navbar-collapse collapse\"><ul class=\"nav navbar-nav navbar-right\">";
  navbar += "<li><a href=\"/\">Status</a></li>  ";
  navbar += "<li><a href=\"/clockconfig\">Clock</a></li>";
  navbar += "<li><a href=\"/time\">Time servers</a></li>  ";
//  navbar += "<li><a href=\"/wlan_config\">WLAN</a></li>";
  navbar += "<li><a href=\"/utility\">Utility</a></li>";
  navbar += "</ul></div></div></nav>";
  return navbar;
} 

/**
   Get the header for a 2 column table
*/
String getTableHead2Col(String tableHeader, String col1Header, String col2Header) {
  String tableHead = "<div class=\"container\" role=\"main\"><h3 class=\"sub-header\">";
  tableHead += tableHeader;
  tableHead += "</h3><div class=\"table-responsive\"><table class=\"table table-striped\"><thead><tr><th>";
  tableHead += col1Header;
  tableHead += "</th><th>";
  tableHead += col2Header;
  tableHead += "</th></tr></thead><tbody>";

  return tableHead;
}

String getTableRow2Col(String col1Val, String col2Val) {
  String tableRow = "<tr><td>";
  tableRow += col1Val;
  tableRow += "</td><td>";
  tableRow += col2Val;
  tableRow += "</td></tr>";

  return tableRow;
}

String getTableRow2Col(String col1Val, int col2Val) {
  String tableRow = "<tr><td>";
  tableRow += col1Val;
  tableRow += "</td><td>";
  tableRow += col2Val;
  tableRow += "</td></tr>";

  return tableRow;
}

String getTableFoot() {
  return "</tbody></table></div></div>";
}

/**
   Get the header for an input form
*/
String getFormHead(String formTitle) {
  String tableHead = "<div class=\"container\" role=\"main\"><h3 class=\"sub-header\">";
  tableHead += formTitle;
  tableHead += "</h3><form class=\"form-horizontal\">";

  return tableHead;
}

/**
   Get the header for an input form
*/
String getFormFoot() {
  return "</form></div>";
}

String getHTMLFoot() {
  return "</body></html>";
}

String getRadioGroupHeader(String header) {
  String result = "<div class=\"form-group\"><label class=\"control-label col-xs-3\">";
  result += header;
  result += "</label>";
  return result;
}

String getRadioButton(String group_name, String text, String value, boolean checked) {
  String result = "<div class=\"col-xs-1\">";
  if (checked) {
    result += "<label class=\"radio-inline\"><input checked type=\"radio\" name=\"";
  } else {
    result += "<label class=\"radio-inline\"><input type=\"radio\" name=\"";
  }
  result += group_name;
  result += "\" value=\"";
  result += value;
  result += "\"> ";
  result += text;
  result += "</label></div>";
  return result;
}

String getRadioGroupFooter() {
  String result = "</div>";
  return result;
}

String getCheckBox(String checkbox_name, String value, String text, boolean checked) {
  String result = "<div class=\"form-group\"><div class=\"col-xs-offset-3 col-xs-9\"><label class=\"checkbox-inline\">";
  if (checked) {
    result += "<input checked type=\"checkbox\" name=\"";
  } else {
    result += "<input type=\"checkbox\" name=\"";
  }

  result += checkbox_name;
  result += "\" value=\"";
  result += value;
  result += "\"> ";
  result += text;
  result += "</label></div></div>";

  return result;
}

String getDropDownHeader(String heading, String group_name, boolean wide) {
  String result = "<div class=\"form-group\"><label class=\"control-label col-xs-3\">";
  result += heading;
  if (wide) {
    result += "</label><div class=\"col-xs-8\"><select class=\"form-control\" name=\"";
  } else {
    result += "</label><div class=\"col-xs-2\"><select class=\"form-control\" name=\"";
  }
  result += group_name;
  result += "\">";
  return result;
}

String getDropDownOption (String value, String text, boolean checked) {
  String result = "";
  if (checked) {
    result += "<option selected value=\"";
  } else {
    result += "<option value=\"";
  }
  result += value;
  result += "\">";
  result += text;
  result += "</option>";
  return result;
}

String getDropDownFooter() {
  return "</select></div></div>";
}

String getNumberInput(String heading, String input_name, int minVal, int maxVal, int value, boolean disabled) {
  String result = "<div class=\"form-group\"><label class=\"control-label col-xs-3\" for=\"";
  result += input_name;
  result += "\">";
  result += heading;
  result += "</label><div class=\"col-xs-2\"><input type=\"number\" class=\"form-control\" name=\"";
  result += input_name;
  result += "\" id=\"";
  result += input_name;
  result += "\" min=\"";
  result += minVal;
  result += "\" max=\"";
  result += maxVal;
  result += "\" value=\"";
  result += value;
  result += "\"";
  if (disabled) {
    result += " disabled";
  }
  result += "></div></div>";

  return result;
}

String getNumberInputWide(String heading, String input_name, byte minVal, byte maxVal, byte value, boolean disabled) {
  String result = "<div class=\"form-group\"><label class=\"control-label col-xs-8\" for=\"";
  result += input_name;
  result += "\">";
  result += heading;
  result += "</label><div class=\"col-xs-2\"><input type=\"number\" class=\"form-control\" name=\"";
  result += input_name;
  result += "\" id=\"";
  result += input_name;
  result += "\" min=\"";
  result += minVal;
  result += "\" max=\"";
  result += maxVal;
  result += "\" value=\"";
  result += value;
  result += "\"";
  if (disabled) {
    result += " disabled";
  }
  result += "></div></div>";

  return result;
}

String getTextInput(String heading, String input_name, String value, boolean disabled) {
  String result = "<div class=\"form-group\"><label class=\"control-label col-xs-3\" for=\"";
  result += input_name;
  result += "\">";
  result += heading;
  result += "</label><div class=\"col-xs-2\"><input type=\"text\" class=\"form-control\" name=\"";
  result += input_name;
  result += "\" id=\"";
  result += input_name;
  result += "\" value=\"";
  result += value;
  result += "\"";
  if (disabled) {
    result += " disabled";
  }
  result += "></div></div>";

  return result;
}

String getTextInputWide(String heading, String input_name, String value, boolean disabled) {
  String result = "<div class=\"form-group\"><label class=\"control-label col-xs-3\" for=\"";
  result += input_name;
  result += "\">";
  result += heading;
  result += "</label><div class=\"col-xs-8\"><input type=\"text\" class=\"form-control\" name=\"";
  result += input_name;
  result += "\" id=\"";
  result += input_name;
  result += "\" value=\"";
  result += value;
  result += "\"";
  if (disabled) {
    result += " disabled";
  }
  result += "></div></div>";

  return result;
}

String getSubmitButton(String buttonText) {
  String result = "<div class=\"form-group\"><div class=\"col-xs-offset-3 col-xs-9\"><input type=\"submit\" class=\"btn btn-primary\" value=\"";
  result += buttonText;
  result += "\"></div></div>";
  return result;
}

// ----------------------------------------------------------------------------------------------------
// ------------------------------------------- JSON functions -----------------------------------------
// ----------------------------------------------------------------------------------------------------

/* Arduino IDE v1.8.10 ArduinoJson v6 == DynamicJsonBuffer -> DynamicJsonDocument
   changes tested and compiled on Wemos D1 Mini R2*/

bool getConfigfromSpiffs() {
  bool loaded = false;
  if (SPIFFS.begin()) { 
    debugMsg("mounted file system for read");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      debugMsg("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        debugMsg("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        
        /* BEGIN CHANGES */
        // DynamicJsonBuffer jsonBuffer;
        // JsonObject& json = jsonBuffer.parseObject(buf.get());
        // json.printTo(Serial);
        DynamicJsonDocument jsonDoc(2048); // <-- fixed space MUST be allocated - IS 2048 ENOUGH?
        auto error = deserializeJson(jsonDoc, buf.get()); // <-- new error handling
        serializeJson(jsonDoc, Serial); // <-- new call to serialize
        debugMsg("\n");
        
        if (!error) {
          //if (json.success()) {
          debugMsg("parsed json");

          timeServerURL = jsonDoc["time_zone"].as<String>();
          debugMsg("Loaded time server URL: " + timeServerURL);

          ntpServer = jsonDoc["ntp_server"].as<String>();
          debugMsg("Loaded NTP server: " + ntpServer);

          ntpTZ = jsonDoc["ntp_tz"].as<String>();
          debugMsg("Loaded NTP timezone: " + ntpTZ);
          myTz.setPosix(ntpTZ);

          ntpInterval = jsonDoc["ntp_interval"].as<int>();
          debugMsg("Loaded NTP interval: " + String(ntpInterval));

          loaded = true;
        } else {
          debugMsg("failed to load json config");
        }
        debugMsg("Closing config file");
        configFile.close();
      }
    }
  } else {
    debugMsg("failed to mount FS");
  }

  SPIFFS.end();
  return loaded;
}

void saveConfigToSpiffs() { // SAME CHANGES AS ABOVE FOR DynamicJsonDocument
  if (SPIFFS.begin()) {
    debugMsg("mounted file system for write");
    debugMsg("saving config");

    DynamicJsonDocument jsonDoc(2048); // Again, here with 2kb allocation
    //JsonObject& json = jsonDoc.createObject();
    jsonDoc["time_zone"]    = timeServerURL;
    jsonDoc["ntp_server"]   = ntpServer;
    jsonDoc["ntp_tz"]       = ntpTZ;
    jsonDoc["ntp_interval"] = ntpInterval;

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
      debugMsg("failed to open config file for writing");
      configFile.close();
      return;
    }

    serializeJson(jsonDoc, Serial);
    debugMsg("\n");
  
    serializeJson(jsonDoc, configFile);
    configFile.close();
    debugMsg("Saved config");
    //end save
  } else {
    debugMsg("failed to mount FS");
  }
  SPIFFS.end();
}

void resetConfigs() {
  timeServerURL = DEFAULT_TIME_SERVER_URL_1;
  ntpServer     = DEFAULT_NTP_SERVER;
  ntpTZ         = DEFAULT_NTP_TZ;
  ntpInterval   = DEFAULT_NTP_INTERVAL;
  saveConfigToSpiffs();
}
