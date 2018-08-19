#include <SPI.h>
#include <WiFi.h>
#include <LoRa.h>
#include<Arduino.h>
#include <TinyGPS++.h>    
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>                   
#define SS      18
#define RST     14
#define DI0     26
#define BAND    915E6  
//display
#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);
#define NUMFLAKES 10
#define XPOS 0
#define YPOS 1
#define DELTAY 2

struct Direction{
  int distanceFt;
  double distanceMiles;
  double heading;
  char ordinal[4];
};
struct Point {
  double lat, lng;
  char label[10];
};
struct Beacon {
  double lat, lng;
  char label[6];
  int rssi;
  int speed;
  Point brcPoint;
  Direction brcDirection;
  Direction locDirection;
};

//device constants
const char deviceName[ ] = "Beta4"; //5 chars ONLY
const int userRadioId = 4; //must be less than modValue
const int modValue = 5; //10 or more for prod,less transmi, more listen
//mode flags
const bool modeNoGpsSender = false; //will turn into a dumb transmitter
const bool modeNoGpsReciever = true;
const bool modeDiagnostic = false; 
const int gpsTimerTick = 333;
//globals
TinyGPSPlus gps;  
int sentPacketCount = 0;
bool gpsFix = false;
int screenIndex = 0; //0-N: overview, 100-1N:detail 
//Roster of beacons: tracks all the currently observed beacons
const int rosterSize = 20; //total size of list of recognized devices
int rosterEntries = 0; //current number of entries in roster
Beacon roster[rosterSize];
//view refresh rate
const long frameRate = 5000;
long frameLastTime = frameRate + 1; //expire the timer for a fresh frame

//Button clicking:

const int buttonPin = 2;     // the number of the pushbutton pin
int buttonState = 0;        
int previousState = 0; //off
// Button timing variables
int debounce = 20;          // ms debounce period to prevent flickering when pressing or releasing the button
int DCgap = 20;            // max ms between clicks for a double click event
int holdTime = 500;        // ms hold period: how long to wait for press+hold event
int longHoldTime = 2000;    // ms long hold period: how long to wait for press+hold event
// Button variables
boolean buttonVal = HIGH;   // value read from button
boolean buttonLast = HIGH;  // buffered value of the button's previous state
boolean DCwaiting = false;  // whether we're waiting for a double click (down)
boolean DConUp = false;     // whether to register a double click on next release, or whether to wait and click
boolean singleOK = true;    // whether it's OK to do a single click
long downTime = -1;         // time the button was pressed down
long upTime = -1;           // time the button was released
boolean ignoreUp = false;   // whether to ignore the button release because the click+hold was triggered
boolean waitForUp = false;        // when held, whether to wait for the up event
boolean holdEventPast = false;    // whether or not the hold event happened already
boolean longHoldEventPast = false;// whether or not the long hold event happened already
int printCounter = 0;

//functions
Point brcClosestPoint (Point myLocation);

void setup() {
  Serial.begin(115200);
  //Setup button
  pinMode(buttonPin, INPUT);
  //Setup GPS:
  Serial1.begin(9600, SERIAL_8N1, 12, 15);   //17-TX 18-RX for GPS
  //Disable radiozz`
  WiFi.mode(WIFI_OFF); 
  btStop();
  //Start LoRa
  SPI.begin(5,19,27,18);
  LoRa.setPins(SS,RST,DI0);
  if (!LoRa.begin(BAND)) { Serial.println("Starting LoRa failed!");  while (1); }
  LoRa.setTxPower(18); // 2-20, *17
  LoRa.setSpreadingFactor(7); //6-12, *7
  LoRa.setCodingRate4(5); //5-8, *5  
  LoRa.setSignalBandwidth(31.25E3);  //7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3, 41.7E3, 62.5E3, *125E3, 250E3.
  LoRa.crc(); //*off
  //setup display
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)
  display.clearDisplay();
  ui_textLarge("booting up...");
  
  Serial.println("");
  int chipid = ESP.getEfuseMac();
  Serial.printf("ESP32 Chip ID = %04X",(uint16_t)(chipid>>32));//print High 2 bytes
  Serial.printf("%04X\n",(uint32_t)chipid);//print Low 4bytes.    
  Serial.println("[bootup successful]");
  display.display();
  addTestBeacon();
}

void loop() {  
   //Serial.println("checking button");
   int b = checkButton();
   if (b == 1) clickEvent();
   if (b == 2) doubleClickEvent();
   if (b == 3) holdEvent();
   if (b == 4) longHoldEvent();
   
  if(modeNoGpsSender == true){
    //this enables transmit mode: transmit every 1 second, do nothing else
    ui_textLarge("Transmit");
    transmitLocation();
    wait(1000);                                      
    return;
  }
  //Startup:
  //disables for testing
  if((gps.satellites.value() <= 3)&&(modeNoGpsReciever == false)){
      Serial.println("Aquiring GPS fix...");
      ui_pendingGps();
      wait(2500);  
      return;  
  }
  
  //ui_textLarge("Listening");

  if((gpsFix == false)&&(modeNoGpsReciever == false)) { //shows on first run
    Serial.println("GPS position found!");
    gpsFix = true;
  }
  
  if((gps.time.second() % modValue == userRadioId) && (gpsFix == true)){
    //time to transmit, adding a small random delay to help ease congestion
    //int randy = random(0,300);
    //wait(randy);
    //Serial.print("Randy: ");
    //Serial.println(randy);
    transmitLocation();
  } else {
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
      // read the packet char by char
      int pointer = 0;
      char packet[50] = "";
      while (LoRa.available()) {
        packet[pointer] = (char)LoRa.read();
        pointer++;
        //Serial.print((char)LoRa.read());
      }
      handleIncomingPacket(packet, sizeof(packet), LoRa.packetRssi());
    }
  }


  //every 5 seconds (ie frame rate) update the display
  if(millis() < (frameLastTime+frameRate)){
        return;
  } 
  updateScreen();
}//End main loop function

void updateScreen(){
  Serial.print("{frame:");
  Serial.print(millis());
  Serial.println("}");
  
  //Serial.println(displayMode);
  if(screenIndex<100){
    ui_dislayOverview();
  } else if(screenIndex<200){
    ui_displayDetail();
  } else {
    Serial.println("ERROR screenIndex is a weirdly large value");
  }  
}

void handleIncomingPacket(char* passPacket, int passSize, int rssi) {
  Beacon beacon;
  char pckt[50];
  strncpy(pckt, passPacket, passSize);
  int dataCounter = 0; //tracks which data field
  int curCounter = 0; //tracks where in the current char
  int startPoint = 0; //track where we start reading current field from
  char curBuffer[50];
  char floatCharArray[13];
  char delimiter = ' ';
  //Serial.println(pckt);
  
  //loop through every char in the packet
  for (int i = 0; i < strlen(pckt); i++) {
   if (pckt[i] != delimiter) {
      //not a space, so add to charCurrent
      curBuffer[i-startPoint] = pckt[i];
      //Serial.print("curBuffer: ");
      //Serial.println(curBuffer);
    } else {
      //we hit a space, end of name, so save the name
      //Serial.println(" delimiter ");
      if(dataCounter == 0) {
        strncpy(beacon.label, curBuffer, 5);
      } else if(dataCounter == 1) {
        beacon.lat = atof(floatCharArray);
      } else if(dataCounter == 2) {
        strncpy(floatCharArray, curBuffer, 11);
        beacon.lng = atof(floatCharArray);
      } else if(dataCounter == 3) {
        strncpy(floatCharArray, curBuffer, 5);
        beacon.speed = atof(floatCharArray);
      } else if(dataCounter > 3){
         Serial.println("Warning: dataCounter too big, ignoring some stuff");
      }
      //nuke the array
      for( int i = 0; i < sizeof(curBuffer);  ++i )
        curBuffer[i] = (char)0;
        
      startPoint = i+1;
      ++dataCounter;
    }
  }
  updateRosterWith(beacon);
}

void updateRosterWith(Beacon pBeacon){
  Beacon beacon = pBeacon;
  int addAtRow = -1;
  for (uint8_t i=0; i< rosterSize; i++) {
    if(strcmp(roster[i].label,beacon.label) == 0 ) {
      Serial.print("Roster: Found beacon at Row ");
      Serial.println(i);
      addAtRow = i;
      break;
    }
  }
  
  if(addAtRow == -1){ //new user, find next empty row
    Serial.print("Roster: Adding a new beacon (label:");
    Serial.print(beacon.label);
    Serial.println(")");
    rosterEntries++;
    for (uint8_t q = 0; q < rosterSize; q++) {
    //loop through list to add
      if(roster[q].lat == 0){
        addAtRow = q;
        break;
      } 
    }
  }

  if(addAtRow == -1){
    Serial.println("ERROR: device list is full");
    Serial.print("Roster Entries: ");
    Serial.println(rosterEntries);
    return;
  }

  //beacon matches: is there new info here or is it close enough to previous value to ignore? 
  Direction beaconChange = getDirections(roster[addAtRow].lat,roster[addAtRow].lng,beacon.lat,beacon.lng);
  if(beaconChange.distanceFt < 30){
    //less than 30 feet from previous value? ignore
    return;
  }

  //we have a new beacon to add, so attach the closest BRC address
  beacon.brcPoint = brcClosestPoint(beacon.lat,beacon.lng);
  beacon.brcDirection = getDirections(beacon.lat,beacon.lng,beacon.brcPoint.lat,beacon.brcPoint.lng);
  if(modeNoGpsReciever == true){
     beacon.locDirection = getDirections(40.78110,-119.21143,beacon.lat,beacon.lng);
  } else {
    beacon.locDirection = getDirections(gps.location.lat(),gps.location.lng(),beacon.lat,beacon.lng);
  }
  //Serial.print("69 69 brc closest: ");
  //Serial.println(beacon.brcPoint.label);
  //Serial.print("direction: ");
  //Serial.println(beacon.brcDirection.heading);
  roster[addAtRow] = beacon;
  return;
}

Direction getDirections(double originLat, double originLng, double targetLat, double targetLng){

  
  double distanceMeters = gps.distanceBetween(originLat, originLng, targetLat, targetLng);
  double courseTo = gps.courseTo(originLat,originLng,targetLat, targetLng);
  const char* ordinal = gps.cardinal(courseTo);
  Serial.print("Distance(meters):");
  Serial.println(distanceMeters); 
  Direction dir;
  dir.distanceFt = (distanceMeters * 3.28084);
  dir.distanceMiles=(distanceMeters / 1609.3400);
  dir.heading = courseTo;
  strcpy(dir.ordinal,ordinal);
  return dir;
}

void transmitLocation(){
  if((gps.satellites.value() <= 3)&&(modeNoGpsSender==false)) {
      Serial.println("Lost GPS fix...");
      ui_textLarge("Lost GPS");
      gpsFix = false;
      wait(1000);
      return;    
  }

  if(modeNoGpsReciever==true){
     Serial.println("NoGpsReciever mode is enabled, skipping packet send");
    return;
  }
  // send packet
  sentPacketCount++;
  const double sendableLat = (gps.location.lat() - 39); 
  const double sendableLng = (gps.location.lng() - 118); 
  const int sendableSpeed = gps.speed.mph(); 

  Serial.print("Sending packet (");
  Serial.print(sendableLat);
  Serial.print(":");
  Serial.print(sendableLng);
  Serial.println(")");

  LoRa.beginPacket();
  LoRa.print("");
  LoRa.print(deviceName);
  LoRa.print(" ");
  LoRa.print(sendableLat, 5); //5th decimal = 1.1m
  LoRa.print(" ");
  LoRa.print(sendableLng, 5);
  LoRa.print(" ");
  LoRa.print(sendableSpeed);
  LoRa.print(" ");
  LoRa.print(sentPacketCount);
  LoRa.print(" "); //eases digestion easier
  LoRa.endPacket();
  
  //digitalWrite(25, HIGH);   // turn the LED on (HIGH is the voltage level)
  //delay(1000);                       // wait for a second
  //digitalWrite(25, LOW);    // turn the LED off by making the voltage LOW
  //delay(1000);                       // wait for a second
}

void addTestBeacon(){
  Beacon beacon;
  beacon.lat = 40.79990;
  beacon.lng = -119.21126;
  beacon.rssi = 69;
  beacon.speed = 30;
  strncpy(beacon.label, "Funky", 6);
  updateRosterWith(beacon);
  Beacon beacon2;
  beacon2.lat = 40.78640;
  beacon2.lng = -119.20650;
  beacon2.rssi = 69;
  beacon2.speed = 88;
  strncpy(beacon2.label, " MAN ", 6);
  updateRosterWith(beacon2);
  
  Beacon beacon3;
  beacon3.lat = 40.98640;
  beacon3.lng = -119.00650;
  beacon3.rssi = 69;
  beacon3.speed = 88;
  strncpy(beacon3.label, "BKN03", 6);
  updateRosterWith(beacon3);
  
  Beacon beacon4;
  beacon4.lat = 40.08640;
  beacon4.lng = -119.70650;
  beacon4.rssi = 69;
  beacon4.speed = 81;
  strncpy(beacon4.label, "BKN04", 6);
  updateRosterWith(beacon4);
}

static void wait(unsigned long ms)                
{   
  unsigned long start = millis();
  do
  {
    while (Serial1.available())
      gps.encode(Serial1.read());
  } while (millis() - start < ms);
}


void ui_textLarge(char* text){
  display.clearDisplay();
  display.setCursor(0,10);
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.println(text);
  display.display();
}

void ui_pendingGps(){
  //Serial.println("displaying gps state ");
  display.clearDisplay();
  display.setCursor(0,10);
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.println("GPS:");
  display.setTextSize(1);
  display.setCursor(48,8);
  display.print("Searching for");
  display.setCursor(48,18);
  display.print(" satellights");
  display.display();
}


void ui_displayDetail(){
  Serial.println("DisplayDetail");
  frameLastTime = millis();  //notes the last draw time 

  int displayIndex;
  if(screenIndex>99){
    displayIndex=(screenIndex-100);
  } else {
    displayIndex=screenIndex;
  }
  display.clearDisplay();
  display.setCursor(0,0);
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.print(" ");
  display.print(roster[displayIndex].label);
  display.println(" ");

  if(roster[displayIndex].locDirection.distanceFt<9999){
    display.print(roster[displayIndex].locDirection.distanceFt,1);
    display.setTextSize(1);
    display.print("ft ");
  } else {
    display.print(roster[displayIndex].locDirection.distanceMiles,2);
    display.setTextSize(1);
    display.print("mi ");   
  }
  display.setTextSize(2);
  display.println(roster[displayIndex].locDirection.ordinal);
  display.display();
}

void ui_dislayOverview(){
  //Serial.println("DisplayOver");
  frameLastTime = millis();  //notes the last draw time 
  int onscreenRows;

  if(rosterEntries > 4){
    onscreenRows = 4;
  } else if(rosterEntries < 0){
    ui_textLarge("No beacons");
  } else {
    onscreenRows = rosterEntries;
  }
  
  
  //Serial.println("Drawing overview on screen");
  //done with prep, move pixels:
  display.clearDisplay();
  display.setCursor(0,0);
  int start = 0;
  int end = start+4;
  for (uint8_t i=start; i< end; i++) {
    //print each rpw
     display.setTextSize(1);
    display.setTextColor(WHITE);
    display.print("");
    display.print(roster[i].label);
    display.print("");
    
    if(i==screenIndex){
      //this is the selected row so highlight
       display.setTextColor(BLACK, WHITE);
    } else {
      display.setTextColor(WHITE, BLACK); 
  }
    display.print(" ");
    display.print(roster[i].brcPoint.label);
    display.println("");
  }
  display.display();

    
}


double lastLocationLat = 0;
double lastLocationLng = 0;
Point brcClosestPoint(double pLat, double pLng){
  //if first loop, save this spot as waypoint 
  //check every time moving 100 ft from a given spot
  const int brcBookSize = 315;
char *brcBookNames[] = { "2:00&Esp","2:30&Esp","3:00&Esp","3:30&Esp","4:00&Esp","4:30&Esp","5:00&Esp",\
"5:30&Esp","6:00&Esp","6:30&Esp","7:00&Esp","7:30&Esp","8:00&Esp","8:30&Esp","9:00&Esp","9:30&Esp",\
"10:00&Esp","12:00&Esp","2:00&A","2:30&A","3:00&A","3:30&A","4:00&A","4:30&A","5:00&A","5:30&A",\
"6:30&A","7:00&A","7:30&A","8:00&A","8:30&A","9:00&A","9:30&A","10:00&A","2:00&B","2:30&B","3:00&B",\
"3:30&B","4:00&B","4:30&B","5:00&B","5:30&B","6:30&B","7:00&B","7:30&B","8:00&B","8:30&B","9:00&B",\
"9:30&B","10:00&B","2:00&C","2:30&C","3:00&C","3:30&C","4:00&C","4:30&C","5:00&C","5:30&C","6:30&C",\
"7:00&C","7:30&C","8:00&C","8:30&C","9:00&C","9:30&C","10:00&C","2:00&D","2:30&D","3:00&D","3:30&D",\
"4:00&D","4:30&D","5:00&D","5:30&D","6:00&D","6:30&D","7:00&D","7:30&D","8:00&D","8:30&D","9:00&D",\
"9:30&D","10:00&D","2:00&E","2:30&E","3:00&E","3:30&E","4:00&E","4:30&E","5:00&E","5:30&E","6:00&E",\
"6:30&E","7:00&E","7:30&E","8:00&E","8:30&E","9:00&E","9:30&E","10:00&E","2:30&F","3:00&F","3:30&F",\
"4:00&F","4:30&F","5:00&F","7:00&F","7:30&F","8:00&F","8:30&F","9:00&F","9:30&F","2:00&G","2:15&G",\
"2:30&G","2:45&G","3:00&G","3:15&G","3:30&G","3:45&G","4:00&G","4:15&G","4:30&G","4:45&G","5:00&G",\
"5:15&G","5:30&G","5:45&G","6:00&G","6:15&G","6:30&G","6:45&G","7:00&G","7:15&G","7:30&G","7:45&G",\
"8:00&G","8:15&G","8:30&G","8:45&G","9:00&G","9:15&G","9:30&G","9:45&G","10:00&G","2:00&H","2:15&H",\
"2:30&H","2:45&H","3:00&H","3:15&H","3:30&H","3:45&H","4:00&H","4:15&H","4:30&H","4:45&H","5:00&H",\
"5:15&H","5:30&H","5:45&H","6:00&H","6:15&H","6:30&H","6:45&H","7:00&H","7:15&H","7:30&H","7:45&H",\
"8:00&H","8:15&H","8:30&H","8:45&H","9:00&H","9:15&H","9:30&H","9:45&H","10:00&H","2:00&I","2:15&I",\
"2:30&I","2:45&I","3:00&I","3:15&I","3:30&I","3:45&I","4:00&I","4:15&I","4:30&I","4:45&I","5:00&I",\
"5:15&I","5:30&I","5:45&I","6:00&I","6:15&I","6:30&I","6:45&I","7:00&I","7:15&I","7:30&I","7:45&I",\
"8:00&I","8:15&I","8:30&I","8:45&I","9:00&I","9:15&I","9:30&I","9:45&I","10:00&I","2:00&J","2:15&J",\
"2:30&J","2:45&J","3:00&J","3:15&J","3:30&J","3:45&J","4:00&J","4:15&J","4:30&J","4:45&J","5:00&J",\
"5:15&J","5:30&J","5:45&J","6:00&J","6:15&J","6:30&J","6:45&J","7:00&J","7:15&J","7:30&J","7:45&J",\
"8:00&J","8:15&J","8:30&J","8:45&J","9:00&J","9:15&J","9:30&J","9:45&J","10:00&J","2:00&K","2:15&K",\
"2:30&K","2:45&K","3:00&K","3:15&K","3:30&K","3:45&K","4:00&K","4:15&K","4:30&K","4:45&K","5:00&K",\
"5:15&K","5:30&K","5:45&K","6:00&K","6:15&K","6:30&K","6:45&K","7:00&K","7:15&K","7:30&K","7:45&K",\
"8:00&K","8:15&K","8:30&K","8:45&K","9:00&K","9:15&K","9:30&K","9:45&K","10:00&K","2:00&L","2:15&L",\
"2:30&L","2:45&L","3:00&L","3:15&L","3:30&L","3:45&L","4:00&L","4:15&L","4:30&L","4:45&L","5:00&L",\
"5:15&L","5:30&L","5:45&L","6:00&L","6:15&L","6:30&L","6:45&L","7:00&L","7:15&L","7:30&L","7:45&L",\
"8:00&L","8:15&L","8:30&L","8:45&L","9:00&L","9:15&L","9:30&L","9:45&L","10:00&L","12:00&CenterC",\
"3:00&CenterC","6:00&CenterC","9:00&CenterC" };
double brcBookCoords[] = { 40.78463,-119.19779,40.78297,-119.19869,40.78155,-119.20013,40.78047,-119.20199,\
40.77978,-119.20417,40.77955,-119.20650,40.77978,-119.20883,40.78047,-119.21101,40.78222,-119.21200,40.78297,\
-119.21431,40.78463,-119.21521,40.78640,-119.21552,40.78817,-119.21521,40.78983,-119.21431,40.79125,-119.21287,\
40.79233,-119.21101,40.79302,-119.20883,40.79125,-119.20013,40.78431,-119.19626,40.78237,-119.19732,40.78070,\
-119.19900,40.77942,-119.20120,40.77862,-119.20376,40.77834,-119.20650,40.77862,-119.20924,40.77944,-119.21184,\
40.78234,-119.21565,40.78431,-119.21674,40.78640,-119.21710,40.78848,-119.21674,40.79043,-119.21568,40.79210,\
-119.21400,40.79338,-119.21180,40.79418,-119.20924,40.78414,-119.19542,40.78204,-119.19657,40.78024,-119.19839,\
40.77885,-119.20077,40.77798,-119.20353,40.77768,-119.20650,40.77798,-119.20947,40.77885,-119.21223,40.78204,\
-119.21643,40.78414,-119.21758,40.78640,-119.21797,40.78866,-119.21758,40.79076,-119.21643,40.79256,-119.21461,\
40.79395,-119.21223,40.79482,-119.20947,40.78397,-119.19459,40.78171,-119.19582,40.77977,-119.19778,40.77828,\
-119.20033,40.77735,-119.20331,40.77703,-119.20650,40.77735,-119.20969,40.77828,-119.21266,40.78171,-119.21718,\
40.78397,-119.21841,40.78640,-119.21883,40.78883,-119.21841,40.79109,-119.21718,40.79303,-119.21522,40.79452,\
-119.21267,40.79545,-119.20969,40.78380,-119.19375,40.78138,-119.19507,40.77931,-119.19717,40.77771,-119.19990,\
40.77671,-119.20309,40.77637,-119.20650,40.77671,-119.20992,40.77771,-119.21310,40.77931,-119.21582,40.78138,\
-119.21793,40.78380,-119.21925,40.78640,-119.21970,40.78900,-119.21925,40.79141,-119.21793,40.79349,-119.21583,\
40.79509,-119.21310,40.79609,-119.20992,40.78363,-119.19292,40.78105,-119.19432,40.77884,-119.19656,40.77714,\
-119.19947,40.77607,-119.20286,40.77571,-119.20650,40.77607,-119.21014,40.77714,-119.21353,40.77884,-119.21644,\
40.78105,-119.21868,40.78363,-119.22008,40.78640,-119.22056,40.78917,-119.22009,40.79174,-119.21868,40.79396,\
-119.21644,40.79566,-119.21353,40.79672,-119.21014,40.78073,-119.19357,40.77838,-119.19594,40.77657,-119.19904,\
40.77544,-119.20264,40.77505,-119.20650,40.77544,-119.21036,40.78346,-119.22092,40.78640,-119.22143,40.78934,\
-119.22092,40.79207,-119.21943,40.79442,-119.21706,40.79623,-119.21397,40.78329,-119.19124,40.78181,-119.19191,\
40.78040,-119.19282,40.77909,-119.19397,40.77791,-119.19533,40.77688,-119.19689,40.77600,-119.19860,40.77531,\
-119.20046,40.77480,-119.20241,40.77450,-119.20444,40.77439,-119.20650,40.77450,-119.20856,40.77480,-119.21059,\
40.77531,-119.21254,40.77600,-119.21440,40.77688,-119.21611,40.77791,-119.21767,40.77909,-119.21903,40.78040,\
-119.22018,40.78181,-119.22109,40.78329,-119.22175,40.78483,-119.22216,40.78640,-119.22229,40.78797,-119.22216,\
40.78951,-119.22176,40.79099,-119.22109,40.79240,-119.22018,40.79371,-119.21903,40.79489,-119.21767,40.79592,\
-119.21612,40.79680,-119.21440,40.79749,-119.21255,40.79800,-119.21059,40.78312,-119.19041,40.78155,-119.19111,\
40.78007,-119.19207,40.77869,-119.19329,40.77745,-119.19472,40.77635,-119.19636,40.77543,-119.19817,40.77470,\
-119.20013,40.77417,-119.20219,40.77385,-119.20433,40.77374,-119.20650,40.77385,-119.20867,40.77417,-119.21081,\
40.77470,-119.21287,40.77543,-119.21483,40.77635,-119.21664,40.77745,-119.21828,40.77869,-119.21971,40.78007,\
-119.22093,40.78155,-119.22189,40.78312,-119.22259,40.78475,-119.22302,40.78640,-119.22316,40.78805,-119.22302,\
40.78968,-119.22259,40.79125,-119.22189,40.79273,-119.22093,40.79411,-119.21972,40.79535,-119.21828,40.79644,\
-119.21664,40.79737,-119.21483,40.79810,-119.21288,40.79863,-119.21081,40.78295,-119.18957,40.78130,-119.19031,\
40.77974,-119.19132,40.77829,-119.19260,40.77698,-119.19411,40.77583,-119.19583,40.77486,-119.19774,40.77409,\
-119.19979,40.77353,-119.20196,40.77319,-119.20421,40.77308,-119.20650,40.77319,-119.20879,40.77353,-119.21103,\
40.77409,-119.21320,40.77486,-119.21526,40.77583,-119.21717,40.77698,-119.21889,40.77829,-119.22040,40.77974,\
-119.22168,40.78130,-119.22269,40.78295,-119.22343,40.78466,-119.22388,40.78640,-119.22402,40.78814,-119.22388,\
40.78985,-119.22343,40.79150,-119.22269,40.79306,-119.22168,40.79451,-119.22040,40.79582,-119.21889,40.79697,\
-119.21717,40.79793,-119.21526,40.79871,-119.21321,40.79927,-119.21104,40.78278,-119.18874,40.78105,-119.18951,\
40.77941,-119.19058,40.77789,-119.19191,40.77651,-119.19350,40.77531,-119.19531,40.77429,-119.19731,40.77349,\
-119.19946,40.77290,-119.20174,40.77254,-119.20410,40.77242,-119.20650,40.77254,-119.20890,40.77290,-119.21126,\
40.77349,-119.21354,40.77429,-119.21569,40.77531,-119.21769,40.77651,-119.21950,40.77789,-119.22109,40.77941,\
-119.22243,40.78105,-119.22349,40.78278,-119.22426,40.78457,-119.22473,40.78640,-119.22489,40.78822,-119.22473,\
40.79002,-119.22426,40.79175,-119.22349,40.79339,-119.22243,40.79491,-119.22109,40.79628,-119.21951,40.79749,\
-119.21770,40.79851,-119.21570,40.79931,-119.21354,40.79990,-119.21126,40.78261,-119.18790,40.78080,-119.18871,\
40.77908,-119.18983,40.77749,-119.19123,40.77605,-119.19289,40.77479,-119.19478,40.77372,-119.19687,40.77288,\
-119.19913,40.77226,-119.20152,40.77189,-119.20399,40.77176,-119.20650,40.77189,-119.20901,40.77226,-119.21148,\
40.77288,-119.21387,40.77372,-119.21613,40.77479,-119.21822,40.77605,-119.22011,40.77749,-119.22177,40.77908,\
-119.22317,40.78080,-119.22429,40.78261,-119.22510,40.78449,-119.22559,40.78640,-119.22576,40.78831,-119.22559,\
40.79019,-119.22510,40.79200,-119.22429,40.79372,-119.22318,40.79531,-119.22178,40.79675,-119.22012,40.79801,\
-119.21822,40.79908,-119.21613,40.79992,-119.21387,40.80054,-119.21148,40.78244,-119.18707,40.78055,-119.18791,\
40.77875,-119.18908,40.77709,-119.19054,40.77558,-119.19227,40.77427,-119.19425,40.77315,-119.19644,40.77227,\
-119.19880,40.77163,-119.20129,40.77124,-119.20387,40.77111,-119.20650,40.77124,-119.20913,40.77163,-119.21171,\
40.77227,-119.21420,40.77315,-119.21656,40.77427,-119.21875,40.77558,-119.22073,40.77709,-119.22246,40.77875,\
-119.22392,40.78055,-119.22509,40.78244,-119.22594,40.78440,-119.22645,40.78640,-119.22662,40.78839,-119.22645,\
40.79036,-119.22594,40.79225,-119.22509,40.79405,-119.22393,40.79571,-119.22247,40.79721,-119.22073,40.79853,\
-119.21875,40.79964,-119.21656,40.80053,-119.21420,40.80117,-119.21171,40.78141,-119.21307,40.78013,-119.21307,\
40.78013,-119.21475,40.78141,-119.21475 };
  double lat = pLat;
  double lng = pLng;
  double closeDistance = 999999999, closeLat=0, closeLng=0;
  int closeNameIndex;
  Point brcPointClosest;

  for (int i=0; i<((brcBookSize*2)); i+=2) {
    double testLat = brcBookCoords[i];
    double testLng = brcBookCoords[(i+1)];
    double testClose = gps.distanceBetween(lat, lng, testLat, testLng);
    
    if(testClose<closeDistance){
      closeDistance = testClose;
      closeLat = testLat;
      closeLng = testLng;
      closeNameIndex = i / 2;
      
      if(0==1){
        Serial.print("New closet desintation(nameIndex=");
        Serial.print(closeNameIndex);
        Serial.print(",i=");
        Serial.print(i);
        Serial.print("): ");
        Serial.print(brcBookNames[closeNameIndex]);
        Serial.print(" ("); 
        Serial.print(testClose);  
        Serial.println("meters away)");  
      }
    }
    strcpy(brcPointClosest.label,brcBookNames[closeNameIndex]);
    brcPointClosest.lat = closeLat;
    brcPointClosest.lng = closeLng;
  }
  return brcPointClosest;
}


int checkButton() {    
   int event = 0;
  
   buttonVal = digitalRead(buttonPin);
   // Button pressed down

   if(printCounter > 10){
    printCounter = 0;
   Serial.print("Debug: buttonVal:");
   Serial.print(buttonVal);
   Serial.print(" buttonLast:");
   Serial.print(buttonLast);
   Serial.print(" dcGap:");
   Serial.print(upTime);
   Serial.println(" ");
   }
   //printCounter++;
   if (buttonVal == HIGH && buttonLast == LOW && (millis() - upTime) > debounce)
   {
       downTime = millis();
       ignoreUp = false;
       waitForUp = false;
       singleOK = true;
       holdEventPast = false;
       longHoldEventPast = false;
       if ((millis()-upTime) < DCgap && DConUp == false && DCwaiting == true)  DConUp = true;
       else  DConUp = false;
       DCwaiting = false;
   }
   // Button released
   else if (buttonVal == LOW && buttonLast == HIGH && (millis() - downTime) > debounce)
   {        
       if (not ignoreUp)
       {
           upTime = millis();
           if (DConUp == false){
            DCwaiting = true;
           } else {
               event = 2;
               DConUp = false;
               DCwaiting = false;
               singleOK = false;
           }
       }
   }

   
   // Test for normal click event: DCgap expired
   if ( buttonVal == LOW && (millis()-upTime) >= DCgap && DCwaiting == true && DConUp == false && singleOK == true && event != 2)
   {
       event = 1;
       DCwaiting = false;
   }
   // Test for hold
   if (buttonVal == HIGH && (millis() - downTime) >= holdTime) {
       // Trigger "normal" hold
       if (not holdEventPast)
       {
           event = 3;
           waitForUp = true;
           ignoreUp = true;
           DConUp = false;
           DCwaiting = false;
           //downTime = millis();
           holdEventPast = true;
       }
       // Trigger "long" hold
       if ((millis() - downTime) >= longHoldTime)
       {
           if (not longHoldEventPast)
           {
               event = 4;
               longHoldEventPast = true;
           }
       }
   }
   buttonLast = buttonVal;
   return event;
}


void clickEvent() {
  //Serial.print("Click event :: screenIndex: pre=");
  //Serial.print(screenIndex);

  if(screenIndex<100){
    //OVERVIEW click, so advance index
    if(screenIndex < (rosterEntries-1)){
      screenIndex++;
    } else {
      screenIndex = 0;
    }
  } else {
    //detail click, do beep stuff
    Serial.println("Beep beep...");
  }
  //Serial.print(" post=");
  //Serial.println(screenIndex);
  updateScreen();
}
void doubleClickEvent() {
  Serial.println("Double click event!");
}
void holdEvent() {
  //Serial.print("Hold event :: screenIndex: pre=");
  //Serial.println(screenIndex);
  if(screenIndex<100){
  //OVERVIEW hold, so go to detail by adding 100 index
    screenIndex += 100;
  } else if(screenIndex<200) {
     //detail hold, so return to overview while still mainting continuity
    screenIndex -= 100; 
  } else {
    Serial.println("WARNING over 200 screenIndex on hold event");
  }
  //Serial.print("Hold event :: screenIndex: post=");
  //Serial.println(screenIndex);
  updateScreen();
}
void longHoldEvent() {
  Serial.print("Long hold event :: screenIndex: pre=");
  Serial.println(screenIndex);
}
