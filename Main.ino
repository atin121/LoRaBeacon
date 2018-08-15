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

//device constants
const char deviceName[ ] = "Beta4"; //5 chars ONLY
const int userRadioId = 4; //must be less than modValue
const int modValue = 5; //10 or more for prod, less transmi, more listen
//mode flags
const bool modeNoGpsTesting = false;
const bool modeDiagnostic = false;
const double displayScrollSeconds = 5;
const int gpsTimerTick = 333;
int displayScrollPauseTime = 0; //allows for new users and error messages to get extra time

//globals
TinyGPSPlus gps;  
int sentPacketCount = 0;
bool gpsFix = false;

//listStuff

const int listSize = 10; //total size of list of recognized devices
int listEntries = 0; //current number of entries in said list 
int listDisplayTick = 0;  //how many ticks (333ms) the current item has been displayed
int listDisplayPointer = 0; //the current row in the list that is being displayed
char nameList[listSize][6];
double latList[listSize], lngList[listSize], speedList[listSize];

void setup() {
  //Setup GPS:
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, 12, 15);   //17-TX 18-RX for GPS
  //disable radiozz`
  WiFi.mode(WIFI_OFF); 
  btStop();
  //startup LoRa
  SPI.begin(5,19,27,18);
  LoRa.setPins(SS,RST,DI0);
  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  //setup LoRa
  LoRa.setTxPower(18); // 2-20, *17
  LoRa.setSpreadingFactor(7); //6-12, *7
  LoRa.setCodingRate4(5); //5-8, *5  
  LoRa.setSignalBandwidth(31.25E3);  //7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3, 41.7E3, 62.5E3, *125E3, 250E3.
  LoRa.crc(); //*off
  //setup display
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)
  ui_textLarge("booting up...");
  display.display();
  //addPacketToList(" MAN ", 40.78640, -119.20650, 0);
  //addPacketToList("FUNKY", 40.79990, -119.21126, 0);
  //addPacketToList("Close", 0.000155, 0.000156, 0);
}

void loop() {  
  //Startup:
  //disables for testing
  if((gps.satellites.value() <= 3)&&(modeNoGpsTesting == false)){
      Serial.println("Aquiring GPS fix...");
      ui_pendingGps();
      gpsWait(2500);    
  } else {  //bumped one tab left for readability
    
  
  if(gpsFix == false){ //shows on first run
    Serial.println("GPS position found!");
    PrintGpsState();
    ui_listening();
    gpsFix = true;
  }
  
  if((gps.time.second() % modValue == userRadioId) && (gpsFix = true)){
    //time to transmit, adding a small random delay to help ease congestion
    const int randy = random(0,200);
    gpsWait(randy);
    Serial.print("Randy: ");
    Serial.println(randy);
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
      digestPacket(packet, sizeof(packet), LoRa.packetRssi());
    }
  }

  //do we have something to display? if so, display it
  displayNextRow();
  gpsWait(gpsTimerTick);                                      
  if (millis() > 5000 && gps.charsProcessed() < 10){
    Serial.println(F("No GPS data received: check wiring")); 
  }

  }//end GPS related if
}


void transmitLocation(){

  if(gps.satellites.value() <= 3){
      Serial.println("Lost GPS fix...");
      ui_textLarge("Lost GPS");
      displayScrollPauseTime = 1;
      gpsFix = false;
      gpsWait(2000);
      return;    
  }
  
  // send packet
  sentPacketCount++;
  Serial.println("Sending packet...");
  LoRa.beginPacket();
  LoRa.print("");
  LoRa.print(deviceName);
  LoRa.print(" ");
  LoRa.print(gps.location.lat(), 6);
  LoRa.print(" ");
  LoRa.print(gps.location.lng(), 6);
  LoRa.print(" ");
  LoRa.print(gps.speed.mph());
  LoRa.print(" ");
  LoRa.print(sentPacketCount);
  LoRa.print(" "); //eases digestion easier
  LoRa.endPacket();
  
  //digitalWrite(25, HIGH);   // turn the LED on (HIGH is the voltage level)
  //delay(1000);                       // wait for a second
  //digitalWrite(25, LOW);    // turn the LED off by making the voltage LOW
  //delay(1000);                       // wait for a second
}

void PrintGpsState()
{
  Serial.println("**********************");
  Serial.print("Latitude  : ");
  Serial.println(gps.location.lat(), 6);
  Serial.print("Longitude : ");
  Serial.println(gps.location.lng(), 6);
  Serial.print("Satellites: ");
  Serial.println(gps.satellites.value());
  Serial.print("Altitude  : ");
  Serial.print(gps.altitude.feet()); // / 3.2808
  Serial.println("M");
  Serial.print("Speed     : ");
  Serial.print(gps.speed.mph());
  Serial.println("mph");
  Serial.print("Time      : ");
  Serial.print(gps.time.hour());
  Serial.print(":");
  Serial.print(gps.time.minute());
  Serial.print(":");
  Serial.println(gps.time.second());
  Serial.println("**********************");
}

static void gpsWait(unsigned long ms)                
{
  unsigned long start = millis();
  do
  {
    while (Serial1.available())
      gps.encode(Serial1.read());
  } while (millis() - start < ms);
}


void digestPacket(char* passPacket, int passSize, int RSSI) {
  char pckt[50];
  strncpy(pckt, passPacket, passSize);
  char senderName[] = "testi"; //this is required for reason i dont get
  double lat, lng, speed;
  int pcktCount;
  
  int dataCounter = 0; //tracks which data field
  int curCounter = 0; //tracks where in the current char
  int startPoint = 0; //track where we start reading string from???
  
  char current[13];
  char floatCharArray[13];
  char delimiter = ' ';
  
  //Serial.println(pckt);
  
  //loop through every char in the packet
  for (int i = 0; i < strlen(pckt); i++) {
   if (pckt[i] != delimiter) {
      //not a space, so add to charCurrent
      current[i-startPoint] = pckt[i];
      //Serial.print("current: ");
      //Serial.println(current);
    } else {
      //we hit a space, end of name, so save the name
      //Serial.println(" delimiter ");

      if(dataCounter == 0) {
        strncpy(senderName, current, 5);
      }
      if(dataCounter == 1) {
        strncpy(floatCharArray, current, 11);
        lat = atof(floatCharArray);
      }
      if(dataCounter == 2) {
        strncpy(floatCharArray, current, 11);
        lng = atof(floatCharArray);
      }
      if(dataCounter == 3) {
        strncpy(floatCharArray, current, 5);
        speed = atof(floatCharArray);
      }
      if(dataCounter == 4) {
        //Serial.print("time for counter: ");
        //Serial.println(current);
        strncpy(floatCharArray, current, sizeof(current));
        pcktCount = atof(floatCharArray);
      }
      if(dataCounter > 4){
         Serial.print("ERROR dataCounter too big");
      }
      //nuke the array
      for( int i = 0; i < sizeof(current);  ++i )
        current[i] = (char)0;
        
      startPoint = i+1;
      ++dataCounter;
    }
  }

  //add to list now!
  addPacketToList(senderName, lat, lng, speed);
  //preDisplayPacket(senderName, lat, lng, speed);
}



void displayNextRow(){
  if(listEntries == 0){
    //nothing to display, ignore
    return;
  }
  //display a fresh entry
  if(listDisplayTick == 0 ){
    displayPacket(nameList[listDisplayPointer], latList[listDisplayPointer], \
    lngList[listDisplayPointer], speedList[listDisplayPointer]);
  }
  listDisplayTick++;
  if(listDisplayTick > ((displayScrollSeconds + displayScrollPauseTime) * (1000 / gpsTimerTick))){
    displayScrollPauseTime = 0;
    listDisplayPointer++;
    if(listDisplayPointer >= listEntries){
      listDisplayPointer = 0;
    }
    listDisplayTick = 0;
  }
}

void displayPacket(char senderName[5], double lat, double lng, double speed){
  //const double EIFFEL_TOWER_LAT = 48.85826;
  //const double EIFFEL_TOWER_LNG = 2.294516;
  double distanceMeters = gps.distanceBetween(gps.location.lat(), gps.location.lng(), lat, lng);
  double courseTo = gps.courseTo(gps.location.lat(),gps.location.lng(),lat, lng);
  const char* ordinal = gps.cardinal(courseTo);

  //do some preperations to the data:
  double distanceMi = (distanceMeters / 1609.3400);
  double distanceFt = (distanceMeters * 3.28084);
  char distanceUnit[3];
  double distanceDisplay; 
  
  if(distanceFt > 9999) {
    strcpy(distanceUnit,"mi"); 
    if(modeNoGpsTesting == true){
      distanceDisplay = distanceMi-7951; //handles the 0,0 values returned
    } else {
      distanceDisplay = distanceMi;
    }
  } else {
    strcpy(distanceUnit,"ft");
    distanceDisplay = distanceFt;
  }

  if(speed > 1.0) {
    Serial.print(" moving at ");
    Serial.print(speed, 1);
    Serial.print("mph");
  }
  ui_displayBeacon(senderName, distanceDisplay, distanceUnit, speed, ordinal);
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


void ui_listening(){
  display.clearDisplay();
  display.setCursor(0,10);
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.println("listening");
  display.display();
}

void ui_textLarge(char* text){
  display.clearDisplay();
  display.setCursor(0,10);
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.println(text);
  display.display();
}

void ui_textSmall(char* text){
  display.clearDisplay();
  display.setCursor(0,10);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.println("listening");
  display.display();
}


void ui_displayBeacon(char senderName[5], double distance, char distanceUnit[2], double speed, const char* ordinal){
  display.clearDisplay();

  ui_draw_FiledRectangle(0,0,76,16);

  display.setCursor(8,0);
  display.setTextSize(2);
  display.setTextColor(BLACK, WHITE);
  display.print(senderName);

  display.setCursor(85,3);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.print(speed, 1);
  display.println("mph");

  display.setCursor(1,18);
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.print(distance);
  display.setTextSize(1);
  display.print(distanceUnit);
  
  display.setCursor(91,18);
  display.setTextSize(2);
  display.print(ordinal);

  display.display();
}

void ui_draw_FiledRectangle(int x, int y, int width, int height){
  for (uint8_t i=0; i< width; i++) {
    for (uint8_t q=0; q< height; q++) {
        display.drawPixel((x+i), (y+q), WHITE);
    }
  }
}

// LIST STUFF
void addPacketToList(char* pName, double pLat, double pLng, double pSpeed){
  char name[6];
  //char tempNameList[listSize][5];
  double lat = pLat, lng = pLng, speed = pSpeed;
  strncpy(name, pName, 6);

  int addAtRow = -1;
  for (uint8_t i=0; i< listSize; i++) {
    if(strcmp(nameList[i],name) == 0 ) {
      //Serial.print("Foudn user at row ");
      //Serial.print(i);
      addAtRow = i;
      break;
    }
  }
  
  if(addAtRow == -1){ //new user, find next empty row
    Serial.print("Packet from new user recieved - creating empty row");
    listEntries++;
    for (uint8_t q = 0; q < listSize; q++) {
    //loop through list to add
      if(latList[q] == 0){
        addAtRow = q;
        break;
      } 
    }
  }

  if(addAtRow == -1){
    Serial.println("ERROR: device list is full");
    Serial.print("List Entries: ");
    Serial.println(listEntries);
    ui_textSmall("!! List Full !!");
    return;
  }
  
  strncpy(nameList[addAtRow], name, 6);
  latList[addAtRow] = lat;
  lngList[addAtRow] = lng;
  speedList[addAtRow] = speed;
  return;
}

void printList(){
  Serial.println("** Printing List **");
  for (uint8_t i=0; i< listSize; i++) {
    //loop through list to add
    if(latList[i] != 0){
       Serial.print("iVal:");
       Serial.print(i);
       Serial.print(" Name:");
       Serial.print(nameList[i]);
       Serial.print(" Lat:");
       Serial.print(latList[i]);
       Serial.print(" Lng:");
       Serial.print(lngList[i]);
       Serial.println("");
    }
  }
}
