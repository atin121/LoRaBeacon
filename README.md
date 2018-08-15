# LoRa Beacon
**An off grid long range low power radio network for location information**
Concieved as art project / beacon finder for Burning Man, with an eye towards other rugged off-grid tracking use cases

The beacons use a Esp32 microcontroller to: 
1) Fix its position on Earth 
2) Distribute that position via a LoRa radio network
3) Display recieved positions in human-meaningful form on a low power display

This project (and all source code) is distributed under a Creative Commons - Attribution-ShareAlike 4.0 International (CC BY-SA 4.0) https://creativecommons.org/licenses/by-sa/4.0/ In short, if you use this code, you cannot sell it, and you need to attribute at some place where you got it from (so the creators can see all the cool stuff you are up to). Inspired by the Future Robot's Burning Man Tailsman art project. 

Pull requests and questions welcome!


# Hardware Specifics
- Esp32 TTGO T-Beam 895/915mhz microcontroller
- SSD1306 32x128px 3.3v OLED display

# General Usage Notes
- Will need an appropriate library for your display and board (though for this specific controller the ESP32 debv board configuration in the Arduino IDE works well)
- Understand the constants (as indicated by comments in the Main.ino code) and make sure they are configured appropriately for your use case
- The TTGO will blink a red LED (next to the power) once a GPS fix is aquired - until that point, no packets are sent or recieved (because it uses the GPS time clock to synch transmission). To test indoors, flip the modeNoGpsTesting mode to true to recieve and display packets only. 
