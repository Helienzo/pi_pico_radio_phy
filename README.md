## Phy layer for the RFM69 radio for PI PICO

This is a TDD based phy layer for the the RFM69 radio specifically for the PI PICO. It currently builds ontop of the https://github.com/Helienzo/rfm69_radio_hal.
  
To test this module check out the examples.  
Note that the examples pulls in a couple of submodules using CMAKE FetchContent. Check the the examples/example/CMakeLists.txt, for source repos.  
  
## Connect the RFM69 chip to Pi Pico As follows:
  
PICO 16 - MISO  
PICO 17 - NSS  
PICO 18 - SCK  
PICO 19 - MOSI  
PICO 20 - RESET  
PICO 21 - DIO0  
  
## Known limitations
 The TDD mode currently only supports two devices in a point to point communication mode.
 There might be issues when trying to send close to max throughput, Im working on that..
