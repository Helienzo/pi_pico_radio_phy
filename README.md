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
  
## Documentation
The module supports three modes of operation.  
Aloha  
TDD Central  
TDD Scan/Peripheral  
  
In Aloha mode the radio is continously in RX waiting for incoming packets, either on the broadcast address or the unique node address. When an outgoing packet transfer is triggered the radio switches to TX mode, sends the packet and then switches back to RX again. It is currently controlled through software, not utelizing the auto mode of the radio.  
  
In TDD central mode the radio periodically sends a beacon sync signal with precise timing using interrupt driven timers. This beacon is used by peripheral devices to get a time reference to when to expect packet RX and TX.
  
In TDD scan/peripheral mode the radio first scans for beacons and once found synchronizes it's internal periodic timer to the timer of the central device. The radio remains synchronized as long as it hears a beacon at least once during a configurable interval.  
  
In TDD mode the send function queues packets on a given slot, the maximum number of packets that fits in the queue can be configured. The main limitation is the time length of the slot and the bitrate of the radio. Queuing more data than we have time to send will make the packet spill over into the next available send slot. That might cause congestion in a high trouhput situation. Once a packet is sent the module automatically pops the next packet from the queue and send it until the TX slot time is complete.  
  
During an RX slot the radio listens continously for incomming packets.  

## Known limitations
 The TDD mode currently only supports two devices in a point to point communication mode. I have plans to implement multidevice support sometimes in the future.

 The send function in TDD mode currently does not warn if the total number of packets queued in a single slot exceeds the maximum available tranfer time. I might add a check for that in a later fix.