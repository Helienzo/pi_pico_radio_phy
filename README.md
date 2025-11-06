## Phy control layer for the RFM69 radio for Pi Pico 2

This is a PHY layer that provides a time-synchronized frame structure suitable for TDD and TDMA-based MAC protocols. It is built for the RFM69 family of radios specifically for the Pi Pico 2. It currently builds on top of the https://github.com/Helienzo/rfm69_radio_hal. The time frame can be configured with multiple slots of different lengths. During runtime each slot can be allocated as receive or transmit.
  
To test this module check out the examples.  
Note that the examples pulls in a couple of submodules using CMAKE FetchContent. Check the examples/example/CMakeLists.txt, for source repos.  
  
## Connect the RFM69 chip to Pi Pico As follows:
  
PICO 16 - MISO  
PICO 17 - NSS  
PICO 18 - SCK  
PICO 19 - MOSI  
PICO 20 - RESET  
PICO 21 - DIO0  
PICO 15 - DIO1
  
## Documentation
The module supports three modes of operation.  
Aloha  
Time framed Central Mode  
Time framed Scan/Peripheral  
  
In Aloha mode the radio is continuously in RX waiting for incoming packets, either on the broadcast address or the unique node address. When an outgoing packet transfer is triggered the radio switches to TX mode, sends the packet and then switches back to RX again. It is currently controlled through software, not utilizing the auto mode of the radio. This mode does not utilize the time frame and is suitable for testing.
  
In central mode the radio periodically sends a beacon sync signal with precise timing using interrupt driven timers. This beacon is used by peripheral devices to get a time reference to when to expect packet RX and TX.
  
In scan/peripheral mode the radio first scans for beacons and once found synchronizes it's internal periodic timer to the timer of the central device. The radio remains synchronized as long as it hears a beacon at least once during a configurable interval.  
  
In these modes the send function queues packets in a common queue, the maximum number of packets that fits in the queue can be configured. At the start of the next frame the packets are queued in the set outbound slot. The main limitation is the time length of the slot and the bitrate of the radio. Queuing more data than there is time available to send in a slot will make the packet spill over into the next available time frame. That might cause congestion in a high throughput situation. Once a packet is sent the module automatically pops the next packet from the queue and send it until the TX slot time is complete.  
  
During an RX slot the radio listens continuously for incoming packets.  
  
The Frame can be configured to use any number of slots and each slot can scheduled either as an RX or IDLE slot. Both RX and IDLE slots can be used to send packets. The slot is temporarily set as TX for a frame as long as there are packets available.
  
## Known limitations
 The send function in central/peripheral mode currently does not warn if the total number of packets queued in a single slot exceeds the maximum available transfer time, the packet queue might take multiple number of frames to empty.