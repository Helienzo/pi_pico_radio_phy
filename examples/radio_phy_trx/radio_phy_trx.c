#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "phy_radio.h"
#include "hal_gpio.h"
#include "pico_bootsel_button.h"

/*
 This example demonstrates how to use the phyRadio module in basic ALOHA mode.
 The phyRadio module supports many more modes of operation to enable higer througputs without
 packet congestion in air. But that is in another example.

 This example can be flashed to two PICO's with a RFM69 radio.
 Both radios will be in RX mode waiting for packets.
 Send a packet by pressing the pico bootsel button. This switches the radio to TX mode, sends the
 packet and then returns to RX mode. Also known as a basic form of the ALHOA protocol.

 The radio is configured in interrupt mode to notify about send complete, and packet available.
 how ever, the callbacks are not in ISR context, they are called through the proccess function.

 Note: that the example uses the broadcast address to enable flashing multiple devices without changing addresses.
       all devices will receive the packet sent. (Excluding the sender)
*/

// Radio configuration defines
#define RADIO_MY_ADDR         (0x02) // Change this to target specific radios
#define RADIO_TARGET_ADDR     (0x01) // Change this to target specific radios
#define RADIO_BROADCAST_ADDR  (0xFF)
#define RADIO_DEFAULT_CHANNEL (868)
#define RADIO_RX_BUFFER_SIZE  (128 + C_BUFFER_ARRAY_OVERHEAD)
#define RADIO_TX_BUFFER_SIZE  (128 + C_BUFFER_ARRAY_OVERHEAD) 
#define RADIO_TX_POWER_DBM    (0)

#ifndef LOG
#define LOG(f_, ...) printf((f_), ##__VA_ARGS__)
#endif

static void device_error();

uint8_t msg[] = {'H', 'e', 'l', 'l', 'o', '!'};


// phyRadio
typedef struct {
    phyRadio_t          phy_radio_inst;
    phyRadioInterface_t phy_interface;

    // Packet management
    bool             available;
    phyRadioPacket_t phy_pkt;
    uint8_t          tx_byte_array[RADIO_TX_BUFFER_SIZE];
    cBuffer_t        tx_buffer;

    // Button management
    picoBootSelButton_t          boot_button;
    picoBootSelButtonInterface_t btn_interface;

    // LED management
    bool test_led_state;
} myInstance_t;

static myInstance_t my_instance = {0};

// Perform initialisation
int pico_led_init(void) {
    // A device like Pico that uses a GPIO for the LED will define PICO_DEFAULT_LED_PIN
    // so we can use normal GPIO functionality to turn the led on and off
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    return PICO_OK;
}

// Turn the led on or off
void pico_set_led(bool led_on) {
    // Just set the GPIO on or off
    gpio_put(PICO_DEFAULT_LED_PIN, led_on);
}

void buttonEventCb(picoBootSelButtonInterface_t *interface, picoBootSelButtonEvent_t event) {
    myInstance_t * inst = CONTAINER_OF(interface, myInstance_t, btn_interface);

    // Make sure that there is no transfer ongoing
    if (!inst->available) {
        return;
    }

    int32_t res = cBufferPrepend(&inst->tx_buffer, msg, sizeof(msg));
    if (res != sizeof(msg)) {
        LOG("RADIO SEND FAILED! %i\n", res);
        device_error();
    }

    // Queue a packet for transmission
    res = phyRadioSendOnSlot(&inst->phy_radio_inst, &inst->phy_pkt, true); // The this_frame param has no effect in alhoa mode

    if (res != PHY_RADIO_SUCCESS) {
        LOG("RADIO SEND FAILED! %i\n", res);
        device_error();
    }

    // The packet is not available until transfer is complete
    inst->available = false;
}

static int32_t phyPacketCallback(phyRadioInterface_t *interface, phyRadioPacket_t *packet) {
    myInstance_t * inst = CONTAINER_OF(interface, myInstance_t, phy_interface);

    inst->test_led_state = !inst->test_led_state;
    pico_set_led(inst->test_led_state);

    int32_t result = cBufferAvailableForRead(packet->pkt_buffer);

    if (result < 0) {
        LOG("Invalid packet received %i.\n", result);
        return result;
    }

    LOG("%i bytes received from %u.\n", result, packet->addr);

    // Print out payload
    LOG("Payload: ");
    for (int32_t i = 0; i < result; i++) {
        LOG("%c", cBufferReadByte(packet->pkt_buffer));
    }
    LOG("\n\n");

    return PHY_RADIO_SUCCESS;
}

static int32_t phyPacketSent(phyRadioInterface_t *interface, phyRadioPacket_t *packet, phyRadioErr_t result) {
    myInstance_t * inst = CONTAINER_OF(interface, myInstance_t, phy_interface);
    if (result != PHY_RADIO_SUCCESS) {
        LOG("Packet send failed %i.\n", result);
        device_error();
    }

    // Note that the packet pointer is the packet we queued before/
    // Useful if we want to keep track of what packet is in flight.

    // The packet is now available
    inst->available = true;
    return PHY_RADIO_SUCCESS;
}

static int32_t phySyncStateCb(phyRadioInterface_t *interface, uint32_t sync_id, const phyRadioSyncState_t *sync_state) {
    myInstance_t * inst = CONTAINER_OF(interface, myInstance_t, phy_interface);
    // Unused in Aloha mode
    return PHY_RADIO_SUCCESS;
}

int main()
{
    stdio_init_all();
    halGpioInit(); // Initialize the gpio module
    int rc = pico_led_init();
    hard_assert(rc == PICO_OK);

    // Prepare bootsel button
    my_instance.btn_interface.event_cb = buttonEventCb;
    int32_t res = picoBootSelButtonInit(&my_instance.boot_button, &my_instance.btn_interface);
    if (res != PICO_BOOTSEL_BTN_SUCCESS) {
        LOG("BUTTON INIT FAILED!\n");
        device_error();
    }

    // Configure the phy interface
    my_instance.phy_interface.packet_cb     = phyPacketCallback;
    my_instance.phy_interface.sent_cb       = phyPacketSent;
    my_instance.phy_interface.sync_state_cb = phySyncStateCb;

    // Initialize the phy radio
    if ((res = phyRadioInit(&my_instance.phy_radio_inst, &my_instance.phy_interface, RADIO_MY_ADDR)) != PHY_RADIO_SUCCESS) {
        LOG("RADIO INIT FAILED! %i\n", res);
        device_error();
    }

    // Set phy radio mode
    if ((res = phyRadioSetAlohaMode(&my_instance.phy_radio_inst)) != PHY_RADIO_SUCCESS) {
        LOG("RADIO SET MODE FAILED! %i\n", res);
        device_error();
    }

    // Init the TX buffer
    if((res = cBufferInit(&my_instance.tx_buffer, my_instance.tx_byte_array, RADIO_RX_BUFFER_SIZE)) != C_BUFFER_SUCCESS) {
        LOG("BUFFER INIT FAILED! %i\n", res);
        device_error();
    }

    // Set the packet as available
    my_instance.available = true;

    // Set up the phy packet
    my_instance.phy_pkt.addr       = RADIO_TARGET_ADDR;       // Only matters for some packet types
    my_instance.phy_pkt.pkt_buffer = &my_instance.tx_buffer;
    my_instance.phy_pkt.type       = PHY_RADIO_PKT_BROADCAST; // Use PHY_RADIO_PKT_DIRECT to target specific radios

    while (true) {
        res = phyRadioProcess(&my_instance.phy_radio_inst);
        if (res != PHY_RADIO_SUCCESS) {
            LOG("RADIO PROCESS FAILED! %i\n", res);
            device_error();
        }

        // Process the button
        res = picoBootSelButtonProcess(&my_instance.boot_button);
        if (res != PICO_BOOTSEL_BTN_SUCCESS) {
            LOG("BUTTON PROCESS FAILED!\n");
            device_error();
        }
    }
}

static void device_error() {
    // Forever blink fast
    while (true) {
        pico_set_led(true);
        sleep_ms(100);
        pico_set_led(false);
        sleep_ms(100);
    }
}
