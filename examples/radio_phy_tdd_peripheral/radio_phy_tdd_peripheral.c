#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "phy_radio.h"
#include "hal_gpio.h"
#include "pico_bootsel_button.h"

/*
 This example demonstrates how to use the phyRadio module as a peripheral device.
 The peripheral device scans for sync messages sent by a central device and synchronizes once detected.

 The purpose of using TDD is to allow maximum throughput and minimizing collisions, by knowing when to send and when
 to listen the risk of colision is very small and all time can be utelized.

 NOTE: Using this code with a radio might not be legal. Allways follow your local radio spectrum regulations.

 This example should be used together with the central example on two PICO's with a RFM69 radio.

 The example works best if a second LED is connected GPIO 9 to show when packets arrive. The PICO on board
 LED is used to show synchronization.

 The radio is configured in interrupt mode to notify about send complete, and packet available.
 how ever, the callbacks are not in ISR context, they are called through the proccess function.

 Note: The example uses the broadcast address to enable flashing multiple devices without changing addresses.
       all devices will receive the packet sent. (Excluding the sender)
*/

// Radio configuration defines
#define RADIO_MY_ADDR         (0x01) // Change this to target specific radios
#define RADIO_TARGET_ADDR     (0x02) // Change this to target specific radios
#define RADIO_BROADCAST_ADDR  (0xFF)
#define RADIO_DEFAULT_CHANNEL (868)
#define RADIO_RX_BUFFER_SIZE  (128 + C_BUFFER_ARRAY_OVERHEAD)
#define RADIO_TX_BUFFER_SIZE  (128 + C_BUFFER_ARRAY_OVERHEAD) 
#define RADIO_TX_POWER_DBM    (0)
#define PKT_LED               (9)
#define SCAN_TIMEOUT_MS       (2000)

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
    bool pkt_led_state;
} myInstance_t;

static myInstance_t my_instance = {0};

// Perform initialisation
int pico_led_init(void) {
    // A device like Pico that uses a GPIO for the LED will define PICO_DEFAULT_LED_PIN
    // so we can use normal GPIO functionality to turn the led on and off
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_init(PKT_LED);
    gpio_set_dir(PKT_LED, GPIO_OUT);
    return PICO_OK;
}

// Turn the led on or off
void pico_set_led(bool led_on) {
    // Just set the GPIO on or off
    gpio_put(PICO_DEFAULT_LED_PIN, led_on);
}

// Turn the led on or off
void set_pkt_led(bool led_on) {
    // Just set the GPIO on or off
    gpio_put(PKT_LED, led_on);
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
    res = phyRadioSendOnSlot(&inst->phy_radio_inst, &inst->phy_pkt);

    if (res != PHY_RADIO_SUCCESS) {
        LOG("RADIO SEND FAILED! %i\n", res);
        device_error();
    }

    // The packet is not available until transfer is complete
    inst->available = false;
}

static int32_t phyPacketCallback(phyRadioInterface_t *interface, phyRadioPacket_t *packet) {
    myInstance_t * inst = CONTAINER_OF(interface, myInstance_t, phy_interface);

    inst->pkt_led_state = !inst->pkt_led_state;
    set_pkt_led(inst->pkt_led_state);

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

    return PHY_RADIO_CB_SUCCESS;
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
    return PHY_RADIO_CB_SUCCESS;
}

static int32_t phySyncStateCb(phyRadioInterface_t *interface, uint32_t sync_id, const phyRadioSyncState_t *sync_state) {
    myInstance_t * inst = CONTAINER_OF(interface, myInstance_t, phy_interface);
    int32_t retval = PHY_RADIO_SUCCESS;

    switch (sync_id) {
        case PHY_RADIO_SYNC_SENT:
           // Called if a central device has successfully sent a sync
           break;
        case PHY_RADIO_FIRST_SYNC:
           // Successfully synchronized with a central device
           inst->test_led_state = true;
           pico_set_led(inst->test_led_state);
           // The information regarding what slot to send on is provided in the sync state
           inst->phy_pkt.slot = sync_state->tx_slot_number;
           // Set peripheral mode
           retval = PHY_RADIO_CB_SET_PERIPHERAL;
           LOG("SYNCHRONIZED!\n");
           break;
        case PHY_RADIO_RE_SYNC:
           // Called every time a new sync is heard,
           inst->test_led_state = !inst->test_led_state;
           pico_set_led(inst->test_led_state);
           // and successfully synchronized to a central device
           break;
        case PHY_RADIO_CONFLICT_SYNC:
           LOG("SYNC CONFLICT!\n");
           // Called if a sync from another central device is heard
           break;
        case PHY_RADIO_SYNC_LOST:
           // Called no sync has been heard for X frames
           inst->test_led_state = false;
           pico_set_led(inst->test_led_state);
           // Return to scan mode
           retval = PHY_RADIO_CB_SET_SCAN;
           LOG("SYNC LOST!\n");
           break;
        case PHY_RADIO_RX_SLOT_START:
           // Called once every time a new RX slot starts
           break;
        case PHY_RADIO_TX_SLOT_START:
           // Called once every time a new TX slot starts
           break;
        case PHY_RADIO_SCAN_TIMEOUT:
           // Scan timeout, no device found
           int32_t res = phyRadioSetScanMode(inst, SCAN_TIMEOUT_MS);
           if (res != PHY_RADIO_SUCCESS) {
               return res;
           }
           LOG("SCAN TIMEOUT, NO DEVICE FOUND!\n");
           break;
        default:
            // We should never end up here!
            return PHY_RADIO_CB_ERROR_INVALID;
    }

    return retval;
}

int main()
{
    stdio_init_all();
    // Initialize the gpio module to make sure all modules can use it
    halGpioInit();
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
    if ((res = phyRadioSetScanMode(&my_instance.phy_radio_inst, SCAN_TIMEOUT_MS)) != PHY_RADIO_SUCCESS) {
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
