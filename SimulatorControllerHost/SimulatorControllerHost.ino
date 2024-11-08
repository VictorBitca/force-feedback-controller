// Simple test of USB Host Joystick
//
// This example is in the public domain

#include "USBHost_t36.h"
#include <SPI.h>

USBHost myusb;
USBHub hub1(myusb);
USBHIDParser hid1(myusb);
#define COUNT_JOYSTICKS 1

JoystickController joysticks[COUNT_JOYSTICKS] = {
  JoystickController(myusb)
};

int user_axis[64];
uint32_t buttons_prev = 0;

USBDriver *drivers[] = { &hub1, &joysticks[0], &hid1 };
#define CNT_DEVICES (sizeof(drivers) / sizeof(drivers[0]))
const char *driver_names[CNT_DEVICES] = { "Hub1", "joystick[0D]", "HID1" };
bool driver_active[CNT_DEVICES] = { false };

// Lets also look at HID Input devices
USBHIDInput *hiddrivers[] = { &joysticks[0] };
#define CNT_HIDDEVICES (sizeof(hiddrivers) / sizeof(hiddrivers[0]))
const char *hid_driver_names[CNT_DEVICES] = { "joystick[0H]" };
bool hid_driver_active[CNT_DEVICES] = { false };
bool show_changed_only = false;

uint16_t joystick_left_trigger_value[COUNT_JOYSTICKS] = { 0 };
uint16_t joystick_right_trigger_value[COUNT_JOYSTICKS] = { 0 };
uint64_t joystick_full_notify_mask = (uint64_t)-1;

// Serial data array to send reflecting the controller state
//-buttons-axis1-axis2-axis3-axis4-axis5-axis6
int32_t dataToSend[7] = { 0, 0, 0, 0, 0, 0, 0 };
//=============================================================================
// Setup
//=============================================================================
void setup() {
  Serial1.begin(9600);
  //  while (!Serial) ; // wait for Arduino Serial Monitor
  Serial.println("\n\nUSB Host Joystick Testing");
  myusb.begin();
}


//=============================================================================
// loop
//================================================================================
void loop() {
  myusb.Task();
  PrintDeviceListChanges();

  if (Serial.available()) {
    int ch = Serial.read();  // get the first char.
    while (Serial.read() != -1)
      ;
    if ((ch == 'b') || (ch == 'B')) {
      Serial.println("Only notify on Basic Axis changes");
      for (int joystick_index = 0; joystick_index < COUNT_JOYSTICKS; joystick_index++)
        joysticks[joystick_index].axisChangeNotifyMask(0x3ff);
    } else if ((ch == 'f') || (ch == 'F')) {
      Serial.println("Only notify on Full Axis changes");
      for (int joystick_index = 0; joystick_index < COUNT_JOYSTICKS; joystick_index++)
        joysticks[joystick_index].axisChangeNotifyMask(joystick_full_notify_mask);

    } else {
      if (show_changed_only) {
        show_changed_only = false;
        Serial.println("\n*** Show All fields mode ***");
      } else {
        show_changed_only = true;
        Serial.println("\n*** Show only changed fields mode ***");
      }
    }
  }

  for (int joystick_index = 0; joystick_index < COUNT_JOYSTICKS; joystick_index++) {
    if (joysticks[joystick_index].available()) {
      uint64_t axis_mask = joysticks[joystick_index].axisMask();
      uint64_t axis_changed_mask = joysticks[joystick_index].axisChangedMask();
      uint32_t buttons = joysticks[joystick_index].getButtons();
      Serial.printf("Joystick(%d): buttons = %u", joystick_index, buttons);
      // Set the buttons sum to the first value in the array struct
      dataToSend[0] = (int32_t)buttons;

      // Iterate through axes and set the data in te data to send
      for (uint8_t i = 0; axis_mask != 0; i++, axis_mask >>= 1) {
        if (axis_mask & 1) {
          int32_t axisValue = (int32_t)joysticks[joystick_index].getAxis(i);
          Serial.printf(" %d:%d", i, axisValue);
          // Set the axis value to the respective index in the array
          dataToSend[i + 1] = axisValue;
        }
      }

      uint16_t ltv;
      uint16_t rtv;
      uint16_t mapped_ltv;
      uint16_t mapped_rtv;

      switch (joysticks[joystick_index].joystickType()) {
        default:
          break;
        case JoystickController::XBOXONE:
          ltv = joysticks[joystick_index].getAxis(3);
          rtv = joysticks[joystick_index].getAxis(4);
          if ((ltv != joystick_left_trigger_value[joystick_index]) || (rtv != joystick_right_trigger_value[joystick_index])) {
            joystick_left_trigger_value[joystick_index] = ltv;
            joystick_right_trigger_value[joystick_index] = rtv;

            //mapped_ltv = map_uint8(ltv, 0, 1024, 0, 255);
            //mapped_rtv = map_uint8(rtv, 0, 1024, 0, 255);

            //joysticks[joystick_index].setRumble(mapped_ltv, mapped_rtv);
            //Serial.printf("Set Rumble %d %d", mapped_ltv, mapped_rtv);
          }
          break;
      }

      joysticks[joystick_index].joystickDataClear();

      // Print the dataToSend array to the Serial Monitor
      Serial1.printf("<%d,%d,%d,%d,%d,%d,%d>",dataToSend[0],dataToSend[1],dataToSend[2],dataToSend[3],dataToSend[4],dataToSend[5],dataToSend[6]);
      Serial.println();
    }
  }
}

uint8_t map_uint8(uint16_t x, uint16_t in_min, uint16_t in_max, uint8_t out_min, uint8_t out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//=============================================================================
// Show when devices are added or removed
//=============================================================================
void PrintDeviceListChanges() {
  for (uint8_t i = 0; i < CNT_DEVICES; i++) {
    if (*drivers[i] != driver_active[i]) {
      if (driver_active[i]) {
        Serial.printf("*** Device %s - disconnected ***\n", driver_names[i]);
        driver_active[i] = false;
      } else {
        Serial.printf("*** Device %s %x:%x - connected ***\n", driver_names[i], drivers[i]->idVendor(), drivers[i]->idProduct());
        driver_active[i] = true;

        const uint8_t *psz = drivers[i]->manufacturer();
        if (psz && *psz) Serial.printf("  manufacturer: %s\n", psz);
        psz = drivers[i]->product();
        if (psz && *psz) Serial.printf("  product: %s\n", psz);
        psz = drivers[i]->serialNumber();
        if (psz && *psz) Serial.printf("  Serial: %s\n", psz);
      }
    }
  }

  for (uint8_t i = 0; i < CNT_HIDDEVICES; i++) {
    if (*hiddrivers[i] != hid_driver_active[i]) {
      if (hid_driver_active[i]) {
        Serial.printf("*** HID Device %s - disconnected ***\n", hid_driver_names[i]);
        hid_driver_active[i] = false;
      } else {
        Serial.printf("*** HID Device %s %x:%x - connected ***\n", hid_driver_names[i], hiddrivers[i]->idVendor(), hiddrivers[i]->idProduct());
        hid_driver_active[i] = true;

        const uint8_t *psz = hiddrivers[i]->manufacturer();
        if (psz && *psz) Serial.printf("  manufacturer: %s\n", psz);
        psz = hiddrivers[i]->product();
        if (psz && *psz) Serial.printf("  product: %s\n", psz);
        psz = hiddrivers[i]->serialNumber();
        if (psz && *psz) Serial.printf("  Serial: %s\n", psz);
      }
    }
  }
}
