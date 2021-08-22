# cs2usb
convert cyberstick data to usb hid joystick<br />
for attiny2313 @20mhz<br />
<br />
depend on https://github.com/obdev/v-usb<br />
<br />
create vusb_joystick folder in Arduino/libraries/<br />
  mkdir Arduino/libraries/vusb_joystick<br />
<br />
copy inside of v-usb/usbdrv/ to Arduino/libraries/vusb_joystick/<br />
  cp v-usb/usbdrv/* Arduino/libraries/vusb_joystick<br />
<br />
rename Arduino/libraries/vusb_joystick/usbconfig-prototype.h to usbconfig-prototype.h.orig<br />
  mv Arduino/libraries/vusb_joystick/usbconfig-prototype.h Arduino/libraries/vusb_joystick/usbconfig-prototype.h.orig<br />
<br />
copy inside of vusb_joystick/ to Arduino/libraries/vusb_joystick/<br />
  cp vusb_joystick/* Arduino/libraries/vusb_joystick<br />
