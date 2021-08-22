
#ifndef __vusb_joystick_h__
#define __vusb_joystick_h__

//#include <avr/pgmspace.h>
//#include <avr/interrupt.h>
//#include <string.h>

#ifdef __cplusplus
extern "C"{
#endif

#include <usbdrv.h>

PROGMEM const char usbHidReportDescriptor[53] = { /* USB report descriptor, size must match usbconfig.h */
//PROGMEM const char usbHidReportDescriptor[58] = { /* USB report descriptor, size must match usbconfig.h */
//char ReportDescriptor[53] = {
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x04,                    // USAGE (Joystick)
    (char)0xa1, 0x01,                    // COLLECTION (Application)
    0x09, 0x01,                    //   USAGE (Pointer)
    (char)0xa1, 0x00,                    //   COLLECTION (Physical)
    0x05, 0x01,                    //     USAGE_PAGE (Generic Desktop)
    0x09, 0x30,                    //     USAGE (X)
    0x09, 0x31,                    //     USAGE (Y)
    0x09, 0x32,                    //     USAGE (Z)
    0x09, 0x35,                    //     USAGE (Rz)
    0x09, 0x38,                    //     USAGE (Wheel)
    0x09, 0x36,                    //     USAGE (Slider)

//    0x15, (char)0x7f,                    //     LOGICAL_MINIMUM (127)x
//    0x25, (char)0x81,              //     LOGICAL_MAXIMUM (-127)
//    0x15, (char)0x81,                    //     LOGICAL_MINIMUM (-127)x
//    0x25, (char)0x7f,              //     LOGICAL_MAXIMUM (127)

//    0x15, (char)0xff,                    //     LOGICAL_MINIMUM (-1)r
//    0x25, 0x01,                    //     LOGICAL_MAXIMUM (1)
//    0x15, 0x01,                    //     LOGICAL_MINIMUM (1)x
//    0x25, (char)0xff,                    //     LOGICAL_MAXIMUM (-1)

    0x15, 0x00,                    //     LOGICAL_MINIMUM (0)def r
    0x26, (char)0xff, 0x00,              //     LOGICAL_MAXIMUM (255)
//    0x16, (char)0xff, 0x00,              //     LOGICAL_MINIMUM (255)x
//    0x25, 0x00,                    //     LOGICAL_MAXIMUM (0)

//    0x35, 0x00,                    //     LOGICAL_MINIMUM (0)r
//    0x46, (char)0xff, 0x00,              //     LOGICAL_MAXIMUM (255)
//    0x36, (char)0xff, 0x00,              //     LOGICAL_MAXIMUM (255)r
//    0x45, 0x00,                    //     LOGICAL_MINIMUM (0)

    0x75, 0x08,                    //     REPORT_SIZE (8)
    (char)0x95, 0x06,                    //     REPORT_COUNT (6)
    (char)0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0x05, 0x09,                    //     USAGE_PAGE (Button)
    0x19, 0x01,                    //     USAGE_MINIMUM (Button 1)
    0x29, 0x10,                    //     USAGE_MAXIMUM (Button 16)
    0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //     LOGICAL_MAXIMUM (1)
    0x75, 0x01,                    //     REPORT_SIZE (1)
    (char)0x95, 0x10,                    //     REPORT_COUNT (16)
    (char)0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    (char)0xc0,                          //   END_COLLECTION
    (char)0xc0                           // END_COLLECTION
//};
};

#define HID_REPORT_DATA_SIZE    8
#define HID_REPORT_DATA         hid_report_data
byte hid_report_data[HID_REPORT_DATA_SIZE];

byte hid_idle_rate = 0;
//bool hid_reportdata_dirty = false;
//unsigned long hid_reportdata_updated_ms = 0;

//#define hid_touch_reportdata()      ( hid_reportdata_dirty = true )


USB_PUBLIC usbMsgLen_t usbFunctionSetup(uchar data[8]) {
  usbRequest_t *rq = (usbRequest_t *)data;

  usbMsgPtr = (usbMsgPtr_t)HID_REPORT_DATA;
  if ((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS) { // class request type
    if (rq->bRequest == USBRQ_HID_GET_REPORT){ // wValue: ReportType (highbyte), ReportID (lowbyte)
      // we only have one report type, so don't look at wValue
      return HID_REPORT_DATA_SIZE;
    } else if (rq->bRequest == USBRQ_HID_GET_IDLE) {
      usbMsgPtr = (usbMsgPtr_t)&hid_idle_rate;
      return 1;
    } else if (rq->bRequest == USBRQ_HID_SET_IDLE) {
      hid_idle_rate = rq->wValue.bytes[1];
    }
  } else {
    /* no vendor specific requests implemented */
  }

  return 0;
}

#ifdef __cplusplus
} // extern "C"
#endif


#endif // __vusb_joystick_h__