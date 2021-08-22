
//convert cyberstick data to usb hid joystick
//for attiny2313 @20mhz
//
//depend on https://github.com/obdev/v-usb
//
//create vusb_joystick folder in Arduino/libraries/
//  mkdir Arduino/libraries/vusb_joystick
//
//copy inside of v-usb/usbdrv/ to Arduino/libraries/vusb_joystick/
//  cp v-usb/usbdrv/* Arduino/libraries/vusb_joystick
//
//rename Arduino/libraries/vusb_joystick/usbconfig-prototype.h to usbconfig-prototype.h.orig
//  mv Arduino/libraries/vusb_joystick/usbconfig-prototype.h Arduino/libraries/vusb_joystick/usbconfig-prototype.h.orig
//
//copy inside of vusb_joystick/ to Arduino/libraries/vusb_joystick/
//  cp vusb_joystick/* Arduino/libraries/vusb_joystick



#include "vusb_joystick.h"

#include <util/delay.h>     /* for _delay_ms() */

//#ifdef __cplusplus
//extern "C"{
//#endif
//
//#include <usbdrv.h>
//
//#ifdef __cplusplus
//} // extern "C"
//#endif

//#define DISABLE_FETCH_STICKDATA

#define DISABLE_ANALOG_HEARTBEAT
#ifndef DISABLE_ANALOG_HEARTBEAT
#define ENABLE_ANALOG_HEARTBEAT
#endif
#define ENABLE_DIGITAL_SELECT_START_BUTTON
//#define ENABLE_DIGITAL_AXIS_TO_BUTTON

#define ENABLE_INVERT_AXIS

//data transfer speed
#define CS_DATASPEED_FULL   1
#define CS_DATASPEED_HALF   2
#define CS_DATASPEED_3RD    4
#define CS_DATASPEED_LOW    8
//byte cs_dataspeed = CS_DATASPEED_FULL;
#define cs_dataspeed       CS_DATASPEED_FULL

//data port
#define CS_DATAPORT       PORTB
#define CS_DATAPORT_IN    PINB
#define CS_DATAPORT_DR    DDRB
#define CS_BIT0_PIN       9
#define READ_DATAPORT()   ( ~CS_DATAPORT_IN )
#define READ_DATAPORT_PIN( n )    bitRead( PINB, n - CS_BIT0_PIN )

//heartbeat led
#define CS_LED_PIN    9
//#define LED_OFF()     bitSet( CS_DATAPORT, CS_LED_PIN - CS_BIT0_PIN )
//#define LED_ON()      bitClear( CS_DATAPORT, CS_LED_PIN - CS_BIT0_PIN )
//#define LED_TURN()    ( CS_DATAPORT ^= bit( CS_LED_PIN - CS_BIT0_PIN ) )
#define LED_OFF()     bitClear( CS_DATAPORT_DR, CS_LED_PIN - CS_BIT0_PIN )
#define LED_ON()      bitSet( CS_DATAPORT_DR, CS_LED_PIN - CS_BIT0_PIN )
#define LED_TURN()    ( CS_DATAPORT_DR ^= bit( CS_LED_PIN - CS_BIT0_PIN ) )

#define LED_TIMER_OFF()     do{ bitClear( TIMSK, OCIE1A ); bitClear( TIMSK, OCIE1B ); } while( 0 )
#define LED_TIMER_ON()      do{ bitSet( TIMSK, OCIE1A ); bitSet( TIMSK, OCIE1B ); } while( 0 )

#define LED_VOLUME          OCR1B
#define LED_VOLUME_MIN      0
#define LED_VOLUME_MAX      0xff
#define LED_VOLUME_TOP      0x100
byte led_count = 0;

//analog mode
#define CS_REQ_PIN    10    // dsub 8 pin
//#define ASSERT_REQ()    digitalWrite( CS_REQ_PIN, LOW )
//#define DEASSERT_REQ()  digitalWrite( CS_REQ_PIN, HIGH )
#define ASSERT_REQ()    bitClear( CS_DATAPORT, CS_REQ_PIN - CS_BIT0_PIN )
#define DEASSERT_REQ()  bitSet( CS_DATAPORT, CS_REQ_PIN - CS_BIT0_PIN )
#define CS_ACK_PIN    11    // dsub 7 pin
#define READ_ACK_PIN()  bitRead( PINB, CS_ACK_PIN - CS_BIT0_PIN )
#define CS_CLK_PIN    12    // dsub 6 pin
#define READ_CLK_PIN()  bitRead( PINB, CS_CLK_PIN - CS_BIT0_PIN )
#define CS_D0_PIN     13    // dsub 1 pin
#define CS_D1_PIN     14    // dsub 2 pin
#define CS_D2_PIN     15    // dsub 3 pin
#define CS_D3_PIN     16    // dsub 4 pin
#define READ_DATA_NIBBLE()    ( ( READ_DATAPORT() >> 4 ) & 0x0f )

//nibble0:  A B C D
#define CS_NIBBLE_ABCD    0
#define CS_NIBBLE0_A        0b1000
#define CS_NIBBLE0_B        0b0100
#define CS_NIBBLE0_C        0b0010
#define CS_NIBBLE0_D        0b0001
//nibble1: E1E2 F G
#define CS_NIBBLE_E1E2FG  1
#define CS_NIBBLE1_E1       0b1000
#define CS_NIBBLE1_E2       0b0100
#define CS_NIBBLE1_F        0b0010
#define CS_NIBBLE1_G        0b0001
//nibble2: <HINIBBLE0>
#define CS_NIBBLE_Y0_HI   2
//nibble3: <HINIBBLE1>
#define CS_NIBBLE_X0_HI   3
//nibble4: <HINIBBLE2>
#define CS_NIBBLE_Y1_HI   4
//nibble5: <HINIBBLE3>
#define CS_NIBBLE_X1_HI   5
//nibble6: <LONIBBLE0>
#define CS_NIBBLE_Y0_LOW  6
//nibble7: <LONIBBLE1>
#define CS_NIBBLE_X0_LOW  7
//nibble8: <LONIBBLE2>
#define CS_NIBBLE_Y1_LOW  8
//nibble9: <LONIBBLE3>
#define CS_NIBBLE_X1_LOW  9
//nibble10: A B A'B'
#define CS_NIBBLE_AABB    10
#define CS_NIBBLE10_A       0b1000
#define CS_NIBBLE10_B       0b0100
#define CS_NIBBLE10_AA      0b0010
#define CS_NIBBLE10_BB      0b0001
//for digital mode axis
#define CS_NIBBLE10_RIGHT   0b10000000
#define CS_NIBBLE10_LEFT    0b01000000
#define CS_NIBBLE10_DOWN    0b00100000
#define CS_NIBBLE10_UP      0b00010000
#define CS_NIBBLE10_TDOWN   CS_NIBBLE10_AA
#define CS_NIBBLE10_TUP     CS_NIBBLE10_BB
//nibble11: <RESERVED>
#define CS_NIBBLE_RESERVED 11

//digital mode
//common high: r l d u a b
//common low:  d ctdtue1e2
#define CS_COMMON_PIN        CS_REQ_PIN
#define SET_COMMON_PIN_HIGH()   bitSet( CS_DATAPORT, CS_REQ_PIN - CS_BIT0_PIN )
#define SET_COMMON_PIN_LOW()    bitClear( CS_DATAPORT, CS_REQ_PIN - CS_BIT0_PIN )
#define CS_B_E2_PIN          CS_ACK_PIN
#define CS_A_E1_PIN          CS_CLK_PIN
#define CS_UP_TUP_PIN        CS_D0_PIN
#define CS_DOWN_TDOWN_PIN    CS_D1_PIN
#define CS_LEFT_C_PIN        CS_D2_PIN
#define CS_RIGHT_D_PIN       CS_D3_PIN
//in XE-1AP: 
//  CS_RIGHT_D_PIN & CS_LEFT_C_PIN? start: 0;
//  CS_DOWN_TDOWN_PIN & CS_UP_TUP_PIN? select: 0;

byte stickdata_raw[12];


//X Y XROT YROT ZROT SLIDER btn_lobyte btn_hibyte
#define USB_HID_X             0
#define USB_HID_Y             1
#define USB_HID_XROT          2
#define USB_HID_YROT          3
#define USB_HID_ZROT          4
#define USB_HID_SLIDER        5
#define USB_HID_BTN_LOBYTE    6
#define USB_HID_BTN_HIBYTE    7
//byte stickdata[8];
#define stickdata         HID_REPORT_DATA


//#define CALC_TIMEOUT( dataspeed )     ( dataspeed )
//bool is_timeout( const unsigned long t ){
////  const unsigned long d = t & 0x80000000UL;
////  return ( t - d + CALC_TIMEOUT( cs_dataspeed ) ) < ( millis() - d );
//  return false;
//}
#define is_timeout( t )   ( !( ++t ) )
//return true: probably in digital mode or unconnected.
bool fetch_stickdata_analog( void ){
//  const unsigned long t = millis();
  byte t;

//assert req
  ASSERT_REQ();
  
  byte i;
  for( i = 0; i < 12; ++i ){
    t = 0;
//wait ack falling edge & clk: lhlhlhlhlhlh
    while( READ_ACK_PIN() || ( READ_CLK_PIN() == !( i & 0x01 ) ) ){
      if( is_timeout( t ) ){ DEASSERT_REQ(); return true; }
    }

    if( i == cs_dataspeed ){
//deassert req
      DEASSERT_REQ();
    }

//read nibble
    stickdata_raw[i] = READ_DATA_NIBBLE();
  }

  return false;
}

#define READ_PIN( b, n )    bitRead( b, n - CS_BIT0_PIN )
#define MASK_RL_PIN  ( bit( CS_RIGHT_D_PIN - CS_BIT0_PIN ) | bit( CS_LEFT_C_PIN - CS_BIT0_PIN ) )
#define MASK_DU_PIN  ( bit( CS_DOWN_TDOWN_PIN - CS_BIT0_PIN ) | bit( CS_UP_TUP_PIN - CS_BIT0_PIN ) )
//common high: d ctdtue1e2
//common low:  r l d u a b
void fetch_stickdata_digital( void ){
  byte b;

//  byte i;
//  for( i = 0; i < sizeof stickdata_raw; ++i ){
//    stickdata_raw[i] = 0x0f;
//  }
  stickdata_raw[CS_NIBBLE_RESERVED] = 0x00;

//  SET_COMMON_PIN_HIGH();
//  delayMicroseconds( 1 );
//  _delay_us(1);
  b = READ_DATAPORT();
  stickdata_raw[CS_NIBBLE_E1E2FG] = READ_PIN( b, CS_A_E1_PIN )? CS_NIBBLE1_E1: 0;
  stickdata_raw[CS_NIBBLE_E1E2FG] |= READ_PIN( b, CS_B_E2_PIN )? CS_NIBBLE1_E2: 0;
#ifdef ENABLE_DIGITAL_SELECT_START_BUTTON
//  if( bitRead( send_datamode, SEND_DATAMODE_XE_1AP ) ){
    const byte mask_rl = ( ( b & MASK_RL_PIN ) == MASK_RL_PIN )? MASK_RL_PIN: 0;
    const byte mask_du = ( ( b & MASK_DU_PIN ) == MASK_DU_PIN )? MASK_DU_PIN: 0;
    stickdata_raw[CS_NIBBLE_E1E2FG] |= mask_rl? CS_NIBBLE1_F: 0;
    stickdata_raw[CS_NIBBLE_E1E2FG] |= mask_du? CS_NIBBLE1_G: 0;
    b &= ~( mask_rl | mask_du );
//  }
#else
    const byte mask_rl = 0;
    const byte mask_du = 0;
#endif
//  stickdata_raw[CS_NIBBLE_ABCD] = ( ( b >> 7 ) | ( ( b & 0x40 ) >> 5 ) ) & 0x03;
//  stickdata_raw[CS_NIBBLE_E1E2FG] = b & 0x0c;
  stickdata_raw[CS_NIBBLE_ABCD] = READ_PIN( b, CS_LEFT_C_PIN )? CS_NIBBLE0_C: 0;
  stickdata_raw[CS_NIBBLE_ABCD] |= READ_PIN( b, CS_RIGHT_D_PIN )? CS_NIBBLE0_D: 0;
#ifdef ENABLE_DIGITAL_AXIS_TO_BUTTON
//  stickdata_raw[CS_NIBBLE_AABB] = ( b >> 2 ) & 0x0c;
  stickdata_raw[CS_NIBBLE_AABB] = READ_PIN( b, CS_DOWN_TDOWN_PIN )? CS_NIBBLE10_TDOWN: 0;
  stickdata_raw[CS_NIBBLE_AABB] |= READ_PIN( b, CS_UP_TUP_PIN )? CS_NIBBLE10_TUP: 0;
#else
  stickdata_raw[CS_NIBBLE_Y1_LOW] = READ_PIN( b, CS_DOWN_TDOWN_PIN )? 0: 0x0f;
  stickdata_raw[CS_NIBBLE_Y1_HI] = READ_PIN( b, CS_DOWN_TDOWN_PIN )? 0: READ_PIN( b, CS_UP_TUP_PIN )? 0x0f: 0x07;
#endif

  SET_COMMON_PIN_LOW();
//  delayMicroseconds( 1 );
  _delay_us(1);
  b = READ_DATAPORT();
  b &= ~( mask_rl | mask_du );
//  stickdata_raw[CS_NIBBLE_ABCD] |= b & 0x0c;
  stickdata_raw[CS_NIBBLE_ABCD] |= READ_PIN( b, CS_A_E1_PIN )? CS_NIBBLE0_A: 0;
  stickdata_raw[CS_NIBBLE_ABCD] |= READ_PIN( b, CS_B_E2_PIN )? CS_NIBBLE0_B: 0;
#ifdef ENABLE_DIGITAL_AXIS_TO_BUTTON
//  stickdata_raw[CS_NIBBLE_AABB] |= b & 0xf0;
  stickdata_raw[CS_NIBBLE_AABB] |= READ_PIN( b, CS_RIGHT_D_PIN )? CS_NIBBLE10_RIGHT: 0;
  stickdata_raw[CS_NIBBLE_AABB] |= READ_PIN( b, CS_LEFT_C_PIN )? CS_NIBBLE10_LEFT: 0;
  stickdata_raw[CS_NIBBLE_AABB] |= READ_PIN( b, CS_DOWN_TDOWN_PIN )? CS_NIBBLE10_DOWN: 0;
  stickdata_raw[CS_NIBBLE_AABB] |= READ_PIN( b, CS_UP_TUP_PIN )? CS_NIBBLE10_UP: 0;
#else
  stickdata_raw[CS_NIBBLE_Y0_LOW] = READ_PIN( b, CS_DOWN_TDOWN_PIN )? 0: 0x0f;
  stickdata_raw[CS_NIBBLE_Y0_HI] = READ_PIN( b, CS_DOWN_TDOWN_PIN )? 0: READ_PIN( b, CS_UP_TUP_PIN )? 0x0f: 0x07;
  stickdata_raw[CS_NIBBLE_X0_LOW] = READ_PIN( b, CS_RIGHT_D_PIN )? 0: 0x0f;
  stickdata_raw[CS_NIBBLE_X0_HI] = READ_PIN( b, CS_RIGHT_D_PIN )? 0: READ_PIN( b, CS_LEFT_C_PIN )? 0x0f: 0x07;
#endif

  SET_COMMON_PIN_HIGH();
}
//X Y XROT YROT ZROT SLIDER btn_lobyte btn_hibyte
void conv_stickdata( void ){
  stickdata[USB_HID_Y] = stickdata_raw[CS_NIBBLE_Y0_HI] << 4;
  stickdata[USB_HID_Y] |= stickdata_raw[CS_NIBBLE_Y0_LOW];
  stickdata[USB_HID_X] = stickdata_raw[CS_NIBBLE_X0_HI] << 4;
  stickdata[USB_HID_X] |= stickdata_raw[CS_NIBBLE_X0_LOW];
  
  stickdata[USB_HID_YROT] = stickdata_raw[CS_NIBBLE_Y1_HI] << 4;
  stickdata[USB_HID_YROT] |= stickdata_raw[CS_NIBBLE_Y1_LOW];
//  stickdata[USB_HID_XROT] = stickdata_raw[CS_NIBBLE_X1_HI] << 4;
//  stickdata[USB_HID_XROT] |= stickdata_raw[CS_NIBBLE_X1_LOW];
//  
//  stickdata[USB_HID_ZROT] = 0x00;
//  stickdata[USB_HID_SLIDER] = 0x00;
  
#ifdef ENABLE_INVERT_AXIS
  stickdata[USB_HID_Y] = 0xff - stickdata[USB_HID_Y];
  stickdata[USB_HID_X] = 0xff - stickdata[USB_HID_X];
  stickdata[USB_HID_YROT] = 0xff - stickdata[USB_HID_YROT];
#endif
  
  stickdata[USB_HID_BTN_HIBYTE] = stickdata_raw[CS_NIBBLE_ABCD] << 4;
  stickdata[USB_HID_BTN_HIBYTE] |= stickdata_raw[CS_NIBBLE_E1E2FG];
  stickdata[USB_HID_BTN_LOBYTE] = stickdata_raw[CS_NIBBLE_AABB];
}


void setup() {
  // put your setup code here, to run once:
//init usb
    noInterrupts();
    usbInit();
    usbDeviceDisconnect();
//    delay( 250 );
    _delay_ms(250);
    usbDeviceConnect();
    interrupts();

//disable Analog Comparator: save energy
  bitSet( ACSR, ACD );

//in:
//  pinMode( CS_ACK_PIN, INPUT_PULLUP );
//  pinMode( CS_CLK_PIN, INPUT_PULLUP );
//  pinMode( CS_D0_PIN, INPUT_PULLUP );
//  pinMode( CS_D1_PIN, INPUT_PULLUP );
//  pinMode( CS_D2_PIN, INPUT_PULLUP );
//  pinMode( CS_D3_PIN, INPUT_PULLUP );
//  CS_DATAPORT = 0xff;
  CS_DATAPORT = 0xfe;

//out:
//  DEASSERT_REQ();
//  pinMode( CS_REQ_PIN, OUTPUT );
//  pinMode( CS_LED_PIN, OUTPUT );
  bitSet( CS_DATAPORT_DR, CS_REQ_PIN - CS_BIT0_PIN );
//  bitSet( CS_DATAPORT_DR, CS_LED_PIN - CS_BIT0_PIN );

#ifdef ENABLE_ANALOG_HEARTBEAT
//init timer1 for heartbeat led:
  OCR1A = LED_VOLUME_MAX;
  LED_VOLUME = LED_VOLUME_MIN;
  bitSet( TCCR1B, WGM12 );
//  bitSet( TCCR1B, CS11 );   // 1Mhz @8Mhz
//  bitSet( TCCR1B, CS10 );   // 125Khz‬ @8Mhz
//  bitSet( TCCR1B, CS11 );   // 125Khz‬ @8Mhz
  bitSet( TCCR1B, CS12 );   // 31.25Khz‬ @8Mhz
  LED_TIMER_ON();
#endif
}

void loop() {
  // put your main code here, to run repeatedly:

  usbPoll();

//  if( hid_is_update() && usbInterruptIsReady() ){
//    usbSetInterrupt( HID_REPORT_DATA, HID_REPORT_DATA_SIZE );
//    hid_reportdata_updated_ms = millis();
//    hid_reportdata_dirty = false;
//  }

#ifndef DISABLE_FETCH_STICKDATA
    bool f_digital;
    byte i;
    for( i = 0; ( f_digital = fetch_stickdata_analog() ) && ( i < 2 ); ++i ){
  //wait 1ms
//      delay( cs_dataspeed );
      _delay_ms( cs_dataspeed );
    }
    if( f_digital ){
//    if( ( f_digital = fetch_stickdata_analog() ) ){
      fetch_stickdata_digital();
    }
    conv_stickdata();
#endif

#ifdef ENABLE_ANALOG_HEARTBEAT
  //heartbeat
//      if( ( ++led_count & 0x1f ) && f_digital ){
      if( ( ++led_count & ( 0x1f << 2 ) ) && f_digital ){
        LED_VOLUME = LED_VOLUME_MIN;
      } else {
//        LED_VOLUME = LED_VOLUME? LED_VOLUME >> 1: LED_VOLUME_TOP;
//        LED_VOLUME = LED_VOLUME? LED_VOLUME - 8: LED_VOLUME_TOP;
        LED_VOLUME = LED_VOLUME? LED_VOLUME - 2: LED_VOLUME_TOP;
      }
#else
  if( ++led_count == 64 ){
    led_count = 0;
    LED_TURN();
  }
#endif

  usbPoll();
  if(usbInterruptIsReady()){
    usbSetInterrupt(HID_REPORT_DATA, sizeof(HID_REPORT_DATA));
  }

//  usbPoll();
  _delay_ms(10);
//  delay( 10 );
}

#ifdef ENABLE_ANALOG_HEARTBEAT
ISR(TIMER1_COMPA_vect) {
  LED_ON();
}
ISR(TIMER1_COMPB_vect) {
  LED_OFF();
}
#endif
