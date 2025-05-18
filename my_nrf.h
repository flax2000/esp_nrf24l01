//made my swedude

#include "esphome.h"
#include "irk.h"


// use to get irk https://github.com/fryefryefrye/Decoding-Random-Bluetooth-Address
// my nrf bt code are also based on above


#if defined(ESP8266)
// interrupt handler and related code must be in RAM on ESP8266,
// according to issue #46.
#define RECEIVE_ATTR IRAM_ATTR
#define VAR_ISR_ATTR
#elif defined(ESP32)
#define RECEIVE_ATTR IRAM_ATTR
#define VAR_ISR_ATTR DRAM_ATTR
#else
#define RECEIVE_ATTR
#define VAR_ISR_ATTR
#endif


//#define use_interrupt  //might be unstable on single core mcu, works good on plain esp32 bit iffy on esp8266 suggest trying without interrupt first

#if defined(ESP8266)
// You can change the define pin.
#define CE 5
// CE_BIT:   Digital Input     Chip Enable Activates RX or TX mode

#define CSN 15
// CSN BIT:  Digital Input     SPI Chip Select

#define IRQ 4
// IRQ BIT:  Digital Output    Maskable interrupt pin

#endif

#if defined(ESP32)

// You can change the define pin.
#define CE 14
// CE_BIT:   Digital Input     Chip Enable Activates RX or TX mode

#define CSN 2
// CSN BIT:  Digital Input     SPI Chip Select

#define IRQ 17
// IRQ BIT:  Digital Output    Maskable interrupt pin
#endif

/*
#if defined(ESP32)  //S2
// You can change the define pin.
#define CE 38
// CE_BIT:   Digital Input     Chip Enable Activates RX or TX mode

#define CSN 33
// CSN BIT:  Digital Input     SPI Chip Select

#define IRQ 37
// IRQ BIT:  Digital Output    Maskable interrupt pin
#endif

*/
//*********************************************

const byte chRf[] = { 2, 26, 80 };
const byte chLe[] = { 37, 38, 39 };
byte channel = 2;  //using single channel to receive
//***************************************************
#define TX_ADR_WIDTH 4    // 5 uint8_ts TX(RX) address width
#define TX_PLOAD_WIDTH 8  // 32 uint8_ts TX payload
#define MAC_LEN 6
uint8_t TX_ADDRESS[TX_ADR_WIDTH] = {
  0x71, 0x91, 0x7d, 0x6b
};  // Define a static TX address


#include "nrfbt_control.h"







#define IRK_LIST_NUMBER 1
uint8_t irk[IRK_LIST_NUMBER][ESP_BT_OCTET16_LEN] = {
  //IRK of A
  { 0xC1, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11 }
};
#if defined(ESP8266)
#define rx_bufferts 10
#else
#define rx_bufferts 30
#endif


volatile VAR_ISR_ATTR uint8_t rx_buff[rx_bufferts][(TX_PLOAD_WIDTH + 1)] = { 0 };
volatile VAR_ISR_ATTR uint8_t rx_bufferts_full = 0;

unsigned long bt_phone_counter = 0;
unsigned long bt_tracker_counter = 0;
volatile VAR_ISR_ATTR unsigned long bt_all_counter = 0;
unsigned long millis_minne_phone = 0;
unsigned long millis_minne_tracker = 0;
unsigned long millis_minne_all = 0;

int have_bt_phone = 0;
int have_bt_tracker = 0;
int start_delay = 1;


void RECEIVE_ATTR BleDataCheckTask() {
  //uint8_t status = SPI_Read(STATUS);  // read register STATUS's value
  //if (status & RX_DR)                       // if receive data ready (TX_DS) interrupt
  //{
  static uint8_t ii = 0;
  if (ii == rx_bufferts) {
    ii = 0;
  }
  if (rx_buff[ii][TX_PLOAD_WIDTH] == 1) {
    rx_bufferts_full = 1;
    SPI_RW_Reg(FLUSH_RX, 0);
    SPI_RW_Reg(WRITE_REG + STATUS, 0xff);  // clear RX_DR or TX_DS or MAX_RT interrupt flag
  } else {



    SPI_Read_Buf(RD_RX_PLOAD, rx_buff[ii], TX_PLOAD_WIDTH);  // read playload to rx_buf
    SPI_RW_Reg(FLUSH_RX, 0);

    rx_buff[ii][TX_PLOAD_WIDTH] = 1;
    ii++;
    bt_all_counter++;

    SPI_RW_Reg(WRITE_REG + STATUS, 0xff);  // clear RX_DR or TX_DS or MAX_RT interrupt flag
  }
  //}
}


#if defined(use_interrupt)

void RECEIVE_ATTR ext_int_1() {
  BleDataCheckTask();
}
#endif
class MyCustomSensor : public PollingComponent {
public:
  Sensor *bt_phone_counter_ = new Sensor();
  Sensor *bt_all_counter_ = new Sensor();
  Sensor *bt_tracker_counter_ = new Sensor();

  MyCustomSensor()
    : PollingComponent(1000) {}
  float get_setup_priority() const override {
    return esphome::setup_priority::AFTER_WIFI;
  }


  //-----------------------------------------------------------------------------------------


  void setup() override {




    pinMode(CE, OUTPUT);
    pinMode(CSN, OUTPUT);
    pinMode(IRQ, INPUT);
  }

  void update() override {
    bt_all_counter_->publish_state(bt_all_counter);
    if (bt_all_counter == 0 && !start_delay) {
      ESP_LOGE("rf24", "bt counter 0");
      SPI_RW_Reg(FLUSH_RX, 0);
      SPI_RW_Reg(WRITE_REG + STATUS, 0xff);
    }

    if (rx_bufferts_full) {
      ESP_LOGE("rf24", "rx_bufferts_full");
      rx_bufferts_full = 0;
    }


    bt_all_counter = 0;
  }
  void loop() override

  {


    if (start_delay) {

      if (millis() > 20000) {

        SPI.begin();
        delay(50);
        init_io();  // Initialize IO port
        uint8_t sstatus = SPI_Read(STATUS);
        RX_Mode();  // set RX mode

        SPI_RW_Reg(FLUSH_RX, 0);
        SPI_RW_Reg(WRITE_REG + STATUS, 0xff);

#if defined(use_interrupt)
        // let IRQ pin only trigger on "data ready" event in RX mode
        attachInterrupt(IRQ, ext_int_1, FALLING);
#endif
        start_delay = 0;
      }


    } else {

#if not defined(use_interrupt)

        if (!digitalRead(IRQ)) BleDataCheckTask();
      
#endif


      uint8_t ii = 0;

      while (ii < rx_bufferts) {

        /*
      for(int i=0; i<TX_PLOAD_WIDTH+1; i++)
      {
          Serial.print(" ");
          Serial.print(rx_buff[ii][i],HEX);                              // print rx_buf
      }
      Serial.println(" ");

*/
        if (rx_buff[ii][TX_PLOAD_WIDTH] == 1) {


          uint8_t i;  //, dataLen = TX_PLOAD_WIDTH;
          uint8_t rx_buff_temp[TX_PLOAD_WIDTH];
          std::copy(rx_buff[ii], rx_buff[ii] + TX_PLOAD_WIDTH, rx_buff_temp);
          rx_buff[ii][TX_PLOAD_WIDTH] = 0;
          //reversing the bits of the complete packet
          for (i = 0; i < TX_PLOAD_WIDTH; i++) {
            rx_buff_temp[i] = reverseBits(rx_buff_temp[i]);
          }
          //de-whiten the packet using the same polynomial
          bleWhiten(rx_buff_temp, TX_PLOAD_WIDTH, bleWhitenStart(chLe[channel]));
          //reversing bits of the crc
          //for (i = 0; i < 3; i++, dataLen++) {
          //  rx_buff[ii][dataLen] = reverseBits(rx_buff[ii][dataLen]);
          //}


          uint8_t AdMac[MAC_LEN];

          //Get the MAC address. Reverse order in BT payload.
          for (byte i = 0; i < MAC_LEN; i++) {
            AdMac[MAC_LEN - 1 - i] = rx_buff_temp[i + 2];
          }

          if (AdMac[0] == 0x38 && AdMac[1] == 0x1F && AdMac[2] == 0x8D && AdMac[3] == 0xB9 && AdMac[4] == 0x57 && AdMac[5] == 0xA2) {
            have_bt_tracker = 1;
          }


          for (byte i = 0; i < IRK_LIST_NUMBER; i++) {
            //Check with all IRK we got one by one.
            if (btm_ble_addr_resolvable(AdMac, irk[i])) {
              have_bt_phone = 1;
            }
          }
        }
        ii++;
      }







      if (have_bt_tracker) {
        if (millis() - millis_minne_tracker > 900) {
          millis_minne_tracker = millis();
          bt_tracker_counter++;
          bt_tracker_counter_->publish_state(bt_tracker_counter);
        }
        have_bt_tracker = 0;
      }


      if (have_bt_phone) {
        if (millis() - millis_minne_phone > 900) {
          millis_minne_phone = millis();
          bt_phone_counter++;
          bt_phone_counter_->publish_state(bt_phone_counter);
        }
        have_bt_phone = 0;
      }
    }
  }
};
