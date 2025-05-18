esphome nrf24l01 bluetooth mac scanner
made my swedude

use this get irk https://github.com/fryefryefrye/Decoding-Random-Bluetooth-Address

my nrf bt code are also based on above

esphome nrf24l01 mac scanner a faster updating mac scanner for presence detection of bluetooth signals from trackers or phones, uses IRK for phone mac adresses
this is faster then a wifi connected esp32, a ethernet connected esp32 are still better then this but this is a faster alternafive when you cant get ethernet.

the code as it is is setup for a esp8266 using the pins below, its made to use interrupts but interrupts on esp8266 are unstable so it polls the irq pins in loop instead.
#Spi pins esp8266

#GPIO14 SCLK

#GPIO12 MISO

#GPIO13 MOSI

#GPIO15 (CS)(CSN)

#other used pins

#GPIO5 CE

#GPIO4 IRQ

if you use esp32 interrupts are a better and faster and can be activated using


//#define use_interrupt  //might be unstable on single core mcu, works good on plain esp32 bit iffy on esp8266 suggest trying without interrupt first

recommended to run this code on a esp32 with a NRF24L01+PA+LNA module with antenna
