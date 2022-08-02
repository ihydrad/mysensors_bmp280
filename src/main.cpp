#include <Arduino.h>
#include <Adafruit_BMP280.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <SPI.h>
#include <RF24.h>
#include <Wire.h>

//#define DEBUG

#ifdef DEBUG
  #include <printf.h>
#endif

#define SBR(port, bit)        port |= (1<<bit)
#define CBR(port, bit)        port &= (~(1<<bit))
#define INV(port, bit)        port ^= (1<<bit)
#define SBRC(port, bit)      ((port & (1<<bit)) == 0)
#define SBRS(port, bit)      ((port & (1<<bit)) != 0)

#define TR_NUM               1
#define TR_DELMIN            30
#define ADCGND               A3
#define ADCBATPIN            A2
#define ADCVCC               A1
#define DIV_ADC              2.15
#define PayloadSize          5
#define Channel              121
#define BMP_ADR              0x76

Adafruit_BMP280 bmp;
RF24            radio(9, 10); //4 for CE and 15 for CSN

uint8_t data[PayloadSize],
        tx_adrr[]="1Node",
        wdt_cnt = 7*TR_DELMIN,
        mcur;

void enWDT(void);
void disWDT();
void initNRF(void);
void sleep(void);
void wakeup(void);
void prepData(void);

ISR(WDT_vect){
  wdt_cnt++;
}

void setup() {  
  mcur = MCUSR;
  MCUSR = 0;
  CLKPR=1<<CLKPCE;
  CLKPR=(0<<CLKPCE)|(0<<CLKPS3)|(0<<CLKPS2)|(1<<CLKPS1)|(1<<CLKPS0);  
  initNRF();
  analogReference(INTERNAL);
  ADCSRA = 0;
  ACSR = 0x80;
  #ifdef DEBUG
    UCSR0A=0x02;
    UCSR0B=0x08;
    UCSR0C=0x06;
    UBRR0H=0x00;
    UBRR0L=0x0C;
    printf_begin();
    radio.printDetails();
    Serial.print("Last reset source: ");    
    Serial.println(MCUSR);
    MCUSR = 0x00;
    delay(2000);
  #endif  
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep();
}

void initNRF(){
  //Конфигурируем SPI************************
  //SPI.setHwCs(true);
  SPI.begin();
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  //*****************************************
  radio.begin();                                        // Инициируем работу nRF24L01+
  radio.setChannel      (125);                      // Указываем канал приёма данных (от 0 до 127), 5 - значит приём данных осуществляется на частоте 2,405 ГГц (на одном канале может быть только 1 приёмник и до 6 передатчиков)
  radio.setDataRate     (RF24_2MBPS);                 // Указываем скорость передачи данных (RF24_250KBPS, RF24_1MBPS, RF24_2MBPS), RF24_1MBPS - 1Мбит/сек
  radio.setPALevel      (RF24_PA_MAX);                 // Указываем мощность передатчика (RF24_PA_MIN=-18dBm, RF24_PA_LOW=-12dBm, RF24_PA_HIGH=-6dBm, RF24_PA_MAX=0dBm)
  radio.disableDynamicPayloads();
  radio.setPayloadSize(5);
  radio.setAutoAck(false);
  radio.setRetries(0, 0);
  radio.setAddressWidth(5);
  radio.openWritingPipe (tx_adrr);
  radio.setCRCLength    (RF24_CRC_8);
  radio.stopListening();
}

void enWDT(){
  cli();
  wdt_reset();
  MCUSR &= ~(1<<WDRF);
  WDTCSR |= (1<<WDCE) | (1<<WDE); //WDT ISR 8sec
  WDTCSR = (1<<WDP3)| (1<<WDP0);
  WDTCSR |= bit (WDIE);
  sei();
  #ifdef DEBUG
    Serial.println("WDT_EN");
  #endif
}

void disWDT(){
  cli();
  wdt_reset();
  MCUSR &= ~(1<<WDRF);
  WDTCSR |= (1<<WDCE) | (1<<WDE); //WDT ISR 8sec
  WDTCSR = (1<<WDP3)| (1<<WDP0);
  WDTCSR  = 0;
  sei();
  #ifdef DEBUG
    Serial.println("WDT_DIS");
  #endif
}

void sleep(){
  radio.powerDown();
  sleep_enable();
  enWDT();
  ADCSRA = 0;
  DDRB = 0XFF;
  DDRD = 0XFF;
  DDRC = 0XCF; // SDA, SCL HARD PULLED UP to VCC, set pin to HI-Z
  PORTB  = 0x00;
  PORTD  = 0x00;
  PORTC  = 0x30;// SDA, SCL HARD PULLED UP to VCC
  // turn off brown-out enable in software
  MCUCR = bit(6)|bit(5);
  MCUCR = bit(6);
  sleep_cpu();
}

void wakeup(){
  //disable sleep
  sleep_disable();
  disWDT();
  radio.powerUp();
  pinMode(ADCBATPIN, INPUT);
  pinMode(ADCVCC, OUTPUT);digitalWrite(ADCVCC, 1);
  pinMode(ADCGND, OUTPUT);digitalWrite(ADCGND, 0);
  ADCSRA = 0x87; //62Khz adc
}

void prepData(){      
  bmp.begin(BMP_ADR);
  data[0] = TR_NUM;  
  if(mcur) {
    data[0] |= mcur<<4;
    mcur=0;   
  }    
  bmp.setSampling(Adafruit_BMP280::MODE_FORCED,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_1);   /* Standby time. */
  int16_t temp = bmp.readTemperature()*100;
  int16_t bat = float(analogRead(ADCBATPIN))/DIV_ADC;    
  data[1] = temp>>8;
  data[2] = temp&0xFF;  
  data[3] = bat>>8;  
  data[4] = bat&0xFF;   

  #ifdef DEBUG    
      Serial.print("STATE:");
      Serial.println(data[0], HEX);
      Serial.print("ADCBAT:");
      Serial.println(bat/100.0F);
      Serial.print("Temp:");
      Serial.println(temp/100.0F);  
      for(uint8_t i = 0;i < 5; i++ ){
        Serial.print(data[i], HEX);
        Serial.print(';');
      }
        
      Serial.println();    
  #endif     
}

void loop() {
  if(wdt_cnt > (7*TR_DELMIN)){    // 7,4*8sec = 1min
    wdt_cnt = 0;    
    wakeup();    
    prepData();
    radio.write(&data, PayloadSize);    
    #ifdef DEBUG
      delay(1000);
    #endif    
  }   
    sleep();
}