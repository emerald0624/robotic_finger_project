#include <SPI.h>
unsigned int reading = 0;

// AS4047P Registers
const unsigned int n_cs             = 2;
const unsigned int select_pin[n_cs] = {10,11};
int val[n_cs];
                             
/** volatile **/
#define NOP                 0x0000  // No operation                                                 // 0000,0000,0000,0000
#define ERRFL               0x0001  // Error regigster                                              // 0000,0000,0000,0001
#define PROG                0x0003  // Programming register                                         // 0000,0000,0000,0011
#define DIAAGC              0x3FFC  // Diagnostic and AGC                                           // 0111,1111,1111,1100
#define CORDICMAG           0x3FFD  // CORDIC magnitude                                             // 0111,1111,1111,1101
#define ANGLEUNC            0x3FFE  // Measured angle without dynamic angle error compensation      // 0111,1111,1111,1110
#define ANGLECOM            0x3FFF  // Measured angle with    dynamic angle error compensation      // 0111,1111,1111,1111

/** non-volatile **/
#define ZPOSM               0x0016  // Zero position MSB
#define ZPOSL               0x0017  // Zero position LSB / MAG diagnostic
#define SETTINGS1           0x0018  // Custom setting register 1
#define SETTINGS2           0x0019  // Custom setting register 2

#define RD                  0x40    // 0100,0000 bit 14 = "1" is Read + parity even
#define WR                  0x3F    // 0011,1111 bit 14 = "0" is Write

// Arduino        : MOSI : D11, MISO : D12, SCK : D13
// Arduino Due    : MOSI : D75, MISO : D74, SCK : D76
SPISettings settings(SPI_CLOCK_DIV4, MSBFIRST, SPI_MODE1);

void setup() {
  // select pin setup
  for (int i = 0; i < n_cs; i++)
  {
    pinMode(select_pin[i], OUTPUT);
  }

  // SPI setup (AS5047 기본 세팅)
  SPI.begin();
  SPI.setDataMode(SPI_MODE1); // properties chip
  SPI.setBitOrder(MSBFIRST);  //properties chip

  // serial setup
  Serial.begin(115200);  // start serial for output

  // AS5047P 기본 setup
  for (int i = 0; i < n_cs; i++)
  {
    AS5047P_Write(select_pin[i] , SETTINGS1, 0x0001);       // DJL was 0x0004);
    AS5047P_Write(select_pin[i] , SETTINGS2, 0x0000);
    AS5047P_Write(select_pin[i] , ZPOSM, 0x0000);           // is it really possible to initially set angle at 0 degrees??
    AS5047P_Write(select_pin[i] , ZPOSL, 0x0000);
  }
}

void loop()
{
  //DumpRegisterValues();  
  
  String print_string = "";
  for (int i = 0; i < n_cs; i++)
  {
    val[i] = AS5047P_Read(select_pin[i], ANGLECOM) & 0x3FFF;    
    if (i != 0)
    {
      print_string  = print_string + ",";
    }
    print_string = print_string + String(val[i]);
    
  }
  
  Serial.println(print_string);
  //delay(200);
}

// multi data read
/*
void DumpRegisterValues()
{
  Serial.print("NOP: "); Serial.println(AS5047P_Read( AS5047P_select_pin, NOP) & 0x3FFF, BIN); // strip bit 14..15
  Serial.print("ERRFL: "); Serial.println(AS5047P_Read( AS5047P_select_pin, ERRFL) & 0x3FFF, BIN); // strip bit 14..15
  Serial.print("PROG: "); Serial.println(AS5047P_Read( AS5047P_select_pin, PROG) & 0x3FFF, BIN); // strip bit 14..15
  Serial.print("DIAAGC: "); Serial.println(AS5047P_Read( AS5047P_select_pin, DIAAGC) & 0x3FFF, BIN); // strip bit 14..15

  Serial.print("CORDICMAG: "); Serial.println(AS5047P_Read( AS5047P_select_pin, CORDICMAG) & 0x3FFF, DEC); // strip bit 14..15
  Serial.print("ANGLEUNC: "); Serial.println(AS5047P_Read( AS5047P_select_pin, ANGLEUNC) & 0x3FFF, DEC); // strip bit 14..15
  Serial.print("ANGLECOM: "); Serial.println(AS5047P_Read( AS5047P_select_pin, ANGLECOM) & 0x3FFF, DEC); // strip bit 14..15

  Serial.print("ZPOSM: "); Serial.println(AS5047P_Read( AS5047P_select_pin, ZPOSM) & 0x3FFF, BIN); // strip bit 14..15
  Serial.print("ZPOSL: "); Serial.println(AS5047P_Read( AS5047P_select_pin, ZPOSL) & 0x3FFF, BIN); // strip bit 14..15
  Serial.print("SETTINGS1: "); Serial.println(AS5047P_Read( AS5047P_select_pin, SETTINGS1) & 0x3FFF, BIN); // strip bit 14..15
  Serial.print("SETTINGS2: "); Serial.println(AS5047P_Read( AS5047P_select_pin, SETTINGS2) & 0x3FFF, BIN); // strip bit 14..15
}
*/

// Write function
// input : select pin / register address / write value
void AS5047P_Write( int SSPin, int address, int value)
{
  // SPI start & select pin set LOW(ON)
  SPI.beginTransaction(settings);
  digitalWrite(SSPin, LOW);

  // LOW/HIGH byte seperate for SPI
  byte v_l = address & 0x00FF;
  byte v_h = (unsigned int)(address & 0x3F00) >> 8;
  if (parity(address & 0x3F) == 1) v_h = v_h | 0x80; // set parity bit

  // Write bits
  SPI.transfer(v_h);
  SPI.transfer(v_l);

  // select pin set HIGH(OFF) & SPI END
  digitalWrite(SSPin, HIGH);
  SPI.endTransaction();

  // delay
  delay(2);

  // SPI start & select pin set LOW(ON)
  SPI.beginTransaction(settings);
  digitalWrite(SSPin, LOW);

  // LOW/HIGH byte seperate for SPI
  v_l = value & 0x00FF;
  v_h = (unsigned int)(value & 0x3F00) >> 8;
  if (parity(value & 0x3F) == 1) v_h = v_h | 0x80; // set parity bit

  // Write bits
  SPI.transfer(v_h);
  SPI.transfer(v_l);

  // select pin set HIGH(OFF) & SPI END
  digitalWrite(SSPin, HIGH);
  SPI.endTransaction();
}

// Read function
// input : select pin / register address
unsigned int AS5047P_Read( int SSPin, unsigned int address)
{
  // result set 0 & HIGH/LOW register set 0
  unsigned int result = 0;
  byte res_h = 0;
  byte res_l = 0;

  // SPI start & select pin set LOW(ON)
  SPI.beginTransaction(settings);
  digitalWrite(SSPin, LOW);

  // LOW/HIGH byte seperate for SPI
  byte v_l = address & 0x00FF;
  byte v_h = (unsigned int)(address & 0x3F00) >> 8;
  if (parity(address | (RD << 8)) == 1) v_h = v_h | 0x80; // set parity bit
  v_h = v_h | RD; // its  a read command

  // Write bits
  res_h = SPI.transfer(v_h);
  res_l = SPI.transfer(v_l);

  // select pin set HIGH(OFF) & SPI END
  digitalWrite(SSPin, HIGH);
  SPI.endTransaction();

  delay(2);

  // SPI start & select pin set LOW(ON)
  SPI.beginTransaction(settings);
  digitalWrite(SSPin, LOW);

  // LOW/HIGH byte seperate for SPI
  res_h = SPI.transfer(0x00);
  res_l = SPI.transfer(0x00);
  res_h = res_h & 0x3F;  // filter bits outside data

  // select pin set HIGH(OFF) & SPI END
  digitalWrite(SSPin, HIGH);
  SPI.endTransaction();

  return (result = (res_h << 8) | res_l);
}

// parity check Code
int parity(unsigned int x) {
  int parity = 0;
  while (x > 0) {
    parity = (parity + (x & 1)) % 2;
    x >>= 1;
  }
  return (parity);
}
