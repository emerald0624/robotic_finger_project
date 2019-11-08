#include <Wire.h> //For I2C/SMBus

//SPI (MCP3208)
#define DATAOUT 75          // MOSI PIN
#define DATAIN 74           // MISO PIN
#define SPICLOCK 76         // Clock PIN

// I2C (SINGLE_TACT)
#define CMD_READ       0x01
#define CMD_WRITE      0x02
#define PACKET_END     0xFF
#define OFFSET_READ    128 //128

// Serial (PC-Arduino)
#define BAUDRATE 115200     // BAUDRATE

//SPI (MCP3208)
const int       n_ch_ADC            = 1;                // CH 갯수
const int       n_ADC               = n_ch_ADC / 8 + 1; // ADC 갯수
const int       pin_ADC[n_ADC]      = {10};             // ADC CS PINs
unsigned int    val_ADC[n_ch_ADC];                      // ADC output
const int       n_avg_ADC           = 1;                // moving avg range
int             Hz                  = 1000000 / 500;    // freq

int val_cs;     // instant val
int val_ch;     // instant val

// I2C (SINGLE_TACT)
const int       n_ch_STAC           = 1;
const byte      Address_STAC[n_ch_STAC]  = {0x04};
short           val_STAC[n_ch_STAC];
const int       n_avg_STAC          = 1;                // moving avg range


void setup() {
  //I2C
  Wire.begin();

  // SPI
  pinMode(DATAOUT, OUTPUT);
  pinMode(DATAIN, INPUT);
  pinMode(SPICLOCK, OUTPUT);
  digitalWrite(DATAOUT, LOW);
  digitalWrite(SPICLOCK, LOW);

  for (int i = 0; i < n_ADC; i++)
  {
    pinMode(pin_ADC[i], OUTPUT);
    digitalWrite(pin_ADC[i], HIGH);
  }

  // Serial
  Serial.begin(BAUDRATE);
  Serial.flush();

}

void loop() {
  //SPI
  for (int i = 0; i < n_ch_ADC; i++)
  {
    val_ADC[i] = 0;
  }

  for (int i = 0; i < n_avg_ADC; i++)
  {
    for (int j = 0; j < n_ch_ADC; j++)
    {
      val_cs = j / 8;
      val_ch = j - 8 * val_cs;
      val_ADC[j] = val_ADC[j] + read_adc(val_cs, val_ch);
    }
  }
  for (int i = 0; i < n_ch_ADC; i++)
  {
    val_ADC[i] = val_ADC[i] / (float)n_avg_ADC;
  }

  //I2C
  for (int i = 0; i < n_ch_STAC; i++)
  {
    val_STAC[i] = 0;
  }

  for (int i = 0; i < n_avg_STAC; i++)
  {
    for (int j = 0; j < n_ch_STAC; j++)
    {
      val_STAC[j] = val_STAC[j] + read_data(Address_STAC[i]);
    }
  }
  for (int i = 0; i < n_ch_STAC; i++)
  {
    val_STAC[i] = val_STAC[i] / (float)n_avg_STAC;
  }
  String str = "";
  for (int i = 0; i < n_ch_ADC; i++)
  {
    if (i!=0)
    {
      str = str + ",";
    }
    str = str + String(val_ADC[i]);
  }
  for (int i = 0; i < n_ch_STAC; i++)
  {
    str = str + "," + String(val_STAC[i]);
  }
  Serial.println(str);
}


short read_data(byte address)
{
  byte i2cPacketLength = 6;     // i2c packet length.
  byte outgoingI2CBuffer[3];    // outgoing array buffer
  byte incomingI2CBuffer[6];    // incoming array buffer

  outgoingI2CBuffer[0] = CMD_READ;//I2c read command
  outgoingI2CBuffer[1] = OFFSET_READ;//Slave data offset
  outgoingI2CBuffer[2] = i2cPacketLength;//require 6 bytes
  Wire.beginTransmission(address);
  // Write bits
  Wire.write(outgoingI2CBuffer, 3);
  byte error = Wire.endTransmission();
  if (error != 0) return -1;

  // Request Data
  Wire.requestFrom(address, i2cPacketLength);
  byte incomeCount = 0;
  while (incomeCount < i2cPacketLength)
  {
    if (Wire.available())
    {
      incomingI2CBuffer[incomeCount] = Wire.read();
      incomeCount++;
    }
    else
    {
      delayMicroseconds(1); //Wait 10us
    }
  }
  short rawData = (incomingI2CBuffer[4] << 8) + incomingI2CBuffer[5];
  return rawData;
}

int read_adc(int n_selpin, int channel) {
  int adcvalue = 0;
  byte commandbits = B11000000; //command bits - start, mode, chn (3), dont care (3)

  // CS initialize
  for (int i = 0; i < n_ADC; i++)
  {
    digitalWrite(pin_ADC[i], HIGH);
  }
  digitalWrite(pin_ADC[n_selpin], LOW);

  // allow channel selection
  commandbits |= ((channel) << 3);

  // setup bits to be written
  for (int i = 7; i >= 3; i--) {
    digitalWrite(DATAOUT, commandbits & 1 << i);
    digitalWrite(SPICLOCK, HIGH);
    digitalWrite(SPICLOCK, LOW);
  }

  digitalWrite(SPICLOCK, HIGH);                     // ignores 2 null bits
  digitalWrite(SPICLOCK, LOW);                      // ignores 2 null bits
  digitalWrite(SPICLOCK, HIGH);                     // ignores 2 null bits
  digitalWrite(SPICLOCK, LOW);                      // ignores 2 null bits

  //read bits from adc
  for (int i = 11; i >= 0; i--) {
    adcvalue += digitalRead(DATAIN) << i;
    //cycle clock
    digitalWrite(SPICLOCK, HIGH);
    digitalWrite(SPICLOCK, LOW);
  }
  return adcvalue;
}
