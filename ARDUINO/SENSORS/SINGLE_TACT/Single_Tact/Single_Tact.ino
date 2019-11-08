  #include <Wire.h> //For I2C/SMBus


# define CMD_READ       0x01
# define CMD_WRITE      0x02
# define PACKET_END     0xFF
# define OFFSET_READ    128 //128

const int n_ch = 1;
const byte i2cAddress[n_ch] = {0x04};
short val;

void setup() {
  Wire.begin(); // join i2c bus (address optional for master)
  //TWBR = 12; //read speed Increase for Mega
  Serial.begin(115200);  // start serial for output
  Serial.flush();

}

void loop() {
  val = read_data(i2cAddress[0]);
  Serial.println(val);
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
      delayMicroseconds(10); //Wait 10us
    }
  }
  short rawData = (incomingI2CBuffer[4] << 8) + incomingI2CBuffer[5];
  return rawData;
}
