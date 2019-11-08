
// DUE_PINSET
//#define DATAOUT 75          // MOSI PIN
//#define DATAIN 74           // MISO PIN
//#define SPICLOCK 76         // Clock PIN
// MEGA_PINSET
#define DATAOUT 51          // MOSI PIN
#define DATAIN 50           // MISO PIN
#define SPICLOCK 52         // Clock PIN
#define BAUDRATE 115200     // BAUDRATE

const int n_ch = 3;                 // CH 갯수
const int n_adc = n_ch / 8 + 1;     // ADC 갯수
const int SELPIN[n_adc] = {10};     // ADC CS PINs
unsigned int value[n_ch];           // ADC output
float val[n_ch];                    // ADC output mean
const int n_avg = 1;                // moving avg range
int Hz = 1000000 / 500;             // freq

int readvalue;  // instant val
int t1;         // instant val
int t2;         // instant val
int val_cs;     // instant val
int val_ch;     // instant val

float R0[n_ch] = {500};             // base resist
float resist[n_ch];                 // resist
float resist_inv[n_ch];             // resist inverse

void setup() {
  //set pin modes
  for (int i = 0; i < n_adc; i++)
  {
    pinMode(SELPIN[i], OUTPUT);
  }
  pinMode(DATAOUT, OUTPUT);
  pinMode(DATAIN, INPUT);
  pinMode(SPICLOCK, OUTPUT);

  //disable device to start with
  for (int i = 0; i < n_adc; i++)
  {
    digitalWrite(SELPIN[i], HIGH);
  }
  digitalWrite(DATAOUT, LOW);
  digitalWrite(SPICLOCK, LOW);

  Serial.begin(BAUDRATE);
  t1 = micros();
}


int read_adc(int n_selpin, int channel) {
  int adcvalue = 0;
  byte commandbits = B11000000; //command bits - start, mode, chn (3), dont care (3)

  // CS initialize
  for (int i = 0; i < n_adc; i++)
  {
    digitalWrite(SELPIN[i], HIGH);
  }
  digitalWrite(SELPIN[n_selpin], LOW);

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
//float resist_calc(float value_i, float r_base) {
//  //return (r_base * float(value_i) / float(4095 - value_i));
//  return (r_base * float(4095 - value_i)) / float(value_i) ;
//}

void loop() {
  t2 = micros();
  if (t2 - t1 > Hz) {
    // value initailize
    for (int i = 0; i < n_ch; i++) {
      value[i] = 0;
    }
    // read value
    for (int i = 0; i < n_avg; i++) {
      for (int k = 0; k < n_ch; k++) {
        val_cs = k / 8;
        val_ch = k - 8 * val_cs;
        value[k] = value[k] + read_adc(val_cs, val_ch);
      }
    }
    for (int i = 0; i < n_ch; i++) {
      val[i] = float(value[i]) / float(n_avg);
    }

    //int delt = 1000000 / (t2 - t1);
    Serial.print(val[0]);
    Serial.print(',');
    Serial.print(val[1]);
    Serial.print(',');
    Serial.println(val[2]);
//    delayMicroseconds(10);
//    Serial.println(val[1]);
    t1 = t2;

    //Serial.println(delt);
  }
}
