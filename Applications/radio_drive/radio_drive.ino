// Radio Drive - April 2020 lockdown exploration bot @jvankooo
#include <EnableInterrupt.h>

#define SERIAL_PORT_SPEED 38400
#define RC_NUM_CHANNELS  3
#define Filter_weight 2
#define Filter_Thresh 100

#define RC_CH1  0
#define RC_CH2  1
#define RC_CH3  2

#define RC_CH1_INPUT  2
#define RC_CH2_INPUT  3
#define RC_CH3_INPUT  4


// Motor Driver Variables

//Motor Left
const int PinL1  = 6;
const int PinL2  = 7; 
bool L1 = false;
bool L2 = false;

//Motor Right
const int PinR1  = 8;   
const int PinR2  = 9;  
bool R1 = false;
bool R2 = false;
 
int EN1 = 11;             
int EN2 = 10;

int LeftSpeed = 0;
int RightSpeed = 0;
int error = 0;
bool drive = false;
bool reverse = false;


template<class T> class ExponentialFilter
{
  // Weight for new values, as a percentage ([0..100])
  T m_WeightNew;

  // Current filtered value. 
  T m_Current;

public:
  ExponentialFilter(T WeightNew, T Initial)
    : m_WeightNew(WeightNew), m_Current(Initial)
  { }

  void Filter(T New)
  {
//    m_Current = (100 * m_WeightNew * New + (100 - m_WeightNew) * m_Current + 50)/100;
    if(abs(New-m_Current/100)>Filter_Thresh){
         m_WeightNew = 50*m_WeightNew;
         m_Current = (100 * m_WeightNew * New + (100 - m_WeightNew) * m_Current + 50)/100;
         m_WeightNew = m_WeightNew/50;
      }
    else{
      m_Current = (100 * m_WeightNew * New + (100 - m_WeightNew) * m_Current + 50)/100;
    }
  }
  
  T Current() const { return (m_Current + 50)/100; }
};

ExponentialFilter<long> CH1(Filter_weight, 0);
ExponentialFilter<long> CH2(Filter_weight, 0);
ExponentialFilter<long> CH3(Filter_weight, 0);


uint16_t rc_values[RC_NUM_CHANNELS];
uint32_t rc_start[RC_NUM_CHANNELS];
volatile uint16_t rc_shared[RC_NUM_CHANNELS];

void rc_read_values() {
  noInterrupts();
  memcpy(rc_values, (const void *)rc_shared, sizeof(rc_shared));
  interrupts();
}

void calc_input(uint8_t channel, uint8_t input_pin) {
  if (digitalRead(input_pin) == HIGH) {
    rc_start[channel] = micros();
  } else {
    uint16_t rc_compare = (uint16_t)(micros() - rc_start[channel]);
    rc_shared[channel] = rc_compare;
  }
}

void calc_ch1() { calc_input(RC_CH1, RC_CH1_INPUT); }
void calc_ch2() { calc_input(RC_CH2, RC_CH2_INPUT); }
void calc_ch3() { calc_input(RC_CH3, RC_CH3_INPUT); }


// ------------------------------------------------ MAIN -------------------------------------------------

void setup() {
  Serial.begin(SERIAL_PORT_SPEED);

//  RC INput Pins

  pinMode(RC_CH1_INPUT, INPUT);
  pinMode(RC_CH2_INPUT, INPUT);
  pinMode(RC_CH3_INPUT, INPUT);

//  Motor Drive Pins

  pinMode(EN1, OUTPUT);
  pinMode(EN2, OUTPUT);
  pinMode(PinL1, OUTPUT);
  pinMode(PinL2, OUTPUT);
  pinMode(PinR1, OUTPUT);
  pinMode(PinR2, OUTPUT); 

  enableInterrupt(RC_CH1_INPUT, calc_ch1, CHANGE);
  enableInterrupt(RC_CH2_INPUT, calc_ch2, CHANGE);
  enableInterrupt(RC_CH3_INPUT, calc_ch3, CHANGE);
}

void loop() {
  rc_read_values();
  CH1.Filter(rc_values[RC_CH1]);
  CH2.Filter(rc_values[RC_CH2]);
  CH3.Filter(rc_values[RC_CH3]);
  Serial.print("CH1:"); Serial.print(CH1.Current());
  Serial.print("\t");
  Serial.print("CH2:"); Serial.print(CH2.Current());
  Serial.print("\t");
  Serial.print("CH3:"); Serial.print(CH3.Current());

//   Drive Logic
  if(CH1.Current() > 1100){
    drive = true;
  }
  else {
    drive = false;
  }
  
//  Bot on drive
  if(drive){
    if(CH3.Current() < 1500){       // forward
      Serial.print("\t");
      Serial.print("Drive");
      L1 = true;
      L2 = false;
      R1 = true;
      R2 = false;
    }
    else{                           // reverse
      Serial.print("\t");
      Serial.print("Reverse");
      L1 = false;
      L2 = true;
      R1 = false;
      R2 = true;
    }
    
    LeftSpeed = map(CH1.Current(), 1000, 2000, 0 , 255) + map(CH2.Current(), 1000, 2000, -128 , 128);
    RightSpeed = map(CH1.Current(), 1000, 2000, 0 , 255) - map(CH2.Current(), 1000, 2000, -128 , 128);
    
    if(LeftSpeed < 0){
      L1 = !L1;
      L2 = !L2;    
    }
    if(RightSpeed < 0){
      R1 = !R1;
      R2 = !R2;
    }

    if(LeftSpeed > 255){
      LeftSpeed = 255;
    }
    if(RightSpeed > 255){
      RightSpeed = 255;
    }

    Serial.print("\t");
    Serial.print("Left:"); Serial.print(LeftSpeed);
    Serial.print("\t");
    Serial.print("Right:"); Serial.print(RightSpeed);
    
    analogWrite(EN1, abs(LeftSpeed));
    analogWrite(EN2, abs(RightSpeed) + error);

    digitalWrite(PinL1, L1);
    digitalWrite(PinL2, L2);
    digitalWrite(PinR1, R1);
    digitalWrite(PinR2, R2);
    
  }
  
//  Bot Brake Stop
  else{
    digitalWrite(PinL1, LOW);
    digitalWrite(PinL2, LOW);
    digitalWrite(PinR1, LOW);
    digitalWrite(PinR2, LOW);
  }
  
  Serial.print("\r \n");
}
