#include <UnoJoy.h>
#include <EnableInterrupt.h>

#define SERIAL_PORT_SPEED 38400
#define RC_NUM_CHANNELS  4
#define Filter_weight 2
#define Filter_Thresh 100

#define RC_CH1  0
#define RC_CH2  1
#define RC_CH3  2
#define RC_CH4  3

#define RC_CH1_INPUT  2
#define RC_CH2_INPUT  3
#define RC_CH3_INPUT  4
#define RC_CH4_INPUT  5

//Filter to remove noise from the data
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
ExponentialFilter<long> CH4(Filter_weight, 0);


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
void calc_ch4() { calc_input(RC_CH4, RC_CH4_INPUT); }

void setup() {
  Serial.begin(SERIAL_PORT_SPEED);

  pinMode(RC_CH1_INPUT, INPUT);
  pinMode(RC_CH2_INPUT, INPUT);
  pinMode(RC_CH3_INPUT, INPUT);
  pinMode(RC_CH4_INPUT, INPUT);

  enableInterrupt(RC_CH1_INPUT, calc_ch1, CHANGE);
  enableInterrupt(RC_CH2_INPUT, calc_ch2, CHANGE);
  enableInterrupt(RC_CH3_INPUT, calc_ch3, CHANGE);
  enableInterrupt(RC_CH4_INPUT, calc_ch4, CHANGE);

  setupUnoJoy();
}

void loop() {
  rc_read_values();
  CH1.Filter(rc_values[RC_CH1]);
  CH2.Filter(rc_values[RC_CH2]);
  CH3.Filter(rc_values[RC_CH3]);
  CH4.Filter(rc_values[RC_CH4]);
  dataForController_t controllerData = getControllerData();
  setControllerData(controllerData);
//  Serial.print("CH1:"); Serial.print(map(CH1.Current(), 1000, 2000, 0, 255));
//  Serial.print("\t");
//  Serial.print("CH2:"); Serial.print(map(CH2.Current(), 1000, 2000, 0, 255));
//  Serial.print("\t");
//  Serial.print("CH3:"); Serial.print(map(CH3.Current(), 1000, 2000, 0, 255));
//  Serial.print("\t");
//  Serial.print("CH4:"); Serial.print(map(CH4.Current(), 1000, 2000, 0, 255));
//  Serial.println();
}

dataForController_t getControllerData(void){

  dataForController_t controllerData = getBlankDataForController();
  //  Map the PWM signal to 8-bit controller data
  controllerData.leftStickX = map(CH4.Current(), 995, 2005, 1, 254);
  controllerData.leftStickY = map(CH3.Current(), 995, 2005, 254, 1);
  controllerData.rightStickX = map(CH1.Current(), 995, 2005, 1, 254);
  controllerData.rightStickY = map(CH2.Current(), 995, 2005, 254, 1);
  // And return the data!
  return controllerData;
}
