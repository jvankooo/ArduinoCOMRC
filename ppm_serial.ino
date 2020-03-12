#include <UnoJoy.h>

#define SERIAL_PORT_SPEED 38400
#define Filter_weight 2
#define Filter_Thresh 100

unsigned long int ft, lt, x, tfail;
int ch, chx[9][13];

const int idx   = 10;
const int total = 11;
const int val   = 12;

// Filter to remove noise from data
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


void setup() {
  Serial.begin(SERIAL_PORT_SPEED);
  ft=0; lt=0; x=0; tfail=0; ch=0;
  for (int i=0; i<9; i++) {
    for (int j=0; j<13; j++) {
      chx[i][j]=0;
    }
  }
  pinMode(2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), decodePPM, FALLING);

  setupUnoJoy();
}

void loop() {
  if ((millis()-tfail)>500) {
    Serial.println("Disconnect");
  } else {
    dataForController_t controllerData = getControllerData();
    setControllerData(controllerData);
    CH1.Filter(chx[1][val]);
    CH2.Filter(chx[2][val]);
    CH3.Filter(chx[3][val]);
    CH4.Filter(chx[4][val]);
//    Serial.print(chx[1][val]);Serial.print("\t");
//    Serial.print(CH1.Current());Serial.print("\t");
//    Serial.print(CH2.Current());Serial.print("\t");
//    Serial.print(CH3.Current());Serial.print("\t");
//    Serial.print(CH4.Current());
//    Serial.println();    
  }
}

void decodePPM() {  
  lt    = micros();
  tfail = millis();
  x     = lt-ft;
  ft    = lt;
  if (x>3000) {
    ch          = 0;
    chx[0][val] = x;
  } else {
    ch+=1;
    int indx       = chx[ch][idx];
    chx[ch][total] = chx[ch][total] - chx[ch][indx];
    chx[ch][indx]  = x;
    chx[ch][total] = chx[ch][total] + chx[ch][indx];
    chx[ch][idx]   = chx[ch][idx] + 1;
    if (chx[ch][idx]>9) {
      chx[ch][idx] = 0;
    }
    chx[ch][val] = chx[ch][total]/10;
  }
}

dataForController_t getControllerData(void){

  dataForController_t controllerData = getBlankDataForController();
  //  Map the filtered data to 8 bit controller data
  controllerData.leftStickX = map(CH4.Current(), 995, 2005, 1, 254);
  controllerData.leftStickY = map(CH3.Current(), 995, 2005, 254, 1);
  controllerData.rightStickX = map(CH1.Current(), 995, 2005, 1, 254);
  controllerData.rightStickY = map(CH2.Current(), 995, 2005, 254, 1);
  // And return the data!
  return controllerData;
}
