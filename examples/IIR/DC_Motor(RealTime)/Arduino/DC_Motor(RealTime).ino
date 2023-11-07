#include <DiscreteControllers.h>

//////////////////////////////////////////////////////////////////////
// DC Motor transfer function
// 
// Note that the motor model expects a signal ranging from -1 to 1, 
// representing the PWM pulse width and direction of the DC Motor.
// Also note that it was discretized with a sample frequency of 100 Hz.

  DiscreteControllers::IIR<float, float> Motor({26.700957475450720 }, {1, -0.778800783071405 } );
  float Fs = 100;
  float Ts = 1.0 / Fs;
  float SetPoint = 0, SystemResponse = 0;
//
//////////////////////////////////////////////////////////////////////
// System time variable that do not depends on micros() or millis() 
 
    float time = 0;
//
//////////////////////////////////////////////////////////////////////
// Setup

  void setup() {
    Serial.begin(115200);
    
    Motor.InitializeBuffers();
  }
//
//////////////////////////////////////////////////////////////////////
// Loop

  void loop() {
    if(Serial.available()){
      float x = Serial.parseFloat();
      SetPoint = x;
      while (Serial.available()) Serial.read();
    }

    SystemResponse = Motor.Update(SetPoint);

    Serial.print( "SetPoint:" + String(SetPoint * 120.71) );   
    Serial.println( ",SystemResponse:" + String(SystemResponse) );

    time+=Ts;
    delay(Ts*1000);
  }
//
//////////////////////////////////////////////////////////////////////