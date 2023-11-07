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
//
//////////////////////////////////////////////////////////////////////
// System time variable that do not depends on micros() or millis() 
  
  float StartTime = 0;
  float EndTime = 1.1;

  float PulseTime1 = 0.1;
  float PulseTime2 = 0.6;
  float PulseAmplitude1 = 1;
  float PulseAmplitude2 = 0;
//
//////////////////////////////////////////////////////////////////////
// Setup

  void setup() {
    Serial.begin(115200);
    Motor.InitializeBuffers();

    for(uint8_t Counter = 0; Counter < 4; Counter++){
      Motor.InitializeBuffers();

      switch(Counter)
      {
        case 0: Serial.print( "t = " ); break;
        case 1: Serial.print( "SetPoint = " ); break;
        case 2: Serial.print( "SystemResponse = "); break;
        case 3: Serial.print( "Error = "); break;
      } Serial.print("[");

      float SetPoint = 0;
      float SystemResponse = Motor.Update(0);
      float Error = 0;

      for(float t = StartTime; t < EndTime; t+=Ts )
      {
        if(t>= PulseTime1){ SetPoint = PulseAmplitude1; }
        if(t>= PulseTime2){ SetPoint = PulseAmplitude2; }
        
        Error = ((SetPoint*120.71) - SystemResponse);
        SystemResponse = Motor.Update(SetPoint);
       
        switch(Counter)
        {
          case 0: Serial.print( String(t) + ", " ); break;
          case 1: Serial.print( String(SetPoint) + ", " ); break;
          case 2: Serial.print( String(SystemResponse) + ", " ); break;
          case 3: Serial.print( String(Error) + ", " ); break;
        }
      }
      Serial.println("];");
    }
    Serial.println();
    Serial.println("plot(t,SetPoint,t,SystemResponse,t,Error)");
    Serial.println("grid()");
  }
//
//////////////////////////////////////////////////////////////////////
// Loop

  void loop() {

  }
//
//////////////////////////////////////////////////////////////////////