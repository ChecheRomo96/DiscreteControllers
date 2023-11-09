#include <DiscreteControllers.h>
#include <iostream>
#include <CPstring.h>

//////////////////////////////////////////////////////////////////////
// System time variable that do not depends on micros() or millis() 
  
  float StartTime = 0;
  float EndTime = 1.1;

  float Fs = 100;
  float Ts = 1.0 / Fs;

  float PulseTime1 = 0.1;
  float PulseTime2 = 0.6;
  float PulseAmplitude1 = 1;
  float PulseAmplitude2 = 0;
//
//////////////////////////////////////////////////////////////////////
// Setup

  int main() {
    DiscreteControllers::IIR<float, float> Motor({26.700957475450720 }, {1, -0.778800783071405 } );
    Motor.InitializeBuffers();

    for(uint8_t Counter = 0; Counter < 4; Counter++){
      Motor.InitializeBuffers();

      switch(Counter)
      {
        case 0: std::cout<<( "t = " ); break;
        case 1: std::cout<<( "SetPoint = " ); break;
        case 2: std::cout<<( "SystemResponse = "); break;
        case 3: std::cout<<( "Error = "); break;
      } std::cout<<("[");

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
        case 0: std::cout<<t<< ", " ; break;
          case 1: std::cout<<SetPoint << ", " ; break;
          case 2: std::cout << SystemResponse << ", "; break;
          case 3: std::cout<<Error<< ", " ; break;
        }
      }
      std::cout<<("];")<<std::endl;
    }
    std::cout<<std::endl;
    std::cout<<("plot(t,SetPoint*120.21,t,SystemResponse,t,Error)")<<std::endl;
    std::cout<<("grid()");

    return 0;
  }
//
//////////////////////////////////////////////////////////////////////