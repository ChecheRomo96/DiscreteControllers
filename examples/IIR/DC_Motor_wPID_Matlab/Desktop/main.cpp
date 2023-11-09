#include <DiscreteControllers.h>
#include <iostream>
#include <CPstring.h>

//////////////////////////////////////////////////////////////////////
// System time variable that do not depends on micros() or millis() 
  
  float StartTime = 0;
  float EndTime = 13;
  
  float Fs = 100;
  float Ts = 1.0 / Fs;

  float PulseTime1 = 1;
  float PulseTime2 = 7;
  float PulseAmplitude1 = 1;
  float PulseAmplitude2 = 0;
//
//////////////////////////////////////////////////////////////////////
// Setup

  int main() {
    DiscreteControllers::IIR<float, float> Motor({26.700957475450720 }, {1, -0.778800783071405 } );

    // Settling Time = 1s
    float Kp = 0.000138369466218086;
    float Ki = 0.0276738932436173;
    float Kd = 0;
    
    // Settling Time = 5s
    Kp = 3.14372641966467e-05;
    Ki = 0.00628745283932935;
    Kd = 0;
    
    DiscreteControllers::IIR<float, float> PID;
    PID.SetCoefficients({ Kp + Ki + Kd, -(Kp)-(2 * Kd), Kd }, { 1, -1 });

    Motor.InitializeBuffers();
    PID.InitializeBuffers();

    for(uint8_t Counter = 0; Counter < 5; Counter++){
      Motor.InitializeBuffers();
      PID.InitializeBuffers();

      switch(Counter)
      {
        case 0: std::cout<<( "t = " ); break;
        case 1: std::cout<<( "SetPoint = " ); break;
        case 2: std::cout<<( "ControllerOutput = " ); break;
        case 3: std::cout<<( "SystemResponse = "); break;
        case 4: std::cout<<( "Error = "); break;
      } std::cout<<("[");

      float SetPoint = 0;
      float SystemResponse = Motor.Update(0);
      float ControllerOutput = PID.Update(0);
      float Error = 0;

      for(float t = StartTime; t < EndTime; t+=Ts )
      {
        if(t>= PulseTime1){ SetPoint = PulseAmplitude1; }
        if(t>= PulseTime2){ SetPoint = PulseAmplitude2; }

        Error = (SetPoint * 120.71 - SystemResponse);
        ControllerOutput = PID.Update(Error / 120.71);
        SystemResponse = Motor.Update(ControllerOutput);

       
        switch(Counter)
        {
          case 0: std::cout<<t<< ", " ; break;
          case 1: std::cout<<SetPoint << ", " ; break;
          case 2: std::cout<<ControllerOutput << ", " ; break;
          case 3: std::cout << SystemResponse << ", "; break;
          case 4: std::cout<<Error<< ", " ; break;
        }
      }
      std::cout<<("];")<<std::endl;
    }
    std::cout<<std::endl;
    std::cout<<("plot(t,SetPoint*120.21,t,SystemResponse,t,Error)")<<std::endl; 
    //plot(t, SetPoint * 120.21, t, SystemResponse, t, Error, t, SystemResponse2, t, Error2)
    std::cout<<("grid()");

    return 0;
  }
//
//////////////////////////////////////////////////////////////////////