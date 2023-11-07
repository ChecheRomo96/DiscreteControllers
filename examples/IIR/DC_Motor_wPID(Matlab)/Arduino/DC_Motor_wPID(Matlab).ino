#include <DiscreteControllers.h>

//////////////////////////////////////////////////////////////////////
// DC Motor transfer function
// 
// Note that the motor model expects a signal ranging from -1 to 1, 
// representing the PWM pulse width and direction of the DC Motor.
// Also note that it was discretized with a sample frequency of 100 Hz.

  float Fs = 100;
  float Ts = 1.0 / Fs;
//
//////////////////////////////////////////////////////////////////////
// System time variable that do not depends on micros() or millis() 
  
  float StartTime = 0;
  float EndTime = 13;

  float PulseTime1 = 1;
  float PulseTime2 = 7;
  float PulseAmplitude1 = 1;
  float PulseAmplitude2 = 0;
//
//////////////////////////////////////////////////////////////////////
// Setup

  void setup() {
    Serial.begin(115200);

    DiscreteControllers::IIR<float, float> Motor({26.700957475450720 }, {1, -0.778800783071405 } );

    // Tiempo de Establecimiento 1s
    float Kp = 0.000138369466218086;
    float Ki = 0.0276738932436173;
    float Kd = 0;
    
    // Tiempo de Establecimiento 5s
    //float Kp = 3.14372641966467e-05;
    //float Ki = 0.00628745283932935;
    //float Kd = 0;
    
    DiscreteControllers::IIR<float, float> PID;
    PID.SetCoefficients({ Kp + Ki + Kd, -(Kp)-(2 * Kd), Kd }, { 1, -1 });

    Motor.InitializeBuffers();
    PID.InitializeBuffers();

    for(uint8_t Counter = 0; Counter < 5; Counter++){
      Motor.InitializeBuffers();

      switch(Counter)
      {
        case 0: Serial.print( "t = " ); break;
        case 1: Serial.print( "SetPoint = " ); break;
        case 2: Serial.print( "ControllerOutput = "); break;
        case 3: Serial.print( "SystemResponse = "); break;
        case 4: Serial.print( "Error = "); break;
      } Serial.print("[");

      float SetPoint = 0;
      float SystemResponse = Motor.Update(0);
      float ControllerOutput = PID.Update(0);
      float Error = 0;

      for(float t = StartTime; t < EndTime; t+=Ts )
      {
        if(t>= PulseTime1){ SetPoint = PulseAmplitude1; }
        if(t>= PulseTime2){ SetPoint = PulseAmplitude2; }
        
        Error = (SetPoint*120.71 - SystemResponse);
        ControllerOutput = PID.Update(Error/120.71);
        SystemResponse = Motor.Update(ControllerOutput);
       
        switch(Counter)
        {
          case 0: Serial.print( String(t) + ", " ); break;
          case 1: Serial.print( String(SetPoint) + ", " ); break;
          case 2: Serial.print( String(ControllerOutput) + ", " ); break;
          case 3: Serial.print( String(SystemResponse) + ", " ); break;
          case 4: Serial.print( String(Error) + ", " ); break;
        }
      }
      Serial.println("];");
    }
    Serial.println();  
    Serial.println("plot(t,SetPoint*120.71,t,ControllerOutput*120.71,t,SystemResponse,t,Error)");
    Serial.println("grid()");
  }
//
//////////////////////////////////////////////////////////////////////
// Loop

  void loop() {

  }
//
//////////////////////////////////////////////////////////////////////