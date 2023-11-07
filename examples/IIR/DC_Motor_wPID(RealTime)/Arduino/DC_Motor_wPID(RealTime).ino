#include <DiscreteControllers.h>

DiscreteControllers::IIR<float, float> Motor({26.700957475450720 }, {1, -0.778800783071405 } );
DiscreteControllers::IIR<float, float> PID;

float Fs = 100;
float Ts = 1.0 / Fs;
float time = 0;
float SetPoint = 0, SystemResponse = 0, ControllerOutput = 0, Error = 0;

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(1);
  
  // Tiempo de Establecimiento 1s
  double Kp = 0.000138369466218086;
  double Ki = 0.0276738932436173;
  double Kd = 0;
  
  // Tiempo de Establecimiento 5s
  //float Kp = 3.14372641966467e-05;
  //float Ki = 0.00628745283932935;
  //float Kd = 0;
  

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Parallel PID Topology (H(s) = Kp + (Ki/s) + (kd*s)) discretized with the Backwards Euler method ( s = (z-1) / (Ts * z) )
  //
  //               Ki                              z^0(kp+ki+kd) + z^-1(-kp-2kd) + z^-2(kd)
  // H(s) = Kp + ------ + (kd * s)    ->   H(z) = --------------------------------------------
  //               s                                           z^0(1) + z^-1(-1)
  //
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
  PID.SetCoefficients({ Kp + Ki + Kd, -(Kp)-(2 * Kd), Kd }, { 1, -1 });
  
  Motor.InitializeOutputBuffer();
  Motor.InitializeInputBuffer();
  PID.InitializeOutputBuffer();
  PID.InitializeInputBuffer();
}

void loop() {
  if(Serial.available()){
    float x = Serial.parseFloat();
    SetPoint = x;
    while (Serial.available()) Serial.read();
  }

  //SetPoint = sin(6.283 * 0.1 * time);
  Error = (SetPoint - SystemResponse);
  ControllerOutput = PID.Update(Error/120.71);
  SystemResponse = Motor.Update(ControllerOutput);

  Serial.print( "SetPoint:" + String(SetPoint) );   
  Serial.print( ",ControllerOutput:" + String(ControllerOutput*120) );
  Serial.print( ",SystemResponse:" + String(SystemResponse) );
  Serial.println( ",Error:" + String(Error) );

  time+=Ts;
  delay(Ts*1000);
}