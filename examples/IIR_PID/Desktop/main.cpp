#include <DiscreteControllers.h>
#include <CPvector.h>
#include <CPstring.h>

//////////////////////////////////////////////////////////////////
//
// Using an IIR filter to simulate the response of the Plant,
// In this case the plant represents a DC motor, whose 
// Discrete Transfer function can be estimated as:
// 
//          z^-1 (26.700957475450720)               fs = 100 Hz
// H(z) =  -----------------------------      @ 
//          z^-1 (-0.778800783071405) + 1           Ts = 0.01 s
//
////////////////////////////////////////////////////////////////// 


DiscreteControllers::IIR<float> Motor(
    cpstd::vector<double>({ 26.700957475450720, 0 }),
    cpstd::vector<double>({ -0.778800783071405, 1 }) );



float StartTime = 0;
float EndTime = 10;

float Fs = 100;
float Ts = 1.0 / Fs;

float PulseTime1 = 1;
float PulseTime2 = 4;
float PulseTime3 = 7;
float PulseAmplitude1 = 60;
float PulseAmplitude2 = 120;
float PulseAmplitude3 = -120;


int main(){

  Motor.SetCoefficients(
      cpstd::vector<double>({ 26.700957475450720, 0 }),
      cpstd::vector<double>({ -0.778800783071405, 1 }));

  // Tiempo de Establecimiento 1s
  float Kp = 0.000138369466218086;
  float Ki = 0.0276738932436173;
  float Kd = 0;
  
  // Tiempo de Establecimiento 5s
  //float Kp = 3.14372641966467e-05;
  //float Ki = 0.00628745283932935;
  //float Kd = 0;
  
  DiscreteControllers::IIR<float> PID;

  // Función de Transferencia de las diapositivas del curso
  
  double a = Kp;
  double b = Ki;
  double c = Kd;
  
  double PID_NTaps[] = {c, -(a)-(2*c), a+b+c};
  double PID_DTaps[] = {-1, 1};

  cpstd::vector<double> PID_NumTaps(PID_NTaps,3);
  cpstd::vector<double> PID_DenTaps(PID_DTaps,2);
  PID.SetCoefficients(PID_NumTaps,PID_DenTaps);



  for (uint8_t Counter = 0; Counter < 5; Counter++)
  {
      Motor.InitializeOutputBuffer();
      Motor.InitializeInputBuffer();
      PID.InitializeOutputBuffer();
      PID.InitializeInputBuffer();

      switch (Counter)
      {
      case 0: std::cout << ("t = "); break;
      case 1: std::cout << ("SetPoint = "); break;
      case 2: std::cout << ("ControllerOutput = "); break;
      case 3: std::cout << ("SystemResponse = "); break;
      case 4: std::cout << ("Error = "); break;
      }

      std::cout<<("[");

      float SetPoint = 0;
      float SystemResponse = Motor.Update(0);
      float ControllerOutput = PID.Update(0);
      float Error = 0;

      for (float t = StartTime; t < EndTime; t += Ts)
      {
          if (t >= PulseTime1) { SetPoint = PulseAmplitude1; }
          if (t >= PulseTime2) { SetPoint = PulseAmplitude2; }
          if (t >= PulseTime3) { SetPoint = PulseAmplitude3; }
          //SetPoint = PulseAmplitude * sin(2*3.14159*2*t);

          Error = (SetPoint - SystemResponse);
          ControllerOutput = PID.Update(Error / 120.71);
          SystemResponse = Motor.Update(ControllerOutput);

          switch (Counter)
          {
          case 0: std::cout << (cpstd::to_string(t) + ", "); break;
          case 1: std::cout << (cpstd::to_string(SetPoint) + ", "); break;
          case 2: std::cout << (cpstd::to_string(ControllerOutput) + ", "); break;
          case 3: std::cout << (cpstd::to_string(SystemResponse) + ", "); break;
          case 4: std::cout << (cpstd::to_string(Error) + ", "); break;
          }
      }
      std::cout << ("];") << std::endl;
  }
  std::cout << std::endl;
  std::cout << ("plot(t,SetPoint,t,ControllerOutput*120.71,t,SystemResponse,t,Error)") << std::endl;
  std::cout << ("grid()") << std::endl;
}