#include <DiscreteControllers.h>

float StartTime = 0;
float EndTime = 10;

float Fs = 100;
float Ts = 1.0/Fs;

float PulseTime1 = 1;
float PulseTime2 = 4;
float PulseTime3 = 7;
float PulseAmplitude1 = 60;
float PulseAmplitude2 = 120;
float PulseAmplitude3 = -120;
  
void setup() {
  Serial.begin(115200);
  
  DiscreteControllers::IIR<float> Motor;

  double Motor_NTaps[] = {26.700957475450720, 0};
  double Motor_Tsaps[] = {-0.778800783071405, 1};

  CPVector::vector<double> Motor_NumTaps(Motor_NTaps,2);
  CPVector::vector<double> Motor_DenTaps(Motor_Tsaps,2);
  Motor.SetCoefficients(Motor_NumTaps,Motor_DenTaps);

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
  double PID_Tsaps[] = {-1, 1};

  CPVector::vector<double> PID_NumTaps(PID_NTaps,3);
  CPVector::vector<double> PID_DenTaps(PID_Tsaps,2);
  PID.SetCoefficients(PID_NumTaps,PID_DenTaps);

  for(uint8_t Counter = 0; Counter < 5; Counter++)
  {
    Motor.InitializeOutputBuffer();
    Motor.InitializeInputBuffer();
    PID.InitializeOutputBuffer();
    PID.InitializeInputBuffer();
    
    switch(Counter)
    {
      case 0: Serial.print( "t = " ); break;
      case 1: Serial.print( "SetPoint = " ); break;
      case 2: Serial.print( "ControllerOutput = "); break;
      case 3: Serial.print( "SystemResponse = "); break;
      case 4: Serial.print( "Error = "); break;
    }
    
    Serial.print("[");
    
    float SetPoint = 0;
    float SystemResponse = Motor.Update(0);
    float ControllerOutput = PID.Update(0);
    float Error = 0;
      
    for(float t = StartTime; t < EndTime; t+=Ts )
    {
      if(t>= PulseTime1){ SetPoint = PulseAmplitude1; }
      if(t>= PulseTime2){ SetPoint = PulseAmplitude2; }
      if(t>= PulseTime3){ SetPoint = PulseAmplitude3; }
      //SetPoint = PulseAmplitude * sin(2*3.14159*2*t);
      
      Error = (SetPoint - SystemResponse);
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
  Serial.println("plot(t,SetPoint,t,ControllerOutput*120.71,t,SystemResponse,t,Error)");
 Serial.println("grid()");
}

void loop() {
  
}