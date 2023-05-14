#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H
	
	#include <DiscreteControllers_BuildSettings.h>
	#include "../IIR/IIR.h"
	
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	//  Example with Trapezoidal Intrgratior
	//
	//                   Ki
	//  Gd(s) = Kp + --------- + (Kd * s)
	//                   s
	//
	//                       (z)		 1     (z-1)
	//  Gd(z) = Kp + Ki Ts ------- + Kd ----- ---------
	//                      (z-1)		 Ts      z
	//
	//
	//  z^-2 [Kd / Ts] + z^-1 [-Kp + Ki - 2(Kd/Ts)] + z^0 [Kp + (Kd/Ts)]    Out(z)
	//  ---------------------------------------------------------------- = --------
	//                z^-2 [0] + z^-1 [-1] + z^0 [1]                	    In(z)
	//
	//
	//   							 				   1
	//  Kp' = Kp           Ki'= Ki Ts        Kd' = Kd ---- = Kd Fs
	//							    				   Ts
	//
	//
	//    z^-2 [Kd'] + z^-1 [-Kp'+Ki'-2Kd'] + z^0 [Kp'+ Kd']       Out(z)
	//  ------------------------------------------------------ = --------
	//                z^-2 [0] + z^-1 [-1] + z^0 [1]                In(z)
	//
	//
	//
	// Variable Substitution: 
	//
	//      a0 = Kd'
	//		a1 = -Kp' + Ki' - 2Kd'
	//		a2 = Kp' + Ki' + Kd'
	//
	//      b0 = 0
	//		b1 = -1
	//		b2 = 1
	//
	// Therefore:
	//
	//  z^-2 [a0] + z^-1 [a1] + z^0 [a2]        Out(z)
	//  ----------------------------------- = --------
	//  z^-2 [b0] + z^-1 [b1] + z^0 [b2]        In (z)
	//
	// Out(n) = In(n-2)*[Kd] + In(n-1)*[-(Kp)-(2*Kd)] + In(n)*[Kp+Ki+Kd] + Out(n-1) 
	//
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////
 	
 	namespace DiscreteControllers
    {

        template <class DataType>
        class  PID
        {	
	        public: 
	        
	        	static constexpr uint8_t IntegratoMethod_Trapezoidal = 0;
	        
	        private:
	        	IIR<DataType> _Filter;
	        
	    		double _Kp;
	    		double _Ki;
	    		double _Kd;
	    		double _Fs;

	    		bool _UpdateMode;
	    		uint64_t _LastUpdateTime;

	        public:

	        	PID()
	        	{
	        		_Kp = 0;
	        		_Ki = 0;
	        		_Kd = 0;

	        		_Fs = 0.1;
	        	}

	    		PID(double Kp, double Ki, double Kd, double Ts)
	    		{
	    			SetSamplingPeriod(Ts, 0);
	    			SetGains(Kp, Ki, Kd, 0);
					RecalculateTaps();
	    		}

	    		PID& operator=(const PID& rhs)
	    		{
	    			SetGains(rhs.Kp(), rhs.Ki(), rhs.Kd());
	    			Reset();
	    			return (*this);
	    		}

	    		void SetGains(double Kp, double Ki, double Kd, bool UpdateTaps = 0)
	    		{
	    			SetKp(Kp,0);
	    			SetKi(Ki,0);
	    			SetKd(Kd,0);

					if(UpdateTaps){RecalculateTaps();}
	    		}

	    		const double Ts() const
	    		{
	    			return 1.0/_Fs;
	    		}

	    		void SetSamplingPeriod(double Ts, bool UpdateTaps = 0)
	    		{

	    			double tmp[] = {Kp(), Ki(), Kd()};
	    			_Fs = 1.0/Ts;
	    			SetGains(tmp[0],tmp[1],tmp[2]);
	    			
					if(UpdateTaps){RecalculateTaps();}
	    		}

	    		const double Fs() const
	    		{
	    			return _Fs;
	    		}

	    		void SetSamplingFrequency(double Fs, bool UpdateTaps = 0)
	    		{
	    			double tmp[] = {Kp(), Ki(), Kd()};
	    			_Fs = Fs;
	    			SetGains(tmp[0],tmp[1],tmp[2]);
	    			
					if(UpdateTaps){RecalculateTaps();}
	    		}

	    		const double Kp() const
	    		{
	    			return _Kp;
	    		}

	    		void SetKp(double Kp, bool UpdateTaps = 0)
	    		{
	    			_Kp = Kp;

					if(UpdateTaps){RecalculateTaps();}
	    		}

	    		const double Ki() const
	    		{
	    			return _Ki / Ts();
	    		}

	    		void SetKi(double Ki, bool UpdateTaps = 0)
	    		{
	    			_Ki = Ki * Ts();

					if(UpdateTaps){RecalculateTaps();}
	    		}

	    		const double Kd() const
	    		{
	    			return _Kd / Fs();
	    		}

	    		void SetKd(double Kd, bool UpdateTaps = 0)
	    		{
	    			_Kd = Kd * Fs();

					if(UpdateTaps){RecalculateTaps();}
	    		}

	    		void RecalculateTaps()
	    		{
					CPVector::vector<double> PID_NumTaps;

					PID_NumTaps.resize(3);

					PID_NumTaps[0] = -_Kd;
					PID_NumTaps[1] = _Kp-_Ki+(2*_Kd);
					PID_NumTaps[2] = -_Kp-_Kd;

					Serial.println();
					Serial.print("Kp = ");
					Serial.print(Kp());
					Serial.print(", Ki = ");
					Serial.print(Ki());
					Serial.print(", Kd = ");
					Serial.println(Kd());

					Serial.print("Kp' = ");
					Serial.print(_Kp);
					Serial.print(", Ki' = ");
					Serial.print(_Ki);
					Serial.print(", Kd' = ");
					Serial.println(_Kd);

					Serial.print("z^-2 [");
					Serial.print(PID_NumTaps[0]);
					Serial.print("] + z^-1 [");
					Serial.print(PID_NumTaps[1]);
					Serial.print("] + z^0 [");
					Serial.print(PID_NumTaps[2]);
					Serial.println("]");

					Serial.println("-----------------------------------------------");

					Serial.print("z^-1 [");
					Serial.print(PID_DenTaps[0]);
					Serial.print("] + z^0 [");
					Serial.print(PID_DenTaps[1]);
					Serial.println("]");

					CPVector::vector<double> PID_DenTaps;

					PID_DenTaps.resize(2);

					PID_DenTaps[0] = 1;
					PID_DenTaps[1] = -1;

 		 			_Filter.SetCoefficients(PID_NumTaps,PID_DenTaps);
	    		}

	    		void Reset()
	    		{
	    			_Filter.InitializeInputBuffer();
	    			_Filter.InitializeOutputBuffer();
	    		}

	    		DataType Update(DataType Input)
	    		{
	    			return _Filter.Update(Input);
	    		}
        };
    }

#endif//PID_CONTROLLER_H