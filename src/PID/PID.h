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
	//                    Ts    (z+1)		 1     (z-1)
	//  Gd(z) = Kp + Ki ----- ------- + Kd ----- ---------
	//                    2     (z-1)		 Ts      z
	//
	//   							 Ts 				   1
	//  kp' = kp           ki'= ki ------        kd' = kd ---- = kd Fs
	//							     2					   Ts
	//
	//  z^-2 [kd'] + z^-1 [-kp'+ki'-2kd'] + z^0 [kp'+ki'+kd']     Out(z)
	//  ------------------------------------------------------ = --------
	//                z^-1 [0] +z^-1 [-1] + z^0 [1]                In(z)
	//
	// Variable Substitution: 
	//
	//      a0 = Kd'
	//		a1 = -kp'+ki'-2kd'
	//		a2 = kp'+ki'+kd'
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

	    			auto tmp[] = {Kp(), Ki(), Kd()};

	    			_Fs = 1.0/Ts;

	    			SetGains(tmp[0],tmp[1],tmp[2], UpdateTaps);
	    		}

	    		const double Fs() const
	    		{
	    			return _Fs;
	    		}

	    		void SetSamplingFrequency(double Fs, bool UpdateTaps = 0)
	    		{
	    			_Fs = Fs;
	    			
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
	    			return _Ki / ( Ts() / 2.0);
	    		}

	    		void SetKi(double Ki, bool UpdateTaps = 0)
	    		{
	    			_Ki = Ki * ( Ts() / 2.0);

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
	    			double PID_NTaps[] = {_Kd, -_Kp-(2*_Kd), _Kp+_Ki+_Kd};
  					double PID_DTaps[] = {-1, 1};

					CPVector::vector<double> PID_NumTaps;

					PID_NumTaps.resize(3);

					PID_NumTaps[0] = _Kd;
					PID_NumTaps[1] = -_Kp-(2*_Kd);
					PID_NumTaps[2] = _Kp+_Ki+_Kd;

					CPVector::vector<double> PID_DenTaps;

					PID_DenTaps[0] = -1;
					PID_DenTaps[1] = 1;

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