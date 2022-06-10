#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H
	
	#include <PID_Controller_BuildSettings.h>
	#include "../IIR/IIR.h"

 	namespace DiscreteControllers
    {
    	extern uint64_t(*_GetTime)(void);
    	void Setup(uint64_t(&GetTime)(void) = NULL);

    	//////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//
    	//  PID TransferFunction
		//
		//                   Ki
		//  Gd(s) = Kp + --------- + (Kd * s)
		//                   s
		//
		//                 Ki
		//  Gd(z) = Kp + ------- + (Kd * (z-1))
		//                (z-1)
		//
		//          z^-2 [Kd] + z^-1 [-Kp-2Kd] + z^0 [Kp+Ki+Kd]
		//  Gd(z) = -------------------------------------------
		//                     z^-1 [-1] + z^0 [1]
		//
		// Variable Substitution: 
		//      a0 = Kd, a1 = -Kp-2Kd, a2 = Kp+Ki+Kd 
		//      b0 = -1, b1 = 1
		//
		// Therefore:
		//
		//  z^-2 [a0] + z^-1 [a1] + z^0 [a2]        Out(z)
		//  ----------------------------------- = --------
		//        z^-1 [b0] + z^0 [b1]              In(z)
		//
		// Expanding unsing a cross multiplication of the fractions
		//
		// 	Out(z) * ( z^-1 [b0] + z^0 [b1] ) = In(z) * (z^-2 [a0] + z^-1 [a1] + z^0 [a2])
		// 	Out(z) * [b1] = In(z)*(z^-2)*[a0] + In(z)*(z^-1)*[a1] + In(z)*(z^0)*[a2] - Out(z)*(z^-1)*[b0]
		// 	Out(z) * [1] = In(z)*(z^-2)*[a0] + In(z)*(z^-1)*[a1] + In(z)*(z^0)*[a2] - Out(z)*(z^-1)*[-1]
		//
		// Solving for Out(z)
		//
		// Out(z) = In(z)*(z^-2)*[a0] + In(z)*(z^-1)*[a1] + In(z)*(z^0)*[a2] + Out(z)*(z^-1)
		//
		// Appliying the inverse Z transformation:
		//
		// Out(n) = In(n-2)*[a0] + In(n-1)*[a1] + In(n)*[a2] + Out(n-1) 
		//
		// Substituting the variables:
		//
		// Out(n) = In(n-2)*[Kd] + In(n-1)*[-(Kp)-(2*Kd)] + In(n)*[Kp+Ki+Kd] + Out(n-1) 
		//
		//////////////////////////////////////////////////////////////////////////////////////////////////////////////


        template <class DataType>
        class  PID
        {	
	        private:
	        	IIR<DataType> _Filter;
	        
	    		double _Kp;
	    		double _Ki;
	    		double _Kd;

	    		bool _UpdateMode;
	    		uint64_t _LastUpdateTime;

	        public:

	        	Controller()
	        	{
	        		_Kp = 0;
	        		_Ki = 0;
	        		_Kd = 0;
	        	}

	    		Controller(double Kp, double Ki, double Kd)
	    		{
	    			SetGains(Kp, Ki, Kd);
	    		}

	    		PID& operator=(const Controller& rhs)
	    		{
	    			SetGains(rhs.Kp(), rhs.Ki(), rhs.Kd());
	    			Reset();
	    			return (*this);
	    		}

	    		void SetGains(double Kp, double Ki, double Kd)
	    		{
	    			_Kp = Kp;
	    			_Ki = Ki;
	    			_Kd = Kd;
					
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

	    		void SetKp(double Kp)
	    		{
	    			_Kp = Kp;
					
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

	    		void SetKi(double Ki)
	    		{
	    			_Ki = Ki;
					
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

	    		void SetKd(double Kd)
	    		{
	    			_Kd = Kd;
					
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
        }
    }

#endif//PID_CONTROLLER_H