/**
 * @file PID.h
 * @brief Infinite Impulse Response Class
 */
#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H
	
	#include <DiscreteControllers_BuildSettings.h>
	#include "../IIR/IIR.h"
 	
 	namespace DiscreteControllers
    {
    	/**
		 * @brief Discrete PID Class. \n\n
		 * A proportional–integral–derivative controller (PID controller ) 
		 * is a control loop mechanism employing feedback
		 * that is widely used in industrial control systems and a variety of other 
		 * applications requiring continuously modulated control. A PID controller 
		 * continuously calculates an error value e(t) as the difference between a 
		 * desired setpoint (SP) and a measured process variable (PV) and applies a 
		 * correction based on proportional, integral, and derivative terms 
		 * (denoted P, I, and D respectively), hence the name.
		 *
		 * This implementation uses a parallel continuous PID as the base model,
		 * whose transfer function is:\n\n
		 * \f$ G(s) = K_p +  \frac{K_i}{s} + K_{d} s\f$\n\n
		 * 
		 * | Billinear | Backwards Euler |
		 * | :--------:|:---------------:| 
		 * | \f$ \displaystyle  s = \frac{2}{Ts} \left( \frac{z-1}{z+1} \right) \f$ | \f$ \displaystyle  s = \frac{1}{Ts} \left( \frac{z-1}{z} \right) \f$ |
		 *
		 * When using the billinear transformation to discretize a PID controller, it yields
		 * the following transfer function in the z domain: \n\n
		 * \f$ \displaystyle
		 *   G\left[ z \right] = \frac{z^2 \left[ K_p' + K_i' + K_d' \right] + z^1 \left[ 2 K_i' - 2 K_d' \right] + z^0 \left[ - K_p' + K_i' + K_d' \right] }{ z^2\left[1\right] + z^1\left[0\right] + z^0\left[-1\right] }
		 * \f$ \n\n
		 * where:\n\n
		 * \f$ \displaystyle K_p' = K_p \f$\n\n
		 * \f$ \displaystyle K_i' = \frac{K_i Ts}{2} \f$\n\n
		 * \f$ \displaystyle K_d' = \frac{2K_d}{Ts} \f$ \n\n
		 * When using the backward euler method to discretize a PID controller, it yields
		 * the following transfer function in the z domain: \n\n
		 * \f$ \displaystyle
		 *   G\left[ z \right] = \frac{z^2 \left[ K_p' + K_i' + K_d' \right] + z^1 \left[ - K_p' - 2 K_d' \right] + z^0 \left[ K_d' \right] }{ z^2\left[1\right] + z^1\left[-1\right] + z^0\left[0\right] }
		 * \f$ \n\n
		 * where:\n\n
		 * \f$ \displaystyle K_p' = K_p \f$\n\n
		 * \f$ \displaystyle K_i' = K_i Ts \f$\n\n
		 * \f$ \displaystyle K_d' = \frac{K_d}{Ts} \f$\n\n
		 * @tparam DataType Data type to be used as input and output signals
		 */
        template <typename InputType, typename OutputType>
        class  PID
        {	
	        public: 

	        	/**
	        	 * @brief Static atribute to be used on SetDiscretizationMode()
	        	 * 
	        	 * Sets the discretization mode to backwards euler. This is the default mode of discretization for this class.
	        	 * When using the backward euler method to discretize a PID controller, it yields
				 * the following transfer function in the z domain: \n\n
				 * \f$ \displaystyle
				 *   G\left[ z \right] = \frac{z^2 \left[ K_p' + K_i' + K_d' \right] + z^1 \left[ - K_p' - 2 K_d' \right] + z^0 \left[ K_d' \right] }{ z^2\left[1\right] + z^1\left[-1\right] + z^0\left[0\right] }
				 * \f$ \n\n
				 * where:\n\n
				 * \f$ \displaystyle K_p' = K_p \f$\n\n
				 * \f$ \displaystyle K_i' = K_i Ts \f$\n\n
				 * \f$ \displaystyle K_d' = \frac{K_d}{Ts} \f$\n\n
	        	 */
	        	static constexpr uint8_t DiscretizationMode_BE = 0;
	        	
	        	/**
	        	 * @brief Static atribute to be used on SetDiscretizationMode()
	        	 * 
	        	 * Sets the discretization mode to backwards euler. This is the default mode of discretization for this class.
	        	 * When using the billinear transformation to discretize a PID controller, it yields
				 * the following transfer function in the z domain: \n\n
				 * \f$ \displaystyle
				 *   G\left[ z \right] = \frac{z^2 \left[ K_p' + K_i' + K_d' \right] + z^1 \left[ 2 K_i' - 2 K_d' \right] + z^0 \left[ - K_p' + K_i' + K_d' \right] }{ z^2\left[1\right] + z^1\left[0\right] + z^0\left[-1\right] }
				 * \f$ \n\n
				 * where:\n\n
				 * \f$ \displaystyle K_p' = K_p \f$\n\n
				 * \f$ \displaystyle K_i' = \frac{K_i Ts}{2} \f$\n\n
				 * \f$ \displaystyle K_d' = \frac{2K_d}{Ts} \f$ \n\n
	        	 */
	        	static constexpr uint8_t DiscretizationMode_BLT = 1;
	        	
	        	/**
	        	 * @brief Static atribute to be used on SetDiscretizationMode()
	        	 * 
	        	 * It is impossible to assign this mode to an object, but it is used to check on the validity
	        	 * of the input parameter of SetDiscretizationMode().
				 *
	        	 */
	        	static constexpr uint8_t DiscretizationMode_Invalid = 2;

	        private:
	        	IIR<InputType, OutputType, double> _Filter;
	        
	    		double _Kp;
	    		double _Ki;
	    		double _Kd;
	    		double _Fs;

	    		uint8_t _DiscretizationMode;
	    		bool _UpdateMode;
	    		uint64_t _LastUpdateTime;

	        public:

	        	/**
	        	 * @brief Default Constructor.
	        	 * 
	        	 * Initializes \f$ K_p \f$, \f$ K_i \f$, and \f$ K_d \f$ to 0. 
	        	 * Discretization Mode initialized to Backward Euler (PID::DiscretizationMode_BE).
	        	 */
	        	PID(){
	        		_DiscretizationMode = DiscretizationMode_BE;

	        		_Kp = 0;
	        		_Ki = 0;
	        		_Kd = 0;

	    			SetSamplingPeriod(0.01, 1);
	        	}

	        	/**
	        	 * @brief Constructor with initialization.
	        	 * 
	        	 * Sets \f$ K_p \f$, \f$ K_i \f$, and \f$ K_d \f$. Sets the sampling Period.
	        	 * Discretization Mode initialized to Backward Euler (PID::DiscretizationMode_BE).
	        	 */
	    		PID(double Kp, double Ki, double Kd, double Ts)
	    		{
	        		_DiscretizationMode = DiscretizationMode_BE;

	    			SetSamplingPeriod(Ts, 0);
	    			SetGains(Kp, Ki, Kd, 0);
					RecalculateTaps();
	    		}

				/**
				 * @brief Assignment operator overload.
				 * @param rhs Reference object 
				 * @return Reference to the target object
				 */
	    		PID& operator=(const PID& rhs)
	    		{
	    			SetGains(rhs.Kp(), rhs.Ki(), rhs.Kd());
	    			Reset();
	    			return (*this);
	    		}

	        	/**
	        	 * @brief Selects the discretization mode used.
	        	 * 
	        	 * Select which disctretization mode is to be used, the available modes are:\n
	        	 * - PID::DiscretizationMode_BE  (Backward Euler Method)\n
	        	 * - PID::DiscretizationMode_BLT (Billinear Transformation\n
	        	 */
	    		void SetDiscretizationMode(uint8_t Mode)
	    		{
	    			if(Mode < DiscretizationMode_Invalid)
	    			{
	    				_DiscretizationMode = Mode;
	    			}
	    		}

	        	/**
	        	 * @brief Returns the current discretization mode selected.
	        	 * @return discretization mode selected.
	        	 */
	    		const uint8_t DiscretizationMode() const
	    		{
	    			return _DiscretizationMode;
	    		}

	        	/**
	        	 * @brief Set the PID gains used by the object.
	        	 * @param Kp Proportional Gain
	        	 * @param Ki Integral Gain
	        	 * @param Kd Derivative Gain
	        	 * @param UpdateTaps 1 to update the transfer function, 0 to not
	        	 */
	    		void SetGains(double Kp, double Ki, double Kd, bool UpdateTaps = 0)
	    		{
	    			SetKp(Kp,0);
	    			SetKi(Ki,0);
	    			SetKd(Kd,0);

					if(UpdateTaps){RecalculateTaps();}
	    		}

	        	/**
	        	 * @brief Returns the current sampling period.
	        	 * @return Sampling period.
	        	 */
	    		const double Ts() const
	    		{
	    			return 1.0/_Fs;
	    		}

	        	/**
	        	 * @brief Set the expected sampling period of the input signal.
	        	 * @param Ts Sampling period in seconds
	        	 * @param UpdateTaps 1 to update the transfer function, 0 to not
	        	 */
	    		void SetSamplingPeriod(double Ts, bool UpdateTaps = 0)
	    		{

	    			double tmp[] = {Kp(), Ki(), Kd()};
	    			_Fs = 1.0/Ts;
	    			SetGains(tmp[0],tmp[1],tmp[2]);
	    			
					if(UpdateTaps){RecalculateTaps();}
	    		}

	        	/**
	        	 * @brief Returns the current sampling frequency.
	        	 * @return Sampling frecuency.
	        	 */
	    		const double Fs() const
	    		{
	    			return _Fs;
	    		}

	        	/**
	        	 * @brief Set the expected sampling frequency of the input signal.
	        	 * @param Fs Sampling frequency in Hz
	        	 * @param UpdateTaps 1 to update the transfer function, 0 to not
	        	 */
	    		void SetSamplingFrequency(double Fs, bool UpdateTaps = 0)
	    		{
	    			double tmp[] = {Kp(), Ki(), Kd()};
	    			_Fs = Fs;
	    			SetGains(tmp[0],tmp[1],tmp[2]);
	    			
					if(UpdateTaps){RecalculateTaps();}
	    		}

	        	/**
	        	 * @brief Returns the current value of the proportional gain.
	        	 * @return Proportional gain.
	        	 */
	    		const double Kp() const
	    		{
					switch(_DiscretizationMode)
					{
						case DiscretizationMode_BE: return _Kp; 
						case DiscretizationMode_BLT: return _Kp; 
						default: return 0;
					}
	    		}

	        	/**
	        	 * @brief Set the proportional gain to be used by the object.
	        	 * @param Kp Proportional gain.
	        	 * @param UpdateTaps 1 to update the transfer function, 0 to not
	        	 */
	    		void SetKp(double Kp, bool UpdateTaps = 0)
	    		{
					switch(_DiscretizationMode)
					{
						case DiscretizationMode_BE: _Kp = Kp; break;
						case DiscretizationMode_BLT: _Kp = Kp; break;
						default: return;
					}

					if(UpdateTaps){RecalculateTaps();}
	    		}

	        	/**
	        	 * @brief Returns the current value of the integral gain.
	        	 * @return Integral gain.
	        	 */
	    		const double Ki() const
	    		{
					switch(_DiscretizationMode)
					{
						case DiscretizationMode_BE: return _Ki * Fs(); 
						case DiscretizationMode_BLT: return _Ki * (2*Fs()); 
						default: return 0;
					}
	    		}

	        	/**
	        	 * @brief Set the integral gain to be used by the object.
	        	 * @param Ki Integral gain.
	        	 * @param UpdateTaps 1 to update the transfer function, 0 to not
	        	 */
	    		void SetKi(double Ki, bool UpdateTaps = 0)
	    		{
					switch(_DiscretizationMode)
					{
						case DiscretizationMode_BE: _Ki = Ki / Fs(); break;
						case DiscretizationMode_BLT: _Ki = Ki / (2.0 * Fs()); break;
						default: return;
					}

					if(UpdateTaps){RecalculateTaps();}
	    		}

	        	/**
	        	 * @brief Returns the current value of the derivative gain.
	        	 * @return Derivative gain.
	        	 */
	    		const double Kd() const
	    		{
					switch(_DiscretizationMode)
					{
						case DiscretizationMode_BE: return _Kd / Fs(); 
						case DiscretizationMode_BLT: return _Kd / (2*Fs());
						default: return;
					}
	    		}

	        	/**
	        	 * @brief Set the derivative gain to be used by the object.
	        	 * @param Kd Derivative gain.
	        	 * @param UpdateTaps 1 to update the transfer function, 0 to not
	        	 */
	    		void SetKd(double Kd, bool UpdateTaps = 0)
	    		{
					switch(_DiscretizationMode)
					{
						case DiscretizationMode_BE: _Kd = Kd * Fs(); break;
						case DiscretizationMode_BLT: _Kd = 2 * Kd * Fs(); break;
						default: return;
					}

					if(UpdateTaps){RecalculateTaps();}
	    		}


	        	/**
	        	 * @brief Recalculates the transfer function coefficients.
	        	 *
	        	 * Recalculates the transfer function coefficients based on the parameters (\f$ K_p \f$,\f$ K_i \f$, \f$ K_d \f$,\f$ Ts \f$) and the selected discretization mode.
	        	 */
	    		void RecalculateTaps()
	    		{
					cpstd::vector<double> PID_NumTaps;
					cpstd::vector<double> PID_DenTaps;

					switch(_DiscretizationMode)
					{
						case DiscretizationMode_BE:
							PID_NumTaps.resize(3);

							PID_NumTaps[0] = _Kp+_Ki+_Kd;
							PID_NumTaps[1] = -_Kp-(2*_Kd);
							PID_NumTaps[2] = _Kd;

							PID_DenTaps.resize(3);

							PID_DenTaps[0] = 1;
							PID_DenTaps[1] = -1;
							PID_DenTaps[2] = 0;
						break;

						case DiscretizationMode_BLT:
							PID_NumTaps.resize(3);

							PID_NumTaps[0] = _Kp+_Ki+_Kd;
							PID_NumTaps[1] = (2*_Ki)-(2*_Kd);
							PID_NumTaps[2] = -_Kp+_Ki+_Kd;

							PID_DenTaps.resize(3);

							PID_DenTaps[0] = 1;
							PID_DenTaps[1] = 0;
							PID_DenTaps[2] = -1;
						break;

						default: return;
					}

 		 			_Filter.SetCoefficients(PID_NumTaps,PID_DenTaps);
	    		}

	        	/**
	        	 * @brief Resets the input and output buffer.
	        	 */
	    		void Reset()
	    		{
	    			_Filter.InitializeInputBuffer();
	    			_Filter.InitializeOutputBuffer();
	    		}

				/**
				 * @brief Recieves a new data point and calculates the transfer function output using the stored data.
				 * @param ErrorSignal Numerical value of the error signal.
				 * @return Numerical value of the control signal
				 */
	    		OutputType& Update(const InputType& ErrorSignal)
	    		{
	    			return _Filter.Update(ErrorSignal);
	    		}
        };
    }

#endif//PID_CONTROLLER_H