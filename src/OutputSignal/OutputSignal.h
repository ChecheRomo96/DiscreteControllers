#ifndef DISCRETE_CONTROLLER_OUTPUT_SIGNAL_H
#define DISCRETE_CONTROLLER_OUTPUT_SIGNAL_H

    #include <DiscreteControllers_BuildSettings.h>

    namespace PID
    {

    	template <class DataType>
    	class OutputSignal
    	{
	    	public:

	    		static constexpr bool UpdateMode_Manual = 0;
	    		static constexpr bool UpdateMode_Auto = 1;

	    		static constexpr bool UpdateCallback_Enabled = 1;
	    		static constexpr bool UpdateCallback_Disabled = 0;

	    	private:
    			DataType(*_Update)();
    			void(*_UpdateCallback)();

	    		float _InputRange_Low = 0;
	    		float _InputRange_High = 1;

	    		float _OutputRange_Low = 0;
	    		float _OutputRange_High = 1;

	    	public:
	    		Signal();
	    		Signal(DataType(&_Update)());
	    		operator=(const Signal& rhs);

	    		void AppendUpdateMethod(DataType(&Update)());
	    		void DettachUpdateMethod();

	    		void AppendUpdateCallback(void(&Callback)());
	    		void DettachUpdateCallback();



	    		void Update(float newValue);
	    		void Update();
    	} 

    }

#endif DISCRETE_CONTROLLER_OUTPUT_SIGNAL_H