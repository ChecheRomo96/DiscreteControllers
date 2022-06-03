#ifndef DISCRETE_CONTROLLER_INPUT_SIGNAL_H
#define DISCRETE_CONTROLLER_INPUT_SIGNAL_H

    #include <DiscreteControllers_BuildSettings.h>
	#include "../UpdateManager/UpdateManager.h"

    namespace DiscreteControllers
    {

    	template <class DataType>
    	class DataRange
    	{
	    	private:

	    		DataType _Min;
	    		DataType _Max;

	    	public:

	    		DataRange()
	    		{
	    			_Min = 0;
	    			_Max = 0;
	    		}

	    		DataRange(const DataType& Min, const DataType& Max)
	    		{
	    			_Min = Min;
	    			_Max = Max;
	    		}

    			template <class XDataType>
	    		DataType Interpolate(const DataRange<XDataType>& XRange, const XDataType& eval )
	    		{
	    			return Interpolate(XRange.Min(), XRange.Max(), eval);
	    		}

    			template <class XDataType>
	    		DataType Interpolate(const XDataType& Min, const XDataType& Max, const XDataType& eval )
	    		{
	    			return Min + ((eval-Min)*(Max()-Min()))/(Max-Min);
	    		}

    	}


    	template <class DataType>
    	class InputSignal
    	{
	    	public:

	    		static constexpr bool UpdateMode_Manual = 0;
	    		static constexpr bool UpdateMode_Auto = 1;

	    		static constexpr bool UpdateCallback_Enabled = 1;
	    		static constexpr bool UpdateCallback_Disabled = 0;

	    		static constexpr uint8_t Clock_Timebase_s = 0;
	    		static constexpr uint8_t Clock_Timebase_ms = 1;
	    		static constexpr uint8_t Clock_Timebase_us = 2;

	    	private:
    			DataType(*_Update)();
    			void(*_UpdateCallback)();

	    		float _SamplePeriod;
	    		bool _UpdateMode;
	    		uint64_t _LastUpdateTime;
	    		uint8_t _Clock_Timebase;

	    		DataType _Output;
	    		DataRange<DataType> _OutputRange;
	    		DataRange<DataType> _InputRange;

	    	public:
	    		Signal()
	    		{
	    			_Clock_Timebase = Clock_Timebase_us;

	    		}

	    		Signal(DataType InMin, DataType InMax, DataType OutMin, DataType OutMax, DataType(&Update)(), void(&Callback)() = NULL)
	    		{
	    			_Update = Update;
	    			_UpdateCallback = Callback;
	    			_InputRange = InputRange;
	    			_OutputRange = OutputRange;
	    		}


	    		DataType InMin()
	    		{
	    			return _InMin;
	    		}

	    		DataType InMax()
	    		{
	    			return _InMax;
	    		}

	    		DataType OutMin()
	    		{
	    			return _OutMin;
	    		}

	    		DataType OutMax()
	    		{
	    			return _OutMax;
	    		}

	    		operator=(const Signal& rhs)
	    		{
	    			_Update = rhs.UpdatePointer();
	    			_UpdateCallback = rhs.CallbackPointer();
	    			_InMin = InMin();
	    			_InMax = InMax();
	    			_OutMin = OutMin();
	    			_OutMax = OutMax();

	    		}

	    		DataType(&)() UpdatePointer() const
	    		{
	    			return _Update;
	    		}

	    		void(&)() CallbackPointer() const
	    		{
	    			return _UpdateCallback;
	    		}

	    		void SetSampleFrequency(float Fs)
	    		{
	    			SetSamplePeriod(1.0/Fs); 
	    		}

	    		float SampleFrequency()
	    		{
	    			return 1.0 / _SamplePeriod;
	    		}

	    		void SetSamplePeriod(float SamplePeriod)
	    		{
	    			_SamplePeriod = SamplePeriod;

	    			switch(_Clock_Timebase)
	    			{
	    				case Clock_Timebase_us: _SamplePeriod *= 1000;
	    				case Clock_Timebase_ms: _SamplePeriod *= 1000;
	    				case Clock_Timebase_s: 	break;
	    				default: break;
	    			}
	    		}

	    		float SamplePeriod()
	    		{
	    			return _SamplePeriod;
	    		}

	    		void AppendUpdateMethod(DataType(&Update)())
	    		{
	    			_Update = Update;
	    		}

	    		void DettachUpdateMethod()
	    		{
	    			_Update = NULL;
	    		}

	    		void AppendUpdateCallback(void(&Callback)())
	    		{
	    			_UpdateCallback = Callback();
	    		}

	    		void DettachUpdateCallback()
	    		{
	    			_UpdateCallback = NULL;
	    		}

	    		void Update(uint64_t CurrentTime)
	    		{
	    			
	    		}

	    		void Update()
	    		{
	    			if(_UpdateMode == UpdateMode_Auto)
	    			{

	    			}
	    		}
    	} 
    }

#endif DISCRETE_CONTROLLER_INPUT_SIGNAL_H