#ifndef DISCRETE_CONTROLLERS_TIMING_H
#define DISCRETE_CONTROLLERS_TIMING_H

	#include <DiscreteControllers_BuildSettings.h>

	namespace DiscreteControllers
	{
		namespace Timing
		{
			static constexpr uint8_t TimeBase_Seconds = 0u;
			static constexpr uint8_t TimeBase_Milliseconds = 1u;
			static constexpr uint8_t TimeBase_Microseconds = 2u;
			static constexpr uint8_t TimeBase_Unknown = 3u;

			extern uint64_t(*_ClockUpdate)();
			extern uint8_t ClockTimeBase;
			extern uint64_t CurrentTime;

			void Start();
			void Start(uint64_t(&ClockUpdate)(), uint8_t TimeBase);
			uint64_t UpdateClock();
		}
	}

#endif//DISCRETE_CONTROLLERS_TIMING_H