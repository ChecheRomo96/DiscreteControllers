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

			extern uint32_t(*_ClockUpdate)();
			extern uint8_t ClockTimeBase;
			extern uint32_t CurrentTime;

			void Start(uint32_t(&ClockUpdate)(), uint8_t TimeBase);
			uint32_t UpdateClock();
		}
	}

#endif//DISCRETE_CONTROLLERS_TIMING_H