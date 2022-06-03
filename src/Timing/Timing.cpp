#include "Timing.h"

uint64_t(*DiscreteControllers::Timing::_ClockUpdate)()
uint8_t DiscreteControllers::Timing::ClockTimeBase;
uint64_t DiscreteControllers::Timing::CurrentTime;

void DiscreteControllers::Timing::Start()
{
	DiscreteControllers::Timing::_ClockUpdate = NULL;
	DiscreteControllers::Timing::ClockTimeBase = DiscreteControllers::Timing::TimeBase_Unknown;
}

void DiscreteControllers::Timing::Start(uint64_t(&ClockUpdate)(), uint8_t TimeBase)
{
	_ClockUpdate = 0;
	ClockTimeBase = TimeBase;
}

uint64_t DiscreteControllers::Timing::UpdateClock()
{
	CurrentTime = 0;

	if(_ClockUpdate != NULL)
	{
		CurrentTime = _ClockUpdate();
	}

	return CurrentTime;
}