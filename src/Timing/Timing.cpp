#include "Timing.h"

uint64_t(*DiscreteControllers::Timing::_ClockUpdate)()
uint8_t DiscreteControllers::Timing::ClockTimeBase;
uint64_t DiscreteControllers::Timing::CurrentTime;

void DiscreteControllers::Timing::Start(uint64_t(&ClockUpdate)(), uint8_t TimeBase)
{
	
}

uint64_t DiscreteControllers::Timing::UpdateClock()
{
	
}