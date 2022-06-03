#include "DiscreteControllers.h"

void DiscreteControllers::Start()
{
	#if defined ARDUINO
		DiscreteControllers::Timing::Start(micros, Timing::Timebase_Microseconds);
	#else
		DiscreteControllers::Timing::Start();
	#endif
}

void DiscreteControllers::Update()
{

}

void DiscreteControllers::Update(uint64_t CurrentTime)
{

}