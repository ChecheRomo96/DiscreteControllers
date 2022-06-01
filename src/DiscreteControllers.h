#ifndef DISCRETE_CONTROLLERS_H
#define DISCRETE_CONTROLLERS_H

/////////////////////////////////////////////////////////////////////////////////////////////
//  ____  _                   _           ____            _             _ _               
// |  _ \(_)___  ___ _ __ ___| |_ ___    / ___|___  _ __ | |_ _ __ ___ | | | ___ _ __ ___ 
// | | | | / __|/ __| '__/ _ \ __/ _ \  | |   / _ \| '_ \| __| '__/ _ \| | |/ _ \ '__/ __|
// | |_| | \__ \ (__| | |  __/ ||  __/  | |__| (_) | | | | |_| | | (_) | | |  __/ |  \__ \
// |____/|_|___/\___|_|  \___|\__\___|   \____\___/|_| |_|\__|_|  \___/|_|_|\___|_|  |___/
//
/////////////////////////////////////////////////////////////////////////////////////////////
 
#include "DiscreteControllers_BuildSettings.h"                                                                                      
#include "DiscreteControllers_UserSetup.h"                                                                                      

	namespace DiscreteControllers
	{
		void Start();
		
		void Update();
		void Update(uint64_t CurrentTime);
	}

	#include "FIR/FIR.h"  
	#include "IIR/IIR.h"        

#endif//DISCRETE_CONTROLLERS_H