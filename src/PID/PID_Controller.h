#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H
	
	#include <PID_Controller_BuildSettings.h>
	#include "PID_Controller_Signal.h"

 	namespace PID
    {
    	extern uint64_t(*_GetTime)(void);
    	void Setup(uint64_t(&GetTime)(void) = NULL);

        template <class SetPointDataType, class FeedbackDataType, class OutputDatatype>
        class  Controller
        {	
	        public:
	    		static constexpr bool UpdateMode_Manual = 0;
	    		static constexpr bool UpdateMode_Auto = 1;

	    		static constexpr bool UpdateCallback_Enabled = 1;
	    		static constexpr bool UpdateCallback_Disabled = 0;

	        private:
	        	InputSignal<SetPointDataType>& _SetPoint;
	        	InputSignal<FeedbackDataType>& _Feedback;
	        	OutputSignal<OutputDatatype>& _Output;
	        
	    		float _Kp;
	    		float _Ki;
	    		float _Kd;

	    		float _SampleFrequency;
	    		bool _UpdateMode;
	    		uint64_t _LastUpdateTime;

	        public:

	        	Controller();
	    		Controller(const Signal& SetPoint, const Signal& Error, const Signal& Output);
	    		operator=(const Controller& rhs);

	    		void SetGains(float Kp, float Ki, float Kd);
	    		void Update(uint64_t CurrentTime);
	    		void Update();
        }
    }

#endif//PID_CONTROLLER_H