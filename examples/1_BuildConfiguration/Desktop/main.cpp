#include <CPSTL.h>
#include <DiscreteControllers.h>
#include <iostream>

int main(){
   
    std::cout << "Discrete Controllers Version: " << DISCRETE_CONTROLLERS_VERSION << std::endl;
    std::cout << " - Using CPSTL version: " << CPSTL_VERSION << std::endl;
    
    
    float alpha = 0.5;

    DiscreteControllers::IIR<float, float, float> Filter({ 1 - alpha , alpha }, { 1, -alpha });

    cpstd::vector<double> input_signal = { 2, 4, 7, 10, 13, 16, 13, 10, 7, 4, 2 };  // Input signal
    cpstd::vector<double> expected_output = { 1, 1.5, 2.75, 3.625, 4.6875, 5.65625, 3.67188, 3.16406, 1.91797, 1.04102, 0.479492 };

    for (size_t i = 0; i < input_signal.size(); i++) {
        std::cout<<(expected_output[i])<<", "<< (Filter.Update(input_signal[i]))<<std::endl;
    }
    
    
    
    
    return 0;


}
