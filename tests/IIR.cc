#if defined(ARDUINO)
    #include <Aunit.h>
    #include <aunit/contrib/gtest.h>
#endif

#if __has_include(<gtest/gtest.h>)
    #include <gtest/gtest.h>
#endif

#include <DiscreteControllers.h>

TEST(DiscreteControllers_OperationTesting, case1){

    float alpha = 0.5;

    DiscreteControllers::IIR<float, float, float> Filter({ 1-alpha , alpha}, { 1, - alpha });

    cpstd::vector<double> input_signal = {2, 4, 7, 10, 13, 16, 13, 10, 7, 4, 2};  // Input signal
    cpstd::vector<double> expected_output = { 1, 1.5, 2.75, 3.625, 4.6875, 5.65625, 3.67188, 3.16406, 1.91797, 1.04102, 0.479492 };

    for(size_t i = 0; i < input_signal.size(); i++){
        ASSERT_EQ(expected_output[i], Filter.Update(input_signal[i]));
    }
}