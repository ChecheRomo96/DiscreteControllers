add_executable( DiscreteControllers_IIR IIR.cc ) 
target_link_libraries( DiscreteControllers_IIR PUBLIC DiscreteControllers PUBLIC GTest::gtest_main )
gtest_discover_tests(DiscreteControllers_IIR)