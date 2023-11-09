/**
 * @file IIR.h
 * @brief Infinite Impulse Response Class
 */

#ifndef DISCRETE_CONTROLLERS_IIR_H
#define DISCRETE_CONTROLLERS_IIR_H                                  

	#include <DiscreteControllers_BuildSettings.h>
	#include <CPvector.h>

	namespace DiscreteControllers
	{
		/**
		 * @brief Infinite Impulse Response System Class \n
		 * An Infinite Impulse Response (IIR) system is a type of digital or analog signal processing system commonly used 
		 * in various applications like filtering, control systems, and telecommunications. It is characterized by its ability 
		 * to have a feedback loop within its design, which results in an infinite impulse response to an input signal
		 * 
		 * # IIR (Infinite Impulse Response) System Class
		 * In engineering, a transfer function of a system, 
		 * sub-system, or component is a mathematical function that 
		 * theoretically models the system's output for each possible input. \n\n
		 * 
		 * Analog electronic filters are predominantly IIR (Infinite Impulse Response), whereas digital filters can be either IIR or FIR (Finite Impulse Response). 
		 * The presence of feedback in the topology of a discrete-time filter, as depicted in the block diagram below, typically generates an IIR response. 
		 * The transfer function of an IIR filter in the z domain contains a non-trivial denominator, encapsulating feedback terms. 
		 * On the contrary, the transfer function of an FIR filter only encompasses a numerator.\n\n
		 * 
		 * Transfer functions associated with IIR analog electronic filters have undergone extensive study and optimization for their 
		 * amplitude and phase characteristics. 
		 * These continuous-time filter functions are described in the Laplace domain. Desired solutions are then translated to 
		 * discrete-time filters, whose transfer functions are expressed in the z domain, using various mathematical techniques such as the 
		 * bilinear transform, impulse invariance, or pole–zero matching method. Consequently, digital IIR filters can be built based on 
		 * established analog filter solutions, like the Chebyshev filter, Butterworth filter, and elliptic filter, inheriting their characteristics.\n\n
		 * A discrete system can be describe with it's tranfer function, below is the generalized form of a discrete transfer function, and below it is also present the difference equation 
		 * , which defines how the output signal is related to the input signal. , and to the right is a generalized block diagram representing IIR filters. \n\n
		 * @htmlonly
		 * <div style="display: flex; align-items: center; justify-content: center;">
		 *     <div style="width: 30%; text-align: center;">
		 * @endhtmlonly
		 * 			\n\n  
		 *         	\f$ H(z) = \frac{Y\left[z\right]}{X\left[z\right]} = \frac{\sum_{i=0}^{P} b_{i}z^{-i}}{\sum_{j=0}^{Q} a_{j}z^{-j}} \f$ \n\n
		 * 			\n\n
		 * 			\f$  y[n] = \frac{1}{a_0} \left( \sum_{i=0}^{P} b_ix[n-i] - \sum_{j=1}^{Q} a_jx[n-j] \right) \f$ \n\n
		 * @htmlonly
		 *     </div>
		 *     <div style="width: 30%; text-align: center;">
		 * @endhtmlonly
		 * 			where:\n\n 
		 * 			\f$ P \f$ is the feedforward filter order \n\n
		 * 			\f$ b_i \f$ are the feedforward filter coefficients \n\n
		 * 			\f$ Q \f$ is the feedback filter order \n\n
		 * 			\f$ a_j \f$ are the feedback filter coefficients \n\n
		 * 			\f$ x \left[ n \right] \f$ is the input signal \n\n
		 * 			\f$ y \left[ n \right] \f$ is the output signal \n\n
		 * @htmlonly
		 *     </div>
		 *     <div style="width: 30%; text-align: center;">
		 *         <img src="IIR.png" style="max-width:80%; height:auto;">
		 *     </div>
		 * </div>
		 * @endhtmlonly
		 * 

		 *
		 * Recursive filters are an efficient way of achieving a long impulse response, 
		 * without having to perform a long convolution. They execute very rapidly, 
		 * but have less performance and flexibility than other digital filters. 
		 * Recursive filters are also called Infinite Impulse Response (IIR) filters, 
		 * since their impulse responses are composed of decaying exponentials. 
		 * This distinguishes them from digital filters carried out by convolution, 
		 * called Finite Impulse Response (FIR) filters.
		 *
		 * ## Example Code
		 *
		 * Below is a simple example that demonstrates the usage of this class.This example implements 
		 * the discrete system with the transfer function described below (Note that it was discretized 
		 * via the billinear transformation): \n\n
		 * \f$  
		 *   G(s) = \left( \frac{1}{0.5 s + 1} \right)
		 *   \quad \rightarrow \quad
		 *   G(z) = \left( \frac{0.0198}{z-0.9802} \right), 
		 *   \{ f_s = 100Hz, Ts = 0.01 s \}
		 * \f$
		 * 
		 * @code{.cpp}
		 * #include <DiscreteControllers.h>
		 *
		 * // Transfer Function (Ts = 0.01): 
		 * //   0.0198 / (z - 0.9802)
		 *
		 * void main()
		 * {
		 *     DiscreteControllers::IIR<float> myIIR;
		 *     
		 *     cpstd::vector<double> NumCoeffs;
		 *     NumCoeffs.resize(1);
		 *     NumCoeffs[0] = 0.0198;
		 *
		 *     cpstd::vector<double> DenCoeffs;
		 *     DenCoeffs.resize(2);
		 *     DenCoeffs[0] = 1;
		 *     DenCoeffs[1] = -0.9802;
		 *     
		 *     myIIR.SetCoefficients(NumCoeffs, DenCoeffs);
		 *     
		 *     float newInput = 1; 
		 *     float systemResponse = myIIR.Update(newInput);
		 * }
		 * @endcode
		 * @tparam DataType Data type to be used for the input and output signals
		 * @tparam CoefficientType Data type to be used as input and output signals
		 */

		template<typename InputType, typename OutputType, typename CoefficientType = double>
		class IIR
		{
			private:
		
				cpstd::vector<CoefficientType> _NumCoefficients;
				cpstd::vector<CoefficientType> _DenCoefficients;
				cpstd::vector<InputType> _InputBuffer;
				cpstd::vector<OutputType> _OutputBuffer;
				uint8_t _InputIndex;
				uint8_t _OutputIndex;

			public:
				/**
				 * @brief Default Class Constructor. 
				 */
				IIR()
				{
					static_assert(cpstd::is_arithmetic<CoefficientType>::value, "CoefficientType must be an arithmetic type");
					static_assert(cpstd::is_arithmetic<InputType>::value, "InputType must be an arithmetic type");
					static_assert(cpstd::is_arithmetic<OutputType>::value, "OutputType must be an arithmetic type");
					
					_InputIndex = 0;
					_OutputIndex = 0;
				}

				/**
				 * @brief Class Constructor with initializer, where: 
				 * NumCoeffs = \f$ \left[ b_0, b_1, \cdots , b_P \right] \f$, and: 
				 * DenCoeffs = \f$ \left[ a_0, a_1, \cdots , a_Q \right] \f$
				 *
				 * Coefficients are from the following representation of a discrete system:\n\n
				 * \f$ \large  
				 *    H(z) = 
				 *    \frac{\sum_{i=0}^{P} b_{i}z^{-i}}{\sum_{j=0}^{Q} a_{j}z^{-j}}
				 * \f$ \n\n
				 * 
				 * For more info see IIR.
				 * @param NumCoeffs Numerator coefficients of the discrete transfer function. Expects a vector containing the 
				 * @param DenCoeffs Denominator coefficients of the discrete transfer function. 
				 */
				IIR(const cpstd::vector<CoefficientType>& NumCoeffs, const cpstd::vector<CoefficientType>& DenCoeffs) : IIR() {
					SetCoefficients(NumCoeffs, DenCoeffs);
				}

				/**
				 * @brief Class Constructor with initializer, where: 
				 * NumCoeffs = \f$ \left[ b_0, b_1, \cdots , b_P \right] \f$, and: 
				 * DenCoeffs = \f$ \left[ a_0, a_1, \cdots , a_Q \right] \f$
				 *
				 * Coefficients are from the following representation of a discrete system:\n\n
				 * \f$ \large  
				 *    H(z) = 
				 *    \frac{\sum_{i=0}^{P} b_{i}z^{-i}}{\sum_{j=0}^{Q} a_{j}z^{-j}}
				 * \f$ \n\n
				 * 
				 * This version of SetCoefficients recieve an rvalue reference to the input 
				 * vectors, this way move semantics are enabled and the vectors are moved effectively 
				 * without having to reallocate them.
				 * 
				 * For more info see IIR.
				 * @param NumCoeffs Numerator coefficients of the discrete transfer function. Expects a vector containing the 
				 * @param DenCoeffs Denominator coefficients of the discrete transfer function. 
				 */
				IIR(const cpstd::vector<CoefficientType>&& NumCoeffs, const cpstd::vector<CoefficientType>&& DenCoeffs) : IIR() {
					SetCoefficients(NumCoeffs, DenCoeffs);
				}

				/**
				 * @brief Assignment operator overload.
				 * @param Rhs Reference object 
				 * @return Reference to the target object
				 */
				IIR& operator= (const IIR& Rhs){
					_NumCoefficients = Rhs.NumCoefficients();
					_DenCoefficients = Rhs.DenCoefficients();
					_InputBuffer = Rhs.InputBuffer();
					_OutputBuffer = Rhs.OutputBuffer();
					_InputIndex = Rhs.InputIndex();
					_OutputIndex = Rhs.OutputIndex();
					return (*this);
				}

				/**
				 * @brief Returns the numerator coefficients:
				 * NumCoeffs = \f$ \left[ b_0, b_1, \cdots , b_P \right] \f$ 
				 * @return A reference to the vector storing the numerator coefficients
				 */
				const cpstd::vector<CoefficientType>& NumCoefficients(){
					return _NumCoefficients;
				}

				/**
				 * @brief Returns the denominator coefficients: 
				 * DenCoeffs = \f$ \left[ a_0, a_1, \cdots , a_Q \right] \f$
				 * @return A reference to the vector storing the denominator coefficients
				 */
				const cpstd::vector<CoefficientType>& DenCoefficients(){
					return _DenCoefficients;
				}

				/**
				 * @brief Sets the coefficients of the transfer function.
				 * 
				 * Where: 
				 * NumCoeffs = \f$ \left[ b_0, b_1, \cdots , b_P \right] \f$, and: 
				 * DenCoeffs = \f$ \left[ a_0, a_1, \cdots , a_Q \right] \f$
				 */
				void SetCoefficients(const cpstd::vector<CoefficientType>& Num, const cpstd::vector<CoefficientType>& Den){
					_NumCoefficients = Num;
					_DenCoefficients = Den;

					_InputBuffer.resize(_NumCoefficients.size());
					_OutputBuffer.resize(_DenCoefficients.size());

					InitializeInputBuffer();
					InitializeOutputBuffer();
				}


				/**
				 * @brief Sets the coefficients of the transfer function.
				 * 
				 * Where: 
				 * NumCoeffs = \f$ \left[ b_0, b_1, \cdots , b_P \right] \f$, and: 
				 * DenCoeffs = \f$ \left[ a_0, a_1, \cdots , a_Q \right] \f$
				 * 
				 * This version of SetCoefficients recieve an rvalue reference to the input 
				 * vectors, this way move semantics are enabled and the vectors are moved effectively 
				 * without having to reallocate them.
				 */
				void SetCoefficients(const cpstd::vector<CoefficientType>&& Num, const cpstd::vector<CoefficientType>&& Den){
					_NumCoefficients = Num;
					_DenCoefficients = Den;

					_InputBuffer.resize(_NumCoefficients.size());
					_OutputBuffer.resize(_DenCoefficients.size());

					InitializeInputBuffer();
					InitializeOutputBuffer();
				}

				/**
				 * @brief Resets all of the data sored in the input buffer to the specified value,
				 * If there is no value is specified, the buffer will initialize to 0.
				 *
				 */
				void InitializeInputBuffer(const InputType& Value = InputType()){	
					_InputIndex = _InputBuffer.size()-1;

					for(uint8_t i = 0; i < _InputBuffer.size(); i++)
					{
						_InputBuffer[i] = Value;
					}
				}

				/**
				 * @brief Resets all of the data sored in the output buffer to the specified value,
				 * If there is no value is specified, the buffer will initialize to 0.
				 *
				 */
				void InitializeOutputBuffer(const OutputType& Value = OutputType()){	
					_OutputIndex = _OutputBuffer.size()-1;

					for(uint8_t i = 0; i < _OutputBuffer.size(); i++)
					{
						_OutputBuffer[i] = Value;
					}
				}

				void InitializeBuffers(const InputType& InValue = InputType(), const OutputType& OutValue = OutputType()){
					InitializeInputBuffer(InValue);
					InitializeOutputBuffer(OutValue);
				}

				/**
				 * @brief Recieves a new data point and calculates the transfer function output using the stored data.
				 * @param NewData Numerical value of the input signal.
				 * @return Numerical value of the output signal
				 */
				OutputType Update(const InputType& NewData){
					if(_DenCoefficients.size() == 0){return OutputType();}
					// Increase input and output indices

					if(_InputIndex < _InputBuffer.size()-1)
					{
						_InputIndex++;
					}
					else
					{
						_InputIndex = 0;
					}

					if(_OutputIndex < _OutputBuffer.size()-1)
					{
						_OutputIndex++;
					}
					else
					{
						_OutputIndex = 0;
					}

					// Store new data and init output

					_InputBuffer[_InputIndex] = NewData;
					_OutputBuffer[_OutputIndex] = 0;

					// y[n] =  (1/a0) ( b_i * in[n-i] + a_j * out[n-j] )

					uint8_t DataIndex;
					uint8_t Len = _NumCoefficients.size();

					for(uint8_t CoefficientIndex = 0; CoefficientIndex < Len; CoefficientIndex++)
					{
						// y[n] =  b_k * x[n-k]
						DataIndex = (_InputIndex + Len - CoefficientIndex ) % Len;
						_OutputBuffer[_OutputIndex] += _InputBuffer[DataIndex] * _NumCoefficients[CoefficientIndex];

					}

					Len = _DenCoefficients.size();

					for(uint8_t CoefficientIndex = 1; CoefficientIndex < Len; CoefficientIndex++)
					{
						// y[n] =  b_k * x[n-k]
						DataIndex = (_OutputIndex + Len - CoefficientIndex ) % Len;

						_OutputBuffer[_OutputIndex] -= _OutputBuffer[DataIndex] * _DenCoefficients[CoefficientIndex];
					}
					
					_OutputBuffer[_OutputIndex] /= _DenCoefficients[0];

					return _OutputBuffer[_OutputIndex];
				}
		};
	}

#endif//DISCRETE_CONTROLLERS_IIR_H