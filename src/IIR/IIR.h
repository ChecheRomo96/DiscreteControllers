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
		 * @brief Infinite Impulse Response Class. \n\n
		 * In engineering, a transfer function of a system, 
		 * sub-system, or component is a mathematical function that 
		 * theoretically models the system's output 
		 * for each possible input. A discrete system transfer function is defined as: \n\n
		 * \f$ \displaystyle  
		 *    H(z) = 
		 *    \frac{Y\left[z\right]}{X\left[z\right]} = 
		 *    \frac{\sum_{i=0}^{P} b_{i}z^{-i}}{\sum_{j=0}^{Q} a_{j}z^{-j}}
		 * \f$ \n\n
		 * Infinite Impulse Response systems are described and implemented in terms of the difference equation 
		 * , which defines how the output signal is related to the input signal. The equation below is 
		 * the IIR system transfer function. \n\n
		 * \f$ \displaystyle y[n] = \frac{1}{a_0} \left( \sum_{i=0}^{P} b_ix[n-i] - \sum_{j=1}^{Q} a_jx[n-j] \right) \f$ \n\n
		 * where:\n
		 * -\f$ P \f$ is the feedforward filter order 
		 * -\f$ b_i \f$ are the feedforward filter coefficients
		 * -\f$ Q \f$ is the feedback filter order
		 * -\f$ a_i \f$ are the feedback filter coefficients
		 * -\f$ x \left[ n \right] \f$ is the input signal
		 * -\f$ y \left[ n \right] \f$ is the output signal.
		 *
		 * Recursive filters are an efficient way of achieving a long impulse response, 
		 * without having to perform a long convolution. They execute very rapidly, 
		 * but have less performance and flexibility than other digital filters. 
		 * Recursive filters are also called Infinite Impulse Response (IIR) filters, 
		 * since their impulse responses are composed of decaying exponentials. 
		 * This distinguishes them from digital filters carried out by convolution, 
		 * called Finite Impulse Response (FIR) filters.
		 *
		 * Below is a simple example that demonstrates the usage of this class, this example 
		 * implements the discrete system with transfer function: \n\n
		 * \f$ \displaystyle 
		 *   G(z) = \left( \frac{0.0198}{z-0.9802} \right), 
		 *   \{ f_s = 100Hz \}
		 * \f$ \n\n
		 * which is the continuous time system with the transfer function that is shown below with a sample rate of 100Hz: \n\n
		 * \f$ \displaystyle 
		 *   G(s) = \left( \frac{1}{0.5 s + 1} \right)
		 * \f$
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
		 * @tparam DataType Data type to be used as input and output signals
		 */
		template<class DataType>
		class IIR
		{
			private:
		
				cpstd::vector<double> _NumCoefficients;
				cpstd::vector<double> _DenCoefficients;
				cpstd::vector<DataType> _InputBuffer;
				cpstd::vector<DataType> _OutputBuffer;
				uint8_t _InputIndex;
				uint8_t _OutputIndex;

			public:
				/**
				 * @brief Default Class Constructor. 
				 */
				IIR()
				{
					_InputIndex = 0;
					_OutputIndex = 0;
				}

				/**
				 * @brief Class Constructor with initializer, where: 
				 * NumCoeffs = \f$ \left[ b_0, b_1, \cdots , b_P \right] \f$, and: 
				 * DenCoeffs = \f$ \left[ a_0, a_1, \cdots , a_Q \right] \f$
				 *
				 * Coefficients are from the following representation of a discrete system:\n
				 * \f$ \displaystyle  
				 *    H(z) = 
				 *    \frac{\sum_{i=0}^{P} b_{i}z^{-i}}{\sum_{j=0}^{Q} a_{j}z^{-j}}
				 * \f$ \n
				 * For more info see IIR.
				 * @param NumCoeffs Numerator coefficients of the discrete transfer function. Expects a vector containing the 
				 * @param DenCoeffs Denominator coefficients of the discrete transfer function. 
				 */
				IIR(const cpstd::vector<double>& NumCoeffs, const cpstd::vector<double>& DenCoeffs)
				{
					SetCoefficients(NumCoeffs, DenCoeffs);
				}

				/**
				 * @brief Assignment operator overload.
				 * @param Rhs Reference object 
				 * @return Reference to the target object
				 */
				IIR& operator= (const IIR& Rhs)
				{
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
				const cpstd::vector<double>& NumCoefficients()
				{
					return _NumCoefficients;
				}

				/**
				 * @brief Returns the denominator coefficients: 
				 * DenCoeffs = \f$ \left[ a_0, a_1, \cdots , a_Q \right] \f$
				 * @return A reference to the vector storing the denominator coefficients
				 */
				const cpstd::vector<double>& DenCoefficients()
				{
					return _DenCoefficients;
				}

				/**
				 * @brief Sets the coefficients of the transfer function.
				 * 
				 * Where: 
				 * NumCoeffs = \f$ \left[ b_0, b_1, \cdots , b_P \right] \f$, and: 
				 * DenCoeffs = \f$ \left[ a_0, a_1, \cdots , a_Q \right] \f$
				 */
				void SetCoefficients(const cpstd::vector<double>& Num, const cpstd::vector<double>& Den)
				{
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
				void InitializeInputBuffer(DataType Value = 0)
				{	
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
				void InitializeOutputBuffer(DataType Value = 0)
				{	
					_OutputIndex = _OutputBuffer.size()-1;

					for(uint8_t i = 0; i < _OutputBuffer.size(); i++)
					{
						_OutputBuffer[i] = Value;
					}
				}

				/**
				 * @brief Recieves a new data point and calculates the transfer function output using the stored data.
				 * @param NewData Numerical value of the input signal.
				 * @return Numerical value of the output signal
				 */
				DataType Update(const DataType NewData)
				{

					if((_DenCoefficients.size() == 0)||(_DenCoefficients.size() == 0)){return 0;}
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