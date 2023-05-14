#ifndef DISCRETE_CONTROLLERS_IIR_H
#define DISCRETE_CONTROLLERS_IIR_H                                  

	#include <DiscreteControllers_BuildSettings.h>
	#include <CPVector.h>

	namespace DiscreteControllers
	{
		template<class DataType>
		class IIR
		{
			private:
		
				CPVector::vector<double> _NumCoefficients;
				CPVector::vector<double> _DenCoefficients;
				CPVector::vector<DataType> _InputBuffer;
				CPVector::vector<DataType> _OutputBuffer;
				uint8_t _InputIndex;
				uint8_t _OutputIndex;

			public:
				IIR()
				{
					_InputIndex = 0;
					_OutputIndex = 0;
				}

				IIR(const CPVector::vector<double>& Coefficients)
				{
					uint8_t _DataIndex;
					DataType _ProccesedData;

					SetCoefficients(Coefficients);
				}

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

				const CPVector::vector<double>& NumCoefficients()
				{
					return _NumCoefficients;
				}

				const CPVector::vector<double>& DenCoefficients()
				{
					return _DenCoefficients;
				}

				const CPVector::vector<DataType>& InputBuffer()
				{
					return _InputBuffer;
				}

				const CPVector::vector<DataType>& OutputBuffer()
				{
					return _OutputBuffer;
				}

				const uint8_t InputIndex()
				{
					return _InputIndex;
				}

				const uint8_t OutputIndex()
				{
					return _OutputIndex;
				}

				void SetCoefficients(const CPVector::vector<double>& Num, const CPVector::vector<double>& Den)
				{
					_NumCoefficients = Num;
					_DenCoefficients = Den;

					_InputBuffer.resize(_NumCoefficients.size());
					_OutputBuffer.resize(_DenCoefficients.size());

					InitializeInputBuffer();
					InitializeOutputBuffer();
				}

				void InitializeInputBuffer()
				{	
					_InputIndex = _InputBuffer.size()-1;

					for(uint8_t i = 0; i < _InputBuffer.size(); i++)
					{
						_InputBuffer[i] = 0.0000;
					}
				}

				void InitializeOutputBuffer()
				{	
					_OutputIndex = _OutputBuffer.size()-1;

					for(uint8_t i = 0; i < _OutputBuffer.size(); i++)
					{
						_OutputBuffer[i] = 0.0000;
					}
				}



				DataType Update(const DataType NewData)
				{
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

					// y[n] =  a_k * in[n-k] + b_m * out[n-m]

					uint8_t DataIndex, CoefficientIndex;
					uint8_t Len = _NumCoefficients.size();

					for(uint8_t CoefficientIndex = 0; CoefficientIndex < Len; CoefficientIndex++)
					{
						// y[n] =  b_k * x[n-k]
						DataIndex = (CoefficientIndex + _InputIndex + 1 ) % Len;

						_OutputBuffer[_OutputIndex] += _InputBuffer[DataIndex] * _NumCoefficients[CoefficientIndex];
						Serial.print(_InputBuffer[DataIndex] * _NumCoefficients[CoefficientIndex]);
						Serial.print(", ");

					}

					Len = _DenCoefficients.size();

					for(uint8_t CoefficientIndex = 0; CoefficientIndex < Len-1; CoefficientIndex++)
					{
						// y[n] =  b_k * x[n-k]
						DataIndex = (CoefficientIndex + _OutputIndex + 1 ) % Len;

						_OutputBuffer[_OutputIndex] -= _OutputBuffer[DataIndex] * _DenCoefficients[CoefficientIndex];
					}

					_OutputBuffer[_OutputIndex] *= (1.0/_DenCoefficients[_DenCoefficients.size()-1]);

					return _OutputBuffer[_OutputIndex];
				}
		};
	}

#endif//DISCRETE_CONTROLLERS_IIR_H