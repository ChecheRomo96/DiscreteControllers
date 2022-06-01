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
					_DataBuffer = Rhs.DataBuffer();
					_DataIndex = Rhs.DataIndex();
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

				const uint8_t DataIndex()
				{
					return _DataIndex;
				}

				void SetCoefficients(const CPVector::vector<double>& Num, const CPVector::vector<double>& Den)
				{
					_NumCoefficients = Num;
					_DenCoefficients = Den;

					uint8_t LastNumSize = _InputBuffer.size();
					uint8_t LastDenSize = _OutputBuffer.size();

					_InputBuffer.resize(_NumCoefficients.size());
					_OutputBuffer.resize(_DenCoefficients.size());

					if(LastSize>0)
					{
						for(uint8_t i = LastSize; i < _DataBuffer.size(); i++)
						{
							_DataBuffer[i] = 0.0000;
						}

						CPVector::vector<DataType> tmp;
						tmp.resize(_DataIndex);

						for(uint8_t i = 0; i < _DataIndex; i++)
						{
							tmp[i] = _DataBuffer[i];
						}

						for(uint8_t i = _DataIndex; i < LastSize; i++)
						{
							_DataBuffer[i - _DataIndex] = _DataBuffer[i];
						}

						for(uint8_t i = 0; i < _DataIndex; i++)
						{
							_DataBuffer[LastSize - _DataIndex - 1] = tmp[i];
						}



					}
					else
					{
						InitializeBuffer();
					}
				}

				void InitializeBuffer()
				{	
					//_DataIndex  = 0;

					for(uint8_t i = 0; i < _InputBuffer.size(); i++)
					{
						_InputBuffer[i] = 0.0000;
					}

					for(uint8_t i = 0; i < _OutputBuffer.size(); i++)
					{
						_OutputBuffer[i] = 0.0000;
					}
				}



				DataType Update(const DataType NewData)
				{
					if(_InputIndex < _InputBuffer.size()-1)
					{
						_InputIndex++;
					}
					else
					{
						_InputIndex = 0;
					}

					_InputBuffer[_InputIndex] = NewData;

					_OutputBuffer[_OutputIndex] = 0;

					uint8_t DataIndex, IIR_Index;
					uint8_t Len = _NumCoefficients.size();

					for(uint8_t i = 0; i < Len; i++)
					{
						// y[n] =  b_k * x[n-k]

						if(i <= _InputIndex){DataIndex = _InputIndex - i;}
						else{DataIndex = Len - i + _InputIndex;}
						
						IIR_Index = i;

						_OutputBuffer[_OutputIndex] += _InputIndex[DataIndex] * _NumCoefficients[IIR_Index];
					}

					Len = _DenCoefficients.size();

					for(uint8_t i = 0; i < Len-1; i++)
					{
						// y[n] =  b_k * x[n-k]

						if(i + 1  <= _OutputIndex){DataIndex = _OutputIndex - i - 1;}
						else{DataIndex = Len - i + _OutputIndex - 1;}
						
						IIR_Index = i;

						_OutputBuffer[_OutputIndex] -= _OutputBuffer[DataIndex] * _DenCoefficients[IIR_Index];
					}

					return _OutputBuffer[_OutputIndex];
				}
		};
	}

#endif//DISCRETE_CONTROLLERS_IIR_H