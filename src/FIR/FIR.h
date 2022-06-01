#ifndef DISCRETE_CONTROLLERS_FIR_H
#define DISCRETE_CONTROLLERS_FIR_H                                  

	#include <DiscreteControllers_BuildSettings.h>
	#include <CPVector.h>

	namespace DiscreteControllers
	{
		template<class DataType>
		class FIR
		{
			private:
		
				CPVector::vector<double> _FirCoefficients;
				CPVector::vector<DataType> _DataBuffer;
				uint8_t _DataIndex;
				DataType _ProccesedData;

			public:
				FIR()
				{
					_DataIndex = 0;
					_ProccesedData = 0;
				}

				FIR(const CPVector::vector<double>& Coefficients)
				{
					uint8_t _DataIndex;
					DataType _ProccesedData;

					SetCoefficients(Coefficients);
				}

				FIR& operator= (const FIR& Rhs)
				{
					_FirCoefficients = Rhs.FirCoefficients();
					_DataBuffer = Rhs.DataBuffer();
					_DataIndex = Rhs.DataIndex();
					return (*this);
				}

				const CPVector::vector<double>& FirCoefficients()
				{
					return _FirCoefficients;
				}

				void SetCoefficients(const CPVector::vector<double>& FirCoefficients)
				{
					_FirCoefficients = FirCoefficients;

					uint8_t LastSize = 0;
					if(_DataBuffer.size() > 1)
					{
						LastSize = _FirCoefficients.size();
						for(uint8_t i = 0; i < LastSize; i++)
						{
							_DataBuffer.swap(0+i,_DataIndex+1+i);
						}
					}

					_DataBuffer.resize(_FirCoefficients.size());

					if(LastSize>0)
					{
						for(uint8_t i = LastSize; i < _DataBuffer.size(); i++)
						{
							_DataBuffer[i] = 0.0000;
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

					for(uint8_t i = 0; i < _DataBuffer.size(); i++)
					{
						_DataBuffer[i] = 0.0000;
					}
				}

				const CPVector::vector<DataType>& DataBuffer()
				{
					return _DataBuffer;
				}

				const uint8_t DataIndex()
				{
					return _DataIndex;
				}


				DataType Update(const DataType NewData)
				{
					if(_DataIndex > 0)
					{
						_DataIndex = (_DataIndex--);
					}
					else
					{
						_DataIndex = _DataBuffer.size()-1;
					}

					_DataBuffer[_DataIndex] = NewData;
					_ProccesedData = 0;

					uint8_t DataIndex, FirIndex;

					for(uint8_t i = 0; i < _DataBuffer.size(); i++)
					{
						// y[n] =  b_k * x[n-k]

						DataIndex = _DataIndex + i;
						DataIndex %= _DataBuffer.size();

						FirIndex = i;

						_ProccesedData += _DataBuffer[DataIndex] * _FirCoefficients[FirIndex];
					}

					return _ProccesedData;
				}

				CPVector::std::vector<DataType>& ProcessBuffer(const CPVector& DataIn, CPVector& DataOut)
				{
					DataOut.resize(DataIn.size());
					InitializeBuffer();
					uint8_t FilterOrder = _FirCoefficients.size();

					for (int i = 0; i < DataIn.size(); ++i)
					{
						

						if(i >= FilterOrder)
						{
							DataOut[i - FilterOrder] = Update(DataIn[i]);
						}
						else
						{
							Update(DataIn[i]);
						}
					}

					for(int i = 0; i < FilterOrder; i++)
					{
						DataOut[DataOut.size() - FilterOrder + i] = Update(0);
					}

					InitializeBuffer();
				}
		};
	}
#endif//FIR_FILTER_H