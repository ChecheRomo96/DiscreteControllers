#ifndef DISCRETE_CONTROLLERS_FIR_H
#define DISCRETE_CONTROLLERS_FIR_H                                  

	#include <DiscreteControllers_BuildSettings.h>
	#include <CPvector.h>

	namespace DiscreteControllers
	{
		template<class DataType>
		class FIR
		{
			private:
		
				cpstd::vector<double> _Coefficients;
				cpstd::vector<DataType> _DataBuffer;
				uint8_t _DataIndex;
				DataType _ProccesedData;

			public:
				FIR()
				{
					_DataIndex = 0;
					_ProccesedData = 0;
				}

				FIR(const cpstd::vector<double>& Coefficients)
				{
					uint8_t _DataIndex;
					DataType _ProccesedData;

					SetCoefficients(Coefficients);
				}

				FIR& operator= (const FIR& Rhs)
				{
					_Coefficients = Rhs.Coefficients();
					_DataBuffer = Rhs.DataBuffer();
					_DataIndex = Rhs.DataIndex();
					return (*this);
				}

				const cpstd::vector<double>& Coefficients()
				{
					return _Coefficients;
				}

				void SetCoefficients(const cpstd::vector<double>& FirCoefficients)
				{
					_Coefficients = FirCoefficients;

					uint8_t LastSize = _DataBuffer.size();

					_DataBuffer.resize(_Coefficients.size());

					if(LastSize>0)
					{
						for(uint8_t i = LastSize; i < _DataBuffer.size(); i++)
						{
							_DataBuffer[i] = 0.0000;
						}

						cpstd::vector<DataType> tmp;
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

					for(uint8_t i = 0; i < _DataBuffer.size(); i++)
					{
						_DataBuffer[i] = 0.0000;
					}
				}

				const cpstd::vector<DataType>& DataBuffer()
				{
					return _DataBuffer;
				}

				const uint8_t DataIndex()
				{
					return _DataIndex;
				}


				DataType Update(const DataType NewData)
				{
					if(_DataIndex < _DataBuffer.size()-1)
					{
						_DataIndex++;
					}
					else
					{
						_DataIndex = 0;
					}

					_DataBuffer[_DataIndex] = NewData;
					_ProccesedData = 0;

					uint8_t DataIndex, FirIndex;
					uint8_t FilterOrder = _Coefficients.size();

					for(uint8_t i = 0; i < FilterOrder; i++)
					{
						// y[n] =  b_k * x[n-k]

						if(i <= _DataIndex){DataIndex = _DataIndex - i;}
						else{DataIndex = FilterOrder - i + _DataIndex;}
						
						FirIndex = i;

						_ProccesedData += _DataBuffer[DataIndex] * _Coefficients[FirIndex];
					}

					return _ProccesedData;
				}

				cpstd::vector<DataType>& ProcessBuffer(const cpstd::vector<DataType>& DataIn, cpstd::vector<DataType>& DataOut)
				{
					DataOut.resize(DataIn.size());
					InitializeBuffer();
					uint8_t Delay = _Coefficients.size() / 2;

					for (int i = 0; i < DataIn.size(); ++i)
					{
						if(i >= Delay)
						{
							DataOut[i - Delay] = Update(DataIn[i]);
						}
						else
						{
							Update(DataIn[i]);
						}
					}

					for(int i = 0; i < Delay; i++)
					{
						DataOut[DataOut.size() - Delay + i] = Update(0);
					}

					InitializeBuffer();

					return DataOut; 
				}
		};
	}
#endif//FIR_FILTER_H