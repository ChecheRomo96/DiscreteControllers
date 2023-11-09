#ifndef DISCRETE_CONTROLLERS_FIR_H
#define DISCRETE_CONTROLLERS_FIR_H                                  

	#include <DiscreteControllers_BuildSettings.h>
	#include <CPvector.h>

	namespace DiscreteControllers
	{
		/**
		 * @brief FIR (Finite Impulse Response) Filter Class
		 *
		 * This class represents a Finite Impulse Response filter designed to process discrete data signals.
		 *
		 * @tparam DataType The type of data to be processed by the FIR filter.
		 */
		template<typename DataType, typename CoefficientsType>
		class FIR
		{
			private:
		
				cpstd::vector<CoefficientsType> _Coefficients;	///< Coefficients of the FIR filter.
				cpstd::vector<DataType> _DataBuffer;	///< Buffer to hold incoming data.
				size_t _DataIndex;						///< Index for tracking the incoming data.
				DataType _ProccesedData;				///< Processed data after applying the filter.

			public:
				/**
				 * @brief Default Constructor
				 *
				 * Initializes the FIR filter's internal variables.
				 */
				FIR(){
					static_assert(cpstd::is_arithmetic<DataType>::value, "DataType must be arithmetic");
					static_assert(cpstd::is_floating_point_v<CoefficientsType>, "DataType must be arithmetic");
					_DataIndex = 0;
					_ProccesedData = 0;
				}

				/**
				 * @brief Constructor with Coefficients
				 *
				 * Initializes the filter with provided coefficients.
				 *
				 * @param Coefficients Coefficients for the FIR filter.
				 */
				FIR(const cpstd::vector<CoefficientsType>& Coefficients) :FIR() {
					SetCoefficients(Coefficients);
				}

				/**
				 * @brief Constructor with Coefficients
				 *
				 * Initializes the filter with provided coefficients.
				 *
				 * @param Coefficients Coefficients for the FIR filter.
				 */
				FIR(const cpstd::vector<CoefficientsType>&& Coefficients) :FIR() {
					SetCoefficients(Coefficients);
				}

				/**
				 * @brief Assignment Operator Overload
				 *
				 * Copies the contents of the FIR filter object.
				 *
				 * @param Rhs The reference object to copy from.
				 * @return Reference to the target object.
				 */
				FIR& operator= (const FIR& Rhs)
				{
					_Coefficients = Rhs.Coefficients();
					_DataBuffer = Rhs.DataBuffer();
					_DataIndex = Rhs.DataIndex();
					return (*this);
				}
				
				/**
				 * @brief Get the FIR filter coefficients.
				 *
				 * @return Reference to the coefficients vector.
				 */
				const cpstd::vector<CoefficientsType>& Coefficients()
				{
					return _Coefficients;
				}

				/**
				 * @brief Set the Coefficients for the FIR filter.
				 *
				 * This method sets the coefficients for the FIR filter, which determines the filter's behavior.
				 *
				 * @param FirCoefficients The vector containing the FIR filter coefficients.
				 */
				void SetCoefficients(const cpstd::vector<CoefficientsType>& FirCoefficients)
				{
					_Coefficients = FirCoefficients;

					uint8_t LastSize = _DataBuffer.size();

					_DataBuffer.resize(_Coefficients.size());

					if (LastSize > 0)
					{
						for (uint8_t i = LastSize; i < _DataBuffer.size(); i++)
						{
							_DataBuffer[i] = 0.0000;
						}

						cpstd::vector<DataType> tmp;
						tmp.resize(_DataIndex);

						for (size_t i = 0; i < _DataIndex; i++)
						{
							tmp[i] = _DataBuffer[i];
						}

						for (size_t i = _DataIndex; i < LastSize; i++)
						{
							_DataBuffer[i - _DataIndex] = _DataBuffer[i];
						}

						for (size_t i = 0; i < _DataIndex; i++)
						{
							_DataBuffer[LastSize - _DataIndex - 1] = tmp[i];
						}
					}
					else
					{
						InitializeBuffer();
					}
				}

				/**
				 * @brief Set the Coefficients for the FIR filter.
				 *
				 * This method sets the coefficients for the FIR filter, which determines the filter's behavior.
				 *
				 * @param FirCoefficients The vector containing the FIR filter coefficients.
				 */
				void SetCoefficients(const cpstd::vector<CoefficientsType>&& FirCoefficients)
				{
					_Coefficients = cpstd::move(FirCoefficients);

					uint8_t LastSize = _DataBuffer.size();

					_DataBuffer.resize(_Coefficients.size());

					if (LastSize > 0)
					{
						for (uint8_t i = LastSize; i < _DataBuffer.size(); i++)
						{
							_DataBuffer[i] = 0.0000;
						}

						cpstd::vector<DataType> tmp;
						tmp.resize(_DataIndex);

						for (size_t i = 0; i < _DataIndex; i++)
						{
							tmp[i] = _DataBuffer[i];
						}

						for (size_t i = _DataIndex; i < LastSize; i++)
						{
							_DataBuffer[i - _DataIndex] = _DataBuffer[i];
						}

						for (size_t i = 0; i < _DataIndex; i++)
						{
							_DataBuffer[LastSize - _DataIndex - 1] = tmp[i];
						}
					}
					else
					{
						InitializeBuffer();
					}
				}
				
				/**
				 * @brief Initialize the Buffer for Data Storage.
				 *
				 * Initializes the buffer used for storing incoming data, setting all values to zero.
				 */
				void InitializeBuffer()
				{	
					//_DataIndex  = 0;

					for(uint8_t i = 0; i < _DataBuffer.size(); i++)
					{
						_DataBuffer[i] = 0.0000;
					}
				}

				/**
				 * @brief Get the Data Buffer.
				 *
				 * Accesses the internal data buffer used for the filter.
				 *
				 * @return Reference to the data buffer.
				 */
				const cpstd::vector<DataType>& DataBuffer()
				{
					return _DataBuffer;
				}

				/**
				 * @brief Get the Data Index.
				 *
				 * Returns the index pointing to the current data within the buffer.
				 *
				 * @return Index of the current data.
				 */
				const uint8_t DataIndex()
				{
					return _DataIndex;
				}

				/**
				 * @brief Update the Filter with New Data.
				 *
				 * Updates the filter with new data, applies the FIR filter operation, and returns the processed data.
				 *
				 * @param NewData The new data point to be processed.
				 * @return Processed data after applying the FIR filter.
				 */
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
				
				/**
				 * @brief Process the Buffer of Data.
				 *
				 * Processes a buffer of data, applying the FIR filter and producing an output buffer.
				 *
				 * @param DataIn The input data buffer to be processed.
				 * @param DataOut The output buffer to store the processed data.
				 * @return Processed output data stored in the DataOut buffer.
				 */
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