class HeartRateFilter{
public:
	struct IrPeak {
		float HeartBeatRate;
		float pTime;
		float minValue;
		float maxValue;
		int flag;
	};

	struct FilterOutput
	{
		float Time;
		float Output;
	};

	private:
		int filter_size;

		double *FIFO;
		double sum;
		float f_value;
		float output;

		const int numFIFO = 28; //�y��`�g���փt�B���^�̒i���z
		float *TimeFIFO = new float[numFIFO];
		float *RectFIFO = new float[numFIFO];
		int ptr0, ptr1, ptr2, ptr3, ptr4, ptr5;   //  0, 26, 24, 21, 20, 19
		int ptr6, ptr7, ptr8, ptr9, ptr10, ptr11; // 14,  9,  8,  7,  4,  2
		float K = 1.0f / numFIFO;

		int PeakToPeak = 1000;

		IrPeak Peak;
		


	public:
		
		HeartRateFilter();



		float GetFilter(float DataInput, float Time);

		float GetHeartBeatRate(float Data);
		
		FilterOutput rectOut;
};


class BreathRateFilter : public HeartRateFilter {
private:
	int nFIFO;

	double *FIFO;
	double sum;
	float f_value;
	float output;

	const int numFIFO = 60; //�y��`�g���փt�B���^�̒i���z
	float *TimeFIFO = new float[numFIFO];
	float *RectFIFO = new float[numFIFO];
	int ptr0, ptr1, ptr2, ptr3, ptr4, ptr5;   //  0, 54, 52, 50, 48, 45
	int ptr6, ptr7, ptr8, ptr9, ptr10, ptr11; // 42,  39,  36,  33,  31,  29
	int ptr12, ptr13, ptr14, ptr15, ptr16, ptr17;   //  27, 25, 23, 21, 18, 16
	int ptr18, ptr19, ptr20, ptr21, ptr22, ptr23; // 14,  11,  8,  6,  4,  2
	int ptr24, ptr25;
	/*
	int ptr26, ptr27, ptr28, ptr29, ptr30, ptr31;   //  0, 54, 52, 50, 48, 45
	int ptr32, ptr33, ptr34, ptr35;	
	int ptr36, ptr37, ptr38, ptr39, ptr40, ptr41; // 42,  39,  36,  33,  31,  29
	int ptr42, ptr43, ptr44, ptr45, ptr46, ptr47;   //  27, 25, 23, 21, 18, 16
	int ptr48, ptr49, ptr50;
	*/
	float K = 1.0f / numFIFO;

	


public:
	

	BreathRateFilter();

	float PeakToPeak = 1000;
	

	IrPeak Peak;
	FilterOutput rectOut2;

	float W = 17.0f / 144;
	int Bcount = 0;

	float BreathFilter(float DataInput, float Time);
	float getBreathRate(float Data);
};
