#include "waveFilter.h"
#define INT_MAX 0x7fffffff


HeartRateFilter::HeartRateFilter() {
	rectOut.Output = 0.0f; rectOut.Time = 0;
	for (int i = 0; i < numFIFO; i++) { RectFIFO[i] = 0.0f; TimeFIFO[i] = 0; }
	ptr0 = 0; ptr1 = 26; ptr2 = 24; ptr3 = 21; ptr4 = 20; ptr5 = 19;
	ptr6 = 14; ptr7 = 9; ptr8 = 8; ptr9 = 7; ptr10 = 4; ptr11 = 2;

	Peak.HeartBeatRate = 0.0f;
	Peak.pTime = 0;
	Peak.minValue = 0;
	Peak.maxValue =(float)INT_MAX;
	Peak.flag = 0;
}

float HeartRateFilter::GetFilter(float DataInput, float Time) {
	rectOut.Time = TimeFIFO[ptr6];
	rectOut.Output += (-1) * (DataInput + RectFIFO[ptr1] + RectFIFO[ptr2])
		+ 2 * (RectFIFO[ptr3] + RectFIFO[ptr4] + RectFIFO[ptr5])
		+ (-2) * (RectFIFO[ptr7] + RectFIFO[ptr8] + RectFIFO[ptr9])
		+ 1 * (RectFIFO[ptr10] + RectFIFO[ptr11] + RectFIFO[ptr0]);
	RectFIFO[ptr0] = DataInput;
	TimeFIFO[ptr0] = Time;
	ptr0++; if (ptr0 == numFIFO) { ptr0 = 0; }
	ptr1++; if (ptr1 == numFIFO) { ptr1 = 0; }
	ptr2++; if (ptr2 == numFIFO) { ptr2 = 0; }
	ptr3++; if (ptr3 == numFIFO) { ptr3 = 0; }
	ptr4++; if (ptr4 == numFIFO) { ptr4 = 0; }
	ptr5++; if (ptr5 == numFIFO) { ptr5 = 0; }
	ptr6++; if (ptr6 == numFIFO) { ptr6 = 0; }
	ptr7++; if (ptr7 == numFIFO) { ptr7 = 0; }
	ptr8++; if (ptr8 == numFIFO) { ptr8 = 0; }
	ptr9++; if (ptr9 == numFIFO) { ptr9 = 0; }
	ptr10++; if (ptr10 == numFIFO) { ptr10 = 0; }
	ptr11++; if (ptr11 == numFIFO) { ptr11 = 0; }
	return rectOut.Output * K;
}

float HeartRateFilter::GetHeartBeatRate(float Data) {
	// rectOut.Time  �y���ݎ����z[�~���b]
	// rectOut.Output�y���ݒl�z
	float curTime = rectOut.Time;
	float value = Data; //  rectOut.Output;
	if (value > 0)
	{
		Peak.flag = 0;     //�y�s�[�N���o�t���O�z0:�ҋ@��
		Peak.minValue = 0;

	}
	else //�yvalue��0�z
	{
		if (value > Peak.minValue)
		{
			curTime = rectOut.Time;                     //�y���ݎ�����擾����z
			if (Peak.pTime != 0)
			{
				Peak.HeartBeatRate = 60.0f / (curTime - Peak.pTime);    //�y��������Z����z�P��[��/��]
			}
			Peak.pTime = curTime;
			Peak.minValue = 0;
			Peak.flag = 1;     //�y�s�[�N���o�t���O�z1:���o
		}
		else
		{
			if (Peak.flag == 0) { Peak.minValue = value; }
		}
	}
	return Peak.HeartBeatRate;

}


BreathRateFilter::BreathRateFilter() {
	rectOut2.Output = 0.0f; rectOut2.Time = 0;
	for (int i = 0; i < numFIFO; i++) { RectFIFO[i] = 0.0f; TimeFIFO[i] = 0; }

	ptr0 = 56; ptr1 = 54; ptr2 = 52; ptr3 = 50; ptr4 = 48; ptr5 = 45;
	ptr6 = 42; ptr7 = 39; ptr8 = 36; ptr9 = 33; ptr10 = 31; ptr11 = 29;
	ptr12 = 27; ptr13 = 25; ptr14 = 23; ptr15 = 21; ptr16 = 18; ptr17 = 16;
	ptr18 = 14; ptr19 = 11; ptr20 = 8; ptr21 = 6; ptr22 = 4; ptr23 = 2; ptr24 = 0; ptr25 = 58;
/*
	ptr0 = 190; ptr1 = 182; ptr2 = 174; ptr3 = 165; ptr4 = 160; ptr5 = 152;
	ptr6 = 145; ptr7 = 136; ptr8 = 127; ptr9 = 118; ptr10 = 110; ptr11 = 104;
	ptr12 = 96; ptr13 = 85; ptr14 = 78; ptr15 = 70; ptr16 = 64; ptr17 = 55;
	ptr18 = 48; ptr19 = 39; ptr20 = 32; ptr21 = 25; ptr22 = 15; ptr23 = 8; ptr24 = 0; ptr25 = 196;

	ptr0 = 112; ptr1 = 110; ptr2 = 107; ptr3 = 105; ptr4 = 103; ptr5 = 101;
	ptr6 = 99; ptr7 = 97; ptr8 = 95; ptr9 = 93; ptr10 = 91; ptr11 = 89;
	ptr12 = 86; ptr13 = 84; ptr14 = 82; ptr15 = 79; ptr16 = 76; ptr17 = 74;
	ptr18 = 72; ptr19 = 70; ptr20 = 68; ptr21 = 66; ptr22 = 64; ptr23 = 62; ptr24 = 60; ptr25 = 58;

	ptr26 = 56; ptr27 = 53; ptr28 = 51; ptr29 = 49; ptr30 = 47; ptr31 = 45;
	ptr32 = 43; ptr33 = 41; ptr34 = 39; ptr35 = 37; ptr36 = 35; ptr37 = 33;
	ptr38 = 30; ptr39 = 27; ptr40 = 25; ptr41 = 22; ptr42 = 19; ptr43 = 17;
	ptr44 = 15; ptr45 = 12; ptr46 = 9; ptr47 = 6; ptr48 = 3; ptr49 = 119; ptr50 = 115;
*/
	Peak.HeartBeatRate = 0.0f;
	Peak.pTime = 0;
	Peak.minValue = 0;
	Peak.maxValue = (float)INT_MAX;
	Peak.flag = 0;
	Bcount = 0;
}

float BreathRateFilter::BreathFilter(float DataInput, float Time) {

/*	
	rectOut2.Time = TimeFIFO[ptr25];
	rectOut2.Output += (DataInput + RectFIFO[ptr1] + RectFIFO[ptr2]
		// + RectFIFO[ptr3]
		+ RectFIFO[ptr4] + RectFIFO[ptr5]// + RectFIFO[ptr6]
		+ RectFIFO[ptr7] + RectFIFO[ptr8] + RectFIFO[ptr9]
		+ RectFIFO[ptr0] + RectFIFO[ptr48] + RectFIFO[ptr49]

		+ RectFIFO[ptr36] + RectFIFO[ptr37]// + RectFIFO[ptr38]
		+ RectFIFO[ptr39] + RectFIFO[ptr40] + RectFIFO[ptr41]
		//+ RectFIFO[ptr42] 
		+ RectFIFO[ptr43] + RectFIFO[ptr44] + RectFIFO[ptr45]
		+ RectFIFO[ptr46] + RectFIFO[ptr47])

		- (RectFIFO[ptr10] + RectFIFO[ptr11] + RectFIFO[ptr12]// + RectFIFO[ptr13]
			+ RectFIFO[ptr13] + RectFIFO[ptr14] + RectFIFO[ptr15] //+ RectFIFO[ptr16]
			+ RectFIFO[ptr17] + RectFIFO[ptr18] + RectFIFO[ptr19]
			+ RectFIFO[ptr20] + RectFIFO[ptr21] + RectFIFO[ptr22]// + RectFIFO[ptr23]
			+ RectFIFO[ptr24] + RectFIFO[ptr25]	+ RectFIFO[ptr27]//+ RectFIFO[ptr28]
			+ RectFIFO[ptr29] + RectFIFO[ptr30] + RectFIFO[ptr31]
			+ RectFIFO[ptr32] + RectFIFO[ptr33] + RectFIFO[ptr34]);//+ RectFIFO[ptr35]);


	rectOut2.Time = TimeFIFO[ptr13];
	rectOut2.Output += (DataInput + RectFIFO[ptr1] + RectFIFO[ptr2]
		// + RectFIFO[ptr3]
		+ RectFIFO[ptr4] + RectFIFO[ptr5]// + RectFIFO[ptr6]
		+ RectFIFO[ptr20] + RectFIFO[ptr21] + RectFIFO[ptr22]
		//+ RectFIFO[
		- (RectFIFO[ptr10] + RectFIFO[ptr11] + RectFIFO[ptr12]// + RectFIFO[ptr13]
			+ RectFIFO[ptr13] + RectFIFO[ptr14] + RectFIFO[ptr15] //+ RectFIFO[ptr16]
			+ RectFIFO[ptr17] + RectFIFO[ptr18] + RectFIFO[ptr19]
			+ RectFIFO[ptr20] + RectFIFO[ptr21] + RectFIFO[ptr22]// + RectFIFO[ptr23]
			+ RectFIFO[ptr24] + RectFIFO[ptr25]	+ RectFIFO[ptr27]//+ RectFIFO[ptr28]
			+ RectFIFO[ptr29] + RectFIFO[ptr30] //+ RectFIFO[ptr31]
			+ RectFIFO[ptr32] + RectFIFO[ptr33] + RectFIFO[ptr34]);//+ RectFIFO[ptr35]);
*/
	rectOut2.Time = TimeFIFO[ptr13];
	rectOut2.Output += (DataInput + RectFIFO[ptr1] + RectFIFO[ptr2]
		// + RectFIFO[ptr3]
		+ RectFIFO[ptr4] + RectFIFO[ptr5]// + RectFIFO[ptr6]
		+ RectFIFO[ptr20] + RectFIFO[ptr21] + RectFIFO[ptr22]
		//+ RectFIFO[ptr23] 
		+ RectFIFO[ptr0] + RectFIFO[ptr24] + RectFIFO[ptr25])
		- (RectFIFO[ptr7] + RectFIFO[ptr8] + RectFIFO[ptr9]// + RectFIFO[ptr10]
			+ RectFIFO[ptr11] + RectFIFO[ptr12]

			+ RectFIFO[ptr13]
			+ RectFIFO[ptr14] + RectFIFO[ptr15] //+ RectFIFO[ptr16]
			+ RectFIFO[ptr17] + RectFIFO[ptr18] + RectFIFO[ptr19]);

	RectFIFO[ptr0] = DataInput;
	TimeFIFO[ptr0] = Time;


	ptr0++; if (ptr0 == numFIFO) { ptr0 = 0; }
	ptr1++; if (ptr1 == numFIFO) { ptr1 = 0; }
	ptr2++; if (ptr2 == numFIFO) { ptr2 = 0; }
	ptr3++; if (ptr3 == numFIFO) { ptr3 = 0; }
	ptr4++; if (ptr4 == numFIFO) { ptr4 = 0; }
	ptr5++; if (ptr5 == numFIFO) { ptr5 = 0; }
	ptr6++; if (ptr6 == numFIFO) { ptr6 = 0; }
	ptr7++; if (ptr7 == numFIFO) { ptr7 = 0; }
	ptr8++; if (ptr8 == numFIFO) { ptr8 = 0; }
	ptr9++; if (ptr9 == numFIFO) { ptr9 = 0; }
	ptr10++; if (ptr10 == numFIFO) { ptr10 = 0; }
	ptr11++; if (ptr11 == numFIFO) { ptr11 = 0; }
	ptr12++; if (ptr12 == numFIFO) { ptr12 = 0; }
	ptr13++; if (ptr13 == numFIFO) { ptr13 = 0; }
	ptr14++; if (ptr14 == numFIFO) { ptr14 = 0; }
	ptr15++; if (ptr15 == numFIFO) { ptr15 = 0; }
	ptr16++; if (ptr16 == numFIFO) { ptr16 = 0; }
	ptr17++; if (ptr17 == numFIFO) { ptr17 = 0; }
	ptr18++; if (ptr18 == numFIFO) { ptr18 = 0; }
	ptr19++; if (ptr19 == numFIFO) { ptr19 = 0; }
	ptr20++; if (ptr20 == numFIFO) { ptr20 = 0; }
	ptr21++; if (ptr21 == numFIFO) { ptr21 = 0; }
	ptr22++; if (ptr22 == numFIFO) { ptr22 = 0; }
	ptr23++; if (ptr23 == numFIFO) { ptr23 = 0; }
	ptr24++; if (ptr24 == numFIFO) { ptr24 = 0; }
	ptr25++; if (ptr25 == numFIFO) { ptr25 = 0; }	
	/*
	ptr26++; if (ptr26 == numFIFO) { ptr26 = 0; }
	ptr27++; if (ptr27 == numFIFO) { ptr27 = 0; }
	ptr28++; if (ptr28 == numFIFO) { ptr28 = 0; }
	ptr29++; if (ptr29 == numFIFO) { ptr29 = 0; }
	ptr30++; if (ptr30 == numFIFO) { ptr30 = 0; }
	ptr31++; if (ptr31 == numFIFO) { ptr31 = 0; }
	ptr32++; if (ptr32 == numFIFO) { ptr32 = 0; }
	ptr33++; if (ptr33 == numFIFO) { ptr33 = 0; }
	ptr34++; if (ptr34 == numFIFO) { ptr34 = 0; }
	ptr35++; if (ptr35 == numFIFO) { ptr35 = 0; }
	ptr36++; if (ptr36 == numFIFO) { ptr36 = 0; }
	ptr37++; if (ptr37 == numFIFO) { ptr37 = 0; }
	ptr38++; if (ptr38 == numFIFO) { ptr38 = 0; }
	ptr39++; if (ptr39 == numFIFO) { ptr39 = 0; }
	ptr40++; if (ptr40 == numFIFO) { ptr40 = 0; }
	ptr41++; if (ptr41 == numFIFO) { ptr41 = 0; }
	ptr42++; if (ptr42 == numFIFO) { ptr42 = 0; }
	ptr43++; if (ptr43 == numFIFO) { ptr43 = 0; }
	ptr44++; if (ptr44 == numFIFO) { ptr44 = 0; }
	ptr45++; if (ptr45 == numFIFO) { ptr45 = 0; }
	ptr46++; if (ptr46 == numFIFO) { ptr46 = 0; }
	ptr47++; if (ptr47 == numFIFO) { ptr47 = 0; }
	ptr48++; if (ptr48 == numFIFO) { ptr48 = 0; }
	ptr49++; if (ptr49 == numFIFO) { ptr49 = 0; }
	ptr50++; if (ptr50 == numFIFO) { ptr50 = 0; }
*/

	return rectOut2.Output * K;
}

float BreathRateFilter::getBreathRate(float Data) {
	// rectOut.Time  �y���ݎ����z[�~���b]
	// rectOut.Output�y���ݒl�z
	float curTime = rectOut2.Time;
	//  float value = curData; //  rectOut.Output;
	f_value = Data;
	if (f_value > 0)
	{
		Peak.flag = 0;     //�y�s�[�N���o�t���O�z0:�ҋ@��
		Peak.minValue = 0;

	}
	else //�yvalue��0�z
	{
		if (f_value > Peak.minValue)
		{
			curTime = rectOut2.Time;                     //�y���ݎ�����擾����z
			if (Peak.pTime != 0)
			{
				PeakToPeak = curTime - Peak.pTime;
				Bcount++;
				Peak.HeartBeatRate = 60.0f / (PeakToPeak);    //�y��������Z����z�P��[��/��]
			}


			Peak.pTime = curTime;
			Peak.minValue = 0;
			Peak.flag = 1;     //�y�s�[�N���o�t���O�z1:���o
		}
		else
		{
			if (Peak.flag == 0) { Peak.minValue = f_value; }
		}
	if ((curTime - Peak.pTime) > PeakToPeak * 2)

		{
			return 0;
		}
	}
	return Peak.HeartBeatRate;
	//   return f_value;
}