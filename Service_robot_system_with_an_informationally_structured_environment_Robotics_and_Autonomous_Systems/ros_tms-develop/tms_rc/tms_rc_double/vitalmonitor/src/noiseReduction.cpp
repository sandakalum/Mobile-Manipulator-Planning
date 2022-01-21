#include "noiseReduction.h"




NoiseReduction::NoiseReduction(int n)
{
	float t, t2;
	nSample = n;
			
	wPtr = rPtr = 0;
	Ka = Kb = 0.0f;
	for (int i = 0; i < nSample; i++)
	{
		t = (float)(-i); // *0.033f; //�y1�t���[����33mS�z
		depthdata[i] = 0.0f;  //�yFIFO��0�N���A�z
		t2 = t * t;      //(-tj)^2
		Ka += t2 * t2;   //��(-tj)^4
		Kb += t2;        //��(-tj)^2
	}
	Ka = 1.0f / Ka;
	Kb = 1.0f / Kb;
	Kc = 1.0f / (float)nSample;
	FIFOfull = false;
}


void NoiseReduction::Estimation(float data)
{
	depthdata[wPtr] = data;
	rPtr = wPtr;
	//�y�����݃|�C���^�̍X�V�z
	wPtr++;
	if (wPtr == nSample)
	{
		wPtr = 0; FIFOfull = true;
	}

	//�yBase Transition Rule�ŕ�����a*t^2+b*t+c�𐄒肷��z�P�O����s
	if (FIFOfull)
	{
		ParabolaEstimation();
		ParabolaEstimation();
		ParabolaEstimation();
		ParabolaEstimation();
		ParabolaEstimation();
		ParabolaEstimation();
		ParabolaEstimation();
		ParabolaEstimation();
		ParabolaEstimation();
		ParabolaEstimation();

		ParabolaEstimation();
		ParabolaEstimation();
		ParabolaEstimation();
		ParabolaEstimation();
		ParabolaEstimation();
		ParabolaEstimation();
		ParabolaEstimation();
		ParabolaEstimation();
		ParabolaEstimation();
		ParabolaEstimation();

			//�y���肵��������a*t^2+b*t+c���玲�ld=-b/(2*a)��v�Z�z
		CalculateParabolaAxis();
	}
	
	if (FIFOfull) {
		C = c;
	}
	else {
		C = 0.0f;
	}


}

int NoiseReduction::Length(){ 
	return nSample; 
}
int NoiseReduction::Pointer(){
	return wPtr; 
}
bool NoiseReduction::Full(){
	return FIFOfull;
}
/*
		float A(){
			if (FIFOfull) {
				return a; 
			} else { 
				return 0.0f; 
			} 
		} 
		float B(){
			if (FIFOfull) {
				return b; 
			}else {
				return 0.0f; 
			}  
		}
		float C(){
			if (FIFOfull) { 
				return c;
			}else {
				return 0.0f;
			} 
		} 
		float D(){
			if (FIFOfull) {
				return d;
			}else { 
				return -1000.0f; 
			} 
		}*/ 


		
NoiseReduction::ParabolaData NoiseReduction::ParabolaParameters()
{
	if (FIFOfull)
	{
		Parabola_Data.A = a;
		Parabola_Data.B = b;
		Parabola_Data.C = c;
		Parabola_Data.D = d;
	}
	else
	{
		Parabola_Data.A = 0.0f;
		Parabola_Data.B = 0.0f;
		Parabola_Data.C = 0.0f;
		Parabola_Data.D = -1000.0f;
	}
	return Parabola_Data;
}

void NoiseReduction::CalculateParabolaAxis()
{
	d = -b / (2.0f * a);
}

void NoiseReduction::ParabolaEstimation()
{
	int j;
		float t, t2, t4, e;
		float Sa, Sb, Sc; // (working parameter) Sa=��t^4*(Sj-Oj), Sb=��t^2*(Sj-Oj), Sc=��(Sj-Oj)
						  //�yEstimate Xc�z
	Sa = 0.0f;
	for (j = 0; j < nSample; j++)
	{
		t = (float)(-j); // *0.033f;
		t2 = t * t;
		t4 = t2 * t2;
		e = depthdata[rPtr] - (a * t2 + b * t + c);
		Sa += t2 * e;
		//�y�Ǐo���|�C���^�̍X�V�z
		rPtr--; if (rPtr < 0) { rPtr = nSample - 1; }
	}
	a += Ka * Sa;
	//�yEstimate Xc�z
	Sb = 0.0f;
	for (j = 0; j < nSample; j++)
	{
		t = (float)(-j); // *0.033f;
		t2 = t * t;
		e = depthdata[rPtr] - (a * t2 + b * t + c);
		Sb += t * e;
		//�y�Ǐo���|�C���^�̍X�V�z
		rPtr--; if (rPtr < 0) { rPtr = nSample - 1; }
	}
	b += Kb * Sb;
	//�yEstimate Xc�z
	Sc = 0.0f;
	for (j = 0; j < nSample; j++)
	{
		t = (float)(-j); // *0.033f;
		t2 = t * t;
		e = depthdata[rPtr] - (a * t2 + b * t + c);
		Sc += e;
		//�y�Ǐo���|�C���^�̍X�V�z
		rPtr--; if (rPtr < 0) { rPtr = nSample - 1; }
	}
	c += Kc * Sc;
}
