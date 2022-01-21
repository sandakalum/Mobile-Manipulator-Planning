const int framerate = 30;

class NoiseReduction {
private:

	int nSample;   // FIFO�i��
	int wPtr;      // FIFO�����݃|�C���^
	int rPtr;      // FIFO�Ǐo���|�C���^
	float a, b, c; // ������O(t)=a*t^2+b*t+c�̖��m��
	float d;       // �������̎��ld=-b/(2*a)
	float Ka;      // Ka=1.0f/��tj^4
	float Kb;      // Kb=1.0f/��tj^2
	float Kc;      // Kb=1.0f/n
	bool FIFOfull;  // FIFOfull flag

public:
	float C = 0.0f;
	float depthdata[framerate * 10];

	struct ParabolaData
	{
		float A; //�y�������W���z
		float B; //�y�������W���z
		float C; //�y�������W���z
		float D; //�y�������̎��zd=-b/(2*a)
	};

	NoiseReduction(int n);

	void Estimation(float data);

	int Length();
	int Pointer();
	bool Full();

public:

	ParabolaData ParabolaParameters();

private:
	ParabolaData Parabola_Data;

	void CalculateParabolaAxis();

	void ParabolaEstimation();
	
};