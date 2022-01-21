#include <string.h>
#include "mat.hpp"

/********************************************************************************************/
/*																							*/
/* �J���}���t�B���^�Ƃ́C�덷���܂񂾊ϑ��ly�������ꂽ�Ƃ��ɁC���m�̏��ԕϐ�x�𐄒肷������	*/
/* ���ԕ����� x = A x + B u																	*/
/* �ϑ������� y = C x	 																	*/
/*																							*/
/* �^�̏��ԕϐ��� (x y ... vx vy ...)		x,y�����̈ʒu�Ƒ��x�i���m�j			xr�@�Ƃ���	*/
/* �ϑ��l�� (x y ...)						x,y�����̈ʒu�̂݁i���m�j			obs	�Ƃ���	*/
/* ���͒l�� (ax ay ...)						x,y�����̉����x�i���m�j				u	�Ƃ���	*/
/* ���肳�ꂽ���ԕϐ��� (x y ... vx vy ...)	x,y�����̈ʒu�Ƒ��x�i�����j			x	�Ƃ���	*/
/*																							*/
/********************************************************************************************/

class Kalman
{
private:
  int n; /* number of x ���ԕϐ��̎��� */
  int m; /* number of y �ϑ��ϐ��̎��� */
  int l; /* number of u ���͕ϐ��̎��� */

  double varw; /*  �����̌덷���U */
  double varu; /*  �����̌덷���U */
  double DT;   /*  �ϕ����� */

  double *x;    // �����l
  double *xt;   // �X�V���ꂽ�����l�i�t�B�[�h�o�b�N�O�j
  double *u;    // ����
  double *obs;  // ���ۂ̊ϑ��l
  double *A;
  double *B;
  double *C;
  double *U;
  double *W;
  double *P;

public:
  Kalman(int kn, int ks, int kl)
  {
    n = kn;
    m = ks;
    l = kl;

    x = new double[n];
    xt = new double[n];
    u = new double[l];
    obs = new double[m];
    A = new double[n * n];
    B = new double[n * l];
    C = new double[m * n];
    U = new double[l * l];
    W = new double[m * m];
    P = new double[n * n];
  }

  ~Kalman()
  {
    delete[] x;
    delete[] xt;
    delete[] u;
    delete[] obs;
    delete[] A;
    delete[] B;
    delete[] C;
    delete[] U;
    delete[] W;
    delete[] P;
  }
  double _nrandom(double variance)
  {
    int i;
    double regran = 0.0;

    for (i = 0; i < 12; i++)
      regran += (double)rand() / pow(2.0, 16.0) * 2.0;

    regran -= 6.0;
    return regran * sqrt(variance);
  }

  int getN()
  {
    return n;
  }
  int getM()
  {
    return m;
  }
  int getL()
  {
    return l;
  }

  double getX(int i)
  {
    return *(x + i);
  }
  double getVW()
  {
    return varw;
  }
  double getVU()
  {
    return varu;
  }

  void getA(double *a)
  {
    memcpy(a, A, n * n * sizeof(double));
  }
  void getB(double *b)
  {
    memcpy(b, B, n * l * sizeof(double));
  }
  void getC(double *c)
  {
    memcpy(c, C, m * n * sizeof(double));
  }
  void getU(double *u)
  {
    memcpy(u, U, l * l * sizeof(double));
  }
  void getW(double *w)
  {
    memcpy(w, W, m * m * sizeof(double));
  }
  void getP(double *p)
  {
    memcpy(p, P, n * n * sizeof(double));
  }

  void setX(int i, double s)
  {
    *(x + i) = s;
  }
  double setVW(double vw)
  {
    varw = vw;
    dzero(m, m, W);
    for (int i = 0; i < m; i++)
      *(W + i * m + i) = varw;
  }
  double setVU(double vu)
  {
    varu = vu;
    dzero(l, l, U);
    for (int i = 0; i < l; i++)
      *(U + i * l + i) = varu;
  }
  void setA(double *a)
  {
    memcpy(A, a, n * n * sizeof(double));
  }
  void setB(double *b)
  {
    memcpy(B, b, n * l * sizeof(double));
  }
  void setC(double *c)
  {
    memcpy(C, c, m * n * sizeof(double));
  }
  void setU(double *u)
  {
    memcpy(U, u, l * l * sizeof(double));
  }
  void setW(double *w)
  {
    memcpy(W, w, m * m * sizeof(double));
  }
  void setP(double *p)
  {
    memcpy(P, p, n * n * sizeof(double));
  }

  void init(double vw = 0.1, double vu = 0.1, double t = 0.1)
  {
    varw = vw;
    varu = vu;
    DT = t;

    dzero(n, 1, x);
    dzero(n, 1, xt);
    dzero(l, 1, u);
    dzero(m, 1, obs);

    deye(n, n, P);  // P�̏����l�@�K��

    // �ȉ��@�������x�^��������
    deye(n, n, A);
    for (int i = n / 2; i < n; i++)
      *(A + (i - n / 2) * n + i) = DT;

    dzero(n, l, B);
    for (int i = 0; i < l; i++)
      *(B + i * l + i) = DT * DT / 2;
    for (int i = 0; i < l; i++)
      *(B + (i + n / 2) * l + i) = DT;

    //�ϑ��͈ʒu
    dzero(m, n, C);
    for (int i = 0; i < m; i++)
      *(C + i * n + i) = 1.0;

    dzero(l, l, U);
    for (int i = 0; i < l; i++)
      *(U + i * l + i) = varu;
    dzero(m, m, W);
    for (int i = 0; i < m; i++)
      *(W + i * m + i) = varw;
  }

  void update(double *observ = NULL, double *input = NULL)
  {
    double *invW;
    double *PD;
    double **matnn;
    double **matnm;
    double **matmn;
    double **matnl;
    double **matln;
    double **vecn;
    double **vecm;

    invW = new double[m * m];
    PD = new double[n * n];

    const int temp = 5;
    matnn = new double *[temp];
    matnm = new double *[temp];
    matmn = new double *[temp];
    matnl = new double *[temp];
    matln = new double *[temp];
    vecn = new double *[temp];
    vecm = new double *[temp];

    for (int i = 0; i < temp; i++)
    {
      matnn[i] = new double[n * n];
      matnm[i] = new double[n * m];
      matmn[i] = new double[m * n];
      matnl[i] = new double[n * l];
      matln[i] = new double[l * n];
      vecn[i] = new double[n];
      vecm[i] = new double[m];
    }

    if (input == NULL)
    {
      memset(u, 0, l * sizeof(double));
    }
    else
    {
      memcpy(u, input, l * sizeof(double));
    }

    if (observ == NULL)
    {
      memset(obs, 0, m * sizeof(double));
    }
    else
    {
      memcpy(obs, observ, m * sizeof(double));
    }

    /***********************************************/
    /* ���������J���}���t�B���^�œ������� x �𐄒� */
    /* �^�����������ԗʂ͌덷���܂ފϑ��l obs ���� */

    /* PD = APA' + BUB' */
    dmat_mul(n, n, n, A, P, matnn[0]);

    dtranspose(n, n, A, matnn[1]);
    dmat_mul(n, n, n, matnn[0], matnn[1], matnn[2]);
    dmat_mul(n, l, l, B, U, matnl[0]);

    dtranspose(n, l, B, matln[0]);
    dmat_mul(n, l, n, matnl[0], matln[0], matnn[3]);
    dmat_add(n, n, matnn[2], matnn[3], PD);

    /* P= (PD^-1 + C'W^-1 C)^(-1) */
    //	dinverse22(PD,matnn[0]); // n ��2�̎�
    dinverse(n, n, PD, matnn[0]);

    dtranspose(m, n, C, matnm[0]);
    // dinverse22(W,invW); // m ��2�̎�
    dinverse(m, m, W, invW);

    dmat_mul(n, m, m, matnm[0], invW, matnm[1]);
    dmat_mul(n, m, n, matnm[1], C, matnn[1]);
    dmat_add(n, n, matnn[0], matnn[1], matnn[2]);

    //	dinverse22(matnn[2],P); // n ��2�̎�
    dinverse(n, n, matnn[2], P);

    /* ���ԕ������Ō덷���Ȃ��Ƃ����Ƃ��̐����l xt */
    /* xt = A x + B u */
    dmat_mul(n, n, 1, A, x, vecn[0]);
    dmat_mul(n, l, 1, B, u, vecn[1]);
    dmat_add(n, 1, vecn[0], vecn[1], xt);

    /* �덷���t�B�[�h�o�b�N���������� */
    /* x = xt + P C' W^-1 ( y - C xt) */
    dtranspose(m, n, C, matnm[0]);
    dmat_mul(n, n, m, P, matnm[0], matnm[1]);
    dmat_mul(n, m, m, matnm[1], invW, matnm[2]);

    dmat_mul(m, n, 1, C, xt, vecm[0]);
    dmat_sub(m, 1, obs, vecm[0], vecm[2]);

    dmat_mul(n, m, 1, matnm[2], vecm[2], vecn[0]);
    dmat_add(n, 1, xt, vecn[0], x);

    /***********************************************/

    delete[] invW;
    delete[] PD;
    for (int i = 0; i < temp; i++)
    {
      delete[] matnn[i];
      delete[] matnm[i];
      delete[] matmn[i];
      delete[] matnl[i];
      delete[] matln[i];
      delete[] vecn[i];
      delete[] vecm[i];
    }
    delete[] matnn;
    delete[] matnm;
    delete[] matmn;
    delete[] matnl;
    delete[] matln;
    delete[] vecn;
    delete[] vecm;
  }
};
