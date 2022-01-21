#include <string.h>
#include "mat.hpp"

/********************************************************************************************/
/*																							*/
/* カルマンフィルタとは，誤差を含んだ観測値yが得られたときに，未知の状態変数xを推定するもの	*/
/* 状態方程式 x = A x + B u																	*/
/* 観測方程式 y = C x	 																	*/
/*																							*/
/* 真の状態変数は (x y ... vx vy ...)		x,y方向の位置と速度（未知）			xr　とする	*/
/* 観測値は (x y ...)						x,y方向の位置のみ（既知）			obs	とする	*/
/* 入力値は (ax ay ...)						x,y方向の加速度（既知）				u	とする	*/
/* 推定された状態変数は (x y ... vx vy ...)	x,y方向の位置と速度（推定）			x	とする	*/
/*																							*/
/********************************************************************************************/

class Kalman
{
private:
  int n; /* number of x 状態変数の次元 */
  int m; /* number of y 観測変数の次元 */
  int l; /* number of u 入力変数の次元 */

  double varw; /*  推定の誤差分散 */
  double varu; /*  推定の誤差分散 */
  double DT;   /*  積分時間 */

  double *x;    // 推定値
  double *xt;   // 更新された推定値（フィードバック前）
  double *u;    // 入力
  double *obs;  // 実際の観測値
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

    deye(n, n, P);  // Pの初期値　適当

    // 以下　等加速度運動を仮定
    deye(n, n, A);
    for (int i = n / 2; i < n; i++)
      *(A + (i - n / 2) * n + i) = DT;

    dzero(n, l, B);
    for (int i = 0; i < l; i++)
      *(B + i * l + i) = DT * DT / 2;
    for (int i = 0; i < l; i++)
      *(B + (i + n / 2) * l + i) = DT;

    //観測は位置
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
    /* ここからカルマンフィルタで内部状態 x を推定 */
    /* 与えられる状態量は誤差を含む観測値 obs だけ */

    /* PD = APA' + BUB' */
    dmat_mul(n, n, n, A, P, matnn[0]);
    dtranspose(n, n, A, matnn[1]);
    dmat_mul(n, n, n, matnn[0], matnn[1], matnn[2]);

    dmat_mul(n, l, l, B, U, matnl[0]);
    dtranspose(n, l, B, matln[0]);
    dmat_mul(n, l, n, matnl[0], matln[0], matnn[3]);

    dmat_add(n, n, matnn[2], matnn[3], PD);

    /* P= (PD^-1 + C'W^-1 C)^(-1) */
    //	dinverse22(PD,matnn[0]); // n が2の時
    dinverse(n, n, PD, matnn[0]);

    dtranspose(m, n, C, matnm[0]);
    // dinverse22(W,invW); // m が2の時
    dinverse(m, m, W, invW);
    dmat_mul(n, m, m, matnm[0], invW, matnm[1]);
    dmat_mul(n, m, n, matnm[1], C, matnn[1]);

    dmat_add(n, n, matnn[0], matnn[1], matnn[2]);

    //	dinverse22(matnn[2],P); // n が2の時
    dinverse(n, n, matnn[2], P);

    /* 状態方程式で誤差がないとしたときの推定値 xt */
    /* xt = A x + B u */
    dmat_mul(n, n, 1, A, x, vecn[0]);
    dmat_mul(n, l, 1, B, u, vecn[1]);
    dmat_add(n, 1, vecn[0], vecn[1], xt);

    /* 誤差をフィードバックした推定量 */
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
