#define ea 1e-10
#define km 300
#define MAXN 100

inline double CalcStdDev(double *val, int iValCnt)
{
  double dSum = 0;
  double dMulSum = 0;
  for (int i = 0; i < iValCnt; i++)
  {
    dSum += val[i];
    dMulSum += val[i] * val[i];
  }
  double dAvg = dSum / iValCnt;
  return sqrt(dMulSum / iValCnt - dAvg * dAvg);
}

// 一様乱数を生成
inline double uniform_random()
{
  return (double)rand() / (double)RAND_MAX;
}

// 正規乱数を生成
inline double gaussian_random()
{
  static int next_gaussian = 0;
  static double saved_gaussian_value;

  double fac, rsq, v1, v2;

  if (next_gaussian == 0)
  {
    do
    {
      v1 = 2.0 * uniform_random() - 1.0;
      v2 = 2.0 * uniform_random() - 1.0;
      rsq = v1 * v1 + v2 * v2;
    } while (rsq >= 1.0 || rsq == 0.0);
    fac = sqrt(-2.0 * log(rsq) / rsq);
    saved_gaussian_value = v1 * fac;
    next_gaussian = 1;
    return v2 * fac;
  }
  else
  {
    next_gaussian = 0;
    return saved_gaussian_value;
  }
}

inline void dmat_mul(int l, int m, int n, void *m1, void *m2, void *m3)  //뛱쀱궻먑럁
{
  int i, j, k;
  for (i = 0; i < l; i++)
    for (j = 0; j < n; j++)
    {
      *((double *)m3 + n * i + j) = 0.0;
      for (k = 0; k < m; k++)
        *((double *)m3 + n * i + j) += *((double *)m1 + m * i + k) * *((double *)m2 + n * k + j);
    }
}

inline void dmat_add(int l, int m, void *m1, void *m2, void *m3)  //뛱쀱궻돿럁
{
  int i, j;

  for (i = 0; i < l; i++)
    for (j = 0; j < m; j++)
      *((double *)m3 + m * i + j) = *((double *)m1 + m * i + j) + *((double *)m2 + m * i + j);
}

inline void dmat_sub(int l, int m, void *m1, void *m2, void *m3)  //뛱쀱궻뙵럁
{
  int i, j;

  for (i = 0; i < l; i++)
    for (j = 0; j < m; j++)
      *((double *)m3 + m * i + j) = *((double *)m1 + m * i + j) - *((double *)m2 + m * i + j);
}

inline void dtranspose(int l, int m, void *m1, void *m2)  //?뭫뛱쀱
{
  int i, j;

  for (i = 0; i < l; i++)
    for (j = 0; j < m; j++)
      *((double *)m2 + l * j + i) = *((double *)m1 + m * i + j);
}

inline int dinverse(int l, int m, void *m1, void *m2)  //땤뛱쀱궻똶럁
{
  int i, j, k;
  double m3[50][100];

  for (i = 0; i < l; i++)
    for (j = 0; j < m; j++)
      m3[i][j] = *((double *)m1 + m * i + j);
  for (i = 0; i < l; i++)
    for (j = 0; j < m; j++)
      if (i == j)
        m3[i][j + m] = 1.0;
      else
        m3[i][j + m] = 0.0;
  for (k = 0; k < l; k++)
  {
    for (j = k + 1; j < 2 * m; j++)
    {
      /*
      if(fabs(m3[k][k]) < 1e-16)
      */
      if (fabs(m3[k][k]) == 0.0)
      {
        for (i = 0; i < l; i++)
          for (j = 0; j < m; j++)
            *((double *)m2 + m * i + j) = 0.0;
        return -1;
      }
      else
        m3[k][j] = m3[k][j] / m3[k][k];
    }
    m3[k][k] = 1.0;
    for (i = 0; i < l; i++)
      if (i != k)
      {
        for (j = k + 1; j < 2 * m; j++)
          m3[i][j] += -m3[i][k] * m3[k][j];
        m3[i][k] = 0.0;
      }
  }

  for (i = 0; i < l; i++)
    for (j = 0; j < m; j++)
      *((double *)m2 + m * i + j) = m3[i][j + m];
  return 1;
}

inline int dinverse22(double mat1[2][2], double mat2[2][2])  //괧?괧궻땤뛱쀱
{
  double delta;

  delta = mat1[0][0] * mat1[1][1] - mat1[1][0] * mat1[0][1];
  if (fabs(delta) > 1e-16)
  {
    mat2[0][0] = mat1[1][1] / delta;
    mat2[1][0] = -mat1[0][1] / delta;
    mat2[0][1] = -mat1[1][0] / delta;
    mat2[1][1] = mat1[0][0] / delta;
    return 1;
  }
  else
  {
    mat2[0][0] = mat2[0][1] = mat2[1][0] = mat2[1][1] = 0.0;
    return -1;
  }
}

inline int dinverse33(double mat1[3][3], double mat2[3][3])  //괨?괨궻땤뛱쀱
{
  double delta;
  double Min = 1e-10;

  delta = mat1[0][0] * mat1[1][1] * mat1[2][2] + mat1[1][0] * mat1[2][1] * mat1[0][2] +
          mat1[2][0] * mat1[0][1] * mat1[1][2] - mat1[0][2] * mat1[1][1] * mat1[2][0] -
          mat1[0][0] * mat1[1][2] * mat1[2][1] - mat1[0][1] * mat1[1][0] * mat1[2][2];

  if (fabs(delta) < Min)
    return -1;

  mat2[0][0] = (mat1[1][1] * mat1[2][2] - mat1[1][2] * mat1[2][1]) / delta;
  mat2[1][0] = -(mat1[1][0] * mat1[2][2] - mat1[1][2] * mat1[2][0]) / delta;
  mat2[2][0] = (mat1[1][0] * mat1[2][1] - mat1[1][1] * mat1[2][0]) / delta;
  mat2[0][1] = -(mat1[0][1] * mat1[2][2] - mat1[0][2] * mat1[2][1]) / delta;
  mat2[1][1] = (mat1[0][0] * mat1[2][2] - mat1[0][2] * mat1[2][0]) / delta;
  mat2[2][1] = -(mat1[0][0] * mat1[2][1] - mat1[0][1] * mat1[2][0]) / delta;
  mat2[0][2] = (mat1[0][1] * mat1[1][2] - mat1[0][2] * mat1[1][1]) / delta;
  mat2[1][2] = -(mat1[0][0] * mat1[1][2] - mat1[0][2] * mat1[1][0]) / delta;
  mat2[2][2] = (mat1[0][0] * mat1[1][1] - mat1[0][1] * mat1[1][0]) / delta;

  return 1;
}

inline void dprintmat(int l, int m, void *mat)  //뛱쀱궻뭷릆귩?렑
{
  int i, j;
  for (i = 0; i < l; i++)
  {
    for (j = 0; j < m; j++)
      printf("%12.6lf \t ", *((double *)mat + m * i + j));
    printf("\n");
  }
  printf("\n");
}

inline void dzero(int l, int m, void *m1)  //봹쀱궸괥귩묆볺
{
  int i, j;

  for (i = 0; i < l; i++)
    for (j = 0; j < m; j++)
      *((double *)m1 + m * i + j) = 0.0;
}

inline void deye(int l, int m, void *m1)  //뭁댧뛱쀱
{
  int i, j;

  for (i = 0; i < l; i++)
    for (j = 0; j < m; j++)
      if (i == j)
        *((double *)m1 + m * i + j) = 1.0;
      else
        *((double *)m1 + m * i + j) = 0.0;
}

inline void dmat_cpy(int l, int m, void *m1, void *m2)  //뛱쀱궻긓긯?
{
  int i, j;

  for (i = 0; i < l; i++)
    for (j = 0; j < m; j++)
      *((double *)m2 + m * i + j) = *((double *)m1 + m * i + j);
}

inline void roti(double phi, void *m)  //굹렡뢂귟궻됷?뛱쀱
{
  *((double *)m + 3 * 0 + 0) = 1.0;
  *((double *)m + 3 * 0 + 1) = 0.0;
  *((double *)m + 3 * 0 + 2) = 0.0;
  *((double *)m + 3 * 1 + 0) = 0.0;
  *((double *)m + 3 * 1 + 1) = cos(phi);
  *((double *)m + 3 * 1 + 2) = -sin(phi);
  *((double *)m + 3 * 2 + 0) = 0.0;
  *((double *)m + 3 * 2 + 1) = sin(phi);
  *((double *)m + 3 * 2 + 2) = cos(phi);
}

inline void rotj(double phi, void *m)  //굺렡뢂귟궻됷?뛱쀱
{
  *((double *)m + 3 * 0 + 0) = cos(phi);
  *((double *)m + 3 * 0 + 1) = 0.0;
  *((double *)m + 3 * 0 + 2) = sin(phi);
  *((double *)m + 3 * 1 + 0) = 0.0;
  *((double *)m + 3 * 1 + 1) = 1.0;
  *((double *)m + 3 * 1 + 2) = 0.0;
  *((double *)m + 3 * 2 + 0) = -sin(phi);
  *((double *)m + 3 * 2 + 1) = 0.0;
  *((double *)m + 3 * 2 + 2) = cos(phi);
}

inline void rotk(double phi, void *m)  //굾렡뢂귟궻됷?뛱쀱
{
  *((double *)m + 3 * 0 + 0) = cos(phi);
  *((double *)m + 3 * 0 + 1) = -sin(phi);
  *((double *)m + 3 * 0 + 2) = 0.0;
  *((double *)m + 3 * 1 + 0) = sin(phi);
  *((double *)m + 3 * 1 + 1) = cos(phi);
  *((double *)m + 3 * 1 + 2) = 0.0;
  *((double *)m + 3 * 2 + 0) = 0.0;
  *((double *)m + 3 * 2 + 1) = 0.0;
  *((double *)m + 3 * 2 + 2) = 1.0;
}

inline void rot(double x, double y, double z, double phi, void *m)  //任意の軸に回転
{
  double r = sqrt(x * x + y * y + z * z);
  x = x / r;
  y = y / r;
  z = z / r;
  *((double *)m + 3 * 0 + 0) = (x * x * (1 - cos(phi)) + cos(phi));
  *((double *)m + 3 * 0 + 1) = (x * y * (1 - cos(phi)) + z * sin(phi));
  *((double *)m + 3 * 0 + 2) = (x * z * (1 - cos(phi)) - y * sin(phi));
  *((double *)m + 3 * 1 + 0) = (x * y * (1 - cos(phi)) - z * sin(phi));
  *((double *)m + 3 * 1 + 1) = (y * y * (1 - cos(phi)) + cos(phi));
  *((double *)m + 3 * 1 + 2) = (y * z * (1 - cos(phi)) + x * sin(phi));
  *((double *)m + 3 * 2 + 0) = (x * z * (1 - cos(phi)) + y * sin(phi));
  *((double *)m + 3 * 2 + 1) = (y * z * (1 - cos(phi)) - x * sin(phi));
  *((double *)m + 3 * 2 + 2) = (z * z * (1 - cos(phi)) + cos(phi));
}

inline void makeSRPYrotateMatrix(double rotateM[3][3], double stage, double roll, double pitch, double yaw)
{
  // SRPY rotation = R(Yaw)*R(Pitch)*R(Roll)*R(Stage)

  double rotateM_Pitch[3][3];
  double rotateM_Roll[3][3];
  double rotateM_Yaw[3][3];
  double rotateM_Stage[3][3];

  roti(roll, rotateM_Roll);
  rotj(pitch, rotateM_Pitch);
  rotk(yaw, rotateM_Yaw);
  rotk(stage, rotateM_Stage);

  double rotateM_Buf1[3][3];
  double rotateM_Buf2[3][3];
  dmat_mul(3, 3, 3, rotateM_Yaw, rotateM_Pitch, rotateM_Buf1);
  dmat_mul(3, 3, 3, rotateM_Buf1, rotateM_Roll, rotateM_Buf2);
  dmat_mul(3, 3, 3, rotateM_Buf2, rotateM_Stage, rotateM);
}

inline void makeSRPYrotateMatrix_Inverse(double rotateM[3][3], double stage, double roll, double pitch, double yaw)
{
  // SRPY rotation inverse = R(-Stage)*R(-Roll)*R(-Pitch)*R(-Yaw)

  double rotateM_Pitch[3][3];
  double rotateM_Roll[3][3];
  double rotateM_Yaw[3][3];
  double rotateM_Stage[3][3];

  roti(-roll, rotateM_Roll);
  rotj(-pitch, rotateM_Pitch);
  rotk(-yaw, rotateM_Yaw);
  rotk(-stage, rotateM_Stage);

  double rotateM_Buf1[3][3];
  double rotateM_Buf2[3][3];
  dmat_mul(3, 3, 3, rotateM_Stage, rotateM_Roll, rotateM_Buf1);
  dmat_mul(3, 3, 3, rotateM_Buf1, rotateM_Pitch, rotateM_Buf2);
  dmat_mul(3, 3, 3, rotateM_Buf2, rotateM_Yaw, rotateM);
}

inline void calcOffsetWithMotion(double resOffset[3], int x, int y, int z, double pitch, double roll, double yaw,
                                 bool acceptMotionEffect)
{
  double offset[3] = {(double)x, (double)y, (double)z};
  double rotateM[3][3];
  if (acceptMotionEffect)
    makeSRPYrotateMatrix(rotateM, 0, roll, pitch, yaw);
  else
    makeSRPYrotateMatrix(rotateM, 0, 0, 0, yaw);
  dmat_mul(3, 3, 1, rotateM, offset, resOffset);
}

inline void rot(double phi, double *v, void *m)
{
  *((double *)m + 3 * 0 + 0) = cos(phi) + (*v) * (*v) * (1.0 - cos(phi));
  *((double *)m + 3 * 0 + 1) = (*v) * (*(v + 1)) * (1.0 - cos(phi)) - (*(v + 2)) * sin(phi);
  *((double *)m + 3 * 0 + 2) = (*v) * (*(v + 2)) * (1.0 - cos(phi)) + (*(v + 1)) * sin(phi);
  *((double *)m + 3 * 1 + 0) = (*v) * (*(v + 1)) * (1.0 - cos(phi)) + (*(v + 2)) * sin(phi);
  *((double *)m + 3 * 1 + 1) = cos(phi) + (*(v + 1)) * (*(v + 1)) * (1.0 - cos(phi));
  *((double *)m + 3 * 1 + 2) = (*(v + 1)) * (*(v + 2)) * (1.0 - cos(phi)) - (*v) * sin(phi);
  *((double *)m + 3 * 2 + 0) = (*v) * (*(v + 2)) * (1.0 - cos(phi)) - (*(v + 1)) * sin(phi);
  *((double *)m + 3 * 2 + 1) = (*(v + 1)) * (*(v + 2)) * (1.0 - cos(phi)) + (*v) * sin(phi);
  *((double *)m + 3 * 2 + 2) = cos(phi) + (*(v + 2)) * (*(v + 2)) * (1.0 - cos(phi));
}

inline void cross3(double *v1, double *v2, double *v3)
{
  *(v3 + 0) = *(v1 + 1) * *(v2 + 2) - *(v1 + 2) * *(v2 + 1);
  *(v3 + 1) = -(*(v1 + 0) * *(v2 + 2) - *(v1 + 2) * *(v2 + 0));
  *(v3 + 2) = *(v1 + 0) * *(v2 + 1) - *(v1 + 1) * *(v2 + 0);
}

inline double dot(int l, double *v1, double *v2)
{
  double sum = 0;
  for (int i = 0; i < l; i++)
    sum += v1[i] * v2[i];
  return sum;
}

inline void nsa_to_abg(double mat[3][3], double vec[3])
{
  double normal[3];
  double slide[3];
  double approach[3];
  double alpha, beta, gamma;
  double MINLBL = 0.01;

  normal[0] = mat[0][0];
  normal[1] = mat[1][0];
  normal[2] = mat[2][0];
  slide[0] = mat[0][1];
  slide[1] = mat[1][1];
  slide[2] = mat[2][1];
  approach[0] = mat[0][2];
  approach[1] = mat[1][2];
  approach[2] = mat[2][2];

  if (sqrt(approach[0] * approach[0] + approach[1] * approach[1]) > MINLBL)
    if (fabs(approach[0]) < MINLBL)
      alpha = M_PI / 2.0;
    else
      alpha = atan2(approach[1], approach[0]);
  else
    alpha = 0.0;

  if (sin(alpha) > MINLBL)
    beta = atan2(approach[1] / sin(alpha), approach[2]);
  else
    beta = atan2(approach[0] / cos(alpha), approach[2]);

  if (sin(beta) > MINLBL)
    gamma = atan2(slide[2], -normal[2]);
  else
    gamma = atan2(cos(alpha) * normal[1] - sin(alpha) * normal[0], cos(alpha) * slide[1] - sin(alpha) * slide[0]);
  vec[0] = alpha;
  vec[1] = beta;
  vec[2] = gamma;
}

inline void abg_to_nsa(double vec[3], double mat[3][3])
{
  double normal[3];
  double slide[3];
  double approach[3];
  double alpha, beta, gamma;

  alpha = vec[0];
  beta = vec[1];
  gamma = vec[2];

  normal[0] = cos(alpha) * cos(beta) * cos(gamma) - sin(alpha) * sin(gamma);
  normal[1] = sin(alpha) * cos(beta) * cos(gamma) + cos(alpha) * sin(gamma);
  normal[2] = -sin(beta) * cos(gamma);

  slide[0] = -cos(alpha) * cos(beta) * sin(gamma) - sin(alpha) * cos(gamma);
  slide[1] = -sin(alpha) * cos(beta) * cos(gamma) + cos(alpha) * cos(gamma);
  slide[2] = sin(beta) * sin(gamma);

  approach[0] = cos(alpha) * sin(beta);
  approach[1] = sin(alpha) * sin(beta);
  approach[2] = cos(beta);

  mat[0][0] = normal[0];
  mat[1][0] = normal[1];
  mat[2][0] = normal[2];
  mat[0][1] = slide[0];
  mat[1][1] = slide[1];
  mat[2][1] = slide[2];
  mat[0][2] = approach[0];
  mat[1][2] = approach[1];
  mat[2][2] = approach[2];
}

/*
inline void qs(int low, int high,t_type** table)
{
int i,j;
double x;
struct t_type *tmp;

if(low<high){
i=low;
j=high;
x=table[i]->lev;
while(i<=j){
while(table[i]->lev<x) i++;
while(table[j]->lev>x) j--;
if(i<=j){
tmp=table[i]; table[i] = table[j]; table[j] = tmp;
i++;
j--;
}
}
qs(low,j,table);
qs(i,high,table);
}
}
*/

inline void djacobi_method(int n, void *a, void *vec, void *val, int *found)
{
  int i, j, i1, j1;
  int k;
  double s1, s2, ax, a0, z1, z2, z3, si, co;
  double tmp[MAXN];

  for (i = 0; i < n; i++)
    for (j = 0; j < n; j++)
      *((double *)tmp + n * i + j) = *((double *)a + n * i + j);

  for (i = 0; i < n; i++)
    for (j = 0; j < n; j++)
      if (i == j)
        *((double *)vec + n * i + j) = 1;
      else
        *((double *)vec + n * i + j) = 0;

  k = 1;
  do
  {
    ax = 0;
    for (i = 0; i < n - 1; i++)
      for (j = i + 1; j < n; j++)
        if (fabs(*((double *)a + n * i + j)) > ax)
        {
          i1 = i;
          j1 = j;
          ax = fabs(*((double *)a + n * i + j));
        }
    if (ax != 0)
    {
      z1 = *((double *)a + n * i1 + i1) - *((double *)a + n * j1 + j1);
      z2 = fabs(z1) / 2;
      z3 = sqrt(z2 * z2 + *((double *)a + n * i1 + j1) * *((double *)a + n * i1 + j1));
      co = 1 / sqrt(2.0) * sqrt(1.0 + z2 / z3);
      si = *((double *)a + n * i1 + j1) / (2.0 * z3 * co);
      if ((z1) < 0)
        si = -si;
      for (i = 0; i < n; i++)
        if ((i != i1) && (i != j1))
        {
          s1 = *((double *)a + n * i1 + i) * co + *((double *)a + n * j1 + i) * si;
          s2 = *((double *)a + n * j1 + i) * co - *((double *)a + n * i1 + i) * si;
          *((double *)a + n * i1 + i) = s1;
          *((double *)a + n * j1 + i) = s2;
        }
      a0 = 2 * *((double *)a + n * i1 + j1) * co * si;
      s1 = *((double *)a + n * i1 + i1) * co * co + *((double *)a + n * j1 + j1) * si * si + a0;
      s2 = *((double *)a + n * j1 + j1) * co * co + *((double *)a + n * i1 + i1) * si * si - a0;
      *((double *)a + n * i1 + i1) = s1;
      *((double *)a + n * j1 + j1) = s2;
      *((double *)a + n * i1 + j1) = 0;
      *((double *)a + n * j1 + i1) = 0;
      for (i = 0; i < n; i++)
      {
        *((double *)a + n * i + i1) = *((double *)a + n * i1 + i);
        *((double *)a + n * i + j1) = *((double *)a + n * j1 + i);
        s1 = *((double *)vec + n * i + i1) * co + *((double *)vec + n * i + j1) * si;
        s2 = *((double *)vec + n * i + j1) * co - *((double *)vec + n * i + i1) * si;
        *((double *)vec + n * i + i1) = s1;
        *((double *)vec + n * i + j1) = s2;
      }
    }
    k = k + 1;
  } while ((fabs(ax) > ea) && (k <= km));
  for (i = 0; i < n; i++)
    *((double *)val + i) = *((double *)a + n * i + i);
  if (k >= km)
    *found = -1;
  else
    *found = 1;

  for (i = 0; i < n; i++)
    for (j = 0; j < n; j++)
      *((double *)a + n * i + j) = *((double *)tmp + n * i + j);
}

inline void dmsort(int n, void *m, void *r, int *stabl)
{
  int i, j;
  for (j = 0; j < n; j++)
    for (i = 0; i < n; i++)
      *((double *)r + n * i + j) = *((double *)m + n * i + *(stabl + j));
}

inline void dminvsub(int n, void *dd, void *vv, int *stabl)
{
  int i, j, k;
  int sam;
  double max, mmax;
  double r[MAXN], mat1[MAXN];

  for (i = 0; i < n; i++)
    for (j = 0; j < n; j++)
      *((double *)r + n * i + j) = 0;

  mmax = 1e9;
  for (i = 0; i < n; i++)
  {
    max = 0;
    for (j = 0; j < n; j++)
    {
      sam = 0;
      if ((max < *((double *)dd + n * j + j)) && (mmax >= *((double *)dd + n * j + j)))
      {
        if (mmax == *((double *)dd + n * j + j))
        {
          sam = 0;
          for (k = 0; k < i; k++)
            if (j == *(stabl + k))
              sam = 1;
        }
        if (sam != 1)
        {
          max = *((double *)dd + n * j + j);
          *(stabl + i) = j;
          *((double *)r + n * i + i) = max;
        }
      }
    }
    mmax = max;
  }
  dmsort(n, vv, mat1, stabl);
  dmat_cpy(n, n, mat1, vv);
  dmat_cpy(n, n, r, dd);
}

inline void dminv2(int m, int n, void *mt, void *mi, int getdof)
{
  int i, j;
  double m2[MAXN], m3[MAXN];
  int found, dof;
  double vecN[MAXN];
  double d[MAXN], sqd[MAXN], v[MAXN], u[MAXN], ut[MAXN];
  int stabl[MAXN];

  for (i = 0; i < n; i++)
    stabl[i] = 1;
  dtranspose(m, n, mt, m3);

  dmat_mul(n, m, n, m3, mt, m2);

  djacobi_method(n, m2, v, vecN, &found);

  for (i = 0; i < n; i++)
    for (j = 0; j < n; j++)
      if (i == j)
        *((double *)d + n * i + i) = vecN[i];
      else
        *((double *)d + n * i + j) = 0;

  dminvsub(n, d, v, stabl);

  /*
  for(i=0;i<n;i++)
  if(fabs(sqd[i][i])<ea) d[i][i] = 0;
  else sqd[i][i] = 1/sqrt(d[i][i]);
  */

  dof = 0;
  for (i = 0; i < n; i++)
    for (j = 0; j < n; j++)
    {
      if (i == j)
      {
        if (dof < getdof)
        {
          *((double *)sqd + n * i + i) = 1 / sqrt(*((double *)d + n * i + i));
          dof = dof + 1;
        }
        else
          *((double *)sqd + n * i + i) = 0;
      }
      else
        *((double *)sqd + n * i + j) = 0;
    }

  dmat_mul(m, n, n, mt, v, m3);
  dmat_mul(m, n, n, m3, sqd, u);

  dtranspose(m, n, u, ut);
  dmat_mul(n, n, n, v, sqd, m3);
  dmat_mul(n, n, m, m3, ut, mi);
}
