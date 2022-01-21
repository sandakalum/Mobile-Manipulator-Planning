/*
 *   Katana Native Interface - A C++ interface to the robot arm Katana.
 *   Copyright (C) 2005 Neuronics AG
 *   Check out the AUTHORS file for detailed contact information.
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include "KNI_LM/lmBase.h"

#include <iostream>

bool VDEBUG = false;
const int MINIMAL_POLY_DISTANCE = 16;
/****************************************************************************/
/****************************************************************************/

// Linear movement using multiple splines
void CLMBase::movLM2P(double X1, double Y1, double Z1, double Ph1, double Th1, double Ps1, double X2, double Y2,
                      double Z2, double Ph2, double Th2, double Ps2, bool exactflag, double vmax, bool wait,
                      int tolerance, long timeout)
{
  // check if the robot buffer is ready to receive a new linear movement
  bool motors_ready = false;
  while (!motors_ready)
  {
    motors_ready = true;
    for (int idx = 0; idx < getNumberOfMotors() - 1; idx++)
    {
      base->GetMOT()->arr[idx].recvPVP();
      motors_ready &= (base->GetMOT()->arr[idx].GetPVP()->msf != 152);
    }
  }

  // distance between the two points
  double distance = sqrt(pow(X2 - X1, 2.0) + pow(Y2 - Y1, 2.0) + pow(Z2 - Z1, 2.0));

  // acceleration limits in mm/s^2, hardcoded for now
  double acc = 1500;
  double dec = 1500;

  // calculate time for whole movement
  double totaltime = totalTime(distance, acc, dec, vmax);

  // calculate number of splines needed
  double maxtimeperspline = 0.5;
  int steps = (int)(totaltime / maxtimeperspline) + 1;
  short timeperspline;
  timeperspline = (short)floor(100 * (totaltime / (steps)) + 1);

  // calculate intermediate points
  int numberofmotors = getNumberOfMotors();
  int i, j;
  double* timearray = new double[steps + 1];
  double** dataarray = new double* [steps + 1];
  for (i = 0; i < (steps + 1); i++)
    dataarray[i] = new double[numberofmotors];
  double relposition, time, lasttime, x, y, z, phi, theta, psi;
  lasttime = 0;
  std::vector< int > solution(numberofmotors, 0), lastsolution(numberofmotors, 0);
  for (i = 0; i <= steps; i++)
  {
    // calculate parameters for i-th position
    if (i < steps)
      time = 0.01 * i * (double)timeperspline;
    else
      time = totaltime;

    relposition = relPosition((double)time, distance, acc, dec, vmax);
    x = X1 + relposition * (X2 - X1);
    y = Y1 + relposition * (Y2 - Y1);
    z = Z1 + relposition * (Z2 - Z1);
    phi = Ph1 + relposition * (Ph2 - Ph1);
    theta = Th1 + relposition * (Th2 - Th1);
    psi = Ps1 + relposition * (Ps2 - Ps1);

    // check kinematics
    try
    {
      IKCalculate(x, y, z, phi, theta, psi, solution.begin());
    }
    catch (Exception NoSolutionException)
    {
      throw KNI::NoSolutionException();
    }

    // store data
    for (j = 0; j < numberofmotors; j++)
    {
      dataarray[i][j] = (double)solution.at(j);
    }
    timearray[i] = time;

    // check joint speeds, stop program if failed
    if (time > 0)
    {
      if (!checkJointSpeed(lastsolution, solution, (time - lasttime)))
      {
        throw JointSpeedException();
      }
    }
    lasttime = time;
    lastsolution.clear();
    lastsolution.assign(solution.begin(), solution.end());
  }

  // calculate spline
  short*** parameters = new short** [steps];
  for (i = 0; i < steps; i++)
    parameters[i] = new short* [numberofmotors];
  for (i = 0; i < steps; i++)
    for (j = 0; j < numberofmotors; j++)
      parameters[i][j] = new short[7];
  double* encoderarray = new double[steps + 1];
  double* arr_p1 = new double[steps];
  double* arr_p2 = new double[steps];
  double* arr_p3 = new double[steps];
  double* arr_p4 = new double[steps];
  double s_time;
  for (i = 0; i < numberofmotors; i++)
  {
    // one motor at a time
    for (j = 0; j <= steps; j++)
    {
      encoderarray[j] = dataarray[j][i];
    }
    splineCoefficients(steps, timearray, encoderarray, arr_p1, arr_p2, arr_p3, arr_p4);
    // store parameters for G command to motor i
    for (j = 0; j < steps; j++)
    {
      // motor number
      parameters[j][i][0] = (short)i;
      // targetencoder
      parameters[j][i][1] = (short)encoderarray[j + 1];
      // robot time (in 10ms steps)
      s_time = (timearray[j + 1] - timearray[j]) * 100;
      if (j < steps - 1)
        parameters[j][i][2] = (short)timeperspline;
      else
        parameters[j][i][2] = (short)s_time;
      // the four spline coefficients
      // the actual position, round
      parameters[j][i][3] = (short)floor(arr_p1[j] + 0.5);
      // shift to be firmware compatible and round
      parameters[j][i][4] = (short)floor(64 * arr_p2[j] / s_time + 0.5);
      parameters[j][i][5] = (short)floor(1024 * arr_p3[j] / pow(s_time, 2) + 0.5);
      parameters[j][i][6] = (short)floor(32768 * arr_p4[j] / pow(s_time, 3) + 0.5);
    }
  }

  // send spline
  long spline_timeout = (long)parameters[0][0][2] * 10;  // - 2;
  KNI::Timer t(timeout), spline_t(spline_timeout);
  t.Start();
  spline_t.Start();
  // wait for motor
  int wait_timeout = 5000;
  if (mKatanaType == 450)
  {
    int totalsplinetime = 0;
    for (i = 0; i < steps; i++)
    {
      // ignore further steps if timeout elapsed
      if (t.Elapsed())
        break;
      // calculate total time from beginning of spline
      totalsplinetime += parameters[i][0][2] * 10;
      // set and start movement
      int activityflag = 0;
      if (i == (steps - 1))
      {
        // last spline, start movement
        activityflag = 1;  // no_next
      }
      else if (totalsplinetime < 400)
      {
        // more splines following, do not start movement yet
        activityflag = 2;  // no_start
      }
      else
      {
        // more splines following, start movement
        activityflag = 0;
      }
      // spline_t.Start();
      std::vector< short > polynomial;
      for (j = 0; j < numberofmotors; j++)
      {
        polynomial.push_back(parameters[i][j][2]);  // time
        polynomial.push_back(parameters[i][j][1]);  // target
        polynomial.push_back(parameters[i][j][3]);  // p0
        polynomial.push_back(parameters[i][j][4]);  // p1
        polynomial.push_back(parameters[i][j][5]);  // p2
        polynomial.push_back(parameters[i][j][6]);  // p3
      }
      setAndStartPolyMovement(polynomial, exactflag, activityflag);
      // std::cout << "time to send and start poly: " << spline_t.ElapsedTime() << "ms" << std::endl;
    }
  }
  else if (mKatanaType == 400)
  {
    int totalsplinetime = 0;
    for (i = 0; i < steps; i++)
    {
      // ignore further steps if timeout elapsed
      if (t.Elapsed())
        break;
      // send parameters
      // spline_t.Start();
      for (j = 0; j < numberofmotors; j++)
      {
        sendSplineToMotor((unsigned short)parameters[i][j][0], parameters[i][j][1], parameters[i][j][2],
                          parameters[i][j][3], parameters[i][j][4], parameters[i][j][5], parameters[i][j][6]);
      }
      totalsplinetime += parameters[i][0][2] * 10;
      // start movement
      if (i == (steps - 1))
      {
        // last spline, start movement
        startSplineMovement(exactflag, 1);
      }
      else if (totalsplinetime < 400)
      {
        // more splines following, do not start movement yet
        startSplineMovement(exactflag, 2);
      }
      else
      {
        // more splines following, start movement
        startSplineMovement(exactflag, 0);
      }
      // std::cout << "time to send and start poly: " << spline_t.ElapsedTime() << "ms" << std::endl;
    }
  }
  else
  {
    for (i = 0; i < steps; i++)
    {
      // ignore further steps if timeout elapsed
      if (t.Elapsed())
        break;
      // wait for motor to finish movement
      waitForMotor(0, 0, tolerance, 2, wait_timeout);
      // send parameters
      for (j = 0; j < numberofmotors; j++)
      {
        sendSplineToMotor((unsigned short)parameters[i][j][0], parameters[i][j][1], parameters[i][j][2],
                          parameters[i][j][3], parameters[i][j][4], parameters[i][j][5], parameters[i][j][6]);
      }
      // start movement
      startSplineMovement(exactflag);
    }
  }
  // std::cout << "time to send and start linmov: " << t.ElapsedTime() << "ms" << std::endl;
  // cleanup
  delete timearray;
  for (i = 0; i < (steps + 1); i++)
    delete dataarray[i];
  delete dataarray;
  for (i = 0; i < steps; i++)
    for (j = 0; j < numberofmotors; j++)
      delete parameters[i][j];
  for (i = 0; i < steps; i++)
    delete parameters[i];
  delete parameters;
  delete encoderarray;
  delete arr_p1;
  delete arr_p2;
  delete arr_p3;
  delete arr_p4;

  // wait for end of linear movement
  if (wait)
  {
    waitFor(MSF_NLINMOV, wait_timeout);
  }
}

double CLMBase::totalTime(double distance, double acc, double dec, double vmax)
{
  // minimum distance to reach vmax
  double borderdistance = pow(vmax, 2.0) / 2.0 * (1 / acc + 1 / dec);

  double time;
  if (distance > borderdistance)
  {
    time = distance / vmax + vmax / 2.0 * (1 / acc + 1 / dec);
  }
  else
  {
    time = sqrt(8 * distance / (acc + dec));
  }

  return time;
}

double CLMBase::relPosition(double reltime, double distance, double acc, double dec, double vmax)
{
  // minimum distance to reach vmax
  double borderdistance = pow(vmax, 2.0) / 2.0 * (1 / acc + 1 / dec);

  double position, totaltime, time;
  if (distance > borderdistance)
  {  // vmax reached during movement
    totaltime = distance / vmax + vmax / 2.0 * (1 / acc + 1 / dec);
    time = reltime;
    if (time < vmax / acc)
    {  // accelerating
      position = acc / 2 * pow(time, 2);
    }
    else if (time < totaltime - (vmax / dec))
    {  // at vmax
      position = vmax * (time - vmax / acc / 2);
    }
    else
    {  // decelerating
      position = distance - dec * (pow(time, 2) / 2 - totaltime * time + pow(totaltime, 2) / 2);
    }
  }
  else
  {  // vmax not reached during movement
    totaltime = sqrt(8 * distance / (acc + dec));
    time = reltime;
    if (time < totaltime * dec / (acc + dec))
    {  // accelerating
      position = acc / 2 * pow(time, 2);
    }
    else
    {  // decelerating
      position = distance - dec * (pow(time, 2) / 2 - totaltime * time + pow(totaltime, 2) / 2);
    }
  }

  return (position / distance);
}

void CLMBase::splineCoefficients(int steps, double* timearray, double* encoderarray, double* arr_p1, double* arr_p2,
                                 double* arr_p3, double* arr_p4)
{
  int i, j;  // countervariables

  // calculate time differences between points and b-coefficients
  double* deltatime = new double[steps];
  double* b = new double[steps];
  for (i = 0; i < steps; i++)
  {
    deltatime[i] = timearray[i + 1] - timearray[i];
    b[i] = 1.0 / deltatime[i];
  }

  // calculate a-coefficients
  double* a = new double[steps - 1];
  for (i = 0; i < (steps - 1); i++)
  {
    a[i] = (2 / deltatime[i]) + (2 / deltatime[i + 1]);
  }

  // build up the right hand side of the linear system
  double* c = new double[steps];
  double* d = new double[steps + 1];
  d[0] = 0;  // f_1' and f_n' equal zero
  d[steps] = 0;
  for (i = 0; i < steps; i++)
  {
    c[i] = (encoderarray[i + 1] - encoderarray[i]) / (deltatime[i] * deltatime[i]);
  }
  for (i = 0; i < (steps - 1); i++)
  {
    d[i + 1] = 3 * (c[i] + c[i + 1]);
  }

  // compose A * f' = d
  double** Alin = new double* [steps - 1];  // last column of Alin is right hand side
  for (i = 0; i < (steps - 1); i++)
    Alin[i] = new double[steps];
  // fill with zeros
  for (i = 0; i < (steps - 1); i++)
  {
    for (j = 0; j < steps; j++)
    {
      Alin[i][j] = 0.0;
    }
  }
  // insert values
  for (i = 0; i < (steps - 1); i++)
  {
    if (i == 0)
    {
      Alin[0][0] = a[0];
      Alin[0][1] = b[1];
      Alin[0][steps - 1] = d[1];
    }
    else
    {
      Alin[i][i - 1] = b[i];
      Alin[i][i] = a[i];
      Alin[i][i + 1] = b[i + 1];
      Alin[i][steps - 1] = d[i + 1];
    }
  }

  // solve linear equation
  boost::numeric::ublas::matrix< double > ublas_A(steps - 1, steps - 1);
  boost::numeric::ublas::matrix< double > ublas_b(steps - 1, 1);
  for (i = 0; i < (steps - 1); i++)
  {
    for (j = 0; j < (steps - 1); j++)
    {
      ublas_A(i, j) = Alin[i][j];
    }
    ublas_b(i, 0) = Alin[i][steps - 1];
  }
  boost::numeric::ublas::permutation_matrix< unsigned int > piv(steps - 1);
  lu_factorize(ublas_A, piv);
  lu_substitute(ublas_A, piv, ublas_b);

  // save result in derivatives array
  double* derivatives = new double[steps + 1];
  derivatives[0] = 0;
  for (i = 0; i < (steps - 1); i++)
  {
    derivatives[i + 1] = ublas_b(i, 0);
  }
  derivatives[steps] = 0;
  // build the hermite polynom with difference scheme
  // Q(t) = a0 + (b0 + (c0 + d0 * t) * (t - 1)) * t = a0 + (b0 - c0) * t +
  //   (c0 - d0) * t^2 + d0 * t^3 = p0 + p1 * t + p2 * t^2 + p3 * t^3
  double a0, b0, c0, d0;
  for (i = 0; i < steps; i++)
  {
    a0 = encoderarray[i];
    b0 = encoderarray[i + 1] - a0;
    c0 = b0 - deltatime[i] * derivatives[i];
    d0 = deltatime[i] * (derivatives[i + 1] + derivatives[i]) - 2 * b0;
    arr_p1[i] = a0;
    arr_p2[i] = b0 - c0;
    arr_p3[i] = c0 - d0;
    arr_p4[i] = d0;
  }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool CLMBase::checkJointSpeed(std::vector< int > lastsolution, std::vector< int > solution, double time)
{
  const int speedlimit = 180;  // encoder per 10ms
  bool speedok = true;
  int localtime = (int)(time * 100);  // in 10ms
  int speed;
  int i;
  // check speed for every motor
  for (i = 0; i < ((int)solution.size()); i++)
  {
    speed = abs(solution.at(i) - lastsolution.at(i)) / localtime;
    if (speed > speedlimit)
      speedok = false;
  }
  return speedok;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int CLMBase::getSpeed(int distance, int acceleration, int time)
{
  // shorten argument names
  int d = distance;
  int a = acceleration;
  int t = time;
  // distance needs to be positive
  if (d < 0)
  {
    if (VDEBUG)
      std::cout << "getSpeed(): distance not positive\n";
    return -1;
  }
  // acceleration needs to be positive
  if (a < 0)
  {
    if (VDEBUG)
      std::cout << "getSpeed(): acceleration not positive\n";
    return -1;
  }
  // time needs to be at least 3 (acceleration, speed and deceleration polynomial with length of at least 1)
  if (t < 3)
  {
    if (VDEBUG)
      std::cout << "getSpeed(): time smaller than 3\n";
    return -1;
  }
  // need to reach at least d with t/2 acceleration and t/2 deceleration
  if (a * t * t < d * 4)
  {
    if (VDEBUG)
      std::cout << "getSpeed(): need to reach at least d with t/2 acceleration and t/2 deceleration\n";
    return -1;
  }
  // calculate speed (derived from 't = d / speed + speed / a')
  int speed = static_cast< int >(ceil(a * t / 2.0 - sqrt(a * a * t * t / 4.0 - a * d)));
  if ((speed % a) != 0)
    speed += (a - speed % a);  // round up to multiple of a to reach in less than t

  // if(VDEBUG) std::cout << "getSpeed(): calculated speed: " << speed << "\n";
  return speed;
}
// Point to point movement using splines
void CLMBase::movP2P(double X1, double Y1, double Z1, double Ph1, double Th1, double Ps1, double X2, double Y2,
                     double Z2, double Ph2, double Th2, double Ps2, bool exactflag, double vmax, bool wait,
                     long timeout)
{
  // variable declaration
  int nOfMot = getNumberOfMotors();
  int amax = 2;
  int smax = abs(static_cast< int >(vmax));
  smax -= smax % amax;  // round down to multiple of amax
  if (smax == 0)
    smax = amax;
  std::vector< int > start_enc(nOfMot);
  std::vector< int > target_enc(nOfMot);
  std::vector< int > distance(nOfMot);
  std::vector< int > dir(nOfMot);
  std::vector< bool > tooShortDistance(nOfMot);
  bool reachmax;
  int maxtime;
  int maxdist = 0;
  int maxmot = 0;

  // wait for motors to be ready
  bool motors_ready = false;
  KNI::Timer t(timeout);
  t.Start();
  while (!motors_ready)
  {
    motors_ready = true;
    for (int idx = 0; idx < nOfMot - 1; idx++)
    {
      base->GetMOT()->arr[idx].recvPVP();
      motors_ready &= (base->GetMOT()->arr[idx].GetPVP()->msf != 152);
    }
    if (t.Elapsed())
      return;
  }

  // calculate start encoders
  IKCalculate(X1, Y1, Z1, Ph1, Th1, Ps1, start_enc.begin());

  // calculate target encoders
  IKCalculate(X2, Y2, Z2, Ph2, Th2, Ps2, target_enc.begin(), start_enc);

  // calculate distances and directions
  for (int i = 0; i < nOfMot; i++)
  {
    distance[i] = abs(target_enc[i] - start_enc[i]);
    dir[i] = target_enc[i] - start_enc[i] < 0 ? -1 : 1;
    if (distance[i] < MINIMAL_POLY_DISTANCE)
      tooShortDistance[i] = true;
    else
      tooShortDistance[i] = false;
  }

  // get maximum of distances (maxdist) and associated motor (maxmot)
  for (int i = 0; i < nOfMot; i++)
  {
    if (distance[i] > maxdist)
    {
      maxmot = i;
      maxdist = distance[i];
    }
  }

  // check if maxmot reaches given maximum speed
  reachmax = (distance[maxmot] >= (((smax / amax) + 1) * smax));

  // calculate time maxmot needs for movement (will be common maxtime)
  int maxpadding = 3;  // maximum number of padding polynomials (with length 1)
  if (reachmax)
  {
    maxtime = (maxdist / smax + 1) + (smax / amax) + maxpadding;
  }
  else
  {
    // s^2 + a*s - a*d = 0  ->  s = sqrt(a^2/4 + a*d) - a/2
    int smaxnew =
        static_cast< int >(sqrt(static_cast< double >(amax * amax) / 4.0 + static_cast< double >(amax * maxdist)) -
                           (static_cast< double >(amax) / 2.0));
    smaxnew -= smaxnew % amax;  // round down to multiple of amax
    if (smaxnew == 0)
      smaxnew = amax;
    maxtime = (maxdist / smaxnew + 1) + (smaxnew / amax) + maxpadding;
  }
  if (maxtime < 6)
    maxtime = 6;

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // spline calculation
  std::vector< int > speed(nOfMot);      // maximum speed for this motor
  std::vector< int > corrspeed(nOfMot);  // speed at correction polynomial
  std::vector< int > t1(nOfMot);         // time for first polynomial (acceleration)
  std::vector< int > t2(nOfMot);         // time for second polynomial (max speed)
  std::vector< int > t3(nOfMot);         // time for third polynomial (deceleration)
  std::vector< int > t4(nOfMot);         // time for second polynomial (correction or padding)
  std::vector< int > t5(nOfMot);         // time for second polynomial (rest of deceleration or padding)
  std::vector< int > t6(nOfMot);         // time for second polynomial (padding)
  std::vector< int > p1_enc(nOfMot);     // position between acceleration and max speed
  std::vector< int > p2_enc(nOfMot);     // position between max speed and deceleration
  std::vector< int > p3_enc(nOfMot);     // position between deceleration and (corr or padd)
  std::vector< int > p4_enc(nOfMot);     // position between (corr or padd) and (rod or padd)
  std::vector< int > p5_enc(nOfMot);     // position between (rod or padd) and padding
  std::vector< short > target(nOfMot);
  std::vector< short > time(nOfMot);
  std::vector< short > pp0(nOfMot);  // polynomial coefficients
  std::vector< short > pp1(nOfMot);
  std::vector< short > pp2(nOfMot);
  std::vector< short > pp3(nOfMot);
  for (int i = 0; i < nOfMot; i++)
  {
    speed[i] = getSpeed(distance[i], amax, maxtime - maxpadding);
    if (speed[i] < 0)
      return;
    if (speed[i] == 0)  // only when distance == 0, avoid division by zero
      speed[i] = amax;
    corrspeed[i] = distance[i] % speed[i];
    corrspeed[i] -= corrspeed[i] % amax;  // round down to multiple of amax
    t1[i] = speed[i] / amax;
    p1_enc[i] = start_enc[i] + dir[i] * t1[i] * speed[i] / 2;
    t2[i] = (distance[i] - speed[i] * speed[i] / amax) / speed[i];  // div speed, rest in correction polynomial
    p2_enc[i] = p1_enc[i] + dir[i] * t2[i] * speed[i];
    t3[i] = (speed[i] - corrspeed[i]) / amax;
    p3_enc[i] = p2_enc[i] + dir[i] * t3[i] * (speed[i] + corrspeed[i]) / 2;
    t4[i] = 1;
    p4_enc[i] = p3_enc[i] + dir[i] * corrspeed[i];
    t5[i] = corrspeed[i] / amax;
    if (t5[i] == 0)  // if padding polynomial (corrspeed == 0), lengh is 1
      t5[i] = 1;
    p5_enc[i] = p4_enc[i] + dir[i] * t5[i] * corrspeed[i] / 2;
    t6[i] = maxtime - t1[i] - t2[i] - t3[i] - t4[i] - t5[i];
    if (VDEBUG && (i == 0))
    {
      std::cout << "\nparams axis " << i + 1 << ":"
                << "\n distance: " << distance[i] << "\n dir: " << dir[i] << "\n speed: " << speed[i]
                << "\n correctionspeed: " << corrspeed[i] << "\n t1: " << t1[i] << "\n t2: " << t2[i]
                << "\n t3: " << t3[i] << "\n t4: " << t4[i] << "\n t5: " << t5[i] << "\n t6: " << t6[i]
                << "\n start_enc " << start_enc[i] << "\n p1_enc: " << p1_enc[i] << "\n p2_enc: " << p2_enc[i]
                << "\n p3_enc: " << p3_enc[i] << "\n p4_enc: " << p4_enc[i] << "\n p5_enc: " << p5_enc[i]
                << "\n target_enc " << target_enc[i] << std::endl;
    }
  }

  // Polynomial 1 (acceleration)
  for (int i = 0; i < nOfMot; i++)
  {
    if (!tooShortDistance[i])
    {
      target[i] = static_cast< short >(p1_enc[i]);
      time[i] = static_cast< short >(t1[i]);
      pp0[i] = static_cast< short >(start_enc[i]);
      pp1[i] = 0;
      pp2[i] = static_cast< short >(1024 * (0.5 * dir[i] * amax));
      pp3[i] = 0;
    }
    else
    {
      target[i] = static_cast< short >(start_enc[i] + dir[i] * distance[i] / 5);
      time[i] = static_cast< short >(maxtime / 6);
      pp0[i] = static_cast< short >(start_enc[i]);
      pp1[i] = 0;
      pp2[i] = 0;
      pp3[i] = 0;
    }
    if (VDEBUG && (i == 0))
    {
      std::cout << "pp axis " << i + 1 << ":"
                                          "\t target: " << target[i] << "\t time: " << time[i] << "\tpp0: " << pp0[i]
                << ", pp1: " << pp1[i] << ", pp2: " << pp2[i] << ", pp3: " << pp3[i] << std::endl;
    }
  }
  if (t.Elapsed())
    return;
  std::vector< short > polynomial;
  for (int i = 0; i < nOfMot; ++i)
  {
    polynomial.push_back(time[i]);
    polynomial.push_back(target[i]);
    polynomial.push_back(pp0[i]);
    polynomial.push_back(pp1[i]);
    polynomial.push_back(pp2[i]);
    polynomial.push_back(pp3[i]);
  }
  setAndStartPolyMovement(polynomial, exactflag, 2);

  // Polynomial 2 (speed)
  for (int i = 0; i < nOfMot; i++)
  {
    if (!tooShortDistance[i])
    {
      target[i] = static_cast< short >(p2_enc[i]);
      time[i] = static_cast< short >(t2[i]);
      pp0[i] = static_cast< short >(p1_enc[i]);
      pp1[i] = static_cast< short >(64 * (dir[i] * speed[i]));
      pp2[i] = 0;
      pp3[i] = 0;
    }
    else
    {
      target[i] = static_cast< short >(start_enc[i] + dir[i] * distance[i] * 2 / 5);
      time[i] = static_cast< short >(maxtime / 6);
      pp0[i] = static_cast< short >(start_enc[i] + dir[i] * distance[i] / 5);
      pp1[i] = 0;
      pp2[i] = 0;
      pp3[i] = 0;
    }
    if (VDEBUG && (i == 0))
    {
      std::cout << "pp axis " << i + 1 << ":"
                                          "\t target: " << target[i] << "\t time: " << time[i] << "\tpp0: " << pp0[i]
                << ", pp1: " << pp1[i] << ", pp2: " << pp2[i] << ", pp3: " << pp3[i] << std::endl;
    }
  }
  if (t.Elapsed())
    return;
  polynomial.clear();
  for (int i = 0; i < nOfMot; ++i)
  {
    polynomial.push_back(time[i]);
    polynomial.push_back(target[i]);
    polynomial.push_back(pp0[i]);
    polynomial.push_back(pp1[i]);
    polynomial.push_back(pp2[i]);
    polynomial.push_back(pp3[i]);
  }
  setAndStartPolyMovement(polynomial, exactflag, 2);

  // Polynomial 3 (deceleration)
  for (int i = 0; i < nOfMot; i++)
  {
    if (!tooShortDistance[i])
    {
      target[i] = static_cast< short >(p3_enc[i]);
      time[i] = static_cast< short >(t3[i]);
      pp0[i] = static_cast< short >(p2_enc[i]);
      pp1[i] = static_cast< short >(64 * (dir[i] * speed[i]));
      pp2[i] = static_cast< short >(1024 * (-0.5 * dir[i] * amax));
      pp3[i] = 0;
    }
    else
    {
      target[i] = static_cast< short >(start_enc[i] + dir[i] * distance[i] * 3 / 5);
      time[i] = static_cast< short >(maxtime / 6);
      pp0[i] = static_cast< short >(start_enc[i] + dir[i] * distance[i] * 2 / 5);
      pp1[i] = 0;
      pp2[i] = 0;
      pp3[i] = 0;
    }
    if (VDEBUG && (i == 0))
    {
      std::cout << "pp axis " << i + 1 << ":"
                                          "\t target: " << target[i] << "\t time: " << time[i] << "\tpp0: " << pp0[i]
                << ", pp1: " << pp1[i] << ", pp2: " << pp2[i] << ", pp3: " << pp3[i] << std::endl;
    }
  }
  if (t.Elapsed())
    return;
  polynomial.clear();
  for (int i = 0; i < nOfMot; ++i)
  {
    polynomial.push_back(time[i]);
    polynomial.push_back(target[i]);
    polynomial.push_back(pp0[i]);
    polynomial.push_back(pp1[i]);
    polynomial.push_back(pp2[i]);
    polynomial.push_back(pp3[i]);
  }
  setAndStartPolyMovement(polynomial, exactflag, 2);

  // Polynomial 4 (correction or padding if correction speed == 0)
  for (int i = 0; i < nOfMot; i++)
  {
    if (!tooShortDistance[i])
    {
      target[i] = static_cast< short >(p4_enc[i]);
      time[i] = static_cast< short >(t4[i]);
      pp0[i] = static_cast< short >(p3_enc[i]);
      pp1[i] = static_cast< short >(64 * (dir[i] * corrspeed[i]));
      pp2[i] = 0;
      pp3[i] = 0;
    }
    else
    {
      target[i] = static_cast< short >(start_enc[i] + dir[i] * distance[i] * 4 / 5);
      time[i] = static_cast< short >(maxtime / 6);
      pp0[i] = static_cast< short >(start_enc[i] + dir[i] * distance[i] * 3 / 5);
      pp1[i] = 0;
      pp2[i] = 0;
      pp3[i] = 0;
    }
    if (VDEBUG && (i == 0))
    {
      std::cout << "pp axis " << i + 1 << ":"
                                          "\t target: " << target[i] << "\t time: " << time[i] << "\tpp0: " << pp0[i]
                << ", pp1: " << pp1[i] << ", pp2: " << pp2[i] << ", pp3: " << pp3[i] << std::endl;
    }
  }
  if (t.Elapsed())
    return;
  polynomial.clear();
  for (int i = 0; i < nOfMot; ++i)
  {
    polynomial.push_back(time[i]);
    polynomial.push_back(target[i]);
    polynomial.push_back(pp0[i]);
    polynomial.push_back(pp1[i]);
    polynomial.push_back(pp2[i]);
    polynomial.push_back(pp3[i]);
  }
  setAndStartPolyMovement(polynomial, exactflag, 2);

  // Polynomial 5 (deceleration or padding if correction speed == 0)
  for (int i = 0; i < nOfMot; i++)
  {
    if (!tooShortDistance[i])
    {
      target[i] = static_cast< short >(p5_enc[i]);
      time[i] = static_cast< short >(t5[i]);
      pp0[i] = static_cast< short >(p4_enc[i]);
      pp1[i] = static_cast< short >(64 * (dir[i] * corrspeed[i]));
      if (corrspeed[i] != 0)
      {
        pp2[i] = static_cast< short >(1024 * (-0.5 * dir[i] * amax));
      }
      else
      {
        pp2[i] = 0;
      }
      pp3[i] = 0;
    }
    else
    {
      target[i] = static_cast< short >(target_enc[i]);
      time[i] = static_cast< short >(maxtime / 6);
      pp0[i] = static_cast< short >(start_enc[i] + dir[i] * distance[i] * 4 / 5);
      pp1[i] = 0;
      pp2[i] = 0;
      pp3[i] = 0;
    }
    if (VDEBUG && (i == 0))
    {
      std::cout << "pp axis " << i + 1 << ":"
                                          "\t target: " << target[i] << "\t time: " << time[i] << "\tpp0: " << pp0[i]
                << ", pp1: " << pp1[i] << ", pp2: " << pp2[i] << ", pp3: " << pp3[i] << std::endl;
    }
  }
  if (t.Elapsed())
    return;
  polynomial.clear();
  for (int i = 0; i < nOfMot; ++i)
  {
    polynomial.push_back(time[i]);
    polynomial.push_back(target[i]);
    polynomial.push_back(pp0[i]);
    polynomial.push_back(pp1[i]);
    polynomial.push_back(pp2[i]);
    polynomial.push_back(pp3[i]);
  }
  setAndStartPolyMovement(polynomial, exactflag, 2);

  // Polynomial 6 (padding)
  for (int i = 0; i < nOfMot; i++)
  {
    if (!tooShortDistance[i])
    {
      target[i] = static_cast< short >(target_enc[i]);
      time[i] = static_cast< short >(t6[i]);
      pp0[i] = static_cast< short >(p5_enc[i]);
      pp1[i] = 0;
      pp2[i] = 0;
      pp3[i] = 0;
    }
    else
    {
      target[i] = static_cast< short >(target_enc[i]);
      time[i] = static_cast< short >(maxtime - (maxtime / 6) * 5);
      pp0[i] = static_cast< short >(target_enc[i]);
      pp1[i] = 0;
      pp2[i] = 0;
      pp3[i] = 0;
    }
    if (VDEBUG && (i == 0))
    {
      std::cout << "pp axis " << i + 1 << ":"
                                          "\t target: " << target[i] << "\t time: " << time[i] << "\tpp0: " << pp0[i]
                << ", pp1: " << pp1[i] << ", pp2: " << pp2[i] << ", pp3: " << pp3[i] << std::endl;
    }
  }
  if (t.Elapsed())
    return;
  polynomial.clear();
  for (int i = 0; i < nOfMot; ++i)
  {
    polynomial.push_back(time[i]);
    polynomial.push_back(target[i]);
    polynomial.push_back(pp0[i]);
    polynomial.push_back(pp1[i]);
    polynomial.push_back(pp2[i]);
    polynomial.push_back(pp3[i]);
  }
  setAndStartPolyMovement(polynomial, exactflag, 1);  // no next & start movement

  // wait for end of linear movement
  if (wait)
  {
    waitFor(MSF_NLINMOV, timeout);
  }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void CLMBase::movLM(double X, double Y, double Z, double Al, double Be, double Ga, bool exactflag, double vmax,
                    bool wait, int tolerance, long timeout)
{
  double arr_tarpos[6] = {X, Y, Z, Al, Be, Ga};
  // target position in cartesian units
  double arr_actpos[6];
  // current position in cartesian units, NO REFRESH, SINCE ALREADY DONE IN moveRobotLinearTo()
  getCoordinates(arr_actpos[0], arr_actpos[1], arr_actpos[2], arr_actpos[3], arr_actpos[4], arr_actpos[5], false);

  movLM2P(arr_actpos[0], arr_actpos[1], arr_actpos[2], arr_actpos[3], arr_actpos[4], arr_actpos[5], arr_tarpos[0],
          arr_tarpos[1], arr_tarpos[2], arr_tarpos[3], arr_tarpos[4], arr_tarpos[5], exactflag, vmax, wait, tolerance,
          timeout);
}

void CLMBase::moveRobotLinearTo(double x, double y, double z, double phi, double theta, double psi,
                                bool waitUntilReached, int waitTimeout)
{
  // refresh encoders to make sure we use the right actual position
  base->recvMPS();

  movLM(x, y, z, phi, theta, psi, _activatePositionController, _maximumVelocity, waitUntilReached, 100, waitTimeout);
}

void CLMBase::moveRobotLinearTo(std::vector< double > coordinates, bool waitUntilReached, int waitTimeout)
{
  moveRobotLinearTo(coordinates.at(0), coordinates.at(1), coordinates.at(2), coordinates.at(3), coordinates.at(4),
                    coordinates.at(5), waitUntilReached, waitTimeout);
}

void CLMBase::moveRobotTo(double x, double y, double z, double phi, double theta, double psi, bool waitUntilReached,
                          int waitTimeout)
{
  // TODO: remove call of old implementation
  // 	std::cout << "moveRobotTo(): calling old implementation." << std::endl;
  // 	CikBase::moveRobotTo(x, y, z, phi, theta, psi, waitUntilReached, waitTimeout);
  // 	return;

  // current position in cartesian units (with encoder refresh)
  double cp[6];
  getCoordinates(cp[0], cp[1], cp[2], cp[3], cp[4], cp[5], true);

  movP2P(cp[0], cp[1], cp[2], cp[3], cp[4], cp[5], x, y, z, phi, theta, psi, _activatePositionController,
         _maximumVelocity, waitUntilReached, waitTimeout);
}

void CLMBase::moveRobotTo(std::vector< double > coordinates, bool waitUntilReached, int waitTimeout)
{
  moveRobotTo(coordinates.at(0), coordinates.at(1), coordinates.at(2), coordinates.at(3), coordinates.at(4),
              coordinates.at(5), waitUntilReached, waitTimeout);
}

// set maximum linear velocity in mm/s
void CLMBase::setMaximumLinearVelocity(double maximumVelocity)
{
  if (maximumVelocity < 1)
    maximumVelocity = 1;
  if (maximumVelocity > 300)
    maximumVelocity = 300;
  _maximumVelocity = maximumVelocity;
}
double CLMBase::getMaximumLinearVelocity() const
{
  return _maximumVelocity;
}

void CLMBase::setActivatePositionController(bool activate)
{
  _activatePositionController = activate;
}
bool CLMBase::getActivatePositionController()
{
  return _activatePositionController;
}
