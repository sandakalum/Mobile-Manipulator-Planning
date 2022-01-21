#pragma once

class CTarget
{
public:
  CTarget();
  virtual ~CTarget();

public:
  int id;
  double px;
  double py;
  int cnt;

public:
  void SetPosi(double x, double y);
};
