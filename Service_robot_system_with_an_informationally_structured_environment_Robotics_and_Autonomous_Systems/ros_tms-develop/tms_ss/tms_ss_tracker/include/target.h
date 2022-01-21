#ifndef TARGET_H
#define TARGET_H

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

#endif  // TARGET_H
