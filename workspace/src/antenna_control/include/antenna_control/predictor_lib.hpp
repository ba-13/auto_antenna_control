#include <Eigen/Dense>
#include <iostream>
#include <utility>
#include <vector>

class MotorPositionPredictor
{
private:
  int TAU = 20;
  double M1 = 0.015913663565769724 * TAU;
  double M2 = -0.017212386416242967 * TAU;
  double m1, m2;
  double S = 8.0;
  double A = 19.2 * 200;
  double B = 0.0;

  double saturation_vel;

  double case2_get_next_target_change(double Ni, int actual_change);

  double case5_get_next_target_change(double Ni, double Ns, int actual_change);

  // slope sum excluding value at Ni, and including at Nf
  inline double m2_slope_sum(int Ni, int Nf, double bias)
  {
    return this->m2 * (Nf * (Nf + 1) - Ni * (Ni + 1)) / 2 + bias * (Nf - Ni);
  }

  // slope sum excluding value at Ni, and including at Nf
  inline double m1_slope_sum(int Ni, int Nf)
  {
    return this->m1 * (Nf * (Nf + 1) - Ni * (Ni + 1)) / 2;
  }

  // sum excluding value at Ni, and including at Nf
  inline double saturated_sum(int Ni, int Nf)
  {
    return this->saturation_vel * (Nf - Ni);
  }

public:
  MotorPositionPredictor() {}
  MotorPositionPredictor(double A, double B) : A(A), B(B) {}
  MotorPositionPredictor(double m1, double m2, double A, double B) : M1(m1), M2(m2), A(A), B(B)
  {
    M1 *= TAU;
    M2 *= TAU;
  }
  MotorPositionPredictor(int tau, double m1, double m2, int saturation, double A, double B) : TAU(tau), M1(m1), M2(m2), S(saturation), A(A), B(B)
  {
    M1 *= TAU;
    M2 *= TAU;
  }

  std::pair<double, double> get_next_target_change(double init_velocity, int actual_change, int timesteps);

  double theta_to_steps(double theta);
  double steps_to_theta(int steps);
};

Eigen::MatrixXf generate_speed_transform(int n);
std::pair<double, double> quadratic_equation(double a, double b, double c);
