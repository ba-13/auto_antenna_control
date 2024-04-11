#include <Eigen/Dense>
#include <iostream>
#include <utility>
#include <vector>

class MotorPositionPredictor {
private:
  int OBSERVATION_RATE = 50;
  int TAU = 20;
  double M1 = 0.015913663565769724;
  double M2 = -0.017212386416242967;
  double m1, m2;
  double S = 8.0;
  double saturation_vel;
  int case2_get_next_target_change(int Ni, int Nf, int actual_change);

  int case3_get_next_target_change(int Ns, int Nf, int actual_change,
                                   double sum_till_saturation);

  int multiple_tau(double time) { return round(time / TAU) * TAU; }

  // slope sum excluding value at Ni, and including at Nf
  inline double m2_slope_sum(int Ni, int Nf, double bias) {
    return m2 * TAU * (Nf * (Nf + 1) - Ni * (Ni + 1)) / 2 + bias * (Nf - Ni);
  }

  // slope sum excluding value at Ni, and including at Nf
  inline double m1_slope_sum(int Ni, int Nf) {
    return m1 * TAU * (Nf * (Nf + 1) - Ni * (Ni + 1)) / 2;
  }

  // sum excluding value at Ni, and including at Nf
  inline double saturated_sum(int Ni, int Nf) {
    return saturation_vel * (Nf - Ni);
  }

public:
  MotorPositionPredictor(){};
  MotorPositionPredictor(int rate, double m1, double m2, int saturation)
      : OBSERVATION_RATE(rate), M1(m1), M2(m2), S(saturation) {
    TAU = 1000 / rate;
  }

  std::pair<double, double> get_next_target_change(double init_velocity,
                                                   int actual_change,
                                                   int timesteps);
};

Eigen::MatrixXf generate_speed_transform(int n);
std::pair<double, double> quadratic_equation(double a, double b, double c);