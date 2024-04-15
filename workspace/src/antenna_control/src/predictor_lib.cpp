#include "antenna_control/predictor_lib.hpp"

using namespace Eigen;

/*
int wrong_get_next_target_change(int Ni, int actual_change) {
  double a =
      (m1 - m2) * (1 / 2 + (m1 - m2) / (2 * m2) - (1 + (m1 - m2) / m2));
  double c = -actual_change - m1 * Ni * (Ni - 1) / 2;
  double Nt = sqrt(-c / a);
  double bias = (m1 - m2) * Nt;
  int N0 = round(-bias / m2);
  double sum_till_zero =
      m2 / 2 * (N0 * (N0 + 1) - Nt * (Nt + 1) + bias * (N0 - Nt));
  double sum_till_top = m1 / 2 * (Nt * (Nt + 1) - Ni * (Ni - 1));
  return round(sum_till_top + sum_till_zero);
}
*/

std::pair<double, double> quadratic_equation(double a, double b, double c) {
  double D = sqrt(b * b - 4 * a * c);
  double Nt1 = (-b + D) / (2 * a);
  double Nt2 = (-b - D) / (2 * a);
  return {Nt1, Nt2};
}

// Ni is start time, Nf is the target time
// up and down before Nf
/*
int MotorPositionPredictor::case2_get_next_target_change(int Ni, int Nf,
                                                         int actual_change) {
  double a = -(m1 - m2) / 2;
  double b = (m1 - m2) * Nf;
  double c = -m1 * Ni * (Ni - 1) / 2 + m2 * Nf * (Nf + 1) / 2 - actual_change;
  std::pair<double, double> Nts = quadratic_equation(a, b, c);
  int Nt;
  if (Ni < Nts.first <= Nf)
    Nt = round(Nts.first);
  else
    Nt = round(Nts.second);
  std::cout << "Nt " << Nt << std::endl;
  return Nt;
}
*/

double MotorPositionPredictor::case2_get_next_target_change(double Ni,
                                                            int actual_change) {
  double Nt = sqrt((fabs(m1) * Ni * Ni + 2 * actual_change) /
                   (fabs(m1) * (1 + fabs(m1) / fabs(m2))));
  std::cout << "Nt " << Nt << std::endl;
  return Nt;
}

double MotorPositionPredictor::case5_get_next_target_change(double Ni,
                                                            double Ns,
                                                            int actual_change) {
  double Nt = (actual_change + saturation_vel * Ns -
               0.5 * fabs(m1) * (Ns * Ns - Ni * Ni) -
               0.5 * saturation_vel * saturation_vel / fabs(m2)) /
              saturation_vel;
  std::cout << "Nt " << Nt << std::endl;
  return Nt;
}

// Ns is saturate time, Nf is target time
// up, saturate and down before Nf
/*
int MotorPositionPredictor::case3_get_next_target_change(
    int Ns, int Nf, int actual_change, double sum_till_saturation) {
  double a = m2 / 2;
  double b = -m2 * (0.5 + Nf);
  double c = saturation_vel * (Nf - Ns) + m2 / 2 * Nf * (Nf + 1) +
             sum_till_saturation - actual_change;
  std::pair<double, double> Nts = quadratic_equation(a, b, c);
  int Nt;
  if (Ns < Nts.first <= Nf)
    Nt = round(Nts.first);
  else
    Nt = round(Nts.second);
  return Nt;
}
*/

// returns the target_step given how much to change and what's the time to
// cover that in. The major approximation taken is that velocities are
// discrete till 0.5 but here it's considered directly as slope * time
// in order to have a closed form solution
/**
 * @returns {actual_change_prediction, target_change}
 */
std::pair<double, double> MotorPositionPredictor::get_next_target_change(
    double init_velocity, int change_needed, int timesteps) {

  int actual_change = change_needed;
  if (actual_change < 0) {
    m1 = M2, m2 = M1;
    saturation_vel = -S - 0.5;
    actual_change = -actual_change;
  } else {
    m1 = M1, m2 = M2;
    saturation_vel = S + 0.5;
  }

  // consider only the case when init_velocity first moves along m1
  double Ni = init_velocity / m1;
  double Ns = saturation_vel / m1;
  double Nf = Ni + timesteps;
  std::cout << "timepoints: Ni " << Ni << " Ns " << Ns << " Nf " << Nf
            << std::endl;
  double sum_till_saturation = m1_slope_sum(Ni - 1, Ns);
  std::cout << "sum till saturation: " << sum_till_saturation << std::endl;

  double actual_change_prediction, target_change;

  if (Nf < Ns) {
    // reaching before saturation
    if (fabs(sum_till_saturation) < fabs(actual_change)) {
      std::cout << "case 1 impossible before saturation\n";
      // case of impossible needs
      double bias = (m1 - m2) * Ns;
      int N0 = -bias / m2;
      double sum_till_zero = m2_slope_sum(Ns, N0, bias);
      std::cout << "bias " << bias << " N0 " << N0 << std::endl;

      std::cout << "sum till zero: " << sum_till_zero << "\n";
      actual_change_prediction = sum_till_saturation;
      target_change = sum_till_saturation + sum_till_zero;
    } else {
      // case of up and down
      std::cout << "case 2 before saturation\n";
      double Nt = case2_get_next_target_change(Ni, actual_change);
      double bias = (m1 - m2) * Nt;
      double N0 = -bias / m2;
      std::cout << "bias " << bias << " N0 " << N0 << std::endl;

      double sum_till_Nt = m1_slope_sum(Ni - 1, Nt);
      double sum_till_zero = m2_slope_sum(Nt, N0, bias);
      std::cout << "sum till zero: " << sum_till_zero << "\n";
      actual_change_prediction = sum_till_Nt + m2_slope_sum(Nt, Nf, bias);
      target_change = sum_till_Nt + sum_till_zero;
    }
  } else {
    // reaching after saturation
    double sum_saturation_to_time = saturated_sum(Ns, Nf);
    std::cout << "sum after saturation and before final time: "
              << sum_saturation_to_time << std::endl;
    if (fabs(sum_till_saturation + sum_saturation_to_time) <
        fabs(actual_change)) {
      // case of impossible needs
      std::cout << "case 3 impossible after saturation\n";
      double bias = (saturation_vel - m2 * Nf);
      double N0 = -bias / m2;
      double sum_till_zero = m2_slope_sum(Nf, N0, bias);
      std::cout << "bias " << bias << " N0 " << N0 << std::endl;
      std::cout << "sum till zero: " << sum_till_zero << "\n";

      actual_change_prediction = sum_till_saturation + sum_saturation_to_time;
      target_change =
          sum_till_saturation + sum_saturation_to_time + sum_till_zero;
    } else {
      double bias = (m1 - m2) * Ns;
      double sum_till_threshold = m2_slope_sum(Ns, Nf, bias);
      if (fabs(sum_till_saturation + sum_till_threshold) >
          fabs(actual_change)) {
        // doesn't saturate
        std::cout << "case 4 Ns < Nf, velocity never saturates\n";
        double Nt = case2_get_next_target_change(Ni, actual_change);
        double bias = (m1 - m2) * Nt;
        double N0 = -bias / m2;
        std::cout << "bias " << bias << " N0 " << N0 << std::endl;

        double sum_till_Nt = m1_slope_sum(Ni - 1, Nt);
        double sum_till_zero = m2_slope_sum(Nt, N0, bias);
        std::cout << "sum till zero: " << sum_till_zero << "\n";
        actual_change_prediction = sum_till_Nt + m2_slope_sum(Nt, Nf, bias);
        target_change = sum_till_Nt + sum_till_zero;
      } else {
        // case of saturating then coming down
        std::cout
            << "case 5 Ns < Nf, velocity saturates for a bit then comes down\n";
        double Nt = case5_get_next_target_change(Ns, Ns, actual_change);
        double bias = saturation_vel - m2 * Nt;
        double N0 = -bias / m2;
        std::cout << "bias " << bias << " N0 " << N0 << std::endl;

        double sum_saturation_to_time = saturated_sum(Ns, Nt);
        double sum_till_zero = m2_slope_sum(Nt, N0, bias);
        std::cout << "sum till zero: " << sum_till_zero << "\n";
        actual_change_prediction = sum_till_saturation +
                                   sum_saturation_to_time +
                                   m2_slope_sum(Nt, Nf, bias);
        target_change =
            sum_till_saturation + sum_saturation_to_time + sum_till_zero;
      }
    }
  }
  return {actual_change_prediction, target_change};

  
}

std::pair<double, double> MotorPositionPredictor::translate_to_steps(double theta, double phi){
    return {A_theta*theta + B_theta, A_phi*phi + B_phi};
  }

MatrixXf generate_speed_transform(int n) {
  MatrixXf mat(n, n);
  mat.setIdentity();
  int shiftDownBy = 2;
  MatrixXf mat2(mat.rows(), mat.cols());
  mat2.bottomRows(mat.rows() - shiftDownBy) =
      mat.topRows(mat.rows() - shiftDownBy);
  mat2.topRows(shiftDownBy) = mat.bottomRows(shiftDownBy);
  MatrixXf transform = -0.5 * mat2 + 0.5 * mat;
  transform(0, 0) = 1;
  transform(1, 0) = -0.5;
  transform(0, mat.cols() - 2) = 0;
  transform(1, mat.cols() - 1) = 0;
  return transform;
}
