#include "antenna_control/predictor_lib.hpp"

using namespace Eigen;

/*
int wrong_get_next_target_change(int Ni, int actual_change) {
  double a =
      (m1 - m2) * TAU * (1 / 2 + (m1 - m2) / (2 * m2) - (1 + (m1 - m2) / m2));
  double c = -actual_change - m1 * TAU * Ni * (Ni - 1) / 2;
  double Nt = sqrt(-c / a);
  double bias = (m1 - m2) * TAU * Nt;
  int N0 = multiple_tau(-bias / m2);
  double sum_till_zero =
      m2 * TAU / 2 * (N0 * (N0 + 1) - Nt * (Nt + 1) + bias * (N0 - Nt));
  double sum_till_top = m1 * TAU / 2 * (Nt * (Nt + 1) - Ni * (Ni - 1));
  return round(sum_till_top + sum_till_zero);
}
*/

std::pair<double, double> quadratic_equation(double a, double b, double c)
{
  double D = sqrt(b * b - 4 * a * c);
  double Nt1 = (-b + D) / (2 * a);
  double Nt2 = (-b - D) / (2 * a);
  return {Nt1, Nt2};
}

// Ni is start time, Nf is the target time
// up and down before Nf
int MotorPositionPredictor::case2_get_next_target_change(int Ni, int Nf,
                                                         int actual_change)
{
  double a = -(m1 - m2) * TAU / 2;
  double b = (m1 - m2) * TAU * Nf;
  double c = -m1 * TAU * Ni * (Ni - 1) / 2 + m2 * TAU * Nf * (Nf + 1) / 2 -
             actual_change;
  std::pair<double, double> Nts = quadratic_equation(a, b, c);
  int Nt;
  if (Ni < Nts.first <= Nf)
    Nt = round(Nts.first);
  else
    Nt = round(Nts.second);
  return Nt;
}

// Ns is saturate time, Nf is target time
// up, saturate and down before Nf
int MotorPositionPredictor::case3_get_next_target_change(
    int Ns, int Nf, int actual_change, double sum_till_saturation)
{
  double a = m2 * TAU / 2;
  double b = -m2 * TAU * (0.5 + Nf);
  double c = saturation_vel * (Nf - Ns) + m2 * TAU / 2 * Nf * (Nf + 1) +
             sum_till_saturation - actual_change;
  std::pair<double, double> Nts = quadratic_equation(a, b, c);
  int Nt;
  if (Ns < Nts.first <= Nf)
    Nt = round(Nts.first);
  else
    Nt = round(Nts.second);
  return Nt;
}

// returns the target_step given how much to change and what's the time to
// cover that in. The major approximation taken is that velocities are
// discrete till 0.5 but here it's considered directly as slope * time
// in order to have a closed form solution
/**
 * @returns {actual_change_prediction, target_change}
 */
std::pair<double, double> MotorPositionPredictor::get_next_target_change(
    double init_velocity, int change, int timesteps)
{

  int actual_change = change;
  if (actual_change < 0)
  {
    m1 = M2, m2 = M1;
    saturation_vel = -S - 0.5;
    actual_change = -actual_change;
  }
  else
  {
    m1 = M1, m2 = M2;
    saturation_vel = S + 0.5;
  }

  // consider only the case when init_velocity first moves along m1
  int Ni = multiple_tau(init_velocity / m1) / TAU;
  int Ns = multiple_tau(saturation_vel / m1) / TAU;
  int Nf = Ni + timesteps;
  double sum_till_saturation = m1_slope_sum(Ni - 1, Ns);
  double actual_change_prediction, target_change;
  if (Nf < Ns)
  {
    // reaching before saturation
    if (sum_till_saturation >= actual_change)
    {
      // case of impossible needs
      double bias = (m1 - m2) * Ns * TAU;
      int N0 = multiple_tau(-bias / m2);
      double sum_till_zero = m2_slope_sum(Ns, N0, bias);
      actual_change_prediction = sum_till_saturation;
      target_change = sum_till_saturation + sum_till_zero;
    }
    else
    {
      // case of up and down
      int Nt = case2_get_next_target_change(Ni, Nf, actual_change);
      double bias = (m1 - m2) * TAU * Nt;
      int N0 = multiple_tau(-bias / m2);
      double sum_till_Nt = m1_slope_sum(Ni - 1, Nt);
      double sum_till_zero = m2_slope_sum(Nt, N0, bias);
      actual_change_prediction = sum_till_Nt + m2_slope_sum(Nt, Nf, bias);
      target_change = sum_till_Nt + sum_till_zero;
    }
  }
  else
  {
    // reaching after saturation
    double sum_saturation_to_time = saturated_sum(Ns, Nf);
    if (sum_till_saturation + sum_saturation_to_time >= actual_change)
    {
      // case of impossible needs
      double bias = (saturation_vel - m2 * TAU * Nf);
      int N0 = multiple_tau(-bias / m2);
      double sum_till_zero = m2_slope_sum(Nf, N0, bias);
      actual_change_prediction = sum_till_saturation + sum_saturation_to_time;
      target_change = sum_till_saturation + sum_saturation_to_time +
                      sum_till_zero;
    }
    else
    {
      double bias = (m1 - m2) * Ns * TAU;
      double sum_till_threshold = m2_slope_sum(Ns, Nf, bias);
      if (sum_till_saturation + sum_till_threshold <= actual_change)
      {
        // doesn't saturate
        int Nt = case2_get_next_target_change(Ni, Nf, actual_change);
        double bias = (m1 - m2) * TAU * Nt;
        int N0 = multiple_tau(-bias / m2);
        double sum_till_Nt = m1_slope_sum(Ni - 1, Nt);
        double sum_till_zero = m2_slope_sum(Nt, N0, bias);
        actual_change_prediction = sum_till_Nt + m2_slope_sum(Nt, Nf, bias);
        target_change = sum_till_Nt + sum_till_zero;
      }
      else
      {
        // case of saturating then coming down
        int Nt = case3_get_next_target_change(Ns, Nf, actual_change,
                                              sum_till_saturation);
        double bias = saturation_vel - m2 * Nt * TAU;
        int N0 = multiple_tau(-bias / m2);
        double sum_saturation_to_time = saturated_sum(Ns, Nt);
        double sum_till_zero = m2_slope_sum(Nt, N0, bias);
        actual_change_prediction = sum_till_saturation +sum_saturation_to_time + m2_slope_sum(Nt, Nf, bias);

        target_change = sum_till_saturation + sum_saturation_to_time + sum_till_zero;
      }
    }
  }
  if(change < 0)
  {
    actual_change_prediction = -actual_change_prediction;
    target_change = -target_change;
  }
  return {actual_change_prediction, target_change};
}

MatrixXf generate_speed_transform(int n)
{
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
