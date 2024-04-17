#include "antenna_control/predictor_lib.hpp"

using namespace Eigen;

std::pair<double, double> quadratic_equation(double a, double b, double c)
{
  double D = sqrt(b * b - 4 * a * c);
  double Nt1 = (-b + D) / (2 * a);
  double Nt2 = (-b - D) / (2 * a);
  return {Nt1, Nt2};
}

double MotorPositionPredictor::case2_get_next_target_change(double Ni, int actual_change)
{
  double Nt = sqrt((fabs(this->m1) * Ni * Ni + 2 * actual_change) /
                   (fabs(this->m1) * (1 + fabs(this->m1) / fabs(this->m2))));
  return Nt;
}

double MotorPositionPredictor::case5_get_next_target_change(double Ni, double Ns, int actual_change)
{
  double Nt = (actual_change + this->saturation_vel * Ns -
               0.5 * fabs(this->m1) * (Ns * Ns - Ni * Ni) -
               0.5 * this->saturation_vel * this->saturation_vel / fabs(this->m2)) /
              this->saturation_vel;
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
    double init_velocity, int change_needed, int timesteps)
{
  int actual_change = change_needed;
  if (actual_change < 0)
  {
    this->m1 = this->M2, this->m2 = this->M1;
    this->saturation_vel = -S - 0.5;
    actual_change = -actual_change;
  }
  else
  {
    this->m1 = this->M1, this->m2 = this->M2;
    this->saturation_vel = S + 0.5;
  }

  // consider only the case when init_velocity first moves along this->m1
  double Ni = init_velocity / this->m1;
  double Ns = this->saturation_vel / this->m1;
  double Nf = Ni + timesteps;
  double sum_till_saturation = MotorPositionPredictor::m1_slope_sum(Ni - 1, Ns);
  double bias, N0;

  double actual_change_prediction, target_change;

  if (Nf < Ns)
  {
    // reaching before saturation
    if (fabs(sum_till_saturation) < fabs(actual_change))
    {
      // std::cout << "case 1 impossible before saturation\n";
      // case of impossible needs
      bias = (this->m1 - this->m2) * Ns;
      N0 = -bias / this->m2;
      double sum_till_zero = MotorPositionPredictor::m2_slope_sum(Ns, N0, bias);

      actual_change_prediction = sum_till_saturation;
      target_change = sum_till_saturation + sum_till_zero;
    }
    else
    {
      // case of up and down
      // std::cout << "case 2 before saturation\n";
      double Nt = this->case2_get_next_target_change(Ni, actual_change);
      bias = (this->m1 - this->m2) * Nt;
      N0 = -bias / this->m2;

      double sum_till_Nt = MotorPositionPredictor::m1_slope_sum(Ni - 1, Nt);
      double sum_till_zero = MotorPositionPredictor::m2_slope_sum(Nt, N0, bias);
      actual_change_prediction = sum_till_Nt + MotorPositionPredictor::m2_slope_sum(Nt, Nf, bias);
      target_change = sum_till_Nt + sum_till_zero;
    }
  }
  else
  {
    // reaching after saturation
    double sum_saturation_to_time = saturated_sum(Ns, Nf);
    if (fabs(sum_till_saturation + sum_saturation_to_time) <
        fabs(actual_change))
    {
      // case of impossible needs
      // std::cout << "case 3 impossible after saturation\n";
      bias = (this->saturation_vel - this->m2 * Nf);
      N0 = -bias / this->m2;
      double sum_till_zero = MotorPositionPredictor::m2_slope_sum(Nf, N0, bias);

      actual_change_prediction = sum_till_saturation + sum_saturation_to_time;
      target_change =
          sum_till_saturation + sum_saturation_to_time + sum_till_zero;
    }
    else
    {
      bias = (this->m1 - this->m2) * Ns;
      double sum_till_threshold = MotorPositionPredictor::m2_slope_sum(Ns, Nf, bias);
      if (fabs(sum_till_saturation + sum_till_threshold) > fabs(actual_change))
      {
        // doesn't saturate
        // std::cout << "case 4 Ns < Nf, velocity never saturates\n";
        double Nt = this->case2_get_next_target_change(Ni, actual_change);
        bias = (this->m1 - this->m2) * Nt;
        N0 = -bias / this->m2;

        double sum_till_Nt = MotorPositionPredictor::m1_slope_sum(Ni - 1, Nt);
        double sum_till_zero = MotorPositionPredictor::m2_slope_sum(Nt, N0, bias);
        actual_change_prediction = sum_till_Nt + MotorPositionPredictor::m2_slope_sum(Nt, Nf, bias);
        target_change = sum_till_Nt + sum_till_zero;
      }
      else
      {
        // case of saturating then coming down
        // std::cout << "case 5 Ns < Nf, velocity saturates for a bit then comes down\n";
        double Nt = this->case5_get_next_target_change(Ns, Ns, actual_change);
        bias = this->saturation_vel - this->m2 * Nt;
        N0 = -bias / this->m2;

        double sum_saturation_to_time = saturated_sum(Ns, Nt);
        double sum_till_zero = MotorPositionPredictor::m2_slope_sum(Nt, N0, bias);
        actual_change_prediction = sum_till_saturation +
                                   sum_saturation_to_time +
                                   MotorPositionPredictor::m2_slope_sum(Nt, Nf, bias);
        target_change = sum_till_saturation + sum_saturation_to_time + sum_till_zero;
      }
    }
  }
  // std::cout << "bias " << bias << " N0 " << N0 << std::endl;
  return {actual_change_prediction, target_change};
}

double MotorPositionPredictor::translate_to_steps(double theta)
{
  return this->A * theta + this->B;
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
