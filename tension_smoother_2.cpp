//
// Created by ljn on 20-5-4.
//
#include "tinyspline_ros/include/tinyspline_ros/tinysplinecpp.h"
#include "OsqpEigen/OsqpEigen.h"
#include "glog/logging.h"
#include "tension_smoother_2.hpp"
#include "tools/tools.hpp"
#include "reference_path.hpp"

TensionSmoother2::TensionSmoother2(const std::vector<State> &input_points,
                                   const State &start_state):
input_points_(input_points),
start_state_(start_state) {}

bool TensionSmoother2::osqpSmooth(const std::vector<double> &x_list,
                                  const std::vector<double> &y_list,
                                  const std::vector<double> &angle_list,
                                  const std::vector<double> &k_list,
                                  const std::vector<double> &s_list,
                                  std::vector<double> *result_x_list,
                                  std::vector<double> *result_y_list,
                                  std::vector<double> *result_s_list) {

    if(x_list.size() != y_list.size())
        std::cout<<"error at x_list size"<<std::endl;

    if(y_list.size() != angle_list.size())
        std::cout<<"error at x_list size"<<std::endl;

    if(angle_list.size() != s_list.size())
        std::cout<<"error at x_list size"<<std::endl;

    auto point_num = x_list.size();
    OsqpEigen::Solver solver;
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);
    solver.data()->setNumberOfVariables(4 * point_num - 1);
    solver.data()->setNumberOfConstraints(3 * (point_num - 1) + 2);
    // Allocate QP problem matrices and vectors.
    Eigen::SparseMatrix<double> hessian;
    Eigen::VectorXd gradient;
    Eigen::SparseMatrix<double> linearMatrix;
    Eigen::VectorXd lowerBound;
    Eigen::VectorXd upperBound;
    setHessianMatrix(point_num, &hessian);
    setGradient(x_list, y_list, &gradient);
    setConstraintMatrix(x_list, y_list, angle_list, k_list, s_list, &linearMatrix, &lowerBound, &upperBound);

    std::cout<<"second  smooth =========================================================="<<std::endl;

    // Input to solver.
    if (!solver.data()->setHessianMatrix(hessian)) return false;
    if (!solver.data()->setGradient(gradient)) return false;
    if (!solver.data()->setLinearConstraintsMatrix(linearMatrix)) return false;
    if (!solver.data()->setLowerBound(lowerBound)) return false;
    if (!solver.data()->setUpperBound(upperBound)) return false;
    // Solve.
    if (!solver.initSolver()) return false;
    if (!solver.solve()) return false;
    const auto &QPSolution{solver.getSolution()};
    // Output.
    result_s_list->clear();
    result_x_list->clear();
    result_y_list->clear();
    double tmp_s = 0;
    for (size_t i = 0; i != point_num; ++i) {
        double tmp_x = QPSolution(i);
        double tmp_y = QPSolution(point_num + i);
        result_x_list->emplace_back(tmp_x);
        result_y_list->emplace_back(tmp_y);
        if (i != 0)
            tmp_s += sqrt(pow(result_x_list->at(i) - result_x_list->at(i - 1), 2)
                              + pow(result_y_list->at(i) - result_y_list->at(i - 1), 2));
        result_s_list->emplace_back(tmp_s);
    }
    return true;
}

void TensionSmoother2::setHessianMatrix(size_t size, Eigen::SparseMatrix<double> *matrix_h) const {
    const size_t x_start_index = 0;
    const size_t y_start_index = x_start_index + size;
    const size_t theta_start_index = y_start_index + size;
    const size_t k_start_index = theta_start_index + size;
    const size_t matrix_size = 4 * size - 1;
    Eigen::MatrixXd hessian = Eigen::MatrixXd::Constant(matrix_size, matrix_size, 0);

    // Deviation and curvature.
    double FLAGS_tension_2_deviation_weight = 2.0;
    double FLAGS_tension_2_curvature_weight = 2.0;
    double FLAGS_tension_2_curvature_rate_weight = 2.0;

    for (int i = 0; i != size; ++i) {
        hessian(x_start_index + i, x_start_index + i) = hessian(y_start_index + i, y_start_index + i)
            = FLAGS_tension_2_deviation_weight * 2;
        if (i != size - 1) hessian(k_start_index + i, k_start_index + i) = FLAGS_tension_2_curvature_weight * 2;
    }

    // Curvature change.
    Eigen::Vector2d coeff_vec{1, -1};
    Eigen::Matrix2d coeff = coeff_vec * coeff_vec.transpose();
    for (int i = 0; i != size - 2; ++i)
    {
        hessian.block(k_start_index + i, k_start_index + i, 2, 2) += 2 * FLAGS_tension_2_curvature_rate_weight * coeff;
    }
    *matrix_h = hessian.sparseView();
}

void TensionSmoother2::setConstraintMatrix(const std::vector<double> &x_list,
                                           const std::vector<double> &y_list,
                                           const std::vector<double> &angle_list,
                                           const std::vector<double> &k_list,
                                           const std::vector<double> &s_list,
                                           Eigen::SparseMatrix<double> *matrix_constraints,
                                           Eigen::VectorXd *lower_bound,
                                           Eigen::VectorXd *upper_bound) const {
    const size_t size = x_list.size();

    const size_t x_start_index = 0;
    const size_t y_start_index = x_start_index + size;
    const size_t theta_start_index = y_start_index + size;
    const size_t k_start_index = theta_start_index + size;

    const size_t cons_x_update_start_index = 0;
    const size_t cons_y_update_start_index = cons_x_update_start_index + size - 1;
    const size_t cons_theta_update_start_index = cons_y_update_start_index + size - 1;
    const size_t cons_x_index = cons_theta_update_start_index + size - 1;
    const size_t cons_y_index = cons_x_index + 1;

    Eigen::MatrixXd cons = Eigen::MatrixXd::Zero(3 * (size - 1) + 2, 4 * size - 1);
    *lower_bound = Eigen::MatrixXd::Zero(3 * (size - 1) + 2, 1);
    *upper_bound = Eigen::MatrixXd::Zero(3 * (size - 1) + 2, 1);
    // Cons.
    for (int i = 0; i != size - 1; ++i) {
        const double ds = s_list[i + 1] - s_list[i];

        cons(cons_x_update_start_index + i, x_start_index + i + 1) = 
        cons(cons_y_update_start_index + i, y_start_index + i + 1)   
        = cons(cons_theta_update_start_index + i, theta_start_index + i + 1) = 1;

        cons(cons_x_update_start_index + i, x_start_index + i) = 
        cons(cons_y_update_start_index + i, y_start_index + i) = 
        cons(cons_theta_update_start_index + i, theta_start_index + i) = -1;
       
        cons(cons_x_update_start_index + i, theta_start_index + i) = ds * sin(angle_list[i]);
        cons(cons_y_update_start_index + i, theta_start_index + i) = -ds * cos(angle_list[i]);
        cons(cons_theta_update_start_index + i, k_start_index + i) = -ds;
    }

    cons(cons_x_index, x_start_index) = cons(cons_y_index, y_start_index) = 1;
    *matrix_constraints = cons.sparseView();
    // Bounds.
    for (int i = 0; i != size - 1; ++i) {
        const double ds = s_list[i + 1] - s_list[i];
        (*lower_bound)(cons_x_update_start_index + i) = (*upper_bound)(cons_x_update_start_index + i) = ds * cos(angle_list[i]);
        (*lower_bound)(cons_y_update_start_index + i) = (*upper_bound)(cons_y_update_start_index + i) = ds * sin(angle_list[i]);
        (*lower_bound)(cons_theta_update_start_index + i) = (*upper_bound)(cons_theta_update_start_index + i) =  -ds * k_list[i];
    }
    (*lower_bound)(cons_x_index) = (*upper_bound)(cons_x_index) = x_list[0];
    (*lower_bound)(cons_y_index) = (*upper_bound)(cons_y_index) = y_list[0];// 只能通过xy的boX进行障碍物约束
}

void TensionSmoother2::setGradient(const std::vector<double> &x_list,
                                   const std::vector<double> &y_list,
                                   Eigen::VectorXd *gradient) {
    const auto size = x_list.size();
    const size_t x_start_index = 0;
    const size_t y_start_index = x_start_index + size;
    *gradient = Eigen::VectorXd::Constant(4 * size - 1, 0);

    double FLAGS_tension_2_deviation_weight = 2.0;

    for (int i = 0; i != size; ++i) {
        (*gradient)(x_start_index + i) = -2 * FLAGS_tension_2_deviation_weight * x_list[i];
        (*gradient)(y_start_index + i) = -2 * FLAGS_tension_2_deviation_weight * y_list[i];
    }
}

bool TensionSmoother2::segmentRawReference(std::vector<double> *x_list,
                                                std::vector<double> *y_list,
                                                std::vector<double> *s_list,
                                                std::vector<double> *angle_list,
                                                std::vector<double> *k_list) const {

    if (s_list_.size() != x_list_.size() || s_list_.size() != y_list_.size()) {
        std::cout << "Raw path x y and s size not equal!";
        return false;
    }

    double max_s = s_list_.back();
    tk::spline x_spline, y_spline;
    x_spline.set_points(s_list_, x_list_);
    y_spline.set_points(s_list_, y_list_);
    // Divide the raw path.
    double delta_s = 1.0;
    s_list->emplace_back(0);
    while (s_list->back() < max_s) {
        s_list->emplace_back(s_list->back() + delta_s);
    }
    if (max_s - s_list->back() > 1) {
        s_list->emplace_back(max_s);
    }
    auto point_num = s_list->size();

    // Store reference states in vectors. They will be used later.
    for (size_t i = 0; i != point_num; ++i) {
        double length_on_ref_path = s_list->at(i);
        double dx = x_spline.deriv(1, length_on_ref_path);
        double dy = y_spline.deriv(1, length_on_ref_path);
        double ddx = x_spline.deriv(2, length_on_ref_path);
        double ddy = y_spline.deriv(2, length_on_ref_path);
        double angle = atan2(dy, dx);
        angle_list->emplace_back(angle);
        double curvature = (dx * ddy - dy * ddx) / pow(dx * dx + dy * dy, 1.5);
        k_list->emplace_back(curvature);
        x_list->emplace_back(x_spline(length_on_ref_path));
        y_list->emplace_back(y_spline(length_on_ref_path));
    }
    return true;
}

bool TensionSmoother2::smooth(std::shared_ptr<ReferencePath> reference_path) {
    std::vector<double> x_list, y_list, s_list, angle_list, k_list;

    bSpline();

    //lihan: calculate the kappa based on the reference line point
    if (!segmentRawReference(&x_list, &y_list, &s_list, &angle_list, &k_list)) return false;
    std::vector<double> result_x_list, result_y_list, result_s_list;
    bool solver_ok = osqpSmooth(x_list,
                               y_list,
                               angle_list,
                               k_list,
                               s_list,
                               &result_x_list,
                               &result_y_list,
                               &result_s_list);
    if (!solver_ok) {
        std::cout << "Tension smoother failed!";
        return false;
    }

    tk::spline x_spline, y_spline;
    x_spline.set_points(result_s_list, result_x_list);
    y_spline.set_points(result_s_list, result_y_list);

    double max_s_result = result_s_list.back() + 3;
    reference_path->setSpline(x_spline, y_spline, max_s_result);

    x_list_ = std::move(result_x_list);
    y_list_ = std::move(result_y_list);
    s_list_ = std::move(result_s_list);


    for(int i =0;i<x_list_.size();++i)
    {
        std::cout<<"x: "<<x_list_[i]<<std::endl;
    }

    for(int i =0;i<y_list_.size();++i)
    {
        std::cout<<"y: "<<y_list_[i]<<std::endl;
    }

    return true;
}



void TensionSmoother2::bSpline() {
  // B spline smoothing.

  double length = 0;
  for (size_t i = 0; i != input_points_.size() - 1; ++i) {
    length += caldistance(input_points_[i], input_points_[i + 1]);
  }
  int degree = 3;
  double average_length = length / (input_points_.size() - 1);
  if (average_length > 10)
    degree = 3;
  else if (average_length > 5)
    degree = 4;
  else
    degree = 5;
  tinyspline::BSpline b_spline_raw(input_points_.size(), 2, degree);
  std::vector<tinyspline::real> ctrlp_raw = b_spline_raw.controlPoints();
  for (size_t i = 0; i != input_points_.size(); ++i) {
    ctrlp_raw[2 * (i)] = input_points_[i].x;
    ctrlp_raw[2 * (i) + 1] = input_points_[i].y;
  }
  b_spline_raw.setControlPoints(ctrlp_raw);
  double delta_t = 1.0 / length;
  double tmp_t = 0;
  while (tmp_t < 1) {
    auto result = b_spline_raw.eval(tmp_t).result();
    x_list_.emplace_back(result[0]);
    y_list_.emplace_back(result[1]);
    tmp_t += delta_t;
  }
  auto result = b_spline_raw.eval(1).result();
  x_list_.emplace_back(result[0]);
  y_list_.emplace_back(result[1]);
  s_list_.emplace_back(0);
  for (size_t i = 1; i != x_list_.size(); ++i) {
    double dis = sqrt(pow(x_list_[i] - x_list_[i - 1], 2) +
                      pow(y_list_[i] - y_list_[i - 1], 2));
    s_list_.emplace_back(s_list_.back() + dis);
  }
}
