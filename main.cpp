#include <QCoreApplication>
#include <iostream>
#include "reference_path.hpp"
#include "base_solver.hpp"
#include "data_struct.hpp"
#include "vehicle_state_frenet.hpp"
#include "tools/tools.hpp"

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    //============= data input =================
    double delta_x = 0.5;
    const size_t total_boundary_size = 20;
    std::shared_ptr<ReferencePath> original_path;
    std::shared_ptr<VehicleState> vec_state;
    std::vector<State> ref_path(total_boundary_size*2);
    original_path = std::make_shared<ReferencePath>();

    for (size_t i = 0; i < total_boundary_size*2; ++i) {
      const double x = delta_x * i;
      if(i < total_boundary_size)
      {
          ref_path[i] = {x,x};
      }else
      {
          ref_path[i] = {x,10};
      }
    }

      // B spline smoothing.
    std::vector<double> x_list_, y_list_, s_list_;

      double length = 0;
      for (size_t i = 0; i != ref_path.size() - 1; ++i) {
        length += caldistance(ref_path[i], ref_path[i + 1]);
      }
      int degree = 3;
      double average_length = length / (ref_path.size() - 1);
      if (average_length > 10)
        degree = 3;
      else if (average_length > 5)
        degree = 4;
      else
        degree = 5;
      tinyspline::BSpline b_spline_raw(ref_path.size(), 2, degree);
      std::vector<tinyspline::real> ctrlp_raw = b_spline_raw.controlPoints();
      for (size_t i = 0; i != ref_path.size(); ++i) {
        ctrlp_raw[2 * (i)] = ref_path[i].x;
        ctrlp_raw[2 * (i) + 1] = ref_path[i].y;
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

      if (s_list_.size() != x_list_.size() || s_list_.size() != y_list_.size()) {
          std::cout << "Raw path x y and s size not equal!";
          return false;
      }

      std::vector<double> x_list, y_list, s_list, angle_list, k_list;

      double max_s = s_list_.back();
      tk::spline x_spline, y_spline;
      x_spline.set_points(s_list_, x_list_);
      y_spline.set_points(s_list_, y_list_);
      // Divide the raw path.
      double delta_s = 0.5;
      s_list.emplace_back(0);
      while (s_list.back() < max_s) {
          s_list.emplace_back(s_list.back() + delta_s);
      }
      if (max_s - s_list.back() > 1) {
          s_list.emplace_back(max_s);
      }
      auto point_num = s_list.size();

      // Store reference states in vectors. They will be used later.
      for (size_t i = 0; i != point_num; ++i) {
          double length_on_ref_path = s_list.at(i);
          double dx = x_spline.deriv(1, length_on_ref_path);
          double dy = y_spline.deriv(1, length_on_ref_path);
          double ddx = x_spline.deriv(2, length_on_ref_path);
          double ddy = y_spline.deriv(2, length_on_ref_path);
          double angle = atan2(dy, dx);
          angle_list.emplace_back(angle);
          double curvature = (dx * ddy - dy * ddx) / pow(dx * dx + dy * dy, 1.5);
          k_list.emplace_back(curvature);
          x_list.emplace_back(x_spline(length_on_ref_path));
          y_list.emplace_back(y_spline(length_on_ref_path));
      }

       ref_path.resize(s_list.size());
      for(int i=0; i < s_list.size();++i)
      {
          ref_path[i]={x_list[i],y_list[i],angle_list[i],k_list[i],s_list[i]};
      }


    State start = ref_path.front();
    State end = ref_path.back();
    vec_state = std::make_shared<VehicleState>(start, end, 0.0, 0.0);

//    vec_state = std::make_shared<VehicleState>(start, end);
    original_path->buildReferenceFromStates(ref_path);
    original_path->updateBounds();
    int iter_num = 200;
    bool hard_constrain = false;
    std::vector<State> result_path;
    BaseSolver base_solver(original_path, vec_state, iter_num, hard_constrain);
    if (!base_solver.solve(&result_path))
    {
        std::cout<<"solve success "<<std::endl;
    }

    for(int i =0;i<result_path.size();++i)
    {
        std::cout<<"x: "<<result_path[i].x<<std::endl;
    }

    for(int i =0;i<result_path.size();++i)
    {
        std::cout<<"y: "<<result_path[i].y<<std::endl;
    }


    return a.exec();
}

