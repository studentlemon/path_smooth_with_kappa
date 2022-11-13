#include <QCoreApplication>
#include <iostream>
#include "tension_smoother_2.hpp"
#include "reference_path.hpp"

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    double delta_x = 0.5;
    double road_width = 10;
   const size_t total_boundary_size = 20;
   std::vector<State> left_boundary(total_boundary_size);
   std::vector<State> right_boundary(total_boundary_size);
   std::vector<State> original_path(total_boundary_size*2);
   std::vector<double> plot_x, plot_left_y, plot_right_y, plot_center_y;

   for (size_t i = 0; i < total_boundary_size*2; ++i) {
     const double x = delta_x * i;
     const double left_y = road_width;
     const double right_y = 0.0;

//      left_boundary[i] = {x, left_y};
//      right_boundary[i] = {x, right_y};

     if(i < total_boundary_size)
     {
         original_path[i] = {x,x};
     }else
     {
         original_path[i] = {x,10};
     }

     plot_x.emplace_back(x);
     plot_left_y.emplace_back(left_y);
     plot_right_y.emplace_back(right_y);
     plot_center_y.emplace_back(road_width / 2.0);
   }

   std::vector<State> input_points;
   State start_state = original_path.front();
   State end_state = original_path.back();

   TensionSmoother2 path_optimzier(original_path, start_state);
   std::shared_ptr<ReferencePath> reference_path_;
   reference_path_ = std::make_shared<ReferencePath>();
   if(!path_optimzier.smooth(reference_path_))
   {
       std::cout<<"path optimization error"<<std::endl;
   }




    return a.exec();
}
