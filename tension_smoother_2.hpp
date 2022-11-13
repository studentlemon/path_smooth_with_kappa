#ifndef PATH_OPTIMIZER_INCLUDE_PATH_OPTIMIZER_REFERENCE_PATH_SMOOTHER_TENSION_SMOOTHER_2_HPP_
#define PATH_OPTIMIZER_INCLUDE_PATH_OPTIMIZER_REFERENCE_PATH_SMOOTHER_TENSION_SMOOTHER_2_HPP_
#include <vector>
#include <cfloat>
#include "Eigen/Dense"
#include "Eigen/Sparse"
#include "data_struct.hpp"
#include "reference_path.hpp"

class TensionSmoother2
{
 public:
    TensionSmoother2(const std::vector<State> &input_points,
               const State &start_state);

    std::vector<double> x_list_, y_list_, s_list_;
    const std::vector<State> &input_points_;
    const State &start_state_;
    bool smooth(std::shared_ptr<ReferencePath> reference_path);
    bool segmentRawReference(std::vector<double> *x_list,
                                                    std::vector<double> *y_list,
                                                    std::vector<double> *s_list,
                                                    std::vector<double> *angle_list,
                                                    std::vector<double> *k_list) const;
 private:
    void bSpline();
    bool osqpSmooth(const std::vector<double> &x_list,
                    const std::vector<double> &y_list,
                    const std::vector<double> &angle_list,
                    const std::vector<double> &k_list,
                    const std::vector<double> &s_list,
                    std::vector<double> *result_x_list,
                    std::vector<double> *result_y_list,
                    std::vector<double> *result_s_list);
    void setHessianMatrix(size_t size, Eigen::SparseMatrix<double> *matrix_h) const;
    void setConstraintMatrix(const std::vector<double> &x_list,
                             const std::vector<double> &y_list,
                             const std::vector<double> &angle_list,
                             const std::vector<double> &k_list,
                             const std::vector<double> &s_list,
                             Eigen::SparseMatrix<double> *matrix_constraints,
                             Eigen::VectorXd *lower_bound,
                             Eigen::VectorXd *upper_bound) const;
    void setGradient(const std::vector<double> &x_list,
                     const std::vector<double> &y_list,
                     Eigen::VectorXd *gradient);
};
#endif //PATH_OPTIMIZER_INCLUDE_PATH_OPTIMIZER_REFERENCE_PATH_SMOOTHER_TENSION_SMOOTHER_2_HPP_
