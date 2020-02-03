/**
 * My Simple SFM (Structure from motion) build on ceres-sover
 * We solve 
 * min_{I,E,P} \sum || [u,v]^T - I \cdot E \cdot P ||^2
 * Where
 * - I is Intrisic of Camera
 * > I is 3x3 matrix (http://ksimek.github.io/2013/08/13/intrinsic/) 
 * - E is Extrinsic of Camera
 * > E is 3x4 Matrix which [R,T] R is 3x3 and T is 3x1 (http://ksimek.github.io/2012/08/22/extrinsic/)
 * - P is 3x1 matrix of point 3D
 * > Point in 3D contain condinate (x,y,z)
 * */

#include "ceres/ceres.h"
#include "glog/logging.h"
using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;


struct ProjectionLoss {
  //fields variable
  double observed_x;
  double observed_y;
  double instrisic_px;
  double instrisic_py;

  static ceres::CostFunction* Create(
    const double observed_x,
    const double observed_y,
    const double intrisic_px = 0,
    const double intrisic_py = 0
  ){
    return (new ceres::AutoDiffCostFunction<ProjectionLoss, 2, 1, 1, 12 >(new ProjectionLoss(observed_x, observed_y, intrisic_px, intrisic_py)));
  }

  ProjectionLoss(double x, double y, double px, double py){
    this->observed_x = x;
    this->observed_y = y;
    this->instrisic_px = px;
    this->instrisic_py = py;
  }
  
  template <typename T> bool operator()(
    const T* const focal_length,
    const T* const axis_skew, //aka. distrotion
    const T* const extrinsic,
    T* residual
  ) const {
    residual[0] = T(10.0) - camera[0];
    return true;
  }
  
   
  
};

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);

  // The variable to solve for with its initial value.
  double initial_x = 5.0;
  double x = initial_x;

  // Build the problem.
  Problem problem;

  // Set up the only cost function (also known as residual). This uses
  // auto-differentiation to obtain the derivative (jacobian).
  CostFunction* cost_function = ProjectionLoss::Create(0,0);
  problem.AddResidualBlock(cost_function, NULL, &x);


  // Run the solver!
  Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;
  Solver::Summary summary;
  Solve(options, &problem, &summary);

  std::cout << summary.BriefReport() << "\n";
  std::cout << "x : " << initial_x
            << " -> " << x << "\n";
  return 0;
}
