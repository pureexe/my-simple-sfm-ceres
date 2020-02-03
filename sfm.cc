#include <cmath>
#include <cstdio>
#include <iostream>
#include <fstream>
#include "ceres/ceres.h"
#include "ceres/rotation.h"

// Read a Bundle Adjustment in the Large dataset.
class BALProblem {
 public:
  ~BALProblem() {
    delete[] point_index_;
    delete[] camera_index_;
    delete[] observations_;
    delete[] parameters_;
  }

  int num_observations()       const { return num_observations_;               }
  const double* observations() const { return observations_;                   }
  double* mutable_cameras()          { return parameters_;                     }
  double* mutable_points()           { return parameters_  + 9 * num_cameras_; }
  double* mutable_camera_for_observation(int i) {
    return mutable_cameras() + camera_index_[i] * 9;
  }
  double* mutable_point_for_observation(int i) {
    return mutable_points() + point_index_[i] * 3;
  }
  double*  mutable_focal_length() {
    return focal_length_index_;
  }
  double*  mutable_distrotion() {
    return distrotion_index_;
  }
  double*  mutable_rotation(int i) {
    return rotation_index_ + camera_index_[i] * 9;
  }
  double*  mutable_translation(int i) {
    return translation_index_ + camera_index_[i] * 3;
  }
  bool LoadFile(const char* filename) {
    FILE* fptr = fopen(filename, "r");
    if (fptr == NULL) {
      return false;
    };
    FscanfOrDie(fptr, "%d", &num_cameras_);
    FscanfOrDie(fptr, "%d", &num_points_);
    FscanfOrDie(fptr, "%d", &num_observations_);
    point_index_ = new int[num_observations_];
    camera_index_ = new int[num_observations_];
    observations_ = new double[2 * num_observations_];
    num_parameters_ = 9 * num_cameras_ + 3 * num_points_;
    parameters_ = new double[num_parameters_];
    for (int i = 0; i < num_observations_; ++i) {
      FscanfOrDie(fptr, "%d", camera_index_ + i);
      FscanfOrDie(fptr, "%d", point_index_ + i);
      for (int j = 0; j < 2; ++j) {
        FscanfOrDie(fptr, "%lf", observations_ + 2*i + j);
      }
    }
    for (int i = 0; i < num_parameters_; ++i) {
      FscanfOrDie(fptr, "%lf", parameters_ + i);
    }
    // change to our focal length style
    focal_length_index_ = new double[1];
    distrotion_index_ =  new double[1];
    rotation_index_ = new double[9*num_cameras_];
    translation_index_ = new double[3*num_cameras_];
    // set intial to avoid devide by 0
    focal_length_index_[0] = 0.01;
    distrotion_index_[0] = 0.01;
    // temporary use inital from previous as random init
    for (int i = 0; i < num_cameras_; ++i) {
      for(int j = 0; j < 9; j++){
        rotation_index_[i*9 + j] = parameters_[(i * 12 + j)];
      }
      for(int j = 0; j < 3; j++){
        rotation_index_[i*3 + j] = parameters_[(i * 12) + (9+j)];
      }
    }
    return true;
  }
  void WriteToPLYFile(const std::string& filename){
    std::ofstream of(filename.c_str());
    of << "ply"
      << '\n' << "format ascii 1.0"
      << '\n' << "element vertex " << num_cameras_ + num_points_
      << '\n' << "property float x"
      << '\n' << "property float y"
      << '\n' << "property float z"
      << '\n' << "property uchar red"
      << '\n' << "property uchar green"
      << '\n' << "property uchar blue"
      << '\n' << "end_header" << std::endl;
    // Export the structure (i.e. 3D Points) as white points.
    const double* points = parameters_ + 9 * num_cameras_;
    for (int i = 0; i < num_points_; ++i) {
      const double* point = points + i * 3;
      for (int j = 0; j < 3; ++j) {
        of << point[j] << ' ';
      }
      of << "255 255 255\n";
    }
    of.close();
  }
 private:
  template<typename T>
  void FscanfOrDie(FILE *fptr, const char *format, T *value) {
    int num_scanned = fscanf(fptr, format, value);
    if (num_scanned != 1) {
      LOG(FATAL) << "Invalid UW data file.";
    }
  }
  int num_cameras_;
  int num_points_;
  int num_observations_;
  int num_parameters_;
  int* point_index_;
  int* camera_index_;
  double* observations_;
  double* parameters_;
  double* focal_length_index_;
  double* distrotion_index_;
  double* rotation_index_;
  double* translation_index_;
};


struct StructureFromMotion {
  StructureFromMotion(double observed_x, double observed_y){
    this->observed_x = observed_x;
    this->observed_y = observed_y;
    this->camera_px = 460.0;
    this->camera_py = 460.0;
  }
  template <typename T>
  bool operator()(
                  const T* const focal_length,
                  const T* const distrotion,
                  const T* const rotation,
                  const T* const translation,
                  const T* const point3d,
                  T* residuals) const {
    
    T projection[3];
    T u[3];
    for(int i=0; i < 3; i++){
      u[i] = translation[i];
      for(int j=0; j <3; j++){
        u[i] += rotation[3*i+j] * point3d[j];
      }
    }
  
    projection[0] = focal_length[0]*u[0] + distrotion[0]*u[1] + camera_px*u[2];
    projection[1] = focal_length[0]*u[1] + camera_py*u[2];
    projection[2] = u[2];
    T predicted_x = projection[0] / projection[2];
    T predicted_y = projection[1] / projection[2];
    residuals[0] = predicted_x - observed_x;
    residuals[1] = predicted_y - observed_y;
    return true;
  }
  static ceres::CostFunction* Create(const double observed_x,
                                     const double observed_y) {
    return (new ceres::AutoDiffCostFunction<StructureFromMotion, 2, 1, 1, 9, 3, 3>(
                new StructureFromMotion(observed_x, observed_y)));
  }
  double observed_x;
  double observed_y;
  double camera_px;
  double camera_py;
};

void SetMinimizerOptions(ceres::Solver::Options* options) {
  options->max_num_iterations = 1000;
}

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::SetLogDestination(google::INFO, "../log/out.log");
  if (argc != 2) {
    std::cerr << "usage: sfm <txtfile>\n";
    return 1;
  }
  BALProblem bal_problem;
  if (!bal_problem.LoadFile(argv[1])) {
    std::cerr << "ERROR: unable to open file " << argv[1] << "\n";
    return 1;
  }
  const double* observations = bal_problem.observations();\
  ceres::Problem problem;
  for (int i = 0; i < bal_problem.num_observations(); ++i) {
    ceres::CostFunction* cost_function =
        StructureFromMotion::Create(observations[2 * i + 0],
                                         observations[2 * i + 1]);
    problem.AddResidualBlock(cost_function,
                             NULL /* squared loss */,
                             bal_problem.mutable_focal_length(),
                             bal_problem.mutable_distrotion(),
                             bal_problem.mutable_rotation(i),
                             bal_problem.mutable_translation(i),
                             bal_problem.mutable_point_for_observation(i));
  }
  ceres::Solver::Options options;
  SetMinimizerOptions(&options);
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << "\n";
  std::string output_structure_filename = "";
  output_structure_filename = output_structure_filename + argv[1] + ".ply";
  bal_problem.WriteToPLYFile(output_structure_filename.c_str());
  return 0;
}