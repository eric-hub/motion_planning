#include <hw_tool.h>

using namespace std;
using namespace Eigen;

void Homeworktool::initGridMap(double _resolution, Vector3d global_xyz_l,
                               Vector3d global_xyz_u, int max_x_id,
                               int max_y_id, int max_z_id) {
  gl_xl = global_xyz_l(0);
  gl_yl = global_xyz_l(1);
  gl_zl = global_xyz_l(2);

  gl_xu = global_xyz_u(0);
  gl_yu = global_xyz_u(1);
  gl_zu = global_xyz_u(2);

  GLX_SIZE = max_x_id;
  GLY_SIZE = max_y_id;
  GLZ_SIZE = max_z_id;
  GLYZ_SIZE = GLY_SIZE * GLZ_SIZE;
  GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE;

  resolution = _resolution;
  inv_resolution = 1.0 / _resolution;

  data = new uint8_t[GLXYZ_SIZE];
  memset(data, 0, GLXYZ_SIZE * sizeof(uint8_t));
}

void Homeworktool::setObs(const double coord_x, const double coord_y,
                          const double coord_z) {
  if (coord_x < gl_xl || coord_y < gl_yl || coord_z < gl_zl ||
      coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu)
    return;

  int idx_x = static_cast<int>((coord_x - gl_xl) * inv_resolution);
  int idx_y = static_cast<int>((coord_y - gl_yl) * inv_resolution);
  int idx_z = static_cast<int>((coord_z - gl_zl) * inv_resolution);

  data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1;
}

bool Homeworktool::isObsFree(const double coord_x, const double coord_y,
                             const double coord_z) {
  Vector3d pt;
  Vector3i idx;

  pt(0) = coord_x;
  pt(1) = coord_y;
  pt(2) = coord_z;
  idx = coord2gridIndex(pt);

  int idx_x = idx(0);
  int idx_y = idx(1);
  int idx_z = idx(2);

  return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE &&
          idx_z >= 0 && idx_z < GLZ_SIZE &&
          (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] < 1));
}

Vector3d Homeworktool::gridIndex2coord(const Vector3i &index) {
  Vector3d pt;

  pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl;
  pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;
  pt(2) = ((double)index(2) + 0.5) * resolution + gl_zl;

  return pt;
}

Vector3i Homeworktool::coord2gridIndex(const Vector3d &pt) {
  Vector3i idx;
  idx << min(max(int((pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1),
      min(max(int((pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1),
      min(max(int((pt(2) - gl_zl) * inv_resolution), 0), GLZ_SIZE - 1);

  return idx;
}

Eigen::Vector3d Homeworktool::coordRounding(const Eigen::Vector3d &coord) {
  return gridIndex2coord(coord2gridIndex(coord));
}

double Homeworktool::OptimalBVP(Eigen::Vector3d _start_position,
                                Eigen::Vector3d _start_velocity,
                                Eigen::Vector3d _target_position) {
  double optimal_cost =
      100000; // this just to initial the optimal_cost, you can delete it
  /*
  STEP 2: go to the hw_tool.cpp and finish the function Homeworktool::OptimalBVP
  the solving process has been given in the document

  because the final point of trajectory is the start point of OBVP, so we input
  the pos,vel to the OBVP

  after finish Homeworktool::OptimalBVP, the Trajctory_Cost will record the
  optimal cost of this trajectory
  */

  //以上使用sympy 获取
  // J的导数，记J_diff，因求T的极值，J_diff=0,左右同乘T**4,消除分母。
  //接下来合并同类项，使用eigen求根。

  double p_x_0 = _start_position(0);
  double p_y_0 = _start_position(1);
  double p_z_0 = _start_position(2);
  double p_x_f = _target_position(0);
  double p_y_f = _target_position(1);
  double p_z_f = _target_position(2);
  double v_x_0 = _start_velocity(0);
  double v_y_0 = _start_velocity(1);
  double v_z_0 = _start_velocity(2);
  double c0 =
      -36.0 * pow(p_x_0, 2) + 72.0 * p_x_0 * p_x_f - 36.0 * pow(p_x_f, 2) -
      36.0 * pow(p_y_0, 2) + 72.0 * p_y_0 * p_y_f - 36.0 * pow(p_y_f, 2) -
      36.0 * pow(p_z_0, 2) + 72.0 * p_z_0 * p_z_f - 36.0 * pow(p_z_f, 2);

  double c1 = -24.0 * p_x_0 * v_x_0 + 24.0 * p_x_f * v_x_0 -
              24.0 * p_y_0 * v_y_0 + 24.0 * p_y_f * v_y_0 -
              24.0 * p_z_0 * v_z_0 + 24.0 * p_z_f * v_z_0;

  double c2 = -4.0 * pow(v_x_0, 2) - 4.0 * pow(v_y_0, 2) - 4.0 * pow(v_z_0, 2);
  double c3 = 0.0;

  Eigen::Matrix<double, 4, 4> matrix_44;
  Eigen::Matrix<complex<double>, Eigen::Dynamic, Eigen::Dynamic>
      matrix_eigenvalues;
  matrix_44 << 0, 0, 0, -c0, 1, 0, 0, -c1, 0, 1, 0, -c2, 0, 0, 1, -c3;

  matrix_eigenvalues = matrix_44.eigenvalues();

  //   std::cout << "----------- " << std::endl;
  //   std::cout << "matrix_eigenvalues: " << std::endl
  //             << matrix_eigenvalues << std::endl;
  for (int i = 0; i < matrix_eigenvalues.size(); i++) {
    if (matrix_eigenvalues(i).imag() == 0.0 &&
        matrix_eigenvalues(i).real() > 0.0) {
      double t = matrix_eigenvalues(i).real();
      //   std::cout << "optimal_t:" << t << std::endl;
      double optimal_J = computerJ(t, p_x_0, p_y_0, p_z_0, p_x_f, p_y_f, p_z_f,
                                   v_x_0, v_y_0, v_z_0);
      if (optimal_J < optimal_cost)
        optimal_cost = optimal_J;
    }
  }

  //   std::cout << "optimal_cost:" << optimal_cost << std::endl;
  return optimal_cost;
}

double Homeworktool::computerJ(double t, double p_x_0, double p_y_0,
                               double p_z_0, double p_x_f, double p_y_f,
                               double p_z_f, double v_x_0, double v_y_0,
                               double v_z_0) {
  return 1.0 *
         (1.0 * pow(t, 4) + 4.0 * pow(t, 2) * pow(v_x_0, 2) +
          4.0 * pow(t, 2) * pow(v_y_0, 2) + 4.0 * pow(t, 2) * pow(v_z_0, 2) +
          12.0 * t * p_x_0 * v_x_0 - 12.0 * t * p_x_f * v_x_0 +
          12.0 * t * p_y_0 * v_y_0 - 12.0 * t * p_y_f * v_y_0 +
          12.0 * t * p_z_0 * v_z_0 - 12.0 * t * p_z_f * v_z_0 +
          12.0 * pow(p_x_0, 2) - 24.0 * p_x_0 * p_x_f + 12.0 * pow(p_x_f, 2) +
          12.0 * pow(p_y_0, 2) - 24.0 * p_y_0 * p_y_f + 12.0 * pow(p_y_f, 2) +
          12.0 * pow(p_z_0, 2) - 24.0 * p_z_0 * p_z_f + 12.0 * pow(p_z_f, 2)) /
         pow(t, 3);
}