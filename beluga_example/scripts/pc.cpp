#include <iostream>

#include <cilantro/utilities/point_cloud.hpp>

const std::string kPathToMap = "/ws/src/beluga/beluga_example/pc.ply";

int main(int argc, char** argv) {
  cilantro::PointCloud3f cloud;

  // Read ply file
  std::cout << "Reading file from " << kPathToMap << '\n';

  cloud.fromPLYFile(kPathToMap);
  std::cout << cloud.points.cols() << " points read" << std::endl;
  std::cout << cloud.normals.cols() << " normals read" << std::endl;
  std::cout << cloud.colors.cols() << " colors read" << std::endl;

  return 0;
}