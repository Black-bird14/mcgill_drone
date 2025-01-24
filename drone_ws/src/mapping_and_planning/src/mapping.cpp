#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <math.h>

#include "Eigen/Dense"
#include "octomap/OcTree.h"
#include "octomap/octomap.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node {
public:
  MinimalPublisher()
      : Node("minimal_publisher"), drone_frame(4, 4), octomap_granularity(3.) {
    this->drone_frame(3, 3) = 1.;
    Eigen::Quaterniond quat(1, 0, 0, 0);
    auto rotation = quat.matrix();
    Eigen::Vector3d translation(10, 20, 30);
    this->drone_frame.block<3, 3>(0, 0) = rotation;
    this->drone_frame.block<3, 1>(0, 3) = translation;

    std::cout << this->drone_frame << std::endl;

    // Example boundary and detection, refer to the mapping docs for how the map looks like
    std::vector<Eigen::Vector2d> boundary{{-10, 30}, {20, 30}, {20, 10},
                                          {30, 10},  {30, 0},  {-30, 0},
                                          {-30, 10}, {-10, 10}};
   
    // First detection should be right above the drone, 5 meters away, with radius 2
    // Second on should be to the right of the drone, 5 meters away, with a radius 10.
    std::vector<Eigen::Vector4d> detections{{0.0, M_PI, 5., 2.}, {M_PI, 2*M_PI, 20., 10.}};

    std::cout << "Boundary :" << std::endl;
    for (auto vec : boundary) {
      std::cout << vec << std::endl;
    }
    std::cout << "Detections :" << std::endl;
    for (auto vec : detections) {
      std::cout << vec << std::endl;
    }

    octomap::OcTree tree = build_octomap(boundary, detections);
    tree.writeBinary("simple_tree.bt");
    std::cout << "wrote octomap file simple_tree.bt" << std::endl << std::endl;
    std::cout << "now you can use octovis to visualize: octovis simple_tree.bt"
              << std::endl;
  }

  octomap::OcTree build_octomap(std::vector<Eigen::Vector2d> boundary,
                                std::vector<Eigen::Vector4d> detections) {
    octomap::OcTree tree(this->octomap_granularity);

    // TODO: Fill your code here
    // Build the tree, refer to
    // https://github.com/OctoMap/octomap/blob/devel/octomap/src/simple_example.cpp
    // for an example of how to use octomap The drone's frame of reference is in
    // the this->drone_frame matrix (homogenous coords) std::vector is C++'s
    // class for an ArrayList, Eigen::Vector2d and Eigen::Vector4d are eigen's
    // implementation of linear algebra vectors
    // Avoid the keyword `new` or any form of heap utilization, just use the
    // stack! If you do use `new`, always remember to use `delete` when
    // appropriate to free the memory.

    return tree;
  }

private:
  Eigen::MatrixXd drone_frame;
  double octomap_granularity;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
