#include <pluginlib/class_list_macros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_core/base_global_planner.h>
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>

namespace test_planner {

    class TestPlanner : public nav_core::BaseGlobalPlanner {
public:
  TestPlanner();
  TestPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  bool makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan) override;
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override {
                // 必要な初期化処理を実装
            }
};
}

PLUGINLIB_EXPORT_CLASS(test_planner::TestPlanner, nav_core::BaseGlobalPlanner)