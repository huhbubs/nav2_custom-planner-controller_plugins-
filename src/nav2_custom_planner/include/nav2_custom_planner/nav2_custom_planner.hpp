#ifndef NAV2_CUSTOM_PLANNER__NAV2_CUSTOM_PLANNER_HPP_
#define NAV2_CUSTOM_PLANNER__NAV2_CUSTOM_PLANNER_HPP_
#include <memory>
#include <string>
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"   //位置定义
#include "rclcpp/rclcpp.hpp"
#include "nav2_core/global_planner.hpp"  //提供基类
#include "nav2_costmap_2d/costmap_2d_ros.hpp"   //栅格地图  
#include "nav2_util/lifecycle_node.hpp"   //rclcpp一个自类
#include "nav2_util/robot_utils.hpp"   //常用工具
#include "nav_msgs/msg/path.hpp"  //返回路径

namespace nav2_custom_planner {
// 自定义导航规划器类
class CustomPlanner : public nav2_core::GlobalPlanner {
public:
  CustomPlanner() = default;
  ~CustomPlanner() = default;
  // 插件配置方法
  void configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
      std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  // 插件清理方法
  void cleanup() override;
  // 插件激活方法
  void activate() override;
  // 插件停用方法
  void deactivate() override;
  // 为给定的起始和目标位姿创建路径的方法
  nav_msgs::msg::Path
  createPlan(const geometry_msgs::msg::PoseStamped &start,
             const geometry_msgs::msg::PoseStamped &goal) override;

private:
  // 坐标变换缓存指针，可用于查询坐标关系
  std::shared_ptr<tf2_ros::Buffer> tf_;
  // 节点指针
  nav2_util::LifecycleNode::SharedPtr node_;
  // 全局代价地图
  nav2_costmap_2d::Costmap2D *costmap_;
  // 全局代价地图的坐标系
  std::string global_frame_, name_;
  // 插值分辨率
  double interpolation_resolution_;
};

} // namespace nav2_custom_planner

#endif // NAV2_CUSTOM_PLANNER__NAV2_CUSTOM_PLANNER_HPP_