#include "nav2_util/node_utils.hpp"
#include "nav2_core/exceptions.hpp"
#include <cmath>
#include <memory>
#include <string>

#include "nav2_core/exceptions.hpp"
#include <stdexcept>                // 这是 PlannerException 内部依赖的
#include "nav2_custom_planner/nav2_custom_planner.hpp"

#include "tf2/LinearMath/Quaternion.h"         // 用于处理四元数（方向）
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" // 用于tf2和

namespace nav2_custom_planner
{

    void CustomPlanner::configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
        std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
    {
        tf_ = tf;
        // parent.lock() 把它从一个弱引用变成一个可以使用的强引用
        node_ = parent.lock();// parent 参数是 PlannerServer 传进来的，它代表 PlannerServer 这个节点，你用 node_ 变量把它存起来
        name_ = name;   // name 参数是 PlannerServer 传进来的，通常是 "CustomPlanner" 这个字符串，你用 name_ 变量把它存起来
        costmap_ = costmap_ros->getCostmap();
        global_frame_ = costmap_ros->getGlobalFrameID();
        // 参数初始化
        nav2_util::declare_parameter_if_not_declared(
            node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.1));
        node_->get_parameter(name_ + ".interpolation_resolution",
                             interpolation_resolution_);
    }

    void CustomPlanner::cleanup()
    {
        RCLCPP_INFO(node_->get_logger(), "正在清理类型为 CustomPlanner 的插件 %s",
                    name_.c_str());
    }

    void CustomPlanner::activate()
    {
        RCLCPP_INFO(node_->get_logger(), "正在激活类型为 CustomPlanner 的插件 %s",
                    name_.c_str());
    }

    void CustomPlanner::deactivate()
    {
        RCLCPP_INFO(node_->get_logger(), "正在停用类型为 CustomPlanner 的插件 %s",
                    name_.c_str());
    }

    // createPlan 函数的正确实现
    nav_msgs::msg::Path
    CustomPlanner::createPlan(const geometry_msgs::msg::PoseStamped & start,
                            const geometry_msgs::msg::PoseStamped & goal)
    {
        RCLCPP_INFO(
            node_->get_logger(), "CustomPlanner: 正在规划从 (%.2f, %.2f) 到 (%.2f, %.2f) 的路径",
            start.pose.position.x, start.pose.position.y,
            goal.pose.position.x, goal.pose.position.y);

        if (costmap_ == nullptr) {
            RCLCPP_ERROR(node_->get_logger(), "错误：代价地图未初始化！");
            throw nav2_core::PlannerException("代价地图未初始化");
        }

        nav_msgs::msg::Path global_path;
        // ====================== 核心修改点 ======================
        // 使用传入的 start 的时间戳，而不是 node_->now()
        global_path.header.stamp = start.header.stamp;
        // ========================================================
        global_path.header.frame_id = global_frame_;

        // ... 后续的计算逻辑保持不变 ...
        double dx = goal.pose.position.x - start.pose.position.x;
        double dy = goal.pose.position.y - start.pose.position.y;
        double dist = std::hypot(dx, dy);

        if (dist < interpolation_resolution_) {
            // 如果距离很近，直接返回包含起点和终点的路径
            // 注意：这里的start和goal已经包含了正确的时间戳
            global_path.poses.push_back(start);
            global_path.poses.push_back(goal);
            return global_path;
        }
        
        int n_points = static_cast<int>(dist / interpolation_resolution_);
        double path_angle = atan2(dy, dx);
        tf2::Quaternion orientation_tf2;
        orientation_tf2.setRPY(0, 0, path_angle);
        geometry_msgs::msg::Quaternion orientation_msg = tf2::toMsg(orientation_tf2);

        for (int i = 0; i <= n_points; ++i) {
            double ratio = static_cast<double>(i) / n_points;
            geometry_msgs::msg::PoseStamped current_pose;
            // ====================== 核心修改点 ======================
            // 确保路径上每个点的时间戳也和路径头保持一致
            current_pose.header = global_path.header;
            // ========================================================
            current_pose.pose.position.x = start.pose.position.x + dx * ratio;
            current_pose.pose.position.y = start.pose.position.y + dy * ratio;
            current_pose.pose.position.z = 0;
            current_pose.pose.orientation = orientation_msg;

            unsigned int mx, my;
            if (!costmap_->worldToMap(current_pose.pose.position.x, current_pose.pose.position.y, mx, my)) {
                RCLCPP_WARN(node_->get_logger(), "路径点 (%.2f, %.2f) 在地图范围外！",
                    current_pose.pose.position.x, current_pose.pose.position.y);
                throw nav2_core::PlannerException("路径点在地图范围外");
            }
            
            unsigned char cost = costmap_->getCost(mx, my);
            if (cost >= nav2_costmap_2d::LETHAL_OBSTACLE) { // 使用 >= 更安全
                RCLCPP_ERROR(node_->get_logger(), "规划失败！路径在 (%.2f, %.2f) 处被障碍物阻挡！",
                    current_pose.pose.position.x, current_pose.pose.position.y);
                throw nav2_core::PlannerException("路径被障碍物阻挡");
            }

            global_path.poses.push_back(current_pose);
        }
        
        // 确保最终目标点也包含在路径中，并且时间戳一致
        geometry_msgs::msg::PoseStamped final_goal = goal;
        final_goal.header = global_path.header;
        global_path.poses.push_back(final_goal);

        RCLCPP_INFO(node_->get_logger(), "规划成功！生成了包含 %zu 个点的路径。", global_path.poses.size());
        return global_path;
    }

} // namespace nav2_custom_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_custom_planner::CustomPlanner,
                       nav2_core::GlobalPlanner)   //向整个ROS系统广播基于GlobalPlanner的CustomPlanner