#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>
#include <limits>
#include <mutex>
#include <vector>
#include <unordered_map>

class Chaser : public rclcpp::Node
{
public:
    Chaser() : Node("chaser"), current_pose_initialized_(false)
    {
        pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
            "turtle1/pose", 10, std::bind(&Chaser::pose_callback, this, std::placeholders::_1));

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&Chaser::chase_turtle, this));

        for (int i = 2; i < 100; ++i)
        {
            std::string topic_name = "turtle" + std::to_string(i) + "/pose";
            auto sub = this->create_subscription<turtlesim::msg::Pose>(
                topic_name, 10, [this, i](turtlesim::msg::Pose::SharedPtr msg) 
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    poses_[i] = *msg;
                });
            subs_.push_back(sub);
        }
    }

private:
    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        current_pose_ = *msg;
        current_pose_initialized_ = true;
    }

    void chase_turtle()
    {
        if (!current_pose_initialized_)
            return;

        int closest_turtle_id = -1;
        double closest_distance = std::numeric_limits<double>::max();

        {
            std::lock_guard<std::mutex> lock(mutex_);
            for (const auto &pair : poses_)
            {
                if (captured_turtles_.count(pair.first) > 0)
                    continue;

                double distance = std::hypot(pair.second.x - current_pose_.x, pair.second.y - current_pose_.y);
                if (distance < closest_distance)
                {
                    closest_distance = distance;
                    closest_turtle_id = pair.first;
                }
            }
        }

        if (closest_turtle_id != -1 && closest_distance < 1.0)
        {
            captured_turtles_.insert(closest_turtle_id);
        }

        move_towards_turtle(closest_turtle_id);
        move_captured_turtles();
    }

    void move_towards_turtle(int turtle_id)
    {
        if (turtle_id == -1 || captured_turtles_.count(turtle_id) > 0)
            return;

        auto target_pose = poses_[turtle_id];
        auto msg = geometry_msgs::msg::Twist();

        double angle_to_target = std::atan2(target_pose.y - current_pose_.y, target_pose.x - current_pose_.x);
        double angle_diff = angle_to_target - current_pose_.theta;

        // 确保角度差在 [-pi, pi] 范围内
        angle_diff = std::atan2(std::sin(angle_diff), std::cos(angle_diff));

        if (std::abs(angle_diff) > 0.1)
        {
            // 调整角速度，使其与角度差成比例
            msg.angular.z = 0.8 * angle_diff;
            msg.linear.x = 0.0; // 仅旋转，不前进
        }
        else
        {
            // 当角度差足够小，向前移动
            msg.angular.z = 0.0;
            msg.linear.x = 1.5; // 设置为适当的速度
        }

        cmd_pub_->publish(msg);
    }

    void move_captured_turtles()
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (captured_turtles_.empty()) return;

        turtlesim::msg::Pose previous_pose = current_pose_;

        for (const auto &id : captured_turtles_)
        {
            auto target_pose = poses_[id];
            auto msg = geometry_msgs::msg::Twist();

            double angle_to_target = std::atan2(previous_pose.y - target_pose.y, previous_pose.x - target_pose.x);
            double angle_diff = angle_to_target - target_pose.theta;

            // 确保角度差在 [-pi, pi] 范围内
            angle_diff = std::atan2(std::sin(angle_diff), std::cos(angle_diff));

            if (std::abs(angle_diff) > 0.2)
            {
                // 调整角速度，使其与角度差成比例
                msg.angular.z = 1.2 * angle_diff;
                msg.linear.x = 0.0; // 仅旋转，不前进
            }
            else
            {
                // 当角度差足够小，向前移动
                msg.angular.z = 0.0;
                msg.linear.x = 2.5; // 设置为适当的速度
                if (std::abs(target_pose.x - previous_pose.x) < 0.5 && std::abs(target_pose.y - previous_pose.y) < 0.5)
                {
                    msg.linear.x = 0;
                }
                
            }

            auto pub = this->create_publisher<geometry_msgs::msg::Twist>("turtle" + std::to_string(id) + "/cmd_vel", rclcpp::QoS(10));
            pub->publish(msg);

            previous_pose = target_pose;
        }
    }

    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr> subs_;

    turtlesim::msg::Pose current_pose_;
    bool current_pose_initialized_;
    std::unordered_map<int, turtlesim::msg::Pose> poses_;
    std::unordered_set<int> captured_turtles_;
    std::mutex mutex_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Chaser>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
