#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"
#include <cstdlib>
#include <ctime>

class Spawner : public rclcpp::Node
{
public:
    Spawner() : Node("spawner")
    {
        client_ = this->create_client<turtlesim::srv::Spawn>("spawn");
        timer_ = this->create_wall_timer(
            std::chrono::seconds(3),
            std::bind(&Spawner::spawn_turtle, this));
        srand(time(0));
    }

private:
    void spawn_turtle()
    {
        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
        request->x = static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / 10.0)) + 1.0;
        request->y = static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / 10.0)) + 1.0;
        request->theta = 0.0;
        request->name = "turtle" + std::to_string(turtle_counter_);

        auto result = client_->async_send_request(request);
        RCLCPP_INFO(this->get_logger(), "Spawned turtle%d at (%.2f, %.2f)",
                    turtle_counter_, request->x, request->y);
        turtle_counter_++;
    }

    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
    int turtle_counter_ = 2; // Starting from 2 as 1 is the main turtle
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Spawner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
