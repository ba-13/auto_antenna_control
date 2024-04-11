#include <SFML/Graphics.hpp>
#include <ros/ros.h>
#include <messages/MotorPose.h>
#include <std_msgs/Header.h>

double step = 0.0f;
double pred_step = 0.0f;
std_msgs::Header header_curr;

void currPoseCallback(const messages::MotorPose &msg) {
    step = msg.step;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sfml_motor_pose");
    ros::NodeHandle nh;
    ros::Rate rate(5);

    int width = 800, height = 800;
    sf::RenderWindow window(sf::VideoMode(width, height), "SFML Window");

    sf::Font font;
    std::string fontLocation = "/usr/share/fonts/truetype/freefont/FreeMono.ttf";

    if (!font.loadFromFile(fontLocation))
    {
        std::cerr << "Failed to load font file " << fontLocation << std::endl;
        return 1;
    }
    // ros::Subscriber sub = nh.subscribe("/sitl/mavros/local_position/odom", 10, poseCallback);
}