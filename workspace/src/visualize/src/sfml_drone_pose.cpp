#include <SFML/Graphics.hpp>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <string>

float pose_x = 0.0f;
float pose_y = 0.0f;
float pose_z = 0.0f;

void poseCallback(const nav_msgs::Odometry &msg)
{
    // Update pose data
    pose_x = msg.pose.pose.position.x;
    pose_y = msg.pose.pose.position.y;
    pose_z = msg.pose.pose.position.z;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sfml_triangle_node");
    ros::NodeHandle nh;

    ros::Rate rate(10);

    int width = 800, height = 800;

    sf::RenderWindow window(sf::VideoMode(width, height), "SFML Window");

    sf::Font font;
    std::string fontLocation = "/usr/share/fonts/truetype/freefont/FreeMono.ttf";

    if (!font.loadFromFile(fontLocation))
    {
        std::cerr << "Failed to load font file " << fontLocation << std::endl;
        return 1;
    }

    // Subscribe to /pose topic
    ros::Subscriber sub = nh.subscribe("/sitl/mavros/local_position/odom", 10, poseCallback);

    // Main loop
    while (window.isOpen() && ros::ok())
    {
        ros::spinOnce();

        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        window.clear();

        sf::Text poseText;
        poseText.setFont(font); // Load font here
        poseText.setCharacterSize(20);
        poseText.setFillColor(sf::Color::White);
        poseText.setPosition(10, 10);
        poseText.setString("Pose: x=" + std::to_string(pose_x) + ", y=" + std::to_string(pose_y) + ", z=" + std::to_string(pose_z));
        window.draw(poseText);

        sf::CircleShape circle(5); // Adjust the radius as needed
        circle.setFillColor(sf::Color::Red);
        circle.setPosition(pose_x * 10 + width / 2.0, -pose_y * 10 + height / 2.0);
        window.draw(circle);

        window.display();

        rate.sleep();
    }

    return 0;
}
