// ref : https://github.com/ros/ros_tutorials/blob/melodic-devel/turtlesim/tutorials/draw_square.cpp
// ref : https://github.com/Ewenwan/Ros/blob/master/%E8%B5%84%E6%96%99%E6%80%BB%E7%BB%93/0.0_ros%E5%9F%BA%E7%A1%80%E7%9F%A5%E8%AF%86%20.md

# include <ros/ros.h> 
# include <geometry_msgs/Twist.h>
# include <stdlib.h>

# define PI 3.1415926535897

void move_straight();
void turn_cw();

int freq = 10;
geometry_msgs::Twist vel_msg;

// It's a publisher to vel, update info for turtlesim_node

int main (int argc, char **argv)
{
    ros::init(argc, argv, "turtle_square_node") ;

    ros::NodeHandle nh;

    ROS_INFO_STREAM("Turtle_Square node Start\n");

    //publisher to vel , 1hz
    
    int count = 0;
    vel_msg.linear.y = 0.0;
    vel_msg.linear.z = 0.0;
    vel_msg.angular.x = 0.0;
    vel_msg.angular.y = 0.0;

    ros::Publisher vel_puber = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel",1000);
    
    ros::Rate loop_rate(freq);

    

    while(ros::ok())
    {
        if(count < freq)
        {
            move_straight();
            count++;
        }
        else
        {
            turn_cw();
            count = 0;
        }
        vel_puber.publish(vel_msg);
        loop_rate.sleep();
        

    }

    return 0;
}

void move_straight()
{
    vel_msg.linear.x = 1.0;
    vel_msg.angular.z = 0.0;
}

void turn_cw()
{
    vel_msg.linear.x = 0.0;
    vel_msg.angular.z = PI / 2 * freq;  // (pi/2)*freq for 90 degree
}