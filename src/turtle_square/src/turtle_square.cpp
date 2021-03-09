# include <ros/ros.h> 

// It's a publisher to vel, update info to turtlesim_node

int main (int argc, char **argv)
{
    ros::init(argc, argv, "MyTurtle") ;

    ros::NodeHandle nh;

    ROS_INFO_STREAM("Turtle Squeard Start\n");


    return 0;
}