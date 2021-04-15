#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include "stdio.h"
#include "stdbool.h"
#include "math.h"

#define XMIN 0
#define YMIN 0 
#define XMAX 99
#define YMAX 99

/** ref doc : 
*  1. A* algorithm implement : https://dev.to/jansonsa/a-star-a-path-finding-c-4a4h
*  2. Youtube Video : https://www.youtube.com/watch?v=T8mgXpW1_vc
*  3. Sub Once : https://charon-cheung.github.io/2019/01/16/ROS/ROS%20Kinetic%E7%9F%A5%E8%AF%86/%E5%8F%AA%E5%8F%91%E5%B8%83%E5%92%8C%E8%AE%A2%E9%98%85%E4%B8%80%E6%AC%A1%E6%B6%88%E6%81%AF/#%E5%8F%AA%E5%8F%91%E5%B8%83%E4%B8%80%E6%AC%A1%E6%B6%88%E6%81%AF
*/ 

/**
* This cpp goal
* 1. load map once before a_star
*    -> need to get topic once
w
* 2. enlarge map to fit the car's size
*    -> enlarge 2 pixel if occupied

* 3. use A* algorithm to find the best path
*    -> build open,closed list
*    -> build node structure
*
* 4. send the nodes in path to topic-pose like lab2
*
*/

/** @brief Algorithm Function list
*
*   1. isValid
*       @input:
*       @rtnval:
*       @des:
*
*   2. isDestination
*       @input: 
*       @rtnval: (isDestination) : true ? false
*       @des:
*
*   3. calculate_Heuristic
*       @input: 
*       @rtnval: double , Heristic Function's Val
*       @des:
*
*/

/**
 * @brief Map Function list
 *
 *  1. EnlargeMap
 *      @input: int data[]   
 * 
 */

 /**
  * @brief Disguess
  *     # isReached do something to topic
  */

  /*structure df*/
  struct Node
  {
      int x,y;
      int parentX,parentY;
      float gCost;
      float fCost;
      float hCost;
  };

  inline bool operator <(const Node& lhs, const Node& rhs)
  {
      return lhs.fCost < rhs.fCost;
  }

  /*msg df*/
  nav_msgs::OccupancyGridConstPtr my_map;
  geometry_msgs::PoseStampedPtr my_robot_goal;

  /*vars*/
  int eMap[100][100] = {0.0};
  int target[4][2] = 
  {
      {3,4},
      {3,-2},
      {-4,-3},
      {-4,4}
  };

  /*function*/
  void EnlargeMap()
  {
      /*enlarge obstacles to fit phsical car size*/ 
      
  }

  static bool isValid(int x, int y)
  {
      if (x <= XMAX && y <= YMAX && x > XMIN && y > YMIN)
      {
        /*check state from eMap*/
        if(eMap[x][y] == 0)
            return true; 
        else 
            return false;
      }
      return false;
  }



  int main(int argc, char** argv)
  {
      ros::init(argc,argv,"A*_Sim");
      ros::NodeHandle nh;
      my_map = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/map");
      ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",1000);
      ros::Rate rate(10);
      sleep(1);

      /* Enlarge the map */
      if(my_map!=nullptr)
      {
        EnlargeMap();

        while(ros::ok())
        {
            /**/
        }
      }
      else
        ROS_INFO("Didn't get anything from /map");
  }
