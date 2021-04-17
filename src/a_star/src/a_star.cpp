/*include*/
/*for ros related*/
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
/*for vector include*/
#include <iostream>
#include <deque>
/*other*/
#include <stdio.h>
#include <stdbool.h>
#include <math.h>

/*define*/
#define LOOP_RATE 10
/*Map Related*/
#define XMAP_SIZE 10    //in meter
#define YMAP_SIZE 10    //in meter
#define X_SHIFT -5.0
#define Y_SHIFT -5.0
#define XMIN 0
#define YMIN 0 
#define E_SIZE 2    //Enlarge size
#define XMAX 99
#define YMAX 99
#define EMPTY -128
#define UNKNOWN -1
#define SAFE 0
#define OCCUPIED 100

/** ref doc : 
*  1. A* algorithm implement:https://dev.to/jansonsa/a-star-a-path-finding-c-4a4h
*  2. Youtube Video:https://www.youtube.com/watch?v=T8mgXpW1_vc
*  3. Sub Once:https://charon-cheung.github.io/2019/01/16/ROS/ROS%20Kinetic%E7%9F%A5%E8%AF%86/%E5%8F%AA%E5%8F%91%E5%B8%83%E5%92%8C%E8%AE%A2%E9%98%85%E4%B8%80%E6%AC%A1%E6%B6%88%E6%81%AF/#%E5%8F%AA%E5%8F%91%E5%B8%83%E4%B8%80%E6%AC%A1%E6%B6%88%E6%81%AF
*  4. STL Vector:https://ithelp.ithome.com.tw/articles/10231601
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
  struct true_Coordinate
  {
      /*unit : meter*/
      float x;
      float y;

      bool operator==(const true_Coordinate& c) const {
        return((c.x == x) && c.y ==y);
      }
  };

  struct grid_Coordinate
  {
      /*unit : grid*/
      int x;
      int y;

      bool operator==(const grid_Coordinate& c) const {
        return((c.x == x) && c.y ==y);
      }
  };
  

  struct Node
  {
      grid_Coordinate self;
      grid_Coordinate parent;
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
  int Map[100][100] = {};
  int eMap[100][100] = {};
  //target in 2x4 matrix
  std::deque<true_Coordinate> destination_queue;
  
  /*function*/
    static bool isValid(int x, int y)
  {
      if (x <= XMAX && y <= YMAX && x >= XMIN && y >= YMIN)
      {
        /*check state from eMap*/
        if(eMap[x][y] == SAFE)
            return true; 
        else 
            return false;
      }
      return false;
  }

  static bool isDestination(int x,int y)
  {
      /*change to meter first*/
      float new_x = (float)(x+1)/XMAP_SIZE + X_SHIFT;   //need to casting to float 
      float new_y = (float)(y+1)/YMAP_SIZE + Y_SHIFT;
      true_Coordinate _current={
          .x = new_x,
          .y = new_y
          
      };
      std::deque<true_Coordinate>::iterator iter = find(destination_queue.begin(),destination_queue.end(),_current);
      /*find in deque*/
      if(iter != destination_queue.end()) 
        return true;
      else
        return false;
  }

  void print_eMap()
  {
      printf("Map\n");
    for (int y = YMAX; y > YMIN-1 ; y--)
    {
        for (int x = XMIN; x < XMAX +1; x++)
        {
            if(Map[x][y] == OCCUPIED)
                printf("+");
            else if(Map[x][y] == SAFE)
                printf(" ");
            else
                printf("+");
        }
        printf("\n");
    }
    printf("\n\n");
    printf("eMap\n");

    for (int y = YMAX; y > YMIN-1 ; y--)
    {
        for (int x = XMIN; x < XMAX +1; x++)
        {
            if(isDestination(x,y))
                printf("o");
            else if(eMap[x][y] == OCCUPIED)
                printf("+");
            else if(eMap[x][y] == SAFE)
                printf(" ");
            else
                printf("+");
        }
        printf("\n");
    }
  }

  void EnlargeMap()
  {
    ROS_INFO("Start Map Update");

    /*make 2D map and init eMap*/
    for (size_t i = XMIN; i < XMAX+1; i++)
    {
        for (size_t j = XMIN; j < YMAX+1; j++)
        {
            Map[i][j] = my_map->data[(XMAX+1)*j+i];
            eMap[i][j] = EMPTY;
        }
    }

    /*enlarge obstacles to fit phsical car size*/ 
    for (size_t i = XMIN; i <= XMAX; i++)
    {
        for (size_t j = XMIN; j <= YMAX; j++)
        {
            /*if occupied then enlarge*/
            if(Map[i][j] == OCCUPIED)
            {
                for(int x_bias = -E_SIZE;x_bias < E_SIZE+1;x_bias++)
                {
                    for(int y_bias = -E_SIZE;y_bias <E_SIZE+1;y_bias++)
                    {
                        /*if in range*/
                        if(i+x_bias >= 0 && i+x_bias <= XMAX && j+y_bias >=0 && j+y_bias <= YMAX)
                        {
                            eMap[i+x_bias][j+y_bias] = OCCUPIED;
                        }
                    }
                }
            }
            /*if eMap empty*/
            else if(eMap[i][j] == EMPTY)
            {
                /*if nothing or unknown then copy*/
                if(Map[i][j] == SAFE || Map[i][j] == UNKNOWN)
                {
                    eMap[i][j] = Map[i][j];
                }
            }  
        } 
    }

    print_eMap();
    ROS_INFO("Finish Map Update");
  }

  void InitTarget()
  {
      true_Coordinate target_1={
          .x = 3.00,
          .y = 4.00
      };

      true_Coordinate target_2={
          .x = 3.00,
          .y = -2.00
      };

      true_Coordinate target_3={
          .x = -4.00,
          .y = -3.00
      };

      true_Coordinate target_4={
          .x = -4.00,
          .y = 4.00
      };

      destination_queue.push_back(target_1);
      destination_queue.push_back(target_2);
      destination_queue.push_back(target_3);
      destination_queue.push_back(target_4);

  }



  int main(int argc, char** argv)
  {
      ros::init(argc,argv,"A_star_Sim");
      ros::NodeHandle nh;
      my_map = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/map");
      ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",1000);
      ros::Rate rate(LOOP_RATE);
      sleep(1);

      /* Enlarge the map */
      if(my_map!=nullptr)
      {
        InitTarget();
        EnlargeMap();
        while(ros::ok())
        {
            /**/

        }
      }
      else
        ROS_INFO("Didn't get anything from /map");
  }
