//
// Created by hvt on 19-9-24.
//

#ifndef BOTTLE_2DNAV_RP_GLOBAL_PLANNER_H
#define BOTTLE_2DNAV_RP_GLOBAL_PLANNER_H
/** include the libraries you need in your planner here */
/** for global path planner interface */
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <tf/transform_datatypes.h>//转换函数头文件

using std::string;

#ifndef GLOBAL_PLANNER_CPP
#define GLOBAL_PLANNER_CPP

namespace global_planner {

    class RoadPointGlobalPlanner : public nav_core::BaseGlobalPlanner {
    public:

        RoadPointGlobalPlanner();

        RoadPointGlobalPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros);

        /** overridden classes from interface nav_core::BaseGlobalPlanner **/
        void initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros);

        bool makePlan(const geometry_msgs::PoseStamped &start,
                      const geometry_msgs::PoseStamped &goal,
                      std::vector<geometry_msgs::PoseStamped> &plan
        );

    private:
        // 地图轨迹
        std::vector<geometry_msgs::PoseStamped> trajectory;
        // 当前路点索引
        int currentIndex;

    };
};
#endif
#endif //BOTTLE_2DNAV_RP_GLOBAL_PLANNER_H
