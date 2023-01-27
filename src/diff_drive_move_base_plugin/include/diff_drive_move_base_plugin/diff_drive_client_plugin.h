#ifndef DIFF_DRIVE_CLIENT_PLUGIN_H_
#define DIFF_DRIVE_CLIENT_PLUGIN_H_
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <diff_drive_msgs/diff_drive_path_planning.h>

namespace diff_drive_move_base_plugin
{
    class DiffDriveClientPlugin : public nav_core::BaseGlobalPlanner
    {
        public:
            DiffDriveClientPlugin();
            DiffDriveClientPlugin(std::string name, costmap_2d::Costmap2DROS * costmap_ros);
            void initialize(std::string name, costmap_2d::Costmap2DROS * costmap_ros);
            bool makePlan(const geometry_msgs::PoseStamped & start, const geometry_msgs::PoseStamped & goal, std::vector<geometry_msgs::PoseStamped> & plan);
            size_t ToIndex(float x, float y);
            void FromIndex(size_t index, int & x, int & y);
            void FromGridToWorld(float & x, float & y);
            void FromWorldToGrid(float & x, float & y);
            bool InGridMapBounds(float & x, float & y);

        private:
            costmap_2d::Costmap2DROS * costmap_ros_;
            costmap_2d::Costmap2D * costmap_;
            bool initialized;
            float origin_x;
            float origin_y;
            float resolution;
            bool path_at_node_center = false;
            float node_center_offset = 0;
            int width;
            int height;
            int map_size;
            ros::ServiceClient makeplan_client;
            void publishPlan(const std::vector<geometry_msgs::PoseStamped> & path);
            ros::Publisher plan_pub;

    };
}
#endif
