#include <pluginlib/class_list_macros.h>
#include <diff_drive_client_plugin.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>

PLUGINLIB_EXPORT_CLASS(diff_drive_move_base_plugin::DiffDriveClientPlugin, nav_core::BaseGlobalPlanner)

namespace diff_drive_move_base_plugin
{
    DiffDriveClientPlugin::DiffDriveClientPlugin()
    {
        initialized = false;
    }

    DiffDriveClientPlugin::DiffDriveClientPlugin(std::string name, costmap_2d::Costmap2DROS * costmap_ros)
    {
        initialized = false;
        initialize(name, costmap_ros);
    }

    void DiffDriveClientPlugin::initialize(std::string name, costmap_2d::Costmap2DROS * costmap_ros)
    {
        if(!initialized)
        {
            ros::NodeHandle private_nh("~/" + name);
            costmap_ros_ = costmap_ros;
            costmap_ = costmap_ros->getCostmap();
            origin_x = costmap_->getOriginX();
            origin_y = costmap_->getOriginY();
            width = costmap_->getSizeInCellsX();
            height = costmap_->getSizeInCellsY();
            resolution = costmap_->getResolution();
            map_size = width * height;
            ROS_INFO("The Origin x = %lf, Origin y = %lf, Width = %d, Height= %d.",origin_x,origin_y,width,height);
            makeplan_client = private_nh.serviceClient<diff_drive_msgs::diff_drive_path_planning>("make_plan");
            makeplan_client.waitForExistence();
            plan_pub = private_nh.advertise<nav_msgs::Path>("plan", 10);
            path_at_node_center = true;
            if (path_at_node_center)
            {
                node_center_offset = resolution / 2;
            }
            initialized = true;
        }
    }

    bool DiffDriveClientPlugin::makePlan(const geometry_msgs::PoseStamped & start, const geometry_msgs::PoseStamped & goal, std::vector<geometry_msgs::PoseStamped> & plan)
    {
        plan.clear();
        std::vector<int> costmap(map_size);
        for(size_t idx = 0; idx < map_size; ++idx)
        {
            int x, y;
            x = idx % width;
            y = std::floor(idx/width);
            costmap.at(idx) = static_cast<int>(costmap_->getCost(x,y));
        }

        float start_x = start.pose.position.x;
        float start_y = start.pose.position.y;
        float goal_x = goal.pose.position.x;
        float goal_y = goal.pose.position.y;

        size_t start_index = 0;
        size_t goal_index = 0;

        if (InGridMapBounds(start_x, start_y) && InGridMapBounds(goal_x, goal_y))
        {
            FromWorldToGrid(start_x, start_y);
            FromWorldToGrid(goal_x, goal_y);

            start_index = ToIndex(start_x, start_y);
            goal_index = ToIndex(goal_x, goal_y);
        }
        else
        {
            ROS_WARN("Start or Goal position outsided of the map's boundaries");
            return false;
        }
        diff_drive_msgs::diff_drive_path_planning makeplan;
        makeplan.request.costmap_2D = costmap;
        makeplan.request.start = start_index;
        makeplan.request.goal = goal_index;
        makeplan.request.width = width;
        makeplan.request.height = height;

        makeplan_client.call(makeplan);

        std::vector<int> index_plan = makeplan.response.plan;
        ROS_DEBUG("Number of points: %d", unsigned(index_plan.size()));

        if (index_plan.size())
        {
            index_plan.insert(index_plan.begin(), start_index);
            index_plan.push_back(goal_index);
            for(int p : index_plan)
            {
                int x, y;
                FromIndex(p, x, y);
                float x_path = static_cast<float>(x);
                float y_path = static_cast<float>(y);

                FromGridToWorld(x_path, y_path);
                geometry_msgs::PoseStamped position;
                position.header.frame_id = start.header.frame_id;
                position.pose.position.x = x_path;
                position.pose.position.y = y_path;
                position.pose.orientation.x = 0;
                position.pose.orientation.y = 0;
                position.pose.orientation.z = 0;
                position.pose.orientation.w = 0;
                plan.push_back(position);
            }
            plan.push_back(goal);
            publishPlan(plan);
            return true;
        }
        else
        {
            return false;
        }
    }
    void DiffDriveClientPlugin::publishPlan(const std::vector<geometry_msgs::PoseStamped> & path)
    {
        nav_msgs::Path gui_path;
        gui_path.poses.resize(path.size());
        gui_path.header.frame_id = "map";
        gui_path.header.stamp = ros::Time::now();
        for(unsigned int i = 0; i < path.size(); i++)
        {
            gui_path.poses[i] = path[i];
        }
        plan_pub.publish(gui_path);
    }

    size_t DiffDriveClientPlugin::ToIndex(float x, float y)
    {
        return y * width + x;
    }

    void DiffDriveClientPlugin::FromIndex(size_t index, int & x, int & y)
    {
        x = index % width;
        y = std::floor(index / width);
    }

    void DiffDriveClientPlugin::FromWorldToGrid(float & x, float & y)
    {
        x = static_cast<size_t>((x - origin_x) / resolution);
        y = static_cast<size_t>((y - origin_y) / resolution);   
    }

    void DiffDriveClientPlugin::FromGridToWorld(float & x , float & y)
    {
        x = x * resolution + origin_x + node_center_offset;
        y = y + resolution + origin_y + node_center_offset;
    }

    bool DiffDriveClientPlugin::InGridMapBounds(float & x, float & y)
    {
        if (x < origin_x || y < origin_y || x > origin_x + (width * resolution) || y > origin_y + (height * resolution))
        {
            return false;
        }
        return true;
    }
};