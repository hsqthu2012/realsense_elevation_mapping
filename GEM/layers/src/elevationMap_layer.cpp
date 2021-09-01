#include <elevationMap_layer/elevationMap_layer.h>
#include <pluginlib/class_list_macros.h>

using costmap_2d::LETHAL_OBSTACLE;

namespace costmap_2d
{

ElevationMapLayer::ElevationMapLayer() {}

ElevationMapLayer::~ElevationMapLayer() {}

// Initialize layer
void ElevationMapLayer::onInitialize()
{
    ObstacleLayer::onInitialize();

    ros::NodeHandle nh("~/" + name_);

    std::string source_topic;
    nh.param<std::string>("source_topic", source_topic, std::string("/velodyne_points"));

    // subscriber for elevation map
    elevation_map_available_ = false;
    elevation_map_sub_ = nh.subscribe(source_topic, 10, &ElevationMapLayer::elevationMapCB, this);
}


// Elevation map callback
void ElevationMapLayer::elevationMapCB(const grid_map_msgs::GridMapConstPtr& msg)
{
    if (!elevation_map_available_)
    {
        grid_map::GridMapRosConverter::fromMessage(*msg, elevation_map_);
        elevation_map_available_ = true;
    }
}


// update bounds (base function in costmap class)
void ElevationMapLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
                                                double* min_x, double* min_y, double* max_x, double* max_y)
{
    if (rolling_window_)
        updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
    if (!enabled_)
        return;
    useExtraBounds(min_x, min_y, max_x, max_y);

    bool current = true;
    current = elevation_map_available_;
    current_ = true;

    // check if the elevation map is available
    if (elevation_map_available_)
    {
        grid_map::Matrix& data = elevation_map_["traver"];
        for (grid_map::GridMapIterator iterator(elevation_map_); !iterator.isPastEnd(); ++iterator)
        {
        
            const grid_map::Index gindex(*iterator);
            bool is_obstacle = (data(gindex(0), gindex(1)) <0.7);

            grid_map::Position pos;
            elevation_map_.getPosition(gindex, pos);
            double px = pos.x(), py = pos.y();

            // now we need to compute the map coordinates for the observation
            unsigned int mx, my;
            if (!worldToMap(px, py, mx, my))
            {
                ROS_DEBUG("Computing map coords failed");
                continue;
            }

            unsigned int index = getIndex(mx, my);

            costmap_[index] = is_obstacle ? LETHAL_OBSTACLE : FREE_SPACE;
            touch(px, py, min_x, min_y, max_x, max_y);
        }

        elevation_map_available_ = false;
    }

    updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}

} // namespace costmap_2d

PLUGINLIB_EXPORT_CLASS(costmap_2d::ElevationMapLayer, costmap_2d::Layer)
