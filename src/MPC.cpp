#include "custom_mpc_local_planner/MPC.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(custom_mpc_local_planner::MPC, nav_core::BaseLocalPlanner)

namespace custom_mpc_local_planner{

MPC::MPC() : costmap_ros_(NULL), tf_(NULL), initialized_(false) {}

MPC::MPC(std::string name, tf2_ros::Buffer* tf,
                           costmap_2d::Costmap2DROS* costmap_ros)
    : costmap_ros_(NULL), tf_(NULL), initialized_(false)
{
    initialize(name, tf, costmap_ros);
}

MPC::~MPC() {}

// Take note that tf::TransformListener* has been changed to tf2_ros::Buffer* in ROS Noetic
void MPC::initialize(std::string name, tf2_ros::Buffer* tf,
                              costmap_2d::Costmap2DROS* costmap_ros)
{
    if(!initialized_)
    {
        tf_ = tf;
        costmap_ros_ = costmap_ros;
        initialized_ = true;
    }
    
}

bool MPC::setPlan(
    const std::vector<geometry_msgs::PoseStamped>& orig_global_plan
)
{
    if(!initialized_)
    {
        ROS_ERROR("This planner has not been initialized");
        return false;
    }
    // You set your mini global plan
    return true;
}

bool MPC::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
    if(!initialized_)
    {
        ROS_ERROR("This planner has not been initialized");
        return false;
    }
    /*
    1. Using mini global plan, current robot pose(x,y,yaw), current robot velocity, and uk(control input applied to robot at time step k),
    2. We call mpc.solve() function to solve the cost function and give us a vector of control commands from time step k+1 to k+f (where f=prediction horizon)
    3. We apply the control command at 0th position of the vector (i.e. u(k+1)) i.e. we convert it to cmd_vel.
    4. We use the entire vector of control commands and the kinematic model of robot to predict the local plan(which we publish for visualization)
    5. Everything is reset for next cycle/time step
    */
    return true;
}

bool MPC::isGoalReached()
{
    if(!initialized_)
    {
        ROS_ERROR("This planner has not been initialized");
        return false;
    }
    return false;
}
}