#include <random>
#include <math.h>
#include <time.h>

#include <ros/ros.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <moveit_msgs/RobotState.h>
#include <moveit_msgs/DisplayRobotState.h>

#include <tf2_eigen/tf2_eigen.h>
#include <geometry_msgs/Pose.h>
#include <Eigen/Geometry>

class Node
{
    public:
        Node(moveit::core::RobotState _state, geometry_msgs::Pose _ee_pose):
            state(_state), ee_pose(_ee_pose)
        {
            x = ee_pose.position.x;
            y = ee_pose.position.y;
            z = ee_pose.position.z;
        }
        moveit::core::RobotState getState(void)
        {
            return state;
        }
        geometry_msgs::Pose getPose(void)
        {
            return ee_pose;
        }
        void fillCartesian(double* xyz)
        {
            xyz[0] = x;
            xyz[1] = y;
            xyz[2] = z;
        }
    private:
        moveit::core::RobotState state;
        geometry_msgs::Pose ee_pose;
        double x;
        double y;
        double z;
};


double nodeDistance(Node n1, Node n2)
{
    double pt1[3];
    double pt2[3];
    n1.fillCartesian(pt1);
    n2.fillCartesian(pt2);

    double x_d = pt1[0] - pt2[0];
    double y_d = pt1[1] - pt2[1];
    double z_d = pt1[2] - pt2[2];
    return sqrt(pow(x_d,2) + pow(y_d,2) + pow(z_d,2));
};


int closestIndex(Node n, std::vector<Node> vec)
{
    double min = -1.0;
    int ret = -1;
    for (std::size_t i = 0; i < vec.size(); i++)
    {
        double dist = nodeDistance(n, vec[i])
        if (min < 0 || dist < min)
        {
            min = dist;
            ret = i;
        }
    }
    return ret;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "devtests");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::NodeHandle n;

    // Get up to date planning scene
    auto psm = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
    psm->startSceneMonitor("/move_group/monitored_planning_scene");
    psm->requestPlanningSceneState("/get_planning_scene");

    // Construct a RobotModel by looking up the description on the parameter server
//    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
//    const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
    const moveit::core::RobotModelConstPtr& kinematic_model = psm->getRobotModel();
    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

    // Make a RobotState that maintains a configuration
    moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
    const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("panda_arm");
    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

    // Get currently stored joint state
    std::vector<double> joint_values;
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }

    bool ik = false;
    if (ik)
    {
        // Pick a random EE point in workspace
        std::default_random_engine generator;
        generator.seed(time(NULL));
        std::uniform_real_distribution<double> distribution(0.0, 1.0);
        double x = 2 * distribution(generator) - 1.0;  // -1 to 1
        double y = 2 * distribution(generator) - 1.0;  // -1 to 1
        double z = distribution(generator);  // 0 to 1

        // Build geometry msg of point
        geometry_msgs::Pose target_pose1;
        target_pose1.orientation.w = 1.0;
        target_pose1.position.x = x;
        target_pose1.position.y = y;
        target_pose1.position.z = z;
        ROS_INFO("Point at %f, %f, %f", x, y, z);

        // Try to find an IK solution
        double timeout = 0.1;
        bool found_ik = kinematic_state->setFromIK(joint_model_group, target_pose1, timeout);

        if (found_ik)
        {
            kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
            for (std::size_t i = 0; i < joint_names.size(); ++i)
            {
                ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
            }
        }
        else
        {
            ROS_INFO("No IK solution!");
        }
    }
    else
    {
        // Pick a random configuration
        kinematic_state->setToRandomPositions(joint_model_group);
        Eigen::Affine3d end_effector = kinematic_state->getGlobalLinkTransform("panda_link8");
        geometry_msgs::Pose ee_pose = tf2::toMsg(end_effector);

        ROS_INFO("Point at %f, %f, %f", ee_pose.position.x, ee_pose.position.y, ee_pose.position.z);
    }

    // Check proposed state for collisions
    collision_detection::CollisionRequest collision_request;
    collision_request.verbose = true;
    collision_detection::CollisionResult collision_result;

    // Because planning_scene is constructed from kinematic_model, this
    // request implicitly checks the current kinematic state?
    planning_scene_monitor::LockedPlanningSceneRO(psm)->checkCollision(collision_request, collision_result, *kinematic_state);

    ROS_INFO_STREAM("Collision result is " << (collision_result.collision ? "true": "false"));

    // Convert new state to a message
    moveit_msgs::RobotState state_msg;
    moveit::core::robotStateToRobotStateMsg(*kinematic_state, state_msg);
    moveit_msgs::DisplayRobotState display_msg;
    display_msg.state = state_msg;

    // Attempt to publish new state
    ros::Publisher state_pub = n.advertise<moveit_msgs::DisplayRobotState>("display_robot_state_test", 1000);
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        state_pub.publish(display_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
