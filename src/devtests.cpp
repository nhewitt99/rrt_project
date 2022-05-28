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
#include <visualization_msgs/Marker.h>
#include <Eigen/Geometry>

#include <algorithm>
#include <utility>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>

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
        double dist = nodeDistance(n, vec[i]);
        if (min < 0 || dist < min)
        {
            min = dist;
            ret = int(i);
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

    // Store nodes
    std::vector<Node> nodes;

    // Set up types for graph
    struct Vertex {Node* ptr;};
    typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, Vertex, double> graph_t;
    typedef boost::graph_traits<graph_t>::vertex_descriptor vertex_t;
    typedef boost::graph_traits<graph_t>::edge_descriptor edge_t;
    graph_t G;

    // Set up message to publish lines
    visualization_msgs::Marker line_list;
    line_list.header.frame_id = "panda_link0";
    line_list.id = 0;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.scale.x = 0.01;
    line_list.color.g = 1.0;
    line_list.color.a = 1.0;

    ros::Publisher state_pub = n.advertise<moveit_msgs::DisplayRobotState>("display_robot_state_test", 1000);
    ros::Publisher graph_pub = n.advertise<visualization_msgs::Marker>("graph_lines", 1000);
    ros::Rate loop_rate(30);
    while (ros::ok())
    {
        line_list.header.stamp = ros::Time::now();

        // Pick a random configuration
        kinematic_state->setToRandomPositions(joint_model_group);
        Eigen::Affine3d end_effector = kinematic_state->getGlobalLinkTransform("panda_link8");
        geometry_msgs::Pose ee_pose = tf2::toMsg(end_effector);

        ROS_INFO("Point at %f, %f, %f", ee_pose.position.x, ee_pose.position.y, ee_pose.position.z);

        Node* thisNodePtr = new Node(*kinematic_state, ee_pose);
        nodes.push_back(*thisNodePtr);
        Vertex thisVertex = {thisNodePtr};

        double min = -1.0;
        graph_t::vertex_iterator v, vend, vclosest;
        for (boost::tie(v, vend) = boost::vertices(G); v != vend; ++v)
        {
            Vertex otherVertex = G[*v];
            Node otherNode = *(otherVertex.ptr);

            double thisxyz[3];
            double otherxyz[3];
            thisNodePtr->fillCartesian(thisxyz);
            otherNode.fillCartesian(otherxyz);
            ROS_INFO("This: %f, %f, %f", thisxyz[0], thisxyz[1], thisxyz[2]);
            ROS_INFO("Other: %f, %f, %f", otherxyz[0], otherxyz[1], otherxyz[2]);

            double dist = nodeDistance(*thisNodePtr, otherNode);
            if (min < 0 || dist < min)
            {
                min = dist;
                vclosest = v;
            }
        }

        if (min < 0)
        {
            ROS_INFO("This is the first node!");
            vertex_t thisVertexDesc = boost::add_vertex(thisVertex, G);
        }
        else
        {
            ROS_INFO("Adding an edge with distance %f!", min);
            vertex_t otherVertexDesc = *vclosest;
            vertex_t thisVertexDesc = boost::add_vertex(thisVertex, G);
            boost::add_edge(thisVertexDesc, otherVertexDesc, min, G);

            // Reclaim closest node
            Vertex otherVertex = G[*vclosest];
            Node otherNode = *(otherVertex.ptr);

            // Visualize this edge
            geometry_msgs::Point thisPoint = thisNodePtr->getPose().position;
            geometry_msgs::Point otherPoint = otherNode.getPose().position;
            line_list.points.push_back(thisPoint);
            line_list.points.push_back(otherPoint);
        }

//        // Find closest existing node
//        int neighbor = closestIndex(thisNode, nodes);
//        if (neighbor == -1)
//        {
//            ROS_INFO("This is the first node!");
//        }
//        else
//        {
//            Node otherNode = nodes[neighbor];
//            ROS_INFO("This node's neighbor is %d with distance %f", neighbor, nodeDistance(thisNode, otherNode));
//        }
//        nodes.push_back(thisNode);

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
        state_pub.publish(display_msg);
        graph_pub.publish(line_list);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
