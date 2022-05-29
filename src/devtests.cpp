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
        moveit::core::RobotStatePtr getStatePtr(void)
        {
            // Return a pointer to a copy of the state (don't ask why)
            moveit::core::RobotStatePtr ret(new moveit::core::RobotState(state));
            return ret;
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


// Cartesian distance between two nodes
// TODO: distance in c-space instead
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


// Set up types for graph
struct Vertex {Node* ptr;};
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, Vertex, double> graph_t;
typedef boost::graph_traits<graph_t>::vertex_descriptor vertex_t;
typedef boost::graph_traits<graph_t>::edge_descriptor edge_t;


// Return distance to closest vertex, update vclosest to point to it
double findClosestVertex(graph_t G, Node* thisNodePtr, graph_t::vertex_iterator &vclosest)
{
    double min = -1.0;
    graph_t::vertex_iterator v, vend;
    for (boost::tie(v, vend) = boost::vertices(G); v != vend; ++v)
    {
        Vertex otherVertex = G[*v];
        Node otherNode = *(otherVertex.ptr);

        double thisxyz[3];
        double otherxyz[3];
        thisNodePtr->fillCartesian(thisxyz);
        otherNode.fillCartesian(otherxyz);
//            ROS_INFO("This: %f, %f, %f", thisxyz[0], thisxyz[1], thisxyz[2]);
//            ROS_INFO("Other: %f, %f, %f", otherxyz[0], otherxyz[1], otherxyz[2]);

        double dist = nodeDistance(*thisNodePtr, otherNode);
        if (min < 0 || dist < min)
        {
            min = dist;
            vclosest = v;
        }
    }
    return min;
};


// Boilerplate for a line_list marker
visualization_msgs::Marker initLineList()
{
    visualization_msgs::Marker line_list;
    line_list.header.frame_id = "panda_link0";
    line_list.id = 0;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.scale.x = 0.01;
    line_list.color.g = 1.0;
    line_list.color.a = 1.0;
    return line_list;
};


// Add an edge between two nodes to an existing line_list marker
void updateLineList(visualization_msgs::Marker* m, Node* node1ptr, Node* node2ptr)
{
    m->header.stamp = ros::Time::now();

    geometry_msgs::Point point1 = node1ptr->getPose().position;
    geometry_msgs::Point point2 = node2ptr->getPose().position;
    m->points.push_back(point1);
    m->points.push_back(point2);
};


// Check whether this one robotstate collides (with self or environment) in the planning scene
bool checkCollisionOnce(std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> psm,
                        moveit::core::RobotStatePtr kinematic_state)
{
    // Query classes
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
//    collision_request.verbose = true;

    planning_scene_monitor::LockedPlanningSceneRO(psm)->checkCollision(collision_request, collision_result, *kinematic_state);

    return collision_result.collision;
};


// Check whether at least one state from a list of states collides in the planning scene
bool checkCollisionMany(std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> psm,
                        std::vector<moveit::core::RobotStatePtr> kinematic_state_vec)
{
    // Query classes
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;

    // Get a readonly copy of planning scene
    auto locked_psm = planning_scene_monitor::LockedPlanningSceneRO(psm);

    for (auto kinematic_state : kinematic_state_vec)
    {
        locked_psm->checkCollision(collision_request, collision_result, *kinematic_state);

        // If at least one collides, return true
        if (collision_result.collision) return true;

        // Reset query for next check
        collision_result.clear();
    }

    return false;
};


// Generate interpolated states between two states
std::vector<moveit::core::RobotStatePtr> interpolateStates(
                                                const moveit::core::RobotModelConstPtr& kinematic_model,
                                                moveit::core::RobotStatePtr state1,
                                                moveit::core::RobotStatePtr state2, int n=10)
{
    std::vector<moveit::core::RobotStatePtr> ret;

    // Get joint parameters from model
    const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("panda_arm");

    // Get joint values for start and end
    std::vector<double> current_values;
    std::vector<double> end_values;
    state1->copyJointGroupPositions(joint_model_group, current_values);
    state2->copyJointGroupPositions(joint_model_group, end_values);

    // Calculate increments
    std::vector<double> increments;
    for (int i = 0; i < current_values.size(); i++)
    {
        increments.push_back((current_values[i] - end_values[i]) / n);
    }

    // Generate states
    for (int i = 0; i < n; i++)
    {
        // Create new state
        moveit::core::RobotStatePtr new_state(new moveit::core::RobotState(kinematic_model));
        new_state->setToDefaultValues();  // without this, fingers are at like 10^243 and crash rviz

        // Set joints to next value
        for (int j = 0; j < current_values.size(); j++)
        {
            current_values[j] = current_values[j] + increments[j];
        }
        new_state->setJointGroupPositions(joint_model_group, current_values);
        new_state->enforceBounds();  // just in case

        // Add to vector
        ret.push_back(new_state);
    }

    return ret;
};


// Check if an edge is valid between two nodes
bool edgeValid(std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> psm,
                const moveit::core::RobotModelConstPtr& kinematic_model, Node* node1, Node* node2)
{
    auto state1 = node1->getStatePtr();
    auto state2 = node2->getStatePtr();
    auto inter_vec = interpolateStates(kinematic_model, state1, state2);
    return !checkCollisionMany(psm, inter_vec);
};


// Convert a kinematic state to a displayable message type
moveit_msgs::DisplayRobotState displayMsgFromKin(moveit::core::RobotStatePtr kinematic_state)
{
        moveit_msgs::RobotState state_msg;
        moveit::core::robotStateToRobotStateMsg(*kinematic_state, state_msg);
        moveit_msgs::DisplayRobotState display_msg;
        display_msg.state = state_msg;
        return display_msg;
}


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

    // Get joint information from model
    const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("panda_arm");
    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

    // Store nodes
    std::vector<Node> nodes;

    // Init graph
    graph_t G;

    // Set up message to publish lines
    auto line_list = initLineList();

    ros::Publisher state_pub = n.advertise<moveit_msgs::DisplayRobotState>("display_robot_state_test", 1000);
    ros::Publisher graph_pub = n.advertise<visualization_msgs::Marker>("graph_lines", 1000);
    ros::Rate loop_rate(100);
    int count = 1;
    while (ros::ok())
    {

        // Pick a random configuration
        moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
        kinematic_state->setToDefaultValues();  // without this, fingers are at like 10^243 and crash rviz
        kinematic_state->setToRandomPositions(joint_model_group);
        Eigen::Affine3d end_effector = kinematic_state->getGlobalLinkTransform("panda_link8");
        geometry_msgs::Pose ee_pose = tf2::toMsg(end_effector);

        // Build a node from this configuration, package into a vertex
        Node* thisNodePtr = new Node(*kinematic_state, ee_pose);
        Vertex thisVertex = {thisNodePtr};

        // Find the closest existing vertex to this one
        graph_t::vertex_iterator vclosest;
        double min = findClosestVertex(G, thisNodePtr, vclosest);

        if (min < 0)
        {
            // Check whether node is valid
            if (!checkCollisionOnce(psm, kinematic_state))
            {
                ROS_INFO("This is the first node!");
                ROS_INFO("Point at %f, %f, %f", ee_pose.position.x, ee_pose.position.y, ee_pose.position.z);
                nodes.push_back(*thisNodePtr);

                vertex_t thisVertexDesc = boost::add_vertex(thisVertex, G);
            }
            else
            {
                ROS_INFO("This node was invalid!");
            }
        }
        else
        {
            // Back out a node pointer from closest vertex
            Vertex otherVertex = G[*vclosest];
            Node* otherNodePtr = otherVertex.ptr;

            // Check whether an edge can be made
            if (edgeValid(psm, kinematic_model, thisNodePtr, otherNodePtr))
            {
                ROS_INFO("Adding an edge with distance %f!", min);
                nodes.push_back(*thisNodePtr);

                vertex_t otherVertexDesc = *vclosest;
                vertex_t thisVertexDesc = boost::add_vertex(thisVertex, G);
                boost::add_edge(thisVertexDesc, otherVertexDesc, min, G);

                // Visualize edge
                updateLineList(&line_list, thisNodePtr, otherNodePtr);
            }
            else
            {
                ROS_INFO("The closest edge was invalid!");
            }
        }

        // Attempt to publish new state
        if (count % 10 == 0)
        {
            state_pub.publish(displayMsgFromKin(kinematic_state));
            graph_pub.publish(line_list);
        }
        ros::spinOnce();
        loop_rate.sleep();
        count++;
    }
    return 0;
}
