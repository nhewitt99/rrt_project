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
#include <boost/graph/dijkstra_shortest_paths.hpp>


const std::vector<double> START_JOINTS = {0.8007, 0.6576, 0.8774, -0.8879, -0.5002, 1.3726, 2.2410};
const std::vector<double> GOAL_JOINTS = {-0.5265, 1.0870, 0.2671, -1.6648, 2.6357, 0.4429, -0.8478};

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
struct Vertex{Node* ptr;};
//struct EdgeProperties{double weight;};
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, Vertex, boost::property<boost::edge_weight_t, double>> graph_t;
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


// Build a line_list marker from a graph
visualization_msgs::Marker linesFromGraph(graph_t G)
{
    // Init an empty list
    auto ret = initLineList();

    // Iterate over edges of graph
    auto es = boost::edges(G);
    for (auto edge_iter = es.first; edge_iter != es.second; ++edge_iter)
    {
        // Get two vertices of edge
        vertex_t v1 = boost::source(*edge_iter, G);
        vertex_t v2 = boost::target(*edge_iter, G);

        // Get nodes from vertices and add line
        Node* n1 = G[v1].ptr;
        Node* n2 = G[v2].ptr;
        updateLineList(&ret, n1, n2);
    }

    return ret;
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


// Create a state pointer and initialize it randomly
moveit::core::RobotStatePtr randomState(const moveit::core::RobotModelConstPtr& kinematic_model)
{
    // Get joint parameters from model
    const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("panda_arm");

    moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();  // without this, fingers are at like 10^243 and crash rviz
    kinematic_state->setToRandomPositions(joint_model_group);

    return kinematic_state;
};


// Get the current state pointer from the planning scene
moveit::core::RobotStatePtr currentState(std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> psm)
{
    moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentState()));
    return kinematic_state;
};


// Build a state from a vector of joints
moveit::core::RobotStatePtr stateFromJoints(const moveit::core::RobotModelConstPtr& kinematic_model, std::vector<double> joints)
{
    // Get joint parameters from model
    const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("panda_arm");

    moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();  // without this, fingers are at like 10^243 and crash rviz

    kinematic_state->setJointGroupPositions(joint_model_group, joints);
    kinematic_state->enforceBounds();  // just in case

    return kinematic_state;
};


// Get a geometry Pose from a state
geometry_msgs::Pose getStatePose(moveit::core::RobotStatePtr kinematic_state)
{
        Eigen::Affine3d end_effector = kinematic_state->getGlobalLinkTransform("panda_link8");
        geometry_msgs::Pose ee_pose = tf2::toMsg(end_effector);
        return ee_pose;
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
        increments.push_back((end_values[i] - current_values[i]) / n);
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


// Generate a state that extends towards another in c-space, less than some length in rads
moveit::core::RobotStatePtr extend(const moveit::core::RobotModelConstPtr& kinematic_model,
                                   moveit::core::RobotStatePtr state1,
                                   moveit::core::RobotStatePtr state2, double max=0.5)
{
    // Get joint parameters from model
    const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("panda_arm");

    // Get joint values for start and end
    std::vector<double> current_values;
    std::vector<double> end_values;
    state1->copyJointGroupPositions(joint_model_group, current_values);
    state2->copyJointGroupPositions(joint_model_group, end_values);

    // Get difference
    std::vector<double> diff_values;
    double magnitude = 0;
    for (int i = 0; i < current_values.size(); i++)
    {
        double diff = end_values[i] - current_values[i];
        diff_values.push_back(diff);
        magnitude += pow(diff, 2);
    }
    magnitude = sqrt(magnitude);

    // Return if already less than max
    if (magnitude <= max)
    {
        return state2;
    }
    else
    {
        // Normalize and scale differences to max
        for (auto& diff : diff_values)
        {
            diff = max * diff / magnitude;
        }

        // Create new state by adding scaled differences to start
        moveit::core::RobotStatePtr new_state(new moveit::core::RobotState(kinematic_model));
        new_state->setToDefaultValues();  // without this, fingers are at like 10^243 and crash rviz

        // Set joints to next value
        for (int i = 0; i < current_values.size(); i++)
        {
            current_values[i] = current_values[i] + diff_values[i];
        }
        new_state->setJointGroupPositions(joint_model_group, current_values);
        new_state->enforceBounds();  // just in case

        return new_state;
    }
}


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


// Reverse a path for dijkstra
std::vector<vertex_t> getPath(const graph_t G, const std::vector<vertex_t>& pMap, const vertex_t& source, const vertex_t& destination)
{
    std::vector<vertex_t> path;
    vertex_t current = destination;
    while (current != source)
    {
        path.push_back(current);
        current = pMap[current];
    }
    path.push_back(source);
    return path;
}


// Dijkstra
std::vector<vertex_t> dijkstra(const graph_t& G, const vertex_t source, const vertex_t destination)
{
    const int numVertices = boost::num_vertices(G);
    std::vector<double> distances(numVertices);
    std::vector<vertex_t> pMap(numVertices);

    auto distanceMap = boost::predecessor_map(
            boost::make_iterator_property_map(pMap.begin(), boost::get(boost::vertex_index, G))).distance_map(
            boost::make_iterator_property_map(distances.begin(), boost::get(boost::vertex_index, G)));
//    auto distanceMap = boost::predecessor_map(&pMap[0]).distance_map(&distances[0]);

    boost::dijkstra_shortest_paths(G, source, distanceMap);
    return getPath(G, pMap, source, destination);
}


// Boilerplate random number generation stuff
std::uniform_real_distribution<double> uniform(0, 1);
std::default_random_engine rng(time(NULL));


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
    psm->startStateMonitor();

    // Construct a RobotModel by looking up the description on the parameter server
//    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
//    const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
    const moveit::core::RobotModelConstPtr& kinematic_model = psm->getRobotModel();
    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

    // Get joint information from model
    const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("panda_arm");
    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

    // Build start and goal states
    auto start_state = stateFromJoints(kinematic_model, START_JOINTS);
    auto goal_state = stateFromJoints(kinematic_model, GOAL_JOINTS);

    // Store nodes
    std::vector<Node> nodes;

    // Init graph
    graph_t G;

    // Set up message to publish lines
    auto line_list = initLineList();

    ros::Publisher state_pub = n.advertise<moveit_msgs::DisplayRobotState>("display_robot_state_test", 1000);
    ros::Publisher graph_pub = n.advertise<visualization_msgs::Marker>("graph_lines", 1000);
    ros::Publisher path_pub = n.advertise<visualization_msgs::Marker>("path_lines", 1000);
    ros::Rate loop_rate(100);
    int count = 1;

    vertex_t start_vertex;
    vertex_t end_vertex;

    while (ros::ok())
    {
        // Pick a random configuration
        auto kinematic_state = randomState(kinematic_model);

        // With some probability, move towards goal instead
        if (uniform(rng) < 0.05 && count > 1)
        {
            ROS_INFO("Attempting to extend to goal");
            kinematic_state = goal_state;
        }
        auto ee_pose = getStatePose(kinematic_state);

        // Build a node from this configuration, package into a vertex
        Node* thisNodePtr = new Node(*kinematic_state, ee_pose);
        Vertex thisVertex = {thisNodePtr};
        vertex_t thisVertexDesc;  // Define here to keep in scope but don't init yet

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
                start_vertex = thisVertexDesc;
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
                ROS_INFO("Adding an edge!");

                // Use extend to move in direction of new point
                auto extended_state = extend(kinematic_model, otherNodePtr->getStatePtr(), thisNodePtr->getStatePtr());
                auto ee_pose = getStatePose(extended_state);

                // Update the candidate node
                thisNodePtr = new Node(*extended_state, ee_pose);
                nodes.push_back(*thisNodePtr);
                thisVertex = {thisNodePtr};

                // Add edge
                vertex_t otherVertexDesc = *vclosest;
                thisVertexDesc = boost::add_vertex(thisVertex, G);
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
//            graph_pub.publish(line_list);
            graph_pub.publish(linesFromGraph(G));
        }

        // After graph is large try to run dijkstra
        if (count > 1000)
        {
            end_vertex = thisVertexDesc;
            auto path = dijkstra(G, start_vertex, end_vertex);
            ROS_INFO("%d", int(path.size()));

            auto path_list = initLineList();
            path_list.color.r = 1.0;
            path_list.color.b = 1.0;
            path_list.color.g = 0.0;
            path_list.scale.x = 0.03;
            for (int i = 0; i < path.size() - 1; i++)
            {
                updateLineList(&path_list, G[path[i]].ptr, G[path[i+1]].ptr);
            }
            path_pub.publish(path_list);
            break;
        }

        ros::spinOnce();
        loop_rate.sleep();
        count++;
    }

    return 0;
}
