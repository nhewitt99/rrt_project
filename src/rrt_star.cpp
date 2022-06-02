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

#include <algorithm>
#include <utility>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>

#include <moveit_msgs/RobotState.h>
#include <moveit_msgs/DisplayRobotState.h>

#include <tf2_eigen/tf2_eigen.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Geometry>

using namespace std;

const std::vector<double> START_JOINTS = {0.8007, 0.6576, 0.8774, -0.8879, -0.5002, 1.3726, 2.2410};
const std::vector<double> GOAL_JOINTS = {-0.5265, 1.0870, 0.2671, -1.6648, 2.6357, 0.4429, -0.8478};

// Boilerplate random number generation stuff
std::uniform_real_distribution<double> uniform(0, 1);
std::default_random_engine rng(time(NULL));

typedef vector<double> Joints;
class Node
{
    public:
        Node(moveit::core::RobotState _state, geometry_msgs::Pose _ee_pose, int i, const moveit::core::RobotModelConstPtr& kinematic_model):
            state(_state), ee_pose(_ee_pose)
        {
            x = ee_pose.position.x;
            y = ee_pose.position.y;
            z = ee_pose.position.z;
            id = i;
            joints = getJointsFromRobotState(_state, kinematic_model);
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
        int getId()
        {
            return id;
        }
        Joints getJoints()
        {
            return joints;
        }
    private:
        Joints getJointsFromRobotState(moveit::core::RobotState _state, const moveit::core::RobotModelConstPtr& kinematic_model)
        {
            // Get joint parameters from model
            const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("panda_arm");

            // Get joint values for model
            Joints j;
            _state.copyJointGroupPositions(joint_model_group, j);
            return j;
        }
        moveit::core::RobotState state;
        geometry_msgs::Pose ee_pose;
        double x;
        double y;
        double z;
        Joints joints;
        int id;
};

struct Vertex {Node* ptr;};

typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, Vertex, double> graph_t;
typedef boost::graph_traits<graph_t>::vertex_descriptor vertex_t;
typedef boost::graph_traits<graph_t>::edge_descriptor edge_t;

class RRTstar
{
    public:
        RRTstar(double r, Vertex start_vertex)
        {
            // Create graph with root node
            vertex_t start_vertex_desc = addVertex(start_vertex);

            // Assign the start vertex a cost of 0
            Cost.push_back(0.0);

            // First vertex is its own parent
            Parents.push_back(start_vertex_desc);

            // Set radius
            radius = r;
        }
        graph_t step(Vertex new_vertex)
        {
            // Input node is random, not in collision, and extended from an existing node

            // Add a new random position in joint space as a vertex to the graph
            // This new vertex is guaranteed to be within specified radius of an existing vertex
            vertex_t new_node_desc = addVertex(new_vertex);

            // Temporarily make new node its own parent
            Parents.push_back(new_node_desc);

            // Get all neighbors within the specified radius
            vector<vertex_t> neighbors = getNeighbors(new_node_desc);

            // Get the nearest neighbor
            vertex_t nearest_neighbor = getNearestNeighbor(new_node_desc, neighbors);

            // Assign a cost to the new vertex based on its distance to its nearest neighbor
            // Note this is only the cost to get from the nearest neighbor to the new vertex,
            // NOT the overall cost to get to the new vertex
            Cost.push_back(calculateCost(new_node_desc, nearest_neighbor));

            // Rewire neighbors so that the new node is now their parent node if this would result in a lower cost for that node
            rewireNeighbors(new_node_desc, neighbors, nearest_neighbor);

            // Link together new vertex with its nearest neighbor
            linkNewVertex(new_node_desc, nearest_neighbor);

            int num_edges = 0;
            graph_t::edge_iterator e, e_end;
            for (boost::tie(e,e_end) = boost::edges(G); e != e_end; ++e)
            {
                num_edges = num_edges + 1;
            }
            int num_vertices = 0;
            graph_t::vertex_iterator v, v_end;
            for (boost::tie(v, v_end) = boost::vertices(G); v != v_end; ++v)
            {
                num_vertices = num_vertices + 1;
            }
            cout << "num_edges: " << num_edges << " | num_vertices: " << num_vertices << endl;

            return G;
        }
        int getNewNodeId()
        {
            return Cost.size();
        }
        graph_t getGraph()
        {
            return G;
        }
        pair<vertex_t, double> getClosestVertex(Node* new_node_ptr)
        {
            cout << "getClosestVertex" << endl;
            double min_distance = -1.0;
            vertex_t v_closest;
            graph_t::vertex_iterator v, v_end;
            int count = 0;
            for (boost::tie(v, v_end) = boost::vertices(G); v != v_end; ++v)
            {
                cout << "count: " << count << endl;
                count = count +1;

                Vertex other_vertex = G[*v];
                Node other_node = *(other_vertex.ptr);

                double distance = calculateJointDistance(new_node_ptr->getJoints(), other_node.getJoints());

                cout << "num joints: " << new_node_ptr->getJoints().size() << " | " << other_node.getJoints().size() << endl;

                bool same_joints = true;
                for (int i = 0; i<new_node_ptr->getJoints().size(); i++)
                {
                    cout << new_node_ptr->getJoints()[i] << " | " << other_node.getJoints()[i] << endl;
                    if (new_node_ptr->getJoints()[i] != other_node.getJoints()[i])
                    {
                        same_joints = false;
                    }
                }

                if (same_joints)
                {
                    cout << "New node has same joints as existing node" << endl;
                }

                // cout << "count: " << count << endl;
                // for (double j : other_node.getJoints())
                // {
                //     cout << "j: " << j << endl;
                // }

                if (min_distance < 0 || distance < min_distance)
                {
                    min_distance = distance;
                    v_closest = *v;
                }
            }
            cout << "v_closest id: " << G[v_closest].ptr->getId() << endl;
            return make_pair(v_closest, min_distance);
        }
    private:
        // Graph for storing search results
        graph_t G;

        // Parent for each vertex. Vertex id is index.
        vector<vertex_t> Parents;

        // Cost for each vertex. Vertex id is index.
        vector<double> Cost;

        // Radius for how far new nodes can be
        double radius;

        // Add a new node as a vertex to the graph. Do not link it to any other vertices.
        vertex_t addVertex(Vertex new_vertex)
        {
            cout << "addVertex()" << endl;
            // Node* node_ptr = &new_node;
            // Vertex vertex = {node_ptr};
            return boost::add_vertex(new_vertex, G);
        }

        double calculateDeltaTheta(double a, double b)
        {
            if (a==b)
            {
                return 0;
            }
            else
            {
                double dp;
                if (a > b)
                {
                    dp = a - 2*3.14159;
                }
                else
                {
                    dp = a + 2*3.14159;
                }
                double d0 = a - b;
                double d1 = dp - b;
                if (abs(d0) <= abs(d1))
                {
                    return d0;
                }
                else
                {
                    return d1;
                }
            }
        }

        // TODO: Account for theta circular wrap-around
        double calculateJointDistance(Joints A, Joints B)
        {
            double squared_sum = 0;
            for (int i=0; i < A.size(); i++)
            {

                // # Case 0: Desired heading is current heading
                // if desired_heading == current_heading:
                //     delta_heading = 0
                // else:
                //     # Case 1: Desired heading greater than current heading
                //     if desired_heading > current_heading:
                //         desired_heading_prime = desired_heading - 2*np.pi

                //     # Case 2: Desired heading less than current heading
                //     else:
                //         desired_heading_prime = desired_heading + 2*np.pi

                //     delta0 = desired_heading - current_heading
                //     delta1 = desired_heading_prime - current_heading
                //     which_delta = np.argmin([np.abs(delta0), np.abs(delta1)])
                //     delta_heading = np.array([delta0, delta1])[which_delta]

                // double delta_theta = (A[i] - B[i]);
                double delta_theta = calculateDeltaTheta(A[i], B[i]);

                squared_sum = squared_sum + pow(delta_theta, 2);
            }
            return sqrt(squared_sum);
        }

        double calculateCost(vertex_t A, vertex_t B)
        {
            return calculateJointDistance(G[A].ptr->getJoints(), G[B].ptr->getJoints());
        }

        // Get all the vertices within
        vector<vertex_t> getNeighbors(vertex_t new_node_desc)
        {
            cout << "getNeighbors()" << endl;
            // Unpack new node
            Vertex new_node_vertex = G[new_node_desc];
            Joints new_node_joints = new_node_vertex.ptr->getJoints();

            // Initialize vector for storing neighbors
            vector<vertex_t> neighbors;

            // Iterate through every vertex in the graph
            graph_t::vertex_iterator v, v_end;
            int n_count = 0;
            for (boost::tie(v, v_end) = boost::vertices(G); v != v_end; ++v)
            {
                // cout << "id: " << G[*v].ptr->getId() << endl;
                // cout << "n_count: " << n_count << endl;
                n_count = n_count + 1;
                // Make sure current vertex i is not the new vertex
                if (G[*v].ptr->getId() != new_node_vertex.ptr->getId())
                {
                    // Unpack current vertex i
                    Vertex i_vertex = G[*v];
                    Joints i_joints = i_vertex.ptr->getJoints();

                    // Calculate the distance between these two nodes in joint space
                    double distance = calculateJointDistance(new_node_joints, i_joints);

                    // cout << "distance " << distance << "radius " << radius << endl;

                    // Vertex is a neighbor if within radius
                    if (distance <= radius)
                    {
                        // cout << "distance<=radius" <<endl;
                        // Add vertex to vector of neighbors
                        neighbors.push_back(*v);
                    }
                }
            }

            cout << "Joints: " << endl;
            for (double& j: G[new_node_desc].ptr->getJoints())
            {
                cout << j << endl;
            }

            // cout << "neighbors.size()" << neighbors.size() << endl;
            return neighbors;
        }

        vertex_t getNearestNeighbor(vertex_t new_node, vector<vertex_t> neighbors)
        {
            cout << "getNearestNeighbor()" << endl;

            // Initialze variables for storing nearest neighbor info
            double min_distance = -1.0;
            // cout << "Set min_distance" << endl;
            vertex_t nearest_neighbor;
            // cout << "Set nearest neighbor" << endl;

            // cout << "neighbors.size(): " << neighbors.size() << endl;

            // Iterate through all neighbors
            int n_count = 0;
            for (vertex_t& neighbor : neighbors)
            {
                // cout << "n_count: " << n_count << endl;
                // Calculate distance to neighbor
                double distance = calculateJointDistance(G[new_node].ptr->getJoints(), G[neighbor].ptr->getJoints());

                // If a nearest neighbor has not been specified
                // OR if this neighbor is closer than the current nearest neighbor
                if (min_distance < 0 || distance < min_distance)
                {
                    // cout << "Set nearest_neighbor" << endl;
                    // Save distance
                    min_distance = distance;
                    // Set as new nearest neighbor
                    nearest_neighbor = neighbor;
                }
            }
            cout << "Joints: " << endl;
            for (double& j: G[new_node].ptr->getJoints())
            {
                cout << j << endl;
            }
            return nearest_neighbor;
        }

        void linkNewVertex(vertex_t new_vertex, vertex_t existing_vertex)
        {
            cout << "linkNewVertex" << endl;
            if (G[new_vertex].ptr->getJoints()[0] == G[existing_vertex].ptr->getJoints()[0])
            {
                cout << "New vertex and existing vertex match. Purposefully causing segfault." << endl;
                cout << "Joints: " << endl;
                for (double& j: G[new_vertex].ptr->getJoints())
                {
                    cout << j << endl;
                }
                Joints j;
                double i = j[0];
            }

            // Assign parent to new vertex
            Parents[Id(new_vertex)] = existing_vertex;
            boost::add_edge(new_vertex, existing_vertex, Cost[G[new_vertex].ptr->getId()], G);
        }

        void deLinkVertices(vertex_t A, vertex_t B)
        {
            cout << "deLinkVertices" << endl;
            boost::remove_edge(A, B, G);
        }

        void rewireNeighbors(vertex_t new_vertex, vector<vertex_t> neighbors, vertex_t nearest_neighbor)
        {
            cout << "rewireNeighbors()" << endl;
            for (vertex_t& neighbor: neighbors)
            {
                if (Id(neighbor) != Id(nearest_neighbor))
                {
                    // If jumping from the new vertex to this neighbor is cheaper than jumping from the
                    // neighbor's parent to the neighbor, then the new vertex is now the parent.
                    // Congrats and good luck on raising that vertex to be a good upstanding citizen
                    // cout << "Cost.size(): " << Cost.size() << " | Id(new_vertex): " << Id(new_vertex) << " | Id(neighbor): " << Id(neighbor) << endl;
                    if (Cost[Id(new_vertex)] + calculateCost(new_vertex, neighbor) < Cost[Id(neighbor)])
                    {
                        cout << "if true" << endl;
                        // Set new cost for the neighbor
                        Cost[Id(neighbor)] = Cost[Id(new_vertex)] + calculateCost(new_vertex, neighbor);
                        // cout << "set cost" << endl;

                        // cout << "Parents.size() " << Parents.size() << endl;
                        // Delink the neighbor from its original parent
                        deLinkVertices(neighbor, Parents[Id(neighbor)]);

                        // Link the neighbor as a "new" vertex to the actually new vertex
                        linkNewVertex(neighbor, new_vertex);
                    }
                }
            }
        }

        int Id(vertex_t v)
        {
            return G[v].ptr->getId();
        }
};

// Boilerplate for a line_list marker
visualization_msgs::Marker initLineList()
{
    visualization_msgs::Marker line_list;
    line_list.header.frame_id = "panda_link0";
    line_list.id = 0;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.scale.x = 0.01;
    line_list.color.r = 1.0;
    line_list.color.g = 1.0;
    line_list.color.a = 1.0;
    return line_list;
};


// Add an edge between two nodes to an existing line_list marker
void updateLineList(visualization_msgs::Marker* m, Node* node1ptr, Node* node2ptr)
{
    cout << "updateLineList()" << endl;
    m->header.stamp = ros::Time::now();

    geometry_msgs::Point point1 = node1ptr->getPose().position;
    geometry_msgs::Point point2 = node2ptr->getPose().position;
    m->points.push_back(point1);
    m->points.push_back(point2);

    cout << point1 << endl;
    cout << point2 << endl;
};


// Build a line_list marker from a graph
visualization_msgs::Marker linesFromGraph(graph_t G)
{
    cout << "linesFromGraph()" << endl;
    // Init an empty list
    auto ret = initLineList();

    // Iterate over edges of graph (only the first N edges)
    int N = 300;
    int n = 0;
    auto es = boost::edges(G);
    for (auto edge_iter = es.first; edge_iter != es.second; ++edge_iter)
    {
        // Get two vertices of edge
        vertex_t v1 = boost::source(*edge_iter, G);
        vertex_t v2 = boost::target(*edge_iter, G);

        // Get nodes from vertices and add line
        Node* n1 = G[v1].ptr;
        Node* n2 = G[v2].ptr;
        if (n < N)
        {
            updateLineList(&ret, n1, n2);
            // n++;
        }
    }

    return ret;
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
}

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

// Check if an edge is valid between two nodes
bool edgeValid(std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> psm,
                const moveit::core::RobotModelConstPtr& kinematic_model, Node* node1, Node* node2)
{
    auto state1 = node1->getStatePtr();
    auto state2 = node2->getStatePtr();
    auto inter_vec = interpolateStates(kinematic_model, state1, state2);
    return !checkCollisionMany(psm, inter_vec);
};

// Generate a state that extends towards another in c-space, less than some length in rads
moveit::core::RobotStatePtr extend(const moveit::core::RobotModelConstPtr& kinematic_model,
                                   moveit::core::RobotStatePtr state1,
                                   moveit::core::RobotStatePtr state2, double max=0.45)
{
    // Get joint parameters from model
    const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("panda_arm");

    // Get joint values for start and end
    std::vector<double> current_values;
    std::vector<double> end_values;
    state1->copyJointGroupPositions(joint_model_group, current_values);
    state2->copyJointGroupPositions(joint_model_group, end_values);


    std::cout << current_values[0] << std::endl;
    std::cout << end_values[0] << std::endl;

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
    const moveit::core::RobotModelConstPtr& kinematic_model = psm->getRobotModel();
    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

    // Get joint information from model
    const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("panda_arm");
    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

    // Set up message to publish lines
    auto line_list = initLineList();

    ros::Publisher state_pub = n.advertise<moveit_msgs::DisplayRobotState>("display_robot_state_test", 1000);
    ros::Publisher graph_pub = n.advertise<visualization_msgs::Marker>("graph_lines", 1000);
    ros::Rate loop_rate(100);
    int count = 1;

    Vertex start_vertex;
    Vertex end_vertex;

    // Generate start vertex
    auto start_state = stateFromJoints(kinematic_model, START_JOINTS);
    geometry_msgs::Pose start_ee_pose = getStatePose(start_state);
    Node* start_node_ptr = new Node(*start_state, start_ee_pose, 0, kinematic_model);
    start_vertex = {start_node_ptr};

    // Generate end vertex
    auto end_state = stateFromJoints(kinematic_model, GOAL_JOINTS);
    geometry_msgs::Pose end_ee_pose = getStatePose(end_state);
    Node* end_node_ptr = new Node(*end_state, end_ee_pose, 0, kinematic_model);
    end_vertex = {end_node_ptr};

    double radius = 0.5;
    RRTstar rrt = RRTstar(radius, start_vertex);

    int add_count = 0;
    int ic = 0;
    bool p = false;
    while (ros::ok())
    // for (int i = 0; i < 100; i++)
    {
        // Pick a random configuration
        auto kinematic_state = randomState(kinematic_model);

        // With some probability, move towards goal instead
        // if (uniform(rng) < 0.999 && count > 1)
        // {
            // ROS_INFO("Attempting to extend to goal");
            // kinematic_state = stateFromJoints(kinematic_model, GOAL_JOINTS);
        // }
        auto ee_pose = getStatePose(kinematic_state);

        // Build a node from this configuration, package into a vertex
        Node* thisNodePtr = new Node(*kinematic_state, ee_pose, rrt.getNewNodeId(), kinematic_model);
        Vertex thisVertex = {thisNodePtr};
        vertex_t thisVertexDesc;  // Define here to keep in scope but don't init yet

        // Find the closest existing vertex to this one
        pair<vertex_t, double> closest_vertex_p = rrt.getClosestVertex(thisNodePtr);
        vertex_t closest_vertex = closest_vertex_p.first;
        double min = closest_vertex_p.second;
        cout << min << endl;

        // Back out a node pointer from closest vertex
        Vertex otherVertex = rrt.getGraph()[closest_vertex];
        Node* otherNodePtr = otherVertex.ptr;

        // Check whether an edge can be made
        if (edgeValid(psm, kinematic_model, thisNodePtr, otherNodePtr))
        {
            ROS_INFO("Adding an edge!");
            cout << "add_count: " << add_count << endl;
            add_count = add_count + 1;

            // Use extend to move in direction of new point
            auto extended_state = extend(kinematic_model, otherNodePtr->getStatePtr(), thisNodePtr->getStatePtr());
            auto ee_pose = getStatePose(extended_state);

            // Update the candidate node
            thisNodePtr = new Node(*extended_state, ee_pose, rrt.getNewNodeId(), kinematic_model);
            thisVertex = {thisNodePtr};

            // Add node to graph
            graph_t latest_graph = rrt.step(thisVertex);

            // Throttle publishing
            graph_pub.publish(linesFromGraph(latest_graph));
            ic++;
        }
        else
        {
            ROS_INFO("The closest edge was invalid!");
        }
    }

}
