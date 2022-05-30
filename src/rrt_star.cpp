#include <algorithm>
#include <utility>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>

using namespace std;

class Node
{
    public:
        Node()
        {

        }
};

typedef vector<double> Joints;
struct Vertex {Joints* ptr;};

typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, Vertex, double> graph_t;
typedef boost::graph_traits<graph_t>::vertex_descriptor vertex_t;
typedef boost::graph_traits<graph_t>::edge_descriptor edge_t;

class RRTstar
{
    public:
        RRTstar(double radius, int num_joints, Joints start_configuration, Joints end_configuration)
        {

        }
        void solve(int num_iterations)
        {
            // Iterate for the specified number of iterations
            for (int i = 0; i < num_iterations; i++)
            {
                // Add a new random position in joint space as a node to the graph
                graph_t::vertex_iterator new_node_itr = addRandomVertex();

                // Get the vertex with the nearest position in the graph
                pair<graph_t::vertex_iterator, double> nn_out = getNearestNeighbor(new_node_itr);
                graph_t::vertex_iterator nearest_neighbor_itr = nn_out.first;
                double nearest_neighbor_cost = nn_out.second;

                // Get all neighbors within the specified radius
                pair<vector<graph_t::vertex_iterator>, graph_t::vertex_iterator> n_out = getNeighbors(new_node_itr);
                vector<graph_t::vertex_iterator> neighbors_it = n_out.first;
                graph_t::vertex_iterator best_neighbor_it = n_out.second;

                // Link together new position with its best neighbor


            }
        }
    private:
        // Graph for storing search results
        graph_t G;

        // Generate a random position in joint space
        Joints generateRandomPosition()
        {
            // Generate num_joints random numbers between 0 and 2pi
        }

        // Generate a random position in joint space that is NOT in collision
        Joints generateValidPosition()
        {
            // Generate a random position
            // Check if this random position causes a collision
            // Keep doing this until you get a position that is not in collision
        }

        // Generate a valid random position and add it as an unconnected
        // node on the graph
        graph_t::vertex_iterator addRandomVertex()
        {

        }

        // Get the nearest position in the graph to specified position
        // Remember this is different from getNeighbors which just gets all nodes
        // within a specified distance.
        // This finds the closest of ALL nodes, without caring what the
        // absolute distance is
        pair<graph_t::vertex_iterator, double> getNearestNeighbor(graph_t::vertex_iterator new_node_itr)
        {
            // Iterate through all the verticies
            // Return the closest one
            double min_distance = -1.0;
            Joints new_node_joints = *G[*new_node_itr].ptr;
            graph_t::vertex_iterator v, v_end, v_closest;
            for (boost::tie(v, v_end) = boost::vertices(G); v != v_end; ++v)
            {
                // Make sure current vertex i is not the new vertex
                if (v != new_node_itr)
                {
                    // Unpack current vertex i
                    Vertex i_vertex = G[*v];
                    Joints i_joints = *(i_vertex.ptr);

                    // Get the distance between these two nodes in joint space
                    double distance = calculateJointDistance(new_node_joints, i_joints);

                    // Save it as the closest node if it's closer than the
                    // currently saved closest node. Or if this is the first run.
                    if (min_distance < 0 || distance < min_distance)
                    {
                        // Save the distance and an iterator pointing to that node
                        min_distance = distance;
                        v_closest = v;
                    }
                }
            }
            return make_pair(v_closest, min_distance);
        }

        // TODO: Account for theta circular wrap-around
        double calculateJointDistance(Joints A, Joints B)
        {
            double squared_sum = 0;
            for (int i=0; i < A.size(); i++)
            {
                squared_sum = squared_sum + pow((A[i] - B[i]), 2);
            }
            return sqrt(squared_sum);
        }

        // Get the closest neighbors and single out the closest neighbor
        // within that group
        pair<vector<graph_t::vertex_iterator>, graph_t::vertex_iterator> getNeighbors(graph_t::vertex_iterator new_node_itr)
        {
            vector<graph_t::vertex_iterator> neighbors;
            graph_t::vertex_iterator best_neighbor;

            graph_t::vertex_iterator v, v_end;
            for (boost::tie(v, v_end) = boost::vertices(G); v != v_end; ++v)
            {
                // Make sure current vertex i is not the new vertex
                if (v != new_node_itr)
                {
                    // Unpack current vertex i
                    Vertex i_vertex = G[*v];
                    Joints i_joints = *(i_vertex.ptr);

                    // Get the distance between these two nodes in joint space
                    double distance = calculateJointDistance(new_node_joints, i_joints);

                    //  Vertex otherVertex = G[*vclosest];
                }
            }
        }

        // // Add an edge between these two nodes
        // void addEdge(Vertex A, Vertex B)
        // {

        // }


};


int main(int argc, char **argv)
{
    // Test that RRT* class works
    // double radius = 0.5;
    // int num_joints = 7;

    // RRTstar rrts(radius, num_joints);
    // return 0;
}