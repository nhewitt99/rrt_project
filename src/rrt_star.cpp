#include <algorithm>
#include <utility>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>

using namespace std;

typedef vector<double> Joints;
class Node
{
    public:
        Node(Joints j, int i)
        {
            joints = j;
            id = i;
        }
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
        RRTstar(double radius, int num_joints, Joints start_configuration, Joints end_configuration)
        {
            // Create graph with root node
        }
        void solve(int num_iterations)
        {
            // Iterate for the specified number of iterations
            for (int i = 0; i < num_iterations; i++)
            {
                // Add a new random position in joint space as a vertex to the graph
                // This new vertex is guaranteed to be within specified radius of an existing vertex
                vertex_t new_node_desc = addRandomVertex();

                // Get all neighbors within the specified radius
                vector<vertex_t> neighbors = getNeighbors(new_node_desc);

                // Get the nearest neighbor
                vertex_t nearest_neighbor = getNearestNeighbor(new_node_desc, neighbors);

                // Assign a cost to the new vertex based on its distance to its nearest neighbor
                // Note this is only the cost to get from the nearest neighbor to the new vertex,
                // NOT the overall cost to get to the new vertex
                Cost.push_back(calculateCost(new_node_desc, nearest_neighbor));

                // Rewire neighbors so that the new node is now their parent node if this would result in a lower cost for that node
                rewireNeighbors(new_node_desc, neighbors);

                // Link together new vertex with its nearest neighbor
                linkNewVertex(new_node_desc, nearest_neighbor);
            }
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

        // Generate a valid random position and add it as an unconnected
        // node on the graph
        vertex_t addRandomVertex()
        {
            // TODO: @Nathan: Add functionality to generate a random joint position
            // and then "extend" a vertex on the graph towards that position
            // Also make sure that the new vertex is NOT in collision
            Joints new_joints = {};
            int id = Cost.size();
            Node* node_ptr = new Node(new_joints, id);
            Vertex vertex = {node_ptr};
            return boost::add_vertex(vertex, G);
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

        double calculateCost(vertex_t A, vertex_t B)
        {
            return calculateJointDistance(G[A].ptr->joints, G[B].ptr->joints);
        }

        // Get all the vertices within
        vector<vertex_t> getNeighbors(vertex_t new_node_desc)
        {
            // Unpack new node
            Vertex new_node_vertex = G[new_node_desc];
            Joints new_node_joints = new_node_vertex.ptr->joints;

            // Initialize vector for storing neighbors
            vector<vertex_t> neighbors;

            // Iterate through every vertex in the graph
            graph_t::vertex_iterator v, v_end;
            for (boost::tie(v, v_end) = boost::vertices(G); v != v_end; ++v)
            {
                // Make sure current vertex i is not the new vertex
                if (G[*v].ptr->id != new_node_vertex.ptr->id)
                {
                    // Unpack current vertex i
                    Vertex i_vertex = G[*v];
                    Joints i_joints = i_vertex.ptr->joints;

                    // Calculate the distance between these two nodes in joint space
                    double distance = calculateJointDistance(new_node_joints, i_joints);

                    // Vertex is a neighbor if within radius
                    if (distance < radius)
                    {
                        // Add vertex to vector of neighbors
                        neighbors.push_back(*v);
                    }
                }
            }
            return neighbors;
        }

        vertex_t getNearestNeighbor(vertex_t new_node, vector<vertex_t> neighbors)
        {
            // Initialze variables for storing nearest neighbor info
            double min_distance = -1.0;
            vertex_t nearest_neighbor;

            // Iterate through all neighbors
            for (vertex_t neighbor : neighbors)
            {
                // Calculate distance to neighbor
                double distance = calculateJointDistance(G[new_node].ptr->joints, G[neighbor].ptr->joints);

                // If a nearest neighbor has not been specified
                // OR if this neighbor is closer than the current nearest neighbor
                if (min_distance < 0 || distance < min_distance)
                {
                    // Save distance
                    min_distance = distance;
                    // Set as new nearest neighbor
                    nearest_neighbor = neighbor;
                }
            }
            return nearest_neighbor;
        }

        void linkNewVertex(vertex_t new_vertex, vertex_t existing_vertex)
        {
            boost::add_edge(new_vertex, existing_vertex, Cost[G[new_vertex].ptr->id], G);
        }

        void deLinkVertices(vertex_t A, vertex_t B)
        {
            boost::remove_edge(A, B, G);
        }

        void rewireNeighbors(vertex_t new_vertex, vector<vertex_t> neighbors)
        {
            for (vertex_t neighbor: neighbors)
            {
                // If jumping from the new vertex to this neighbor is cheaper than jumping from the
                // neighbor's parent to the neighbor, then the new vertex is now the parent.
                // Congrats and good luck on raising that vertex to be a good upstanding citizen
                if (Cost[Id(new_vertex)] + calculateCost(new_vertex, neighbor) < Cost[Id(neighbor)])
                {
                    // Set new cost for the neighbor
                    Cost[Id(neighbor)] = Cost[Id(new_vertex)] + calculateCost(new_vertex, neighbor);

                    // Delink the neighbor from its original parent
                    deLinkVertices(neighbor, Parents[Id(neighbor)]);

                    // Link the neighbor as a "new" vertex to the actually new vertex
                    linkNewVertex(neighbor, new_vertex);
                }
            }
        }

        int Id(vertex_t v)
        {
            return G[v].ptr->id;
        }
};


int main(int argc, char **argv)
{
    return 0;
}