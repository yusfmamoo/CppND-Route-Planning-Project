#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    this->start_node = &(m_Model.FindClosestNode(start_x, start_y));
	this->end_node = &(m_Model.FindClosestNode(end_x, end_y));
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node){
	return (node->distance(*end_node));
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {

	current_node->FindNeighbors();
	for(auto node : current_node->neighbors)
	{
		node->parent = current_node;
		node->h_value = CalculateHValue(node);
		node->g_value = current_node->distance(*node) + current_node->g_value;
		node->visited = true;
		this->open_list.push_back(node);
	}
}

RouteModel::Node *RoutePlanner::NextNode() {
	std::sort(open_list.begin(), open_list.end(), [](const RouteModel::Node* lhs, const RouteModel::Node* rhs)
	{
		return ((lhs->g_value + lhs->h_value) > (rhs->g_value + rhs->h_value));
	});

	RouteModel::Node* return_node{open_list.back()};
	open_list.pop_back();

	return return_node;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

	RouteModel::Node traversed_node = *current_node;

	path_found.push_back(traversed_node);

    while(!(traversed_node == *(this->start_node)))
    {
		path_found.push_back(*traversed_node.parent);
		distance += traversed_node.distance(*traversed_node.parent);
		traversed_node = *traversed_node.parent;
    }

    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // TODO: Implement your solution here.
    current_node = start_node;
    current_node->visited = true;
    open_list.push_back(start_node);
    while(current_node != (this->end_node))
    {
	    current_node = NextNode();
    	AddNeighbors(current_node);
    }
    auto final_path = ConstructFinalPath(current_node);
    m_Model.path = final_path;
}
