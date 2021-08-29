#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y,
                           float end_x, float end_y)
    : m_Model(model) {
  // Convert inputs to percentage:
  start_x *= 0.01;
  start_y *= 0.01;
  end_x *= 0.01;
  end_y *= 0.01;

  start_node = &(model.FindClosestNode(start_x, start_y));
  end_node = &(model.FindClosestNode(end_x, end_y));
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
  return node->distance(*end_node);
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
  current_node->FindNeighbors();
  for (auto& neighbor : current_node->neighbors) {
    if (neighbor->visited == false) {
      neighbor->parent = current_node;
      neighbor->g_value =
          current_node->g_value + neighbor->distance(*current_node);
      neighbor->h_value = CalculateHValue(neighbor);
      neighbor->visited = true;
      open_list.emplace_back(neighbor);
    }
  }
}

RouteModel::Node *RoutePlanner::NextNode() {
  std::sort(std::begin(open_list), std::end(open_list),
            [](RouteModel::Node *a, RouteModel::Node *b) {
              return (a->g_value + a->h_value) > (b->g_value + b->h_value);
            });

  RouteModel::Node *candidate_node = open_list.back();
  open_list.pop_back();
  return candidate_node;
}

std::vector<RouteModel::Node>
RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
  // Create path_found vector
  distance = 0.0f;
  std::vector<RouteModel::Node> path_found;

  while (current_node != start_node) {
    distance += current_node->distance(*(current_node->parent));
    path_found.emplace_back(*current_node);
    current_node = current_node->parent;
  }
  path_found.emplace_back(*current_node);
  std::reverse(path_found.begin(), path_found.end());

  distance *= m_Model.MetricScale(); // Multiply the distance by the scale of
                                     // the map to get meters.
  return path_found;
}

void RoutePlanner::AStarSearch() {
  RouteModel::Node *current_node = nullptr;

  current_node = start_node;
  current_node->visited = true;

  while (current_node != end_node) {
    AddNeighbors(current_node);
    current_node = NextNode();
  }
  m_Model.path = ConstructFinalPath(current_node);
}