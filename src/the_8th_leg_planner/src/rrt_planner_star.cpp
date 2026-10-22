#include <map>
#include <random>
#include <thread>
#include <cmath>
#include <limits>
#include <algorithm>
#include <chrono>
#include <fstream>
#include <cstdlib>
 
#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <visualization_msgs/msg/marker.hpp>
 
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
 
using Config = std::vector<double>;   // 6 joint angles
 
struct TreeNode {
  Config q;            // joint configuration
  int parent;          // index of parent in tree, -1 for root
  double cost;         // path cost from root 
  Eigen::Vector3d ee;  // end-effector position (cached FK, for drawing)
};
 
struct RunResult {
  bool success;
  int iters;
  size_t treeSize;
  double cost;
  size_t waypoints;
  double ms;
};
 
Config sample(std::mt19937& rng, const std::vector<std::pair<double, double>>& bounds, const Config& goal, double goal_bias) {
  // Returns 6 position config position (pose) within bounds
  // & on rng doesn't make copy of orgininal and repeat same rng
  // goal_bias <% of picking towards goal in decimal> 
  Config randomConfig{};
  std::uniform_real_distribution<double> distbias(0.0, 1.0);
  double bias = distbias(rng); // generate rng to determine action 1. Random Config 2. Rrturn Goal
  if (bias < goal_bias) {
    return goal;
  } else {
    for (const auto& [min, max] : bounds) {
      std::uniform_real_distribution<double>dist(min,max);
      randomConfig.push_back(dist(rng));
    }
  }
  return randomConfig;
}
 
double getDist(const Config& pointA, const Config& pointB){
  // Simple euclidian distance of motion poses
  double dist {0};
  for (size_t i{0}; i<pointA.size(); i++){
    double d = pointA[i] - pointB[i];
    dist += d * d;
  }
  dist = std::sqrt(dist);
  return dist;
}
 
int nearest(const Config& pose, const std::vector<TreeNode>& ntree){
  // returns idx of the nearest TreeNode from a pose
  int smallestIdx{0}; 
  double smallestDist{std::numeric_limits<double>::infinity()};
  double dist{};
 
  for (size_t i{0}; i<ntree.size(); i++) {
    dist = getDist(pose, ntree[i].q);
    if (dist < smallestDist) {
      smallestDist = dist;
      smallestIdx = static_cast<int>(i);
    }
  }
  return smallestIdx;
}
 
Config steer(const Config& nearPose, const Config& randomPose, const double step){
  // returns the pose to be added to tree
  // step is how far from nearest in rad  
  double dist {getDist(nearPose, randomPose)};
  if (dist <= step) {
    return randomPose; 
  }
 
  Config newPose {nearPose};
  for (size_t i{0}; i<nearPose.size(); i++) {
    newPose[i] += ((randomPose[i]-nearPose[i]) * (step/dist)); // proportionate math: might consider an actual distance traveled math later
  }
  return newPose;
}
 
std::vector<Config> extractPath(const std::vector<TreeNode>& tree, int goalIdx) {
  // shows the full path if path is found from Root -> Goal
  std::vector<Config> fullPath {};
  int tracker {goalIdx}; 
 
  while (tracker != -1) {
    fullPath.push_back(tree[tracker].q);
    tracker = tree[tracker].parent;
  }
  std::reverse(fullPath.begin(), fullPath.end());
  return fullPath;
 
}

std::vector<int> nearbyNodes (const Config& newNode, const std::vector<TreeNode>& tree, const double radius) {
  // returns vector<in> in index of nearby nodes from newNode
  std::vector<int> nearList {};

  for(size_t i {0}; i < tree.size(); i++) {
    if (getDist(newNode, tree[i].q) <= radius) {
      nearList.push_back(static_cast<int>(i));
    }
  }
  return nearList;
}
 
int main(int argc, char** argv) {
  rclcpp::init(argc, argv); // reading launch args
  // create node for param, makes sure it takes input params
  auto node = std::make_shared<rclcpp::Node>("rrt_planner_star",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
 
  // Spin in the background so MoveIt's monitors can receive messages
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  std::thread spinner([&exec]() { exec.spin(); });
 
  // --- MoveIt planning scene: robot model, current state, the box ---
  auto psm = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node, "robot_description");
  psm->startSceneMonitor();
  psm->startStateMonitor();
  bool sceneOk = psm->requestPlanningSceneState();
  RCLCPP_INFO(node->get_logger(), "scene request ok? %d", sceneOk);
  psm->getStateMonitor()->waitForCompleteState("arm", 5.0);
 
  // Freeze a private copy of the scene for this planning run
  planning_scene::PlanningScenePtr scene;
  {
    planning_scene_monitor::LockedPlanningSceneRO locked(psm);
    scene = planning_scene::PlanningScene::clone(locked);
  }
 
    // Baseline obstacle: same box (0.1 x 0.1 x 0.3 at 0.31, 0.21, 0.25)
  moveit_msgs::msg::CollisionObject box;
  box.header.frame_id = "world";
  box.id = "box1";
  shape_msgs::msg::SolidPrimitive prim;
  prim.type = shape_msgs::msg::SolidPrimitive::BOX;
  prim.dimensions = {0.1, 0.1, 0.3};
  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.31;
  pose.position.y = 0.21;
  pose.position.z = 0.25;
  pose.orientation.w = 1.0;
  box.primitives.push_back(prim);
  box.primitive_poses.push_back(pose);
  box.operation = moveit_msgs::msg::CollisionObject::ADD;
  scene->processCollisionObjectMsg(box);
 
  const auto* jmg = scene->getRobotModel()->getJointModelGroup("arm"); // const so read only
  moveit::core::RobotState state = scene->getCurrentState();
 
  // Joint bounds from your URDF limits for the planner to select within limits
  std::vector<std::pair<double, double>> bounds;
  for (const auto* jm : jmg->getActiveJointModels()) { // skip world joint
    const auto& b = jm->getVariableBounds()[0]; // [0] for angle
    bounds.emplace_back(b.min_position_, b.max_position_);
  }
 
  // Start = current state. Goal = goal_a hard coded in SRDF
  Config start;
  state.copyJointGroupPositions(jmg, start);
  std::map<std::string, double> goal_map;
  jmg->getVariableDefaultPositions("goal_a", goal_map);
  Config goal;
  for (const auto& name : jmg->getActiveJointModelNames()) goal.push_back(goal_map[name]);
 
  // Collision check: box, self-collision, SRDF disabled pairs
  auto isValid = [&](const Config& q) {
    state.setJointGroupPositions(jmg, q);
    state.update(); // recalc
    return scene->isStateValid(state, "arm");
  };
 
  // Forward kinematics: joint angles -> end-effector position
  auto fk = [&](const Config& q) {
    state.setJointGroupPositions(jmg, q);
    state.update();
    return Eigen::Vector3d(state.getGlobalLinkTransform("link_7").translation());
  };
 
    // Debug helper: turn a Config into a printable string
  auto str = [](const Config& q) {
    std::string s = "[";
    for (double v : q) s += std::to_string(v) + " ";
    return s + "]";
  };
 
  // Sanity check comments
  RCLCPP_INFO(node->get_logger(), "start = %s", str(start).c_str());
  RCLCPP_INFO(node->get_logger(), "goal  = %s", str(goal).c_str());
  for (size_t i = 0; i < bounds.size(); ++i)
  RCLCPP_INFO(node->get_logger(), "joint %zu bounds: [%f, %f]", i, bounds[i].first, bounds[i].second);
  RCLCPP_INFO(node->get_logger(), "start valid? %d   goal valid? %d", isValid(start), isValid(goal));
  RCLCPP_INFO(node->get_logger(), "obstacles in scene: %zu", scene->getWorld()->size());
  // Fixed seed -> identical runs for now
  std::mt19937 rng(42);
 
  // Tree visualization
  auto marker_pub = node->create_publisher<visualization_msgs::msg::Marker>("rrt_tree", 10);
  std::vector<TreeNode> tree;
  auto publishTree = [&]() {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "world";
    m.header.stamp = node->now();
    m.ns = "rrt_tree";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::LINE_LIST;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.002;
    m.color.g = 1.0;
    m.color.a = 0.8;
    for (size_t i = 1; i < tree.size(); ++i) {
      const auto& a = tree[i].ee;
      const auto& b = tree[tree[i].parent].ee;
      geometry_msgs::msg::Point pa, pb;
      pa.x = a.x(); pa.y = a.y(); pa.z = a.z();
      pb.x = b.x(); pb.y = b.y(); pb.z = b.z();
      m.points.push_back(pa);
      m.points.push_back(pb);
    }
    marker_pub->publish(m);
  };
 
  tree.push_back({start, -1, 0.0, fk(start)});
  RCLCPP_INFO(node->get_logger(), "Start and goal loaded. Planning...");
 
  // RRT LOGIC ======================================
  // Functions that need Moveit
  auto edgeValid = [&](const Config& newNode, const Config& nearNode, double checkRes) -> bool {
    // Checks evenly spaced points from nearNode to newNode for collisions.
    // nearNode is already in the tree (already valid), so start at k = 1.
    double dist {getDist(newNode, nearNode)};
    int n {static_cast<int>(std::ceil(dist / checkRes))};
 
    Config point(nearNode.size());
    for (int k {1}; k <= n; k++) {
      double t {static_cast<double>(k) / n}; // fraction of the way along the edge
      for (size_t i {0}; i < nearNode.size(); i++) {
        point[i] = nearNode[i] + t * (newNode[i] - nearNode[i]);
      }
      if (!isValid(point)) return false;
    }
    return true; // every check passed
  };

  auto bestParent = [&](const Config& newNode, const std::vector<int>& neighbors, 
                        const std::vector<TreeNode>& ntree, const int idx, const double checkRes) -> int {
    // return cloest parent index for nearNode Optimization based on cost
    double minCost {ntree[idx].cost + getDist(newNode, ntree[idx].q)};
    int minCostIdx {idx};

    for(size_t i {0}; i < neighbors.size(); i++) {
    // find which of is smallest cost
    int n {neighbors[i]};
    if (ntree[n].cost + getDist(newNode, ntree[n].q) < minCost) {
      if (edgeValid(newNode, ntree[n].q, checkRes))
        {
        minCost = ntree[n].cost + getDist(newNode, ntree[n].q);
        minCostIdx = n;
        }
    }
    }
    return minCostIdx;
  };

  auto rewire = [&](const Config& newNode, std::vector<TreeNode>& tree, const std::vector<int>& neighbors, const double checkRes)-> void {
    // rewire surrounding nodes for better optimization
    double potentialCost {};
    int newIdx {static_cast<int>(tree.size()) - 1}; //idx of new Node

    for(size_t i {0}; i < neighbors.size(); i++) {
      int n {neighbors[i]};
      potentialCost = tree[newIdx].cost + getDist(newNode, tree[n].q);
      if (potentialCost < tree[n].cost && edgeValid(newNode, tree[n].q, checkRes)) {
        tree[n].parent = newIdx; // new parent is new Node
        tree[n].cost = potentialCost;
      }
    }
    return;
  };
  // Constants (mostly tunable)
  const double stepSize {0.5}; // furthest dist (rad) it'll plot next pose
  const double goalBias {0.05}; // use goal pose how often
  const double checkRes {.02}; // fine checking between node. clean number
  const double golTol {0.05}; // how close to goal before counting as solved
  const int maxIter {15000}; // Max iter before no path found
  const double radius {0.75}; // 1.5x stepsize
  // BENCHMARK
  auto runRRT = [&](unsigned seed) -> RunResult {
    rng.seed(seed); // different seed each run
    tree.clear(); // fresh tree each run
    tree.push_back({start, -1, 0.0, fk(start)}); // root
    int itersUsed {maxIter};                  
    size_t waypointsUsed {0};                   
    auto t0 = std::chrono::steady_clock::now(); // timer start
  
    // Core Logic 
    Config randNode {};
    Config newNode {};
    int idx {};
    bool reached {false};
    std::vector<int> neighbors {};
    int closestNeighbor {};

    for (int iter{0}; iter<maxIter; iter++){
      randNode = sample(rng, bounds, goal, goalBias); // random pose gen
      idx = nearest(randNode, tree); // nearest node find
      newNode = steer(tree[idx].q, randNode, stepSize); // random pose -> pose to add
  
      if (!edgeValid(newNode, tree[idx].q, checkRes)){continue;}
      else { // find closest, put that in, rewire neighbors
        neighbors = nearbyNodes(newNode, tree, radius);
        closestNeighbor = bestParent(newNode, neighbors, tree, idx, checkRes);
        tree.push_back({newNode, closestNeighbor, tree[closestNeighbor].cost + getDist(newNode, tree[closestNeighbor].q), fk(newNode)});
        rewire(newNode, tree, neighbors, checkRes);
      }
      if (tree.size() % 50 == 0) {publishTree();}
      if (getDist(goal, newNode) <= golTol) {
        // GOAL FOUND
        std::vector<Config> goalPath {};
        RCLCPP_INFO(node->get_logger(), "Goal reached at iter %d, tree size %zu, path cost %.3f rad",
              iter, tree.size(), tree.back().cost);
        reached = true;
  
        int goalIdx {static_cast<int>(tree.size()) - 1};
        goalPath = extractPath(tree, goalIdx); //return full path
        itersUsed = iter + 1;           
        waypointsUsed = goalPath.size();        
        RCLCPP_INFO(node->get_logger(), "path has %zu waypoints", goalPath.size());
        RCLCPP_INFO(node->get_logger(), "first = %s", str(goalPath.front()).c_str());
        RCLCPP_INFO(node->get_logger(), "last  = %s", str(goalPath.back()).c_str());
        break;
      }
    }
    auto t1 = std::chrono::steady_clock::now();  
    double ms {std::chrono::duration<double, std::milli>(t1 - t0).count()}; 
    if (!reached) {
      RCLCPP_INFO(node->get_logger(), "No path found after %d iterations, tree size %zu", maxIter, tree.size());
    }
    return {reached, itersUsed, tree.size(), reached ? tree.back().cost : 0.0, waypointsUsed, ms}; 
  }; 
 
  // 10 seeds
  std::string csvPath {std::string(std::getenv("HOME")) + "/projects/rbe550FinalProj/results/benchmarks/rrtOnly.csv"};
  std::ofstream csv(csvPath);
  if (!csv) {
    RCLCPP_ERROR(node->get_logger(), "Could not open %s (does the folder exist?)", csvPath.c_str());
  }
  csv << "seed,success,iters,tree_size,waypoints,path_cost_rad,planning_time_ms\n";
  for (unsigned seed {1}; seed <= 10; seed++) {
    RunResult r {runRRT(seed)};
    RCLCPP_INFO(node->get_logger(), "seed %u: success=%d iters=%d tree=%zu waypoints=%zu cost=%.3f rad time=%.1f ms",
          seed, r.success, r.iters, r.treeSize, r.waypoints, r.cost, r.ms);
    csv << seed << "," << r.success << "," << r.iters << "," << r.treeSize << ","
        << r.waypoints << "," << r.cost << "," << r.ms << "\n";
  }
  csv.close();
  RCLCPP_INFO(node->get_logger(), "Saved to %s", csvPath.c_str());
 
  publishTree();
  rclcpp::sleep_for(std::chrono::seconds(2)); // just to make sure tree publishes
 
  rclcpp::shutdown();
  spinner.join();
  return 0;
}
