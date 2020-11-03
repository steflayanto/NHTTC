#pragma once
#include <sgd/ttc_sgd_problem.h>
#include <iostream>




enum class AType { V, A, DD, ADD, CAR, ACAR, MUSHR };

class Agent {
private:
  int example;

public:
  TTCSGDProblem* prob;
  SGDOptParams opt_params;
  int u_dim, x_dim;
  bool reactive, controlled, done = false;
  AType a_type;
  std::string type_name;
  Eigen::Vector2f goal;

  Agent(SGDOptParams opt_params_in);
  Agent(std::vector<std::string> parts, SGDOptParams opt_params_in);

  // Added for easier interfacing

  void SetPlanTime(float agent_plan_time_ms);

  // Pass in own index if agent is itself one of the obstacles passed in, otherwise pass in -1
  void SetObstacles(std::vector<TTCObstacle*> obsts, size_t own_index);

  // Update Goal by passing in a new vector. If unchanged, pass in NULL. Calling the function will update goal and pass it into SGD
  void UpdateGoal(Eigen::Vector2f new_goal);

  // Sets ego of Agent
  // Note: For the MuSHR car, x is [x, y, heading_angle] and u is [velocity, steering_angle]
  void SetEgo(Eigen::VectorXf new_x);

  // Controls can be extracted with agent->prob.params.ucurr at any time. This function runs the update and also returns the controls.
  // Note: For the MuSHR car, x is [x, y, heading_angle] and u is [velocity, steering_angle]
  Eigen::VectorXf UpdateControls();

  // Original From nhttc_utils
  float GetBestGoalCost(float dt, const Eigen::Vector2f& pos);
  float GetBestCost();
  bool AtGoal();
  void SetStop();
  void PrepareSGDParams();
};

// Global Functions

std::vector<std::string> GetAgentParts(int agent_type, Eigen::Vector2f& pos, bool reactive, Eigen::Vector2f& goal);

void ConstructGlobalParams(SGDOptParams *opt_params);

std::vector<TTCObstacle*> BuildObstacleList(std::vector<Agent> agents);



/*
EXAMPLE INTERFACE USAGE

DECLARATION
------------------------------------------------------
1. Prepare Global Params for SGD

SGDOptParams global_params;
ConstructGlobalParams(&global_params);


2. Declare Agents

std::vector<Agent> agents;
for every agent:
  agents.emplace_back(GetAgentParts(int agent_type, Eigen::Vector2f& pos, bool reactive, Eigen::Vector2f& goal), global_params);


3. Declare Global Obstacle List

std::vector<TTCObstacle*> obstacles = BuildObstacleList(agents);

PLANNING CYCLES
------------------------------------------------------

1. Update positions for each agent. Might want to expand this to set control actions too

for every agent:
  agent.SetEgo(Eigen::VectorXf new_x_o); // Note: For the MuSHR car, x is [x, y, heading_angle] and u is [velocity, steering_angle]


2. Build global obstacle list (right now only consists of agents, but we can add other obstacles down the road)

std::vector<TTCObstacle*> obstacles = BuildObstacleList(agents);


for every agent:
  3. Set the max planning time. May later expand to allow setting of more variables

  agent.SetPlanTime(optimal_planning_time_ms) // example: float agent_plan_time_ms = 0.95f * PLANNING_TIME / n_controlled * 1000.0f;


  4. Set individual agent's obstacle list 
  agent.SetObstacles(agent, own_index, agent_obstacles) // will add all obstacles except for itself, by omitting obstacle at own index


  5. Update the goal if desired (later when using carrot goal planner)
  agent.UpdateGoal(Eigen::Vector2f new_goal);


  6. Plan and update controls! Controls can also be accessed at anytime without updating via agent->prob.params.u_curr
  Eigen::VectorXf controls = agent.PlanControls(); 

*/