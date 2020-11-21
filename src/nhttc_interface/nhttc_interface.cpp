/*
Copyright 2020 University of Minnesota and Clemson University

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

MODIFIED BY STEFAN LAYANTO on behalf of the MuSHR Project of the Personal Robotics Lab at the University of Washington

*/

#include <nhttc_interface/nhttc_interface.h>
#include <sgd/ttc_sgd_problem_models.h>

// UTILS
std::vector<std::string> GetParts(std::string s, char delim) {
  std::stringstream ss(s);
  std::vector<std::string> parts;
  for (std::string part; std::getline(ss, part, delim); ) {
    parts.push_back(std::move(part));
  }
  return parts;
}

std::vector<std::vector<std::string>> LoadFileByToken(std::string file_name, int n_skip, char delim) {
  std::ifstream datafile(file_name);
  std::vector<std::vector<std::string>> data_vec;
  int n_lines_read = 0;
  for (std::string line; std::getline(datafile, line); ) {
    n_lines_read++;
    if (n_lines_read <= n_skip) {
      continue;
    }
    std::vector<std::string> parts = GetParts(line, delim);
    data_vec.push_back(parts);
  }
  datafile.close();
  return data_vec;
}

void SetBoundsV(TTCParams &params) {
  params.box_constraint = false;
  params.circle_u_limit = 0.3f;
  params.u_lb = Eigen::Vector2f(-0.3f, -0.3f);
  params.u_ub = Eigen::Vector2f(0.3f, 0.3f);
}
void SetBoundsA(TTCParams &params) {
  params.u_lb = Eigen::Vector2f(-1.0f, -1.0f);
  params.u_ub = Eigen::Vector2f(1.0f, 1.0f);
}
void SetBoundsDD(TTCParams &params) {
  params.u_lb = Eigen::Vector2f(-0.3f, -1.0f);
  params.u_ub = Eigen::Vector2f(0.3f, 1.0f);
}
void SetBoundsADD(TTCParams &params) {
  params.u_lb = Eigen::Vector2f(-1.0f, -1.0f);
  params.u_ub = Eigen::Vector2f(1.0f, 1.0f);
}
void SetBoundsCAR(TTCParams &params) {
  params.u_lb = Eigen::Vector2f(-0.3f, -0.25f * M_PI);
  params.u_ub = Eigen::Vector2f(0.3f, 0.25f * M_PI);
}
void SetBoundsACAR(TTCParams &params) {
  params.u_lb = Eigen::Vector2f(-1.0f, -0.25f * M_PI);
  params.u_ub = Eigen::Vector2f(1.0f, 0.25f * M_PI);
}

void SetBoundsMUSHR(TTCParams &params) {
  params.u_lb = Eigen::Vector2f(-0.3f, -0.25f * M_PI);
  params.u_ub = Eigen::Vector2f(0.3f, 0.25f * M_PI);
}

int GetVector(const std::vector<std::string>& parts, int offset, int v_len, Eigen::VectorXf& v) {
  if (parts[offset] == "r") {
    v = Eigen::VectorXf::Random(v_len);
    offset++;
  } else {
    v = Eigen::VectorXf::Zero(v_len);
    for (int i = 0; i < v_len; ++i) {
      v[i] = static_cast<float>(std::atof(parts[offset].c_str()));
      offset++;
    }
  }
  return offset;
}

// END UTILS


// AGENT DEFINITIONS

Agent::Agent(SGDOptParams opt_params_in) {
  opt_params = opt_params_in;
}

Agent::Agent(std::vector<std::string> parts, SGDOptParams opt_params_in) {
  TTCParams params;
  params.find_all_ttcs = false;
  type_name = parts[0];
  if (parts[0] == "v") {
    a_type = AType::V;
    u_dim = x_dim = 2;
    SetBoundsV(params);
    prob = new VTTCSGDProblem(params);
  } else if (parts[0] == "a") {
    a_type = AType::A;
    u_dim =  2;
    x_dim = 4;
    SetBoundsA(params);
    prob = new ATTCSGDProblem(params);
  } else if (parts[0] == "dd") {
    a_type = AType::DD;
    u_dim =  2;
    x_dim = 3;
    SetBoundsDD(params);
    prob = new DDTTCSGDProblem(params);
  } else if (parts[0] == "add") {
    a_type = AType::ADD;
    u_dim =  2;
    x_dim = 5;
    SetBoundsADD(params);
    prob = new ADDTTCSGDProblem(params);
  } else if (parts[0] == "car") {
    a_type = AType::CAR;
    u_dim =  2;
    x_dim = 3;
    SetBoundsCAR(params);
    prob = new CARTTCSGDProblem(params);
  } else if (parts[0] == "acar") {
    a_type = AType::ACAR;
    u_dim =  2;
    x_dim = 5;
    SetBoundsACAR(params);
    prob = new ACARTTCSGDProblem(params);
  } else if (parts[0] == "mushr") {
    a_type = AType::MUSHR;
    u_dim =  2;
    x_dim = 3;
    SetBoundsMUSHR(params);
    
    // Additional parameter config for MuSHR
    // wheelbase is ~17.5 inches(based on redcat chassis) -> 0.465m long.
    float length = 0.465f; 
    float width = 0.285f;
    // Inside car kinematics, length is computed as 2 * 2 / sqrt(5) * radius.
    // Therefore, reversing it, radius = length * sqrt(5) / 4.0
    // params.radius = 0.2;
    params.radius = std::sqrt(width * width + length * length) / 2;  // radius is half the diagonal
    // params.safety_radius = 0.05;
    params.safety_radius = 0.05f; // purpose of safety radius is unclear, set to 0.05m for now
    params.max_ttc = 20;//std::min(6/max_velocity,20);
    params.vel_limit = 0.3f; // top speed of mushr set to 0.3 m/s?
    params.wheel_base = 0.29f;

    // params.steer_limit = ; // still needs to be set
    
    prob = new MUSHRTTCSGDProblem(params);
  } else {
    std::cerr << "Unsupported Dynamics Model: " << parts[0] << std::endl;
    exit(-1);
  }
  controlled = (parts[1] == "y");
  reactive = (parts[2] == "y");
  int offset = 3;
  Eigen::VectorXf u_0, x_0, g;
  offset = GetVector(parts, offset, x_dim, x_0);
  offset = GetVector(parts, offset, u_dim, u_0);
  offset = GetVector(parts, offset, 2, g);

  prob->params.u_curr = u_0;
  prob->params.x_0 = x_0;
  goal = g;

  opt_params = opt_params_in;
  opt_params.x_lb = prob->params.u_lb;
  opt_params.x_ub = prob->params.u_ub;
  opt_params.x_0 = u_0;
}

void Agent::SetPlanTime(float agent_plan_time_ms) {
  opt_params.max_time = agent_plan_time_ms;
}

void Agent::SetObstacles(std::vector<TTCObstacle*> obsts, size_t own_index) {
  prob->params.obsts.clear();
  // Don't plan for non reactive agents
  if (!reactive) {
    return;
  }
  for (size_t b_idx = 0; b_idx < obsts.size(); ++b_idx) {
    if (b_idx == own_index) {
      continue;
    }
    float dist = (prob->params.x_0.head<2>() - obsts[b_idx]->p.head<2>()).norm();
    // Ignore any obstacles that cannot interact with us within our ttc horizon
    if (dist < 2.0 * prob->params.vel_limit * prob->params.max_ttc) {
      prob->params.obsts.push_back(obsts[b_idx]);
    }
  }
}

void Agent::UpdateGoal(Eigen::Vector2f new_goal) {
  // if (new_goal != NULL) {
  goal = new_goal;
  // }

}

void Agent::SetEgo(Eigen::VectorXf new_x) {
  prob->params.x_0 = new_x;
}

Eigen::VectorXf Agent::UpdateControls() {
  
  // Push latest Agent goal to SGD params
  prob->params.goals.clear();
  for (size_t i = 0; i < prob->params.ts_goal_check.size(); ++i) {
    prob->params.goals.push_back(goal);
  }
  
  // Prepare params using global params
  PrepareSGDParams();

  // Solve SGD
  float sgd_opt_cost;
  Eigen::VectorXf u_new = SGD::Solve(prob, opt_params, &sgd_opt_cost);
  prob->params.u_curr = 0.5f * (u_new + prob->params.u_curr); // Reciprocity
  return prob->params.u_curr;
}

float Agent::GetBestGoalCost(float dt, const Eigen::Vector2f& g_pos) {
  Eigen::Vector2f pos = prob->params.x_0.head<2>();
  float curr_dist = (pos - g_pos).norm();
  if (curr_dist < dt * prob->params.vel_limit) {
    return -curr_dist;
  }
  pos += prob->params.vel_limit * dt * (g_pos - pos).normalized();
  float new_dist = (pos - g_pos).norm();
  return prob->params.k_goal * (new_dist - curr_dist);
}

// Lower bound estimate of optimal cost
float Agent::GetBestCost() {
  float tot_cost = 0.0f;
  for (size_t i = 0; i < prob->params.ts_goal_check.size(); ++i) {
    tot_cost += GetBestGoalCost(prob->params.ts_goal_check[i], prob->params.goals[i]);
  }
  return tot_cost;
}

bool Agent::AtGoal() {
  return (prob->GetCollisionCenter(prob->params.x_0) - goal).norm() < 0.05f;
}

void Agent::SetStop() {
  prob->params.u_curr = Eigen::VectorXf::Zero(u_dim);
  if (a_type == AType::V) {
    // Nothing to do
  } else if (a_type == AType::A) {
    prob->params.x_0.tail<2>() = Eigen::VectorXf::Zero(2);
  } else if (a_type == AType::DD) {
    // Nothing to do
  } else if (a_type == AType::ADD) {
    prob->params.x_0.tail<2>() = Eigen::VectorXf::Zero(2);
  } else if (a_type == AType::CAR) {
    // Nothing to do
  } else if (a_type == AType::ACAR) {
    prob->params.x_0[3] = 0.0f;
  } else if (a_type == AType::MUSHR) {
    //Nothing to do
  }
}

void Agent::PrepareSGDParams() {
  float best_possible_cost = GetBestCost();
  opt_params.polyak_best = best_possible_cost;
  opt_params.x_0 = prob->params.u_curr;
  opt_params.x_lb = prob->params.u_lb;
  opt_params.x_ub = prob->params.u_ub;
}

void ConstructGlobalParams(SGDOptParams *opt_params) {
  opt_params->max_time = 10.0f;
  opt_params->opt_mode = OptMode::SGD;
  opt_params->alpha_mode = SGDAlphaMode::PolyakSemiKnown;
  opt_params->sk_mode = SGDSkMode::Filtered;
}

std::vector<std::string> GetAgentParts(int agent_type, Eigen::VectorXf& pos, bool reactive) {
  Eigen::Vector2f goal(pos[0], pos[1]); // Initialize goal to own position by default
  return GetAgentParts(agent_type, pos, reactive, goal);
}

std::vector<std::string> GetAgentParts(int agent_type, Eigen::VectorXf& pos, bool reactive, Eigen::Vector2f& goal) {
  std::string type;
  int p_dim, u_dim;
  switch (agent_type) {
    case 0:
      type = "v";
      p_dim = 2; u_dim = 2;
      break;
    case 1:
      type = "a";
      p_dim = 4; u_dim = 2;
      break;
    case 2:
      type = "dd";
      p_dim = 3; u_dim = 2;
      break;
    case 3:
      type = "add";
      p_dim = 5; u_dim = 2;
      break;
    case 4:
      type = "car";
      p_dim = 3; u_dim = 2;
      break;
    case 5:
      type = "acar";
      p_dim = 5; u_dim = 2;
      break;
    case 6:
      type = "mushr";
      p_dim = 3; u_dim = 2;
      break;
    default:
      std::cerr << "Invalid virtual agent type: " << agent_type << std::endl;
      exit(-1);
  }
  Eigen::VectorXf p = Eigen::VectorXf::Zero(p_dim);
  Eigen::VectorXf u = Eigen::VectorXf::Zero(u_dim);
  
  // Set XY Heading
  p[0] = pos[0];
  p[1] = pos[1];
  if (agent_type > 1) {
    p[2] = pos[2];
  }
  std::vector<std::string> parts(3 + p_dim + u_dim + 2);
  parts[0] = type;
  parts[1] = "y";
  parts[2] = (reactive ? "y" : "n");
  for (int i = 0; i < p.size(); ++i) {
    parts[3+i] = std::to_string(p[i]);
  }
  for (int i = 0; i < u.size(); ++i) {
    parts[3 + p_dim + i] = std::to_string(u[i]);
  }
  parts[3 + p_dim + u_dim] = std::to_string(goal[0]);
  parts[3 + p_dim + u_dim + 1] = std::to_string(goal[1]);

  return parts;
}
std::vector<TTCObstacle*> BuildObstacleList(std::vector<Agent> agents) {
  std::vector<TTCObstacle*> obsts;
  for (size_t a_idx = 0; a_idx < agents.size(); ++a_idx) {
    obsts.push_back(agents[a_idx].prob->CreateObstacle());
  }
  return obsts;
}

void SetAgentObstacleList(Agent& a, size_t a_idx, std::vector<TTCObstacle*> obsts) {
  a.prob->params.obsts.clear();
  if (!a.reactive) {
    return;
  }
  for (size_t b_idx = 0; b_idx < obsts.size(); ++b_idx) {
    if (b_idx == a_idx) {
      continue;
    }
    float dist = (a.prob->params.x_0.head<2>() - obsts[b_idx]->p.head<2>()).norm();
    // Ignore any obstacles that cannot interact with us within our ttc horizon
    if (dist < 2.0 * a.prob->params.vel_limit * a.prob->params.max_ttc) {
      a.prob->params.obsts.push_back(obsts[b_idx]);
    }
  }
}
