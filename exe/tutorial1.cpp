// Author: Akash Patel
// Date: 5/23/19

#include <ompl/base/SpaceInformation.h>      // ompl::base
#include <ompl/base/spaces/SE3StateSpace.h>  // make_shared<ob::SE3StateSpace>()
#include <ompl/geometric/PathSimplifier.h>   // PathSimplifier
#include <ompl/geometric/SimpleSetup.h>      // ompl::geometric, simpleSetup
#include <ompl/geometric/planners/rrt/RRTConnect.h>  // RRTConnect

//#include <ompl/config.h>
#include <iostream>
#include <vector>

/*
 * Identify the space we are planning in:
 *  SE(3)
 * Select a corresponding state space from the available ones, or implement one.
 *  ompl::base::SE3StateSpace
 * For SE(3) contains an R^3 component, we need to define bounds
 *  Choose 100x100x100
 * Define the notion of state validity
 *  I guess some collision
 * Define start states and a goal representation
 *  start = (25,25,25)
 *  goal = (75,75,75)
 */

namespace ob = ompl::base;
namespace og = ompl::geometric;

// Function Prototypes
// // Check if the passed state is valid/feasible
bool isStateValid(const ob::State *state);

// // Using the simple setup to plan
void planWithSimpleSetup();

// // Not using the simple setup to plan
void planWithOutSimpleSetup();

// Functions
// // Check if the passed state is valid/feasible
bool isStateValid(const ob::State *state) { return true; }

// // Using the simple setup to plan
void planWithSimpleSetup() {
  // construct the state space we are in planning in
  auto space(std::make_shared<ob::SE3StateSpace>());

  // set bounds for the R3 component of the state space
  ob::RealVectorBounds bounds(3);
  bounds.setLow(0, 0);
  bounds.setLow(1, 0);
  bounds.setLow(2, 0);
  bounds.setHigh(0, 100);
  bounds.setHigh(1, 100);
  bounds.setHigh(2, 100);

  space->setBounds(bounds);

  // create an instance of og::SimpleSetup. Instances of ob::SpaceInformation &
  // ob::ProblemDefinition

  og::SimpleSetup ss(space);

  // Set the stte validity checker
  ss.setStateValidityChecker(
      [](const ob::State *state) { return isStateValid(state); });

  // Create a start state
  ob::ScopedState<> start(space);
  std::vector<double> start_vec = {25, 25, 25, 1, 0, 0, 0};
  // orientation 1,0,0,0 is +x
  start = start_vec;
  std::cout << start << std::endl;
  // start.random();

  // std::cout << start.reals() << std::endl;

  // for (int i = 0; i < start.reals().size(); ++i) {
  //  std::cout << start[i] << ' ';
  //}
  // std::cout << " " << std::endl;

  ob::ScopedState<> goal(space);
  std::vector<double> goal_vec = {75, 75, 75, 0, 0, -1, 0};
  // orientation 0,0,-1,0 is -x
  goal = goal_vec;
  std::cout << goal << std::endl;

  // set these states as start and goal for simplesetup
  ss.setStartAndGoalStates(start, goal);

  // Solve the problem
  ob::PlannerStatus solved = ss.solve(1.0);
  std::cout << solved << std::endl;

  if (solved) {
    std::cout << "Found solution:" << std::endl;
    // print the path to screen
    ss.simplifySolution();
    ss.getSolutionPath().print(std::cout);
  }
}

// // Not using the simple setup to plan
void planWithOutSimpleSetup() {
  // construct the state space we are in planning in
  auto space(std::make_shared<ob::SE3StateSpace>());

  // set bounds for the R3 component of the state space
  ob::RealVectorBounds bounds(3);
  bounds.setLow(0, 0);
  bounds.setLow(1, 0);
  bounds.setLow(2, 0);
  bounds.setHigh(0, 100);
  bounds.setHigh(1, 100);
  bounds.setHigh(2, 100);

  space->setBounds(bounds);

  // Create an instance of ompl::base::SpaceInformation for the state space
  auto si(std::make_shared<ob::SpaceInformation>(space));
  // Set the state validity checker
  si->setStateValidityChecker(isStateValid);
  // Create a start state
  ob::ScopedState<> start(space);
  std::vector<double> start_vec = {25, 25, 25, 1, 0, 0, 0};
  // orientation 1,0,0,0 is +x
  start = start_vec;
  std::cout << start << std::endl;
  // start.random();

  // std::cout << start.reals() << std::endl;

  // for (int i = 0; i < start.reals().size(); ++i) {
  //  std::cout << start[i] << ' ';
  //}
  // std::cout << " " << std::endl;

  ob::ScopedState<> goal(space);
  std::vector<double> goal_vec = {75, 75, 75, 0, 0, -1, 0};
  // orientation 0,0,-1,0 is -x
  goal = goal_vec;
  std::cout << goal << std::endl;

  // Create an instance of ompl::base::ProblemDefinition
  auto pdef(std::make_shared<ob::ProblemDefinition>(si));

  // Set the start and goal states for the problem definition
  pdef->setStartAndGoalStates(start, goal);

  // create an instance of a planner
  auto planner(std::make_shared<og::RRTConnect>(si));

  // Tell the planner which problem we are interested in solving
  planner->setProblemDefinition(pdef);

  // Make sure all the settings for the space and planner are in order. This
  // will also lead to the runtime computation of the state validity checking
  // resolution.
  planner->setup();

  // Solve
  ob::PlannerStatus solved = planner->ob::Planner::solve(1.0);

  if (solved) {
    // Get the goal representation from the problem definition (not the same as
    // the goal state) and inquire about the found path

    // create a path simplifier
    // auto pathSimplifier(std::make_shared<og::PathSimplifier>(si));
    og::PathSimplifier ps(si);

    // ob::PathPtr path = pdef->getSolutionPath();
    og::PathGeometric path(
        dynamic_cast<const og::PathGeometric &>(*pdef->getSolutionPath()));
    // og::PathGeometric pathGeo = path->asGeometric();

    // ps.simplify(pathGeo, 5.0, true);
    ps.simplify(path, 5.0, true);

    std::cout << "Found solution:" << std::endl;

    // print the path to screen
    path.print(std::cout);
  }
}

// Main
int main() {
  std::cout << "Planning...\n" << std::endl;

  // planWithSimpleSetup();

  planWithOutSimpleSetup();

  std::cout << "|> Done" << std::endl;

  std::cout << "Program Finished." << std::endl;
  return 0;
}
