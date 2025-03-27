///////////////////////////////////////
// RBE 550
// Project 4
// Authors: Pranay Katyal
//////////////////////////////////////

#include <iostream>
#include <cmath>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/ProjectionEvaluator.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/ODESolver.h>
#include <ompl/base/PlannerStatus.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include "RG-RRT.h"

namespace ob = ompl::base;
namespace oc = ompl::control;

const double GRAVITY = 9.81;

// --- Pendulum Projection: simply use [theta, omega]
class PendulumProjection : public ob::ProjectionEvaluator
{
public:
    PendulumProjection(const ob::StateSpace *space) : ProjectionEvaluator(space) {}
    unsigned int getDimension() const override { return 2; }
    void project(const ob::State *state, Eigen::Ref<Eigen::VectorXd> projection) const override
    {
        const double *vals = state->as<ob::RealVectorStateSpace::StateType>()->values;
        projection[0] = vals[0]; // theta
        projection[1] = vals[1]; // omega
    }
};

// --- Pendulum dynamics ODE
// State: [theta, omega]; Control: [tau]
void pendulumODE(const oc::ODESolver::StateType &q, const oc::Control *control,
                 oc::ODESolver::StateType &qdot)
{
    const double *u = control->as<oc::RealVectorControlSpace::ControlType>()->values;
    double theta = q[0];
    double omega = q[1];
    qdot[0] = omega;                   // theta_dot
    qdot[1] = -GRAVITY * cos(theta) + u[0]; // omega_dot = -g*cos(theta) + tau
}

// --- Create the pendulum planning setup; torque is maximum absolute torque.
oc::SimpleSetupPtr createPendulum(double torque)
{
    // Create a 2D state space: [theta, omega]
    ob::StateSpacePtr stateSpace(new ob::RealVectorStateSpace(2));
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0, -M_PI);  bounds.setHigh(0, M_PI); // theta bounds
    bounds.setLow(1, -10);    bounds.setHigh(1, 10);     // omega bounds
    stateSpace->as<ob::RealVectorStateSpace>()->setBounds(bounds);

    // Control space: 1D, tau ∈ [ -torque, torque ]
    oc::ControlSpacePtr controlSpace(new oc::RealVectorControlSpace(stateSpace, 1));
    ob::RealVectorBounds cbounds(1);
    cbounds.setLow(-torque); cbounds.setHigh(torque);
    controlSpace->as<oc::RealVectorControlSpace>()->setBounds(cbounds);

    oc::SimpleSetupPtr ss(new oc::SimpleSetup(controlSpace));
    ss->setStateSpace(stateSpace);

    // Validity: here we only check that |omega| <= 10 (already in bounds)
    ss->setStateValidityChecker([](const ob::State *state) -> bool {
        const double *vals = state->as<ob::RealVectorStateSpace::StateType>()->values;
        return fabs(vals[1]) <= 10;
    });

    // Set the ODE with an integration step of 0.05 sec.
    oc::ODESolverPtr odeSolver(new oc::ODESolver(ss->getSpaceInformation(), pendulumODE));
    ss->setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver, 0.05));

    // Register projection for KPIECE1.
    stateSpace->registerProjection("PendulumProjection", std::make_shared<PendulumProjection>(stateSpace.get()));

    // Set start state: [theta = -pi/2, omega = 0]
    ob::ScopedState<> start(stateSpace);
    start->as<ob::RealVectorStateSpace::StateType>()->values[0] = -M_PI/2;
    start->as<ob::RealVectorStateSpace::StateType>()->values[1] = 0;
    
    // Set goal state: [theta = pi/2, omega = 0]
    ob::ScopedState<> goal(stateSpace);
    goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = M_PI/2;
    goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = 0;

    ss->setStartAndGoalStates(start, goal, 0.1);

    return ss;
}

// --- Plan the pendulum motion using the chosen planner.
void planPendulum(oc::SimpleSetupPtr &ss, int choice)
{
    ob::PlannerPtr planner;
    if (choice == 1)
        planner = std::make_shared<oc::RRT>(ss->getSpaceInformation());
    else if (choice == 2)
        planner = std::make_shared<oc::KPIECE1>(ss->getSpaceInformation());
    else if (choice == 3)
        planner = std::make_shared<oc::RG_RRT>(ss->getSpaceInformation());
    else
    {
        std::cerr << "Invalid planner choice." << std::endl;
        return;
    }
    ss->setPlanner(planner);
    ob::PlannerStatus solved = ss->solve(10.0);
    if (solved)
    {
        std::cout << "Pendulum solution found!" << std::endl;
        ss->getSolutionPath().printAsMatrix(std::cout);
    }
    else
        std::cout << "No solution found for pendulum." << std::endl;
}

// --- Benchmark the pendulum planning (placeholder)
void benchmarkPendulum(oc::SimpleSetupPtr &ss)
{
    ob::PlannerStatus solved = ss->solve(10.0);
    if (solved)
        std::cout << "Pendulum benchmark: solution found." << std::endl;
    else
        std::cout << "Pendulum benchmark: no solution found." << std::endl;
}

int main(int /* argc /, char * /* argv */)
{
    int choice;
    do
    {
        std::cout << "Plan or Benchmark? " << std::endl;
        std::cout << " (1) Plan" << std::endl;
        std::cout << " (2) Benchmark" << std::endl;
        std::cin >> choice;
    } while (choice < 1 || choice > 2);

    int which;
    do
    {
        std::cout << "Select Torque Bound: " << std::endl;
        std::cout << " (1)  3" << std::endl;
        std::cout << " (2)  5" << std::endl;
        std::cout << " (3) 10" << std::endl;
        std::cin >> which;
    } while (which < 1 || which > 3);

    double torques[] = {3.0, 5.0, 10.0};
    double torque = torques[which - 1];

    oc::SimpleSetupPtr ss = createPendulum(torque);

    if (choice == 1)
    {
        int planner;
        do
        {
            std::cout << "Choose Planner: " << std::endl;
            std::cout << " (1) RRT" << std::endl;
            std::cout << " (2) KPIECE1" << std::endl;
            std::cout << " (3) RG-RRT" << std::endl;
            std::cin >> planner;
        } while (planner < 1 || planner > 3);
        planPendulum(ss, planner);
    }
    else if (choice == 2)
        benchmarkPendulum(ss);
    else
        std::cerr << "Invalid choice." << std::endl;

    return 0;
}