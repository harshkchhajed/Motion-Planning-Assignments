///////////////////////////////////////
// RBE 550
// Project 4
// Authors: FILL ME OUT!!
//////////////////////////////////////

#include <iostream>
#include <cmath>
#include <vector>
#include <memory>

#include <ompl/base/ProjectionEvaluator.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

#include "CollisionChecking.h"
#include "RG-RRT.h"

namespace ob = ompl::base;
namespace oc = ompl::control;

// Your projection for the car
class CarProjection : public ob::ProjectionEvaluator
{
public:
    CarProjection(const ob::StateSpace *space) : ProjectionEvaluator(space)
    {
    }

    unsigned int getDimension() const override
    {
        // TODO: set appropriate projection dimension (for example, 2 for (x,y))
        return 2;
    }

    void project(const ob::State *state, Eigen::Ref<Eigen::VectorXd> projection) const override
    {
        // Assume state is [x, y, theta, v]
        const double *vals = state->as<ob::RealVectorStateSpace::StateType>()->values;
        projection[0] = vals[0]; // x
        projection[1] = vals[1]; // y
    }
};

// --- Car Dynamics ODE ---
// State: [x, y, theta, v]; Control: [omega, a]
void carODE(const oc::ODESolver::StateType &q, const oc::Control *control,
            oc::ODESolver::StateType &qdot)
{
    // Using RealVectorControlSpace, so include the header above.
    const double *u = control->as<oc::RealVectorControlSpace::ControlType>()->values;
    double theta = q[2];
    double v = q[3];
    qdot[0] = v * cos(theta); // x_dot
    qdot[1] = v * sin(theta); // y_dot
    qdot[2] = u[0];           // theta_dot (angular rate)
    qdot[3] = u[1];           // v_dot (acceleration)
}

void makeStreet(std::vector<Rectangle> &obstacles)
{
    // Example: create two obstacles.
    Rectangle r1; r1.x = 2.0; r1.y = 2.0; r1.width = 2.0; r1.height = 1.0;
    obstacles.push_back(r1);
    Rectangle r2; r2.x = -3.0; r2.y = -1.0; r2.width = 1.5; r2.height = 3.0;
    obstacles.push_back(r2);
}

oc::SimpleSetupPtr createCar(std::vector<Rectangle> &obstacles)
{
    // Create a 4D state space: [x, y, theta, v]
    ob::StateSpacePtr stateSpace(new ob::RealVectorStateSpace(4));
    ob::RealVectorBounds bounds(4);
    bounds.setLow(0, -10); bounds.setHigh(0, 10);   // x bounds
    bounds.setLow(1, -10); bounds.setHigh(1, 10);   // y bounds
    bounds.setLow(2, -M_PI); bounds.setHigh(2, M_PI); // theta bounds
    bounds.setLow(3, -5); bounds.setHigh(3, 5);       // v bounds
    stateSpace->as<ob::RealVectorStateSpace>()->setBounds(bounds);

    // Create a 2D control space: [omega, a]
    oc::ControlSpacePtr controlSpace(new oc::RealVectorControlSpace(stateSpace, 2));
    ob::RealVectorBounds cbounds(2);
    cbounds.setLow(0, -1); cbounds.setHigh(0, 1);
    cbounds.setLow(1, -1); cbounds.setHigh(1, 1);
    controlSpace->as<oc::RealVectorControlSpace>()->setBounds(cbounds);

    // Create the SimpleSetup using the control space.
    oc::SimpleSetupPtr ss(new oc::SimpleSetup(controlSpace));

    // Set the state validity checker: ensure (x,y) is collision-free.
    ss->setStateValidityChecker([&obstacles](const ob::State *state) -> bool {
        const double *vals = state->as<ob::RealVectorStateSpace::StateType>()->values;
        double x = vals[0], y = vals[1];
        return isValidPoint(x, y, obstacles);
    });

    // Create a concrete ODE solver using the ODEBasicSolver (Euler integration)
    oc::ODESolverPtr odeSolver(new oc::ODEBasicSolver<>(ss->getSpaceInformation(), carODE));

    // Instead of using getStatePropagator, set a lambda that calls the ODE solver.
    ss->setStatePropagator([odeSolver](const ob::State *state, const oc::Control *control, double duration, ob::State *result)
    {
        odeSolver->solve(*state, control, duration, result);
    });

    // Register the projection for KPIECE1
    stateSpace->registerProjection("CarProjection", std::make_shared<CarProjection>(stateSpace.get()));

    // Define start and goal states.
    ob::ScopedState<> start(stateSpace);
    start->as<ob::RealVectorStateSpace::StateType>()->values[0] = -8;
    start->as<ob::RealVectorStateSpace::StateType>()->values[1] = -8;
    start->as<ob::RealVectorStateSpace::StateType>()->values[2] = 0;
    start->as<ob::RealVectorStateSpace::StateType>()->values[3] = 0;
    
    ob::ScopedState<> goal(stateSpace);
    goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = 8;
    goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = 8;
    goal->as<ob::RealVectorStateSpace::StateType>()->values[2] = 0;
    goal->as<ob::RealVectorStateSpace::StateType>()->values[3] = 0;

    ss->setStartAndGoalStates(start, goal, 0.5); // 0.5 tolerance

    return ss;
}

void planCar(oc::SimpleSetupPtr &ss, int choice)
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
        std::cout << "Car solution found!" << std::endl;
        ss->getSolutionPath().printAsMatrix(std::cout);
    }
    else
        std::cout << "No solution found for car." << std::endl;
}

void benchmarkCar(oc::SimpleSetupPtr &ss)
{
    ob::PlannerStatus solved = ss->solve(10.0);
    if (solved)
        std::cout << "Car benchmark: solution found." << std::endl;
    else
        std::cout << "Car benchmark: no solution found." << std::endl;
}

int main(int /* argc */, char ** /* argv */)
{
    std::vector<Rectangle> obstacles;
    makeStreet(obstacles);

    int choice;
    do
    {
        std::cout << "Plan or Benchmark? " << std::endl;
        std::cout << " (1) Plan" << std::endl;
        std::cout << " (2) Benchmark" << std::endl;
        std::cin >> choice;
    } while (choice < 1 || choice > 2);

    oc::SimpleSetupPtr ss = createCar(obstacles);

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
        planCar(ss, planner);
    }
    else if (choice == 2)
        benchmarkCar(ss);
    else
        std::cerr << "Invalid choice." << std::endl;

    return 0;
}
