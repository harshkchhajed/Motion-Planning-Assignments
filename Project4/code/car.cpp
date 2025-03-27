#include <iostream>
#include <vector>
#include <cmath>
#include <memory>

// Include the necessary OMPL headers for state and control spaces.
#include <ompl/base/ProjectionEvaluator.h>
#include <ompl/base/State.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>      // Added
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>   // Added
#include <ompl/control/planners/rrt/RRT.h>

#include "CollisionChecking.h"
#include "RG-RRT.h"  // Placeholder for your RG-RRT implementation

#include <Eigen/Core>

namespace ob = ompl::base;
namespace oc = ompl::control;

//---------------------------------------------------------
// Projection for the Car: reduce 4D state [x, y, theta, v]
// to a 2D projection (x, y)
class CarProjection : public ob::ProjectionEvaluator
{
public:
    CarProjection(const ob::StateSpace *space) : ProjectionEvaluator(space) {}

    unsigned int getDimension() const override
    {
        return 2;
    }

    void project(const ob::State *state, Eigen::Ref<Eigen::VectorXd> projection) const override
    {
        // The state is a RealVectorStateSpace state of dimension 4.
        const double *vals = state->as<ob::RealVectorStateSpace::StateType>()->values;
        projection[0] = vals[0]; // x
        projection[1] = vals[1]; // y
    }
};

//---------------------------------------------------------
// ODE for the Car dynamics.
// q = [x, y, theta, v] and control u = [omega, a]
// where:
//   x_dot = v * cos(theta)
//   y_dot = v * sin(theta)
//   theta_dot = omega
//   v_dot = a
void carODE(const oc::ODESolver::StateType &q, const oc::Control *control,
            oc::ODESolver::StateType &qdot)
{
    const double *u = control->as<oc::RealVectorControlSpace::ControlType>()->values;
    double omega = u[0];
    double a     = u[1];
    double theta = q[2];
    double v     = q[3];

    qdot.resize(q.size(), 0);
    qdot[0] = v * std::cos(theta);  // x_dot
    qdot[1] = v * std::sin(theta);  // y_dot
    qdot[2] = omega;                // theta_dot
    qdot[3] = a;                    // v_dot
}

//---------------------------------------------------------
// Create a simple street environment by filling the obstacles vector.
void makeStreet(std::vector<Rectangle> &obstacles)
{
    // Example obstacles – adjust these to suit your environment.
    obstacles.push_back({-2.0, -2.0, 4.0, 1.0});
    obstacles.push_back({3.0, 0.0, 2.0, 3.0});
    obstacles.push_back({-8.0, 4.0, 3.0, 2.0});
}

//---------------------------------------------------------
// Validity checker for the car state.
// We check the (x,y) location using our collision-checking routine.
bool isCarStateValid(const ob::State *state, const std::vector<Rectangle> &obstacles)
{
    const double *vals = state->as<ob::RealVectorStateSpace::StateType>()->values;
    double x = vals[0];
    double y = vals[1];
    return isValidPoint(x, y, obstacles);
}

//---------------------------------------------------------
// Create and setup the car's state space, control space, state propagator,
// validity checker, and projection evaluator.
oc::SimpleSetupPtr createCar(const std::vector<Rectangle> &obstacles)
{
    // State space: 4-dimensional [x, y, theta, v]
    ob::StateSpacePtr stateSpace(new ob::RealVectorStateSpace(4));
    ob::RealVectorBounds bounds(4);
    bounds.setLow(0, -10.0);  // x lower bound
    bounds.setHigh(0, 10.0);  // x upper bound
    bounds.setLow(1, -10.0);  // y lower bound
    bounds.setHigh(1, 10.0);  // y upper bound
    bounds.setLow(2, -M_PI);  // theta lower bound
    bounds.setHigh(2, M_PI);  // theta upper bound
    bounds.setLow(3, -5.0);   // velocity lower bound
    bounds.setHigh(3, 5.0);   // velocity upper bound
    stateSpace->as<ob::RealVectorStateSpace>()->setBounds(bounds);

    // Control space: 2-dimensional [omega, acceleration]
    oc::ControlSpacePtr controlSpace(new oc::RealVectorControlSpace(stateSpace, 2));
    ob::RealVectorBounds cbounds(2);
    cbounds.setLow(0, -1.0); // omega lower bound
    cbounds.setHigh(0, 1.0); // omega upper bound
    cbounds.setLow(1, -1.0); // acceleration lower bound
    cbounds.setHigh(1, 1.0); // acceleration upper bound
    controlSpace->as<oc::RealVectorControlSpace>()->setBounds(cbounds);

    // Create the SimpleSetup object.
    oc::SimpleSetupPtr ss(new oc::SimpleSetup(controlSpace));

    // Use a concrete ODE solver. Here we use ODEBasicSolver with our carODE.
    oc::ODESolverPtr odeSolver(new oc::ODEBasicSolver<>(ss->getSpaceInformation(), carODE));
    ss->getSpaceInformation()->setStatePropagator(
        oc::ODESolver::getStatePropagator(odeSolver, nullptr) // Pass nullptr as post propagation event.
    );

    // Set the state validity checker (using our collision-checking function).
    ss->setStateValidityChecker([obstacles](const ob::State *state) -> bool {
        return isCarStateValid(state, obstacles);
    });

    // Register the projection evaluator.
    stateSpace->registerProjection("CarProjection", std::make_shared<CarProjection>(stateSpace.get()));

    return ss;
}

//---------------------------------------------------------
// Perform motion planning for the car using the chosen planner.
void planCar(oc::SimpleSetupPtr &ss, int choice)
{
    // Define start state: e.g., (-8, -8, 0, 0)
    ob::ScopedState<> start(ss->getStateSpace());
    start[0] = -8.0;
    start[1] = -8.0;
    start[2] = 0.0;
    start[3] = 0.0;

    // Define goal state: e.g., (8, 8, 0, 0)
    ob::ScopedState<> goal(ss->getStateSpace());
    goal[0] = 8.0;
    goal[1] = 8.0;
    goal[2] = 0.0;
    goal[3] = 0.0;

    ss->setStartAndGoalStates(start, goal, 0.5);

    ss->getSpaceInformation()->setMinMaxControlDuration(1, 10);
    ss->getSpaceInformation()->setPropagationStepSize(0.01);

    // Choose planner based on user input.
    if (choice == 1)
    {
        ss->setPlanner(ob::PlannerPtr(new oc::RRT(ss->getSpaceInformation())));
    }
    else if (choice == 2)
    {
        // For example, use KPIECE1 (if implemented).
        ss->setPlanner(ob::PlannerPtr(new oc::RRT(ss->getSpaceInformation()))); // fallback to RRT
    }
    else if (choice == 3)
    {
        // Use RG-RRT if available.
        ss->setPlanner(ob::PlannerPtr(new oc::RRT(ss->getSpaceInformation()))); // fallback to RRT
    }
    else
    {
        std::cerr << "Invalid planner choice. Defaulting to RRT." << std::endl;
        ss->setPlanner(ob::PlannerPtr(new oc::RRT(ss->getSpaceInformation())));
    }

    // Solve the planning problem.
    ob::PlannerStatus solved = ss->solve(5.0);
    if (solved)
    {
        std::cout << "Car plan found:" << std::endl;
        ss->getSolutionPath().printAsMatrix(std::cout);
    }
    else
    {
        std::cout << "No solution found for the car problem." << std::endl;
    }
}

//---------------------------------------------------------
// Dummy benchmarking function for the car.
void benchmarkCar(oc::SimpleSetupPtr &ss)
{
    std::cout << "Benchmarking for car not implemented." << std::endl;
}

//---------------------------------------------------------
// Main function for the car planning executable.
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
            std::cout << "Select Planner:" << std::endl;
            std::cout << " (1) RRT" << std::endl;
            std::cout << " (2) KPIECE1" << std::endl;
            std::cout << " (3) RG-RRT" << std::endl;
            std::cin >> planner;
        } while (planner < 1 || planner > 3);
        planCar(ss, planner);
    }
    else if (choice == 2)
    {
        benchmarkCar(ss);
    }
    else
    {
        std::cerr << "Invalid choice." << std::endl;
    }

    return 0;
}
