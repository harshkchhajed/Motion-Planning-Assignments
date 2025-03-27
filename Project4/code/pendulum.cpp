#include <iostream>
#include <cmath>
#include <memory>

// Include the necessary OMPL headers.
#include <ompl/base/ProjectionEvaluator.h>
#include <ompl/base/State.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>      // Added
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>   // Added
#include <ompl/control/planners/rrt/RRT.h>

#include "RG-RRT.h"  // Placeholder for your RG-RRT implementation

namespace ob = ompl::base;
namespace oc = ompl::control;

//---------------------------------------------------------
// Projection for the Pendulum: reduce 2D state [theta, omega] to 1D (theta)
class PendulumProjection : public ob::ProjectionEvaluator
{
public:
    PendulumProjection(const ob::StateSpace *space) : ProjectionEvaluator(space) {}

    unsigned int getDimension() const override
    {
        return 1;
    }

    void project(const ob::State *state, Eigen::Ref<Eigen::VectorXd> projection) const override
    {
        // The state is a RealVectorStateSpace state of dimension 2.
        const double *vals = state->as<ob::RealVectorStateSpace::StateType>()->values;
        projection[0] = vals[0]; // theta
    }
};

//---------------------------------------------------------
// ODE for the Pendulum dynamics.
// q = [theta, omega] and control u = [tau]
// where:
//   theta_dot = omega
//   omega_dot = -g*cos(theta) + tau
void pendulumODE(const oc::ODESolver::StateType &q, const oc::Control *control,
                 oc::ODESolver::StateType &qdot)
{
    const double g = 9.81;
    const double *u = control->as<oc::RealVectorControlSpace::ControlType>()->values;
    double tau = u[0];

    qdot.resize(q.size(), 0);
    qdot[0] = q[1];                     // theta_dot = omega
    qdot[1] = -g * std::cos(q[0]) + tau;  // omega_dot = -g*cos(theta) + tau
}

//---------------------------------------------------------
// Validity checker for the pendulum state.
// Here we ensure that |omega| is <= 10.
bool isPendulumStateValid(const ob::State *state)
{
    const double *vals = state->as<ob::RealVectorStateSpace::StateType>()->values;
    double omega = vals[1];
    return (std::fabs(omega) <= 10.0);
}

//---------------------------------------------------------
// Create and setup the pendulum's state space, control space, state propagator,
// and state validity checker.
oc::SimpleSetupPtr createPendulum(double torqueBound)
{
    // State space: 2-dimensional [theta, omega]
    ob::StateSpacePtr stateSpace(new ob::RealVectorStateSpace(2));
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0, -M_PI);
    bounds.setHigh(0, M_PI);
    bounds.setLow(1, -10.0);
    bounds.setHigh(1, 10.0);
    stateSpace->as<ob::RealVectorStateSpace>()->setBounds(bounds);

    // Control space: 1-dimensional for torque.
    oc::ControlSpacePtr controlSpace(new oc::RealVectorControlSpace(stateSpace, 1));
    ob::RealVectorBounds cbounds(1);
    cbounds.setLow(-torqueBound);
    cbounds.setHigh(torqueBound);
    controlSpace->as<oc::RealVectorControlSpace>()->setBounds(cbounds);

    oc::SimpleSetupPtr ss(new oc::SimpleSetup(controlSpace));

    // Use ODEBasicSolver with pendulumODE.
    oc::ODESolverPtr odeSolver(new oc::ODEBasicSolver<>(ss->getSpaceInformation(), pendulumODE));
    ss->getSpaceInformation()->setStatePropagator(
        oc::ODESolver::getStatePropagator(odeSolver, nullptr)
    );

    ss->setStateValidityChecker(isPendulumStateValid);

    // Register the projection evaluator.
    stateSpace->registerProjection("PendulumProjection", std::make_shared<PendulumProjection>(stateSpace.get()));

    return ss;
}

//---------------------------------------------------------
// Perform motion planning for the pendulum using the chosen planner.
void planPendulum(oc::SimpleSetupPtr &ss, int choice)
{
    // Define start state: (-pi/2, 0)
    ob::ScopedState<> start(ss->getStateSpace());
    start[0] = -M_PI / 2;
    start[1] = 0.0;

    // Define goal state: (pi/2, 0)
    ob::ScopedState<> goal(ss->getStateSpace());
    goal[0] = M_PI / 2;
    goal[1] = 0.0;

    ss->setStartAndGoalStates(start, goal, 0.1);

    ss->getSpaceInformation()->setMinMaxControlDuration(1, 10);
    ss->getSpaceInformation()->setPropagationStepSize(0.01);

    if (choice == 1)
    {
        ss->setPlanner(ob::PlannerPtr(new oc::RRT(ss->getSpaceInformation())));
    }
    else if (choice == 2)
    {
        ss->setPlanner(ob::PlannerPtr(new oc::RRT(ss->getSpaceInformation())));
    }
    else if (choice == 3)
    {
        ss->setPlanner(ob::PlannerPtr(new oc::RRT(ss->getSpaceInformation())));
    }
    else
    {
        std::cerr << "Invalid planner choice. Defaulting to RRT." << std::endl;
        ss->setPlanner(ob::PlannerPtr(new oc::RRT(ss->getSpaceInformation())));
    }

    ob::PlannerStatus solved = ss->solve(5.0);
    if (solved)
    {
        std::cout << "Pendulum plan found:" << std::endl;
        ss->getSolutionPath().printAsMatrix(std::cout);
    }
    else
    {
        std::cout << "No solution found for the pendulum problem." << std::endl;
    }
}

//---------------------------------------------------------
// Dummy benchmarking function for the pendulum.
void benchmarkPendulum(oc::SimpleSetupPtr &ss)
{
    std::cout << "Benchmarking for pendulum not implemented." << std::endl;
}

//---------------------------------------------------------
// Main function for the pendulum planning executable.
int main(int /* argc */, char ** /* argv */)
{
    int choice;
    do
    {
        std::cout << "Plan or Benchmark? " << std::endl;
        std::cout << " (1) Plan" << std::endl;
        std::cout << " (2) Benchmark" << std::endl;
        std::cin >> choice;
    } while (choice < 1 || choice > 2);

    int planner;
    do
    {
        std::cout << "Select Planner:" << std::endl;
        std::cout << " (1) RRT" << std::endl;
        std::cout << " (2) KPIECE1" << std::endl;
        std::cout << " (3) RG-RRT" << std::endl;
        std::cin >> planner;
    } while (planner < 1 || planner > 3);

    double torques[] = {3.0, 5.0, 10.0};
    int which;
    do
    {
        std::cout << "Torque? " << std::endl;
        std::cout << " (1) 3" << std::endl;
        std::cout << " (2) 5" << std::endl;
        std::cout << " (3) 10" << std::endl;
        std::cin >> which;
    } while (which < 1 || which > 3);
    double torque = torques[which - 1];

    oc::SimpleSetupPtr ss = createPendulum(torque);

    if (choice == 1)
    {
        planPendulum(ss, planner);
    }
    else if (choice == 2)
    {
        benchmarkPendulum(ss);
    }
    else
    {
        std::cerr << "Invalid choice." << std::endl;
    }
    return 0;
}
