///////////////////////////////////////
// RBE550
// Project 4
// Authors: FILL ME OUT!!Pranay Katyal//////////////////////////

#include "RG-RRT.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <limits>

namespace ob = ompl::base;
namespace oc = ompl::control;

ob::PlannerStatus oc::RG_RRT::solve(const ob::PlannerTerminationCondition &ptc)
{
    auto si = getSpaceInformation();
    if (motions_.empty())
    {
        // Initialize tree with the start state.
        Motion *motion = new Motion(si->cloneState(si->getStartState(0)));
        motions_.push_back(motion);
    }

    ob::State *randomState = si->allocState();

    while (!ptc())
    {
        si->getStateSpace()->sampleUniform(randomState);

        // Find nearest motion using Euclidean distance.
        Motion *nearest = nullptr;
        double minDist = std::numeric_limits<double>::infinity();
        for (auto m : motions_)
        {
            double dist = si->distance(m->state, randomState);
            if (dist < minDist)
            {
                minDist = dist;
                nearest = m;
            }
        }
        if (!nearest)
            continue;

        // Propagate from the nearest state with a random control for a fixed duration.
        ob::State *newState = si->allocState();
        oc::Control *control = si->allocControl();
        // Sample control uniformly from the control space.
        si->getControlSpace()->sampleUniform(control);
        double duration = 0.1; // fixed time step
        si->propagate(nearest->state, control, duration, newState);

        if (si->isValid(newState))
        {
            Motion *motion = new Motion(si->cloneState(newState));
            motion->parent = nearest;
            motions_.push_back(motion);

            // Check if newState is within goal tolerance.
            if (si->distance(newState, si->getGoal()->as<ob::GoalState>()->getState()) < si->getGoalTolerance())
            {
                // Build solution path.
                ob::PathPtr path(new ob::Path(si->getStateSpace()));
                for (Motion *m = motion; m != nullptr; m = m->parent)
                    path->append(m->state);
                // Reverse path order.
                ob::PathPtr revPath = path->reverse();
                goal_->setSolutionPath(revPath);
                si->freeState(randomState);
                si->freeControl(control);
                return base::PlannerStatus::EXACT_SOLUTION;
            }
        }
        si->freeState(newState);
        si->freeControl(control);
    }
    si->freeState(randomState);
    return base::PlannerStatus::TIMEOUT;
}

void oc::RG_RRT::clear()
{
    for (auto m : motions_)
    {
        getSpaceInformation()->freeState(m->state);
        delete m;
    }
    motions_.clear();
    base::Planner::clear();
}

void oc::RG_RRT::getPlannerData(ob::PlannerData &data) const
{
    data.clear();
    for (auto m : motions_)
    {
        data.addVertex(m->state);
        if (m->parent)
            data.addEdge(m->parent->state, m->state);
    }
}
