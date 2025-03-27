///////////////////////////////////////
// RBE 550
// Project 4
// Authors: Pranay Katyal
//////////////////////////////////////

#ifndef RGRRT_H
#define RGRRT_H

#include <ompl/base/Planner.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/PlannerData.h>

namespace ompl
{
    namespace control
    {
        // A minimal implementation of Reachability-Guided RRT (RG-RRT)
        class RG_RRT : public base::Planner
        {
        public:
            RG_RRT(const base::SpaceInformationPtr &si) : base::Planner(si, "RG_RRT")
            {
                specs_.approximateSolutions = true;
                setName("RG_RRT");
            }
            ~RG_RRT(void) override {}

            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;
            void clear() override;
            void getPlannerData(base::PlannerData &data) const override;

        protected:
            // A simple motion structure to store states and parent pointers.
            struct Motion
            {
                base::State *state;
                Motion *parent;
                Motion(base::State *s) : state(s), parent(nullptr) {}
            };
            std::vector<Motion*> motions_;
        };
    } // namespace control
} // namespace ompl

#endif