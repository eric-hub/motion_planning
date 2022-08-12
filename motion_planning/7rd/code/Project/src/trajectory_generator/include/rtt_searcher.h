#ifndef _DCK_RRT_SEARCHER_H
#define _DCK_RRT_SEARCHER_H

#include "Astar_searcher.h"

#include <ompl/config.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/Path.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/LazyPRM.h>
#include <ompl/geometric/SimpleSetup.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;


// Our collision checker. For this demo, our robot's state space
class ValidityChecker : public ob::StateValidityChecker {
    private:
        AstarPathFinder *grid_checker_;
    public:
        ValidityChecker(const ob::SpaceInformationPtr& si, AstarPathFinder *checker_ptr) :
            ob::StateValidityChecker(si){
                grid_checker_ = checker_ptr;
        }
        bool isValid(const ob::State* state) const {
            const ob::RealVectorStateSpace::StateType* state3D =
                state->as<ob::RealVectorStateSpace::StateType>();
            Eigen::Vector3d pt;
            Eigen::Vector3i idx;

            pt(0) = state3D->values[0];
            pt(1) = state3D->values[1];
            pt(2) = state3D->values[2];
            idx = grid_checker_->coord2gridIndex(pt);

            return !grid_checker_->isOccupied(idx);
        }
};

class RttSearcher {
private:
    Eigen::Vector3d map_lower_;
    Eigen::Vector3d map_upper_;
    ob::StateSpacePtr space_;
    ob::SpaceInformationPtr si_;
    std::shared_ptr<ValidityChecker> checker_;
public:
    RttSearcher(AstarPathFinder *grid_checker_, Eigen::Vector3d map_lower, Eigen::Vector3d map_upper);
    ~RttSearcher();
    std::vector<Eigen::Vector3d> RttPathFinding(const Eigen::Vector3d start_pt, const Eigen::Vector3d target_pt);
    ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si);
};



#endif