#include "rtt_searcher.h"

RttSearcher::RttSearcher(AstarPathFinder *grid_checker_, Eigen::Vector3d map_lower, Eigen::Vector3d map_upper) {
    space_ = std::make_shared<ob::RealVectorStateSpace>(3);
    map_lower_ = map_lower;
    map_upper_ = map_upper;
    // Set the bounds of space to be in [0,1].
    ob::RealVectorBounds bounds(3);
    bounds.setLow(0, map_lower_(0)-10);
    bounds.setLow(1, map_lower_(1)-10);
    bounds.setLow(2, map_lower_(2)-1);

    bounds.setHigh(0, map_upper_(0)+10);
    bounds.setHigh(1, map_upper_(1)+10);
    bounds.setHigh(2, map_upper_(2)+1);
    space_->as<ob::RealVectorStateSpace>()->setBounds(bounds);

    si_ = std::make_shared<ob::SpaceInformation>(space_);

    checker_ = std::make_shared<ValidityChecker>(si_, grid_checker_);
    si_->setStateValidityChecker(checker_);
    si_->setup();
}

RttSearcher::~RttSearcher() {
}


ob::OptimizationObjectivePtr RttSearcher::getPathLengthObjective(const ob::SpaceInformationPtr& si) {
  ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
  // auto obj = std::shared_ptr<ob::PathLengthOptimizationObjective>(si);
  // obj->setCostThreshold(ob::Cost(1.51)); // 设置阈值
  return obj;
}

std::vector<Eigen::Vector3d> RttSearcher::RttPathFinding(const Eigen::Vector3d start_pt, const Eigen::Vector3d target_pt) {
    ob::ScopedState<> start(space_);
    start->as<ob::RealVectorStateSpace::StateType>()->values[0] = start_pt.x();
    start->as<ob::RealVectorStateSpace::StateType>()->values[1] = start_pt.y();
    start->as<ob::RealVectorStateSpace::StateType>()->values[2] = start_pt.z();

    ob::ScopedState<> goal(space_);
    goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = target_pt.x();
    goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = target_pt.y();
    goal->as<ob::RealVectorStateSpace::StateType>()->values[2] = target_pt.z();

    ob::ProblemDefinitionPtr pdef = std::make_shared<ob::ProblemDefinition>(si_);
    pdef->setStartAndGoalStates(start, goal);
    pdef->setOptimizationObjective(getPathLengthObjective(si_));

    ob::PlannerPtr optimizingPlanner =  std::make_shared<og::RRTstar>(si_);
    optimizingPlanner->setProblemDefinition(pdef);
    optimizingPlanner->setup();

    ob::PlannerStatus solved = optimizingPlanner->solve(0.1);
    
    std::vector<Eigen::Vector3d> path_points;

    if (solved) {
        og::PathGeometric* path = pdef->getSolutionPath()->as<og::PathGeometric>();

        for (size_t path_idx = 0; path_idx < path->getStateCount (); path_idx++) {
            const ob::RealVectorStateSpace::StateType *state = path->getState(path_idx)->as<ob::RealVectorStateSpace::StateType>(); 
            path_points.emplace_back(state->values[0], state->values[1], state->values[2]);
        }
    }
    return path_points;
}
