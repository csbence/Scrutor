//
// Created by Bence Cserna on 10/21/15.
//

#ifndef REALTIMESEARCH_ABSTRACTDOMAIN_HPP
#define REALTIMESEARCH_ABSTRACTDOMAIN_HPP

namespace rts {

class AbstractDomain {
 public:
  typedef State;

  virtual State getStartState() = 0;
  virtual std::vector expand(State state) = 0;
  virtual double heuristicValue(State) = 0;
  virtual double distanceEstimate(State) = 0;
  virtual bool isGoal(State state) = 0;
};

}

#endif //REALTIMESEARCH_ABSTRACTDOMAIN_HPP
