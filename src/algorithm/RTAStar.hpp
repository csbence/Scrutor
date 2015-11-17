//
// Created by Bence Cserna on 10/19/15.
//
#ifndef REALTIMESEARCH_A_STAR_H
#define REALTIMESEARCH_A_STAR_H

#include "easylogging++.h"
#include <queue>
#include <vector>
#include <unordered_set>
#include <boost/optional.hpp>
#include <boost/pool/object_pool.hpp>

namespace rts {

template<typename Domain>
class RTAStar {
 public:
  typedef typename Domain::State State;

  class Node {
   public:
    Node(State state) :
        state(std::move(state)),
        parent(NULL),
        heuristicValue(0),
        gValue(0) {
    }

    Node(State state, Node* parent, double h, double g) :
        state(std::move(state)),
        parent(parent),
        heuristicValue(h),
        gValue(g) {
    }

    void setParent(Node* parent) {
      this->parent = parent;
    }

    Node& getParent() const {
      return *parent;
    }

    bool hasParent() const {
      return parent == nullptr;
    }

    const State& getState() const {
      return state;
    };

    double getHeuristicValue() const {
      return heuristicValue;
    }

    double getGValue() const {
      return heuristicValue + gValue;
    }

    double getFValue() const {
      return gValue;
    }

   private:
    const State state;
    Node* parent;
    const double heuristicValue;
    const double gValue;
  };

  RTAStar(Domain domain) : domain(std::move(domain)) {
  }

  RTAStar(RTAStar&&) = default;

  std::vector<State> solve() {
    boost::object_pool<Node> nodePool(4096, 0);
    LOG(INFO) << "Solve A*!" << std::endl;

    std::unordered_set<const State*, typename Domain::StateHash, typename Domain::StateEquals> visitedStates;
    State initialState = domain.getInitialState();
    Node* nextNode = nodePool.construct(std::move(initialState));

    while (nextNode != nullptr) {
      const State& currentState = nextNode->getState();

      if (domain.isGoal(currentState)) {
        LOG(INFO) << "Solution found!" << std::endl;
        return buildSolution(nextNode);
      }

      // Move to node


      LOG(INFO) << "Expand state: " << currentState << std::endl;

      // TODO Increment expansion counter
      std::vector<State> expandedStates = domain.expand(currentState);
      Node* bestSuccessorNode = nullptr;

      for (State state : expandedStates) {
        if (visitedStates.find(&state) == visitedStates.end()) {
          LOG(INFO) << "Add state: heuristic value: " << domain.heuristicValue(state) << state << std::endl;
          const double heuristicValue = domain.heuristicValue(state);

          const Node
              nodeToConstruct = Node(std::move(state), currentNode, heuristicValue, (currentNode->getGValue() + 1));
          Node* node = nodePool.construct(nodeToConstruct);
          const State& realState = node->getState();
        } else {
          LOG(INFO) << "Discard duplicated state: heuristic value: " << domain.heuristicValue(state) << state <<
              std::endl;
        }
      }

      nextNode = bestSuccessorNode;
    }

    LOG(INFO) << "No solution found!" << std::endl;
    return std::vector<Domain::State>();
  }

 private:

  double miniminLookahead(const Node* node) {
    return 0;
  }

  std::vector<Domain::State> buildSolution(const Node* node) const {
    return std::vector<State>();
  }
  const Domain domain;
};

template<typename Domain>
inline bool operator<(const typename RTAStar<Domain>::Node& lhs, const typename RTAStar<Domain>::Node& rhs) {
  return lhs.getFValue() < rhs.getFValue();
}

template<typename Domain>
inline bool operator>(const typename RTAStar<Domain>::Node& lhs, const typename RTAStar<Domain>::Node& rhs) {
  return rhs < lhs;
}

template<typename Domain>
inline bool operator<=(const typename RTAStar<Domain>::Node& lhs, const typename RTAStar<Domain>::Node& rhs) {
  return !(lhs > rhs);
}

template<typename Domain>
inline bool operator>=(const typename RTAStar<Domain>::Node& lhs, const typename RTAStar<Domain>::Node& rhs) {
  return !(lhs < rhs);
}

template<typename Domain>
inline bool operator<(const typename RTAStar<Domain>::Node* lhs, const typename RTAStar<Domain>::Node* rhs) {
  return lhs->getFValue() < rhs->getFValue();
}

template<typename Domain>
inline bool operator>(const typename RTAStar<Domain>::Node* lhs, const typename RTAStar<Domain>::Node* rhs) {
  return rhs < lhs;
}

template<typename Domain>
inline bool operator<=(const typename RTAStar<Domain>::Node* lhs, const typename RTAStar<Domain>::Node* rhs) {
  return !(lhs > rhs);
}

template<typename Domain>
inline bool operator>=(const typename RTAStar<Domain>::Node* lhs, const typename RTAStar<Domain>::Node* rhs) {
  return !(lhs < rhs);
}

}

#endif //REALTIMESEARCH_A_STAR_H
