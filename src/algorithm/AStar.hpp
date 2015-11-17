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
class AStar {
 public:
  typedef typename Domain::State State;

  class Node {
   public:
    Node(State state) :
        state(std::move(state)),
        parent(nullptr),
        heuristicValue(0),
        gValue(0) {
    }

    Node(State state, const Node* parent, double h, double g) :
        state(std::move(state)),
        parent(parent),
        heuristicValue(h),
        gValue(g) {
    }

    const Node& getParent() const {
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
    const Node* parent;
    const double heuristicValue;
    const double gValue;
  };

  AStar(Domain domain) : domain(std::move(domain)) {
  }

  AStar(AStar&&) = default;

  std::vector<State> solve() {
    boost::object_pool<Node> nodePool(4096, 0);
    LOG(INFO) << "Solve A*!" << std::endl;

    std::priority_queue<Node*, std::vector<Node*>, std::less<Node*>> openList;
    std::unordered_set<const State*, typename Domain::StateHash, typename Domain::StateEquals> closedList;

    State initialState = domain.getInitialState();
    Node* startNode = nodePool.construct(initialState);
    openList.push(startNode);

    while (!openList.empty()) {
      const Node* currentNode = openList.top();
      openList.pop();
      const State& currentState = currentNode->getState();

      if (domain.isGoal(currentState)) {
        LOG(INFO) << "Solution found!" << std::endl;
        return buildSolution(*currentNode);
      }

      LOG(INFO) << "Expand state: " << currentState << std::endl;

      // TODO Increment expansion counter
      std::vector<State> expandedStates = domain.expand(currentState);
      closedList.insert(&currentState);

      for (State state : expandedStates) {
        if (closedList.find(&state) == closedList.end()) {
          LOG(INFO) << "Add state: heuristic value: " << domain.heuristicValue(state) << state << std::endl;
          const double heuristicValue = domain.heuristicValue(state);

          const Node nodeToConstruct = Node(std::move(state), currentNode, heuristicValue, (currentNode->getGValue() + 1));
          Node* node = nodePool.construct(nodeToConstruct);
          const State& realState = node->getState();
          openList.push(node);
        } else {
          LOG(INFO) << "Discard duplicated state: heuristic value: " << domain.heuristicValue(state) << state <<
              std::endl;
        }
      }
    }

    LOG(INFO) << "No solution found!" << std::endl;
    return std::vector<State>();
  }

  std::vector<State> buildSolution(const Node& node) const{
    return std::vector<State>();
  }

 private:
  const Domain domain;
};

template<typename Domain>
inline bool operator<(const typename AStar<Domain>::Node& lhs, const typename AStar<Domain>::Node& rhs) {
  return lhs.getFValue() < rhs.getFValue();
}

template<typename Domain>
inline bool operator>(const typename AStar<Domain>::Node& lhs, const typename AStar<Domain>::Node& rhs) {
  return rhs < lhs;
}

template<typename Domain>
inline bool operator<=(const typename AStar<Domain>::Node& lhs, const typename AStar<Domain>::Node& rhs) {
  return !(lhs > rhs);
}

template<typename Domain>
inline bool operator>=(const typename AStar<Domain>::Node& lhs, const typename AStar<Domain>::Node& rhs) {
  return !(lhs < rhs);
}

template<typename Domain>
inline bool operator<(const typename AStar<Domain>::Node* lhs, const typename AStar<Domain>::Node* rhs) {
  return lhs->getFValue() < rhs->getFValue();
}

template<typename Domain>
inline bool operator>(const typename AStar<Domain>::Node* lhs, const typename AStar<Domain>::Node* rhs) {
  return rhs < lhs;
}

template<typename Domain>
inline bool operator<=(const typename AStar<Domain>::Node* lhs, const typename AStar<Domain>::Node* rhs) {
  return !(lhs > rhs);
}

template<typename Domain>
inline bool operator>=(const typename AStar<Domain>::Node* lhs, const typename AStar<Domain>::Node* rhs) {
  return !(lhs < rhs);
}

}

#endif //REALTIMESEARCH_A_STAR_H
