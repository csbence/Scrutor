//
// Created by Bence Cserna on 10/19/15.
//
#ifndef REALTIMESEARCH_A_STAR_H
#define REALTIMESEARCH_A_STAR_H

#include <queue>
#include <vector>
#include <unordered_set>
#include <boost/optional.hpp>

namespace rts {

template<typename Domain>
class AStar {
 public:
  typedef typename Domain::State State;

  class Node {
   public:
    Node(State state) : parent(*this), state(state), heuristicValue(0) {
    }

    Node(State state, Node& parent, double h) : parent(parent), state(state), heuristicValue(h) {
    }

    void setParent(Node parent) {
      this->parent = parent;
    }

    Node& getParent() const {
      return parent;
    }

    const State& getState() const {
      return state;
    };

    double getHeuristicValue() const {
      return heuristicValue;
    }

   private:
    Node& parent;
    const State state;
    const double heuristicValue;
  };

  AStar(Domain domain) : domain(std::move(domain)) {
  }

  AStar(AStar&&) = default;

  std::vector<State> solve() {
    State initialState = domain.getInitialState();

    std::priority_queue<Node*, std::vector<Node*>, std::greater<Node*>> openList;
    std::unordered_set<State*, typename Domain::StateHash, typename Domain::StateEquals> closedList;
    std::vector<Node> ownerList;

    ownerList.emplace_back(Node(initialState));
    openList.push(&ownerList.back());

    std::vector<State> expandedStates;

    while (!openList.empty()) {
      Node* currentNode = openList.top();
      openList.pop();
      const State& currentState = currentNode->getState();

      if (domain.isGoal(currentState)) {
        return buildSolution(*currentNode);
      }

      // Increment expansion counter
      auto expandedStates = domain.expand(currentState);

      for (State state : expandedStates) {
        if (closedList.find(&state) == closedList.end()) {
          ownerList.emplace_back(Node(state, *currentNode, domain.heuristicValue(state)));
          openList.push(&ownerList.back());
          closedList.insert(&state);
        }
      }
    }

    return std::vector<SlidingTiles::State>();
  }

  std::vector<SlidingTiles::State> buildSolution(Node& node) {
    return std::vector<State>();
  }

 private:
  const Domain domain;
};

template<typename Domain>
inline bool operator<(const typename AStar<Domain>::Node& lhs, const typename AStar<Domain>::Node& rhs) {
  return lhs.getHeuristicValue() < rhs.getHeuristicValue();
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
  return lhs->getHeuristicValue() < rhs->getHeuristicValue();
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
