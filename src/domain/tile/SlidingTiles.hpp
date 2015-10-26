//
// Created by Bence Cserna on 10/21/15.
//

#ifndef REALTIMESEARCH_SLIDINGTILES_HPP
#define REALTIMESEARCH_SLIDINGTILES_HPP

#include <vector>
#include <boost/multi_array.hpp>
#include <boost/assert.hpp>
#include <boost/functional/hash.hpp>
#include <cmath>

namespace rts {

class SlidingTiles {
 public:
  /**
   * State of a sliding tile puzzle.
   *
   *  width (x)
   * -------
   * |0|1|2|
   * |3|4|5| height(y)
   * |6|7|8|
   * -------
   *
   * (0, 0) == 0
   * (1, 0) == 1
   * (0, 1) == 3
   */
  class State {
   public:
    typedef boost::multi_array<int, 2> GridType;
    typedef typename GridType::index Index;

    State(GridType grid, Index emptyCellX, Index emptyCellY) :
        grid(grid),
        emptyCellX(emptyCellX),
        emptyCellY(emptyCellY) {
    }

    State(const State&) = default;
    State(State&&) = default;

    const GridType& getGrid() const {
      return grid;
    }

    const GridType grid;
    const Index emptyCellX, emptyCellY;
  };

  /**
   * Hash functor for SlidingTiles::State
   */
  struct StateHash {
    std::size_t operator()(const State *state) const {
      const State::GridType& grid = state->getGrid();
      std::size_t seed = 0;

      State::Index size = grid.shape()[0];

      for (State::Index x = 0; x < size; ++x) {
        for (State::Index y = 0; y < size; ++y) {
          // TODO Cannot be used on large grids
          char value = grid[x][y];
          seed ^= value << (x + y);
        }
      }

      return seed;
    }
  };

  struct StateEquals {
    bool operator()(const State *lhs, const State *rhs) const {
      const State::GridType& lhsGrid = lhs->getGrid();
      const State::GridType& rhsGrid = rhs->getGrid();

      BOOST_ASSERT_MSG(lhsGrid.shape()[0] == rhsGrid.shape()[0], "Invalid state dimensions!");
      BOOST_ASSERT_MSG(lhsGrid.shape()[1] == rhsGrid.shape()[1], "Invalid state dimensions!");

      State::Index size = lhsGrid.shape()[0];
      for (State::Index x = 0; x < size; ++x) {
        for (State::Index y = 0; y < size; ++y) {
          if (lhsGrid[x][y] != rhsGrid[x][y]) {
            return false;
          }
        }
      }

      return true;
    }
  };

  SlidingTiles(int size, State initialState) :
      size(size),
      initialState(std::move(initialState)) {
  }

  SlidingTiles(const SlidingTiles&) = default;
  SlidingTiles(SlidingTiles&&) = default;

  int getSize() const {
    return size;
  }

  /// Domain specific methods ///

  const State& getInitialState() const {
    return initialState;
  };

  std::vector<State> expand(const State& state) const {
    std::vector<State> states;

    if (state.emptyCellX > 0) { // Left
      states.emplace_back(State(expandDirection(state, -1, 0)));
    }
    if (state.emptyCellX < size - 1) { // Right
      states.emplace_back(State(expandDirection(state, 1, 0)));
    }
    if (state.emptyCellY > 0) { // Up
      states.emplace_back(State(expandDirection(state, 0, -1)));
    }
    if (state.emptyCellY < size - 1) { // Down
      states.emplace_back(State(expandDirection(state, 0, 1)));
    }

    return states;
  }

  State expandDirection(const State& state, State::Index relativeX, State::Index relativeY) const {
    State::GridType grid = state.getGrid();

    State::Index targetX = state.emptyCellX + relativeX;
    State::Index targetY = state.emptyCellY + relativeY;

    grid[state.emptyCellX][state.emptyCellY] = grid[targetX][targetY];
    grid[targetX][targetY] = 0;

    return State(std::move(grid), targetX, targetY);
  }

  double heuristicValue(State state) const {
    int manhattanSum = 0;
    int targetY = 0;
    int targetX = 0;

    auto grid = state.getGrid();

    for (int x = 0; x < size; ++x) {
      for (int y = 0; y < size; ++y) {
        char cellValue = grid[x][y];
        if (cellValue == 0) continue;

        targetY = cellValue / size;
        targetX = cellValue % size;

        manhattanSum += std::abs(targetX - x) + std::abs(targetY - y);
      }
    }

    return manhattanSum;
  }

  double distanceEstimate(State) {
    return 0;
  }

  bool isGoal(State state) const {
    return heuristicValue(state) == 0;
  }

 private:
  const int size;
  const State initialState;
};

std::ostream& operator<<(std::ostream& stream, const SlidingTiles::State& slidingTiles) {
  const SlidingTiles::State::GridType& grid = slidingTiles.getGrid();
  SlidingTiles::State::Index size = grid.shape()[0];

  for (int y = 0; y < size; ++y) {
    for (int x = 0; x < size; ++x) {
      stream << grid[x][y];
    }
    stream << std::endl;
  }

  return stream;
}

std::ostream& operator<<(std::ostream& stream, const SlidingTiles::State *slidingTiles) {
  return stream << &slidingTiles;
}

}

#endif //REALTIMESEARCH_SLIDINGTILES_HPP
