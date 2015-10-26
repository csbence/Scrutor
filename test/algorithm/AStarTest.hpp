//
// Created by Bence Cserna on 10/25/15.
//

#ifndef REALTIMESEARCH_ASTARTEST_HPP
#define REALTIMESEARCH_ASTARTEST_HPP

#include "../dependencies/catch.hpp"
#include "../../src/domain/tile/SlidingTiles.hpp"
#include "../../src/algorithm/AStar.hpp"

namespace rts {

namespace {

TEST_CASE("AStar algorithm", "[AStar]") {
  typedef typename SlidingTiles::State State;
  typedef typename SlidingTiles::State::GridType GridType;

  GridType grid(boost::extents[2][2]);

  grid[0][0] = 1;
  grid[1][0] = 3;
  grid[0][1] = 2;
  grid[1][1] = 0;

  State state(grid, 1, 1);

  const SlidingTiles tiles(2, state);
  AStar<SlidingTiles> aStar(std::move(tiles));

  SECTION("Simple sliding tile puzzle") {
    aStar.solve();
  }

}

}

}

#endif //REALTIMESEARCH_ASTARTEST_HPP
