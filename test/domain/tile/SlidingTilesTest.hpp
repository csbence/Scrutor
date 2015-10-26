//
// Created by Bence Cserna on 10/24/15.
//

#ifndef REALTIMESEARCH_SLIDINGTILESTEST_HPP
#define REALTIMESEARCH_SLIDINGTILESTEST_HPP

#include "../dependencies/catch.hpp"
#include "../../../src/domain/tile/SlidingTiles.hpp"
#include <cmath>

namespace rts {

namespace tuyi {

TEST_CASE("Sliding tiles domain", "[SlidingTiles]") {
  typedef typename SlidingTiles::State State;
  typedef typename SlidingTiles::State::GridType GridType;

  GridType grid1(boost::extents[2][2]);
  GridType grid2(boost::extents[2][2]);
  GridType grid3(boost::extents[2][2]);
  GridType grid4(boost::extents[2][2]);

  grid1[0][0] = 0;
  grid1[1][0] = 1;
  grid1[0][1] = 2;
  grid1[1][1] = 3;

  grid2[0][0] = 0;
  grid2[1][0] = 1;
  grid2[0][1] = 2;
  grid2[1][1] = 3;

  grid3[0][0] = 1;
  grid3[1][0] = 0;
  grid3[0][1] = 2;
  grid3[1][1] = 3;

  grid4[0][0] = 1;
  grid4[1][0] = 2;
  grid4[0][1] = 3;
  grid4[1][1] = 0;

  State state1(grid1, 0, 0);
  State state2(grid2, 0, 0);
  State state3(grid3, 0, 1);
  State state4(grid4, 1, 1);


  SlidingTiles tiles(2, state1);

  SECTION("Heuristic function") {
    REQUIRE(std::abs(tiles.heuristicValue(state1)) < std::numeric_limits<double>::epsilon());
    REQUIRE(std::abs(tiles.heuristicValue(state1) - tiles.heuristicValue(state2))
                < std::numeric_limits<double>::epsilon());
    REQUIRE_FALSE(std::abs(tiles.heuristicValue(state2) - tiles.heuristicValue(state3))
                      < std::numeric_limits<double>::epsilon());
    REQUIRE(std::abs(tiles.heuristicValue(state3) - 1) < std::numeric_limits<double>::epsilon());
    REQUIRE(std::abs(tiles.heuristicValue(state4) - 4) < std::numeric_limits<double>::epsilon());

  }

  SECTION("Hash funciton") {
    SlidingTiles::StateHash hashFunction;

    REQUIRE(hashFunction(&state1) == hashFunction(&state1));
    REQUIRE(hashFunction(&state2) == hashFunction(&state1));
    REQUIRE_FALSE(hashFunction(&state2) == hashFunction(&state3));
  }
}

}

}

#endif //REALTIMESEARCH_SLIDINGTILESTEST_HPP
