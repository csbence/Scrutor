#include <iostream>
#include "easylogging++.h"
#include "domain/grid/Grid.hpp"
#include "domain/tile/SlidingTiles.hpp"
#include "algorithm/AStar.hpp"

INITIALIZE_EASYLOGGINGPP

int main(int argc, char *argv[]) {
  LOG(INFO) << "Scrutor Search Framework";

  rts::SlidingTiles::State::GridType grid(boost::extents[2][2]);

  grid[0][0] = 1;
  grid[1][0] = 3;
  grid[0][1] = 2;
  grid[1][1] = 0;

  rts::SlidingTiles::State state(grid, 1, 1);

  const rts::SlidingTiles tiles(2, state);
  rts::AStar<rts::SlidingTiles> aStar(std::move(tiles));

  aStar.solve();

  return 0;
}
