//
// Created by Bence Cserna on 10/19/15.
//
#ifndef REALTIMESEARCH_GRID_FACTORY_H
#define REALTIMESEARCH_GRID_FACTORY_H

#include <random>

namespace rts {

static Grid uniformGrid(u_int width, u_int height, double obstacleProbability, int seed = 0) {
  Grid grid(width, height);

  std::default_random_engine randomEngine(seed);

  std::uniform_int_distribution<int> widthDistribution(0, width - 1);
  std::uniform_int_distribution<int> heightDistribution(0, height - 1);

  std::bernoulli_distribution obstacleDistribution(obstacleProbability);

  for (int x = 0; x < width; ++x) {
    for (int y = 0; y < height; ++y) {

    }
  }

  return grid;
};

}

#endif //REALTIMESEARCH_GRID_FACTORY_H