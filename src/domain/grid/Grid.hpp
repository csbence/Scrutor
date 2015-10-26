//
// Created by Bence Cserna on 10/19/15.
//
#ifndef REALTIMESEARCH_GRID_H
#define REALTIMESEARCH_GRID_H

#include <boost/multi_array.hpp>
#include <boost/assert.hpp>

namespace rts {


template<typename Cell>
class Grid {
 public:
  typedef boost::multi_array<Cell, 2> GridType;
  typedef typename GridType::index Index;

  Grid(Index width, Index height) :
      grid(boost::extents[width][height]),
      width(width),
      height(height) {
  }

  inline char &operator()(Index x, Index y) {
    validateCoordinates(x, y);

    return grid[x][y];
  }

  inline void setCell(Index x, Index y, Cell value) {
    validateCoordinates(x, y);

    grid[x][y] = value;
  }

  inline Cell getCell(Index x, Index y) const {
    validateCoordinates(x, y);

    return grid[x][y];
  }

  Index getWidth() const {
    return width;
  }

  Index getHeight() const {
    return height;
  }

  bool isEmpty(Index x, Index y) const {
    validateCoordinates(x, y);

    return grid[x][y] == Cell::EMPTY;
  }

 private:
  inline void validateCoordinates(Index x, Index y) const {
    BOOST_ASSERT_MSG(x < width, "Grid::setCell: x must be smaller than the width of the grid.");
    BOOST_ASSERT_MSG(y < height, "Grid::setCell: y must be smaller than the height of the grid.");
  }

  const Index width;
  const Index height;
  GridType grid;
};

}

#endif //REALTIMESEARCH_GRID_H
