//
// Created by Bence Cserna on 10/19/15.
//
#ifndef REALTIMESEARCH_BASICFIELD_H
#define REALTIMESEARCH_BASICFIELD_H

namespace rts {

enum class BasicField: char {
  OBSTACLE = '#',
  EMPTY = '_',
  AGENT = '@',
  GOAL = '*'
};

}
#endif //REALTIMESEARCH_BASICFIELD_H
