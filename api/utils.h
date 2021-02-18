#ifndef __GRIDUTILS_H__
#define __GRIDUTILS_H__

#include <tuple>

struct Pos
{
  //! x-position
  int x;
  //! y-position
  int y;
  Pos() : x(0), y(0) {}
  Pos(int x, int y)
      : x(x),
        y(y) {}
};

bool operator<(const Pos &x, const Pos &y);

std::vector<Pos> create_line(const int x0, const int y0, const int x1, const int y1);

#endif /*__GRIDUTILS_H__*/
