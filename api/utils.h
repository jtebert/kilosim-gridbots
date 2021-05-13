#ifndef __GRIDUTILS_H__
#define __GRIDUTILS_H__

#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>
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

// bool operator<(const Pos &x, const Pos &y);

// std::vector<Pos> create_line(const int x0, const int y0, const int x1, const int y1);

// Bresenham's line algorithm
// https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm

static bool operator<(const Pos &x, const Pos &y)
{
  return std::tie(x.x, x.y) < std::tie(y.x, y.y);
}

static std::vector<Pos> line_low(const int x0, const int y0, const int x1, const int y1)
{
  int dx = x1 - x0;
  int dy = y1 - y0;
  int yi = 1;
  if (dy < 0)
  {
    yi = -1;
    dy = -dy;
  }
  int D = (2 * dy) - dx;
  int y = y0;
  int x_len = x1 - x0 + 1;
  int x;
  std::vector<Pos> path(x_len);
  for (int x_ind = 0; x_ind <= x_len; x_ind++)
  {
    x = x_ind + x0;
    path[x_ind] = {x, y};
    if (D > 0)
    {
      y += yi;
      D += (2 * (dy - dx));
    }
    else
    {
      D += 2 * dy;
    }
  }
  return path;
}

static std::vector<Pos> line_high(const int x0, const int y0, const int x1, const int y1)
{
  int dx = x1 - x0;
  int dy = y1 - y0;
  int xi = 1;
  if (dx < 0)
  {
    xi = -1;
    dx = -dx;
  }
  int D = (2 * dx) - dy;
  int x = x0;
  int y_len = y1 - y0 + 1;
  int y;
  std::vector<Pos> path(y_len);
  for (int y_ind = 0; y_ind <= y_len; y_ind++)
  {
    y = y_ind + y0;
    path[y_ind] = {x, y};
    if (D > 0)
    {
      x += xi;
      D += (2 * (dx - dy));
    }
    else
    {
      D += 2 * dx;
    }
  }
  return path;
}

static std::vector<Pos> create_line(const int x0, const int y0, const int x1, const int y1)
{
  /*
     * Returned vectors are created to have positions popped from the end.
     * i.e., the *last* element in the vector is the *next* position for the
     * robot. (This is different from Gridsim.) This is counterintuitive because
     * it means (x1, y1) is now the first element of the vector.
     */
  std::vector<Pos> path;
  bool is_reversed = false;

  // Edge cases: vertical and horizontal lines
  if (x0 == x1)
  {
    // vertical
    int y_len = y1 - y0 + 1;
    // std::cout << y_len << std::endl;
    path.resize(y_len);
    for (int y_ind = 0; y_ind <= y_len; y_ind++)
    {
      path[y_ind] = {x0, y_ind + y0};
    }
  }
  else if (y0 == y1)
  {
    // horizontal
    int x_len = x1 - x0 + 1;
    // std::cout << x_len << std::endl;
    path.resize(x_len);
    for (int x_ind = 0; x_ind <= x_len; x_ind++)
    {
      path[x_ind] = {x_ind + x0, y0};
    }
  }
  else if (std::abs(y1 - y0) < std::abs(x1 - x0))
  {
    if (x0 > x1)
    {
      path = line_low(x1, y1, x0, y0);
    }
    else
    {
      path = line_low(x0, y0, x1, y1);
      is_reversed = true;
    }
  }
  else
  {
    if (y0 > y1)
    {
      path = line_high(x1, y1, x0, y0);
    }
    else
    {
      path = line_high(x0, y0, x1, y1);
      is_reversed = true;
    }
  }
  if (is_reversed)
  {
    std::reverse(path.begin(), path.end());
  }
  // Remove the current (last) position from the vector so that the robot will
  // move on the next tick
  path.pop_back();

  return path;
}

#endif /*__GRIDUTILS_H__*/
