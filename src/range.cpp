#include "diffdrive_arduino/range.h"

#include <cmath>


Range::Range(const std::string &range_name)
{
  setup(range_name);
}


void Range::setup(const std::string &range_name)
{
  name = range_name;
}

double Range::getRange()
{
  return range;
}
