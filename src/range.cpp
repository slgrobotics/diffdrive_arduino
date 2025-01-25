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

void Range::setRange(int range_cm)
{
  range = range_cm > 250 ? INFINITY : ((double)range_cm) / 100.0; // meters;
}

double Range::getRange()
{
  return range;
}
