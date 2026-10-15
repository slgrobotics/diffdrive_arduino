#include "diffdrive_arduino/range.h"

#include <cmath>


Range::Range(const std::string &range_name)
{
  setup(range_name);
}


void Range::setup(const std::string &range_name, double no_return_range)
{
  name = range_name;
  no_return_range_ = no_return_range;
}

void Range::setRange(int range_cm)
{
  range = range_cm > 250 ? no_return_range_ : ((double)range_cm) / 100.0; // meters;
}

double Range::getRange()
{
  return range;
}
