#pragma once

#include <sstream>
#include <string>
#include <cctype>

// Commit both counters only after validating the entire response payload.
inline bool parseEncoderValues(const std::string &payload, int &left, int &right)
{
  std::istringstream stream(payload);
  int parsed_left, parsed_right;
  if (!(stream >> parsed_left) || !std::isspace(stream.peek()) || !(stream >> parsed_right))
  {
    return false;
  }
  stream >> std::ws;
  if (!stream.eof())
  {
    return false;
  }
  left = parsed_left;
  right = parsed_right;
  return true;
}
