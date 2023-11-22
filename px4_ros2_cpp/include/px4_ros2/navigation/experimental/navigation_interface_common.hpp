#pragma once

// Define navigation interface return codes
enum class NavigationInterfaceReturnCode : int
{
  SUCCESS = 0,
  ESTIMATE_EMPTY = 1,
  ESTIMATE_VARIANCE_INVALID = 2,
  ESTIMATE_FRAME_UNKNOWN = 3,
  ESTIMATE_VALUE_NAN = 4,
  ESTIMATE_MISSING_TIMESTAMP = 5
};
