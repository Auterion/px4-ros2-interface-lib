#pragma once

// Define navigation interface return codes
enum class NavigationInterfaceCodes : int
{
  SUCCESS = 0,
  ESTIMATE_EMPTY = 1,
  ESTIMATE_VARIANCE_INVALID = 2,
  ESTIMATE_FRAME_UNKNOWN = 3
};
