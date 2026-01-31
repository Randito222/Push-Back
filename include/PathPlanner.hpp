#pragma once
#include <vector>
#include <string>

struct PathPoint {
  double x;          // position units from file (commonly inches in LemLib exports)
  double y;          // position units from file
  double speed;      // speed hint (often 0..127-ish)
  double headingDeg; // optional; NaN if missing
};

struct LoadedPath {
  std::vector<PathPoint> pts;
  bool ok = false;
  std::string err;
};

namespace PathPlanner {
  // Supports BOTH:
  //  1) "/usd/whatever.txt"  (SD card file)
  //  2) ASSET(myPath_txt)    (embedded /static file contents)
  LoadedPath loadJerry(const char* filepathOrAsset);
}
