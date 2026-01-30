#pragma once
#include <vector>
#include <string>

struct PathPoint {
  double x;          // raw from file (likely mm)
  double y;          // raw from file (likely mm)
  double speed;      // raw speed hint
  double headingDeg; // optional; NaN if missing
};

struct LoadedPath {
  std::vector<PathPoint> pts;
  bool ok = false;
  std::string err;
};

namespace PathPlanner {
  // Reads path.jerryio format:
  // waits for "#PATH-POINTS-START", then reads CSV rows until EOF / end marker / JSON section
  LoadedPath loadJerry(const char* filepath);
}
