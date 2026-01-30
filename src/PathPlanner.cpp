#include "PathPlanner.hpp"
#include <cstdio>
#include <cstring>
#include <cmath>

static bool parseCSV3or4(const char* line, double& x, double& y, double& s, double& h, int& nOut) {
  // Accepts:
  // x,y,s
  // x,y,s,h
  nOut = std::sscanf(line, " %lf , %lf , %lf , %lf", &x, &y, &s, &h);
  return (nOut == 3 || nOut == 4);
}

LoadedPath PathPlanner::loadJerry(const char* filepath) {
  LoadedPath out;

  FILE* f = std::fopen(filepath, "r");
  if (!f) {
    out.ok = false;
    out.err = "Could not open file";
    return out;
  }

  bool started = false;
  char line[256];

  while (std::fgets(line, sizeof(line), f)) {
    // Start marker
    if (!started) {
      if (std::strstr(line, "#PATH-POINTS-START") != nullptr) {
        started = true;
      }
      continue;
    }

    // Stop markers
    if (std::strstr(line, "#PATH-POINTS-END") != nullptr) break;
    if (std::strstr(line, "#PATH.JERRYIO-DATA") != nullptr) break;

    // Skip blanks/comments
    if (line[0] == '#' || line[0] == '\n' || line[0] == '\r') continue;

    double x=0, y=0, s=0, h=0;
    int n=0;
    if (!parseCSV3or4(line, x, y, s, h, n)) continue;

    PathPoint p;
    p.x = x;
    p.y = y;
    p.speed = s;
    p.headingDeg = (n == 4) ? h : NAN;
    out.pts.push_back(p);
  }

  std::fclose(f);

  if (out.pts.size() < 2) {
    out.ok = false;
    out.err = "Not enough points found after #PATH-POINTS-START";
    return out;
  }

  out.ok = true;
  return out;
}
