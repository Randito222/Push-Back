#include "PathPlanner.hpp"
#include <cstring>  // strlen, memmove, strstr
#include <cstdlib>  // strtol
#include <cstdio>   // fopen, fgets, sscanf
#include <cmath>    // NAN, isfinite

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ------------------------------------------------------------
// PATH.JERRYIO / LemLib-format parser
//
// This loader supports BOTH of these usages:
//
// 1) SD / filesystem path:
//      auto p = PathPlanner::loadJerry("/usd/myPath.txt");
//
// 2) PROS "ASSET(...)" embedded path file (recommended for path.jerryio):
//      ASSET(myPath_txt);
//      auto p = PathPlanner::loadJerry(myPath_txt);
//
// In case (2), the argument is NOT a real file path; it is a pointer to the
// file contents in flash. We detect this and parse from the buffer.
// ------------------------------------------------------------

static void sanitizeLine(char* s) {
  // Strip trailing CR/LF (Windows line endings)
  size_t len = std::strlen(s);
  while (len > 0 && (s[len - 1] == '\n' || s[len - 1] == '\r')) {
    s[--len] = '\0';
  }

  // Strip UTF-8 BOM if present (sometimes Windows adds this)
  if (len >= 3 &&
      (unsigned char)s[0] == 0xEF &&
      (unsigned char)s[1] == 0xBB &&
      (unsigned char)s[2] == 0xBF) {
    std::memmove(s, s + 3, std::strlen(s + 3) + 1);
  }

  // Replace Unicode minus (UTF-8 E2 88 92) with ASCII '-'
  for (int i = 0; s[i] && s[i + 2]; i++) {
    if ((unsigned char)s[i]   == 0xE2 &&
        (unsigned char)s[i+1] == 0x88 &&
        (unsigned char)s[i+2] == 0x92) {
      s[i] = '-';
      // shift left by 2 bytes (remove the remaining UTF-8 bytes)
      std::memmove(&s[i + 1], &s[i + 3], std::strlen(&s[i + 3]) + 1);
    }
  }

  // Trim leading spaces
  while (s[0] == ' ' || s[0] == '\t') {
    std::memmove(s, s + 1, std::strlen(s + 1) + 1);
  }

  // Trim trailing spaces
  len = std::strlen(s);
  while (len > 0 && (s[len - 1] == ' ' || s[len - 1] == '\t')) {
    s[--len] = '\0';
  }
}

static bool parseCSV3or4(const char* line,
                         double& x, double& y, double& s, double& h,
                         int& nOut) {
  x = y = s = h = 0;
  nOut = 0;

  // Tolerant parsing: "x,y,s" or "x, y, s" or "x y s"
  int n = std::sscanf(line, " %lf %*[, ] %lf %*[, ] %lf %*[, ] %lf",
                      &x, &y, &s, &h);

  if (n == 3) { nOut = 3; return true; }
  if (n == 4) { nOut = 4; return true; }
  return false;
}

static bool looksLikeEmbeddedBuffer(const char* s) {
  if (!s) return false;

  // If it contains a newline very early, it's almost certainly file contents.
  // If it contains commas, spaces, or comment markers, that's also a strong hint.
  for (int i = 0; i < 512 && s[i]; i++) {
    if (s[i] == '\n') return true;
    if (s[i] == ',' && i > 0) return true;
    if (s[i] == '#' && i < 64) return true;
  }

  // If it looks like a path (contains / or .txt), treat it as a filename.
  if (std::strstr(s, "/") != nullptr) return false;
  if (std::strstr(s, ".txt") != nullptr) return false;

  return false;
}

static LoadedPath parseJerryFromFILE(FILE* f) {
  LoadedPath out;

  bool started = false;
  bool sawAnyPoint = false;
  char line[256];

  while (std::fgets(line, sizeof(line), f)) {
    sanitizeLine(line);

    // Stop at metadata / end markers (these appear in some exports)
    if (std::strstr(line, "#PATH.JERRYIO-DATA") != nullptr) break;
    if (std::strstr(line, "endData") != nullptr) break;
    if (std::strstr(line, "#PATH-POINTS-END") != nullptr) break;

    // Start marker (if present)
    if (!started && std::strstr(line, "#PATH-POINTS-START") != nullptr) {
      started = true;
      continue;
    }

    // Skip blank lines/comments AFTER checking marker
    if (line[0] == '#' || line[0] == '\0') continue;

    // Auto-start if file doesn't have the marker (common in LemLib exports)
    if (!started) {
      // Skip single-number header lines (LemLib format starts with many of these)
      // NOTE: The point lines have 3 (or 4) numbers, so they're unaffected.
      double v = 0;
      char extra = 0;
      if (std::sscanf(line, " %lf %c", &v, &extra) == 1) {
        continue;
      }

      // If line is a point row, start now
      double x=0, y=0, s=0, h=0;
      int n=0;
      if (parseCSV3or4(line, x, y, s, h, n)) {
        started = true;

        PathPoint p;
        p.x = x;
        p.y = y;
        p.speed = s;
        p.headingDeg = (n == 4) ? h : NAN;

        out.pts.push_back(p);
        sawAnyPoint = true;
      }
      continue;
    }

    // Normal point parsing
    double x=0, y=0, s=0, h=0;
    int n=0;
    if (!parseCSV3or4(line, x, y, s, h, n)) continue;

    PathPoint p;
    p.x = x;
    p.y = y;
    p.speed = s;
    p.headingDeg = (n == 4) ? h : NAN;

    out.pts.push_back(p);
    sawAnyPoint = true;
  }

  if (!sawAnyPoint || out.pts.size() < 2) {
    out.ok = false;
    out.err = "Not enough points found in file";
    return out;
  }

  out.ok = true;
  return out;
}

static LoadedPath parseJerryFromBuffer(const char* buf) {
  LoadedPath out;
  if (!buf) {
    out.ok = false;
    out.err = "Null buffer";
    return out;
  }

  bool started = false;
  bool sawAnyPoint = false;

  // Read line-by-line from a C-string buffer.
  const char* p = buf;
  while (*p) {
    char line[256];
    int  li = 0;

    // Copy until newline or null
    while (*p && *p != '\n' && li < (int)sizeof(line) - 1) {
      line[li++] = *p++;
    }
    line[li] = '\0';

    // Skip the '\n'
    if (*p == '\n') p++;

    sanitizeLine(line);

    if (std::strstr(line, "#PATH.JERRYIO-DATA") != nullptr) break;
    if (std::strstr(line, "endData") != nullptr) break;
    if (std::strstr(line, "#PATH-POINTS-END") != nullptr) break;

    if (!started && std::strstr(line, "#PATH-POINTS-START") != nullptr) {
      started = true;
      continue;
    }

    if (line[0] == '#' || line[0] == '\0') continue;

    if (!started) {
      // Skip LemLib single-number header lines
      double v = 0;
      char extra = 0;
      if (std::sscanf(line, " %lf %c", &v, &extra) == 1) {
        continue;
      }

      double x=0, y=0, s=0, h=0;
      int n=0;
      if (parseCSV3or4(line, x, y, s, h, n)) {
        started = true;
        PathPoint pt;
        pt.x = x;
        pt.y = y;
        pt.speed = s;
        pt.headingDeg = (n == 4) ? h : NAN;
        out.pts.push_back(pt);
        sawAnyPoint = true;
      }
      continue;
    }

    double x=0, y=0, s=0, h=0;
    int n=0;
    if (!parseCSV3or4(line, x, y, s, h, n)) continue;

    PathPoint pt;
    pt.x = x;
    pt.y = y;
    pt.speed = s;
    pt.headingDeg = (n == 4) ? h : NAN;
    out.pts.push_back(pt);
    sawAnyPoint = true;
  }

  if (!sawAnyPoint || out.pts.size() < 2) {
    out.ok = false;
    out.err = "Not enough points found in buffer";
    return out;
  }

  out.ok = true;
  return out;
}

LoadedPath PathPlanner::loadJerry(const char* filepathOrAsset) {
  // 1) Try to parse as embedded file contents (ASSET) if it looks like it
  if (looksLikeEmbeddedBuffer(filepathOrAsset)) {
    return parseJerryFromBuffer(filepathOrAsset);
  }

  // 2) Try to open as a normal file path
  FILE* f = std::fopen(filepathOrAsset, "r");
  if (f) {
    LoadedPath out = parseJerryFromFILE(f);
    std::fclose(f);
    return out;
  }

  // 3) Fallback: if fopen failed but it STILL looks like data, parse buffer anyway
  if (filepathOrAsset && (std::strstr(filepathOrAsset, ",") != nullptr)) {
    return parseJerryFromBuffer(filepathOrAsset);
  }

  LoadedPath out;
  out.ok = false;
  out.err = "Could not open file (and input didn't look like ASSET data)";
  return out;
}
