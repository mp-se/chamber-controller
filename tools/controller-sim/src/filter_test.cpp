// Feeds fixed sequences into the CascadedFilter and prints the full precision output in Celsius
// after every sample: "<b> <sequence> <sample> <value>". Sequence 0 is a step response, 1 a random
// walk, 2 a slow sine with noise. With the argument "steps" only sequence 0 is printed.
// Builds against fixed point libraries too (git revisions before the floating point rewrite).
#include <FilterCascaded.hpp>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

uint32_t g_millis = 0;
uint32_t g_lastDelay = 0;

#ifdef TEMP_IS_DOUBLE
static double toC(CascadedFilter& f) { return f.readOutput(); }
static void feed(CascadedFilter& f, double c) { f.add(c); }
static void initTo(CascadedFilter& f, double c) { f.init(c); }
#else
// fixed point: the value is (C - 48) * 512 with 16 extra fraction bits
static double toC(CascadedFilter& f) { return f.readOutputDoublePrecision() / 33554432.0 + 48.0; }
static void feed(CascadedFilter& f, double c) { f.add(doubleToTemp(c)); }
static void initTo(CascadedFilter& f, double c) { f.init(doubleToTemp(c)); }
#endif

// sensor like inputs: multiples of 1/16 degree, so they are exact in both representations
static double quant(double c) { return std::floor(c * 16.0 + 0.5) / 16.0; }

int main(int argc, char** argv) {
  const bool stepsOnly = argc > 1 && std::strcmp(argv[1], "steps") == 0;
  std::vector<std::vector<double>> sequences;

  {  // step responses
    std::vector<double> s;
    for (int i = 0; i < 300; i++) s.push_back(20.0);
    for (int i = 0; i < 6000; i++) s.push_back(25.0);
    for (int i = 0; i < 6000; i++) s.push_back(9.5);
    sequences.push_back(s);
  }
  if (!stepsOnly) {
    {  // random walk, 100000 samples
      std::vector<double> s;
      uint32_t r = 7;
      double v = 18.0;
      for (int i = 0; i < 100000; i++) {
        r = r * 1664525u + 1013904223u;
        v += ((r >> 16) % 5 - 2) * 0.01;
        if (v < 2) v = 2;
        if (v > 30) v = 30;
        s.push_back(quant(v));
      }
      sequences.push_back(s);
    }
    {  // slow sine with noise, like a fermentation temperature with cooling and heating cycles
      std::vector<double> s;
      uint32_t r = 99;
      for (int i = 0; i < 100000; i++) {
        r = r * 1664525u + 1013904223u;
        double noise = (((r >> 8) / 16777216.0) - 0.5) * 0.06;
        s.push_back(quant(15.0 + 4.0 * std::sin(i / 1500.0) + 0.5 * std::sin(i / 90.0) + noise));
      }
      sequences.push_back(s);
    }
  }

  for (int b = 1; b <= 6; b++) {
    for (size_t q = 0; q < sequences.size(); q++) {
      CascadedFilter f;
      f.setCoefficients(b);
      initTo(f, sequences[q][0]);
      for (size_t i = 0; i < sequences[q].size(); i++) {
        feed(f, sequences[q][i]);
        std::printf("%d %zu %zu %.9f\n", b, q, i, toC(f));
      }
    }
  }
  return 0;
}
