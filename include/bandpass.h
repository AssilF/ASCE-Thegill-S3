#ifndef BANDPASS_H
#define BANDPASS_H
class BandPass {
  float lowCut = 0.01;
  float highCut = 0.98;

  float lowPassOut = 0;
  float highPassOut = 0;

public:
  BandPass(float lowAlpha = 0.01, float highAlpha = 0.98)
    : lowCut(lowAlpha), highCut(highAlpha) {}

  float update(float input) {
    lowPassOut += lowCut * (input - lowPassOut);
    highPassOut = highCut * (highPassOut + input - lowPassOut);
    return highPassOut;
  }
};
#endif // BANDPASS_H