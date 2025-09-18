// Kalman.h
#ifndef KALMAN_H
#define KALMAN_H

class KalmanFilter {
  public:
    float angle = 0;   // Fused angle output
    float bias = 0;    // Estimated gyro bias
    float rate = 0;    // Unbiased rate

    float P[2][2] = {{0, 0}, {0, 0}};  // Error covariance matrix

    float Q_angle = 0.001f;     // Process noise for angle
    float Q_bias  = 0.003f;     // Process noise for gyro bias
    float R_measure = 0.03f;    // Measurement noise (accel)

    float update(float newAngle, float newRate, float dt) {
      // Prediction
      rate = newRate - bias;
      angle += dt * rate;

      P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
      P[0][1] -= dt * P[1][1];
      P[1][0] -= dt * P[1][1];
      P[1][1] += Q_bias * dt;

      // Correction
      float y = newAngle - angle;
      float S = P[0][0] + R_measure;
      float K[2];
      K[0] = P[0][0] / S;
      K[1] = P[1][0] / S;

      angle += K[0] * y;
      bias  += K[1] * y;

      float P00_temp = P[0][0];
      float P01_temp = P[0][1];

      P[0][0] -= K[0] * P00_temp;
      P[0][1] -= K[0] * P01_temp;
      P[1][0] -= K[1] * P00_temp;
      P[1][1] -= K[1] * P01_temp;

      return angle;
    }
};


#endif