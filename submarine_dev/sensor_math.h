#pragma once
#include <Arduino.h>
#include <math.h>

// ==========================================
// 🧮 수학 및 통계 헬퍼 함수 모음 (Service Layer / Utils)
// ==========================================

// 배열 평균
inline float calcMean(float* arr, int size) {
  float sum = 0;
  for (int i = 0; i < size; i++) sum += arr[i];
  return sum / size;
}

// 배열 표준편차
inline float calcStd(float* arr, int size, float mean) {
  float sumSq = 0;
  for (int i = 0; i < size; i++) {
    float diff = arr[i] - mean;
    sumSq += diff * diff;
  }
  return sqrtf(sumSq / size);
}

// 배열 RMS
inline float calcRMS(float* arr, int size) {
  float sumSq = 0;
  for (int i = 0; i < size; i++) sumSq += arr[i] * arr[i];
  return sqrtf(sumSq / size);
}

// 선형 회귀 기울기 (drift 방향 감지)
inline float calcSlope(float* arr, int size) {
  float sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
  for (int i = 0; i < size; i++) {
    sumX  += i;
    sumY  += arr[i];
    sumXY += i * arr[i];
    sumX2 += i * i;
  }
  float denom = (size * sumX2 - sumX * sumX);
  if (fabsf(denom) < 1e-10) return 0.0f;
  return (size * sumXY - sumX * sumY) / denom;
}

// 배열 최대값
inline float calcMax(float* arr, int size) {
  float maxVal = arr[0];
  for (int i = 1; i < size; i++) {
    if (arr[i] > maxVal) maxVal = arr[i];
  }
  return maxVal;
}

// 피어슨 상관계수
inline float calcCorrelation(float* x, float* y, int size) {
  float mx = calcMean(x, size);
  float my = calcMean(y, size);
  float num = 0, dx2 = 0, dy2 = 0;
  for (int i = 0; i < size; i++) {
    float dx = x[i] - mx;
    float dy = y[i] - my;
    num += dx * dy;
    dx2 += dx * dx;
    dy2 += dy * dy;
  }
  float denom = sqrtf(dx2 * dy2);
  if (denom < 1e-10) return 0.0f;
  return num / denom;
}

// 기존 센서 코드에서 사용하던 정렬 및 절사평균도 필요할 경우를 대비해 남겨둡니다.
inline void sortArray(float* arr, int size) {
  for (int i = 0; i < size - 1; ++i) {
    for (int j = 0; j < size - i - 1; ++j) {
      if (arr[j] > arr[j + 1]) {
        float tmp = arr[j];
        arr[j] = arr[j + 1];
        arr[j + 1] = tmp;
      }
    }
  }
}

inline float getTrimmedMean(float* arr, int size, int trimCount) {
  sortArray(arr, size);
  float sum = 0;
  int cnt = 0;
  for (int i = trimCount; i < size - trimCount; ++i) {
    sum += arr[i];
    cnt++;
  }
  if (cnt == 0) return 0.0f;
  return sum / cnt; 
}
