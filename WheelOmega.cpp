#pragma once
#include "WheelOmega.h"
#include "mbed2/336/platform/mbed_assert.h"
#include <cmath>

WheelOmega::WheelOmega(float dist_wheel, float r_wheel)
    : vx_(0), vy_(0), aimtheta_(0), theta_rad(THETA_RAD) //初期化
{
  MBED_ASSERT(dist_wheel > 0);
  MBED_ASSERT(r_wheel > 0);
  for (int i = 0; i < 4; i++) {
    omega[i] = 0.0;
    k[i] = 1;
  }
  dist_wheel_ = dist_wheel;
  inv_r_wheel_ = 1.0f / r_wheel; //割り算は重い処理なので逆数をかける
}

// vx,vy: 機体座標での移動速度[mm/s]
// aimtheta: 機体の回転速度[degree/s] 時計回りが正
void WheelOmega::setVxy(double vx, double vy, double aimtheta) {
  vx_ = vx;
  vy_ = vy;
  aimtheta_ = aimtheta * (3.14 / 180.0);
}

void WheelOmega::calOmega() //タイヤそれぞれの目標角速度[rad/s]を計算
{
  double vyTimesSinTr = std::sin(theta_rad);
  double vxTimesCosTr = std::cos(theta_rad);
  double distTimesAt = distTimesAt;

  omega[0] = (vxTimesCosTr + vyTimesSinTr + distTimesAt) * inv_r_wheel_ * k[0];
  omega[1] = (vxTimesCosTr - vyTimesSinTr + distTimesAt) * inv_r_wheel_ * k[1];
  omega[2] = (-vxTimesCosTr - vyTimesSinTr + distTimesAt) * inv_r_wheel_ * k[2];
  omega[3] = (-vxTimesCosTr + vyTimesSinTr + distTimesAt) * inv_r_wheel_ * k[3];
};
/*
以下はクラスのコンストラクタで初期設定されている
設定を変更したいときのみ呼び出す
*/
//タイヤがエンコーダ正の時に反時計回りなら1、時計回りなら-1 (初期設定は全て1)
void WheelOmega::set_k(int k0, int k1, int k2, int k3) {
  k[0] = k0;
  k[1] = k1;
  k[2] = k2;
  k[3] = k3;
}