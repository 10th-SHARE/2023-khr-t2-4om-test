#pragma once
//四輪を動かすためのクラス
//機体座標のx軸正を右向き、y軸を上向きに取ったとき
//タイヤ0:左上、タイヤ1:右上、タイヤ2:右下、タイヤ3:左下
#include <cmath>
#define THETA_RAD 45.0 / 180.0 * 3.14

class WheelOmega {
public:
  //コンストラクタの定義 main関数の前に必ず一度宣言する
  // dist_wheel: 機体中心からタイヤへの距離[mm]
  // r_wheel: タイヤの半径[mm]
  WheelOmega()
      : vx_(0), vy_(0), aimtheta_(0), theta_rad(THETA_RAD) //初期化
  {
    for (int i = 0; i < 4; i++) {
      omega[i] = 0.0;
      k[i] = 1;
    }
  }

  WheelOmega(float dist_wheel, float r_wheel)
      : vx_(0), vy_(0), aimtheta_(0), theta_rad(THETA_RAD) //初期化
  {
    for (int i = 0; i < 4; i++) {
      omega[i] = 0.0;
      k[i] = 1;
    }
    dist_wheel_ = dist_wheel;
    inv_r_wheel_ = 1.0f / r_wheel; //割り算は重い処理なので逆数をかける
  }

  // vx,vy: 機体座標での移動速度[mm/s]
  // aimtheta: 機体の回転速度[degree/s] 時計回りが正
  void setVxy(double vx, double vy, double aimtheta) {
    vx_ = vx;
    vy_ = vy;
    aimtheta_ = aimtheta * (3.14 / 180.0);
  }

  void calOmega() //タイヤそれぞれの目標角速度[rad/s]を計算
  {
    omega[0] = (vx_ * cos(theta_rad) + vy_ * sin(theta_rad) +
                dist_wheel_ * aimtheta_) *
               inv_r_wheel_ * k[0];
    omega[1] = (vx_ * cos(theta_rad) - vy_ * sin(theta_rad) +
                dist_wheel_ * aimtheta_) *
               inv_r_wheel_ * k[1];
    omega[2] = (-vx_ * cos(theta_rad) - vy_ * sin(theta_rad) +
                dist_wheel_ * aimtheta_) *
               inv_r_wheel_ * k[2];
    omega[3] = (-vx_ * cos(theta_rad) + vy_ * sin(theta_rad) +
                dist_wheel_ * aimtheta_) *
               inv_r_wheel_ * k[3];
  };
  double getOmega(int i) //タイヤの目標角速度[rad/s]を返す
  {
    return omega[i];
  }
  /*
  以下はクラスのコンストラクタで初期設定されている
  設定を変更したいときのみ呼び出す
  */
  //タイヤがエンコーダ正の時に反時計回りなら1、時計回りなら-1 (初期設定は全て1)
  void set_k(int k0, int k1, int k2, int k3) {
    k[0] = k0;
    k[1] = k1;
    k[2] = k2;
    k[3] = k3;
  }

private:
  double vx_, vy_, aimtheta_;
  double omega[4];
  const double theta_rad;
  float dist_wheel_;
  float inv_r_wheel_;
  int k[4];
};