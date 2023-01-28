#pragma once
//四輪を動かすためのクラス
//機体座標のx軸正を右向き、y軸を上向きに取ったとき
//タイヤ0:左上、タイヤ1:右上、タイヤ2:右下、タイヤ3:左下
#define THETA_RAD 45.0 / 180.0 * 3.14

class WheelOmega {
public:
  //コンストラクタの定義 main関数の前に必ず一度宣言する
  // dist_wheel: 機体中心からタイヤへの距離[mm]
  // r_wheel: タイヤの半径[mm]
  WheelOmega(float dist_wheel, float r_wheel); //初期化

  // vx,vy: 機体座標での移動速度[mm/s]
  // aimtheta: 機体の回転速度[degree/s] 時計回りが正
  void setVxy(double vx, double vy, double aimtheta);

  void calOmega(); //タイヤそれぞれの目標角速度[rad/s]を計算
  double getOmega(int i) //タイヤの目標角速度[rad/s]を返す
  {
    return omega[i];
  }
  /*
  以下はクラスのコンストラクタで初期設定されている
  設定を変更したいときのみ呼び出す
  */
  //タイヤがエンコーダ正の時に反時計回りなら1、時計回りなら-1 (初期設定は全て1)
  void set_k(int k0, int k1, int k2, int k3);

private:
  double vx_, vy_, aimtheta_;
  double omega[4];
  const double theta_rad;
  float dist_wheel_;
  float inv_r_wheel_;
  int k[4];
};
