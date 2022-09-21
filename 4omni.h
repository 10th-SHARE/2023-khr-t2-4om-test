#pragma once
class WheelOmega
{
public:
    WheelOmega(): vx_(0),vy_(0),rotate_(0) //初期化
    {
        for(int i =0; i<4; i++) {
            omega[i]=0;
        }
    }
    void setVxy(double vx,double vy,double rotate)
    {
        //vx,vyは機体の移動速度[mm/s]
        //rotateは機体の回転速度[degree/s] 時計回りが正
        vx_=vx;
        vy_=vy;
        rotate_=rotate;
    }
    double deg_to_rad(double rotate_) //degreeをradに変換
    {
        return rotate_*(3.14/180);
    }
    void calOmega() //タイヤそれぞれの目標角速度[rad/s]を計算
    {
        omega[0]=(vx_*cos(theta_rad)+vy_*sin(theta_rad)+dist_wheel*deg_to_rad(rotate_))*inv_rad_wheel*k0;
        omega[1]=(vx_*cos(theta_rad)-vy_*sin(theta_rad)+dist_wheel*deg_to_rad(rotate_))*inv_rad_wheel*k1;
        omega[2]=(-vx_*cos(theta_rad)-vy_*sin(theta_rad)+dist_wheel*deg_to_rad(rotate_))*inv_rad_wheel*k2;
        omega[3]=(-vx_*cos(theta_rad)+vy_*sin(theta_rad)+dist_wheel*deg_to_rad(rotate_))*inv_rad_wheel*k3;
    };
    double getOmega(int i) //タイヤの目標角速度[rad/s]返す
    {
        return omega[i];
    }
private:
    double vx_,vy_,rotate_;
    double omega[4];
    const double theta_rad=45.0/180*3.14;
    const double dist_wheel=208.42; //機体中心からタイヤへの距離[mm]
    const double inv_rad_wheel=0.0196; //タイヤの半径の逆数[1/mm] 割り算は重い処理なので逆数かける
    const int k0=1, k1=1, k2=1, k3=1; //タイヤが正転時に反時計回りなら1,時計回りなら-1
};

