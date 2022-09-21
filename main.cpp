/*
*手動機R1用
*F446RE, SPIマスター, 4輪オムニ
*os2
*
*classの使い方とかよくわかってないので、有識者様、修正してください。
*
*/
#include "mbed.h"
#include "math.h"
#include "EC.h"
#include "MotorController.h"
#include "CalPID.h"
#include "4omni.h"

//定数
const int RESOLUTION = 2048; //AMT102(default)は2048  分解能によって読み飛ばし量は変わらないっぽい(500)との比較

//ピン設定
//SPI用
SPI spi(PC_12, PC_11, PC_10); // mosi, miso, sclk
DigitalOut ss(PD_2);  //ssel  defaultPin

//SPI用データ配列
static const int buf_size = 8;   //データ配列の大きさ
uint8_t tx_buf[buf_size] = {};  //送信データ用バッファ配列   配列の先頭アドレスの値が格納される 多分今回は要らない
uint8_t rx_buf[buf_size] = {};  //受信データ用バッファ配列

int8_t RX, RY, LX, LY;
uint8_t R2, L2, up, down, left, right, square, cross, circle, triangle, R1, L1, R3, L3, share, options, PSButton, touchPad;
bool connect = 0;


//角速度を計算する間隔 303k8では0.01以上下げるとうまくいかなかった
#define DELTA_T 0.01
//dutyの絶対値の上限を決めて暴走を防ぐ
//タイヤCのみ変えているのはギア比が異なるためP制御がうまくかからなかったため
#define DUTY_MAX 0.40 //タイヤA,
//角度の上限 今回は不要だけどCalPIDの引数なので消さずに放置してください
#define OMEGA_MAX 6
//角速度を格納する配列の要素数
#define NUM_DATA 500

//上から順にタイヤA,B,Cの配列
//エンコーダ（A層,B層,分解能）
Ec2multi ec[]= {
    Ec2multi(PC_1,PC_0,RESOLUTION),
    Ec2multi(PC_2,PC_3,RESOLUTION),
    Ec2multi(PC_13,PA_4,RESOLUTION),
    Ec2multi(PA_15,PB_7,RESOLUTION)
};
//速度制御のPIDの値,角速度を計算する間隔,dutyの絶対値の上限
//2022.9.21現在未設定
CalPID speed_pid[]={
    CalPID(0.045,0.0,0.0,DELTA_T,DUTY_MAX),
    CalPID(0.05,0.0,0.0,DELTA_T,DUTY_MAX),
    CalPID(0.02,0.0,0.0005,DELTA_T,DUTY_MAX),
    CalPID(0.02,0.0,0.0005,DELTA_T,DUTY_MAX)
};
//角度制御のPID 今回は不要だけどMotorContorollerの引数なので消さずに放置してください
CalPID angle_omega_pid(0.0,0.0,0.0,DELTA_T,OMEGA_MAX);
//モーター（正転,逆転,PWM周期,エンコーダ,CALPIDの角速度PID,角度PID)
MotorController motor[]= {
    MotorController(PB_4,PB_5,50,ec[0],speed_pid[0],angle_omega_pid),
    MotorController(PB_2,PB_10,50,ec[1],speed_pid[1],angle_omega_pid),
    MotorController(PB_6,PA_9,50,ec[2],speed_pid[2],angle_omega_pid),
    MotorController(PA_5,PA_7,50,ec[3],speed_pid[3],angle_omega_pid)
};

Ticker ticker;  //割り込みタイマー
WheelOmega wheel; //wheelOmega型クラスのwheelを生成 4omni.h依存

//角速度を保存する変数と関数
float omega_saved[4][NUM_DATA]= {};
int omega_count=0;
float target_speed=0;
int save_count;
#define SAVE_COUNT_THRESHOLD 5 //5回ごとに角速度を配列に格納

//関数宣言
//モーターを目標角速度で動かそうとし、一定の間隔で角速度を配列に保存する関数
void saveOmega();
void motorOut();
void displayData();
//コントローラ(ESP32)と通信してコントローラの情報を得る
void getControl();
void unzipControl();

/////////////////////////////////////////////メイン///////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//loopするわけじゃない
int main(){
    //SPI初期化
    ss = 1;
    spi.format(buf_size * 8, 3);   //256bit,mode3
    spi.frequency(1400000); //要調整
    //SPI3はAPB1(45MHz)というクロックに繋がっているため、この分周でしか正しく通信できない 
    //ESP32のHSPIとの通信は1.4MHzでノイズ少なくできた   
    //なお、GNDだけでなく5Vも共通で接続することでノイズが激減した
    
    //現在の機体情報
    double vx = 0;
    double vy = 0;
    double aimtheta = 0;  //機体の回転速度[degree/s] 時計回りが正
    for(int i=0; i<3; i++) {
        //モーター加速度の上限
        //無視するために大きくしてある
        //機体が動き始めたときに動きがずれるなら加速度の上限を下げる
        motor[i].setAccelMax(10000000);
        //ギア比　タイヤ:エンコーダ＝１:rとしたときのrの値
        ec[i].setGearRatio(0.6667);
    }
    //角速度とduty比の関係式設定
    //横軸をタイヤの角速度[rad/s]とduty比とした時の傾きとy切片
    //正転時の傾き,y切片,逆転時の傾き,y切片
    motor[0].setEquation(0.009911272,0.008627973,-0.009919857,-0.009919857); 
    motor[1].setEquation(0.009657781,0.009657781,-0.010037358,-0.010037358);
    motor[2].setEquation(0.010134584,0.010134584,-0.010120037,0.014440573);
    motor[3].setEquation(0.010158297,0.010158297,-0.010374406,-0.010374406);

    printf("start\r\n");
    
    //機体の移動速度,回転速度を設定
    while(1) {
        getControl();   //コントローラ情報取得
        unzipControl(); //値解凍
        printf("Rx:%d  Ry:%d  ",RX,RY);
        printf("conn:%d  ",connect);
        if(!connect){
            printf("Disconnected\r\n");
            continue;  //コントローラが接続されていないとき、loop先頭へ戻る（暴走防止）
        }
        printf(" Rx : ");
        for(int i = 0; i < buf_size; i++){
            printf("%d,",rx_buf[i]);
        }
        printf("\r\n");
    }
    
    /*
    //break後
    //機体の移動速度,回転速度を記録
    printf("%f,%f,%f\r\n",vx,vy,aimtheta);
    //それぞれのタイヤの目標角速度を計算
    wheel.setVxy(vx, vy, aimtheta);
    wheel.calOmega();
    //それぞれのタイヤの目標角速度を記録
    printf("%f,%f,%f\r\n",wheel.getOmega(0),wheel.getOmega(1),wheel.getOmega(2));
    //3秒間モーターを回転させる
    ticker.attach(&motorOut,DELTA_T);
    wait(3.0);
    //モーターを止める
    ticker.detach();
    for(int i=0; i<3; i++) {
        motor[i].stop();
    }
    //タイヤの実際に出力されていた角速度を記録
    displayData();
    printf("\r\nfinish\r\n");
    */
}

void saveOmega(){//角速度を保存する
    if(omega_count<NUM_DATA) {
        for(int i=0; i<3; i++) {
            omega_saved[i][omega_count]=ec[i].getOmega();
        }
        omega_count++;
    }
}

void motorOut(){//モーターを目標角速度で動かそうとし、一定の間隔で角速度を配列に保存
    for(int i=0; i<3; i++) {
        motor[i].Sc(wheel.getOmega(i));
        if(save_count>=SAVE_COUNT_THRESHOLD) {
            saveOmega();
            save_count=0;
        }
    }
    save_count++;
}
void displayData(){//保存したデータをprintf
    printf("A\r\n");
    for(int i=0; i<omega_count; i++) {
        printf("%f,",omega_saved[0][i]);
        wait(0.05);//printf重いのでマイコンが落ちないように
    }
    printf("\r\nB\r\n");
    for(int i=0; i<omega_count; i++) {
        printf("%f,",omega_saved[1][i]);
        wait(0.05);//printf重いのでマイコンが落ちないように
    }
    printf("\r\nC\r\n");
    for(int i=0; i<omega_count; i++) {
        printf("%f,",omega_saved[2][i]);
        wait(0.05);//printf重いのでマイコンが落ちないように
    }
    omega_count=0;
}
//コントローラ情報取得
void getControl(){
    //通信開始
    ss = 0;
    for(int i = 0; i < buf_size; i++){
        rx_buf[i] = spi.write(0);  //受信用配列に格納
    }
    ss = 1;
    //通信終了    
}

//データ解凍
//rx_buf[0-3]がRスティックの情報
void unzipControl(){
    RX = rx_buf[0] - 128; //正と負を単純に足す<-128~127>
    RY = rx_buf[1] - 128;
    LX = rx_buf[2] - 128;
    LY = rx_buf[3] - 128;
    R2 = rx_buf[4]; //トリガ値 <0~255>
    L2 = rx_buf[5];

    up = (rx_buf[6] & 0b10000000) >> 7;    //rx_buf[10]の値はボタン8個の値を合成したものだから、ビットシフトで1桁に解凍
    down = (rx_buf[6] & 0b01000000) >> 6;
    left = (rx_buf[6] & 0b00100000) >> 5;
    right = (rx_buf[6] & 0b00010000) >> 4;
    square = (rx_buf[6] & 0b00001000) >> 3;
    cross = (rx_buf[6] & 0b00000100) >> 2;
    circle = (rx_buf[6] & 0b00000010) >> 1;
    triangle = rx_buf[6] & 0b00000001;
    R1 = (rx_buf[7] & 0b10000000) >> 7;
    L1 = (rx_buf[7] & 0b01000000) >> 6;
    R3 = (rx_buf[7] & 0b00100000) >> 5;
    L3 = (rx_buf[7] & 0b00010000) >> 4;
    share = (rx_buf[7] & 0b00001000) >> 3;
    options = (rx_buf[7] & 0b00000100) >> 2;
    PSButton = (rx_buf[7] & 0b00000010) >> 1;
    touchPad = rx_buf[7] & 0b00000001;
    //コントローラ非接続時暴走防止
    if(RX== -128 && RY== -128){
        connect = 0;
    }else{
        connect = 1;
    }
}