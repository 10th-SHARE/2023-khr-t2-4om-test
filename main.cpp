#include "mbed.h"
#include "WheelOmega.h"
#include "EC.h"
#define RESOLUTION 2048
#include "CalPID.h"
#include "MotorController.h"
Serial pc(USBTX,USBRX);

//デバッグ用
int8_t RX, RY, LX, LY;
uint8_t R2, L2, up, dwn, lft, rit, sqr, crss, cicl, tri, R1, L1, R3, L3, shr, opt, PSb, touc;
//const int RECIEVE_CAN_ID = 0x12;


const int RECIEVE_CAN_ID = 0x10; //コントローラーデバッグ時はコメントアウト
const int CAN_Hz = 1000000;//CANに使用するクロック周波数[Hz]. CAN通信相手と共通させる

CAN can(PA_11,PA_12,CAN_Hz);//CAN_RD, CAN_TDの順

//角速度を計算する間隔 303k8では0.01以上下げるとうまくいかなかった
#define DELTA_T 0.01
//5回ごとに角速度を配列に格納
#define SAVE_COUNT_THRESHOLD 5
//dutyの絶対値の上限を決めて暴走を防ぐ
//タイヤCのみ変えているのはギア比が異なるためP制御がうまくかからなかったため
#define DUTY_MAX 0.5
//角度の上限 今回は不要だけどCalPIDの引数なので消さずに放置してください
#define OMEGA_MAX 6
//角速度を格納する配列の要素数
#define NUM_DATA 100
// オムニ輪の機体中心からの距離
#define WHEEL_DIST 0
// オムニ輪の半径(mm)
#define WHEEL_R 0


//上から順にタイヤA,B,Cの配列
//エンコーダ（A層,B層,分解能）
Ec2multi ec[]= {
    Ec2multi(PC_3,PC_2,RESOLUTION),
    Ec2multi(PC_12,PC_10,RESOLUTION),
    Ec2multi(PB_14,PB_15,RESOLUTION),
    Ec2multi(PC_6,PC_8,RESOLUTION) 
};
//速度制御のPIDの値,角速度を計算する間隔,dutyの絶対値の上限
CalPID speed_pid[]= {
    CalPID(0.0,0.0,0.0,DELTA_T,DUTY_MAX),
    CalPID(0.0,0.0,0.0,DELTA_T,DUTY_MAX),
    CalPID(0.0,0.0,0.0,DELTA_T,DUTY_MAX),
    CalPID(0.0,0.0,0.0,DELTA_T,DUTY_MAX)
};
//角度制御のPID 今回は不要だけどMotorContorollerの引数なので消さずに放置してください
CalPID angle_omega_pid(0.0,0.0,0.0,DELTA_T,OMEGA_MAX);
//モーター（正転,逆転,PWM周期,エンコーダ,CALPIDの角速度PID,角度PID)
MotorController motor[]= {
    MotorController(PA_11,PB_1,50,ec[0],speed_pid[0],angle_omega_pid),
    MotorController(PB_5,PB_4,50,ec[1],speed_pid[1],angle_omega_pid),
    MotorController(PA_0,PA_1,50,ec[2],speed_pid[2],angle_omega_pid),
    MotorController(PA_8,PA_9,50,ec[3],speed_pid[3],angle_omega_pid)
};

Ticker ticker;  //割り込みタイマー
WheelOmega wheel(WHEEL_DIST, WHEEL_R); //クラスwheelOmegaを以下wheelとする

//角速度を保存する変数と関数
float aimomega_saved[4][NUM_DATA] = {};
float omega_saved[4][NUM_DATA] = {};
int omega_count=0;
float target_speed=0;
int save_count;

void saveData();
void timercallback();
void displayData();
void CAN_recieve();
void unzipControl();//デバッグ用

double vx = 0;
double vy = 0;
double aimtheta = 0;  //機体の回転速度[degree/s] 時計回りが正
double theta = 0; //ジャイロ情報

int main()
{

    can.filter(RECIEVE_CAN_ID, 0xFFF, CANStandard, 0);////MY_CAN_ID以外のデータを受け取らないよう設定．後半の0xFFF, CANStandard, 0);は基本変えない

    
    for(int i=0; i<4; i++) {
        //モーター加速度の上限
        //無視するために大きくしてある
        //機体が動き始めたときに動きがずれるなら加速度の上限を下げる
        motor[i].setAccelMax(10000000);
        //ギア比　タイヤ:エンコーダ＝１:rとしたときのrの値
        ec[i].setGearRatio(1.0);
    }
    //角速度とduty比の関係式設定
    //横軸をタイヤの角速度[rad/s]とduty比とした時の傾きとy切片
    //正転時の傾き,y切片,逆転時の傾き,y切片
    motor[0].setEquation(0.037744647,0.011749343,-0.03795788,0.010164383);
    motor[1].setEquation(0.038135727,0.008930954,-0.039235654,0.016065535);
    motor[2].setEquation(0.048000336,0.045173443,-0.037912833,0.054170503);
    motor[3].setEquation(0.040257293,0.005462202,-0.043409469,0.00782965);

    printf("start\r\n");
    ticker.attach(&timercallback,DELTA_T);
}

void saveData()
{
    if(omega_count<NUM_DATA) {
        for(int i=0; i<4; i++) {
            aimomega_saved[i][omega_count]=wheel.getOmega(i);
            omega_saved[i][omega_count]=ec[i].getOmega();
        }
        omega_count++;
    }
}

void timercallback()//モーターを目標角速度で動かそうとし、一定の間隔で角速度を配列に保存
{
    CAN_recieve();
    //unzipControl(); //デバッグ用
    ///////ここvxvyベタ打ちで動く？？？？


    printf("vx:%11lf, ",vx);
    printf("vy:%11lf, ",vy);
    printf("aimtheta:%11lf, ",aimtheta);
    printf("theta:%11lf, ",theta);
    printf("\r\n");


    wheel.setVxy(vx, vy, aimtheta);
    wheel.calOmega();
    for(int i=0; i<4; i++) {
        motor[i].Sc(wheel.getOmega(i));
        if(save_count>=SAVE_COUNT_THRESHOLD) {
            saveData();
            save_count=0;
        }
    }
    save_count++;
}

void displayData()//保存したデータをprintf
{
    printf("A_aimomega\r\n");
    for(int i=0; i<omega_count; i++) {
        printf("%f,",aimomega_saved[0][i]);
        wait(0.05);//printf重いのでマイコンが落ちないように
    }
    printf("\r\nB_aimomega\r\n");
    for(int i=0; i<omega_count; i++) {
        printf("%f,",aimomega_saved[1][i]);
        wait(0.05);//printf重いのでマイコンが落ちないように
    }
    printf("\r\nC_aimomega\r\n");
    for(int i=0; i<omega_count; i++) {
        printf("%f,",aimomega_saved[2][i]);
        wait(0.05);//printf重いのでマイコンが落ちないように
    }
    printf("\r\nD_aimomega\r\n");
    for(int i=0; i<omega_count; i++) {
        printf("%f,",aimomega_saved[3][i]);
        wait(0.05);//printf重いのでマイコンが落ちないように
    }
    printf("A_omega\r\n");
    for(int i=0; i<omega_count; i++) {
        printf("%f,",omega_saved[0][i]);
        wait(0.05);//printf重いのでマイコンが落ちないように
    }
    printf("\r\nB_omega\r\n");
    for(int i=0; i<omega_count; i++) {
        printf("%f,",omega_saved[1][i]);
        wait(0.05);//printf重いのでマイコンが落ちないように
    }
    printf("\r\nC_omega\r\n");
    for(int i=0; i<omega_count; i++) {
        printf("%f,",omega_saved[2][i]);
        wait(0.05);//printf重いのでマイコンが落ちないように
    }
    printf("\r\nD_omega\r\n");
    for(int i=0; i<omega_count; i++) {
        printf("%f,",omega_saved[3][i]);
        wait(0.05);//printf重いのでマイコンが落ちないように
    }
    omega_count=0;
}


void CAN_recieve()
{
    CANMessage msg;// 送られてきたデータを入れる箱

    if(can.read(msg)) {
        //CANデータの読み取り
        vx = (msg.data[0] - 128)*4; //<-128~127>に変換
        vy = (msg.data[1] - 128)*4;
        aimtheta = msg.data[2] - 128;
        theta = (msg.data[3] - 128)*2;
    }
}

//デバッグ用
void unzipControl()
{
    CANMessage msg;// 送られてきたデータを入れる箱

    if(can.read(msg)) {
        ;//CANデータの読み取り


        RX = msg.data[0] - 128; //<-128~127>に変換
        RY = msg.data[1] - 128;
        LX = msg.data[2] - 128;
        LY = msg.data[3] - 128;
        R2 = msg.data[4]; //トリガ値 <0~255>
        L2 = msg.data[5];
        up = (msg.data[6] & 0b10000000) >> 7;    //rx_buf[6]の値はボタン8個の値を合成したものだから、ビットシフトで1桁に解凍
        dwn = (msg.data[6] & 0b01000000) >> 6;
        lft = (msg.data[6] & 0b00100000) >> 5;
        rit = (msg.data[6] & 0b00010000) >> 4;
        sqr = (msg.data[6] & 0b00001000) >> 3;
        crss = (msg.data[6] & 0b00000100) >> 2;
        cicl = (msg.data[6] & 0b00000010) >> 1;
        tri = msg.data[6] & 0b00000001;
        R1 = (msg.data[7] & 0b10000000) >> 7;
        L1 = (msg.data[7] & 0b01000000) >> 6;
        R3 = (msg.data[7] & 0b00100000) >> 5;
        L3 = (msg.data[7] & 0b00010000) >> 4;
        shr = (msg.data[7] & 0b00001000) >> 3;
        opt = (msg.data[7] & 0b00000100) >> 2;
        PSb = (msg.data[7] & 0b00000010) >> 1;
        touc = msg.data[7] & 0b00000001;

        if(-10 < LX && LX <10) {
            LX = 0;
        }

        if(-10 < LY && LY < 10) {
            LY = 0;
        }

        vx = LX*4;
        vy = LY*4;
        aimtheta = (R2 - L2)/3;

        /*
                printf("LX:%4d, ",LX);
                printf("LY:%4d, ",LY);
                printf("RX:%4d, ",RX);
                printf("RY:%4d, ",RY);
                printf("L2:%3d, ",L2);
                printf("R2:%3d, ",R2);
                printf("L1:%d, ",L1);
                printf("R1:%d, ",R1);
                printf("L3:%d, ",L3);
                printf("R3:%d, ",R3);
                printf("↑:%d, ",up);
                printf("↓:%d, ",dwn);
                printf("←:%d, ",lft);
                printf("→:%d, ",rit);
                printf("□:%d, ",sqr);
                printf("×:%d, ",crss);
                printf("〇:%d, ",cicl);
                printf("△:%d, ",tri);
                printf("shr:%d, ",shr);
                printf("opt:%d, ",opt);
                printf("PSb:%d, ",PSb);
                printf("touc:%d, ",touc);
                printf("\r\n");
        */
    }
}