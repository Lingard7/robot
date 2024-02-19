#define dataLength 100                    //データ最大長
#define sizeAry 10                        //配列サイズ
#define sizeDis 40                        //距離データ長
#define stopDis 300                       //LiDAR ストップ距離
#define maxDiv 360                        //最大分割数
#define dispR 1
#define dispL 0

int lost_flg = 0;                         //ロストフラグ

enum states {                              //列挙型で変数を定義
  WAIT_AA,
  WAIT_55,
  READING_DATA,
  ANALYSIS_DATA
};

byte state = WAIT_AA;
static byte dataNo = 0;

unsigned int data[sizeAry][dataLength];      //データ配列
unsigned int lsn[sizeAry];                   //点群データ数
unsigned int fsa[sizeAry];                   //開始角度配列
unsigned int lsa[sizeAry];                   //終了角度配列
unsigned int ang[sizeAry];                   //角度配列
unsigned int dis[sizeAry][sizeDis];          //距離配列
unsigned int angDiv[maxDiv];                 //分割角度配列
unsigned int disDiv[maxDiv];                 //分割距離配列

const int MOTOR_LEFT1 = 3;        //右モーター
const int MOTOR_LEFT2 = 4;
const int MOTOR_RIGHT1 = 6;       //左モーター
const int MOTOR_RIGHT2 = 7;
const int MOTOR_PWMleft = 9;      //左モーターのpwm
const int MOTOR_PWMright = 10;    //右モーターのpwm
static float Lspeed = 0;          //左モーター速度
static float Rspeed = 0;          //右モーター速度
int obsAng;                       //障害物角度

void setup()
{
  Serial.begin(115200);                   //シリアル通信設定
  Serial1.begin(115200);

  pinMode(MOTOR_LEFT1, OUTPUT);
  pinMode(MOTOR_LEFT2, OUTPUT);
  pinMode(MOTOR_RIGHT1, OUTPUT);
  pinMode(MOTOR_RIGHT2, OUTPUT);

  pinMode(5, OUTPUT);
}

void loop()
{
  lid();
}

void lid() {
  while (Serial1.available() > 0) {
    switch (state) {                                  //stateに応じて処理
      case WAIT_AA:
        if (Serial1.read() == 0xAA) {                 //AA取得確認
          state = WAIT_55;
        }
        break;

      case WAIT_55:                                   //55取得確認
        if (Serial1.read() == 0x55) {
          state = READING_DATA;
        }
        else {
          state = WAIT_AA;
        }
        break;

      case READING_DATA:                              //AA55以降のデータを読み込み配列に格納
        static byte count = 0;
        data[dataNo][count] = Serial1.read();
        lsn[dataNo] = data[dataNo][1];
        count++;

        if (count >= lsn[dataNo] * 2 + 8) {
          state = WAIT_AA;
          count = 0;
          dataNo++;
          if (dataNo >= sizeAry)    state = ANALYSIS_DATA;
        }
        break;

      case ANALYSIS_DATA:                             //データ解析
        getFsa();                                     //開始角度取得関数
        getLsa();                                     //終了角度取得関数
        getDis();                                     //距離取得関数
        //lidStop();                          //一定範囲に障害物が入ると止まる
        splitAng();                                   //分割角度,距離取得関数
        liddemo();                                    //lidar機能確認関数
        lidm();                                       //障害物回避関数
        dataClr();                                    //全配列クリア関数
        state = WAIT_AA;
        break;
    }
  }
}

void getFsa() {                                       //開始角度取得関数
  unsigned int anaData;
  for (byte i = 0; i < sizeAry; i++) {
    anaData = (data[i][3] << 8) + data[i][2];          //エンディアン変換
    fsa[i] = (anaData >> 1) / 64;
    if (fsa[i] > 360)  fsa[i] = fsa[i] - 360;
  }
}

void getLsa() {                                         //終了角度取得関数
  unsigned int anaData;
  for (byte i = 0; i < sizeAry; i++) {
    anaData = (data[i][5] << 8) + data[i][4];           //エンディアン変換
    lsa[i] = (anaData >> 1) / 64;
    if (lsa[i] > 360)  lsa[i] = lsa[i] - 360;
  }
}

void getDis() {                                         //距離取得関数
  static byte j = 0;
  static int cnt = 0;
  for (byte disNo = 0; disNo < sizeAry; disNo++) {
    j = 0;
    for (byte i = 8; i < lsn[disNo] * 2 + 8; i += 2) {
      dis[disNo][j] = ((data[disNo][i + 1] << 8) + data[disNo][i]) / 4;
      j++;
    }
  }
  cnt = 0;
}

void lidStop() {                                              //一定範囲に障害物が入ると止まる関数
  for (byte i = 0; i < sizeAry; i++) {
    for (byte j = 0; j < sizeDis; j++) {
      if (dis[i][j] < stopDis){                               //stopDis内に物があれば停止
        Lspeed = 0;
        Rspeed = 0;
        motor(Lspeed, Rspeed);
      }
    }
  }
}

void splitAng() {                   //角度,距離分割関数
  float diffAng, divs;
  float plusAng = 0;
  static float plus = 0;
  static int cnt = 0;

  //差角を求める処理
  if (fsa[cnt] > lsa[cnt])  diffAng = 360 - fsa[cnt] + lsa[cnt];
  else  diffAng = lsa[cnt] - fsa[cnt];
  plus = diffAng / lsn[cnt];

  //角度,距離を点群の数で分割し配列に格納
  //配列に格納する際, 配列の要素番号と角度, 対応した距離を同じにして格納
  for (int i = 0; i < lsn[cnt]; i++) {
    divs = fsa[cnt] + plusAng;
    plusAng = plusAng + plus;
    if (divs >= 360)  divs = divs - 360;
    int int_divs = divs;
    angDiv[int_divs] = int_divs;
    disDiv[int_divs] = dis[cnt][i];
  }

  cnt++;
  if (cnt >= sizeAry)   cnt = 0;
}

int findObs(int sta, int eda) {                   //1番近い距離を返す関数   引数:(始まりの角度, 終わりの角度)
  int dis1 = 100000;
  int dis2 = 100000;
  int dis3 = 100000;
  int obsAng1, obsAng2;

  //引数が2通りの場合を考える
  if (sta > eda) {                                //開始角度が終了角度より大きい場合
    for (int i = sta; i < maxDiv; i++) {          //開始角度~360°までループ
      if (dis1 > disDiv[i] && disDiv[i] > 0){
        dis1 = disDiv[i];                         //開始角度~360°で1番近い距離をdis1に格納
        obsAng1 = i;                              //dis1の角度を格納
      }
    }
    for (int i = 0; i < eda; i++) {               //0°~終了角度までループ
      if (dis2 > disDiv[i] && disDiv[i] > 0) {
        dis2 = disDiv[i];                         //0°~終了角度で1番近い距離をdis2に格納
        obsAng2 = i;                              //dis2の角度を格納
      }
    }

    //dis1, dis2を比較して小さい方をdis3に,obsAngをそれぞれ格納
    if (dis1 < dis2){
      dis3 = dis1;
      obsAng = obsAng1;
    }
    if (dis1 > dis2){
      dis3 = dis2;
      obsAng = obsAng2;
    }
  }

  else {                                          //開始角度が終了角度より小さい場合
    for (int i = sta; i < eda; i++) {             //開始角度~終了角度までループ
      if (dis3 > disDiv[i] && disDiv[i] > 0) {    //1番近い距離,角度を格納
        dis3 = disDiv[i];
        obsAng = i;
      }
    }
  }

  if (dis3 < 0)  dis3 = 0;
  if (dis3 != 100000 && dis3 >= 0)  return dis3;  //初期値以外かつ0以上を返す
}

void liddemo() {                                                  //1m以内にある1番近いものに反応して追う機能
  int obs = findObs(0, 360);
  
  if (obs <= 1000 && obs > 0) {                                   //検出距離が1m以内にある場合
    if (obsAng > 20 && obsAng < 180) {                            //検出距離が20°~180°にある場合
      Lspeed = 80;
      Rspeed = 80;
      motorR(Lspeed, Rspeed);                                     //右に回転 

      static unsigned long millis_buf = 0;                        //delay処理
      while ((millis() - millis_buf) < 290 + obsAng) {}           //検出角度に応じてdelay時間を変更
      millis_buf = millis();
    }
    
    else if (obsAng < 340 && obsAng > 180) {                      //検出距離が180°~340°にある場合
      Lspeed = 80;
      Rspeed = 80;
      motorL(Lspeed, Rspeed);                                     //左に回転

      static unsigned long millis_buf = 0;                        //delay処理
      while ((millis() - millis_buf) < 290 + obsAng - 180) {}     //検出角度に応じてdelay時間を変更
      millis_buf = millis();
    }
    
    else {                                                        //1番近い距離が340°~20°にある場合
      if (obs >= 450) {                                           //距離が45cm以上の場合
        Lspeed = 80;
        Rspeed = 80;
        motor(Lspeed, Rspeed);                                    //前進

        static unsigned long millis_buf = 0;                     //delay処理
        while ((millis() - millis_buf) < obs * 1.5) {}           //検出距離に応じてdelay時間を変更
        millis_buf = millis();
      }
      
      else{                                                     //45cm以下を検出した場合
        Lspeed = 0;
        Rspeed = 0;
        motor(Lspeed, Rspeed);                                  //モーター停止
      }
    }
  }
  
  else {                                                        //1m以上を検出した場合
    Lspeed = 0;
    Rspeed = 0;
    motor(Lspeed, Rspeed);                                     //モーター停止
  }
}

void lidm() {                                          //障害物回避関数
  int fl = findObs(310, 360);
  int fr = findObs(0, 50);

  if (fr <= 1100 && fr > 0) {
    Lspeed = fr * 0.2;
    if (Lspeed < 0)  Lspeed = 0;
    motor(Lspeed, Rspeed);

    static unsigned long millis_buf = 0;                //delay処理
    while ((millis() - millis_buf) < 200) {}
    millis_buf = millis();
  }

  else if (fl <= 1100 && fl > 0) {
    Rspeed = fl * 0.2;
    if (Rspeed < 0)  Rspeed = 0;
    motor(Lspeed, Rspeed);

    static unsigned long millis_buf = 0;                //delay処理
    while ((millis() - millis_buf) < 200) {}
    millis_buf = millis();
  }

  else {
    Lspeed += 50;
    Rspeed += 50;
    if (Lspeed > 255)  Lspeed = 255;
    if (Rspeed > 255)  Rspeed = 255;
    motor(Lspeed, Rspeed);
  }
}

void motor(int Lspeed, int Rspeed) {        //モーター前回転
  analogWrite(MOTOR_PWMleft, Lspeed);
  digitalWrite(MOTOR_LEFT1, HIGH);
  digitalWrite(MOTOR_LEFT2, LOW);

  analogWrite(MOTOR_PWMright, Rspeed);
  digitalWrite(MOTOR_RIGHT1, HIGH);
  digitalWrite(MOTOR_RIGHT2, LOW);
}

void motorL(int Lspeed, int Rspeed) {       //モーター左に回転
  analogWrite(MOTOR_PWMleft, Lspeed);
  digitalWrite(MOTOR_LEFT1, LOW);
  digitalWrite(MOTOR_LEFT2, HIGH);

  analogWrite(MOTOR_PWMright, Rspeed);
  digitalWrite(MOTOR_RIGHT1, HIGH);
  digitalWrite(MOTOR_RIGHT2, LOW);
}

void motorR(int Lspeed, int Rspeed) {       //モーター右に回転
  analogWrite(MOTOR_PWMleft, Lspeed);
  digitalWrite(MOTOR_LEFT1, HIGH);
  digitalWrite(MOTOR_LEFT2, LOW);

  analogWrite(MOTOR_PWMright, Rspeed);
  digitalWrite(MOTOR_RIGHT1, LOW);
  digitalWrite(MOTOR_RIGHT2, HIGH);
}

void dataClr() {                              //全配列クリア関数
  for (int i = 0; i < sizeAry; i++) {
    for (int j = 0; j < dataLength; j++) {
      data[i][j] = 0;
    }
  }

  for (int i = 0; i < sizeAry; i++) {
    for (int j = 0; j < sizeDis; j++) {
      dis[i][j] = 0;
    }
  }

  for (int i = 0; i < sizeAry; i++) {
    lsn[i] = 0;
    fsa[i] = 0;
    lsa[i] = 0;
    ang[i] = 0;
  }

  for (int i = 0; i < maxDiv; i++) {
    angDiv[i] = 0;
    disDiv[i] = 0;
  }
}
