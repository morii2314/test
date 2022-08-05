#include <Arduino.h>
#include <iostream>
#include <iomanip>
#include <vector>  
using namespace std;

const float speed=10000;  //初期速度
const float Kp=700,Kd=200;//比例、微分ゲイン
float control;            //制御量
float error;              //偏差
float difference;         //偏差の変化量
float sum;                //センサ値の合計値 (センサ数が8なので範囲は0~8)
long timer;               //制御周期測定用のタイマー（マイコン起動からの時間をmsで保持）

vector<int> adc_pin{26,27,25,32,36,34,35,33};             //adc使用ピン
vector<float> sensor_position{-28,-20,-12,-4,4,12,20,28}; //センサ取付位置(mm)

class Sensor {
  private:
    int m_pin;  //使用するgpio
    int m_white;//白色での値
    int m_black;//黒色での値

  public:
    // デフォルトコンストラクタ
    Sensor();
    
    // コンストラクタ
    Sensor(int pin) : m_pin(pin){
        pinMode(m_pin,INPUT);
        //cout << "Call Constructor " << endl; 
    }
    
    // デストラクタ
    ~Sensor(){
        //cout << "Call Destructor" << endl;
    }
    
    // コピーコンストラクタ
    Sensor (const Sensor& cp) : m_pin(cp.m_pin){
        //cout << "Call Copy Constructor " << endl;
    }

    //メンバ変数取得用関数
    int getPin(){return m_pin;}
    int get_white(){return m_white;}
    int get_black(){return m_black;}

    //呼び出すたびにローパスフィルタを介して値が更新される
    void measure_white(){m_white = m_white*0.95+ analogRead(m_pin)*0.05;}
    void measure_black(){m_black = m_black*0.95+ analogRead(m_pin)*0.05;}

    //センサデータを補正したうえで返す関数
    float get_val(){
      int Raw_data=analogRead(m_pin);
      float val=float(Raw_data-m_white)/float(m_black-m_white);

      //事前に黒白での値を正しく計測できていれば0~1に収まるはずなので上下限を設定
      if(val>1.0f)val=1.0f;
      if(val<0.0f)val=0.0f;
      return val;
    }
};

vector<Sensor> sensor;

//最初の一回だけ呼び出される
void setup(void){
  Serial.begin(115200);//シリアルモニタのボーレート

  //sensorベクターにSensorクラスを格納
  for(auto itr = adc_pin.begin();itr != adc_pin.end() ;++itr){//宣言したadcピンの数だけ繰り返し
    sensor.push_back(Sensor(*itr));
  }
  
  //黒色での値を記録
  cout << "black" << endl;
  for (size_t i = 0; i < 3000; i++)
    for (size_t j = 0; j < adc_pin.size(); j++)
      sensor.at(j).measure_black();

  delay(2000);//2秒待機

  //白色での値を記録
  cout << "white" << endl;
  for (size_t i = 0; i < 3000; i++)
    for (size_t j = 0; j < adc_pin.size(); j++)
      sensor.at(j).measure_white();

  //モータドライバに指示するためのPWM信号の設定
  ledcSetup(0,12800,8);//(チャンネル、初期周波数、分解能bit)
  ledcWrite(0,128);//(チャンネル、デューティ比)
  ledcAttachPin(18,0);//(ピン番号、チャンネル)
  ledcWriteTone(0,0.0001);//(チャンネル、周波数)
  ledcSetup(2,12800,8);
  ledcWrite(2,128);
  ledcAttachPin(23,2);
  ledcWriteTone(2,0.0001);

  timer=millis();//制御周期を測定するためのタイマー
}


//setup関数の後にループする
void loop(){
  //センサ値の取得
  cout << "\nSensorArray= [ ";
  vector<float>sensor_val;
  for (size_t i = 0; i < adc_pin.size(); i++){
    float Corrected_data=sensor.at(i).get_val();
    sensor_val.push_back(Corrected_data);
    cout << fixed << setprecision(2) << sensor_val[i] << " " ;
  }
  cout << "]  ";

  difference=error;//前回の偏差の保存

  // 最初の要素を指すイテレータ
  auto val = sensor_val.begin();
  auto pos = sensor_position.begin();

  error=0;
  sum=0;

  //偏差:errorと測定値の和:sumを算出
  for (size_t i = 0; i < adc_pin.size(); i++){
    error += *val * *pos;
    sum += *val;
    ++val;
    ++pos;
  }
  error /= sum;

  difference=error-difference;//偏差の変化量
  control=Kp*error + Kd*difference/(millis()-timer);//制御量の決定
  timer=millis();//制御周期を測定するためのタイマーの更新

  cout << "Error= " << fixed << setprecision(2) << error << "  " ;

  float R=speed,L=speed;

  //センサ値の合計が一定以下の場合、黒色ラインを見失ったとする
  if(sum>0.5){//ラインを検出しているため制御量に応じて曲がる

    //制御量の分だけ片輪の回転速度を落とす
    if(control>0)R-=control;
    else{L+=control;}
  }

  else{ //ラインを見失っているため左に急旋回

    R=speed/2;
    L=0;
  }


  //マイナスの速度を入力できないため下限を設ける
  if(R<0)R=0;
  if(L<0)L=0;
  
  //モータドライバに速度を指示
  ledcWriteTone(0,L);
  ledcWriteTone(2,R);
}