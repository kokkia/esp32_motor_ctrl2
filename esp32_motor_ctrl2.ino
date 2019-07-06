#include "kal/kal.h"

#define DEBUG 1

//motor
kal::nxtmotor motor;
//motor control gain
#define KP 90.0
#define KD 0.2
#define KDD 0.05

//reference
kal::wave sin_wave(0.0,PI/3,0.5,SIN);

//robotdata
kal::RobotData ref;
kal::RobotData state;

//differentiator
kal::Diff<double> dtheta_st(0.0,100.0);
kal::Diff<double> dtheta_ref(0.0,100.0);

double t = 0.0;//time

//timer関連
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onTimer() {  /* this function must be placed in IRAM */
  portENTER_CRITICAL_ISR(&timerMux);
  //control---------------------------------------------------------------------------------------------------------------------------/
  t += Ts;
  //状態取得
  motor.get_angle(PCNT_UNIT_0,state.theta);
  dtheta_st.update(state.theta);
  state.dtheta = dtheta_st.x;
  
  //目標値計算
  sin_wave.update();
  ref.theta = sin_wave.output;
  dtheta_ref.update(ref.theta);
  ref.dtheta = dtheta_ref.x;
  //出力計算
  double u = KP*(ref.theta-state.theta) + KD * (ref.dtheta - state.dtheta);
  motor.drive(u);

#if DEBUG    
  Serial.print(ref.theta*RAD2DEG);
  Serial.print(",");
  Serial.println(state.theta*RAD2DEG);
#endif
  //-------------------------------------------------------------------------------------------------------------------------------------/
  portEXIT_CRITICAL_ISR(&timerMux);
  }


void setup() {
  Serial.begin(115200);
  Serial.println("started");
  
  //motorの設定
  motor.GPIO_setup(GPIO_NUM_14,GPIO_NUM_27);//方向制御ピン設定
  motor.PWM_setup(GPIO_NUM_12,0);//PWMピン設定
  motor.encoder_setup(PCNT_UNIT_0,GPIO_NUM_25,GPIO_NUM_26);//エンコーダカウンタ設定

  //timer割り込み設定
  timer = timerBegin(0, 80, true);//プリスケーラ設定
  timerAttachInterrupt(timer, &onTimer, true);//割り込み関数指定
  timerAlarmWrite(timer, (int)(Ts*1000000), true);//Ts[s]ごとに割り込みが入るように設定
  timerAlarmEnable(timer);//有効化
}
void loop() {
}
