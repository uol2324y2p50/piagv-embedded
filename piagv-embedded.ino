#include "PinChangeInterrupt.h"

////////////////////////////////////////////////////////ENCODER////////////////////////////////////////////////////////

// 用于存储脉冲数的变量
volatile long encPulses[4] = {0, 0, 0, 0};
// 上一次A相和B相的状态
volatile byte lactEnc[4] = {0, 0, 0, 0};

template <int encNo, int pinA, int pinB>
class Encoder {
public:
  Encoder() {
    // 将引脚设为输入
    pinMode(pinA, INPUT_PULLUP);
    pinMode(pinB, INPUT_PULLUP);
        
    // 读取初始的A相和B相状态，组合成两位的状态
    lactEnc[encNo] = (digitalRead(pinA) << 1) | digitalRead(pinB);
      
    // 设置中断，当A或B相发生变化时调用encoderISR
    attachPCINT(digitalPinToPCINT(pinA), &Encoder::isr, CHANGE);
    attachPCINT(digitalPinToPCINT(pinB), &Encoder::isr, CHANGE);
  }

  static void isr() {
    // 读取当前的A相和B相状态，组合成两位的状态
    byte encoded = (digitalRead(pinA) << 1) | digitalRead(pinB);
    byte sum = (lactEnc[encNo] << 2) | encoded;
      
    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encPulses[encNo]++;
    if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encPulses[encNo]--;
      
    lactEnc[encNo] = encoded; // 更新上一次的状态
  }
};

////////////////////////////////////////////////////////FILTER////////////////////////////////////////////////////////

class LowPassFilter {
public:
  // 构造函数，alpha取值范围为0到1
  LowPassFilter(float alpha) : y_prev(0), alpha(alpha) {}

  // 执行滤波操作
  float filter(float x) {
    float y = alpha * x + (1 - alpha) * y_prev;
    y_prev = y;
    return y;
  }
  
private:
  float y_prev; // 上一次的滤波结果
  float alpha;  // 滤波系数
};

////////////////////////////////////////////////////////MOTOR////////////////////////////////////////////////////////

class Motor {
public:
  Motor(int _pwmPin, int _dirPin, int _ppr) : pwmPin(_pwmPin), dirPin(_dirPin), ppr(_ppr), lastEnc(0), lastTime(millis()), speed(0), delta(0) {
    pinMode(this->pwmPin, OUTPUT);
    pinMode(this->dirPin, OUTPUT);
    analogWrite(this->pwmPin, 0);
  }
  
  void updateEnc(long enc) {
    unsigned long time_now = millis();
    this->delta = (float)(enc - this->lastEnc) / (float)this->ppr * 2.0f * PI;
    this->speed = this->delta / (float)(time_now - this->lastTime) * 1000.0f;
    this->lastEnc = enc;
    this->lastTime = time_now;
  }

  void setOutput(float output) {
    output = constrain(output, -1.0f, 1.0f);
    if (output >= 0) {
      digitalWrite(this->dirPin, HIGH);
    } else {
      digitalWrite(this->dirPin, LOW);
    }
    analogWrite(this->pwmPin, (int) (abs(output) * 255.0f));
  }
  
  int ppr;
  float delta;
  float speed;
private:
  int pwmPin;
  int dirPin;
  long lastEnc;
  unsigned long lastTime;
};

////////////////////////////////////////////////////////PID////////////////////////////////////////////////////////

class PID {
public:
  PID(float _kp, float _ki, float _kd, float _max_i, float _iout_max, float _out_max) : kp(_kp), ki(_ki), kd(_kd), max_i(_max_i), iout_max(_iout_max), out_max(_out_max), last_error(0.0f), error_sum(0.0f) {}
  float calc(float set, float fdb) {
    float error = set - fdb;
    // kp
    float kp_out = this->kp * error;
    // ki
    this->error_sum += error;
    this->error_sum = constrain(this->error_sum, -this->max_i, this->max_i);
    float ki_out = this->ki * this->error_sum;
    ki_out = constrain(ki_out, -this->iout_max, this->iout_max);
    // kd
    float kd_out = this->kd * (this->last_error - error);
    this->last_error = error;
    // sum
    float output = kp_out + ki_out + kd_out;
    output = constrain(output, -this->out_max, this->out_max);
    
    return output;
  }

private:
  // pid params
  float kp;
  float ki;
  float kd;
  float max_i;
  float iout_max;
  float out_max;
  // pid states
  float last_error;
  float error_sum;
};

////////////////////////////////////////////////////////MECANUM////////////////////////////////////////////////////////

class MecanumResolver {
public:
  MecanumResolver(float _a, float _b, float _r) : a(_a), b(_b), r(_r) {}
  
  void forwardResolve(float wheel0, float wheel1, float wheel2, float wheel3, float *vx, float *vy, float *az) {
    *vx = ((-wheel0 + wheel1 + wheel2 - wheel3) / 4) * this->r;
    *vy = ((-wheel0 - wheel1 + wheel2 + wheel3) / 4) * this->r;
    *az = (((-wheel0 - wheel1 - wheel2 - wheel3) / 4) * this->r * 1.5) / (this->a + this->b);
  }
  
  void inverseResolve(float vx, float vy, float az, float *wheel0, float *wheel1, float *wheel2, float *wheel3) {
    *wheel0 = (- vx - vy - az * (this->a + this->b)) / this->r;
    *wheel1 = (+ vx - vy - az * (this->a + this->b)) / this->r;
    *wheel2 = (+ vx + vy - az * (this->a + this->b)) / this->r;
    *wheel3 = (- vx + vy - az * (this->a + this->b)) / this->r;
  }
private:
  float a;
  float b;
  float r;
};

////////////////////////////////////////////////////////VAR////////////////////////////////////////////////////////

// encoders
Encoder<0, A0, A1> encoder0;
Encoder<1, A4, A5> encoder1;
Encoder<2, A2, A3> encoder2;
Encoder<3, 9, 10> encoder3;

// motors
Motor motors[4] = {{5, 8, 2496}, {6, 7, 2496}, {3, 4, 2496}, {11, 12, 2496}}; // ppr = 2496

// filter
LowPassFilter speed_filters[4] = {0.8f, 0.8f, 0.8f, 0.8f};
LowPassFilter odom_filters[4] = {0.5f, 0.5f, 0.5f, 0.5f};

// pids
#define KP 0.15f
#define KI 0.04f
#define KD 0.0f
#define MAX_I 1000.0f
#define IOUT_MAX 5.0f
#define OUT_MAX 10.0f
PID pids[4] = {{KP, KI, KD, MAX_I, IOUT_MAX, OUT_MAX}, {KP, KI, KD, MAX_I, IOUT_MAX, OUT_MAX}, {KP, KI, KD, MAX_I, IOUT_MAX, OUT_MAX}, {KP, KI, KD, MAX_I, IOUT_MAX, OUT_MAX}};

// v_set
float vx_set = 0.0f, vy_set = 0.0f, az_set = 0.0f;

// mecanum
MecanumResolver mecanumResolver(0.095f, 0.0975f, 0.0325f);

// odom
float odom_vx = 0.0f, odom_vy = 0.0f, odom_az = 0.0f;
float odom_x = 0.0f, odom_y = 0.0f, odom_theta = 0.0f;

////////////////////////////////////////////////////////CONTROL////////////////////////////////////////////////////////

void control_loop(){
  float motor_speed_set[4];
  float motor_speed_fdb[4];
  float motor_speed_out[4];
  float odom_speed_fdb[4];

  // get each motor speed set from chassis speed set
  mecanumResolver.inverseResolve(vx_set, vy_set, az_set, &motor_speed_set[0], &motor_speed_set[1], &motor_speed_set[2], &motor_speed_set[3]);

  // update motor and odom speed feedback
  for (int i = 0; i < 4; i++) {
    motors[i].updateEnc(encPulses[i]);
    motor_speed_fdb[i] = speed_filters[i].filter(motors[i].speed);
    odom_speed_fdb[i] = odom_filters[i].filter(motors[i].speed);
  }

  // scale pid out whthin [-1,1]
  float max_motor_speed_out = 0.0f;
  for (int i = 0; i < 4; i++) {
    motor_speed_out[i] = pids[i].calc(motor_speed_set[i], motor_speed_fdb[i]);
    max_motor_speed_out = max(max_motor_speed_out, abs(motor_speed_out[i]));
  }
  float motor_speed_scale = 1.0f;
  if (max_motor_speed_out > 1.0f) {
    motor_speed_scale = 1.0f / max_motor_speed_out;
  }

  // set motor output
  for (int i = 0; i < 4; i++) {
    motors[i].setOutput(motor_speed_out[i] * motor_speed_scale);
  }

  // get each odom speed feedback from each motor speed feedback
  mecanumResolver.forwardResolve(odom_speed_fdb[0], odom_speed_fdb[1], odom_speed_fdb[2], odom_speed_fdb[3], &odom_vx, &odom_vy, &odom_az);

  // integral odom position
  static unsigned long last_time = millis();
  float dt = (float)(millis() - last_time) / 1000.0f;
  odom_x += (cos(odom_theta) * odom_vx - sin(odom_theta) * odom_vy) * dt;
  odom_y += (sin(odom_theta) * odom_vx + cos(odom_theta) *odom_vy) * dt;
  odom_theta += odom_az * dt;
  last_time = millis();
}

void comm_loop() {
  // send odom velocity and position
  Serial.print(" vx ");
  Serial.print(odom_vx);
  Serial.print(" vy ");
  Serial.print(odom_vy);
  Serial.print(" az ");
  Serial.print(odom_az);
  Serial.print(" x ");
  Serial.print(odom_x);
  Serial.print(" y ");
  Serial.print(odom_y);
  Serial.print(" theta ");
  Serial.print(odom_theta);
  Serial.println();

  // receive command
  char inputBuffer[50];
  static unsigned long last_cmd_timestamp = 0;
  // drop data not begin with x y a
  while (Serial.available() && (Serial.peek() != 'x' && Serial.peek() != 'y' && Serial.peek() != 'a')) Serial.read();
  // a packet must longer than 7 byte
  while (Serial.available() >= 7) {
    int bytesRead = Serial.readBytesUntil('\n', inputBuffer, sizeof(inputBuffer));
    inputBuffer[bytesRead] = 0; // 确保字符串以null字符结尾
    if ((bytesRead == 6 && inputBuffer[1] == ' ' && inputBuffer[3] == '.') || (bytesRead == 7 && inputBuffer[1] == ' ' && inputBuffer[2] == '-' && inputBuffer[4] == '.')) {
      // 尝试手动分割字符串并解析浮点数
      char* token = strtok(inputBuffer, " "); // 使用空格作为分隔符
      if (token != NULL) {
        float set = atof(strtok(NULL, " "));
        if (token[0] == 'x') {
          vx_set = set;
          last_cmd_timestamp = millis();
        } else if (token[0] == 'y') {
          vy_set = set;
          last_cmd_timestamp = millis();
        } else if (token[0] == 'a') {
          az_set = set;
          last_cmd_timestamp = millis();
        }
      }
    }
  }
    
  // failsafe
  if(millis()- last_cmd_timestamp > 500) {
    vx_set = 0.0f;
    vy_set = 0.0f;
    az_set = 0.0f;
  }
}

////////////////////////////////////////////////////////MAIN////////////////////////////////////////////////////////

void setup() {
  Serial.begin(1000000);
  Serial.setTimeout(0);
  delay(1000);
}

void loop() {
  static unsigned long last_control_stamp = 0, last_comm_stamp = 0;
  // control 250hz communication 100hz
  const long control_interval = 4, comm_interval = 10;

  if (millis() - last_control_stamp >= control_interval) {
    control_loop();
    last_control_stamp = millis();
  }
  if (millis() - last_comm_stamp >= comm_interval) {
    comm_loop();
    last_comm_stamp = millis();
  }
}
