#include <Servo.h>  // 默认占用 Timer1（影响 D9、D10）

#include "Adafruit_TCS34725.h"
Servo myServo;
int servoDeg = -1;

const int motorPin = 5;  // 电机 D5（Timer0）PWM 控制电压
const int servoPin = 9;  // 舵机 D9 (Timer1)

/* Example code for the Adafruit  breakout library */

/* 
  TCS34725:
   Connect SCL    to analog 5
   Connect SDA    to analog 4
   Connect VDD    to 3.3V DC
   Connect GROUND to common ground */

// 初始化传感器对象（推荐参数：积分时间2.4ms，增益4x）
Adafruit_TCS34725 tcs =
    Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_4X);

enum Color {
  RED = 0,
  ORANGE = 1,
  YELLOW = 2,
  GREEN = 3,
  BLUE = 4,
  BROWN = 5,
  UNKNOWN = 6
};

Color lastGetColor = UNKNOWN;
unsigned long lastGetColorTime = 0;
unsigned long lastReadTime = 0;
unsigned long T = 852;
const uint16_t readInterval = 24 + 5;  // 24ms积分 + 5ms缓冲

const int SampleCount = 8;
int sampleIndex = 0;
uint8_t sampleR[SampleCount];
uint8_t sampleG[SampleCount];
uint8_t sampleB[SampleCount];

void bubbleSort(float arr[], int n) {
  for (int i = 0; i < n - 1; i++) {
    for (int j = 0; j < n - i - 1; j++) {
      if (arr[j] > arr[j + 1]) {
        int temp = arr[j];
        arr[j] = arr[j + 1];
        arr[j + 1] = temp;
      }
    }
  }
}

bool between(float v, float start, float end) {
  if (start > end) {
    float t = start;
    start = end;
    end = t;
  }
  return v <= end && v >= start;
}

Color detectColor(uint16_t r, uint16_t g, uint16_t b) {
  if (between(r, 100, 106) && between(g, 96, 92) && between(b, 65, 67)) {  // 110 89 62 橙色 - 107 91 65
    return RED;
  } else if (between(r, 103, 120) && between(g, 85, 93) && between(b, 57, 64)) { 
    return ORANGE; // between(r, 105, 120) && between(g, 85, 93) && between(b, 57, 63)
  } else if (between(r, 100, 107) && between(g, 96, 98) && between(b, 55, 61)) {
    return YELLOW;
  } else if (between(r, 85, 94) && between(g, 104, 110) && between(b, 60, 73)) {
    return GREEN;
  } else if (between(r, 80, 85) && between(g, 101, 104) && between(b, 76, 81)) {
    return BLUE;
    // } else if (between(r, 85, 87) && between(g, 101, 102) && between(b, 70,
    // 71)) {  // 85 102 71 - 86 101 70 - 87 102 70 - 87 102 71
    //   return BROWN;
  } else if ((r == 85 && g == 102 && b == 71) ||
             (r == 86 && g == 101 && b == 70) ||
             (r == 87 && g == 102 && b == 70) ||
             (r == 87 && g == 102 && b == 71)) {
    return BROWN;
  }
  return UNKNOWN;
}

void controlBeanWithColorH(Color h) {
  int tServoDeg = 0;

  switch (h) {
    case RED:
      tServoDeg = 0;
      break;
    case ORANGE:
      tServoDeg = 30;
      break;
    case YELLOW:
      tServoDeg = 60;
      break;
    case GREEN:
      tServoDeg = 120;
      break;
    case BLUE:
      tServoDeg = 150;
      break;
    default:
      tServoDeg = 90;
      break;
  }

  if (servoDeg == tServoDeg) {
    return;
  }

  servoDeg = tServoDeg;

  // Serial.print(" H:");
  // Serial.print(h);
  Serial.print(" D:");
  Serial.print(servoDeg);
  Serial.println();
  myServo.write(servoDeg);
}

void RGBtoHSV(float r, float g, float b, float *h, float *s, float *v) {
  r /= 255.0;
  g /= 255.0;
  b /= 255.0;

  float maxVal = max(max(r, g), b);
  float minVal = min(min(r, g), b);
  float delta = maxVal - minVal;

  // 计算色调(H)
  if (delta == 0) {
    *h = 0;
  } else if (maxVal == r) {
    *h = 60 * fmod(((g - b) / delta), 6);
  } else if (maxVal == g) {
    *h = 60 * (((b - r) / delta) + 2);
  } else if (maxVal == b) {
    *h = 60 * (((r - g) / delta) + 4);
  }

  if (*h < 0) {
    *h += 360;
  }

  // 计算饱和度(S)
  if (maxVal == 0) {
    *s = 0;
  } else {
    *s = (delta / maxVal) * 100;
  }

  // 计算明度(V)
  *v = maxVal * 100;
}

int sum(uint8_t sample[], int count) {
  int sum = 0;
  for (size_t i = 0; i < count; i++) {
    sum += sample[i];
  }
  return sum;
}

void readRGB() {
  uint16_t r, g, b, c;

  // 读取原始RGBC值（16位整数）
  tcs.getRawData(&r, &g, &b, &c);

  // 避免除以0的错误（当C值为0时返回黑色）
  if (c == 0) {
    Serial.println("Error: C == 0");
    return;
  }

  // 归一化到0-255范围
  float scale = 255.0 / c;
  uint8_t red = r * scale;
  uint8_t green = g * scale;
  uint8_t blue = b * scale;

  // 限制数值范围（防止意外溢出）
  red = constrain(red, 0, 255);
  green = constrain(green, 0, 255);
  blue = constrain(blue, 0, 255);

  sampleR[sampleIndex] = red;
  sampleG[sampleIndex] = green;
  sampleB[sampleIndex] = blue;
  sampleIndex++;
  if (sampleIndex < SampleCount) {
    return;
  }

  sampleIndex = 0;

  uint8_t avgR = sum(sampleR, SampleCount) / SampleCount;
  uint8_t avgG = sum(sampleG, SampleCount) / SampleCount;
  uint8_t avgB = sum(sampleB, SampleCount) / SampleCount;

  red = avgR;
  green = avgG;
  blue = avgB;

  Color color = detectColor(red, green, blue);
  Serial.print(" Color:");
  Serial.print(color);
  // Serial.print(" L1:");
  // Serial.print(-1);

  // Serial.print(" L2:");
  // Serial.print(7);

  // 转换为HSV
  // float h, s, v;
  // RGBtoHSV(red, green, blue, &h, &s, &v);

  Serial.print(" R:");
  Serial.print(red);
  Serial.print(" G:");
  Serial.print(green);
  Serial.print(" B:");
  Serial.print(blue);

  // Serial.print(" H:");
  // Serial.print(h);
  // Serial.print(" S:");
  // Serial.print(s);
  // Serial.print(" V:");
  // Serial.print(v);

  Serial.print(" RL1:");
  Serial.print(0);

  Serial.print(" RL2:");
  Serial.print(255);

  Serial.println();

  /*
   * 检测到正常色，记录时间和颜色，并在T/2ms后转动舵机到指定位置
   * 如果检测到非正常色，只有当前时间减去记录的时间大于T*1.5秒（确保前面的正常色已经落下）才转动到默认位置
   */
  unsigned long now = millis();
  switch (color) {
    case BROWN:
    case UNKNOWN: {
      if (now - lastGetColorTime > T*1.1) {
        controlBeanWithColorH(color);
      }
      break;
    }
    default: {
      lastGetColorTime = now;
      lastGetColor = color;
      break;
    }
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(motorPin, OUTPUT);
  analogWrite(motorPin, 128);  // 1.5V 等效 PWM（假设电源 3.7V）

  myServo.attach(servoPin);

  if (!tcs.begin()) {
    Serial.println("could not find TCS34725");
    while (1);  // 停止程序
  }
}

// TCS34725 有一个 VALID 状态位（寄存器 0x13 的 bit
// 0），表示数据已更新。可通过检查该位避免无效读取：
bool isDataReady() {
  uint8_t status = tcs.read8(TCS34725_STATUS);
  return (status & 0x01);  // VALID位为1表示数据就绪
}

void loop() {
  unsigned long now = millis();

  if (isDataReady() && now - lastReadTime >= readInterval) {
    lastReadTime = now;
    readRGB();
  }
  if (lastGetColor != UNKNOWN && now - lastGetColorTime > T * 0.75) {
    controlBeanWithColorH(lastGetColor);
    lastGetColor = UNKNOWN;
  }
}
