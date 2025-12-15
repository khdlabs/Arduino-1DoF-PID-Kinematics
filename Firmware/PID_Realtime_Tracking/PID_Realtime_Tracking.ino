// PROGRAM SMOOTH SYSTEM + X Y

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>

// Pin definisi
const int RPWM = 5;
const int LPWM = 6;
const int R_EN = 7;
const int L_EN = 8;
const int POT_AKTUAL_PIN = A0;
const int POT_TARGET_PIN = A1;

// Kalibrasi ADC dari potensiometer
const int potMin = 13;
const int potMid = 385;
const int potMax = 720;

// PID parameters
float Kp = 1.0;
float Ki = 0.005;
float Kd = 0.4;

// PID variables
float previousError = 0;
float integral = 0;
int deadzone = 10;

int posisiTarget = 0;

// Inisialisasi LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);

const float armLength = 22.0; // cm

void setup() {
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(R_EN, OUTPUT);
  pinMode(L_EN, OUTPUT);

  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);

  lcd.begin();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("---Kelompok 1---");
  delay(1000);
  lcd.clear();
}

void loop() {
  int potValue = analogRead(POT_AKTUAL_PIN);
  int potTargetValue = analogRead(POT_TARGET_PIN);

  // Pemetaan target ke nilai ADC motor
  posisiTarget = map(potTargetValue, 0, 1023, potMin, potMax);

  // Untuk nilai aktual
  int aktual = 0;
  if (potValue <= potMid) {
    aktual = map(potValue, potMin, potMid, 0, 90);
  } else {
    aktual = map(potValue, potMid, potMax, 90, 185);
  }
  aktual = constrain(aktual, 0, 180);

  // Hitung error berdasarkan ADC, bukan derajat
  int error = posisiTarget - potValue;

  // PID computations
  float P = Kp * error;
  integral += error;
  integral = constrain(integral, -100, 100);
  float I = Ki * integral;
  float D = Kd * (error - previousError);
  int output = P + I + D;

  // Adaptasi output
  int maxOutput = map(abs(error), 0, 100, 30, 70);
  output = constrain(output, -maxOutput, maxOutput);

  if (abs(error) < 10) {
    output = constrain(output, -50, 50);
  } else {
    output = constrain(output, -70, 70);
  }

  // Kontrol motor
  if (abs(error) <= deadzone) {
    analogWrite(RPWM, 0);
    analogWrite(LPWM, 0);
  } else if (output > 0) {
    analogWrite(RPWM, output);
    analogWrite(LPWM, 0);
  } else {
    analogWrite(RPWM, 0);
    analogWrite(LPWM, -output);
  }

  previousError = error;

  // Hitung derajat target sebagai float
  float targetDegreeSmooth;
  if (posisiTarget <= potMid) {
    targetDegreeSmooth = map(posisiTarget, potMin, potMid, 0, 90);
  } else {
    targetDegreeSmooth = map(posisiTarget, potMid, potMax, 90, 185);
  }
  targetDegreeSmooth = constrain(targetDegreeSmooth, 0, 180);

  // Konversi ke koordinat (x, y)
  float targetRadian = targetDegreeSmooth * (M_PI / 180.0);
  float x = armLength * cos(targetRadian);
  float y = armLength * sin(targetRadian);

  // Koreksi jika y sangat kecil, dibulatkan ke nol
  if (abs(y) < 0.01) {
    y = 0.0;
  }

  // Tampilkan di LCD
  lcd.setCursor(0, 0);
  lcd.print("T:");
  lcd.print((int)targetDegreeSmooth);
  lcd.print("\xDF ");
  lcd.print("A:");
  lcd.print(aktual);
  lcd.print("\xDF ");
  
  lcd.setCursor(0, 1);
  lcd.print("X:");
  lcd.print(x, 1);
  lcd.print(" Y:");
  lcd.print(y, 1);
}
