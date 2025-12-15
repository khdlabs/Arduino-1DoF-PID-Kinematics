//PROGRAM GEDEK GEDEK + X Y

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Pin definisi
const int RPWM = 5;
const int LPWM = 6;
const int R_EN = 7;
const int L_EN = 8;
const int POT_AKTUAL_PIN = A0;
const int POT_TARGET_PIN = A1;

// Kalibrasi ADC dari potensiometer
const int potMin = 15;
const int potMax = 713;
const int potMid = 392;

// PID parameters
float Kp = 1.0;   // Responsif untuk P
float Ki = 0.01;  // Perbaikan I untuk stabilitas
float Kd = 0.4;   // D lebih lembut untuk mengurangi osilasi

// PID variables
float previousError = 0;
float integral = 0;
int deadzone = 10;

int posisiAwal = potMin;  // Posisi awal (tengah)
int posisiTarget = 0;

// Waktu stabilisasi
unsigned long waktuStabilMulai = 0;
bool stabil = false;
bool menujuAwal = true; // Status menuju awal atau target

// Inisialisasi LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);

const float armLength = 22.0; // Panjang lengan dalam cm

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

  // Pemetaan target berdasarkan status
  if (menujuAwal) {
    posisiTarget = posisiAwal;
  } else {
    posisiTarget = map(potTargetValue, 0, 1023, potMin, potMax);
  }

  // Pemetaan nilai aktual ke derajat
  int aktual = 0;
  if (potValue <= potMid) {
    aktual = map(potValue, potMin, potMid, 0, 90);
  } else {
    aktual = map(potValue, potMid, potMax, 90, 180);
  }
  aktual = constrain(aktual, 0, 180);

  // Hitung error
  int error = posisiTarget - potValue;

  // PID computations
  float P = Kp * error;
  integral += error;
  integral = constrain(integral, -100, 100); // Anti wind-up
  float I = Ki * integral;
  float D = Kd * (error - previousError);
  int output = P + I + D;

  // Kurangi kecepatan saat mendekati target
  if (abs(error) < 10) {
    output = constrain(output, -50, 50);
  } else {
    output = constrain(output, -70, 70);
  }

  // Atur motor
  if (abs(error) <= deadzone) {
    analogWrite(RPWM, 0);
    analogWrite(LPWM, 0);

    // Stabilisasi
    if (!stabil) {
      stabil = true;
      waktuStabilMulai = millis();
    } else if (millis() - waktuStabilMulai >= 2000) {
      // Setelah stabil selama 2 detik, ubah status target
      stabil = false;
      menujuAwal = !menujuAwal;
    }
  } else {
    stabil = false;
    if (output > 0) {
      analogWrite(RPWM, output);
      analogWrite(LPWM, 0);
    } else {
      analogWrite(RPWM, 0);
      analogWrite(LPWM, -output);
    }
  }

  previousError = error;

  // Pemetaan nilai target ke derajat
  int targetDegree = 0;
  if (posisiTarget <= potMid) {
    targetDegree = map(posisiTarget, potMin, potMid, 0, 90);
  } else {
    targetDegree = map(posisiTarget, potMid, potMax, 90, 180);
  }
  targetDegree = constrain(targetDegree, 0, 180);

  // Tampilkan data di LCD
  lcd.setCursor(0, 0);
  lcd.print("T:");
  lcd.print(targetDegree);
  lcd.print("\xDF ");
  lcd.print(" A:");
  lcd.print(aktual);
  lcd.print("\xDF");
  lcd.print("   ");

  float targetRadian = targetDegree * (M_PI / 180.0); // Konversi derajat ke radian
  float x = armLength * cos(targetRadian);
  float y = armLength * sin(targetRadian);

  lcd.setCursor(0, 1);
  lcd.print("X:");
  lcd.print(x, 1); // 1 angka desimal
  lcd.print(" Y:");
  lcd.print(y, 1); // 1 angka desimal
}