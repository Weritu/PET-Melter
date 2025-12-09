//  Código referente ao TCC Manufatura Aditiva: O Upcycling de Garrafas PET
// É responsável por controlar a temepratura do bloco de aquecimento via PID
// e um sistema de tração com um motor via PWM e Potenciometro
// exibir os dados em um display 16x2 para feedback visual
//  Autor: Wellington Andrade (2025).

#include "SmoothThermistor.h"
#include "HardwareSerial.h"
#include "PID_v1_bc.h"
#include "LiquidCrystal_I2C.h"

#define NTC_PIN A5
#define LED_PIN 2
#define HOT_CARTIDGE 6 
#define TEMP_ALVO 240.0 
#define POT_PIN A2
#define MOTOR_PIN 9

int pot_val = 0;
int pwm = 0;
float temp_filtrada = 0;
double Setpoint, Input, Output;
double Kp = 10.0, Ki = 0.05, Kd = 80.0;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

SmoothThermistor thermistor(NTC_PIN, ADC_SIZE_10_BIT, 100000, 1000, 3950, 25, 10);

LiquidCrystal_I2C lcd(0x27,20,4);

void setup() {
  Serial.begin(9600);
  
  pinMode(LED_PIN, OUTPUT);
  pinMode(HOT_CARTIDGE, OUTPUT);
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(POT_PIN, INPUT);

  temp_filtrada = thermistor.temperature();

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0);

  // Configurar PID
  Setpoint = TEMP_ALVO;
  myPID.SetSampleTime(300); 
  myPID.SetOutputLimits(0, 255); 
  myPID.SetMode(AUTOMATIC); 

}

void loop() {

  // Filtro para diminuição de ruidos na leitura da temperatura
  float alpha = 0.1;
  temp_filtrada = (alpha * thermistor.temperature()) + ((1.0 - alpha) * temp_filtrada);
  Input = temp_filtrada; // Input temp filtrada para o PID

  //Input = thermistor.temperature();

  myPID.Compute();

  analogWrite(HOT_CARTIDGE, Output);

  delay(200);

  Serial.println(Input);

  // Exibe dados no Display
  lcd.print("Temp. Alvo: ");
  lcd.print(Setpoint);
  lcd.setCursor(0,1);
  lcd.print("Temp. Atual: ");
  lcd.print(Input);
 
  delay(300);
}