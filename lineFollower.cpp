#include <stdio.h>
#include <stdlib.h>
#include <BluetoothSerial.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_task_wdt.h>
#include <ArduinoJson.h>
#include <ArduinoJson.hpp>
#include <QTRSensors.h>
#include "SPIFFS.h"

int total = 3000;               // comprimento do vetor de dados do encoder
int vet[3000] = {0};  //  1 = ligado

//////Prototipos//////////////////////////////
void EncoderRead( void * pvParameters );
void IRAM_ATTR EncoderDireito();
int GetPos();
void readSensors();
void Calibracao();
void Motores(int esq, int dir);
void MotorEsq(int value);
void MotorDir(int value);
void BT_Manager();
int PIDLambo(int pos, float Kp, float Kd);
TaskHandle_t Task1;
TaskHandle_t Task2;
//////////////////////////////////////////////

///// Configure Bluetooth start /////////////////////
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
BluetoothSerial SerialBT;
String  BT_Data = "0";
String ch="0";
///// Configure Bluetooth end /////////////////////


////// esc ///////////////////////////////////
const int servoPin = 5;
int dutyCycle = 0;
/* Setting PWM properties */
const int PWMFreq = 50;
const int PWMChannel = 3;
const int PWMResolution = 8;
/////////////////////////////////////////////

# define AIN1 18    // pin 1 do Motor Esquerdo
# define AIN2 19    // pin 2 do Motor Esquerdo
# define PWMA 23    // pin PWM do Motor Esquerdo

# define BIN1 21    // pin 1 do Motor Direito
# define BIN2 4    // pin 2 do Motor Direito
# define PWMB 22    // pin PWM do Motor Direito

# define EncoderPin 23
# define PINLED 2


int v_s_min[8] = { 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023 };
int v_s_max[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
int erros[8] = {-5.0, -2.5, -1.2, -0.2, 0.2, 1.2, 2.5, 5.0};
volatile int s_p[8];
boolean online;

int pos;
int l_pos;

bool run = false;
bool runing = false;

int base = 512;
float Kprop = 2.8;  //1.2
float Kderiv = 10.0; //7.5
int setpoint = 0;
int last_error = 0;

volatile int pulsoD = 0; // encoder Direito

void setup() {
  Serial.begin(115200);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  pinMode(PINLED, OUTPUT);


  /////////////// duty cycle MOTORES com 10bits (2^10 = 1024) de resolução, 50% devemos escrever 512. max de 1024 ou 1025?////////////////////////////
  ledcAttachPin(PWMA, 0); //Atribuimos o pino esqPWM ao canal 0.
  ledcSetup(0, 1000, 10);   ////Atribuimos ao canal 0 a frequencia de 1000Hz com resolucao de 10bits.
  ledcAttachPin(PWMB, 1); //Atribuimos o pino dirPWM ao canal 1.
  ledcSetup(1, 1000, 10);   ////Atribuimos ao canal 1 a frequencia de 1000Hz com resolucao de 10bits.
  ////////////////////////////////////////////////////////////////////////////

  ///////////// esc //////////////////////////////////////
  
  /* Attach the LED PWM Channel to the GPIO Pin */
  ledcAttachPin(servoPin, PWMChannel);  
  ledcSetup(PWMChannel, PWMFreq, PWMResolution);
  ///////////////////////////////////////////////////////

  //create a task that will be executed in the EncoderRead() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
                    EncoderRead,   /* Task function. */
                    "Task1",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task1,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */                  
  delay(500); 

  //create a task that will be executed in the PidController() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
                    PidController,   /* Task function. */
                    "Task2",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task2,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */
    delay(500); 
    ledcWrite(PWMChannel, 120);
}

void IRAM_ATTR EncoderDireito(){
  pulsoD++;    
}

void EncoderRead( void * pvParameters ){
  attachInterrupt(EncoderPin, EncoderDireito, RISING);
  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());
  for(int i = 0; i < total; i++){
    vet[i] = 0;
  }
  Serial.print("Delay ...");
  delay(10000);
  esp_task_wdt_init(3000, false);
  Serial.print("Go encoder ...");

  for(;;){
    //digitalWrite(PINLED, vet[pulsoD]);
    delay(500);
    /*
    int inCurva[50]
    if(pulso > incurva[0] && pulso < inCurva[1]){
      liga turbina
    }
    */
  } 
}

void PidController( void * pvParameters ){
  Serial.print("Task2 running on core ");
  Serial.println(xPortGetCoreID());
  esp_task_wdt_init(3000, false);
   //////////////////////// setup bluetooth start ////////////////////////
  SerialBT.begin("ESP32test"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
  //////////////////////// setup bluetooth start ////////////////////////

  digitalWrite(PINLED, HIGH);
  Motores(0, 0);
  Calibracao();
  digitalWrite(PINLED, LOW);
  delay(2000);
  Serial.print("Go Pid...");
  for(;;){
      BT_Manager();
    if(!run && !runing){
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, LOW);
      digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, LOW);
    }
    while(run){
      
      runing = true;
      BT_Manager();

      Motores(base  , base); 
      /*
      int line_position = GetPos();
      int Correction_power = PIDLambo(line_position, Kprop, Kderiv);
          
      if(Correction_power >= 0){
        Motores(base , base - Correction_power);  
      }else{
        Motores(base  + Correction_power, base);  
      }  
      */
    }
   
   while(runing){ // Parar de forma regrtessiva da velocidade
    for(int i = base; i > 0; i--){
      int line_position = GetPos();
        int Correction_power = PIDLambo(line_position, Kprop, Kderiv);
            
        if(Correction_power >= 0){
          Motores(i , i - Correction_power);  
        }else{
          Motores(i  + Correction_power, i);  
        }  
        //delay(3);
    }
      runing = false;
   }  
  }
}

void loop() {
  }

int PIDLambo(int pos, float Kp, float Kd) {

  int error = pos - setpoint;
  int derivative = error - last_error;
  last_error = error;
  int pot_giro = (error * Kp + derivative * Kd);


    
  return pot_giro;
}

void BT_Manager(){
 
  if (SerialBT.available() > 0) {
    // recebe dados do Bluetooth
    BT_Data = SerialBT.readStringUntil('\n');
    BT_Data.trim(); // Remove caracteres em branco no início e no fim
    Serial.println(BT_Data);

    
    
    // liga ou desliga os motores
    if (BT_Data == "OnOff"){ 
      if(run){
        run = false;
      }else{
        run = true;
        //base = pwm;
      }
    }

    if (BT_Data == "TurbinaNone"){ 
      Serial.println("NONE");
      ledcWrite(PWMChannel, 120);
    }
    if (BT_Data == "Turbina1"){ 
      Serial.println("T1");
      ledcWrite(PWMChannel, 122);
    }
    if (BT_Data == "Turbina2"){ 
      Serial.println("T2");
      ledcWrite(PWMChannel, 123);
    }

    // envia dados do robo para o celular
    if (BT_Data == "GetStat"){
      SerialBT.print((String)"D" + "," + (String)Kprop + "," + (String)Kderiv + "," + (String)Kderiv + "," + (String)base + "," + (String)erros[0] + "," + (String)erros[1] + "," + (String)erros[2] + "," + (String)erros[3]);
      Serial.print(esp_get_free_heap_size());
      Serial.println(" : is free heap");
    }

    // BT_Data para json e tratamento dos dados recebidos
    if (BT_Data.startsWith("{")) { 
      
      ch.concat(BT_Data);
      Serial.print ("Received:");//print on serial monitor
      StaticJsonDocument <800> doc;

      const char * json = BT_Data.c_str();

      DeserializationError error = deserializeJson(doc, json);

      if (error) {
            Serial.print(F("deserializeJson() failed: "));
            Serial.println(error.f_str());
            return;
      }

      // recebe novos valores para PID
      if(BT_Data[2] == 'P'){
        Kprop = atof(doc["P"]);
        //Ki = atof(doc["I"]);
        Kderiv = atof(doc["D"]);
      }

      // recebe novos valores de erro
      if(BT_Data[2] == '0'){
        erros[0] = atof(doc["0"]);
        erros[1] = atof(doc["1"]);
        erros[2] = atof(doc["2"]);
        erros[3] = atof(doc["3"]);
        erros[4] = erros[3] * (-1);
        erros[5] = erros[2] * (-1);
        erros[6] = erros[1] * (-1);
        erros[7] = erros[0] * (-1);
        Serial.print ("Erros recebidos");//print on serial monitor
      }

      // recebe nova velocidade maxima
      if(BT_Data[2] == 'S'){
        base = atof(doc["Speed"]);
        //base = pwm;
      }      
    }
  }  
}

void MotorEsq(int value) {
  if (value >= 0) {
    // si valor positivo vamos hacia adelante

    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else {
    // si valor negativo vamos hacia atras
    if(value < -1023){
      value = -1023;
    }
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    value *= -1;
  }

  // Setea Velocidad

  //analogWrite(PWMA, value);
  //ledcWrite(canal, valor);
  ledcWrite(0, value);
  //Serial.println("Esq");
}


void MotorDir(int value) {
  if (value >= 0) {
    // si valor positivo vamos hacia adelante

    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  } else {
    // si valor negativo vamos hacia atras
    if(value < -1023){
      value = -1023;
    }
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    value *= -1;
  }

  // Seta Velocidade
  //analogWrite(PWMB, value);
  //ledcWrite(canal, valor);
  ledcWrite(1, value);
  //Serial.println("Dir");
}


void Motores(int esq, int dir) {  
  MotorEsq(esq);
  MotorDir(dir);
}


void Calibracao() {

  int v_s[8];

  for (int j = 0; j < 200; j++) {
    delay(10);
    
    v_s[0] = analogRead(26);
    v_s[1] = analogRead(25);
    v_s[2] = analogRead(33);
    v_s[3] = analogRead(32);
    v_s[4] = analogRead(35);
    v_s[5] = analogRead(34);
    v_s[6] = analogRead(39);
    v_s[7] = analogRead(36);


    for (int i = 0; i < 8; i++) {
    }
    for (int i = 0; i < 8; i++) {
      if (v_s[i] < v_s_min[i]) {
        v_s_min[i] = v_s[i];
      }
    }


    for (int i = 0; i < 8; i++) {
      if (v_s[i] > v_s_max[i]) {
        v_s_max[i] = v_s[i];
      }
    }
  }
}

void readSensors() {

  volatile int s[8];

  s[0] = analogRead(26);
  s[1] = analogRead(25);
  s[2] = analogRead(33);
  s[3] = analogRead(32);
  s[4] = analogRead(35);
  s[5] = analogRead(34);
  s[6] = analogRead(39);
  s[7] = analogRead(36);

  for (int i = 0; i < 8; i++) {
    if (s[i] < v_s_min[i]) {
      s[i] = v_s_min[i];
    }

    if (s[i] > v_s_max[i]) {
      s[i] = v_s_max[i];
    }
    s_p[i] = map(s[i], v_s_min[i], v_s_max[i], 100, 0);
  }


  volatile int sum = s_p[0] + s_p[1] + s_p[2] + s_p[3] + s_p[4] + s_p[5] + s_p[6] + s_p[7];
  if (sum > 100) {
    online = 1;

  } else {
    online = 0;
    sum = 100;
  }
}

int GetPos() {
  readSensors();
  int prom = -6.0 * s_p[0] -3.0 * s_p[1] - 1.2 * s_p[2] - 0.3 * s_p[3] + 0.3 * s_p[4] + 1.2 * s_p[5] + 3.0 * s_p[6] + 6.0 * s_p[7];
  //int prom = -7.5 * s_p[0] -3.7 * s_p[1] - 1.8 * s_p[2] - 0.3 * s_p[3] + 0.3 * s_p[4] + 1.8 * s_p[5] + 3.7 * s_p[6] + 7.5 * s_p[7];
  //int prom = -erros[0] * s_p[0] -erros[1] * s_p[1] - erros[2] * s_p[2] - erros[3] * s_p[3] + erros[4] * s_p[4] + erros[5] * s_p[5] + erros[6] * s_p[6] + erros[7] * s_p[7];
  int sum = s_p[0] + s_p[1] + s_p[2] + s_p[3] + s_p[4] + s_p[5] + s_p[6] + s_p[7];

  if (online) {
    pos = int(100.0 * prom / sum);
  } else {
    if (l_pos < 0) {
      pos = -1023;
    }
    if (l_pos >= 0) {
      pos = 1023;
    }
  }
  l_pos = pos;  
  return pos;
}