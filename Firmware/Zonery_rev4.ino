#include <PS4Controller.h>
#include <Arduino.h>
#include <driver/ledc.h>
#include <Ticker.h>
#include <iostream>

//Limitação do PWM (0 a 255)
int maxPWM = 170;

//Pinout dos canais PWM (Motores)
  //Motor 1 (Dianteira Esquerda)
  const int pinPWM_D_E_F = 13;
  const int pinPWM_D_E_R = 12;

  //Motor 2 (Dianteira Direita)
  const int pinPWM_D_D_F = 18;
  const int pinPWM_D_D_R = 19;

  //Motor 3 (Traseira Esquerda)
  const int pinPWM_T_E_F = 16;
  const int pinPWM_T_E_R = 17;

  //Motor 4 (Traseira Direita)
  const int pinPWM_T_D_F = 25;
  const int pinPWM_T_D_R = 26;
//

//INICIALIZA RGB LED controle
uint8_t setLed_R = 0;
uint8_t setLed_G = 0;
uint8_t setLed_B = 254;

// Leitura da tensão da bateria
const int batteryPin = 4; // Pino de entrada analógica
const float maxVoltage = 3.0; // Tensão máxima de referência (volts)
Ticker batteryTicker;

//Variaveis dos comandos do Controle PS4
unsigned long lastTimeStamp = 0;
int AnalogLY = 0;
int AnalogRX = 0;
bool nullAnalogLY = false;
bool nullAnalogRX = false;
bool nullDualAnalog = false;
bool nullGiro = false;
int AnalogMin = 10;
int AnalogMax = 127;
int L2;
int R2;
int R1;

//Variaveis auxiliares para controle de direção
int limit;
int invertedPWM;

//NotificaçãO dos comandos do Controle PS4 no serial Monitor
void notify() {
  char messageString[200];
  sprintf(messageString, "%4d,%4d,%4d,%4d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d",
  PS4.LStickX(),
  PS4.LStickY(),
  PS4.RStickX(),
  PS4.RStickY(),
  PS4.Left(),
  PS4.Down(),
  PS4.Right(),
  PS4.Up(),
  PS4.Square(),
  PS4.Cross(),
  PS4.Circle(),
  PS4.Triangle(),
  PS4.L1(),
  PS4.R1(),
  PS4.L2(),
  PS4.R2(),  
  PS4.Share(),
  PS4.Options(),
  PS4.PSButton(),
  PS4.Touchpad(),
  PS4.Charging(),
  PS4.Audio(),
  PS4.Mic(),
  PS4.Battery());

  //Tirar o comentario para ver na os comandos dos controles no Serial Monitor
 if (millis() - lastTimeStamp > 50) {
    lastTimeStamp = millis();
    
    Serial.print("LY: ");
    Serial.println(AnalogLY);

    Serial.print("RX: ");
    Serial.println(AnalogRX);

    Serial.print("L2: ");
    Serial.println(L2);

    Serial.print("R2: ");
    Serial.println(R2);

    Serial.print("R1: ");
    Serial.println(R1);
  }
}

//Status de conexão do Controle PS4
void onConnect() {
  Serial.println("Controle Conectado!.");
}

void onDisConnect() {
  Serial.println("Controle Desconectado!.");
  setPWM(0, 0, 0, 0, 0, 0);
  setPWM(1, 0, 0, 0, 0, 0);
  setPWM(2, 0, 0, 0, 0, 0);
  setPWM(3, 0, 0, 0, 0, 0);
  setPWM(4, 0, 0, 0, 0, 0);
  setPWM(5, 0, 0, 0, 0, 0);
  setPWM(6, 0, 0, 0, 0, 0);
  setPWM(7, 0, 0, 0, 0, 0);
}

//Função de inicialização de canais PWM
void setupPWM(uint8_t pinPWM, uint8_t canalPWM) {
  ledcSetup(canalPWM, 5000, 8);
  ledcAttachPin(pinPWM, canalPWM);
}

//Função de controle PWM
void setPWM(uint8_t canalPWM, int setpoint, int inputMin, int inputMax, int outputMin, int outputMax) {
  int pwmValue = map(setpoint, inputMin, inputMax, outputMin, outputMax);
  pwmValue = constrain(pwmValue, outputMin, outputMax);
  ledcWrite(canalPWM, pwmValue);
}

//Função para leitura da tensão da bateria do Robô Zonery
void readBatteryVoltage() {
  // Leitura da tensão da bateria
  long int rawValueBattery = analogRead(batteryPin);
  float tensaoMV = 0.7326*rawValueBattery;
  float tensaoBattery = scaller(tensaoMV, 0.0, 2820.0, 0.0, 12.0);

  // Converte o valor lido para uma escala de 0 a 100%
  float percentageBattery = (tensaoBattery / 12.0) * 100.0;
  // Imprime o valor na porta serial
  Serial.print("Valor AD: ");
  Serial.print(rawValueBattery);
  Serial.print(" Tensão AD: ");
  Serial.print(tensaoMV);
  Serial.print("mV");
  Serial.print(" Valor de Tensão bateria 12V: ");
  Serial.print(tensaoBattery);
  Serial.print("V");
  Serial.print(" (");
  Serial.print(percentageBattery, 2);
  Serial.println("%)");
  //envia as cores para o controle de acordo com o valor de tensão da bateria
  if (tensaoBattery >= 11.5 && rawValueBattery < 4094) {
    // Verde
    setLed_R = 0;
    setLed_G = 255;
    setLed_B = 0;
  } else if (tensaoBattery >= 10.0 && tensaoBattery < 11.5 && rawValueBattery < 4094) {
    // Amarelo
    setLed_R = 255;
    setLed_G = 255;
    setLed_B = 0;
  } else if (tensaoBattery >=8.5  && tensaoBattery < 10.0 && rawValueBattery < 4094) {
    // Laranja
    setLed_R = 255;
    setLed_G = 100;
    setLed_B = 0;
  } else if (tensaoBattery < 8.5 && rawValueBattery < 4094)  {
    // Vermelho
    setLed_R = 255;
    setLed_G = 0;
    setLed_B = 0;
  }

  // Azul (Medição desconectada)
  if (rawValueBattery < 50 || rawValueBattery >= 4094){
    // AZUL
    setLed_R = 0;
    setLed_G = 0;
    setLed_B = 255;
  }

  PS4.setLed(setLed_R, setLed_G, setLed_B);
  PS4.sendToController();
}

// Função escalonamento direto (float)
float scaller(float valor, float minimoEntrada, float maximoEntrada, float minimoSaida, float maximoSaida) {
  // Primeiro, verifique se o valor está dentro do intervalo de entrada
  if (valor < minimoEntrada) {
    valor = minimoEntrada;
  } else if (valor > maximoEntrada) {
    valor = maximoEntrada;
  }
  
  // Calcule a escala (fator de conversão)
  float escala = (maximoSaida - minimoSaida) / (maximoEntrada - minimoEntrada);
  
  // Aplique a escala e deslocamento
  float valorEscalonado = (valor - minimoEntrada) * escala + minimoSaida;
  
  return valorEscalonado;
}


//Função de escalonamento inverso (Controle de direção)
int inverted_analog_input_scaler(int input_value, int min_input, int max_input, int scaled_min, int scaled_max) {
  // Garantir que o valor de entrada esteja dentro do intervalo original
  input_value = max(min(input_value, max_input), min_input);
  
  // Calcular a escala invertida para o valor de entrada
  int input_range = max_input - min_input;
  int scaled_range = scaled_max - scaled_min;
  
  int inverted_scaled_value = ((max_input - input_value) / (float) input_range) * scaled_range + scaled_min;
  return inverted_scaled_value;
}

void setup() {
  
  //Inicializa serial
  Serial.begin(115200);

  //Inicializa Controle PS4
  PS4.attach(notify);
  PS4.attachOnConnect(onConnect);
  PS4.attachOnDisconnect(onDisConnect);
  PS4.begin();
  PS4.setLed(setLed_R, setLed_G, setLed_B);
  PS4.sendToController();
  Serial.println("Ready.");
  
  //Inicializa canais PWM
    //Motor 1
    setupPWM(pinPWM_D_E_F, 0);
    setupPWM(pinPWM_D_E_R, 1);

    //Motor 2
    setupPWM(pinPWM_D_D_F, 2);
    setupPWM(pinPWM_D_D_R, 3);

    //Motor 3
    setupPWM(pinPWM_T_E_F, 4);
    setupPWM(pinPWM_T_E_R, 5);

    //Motor 4
    setupPWM(pinPWM_T_D_F, 6);
    setupPWM(pinPWM_T_D_R, 7);
  //  

  // Chama a função readBatteryVoltage a cada 10 segundos
  batteryTicker.attach(10, readBatteryVoltage); 

}

void loop() {

  //Atribui comandos do controle para as variaveis
  AnalogLY = PS4.LStickY();
  AnalogRX = PS4.RStickX();
  L2 = PS4.L2();
  R2 = PS4.R2();
  R1 = PS4.R1();

  //Identifica posições nulas nos Analógicos do controle (centro)
  if (AnalogLY >= -AnalogMin && AnalogLY <= AnalogMin){
    nullAnalogLY = true;
  } else {nullAnalogLY = false;}

  if (AnalogRX >= -AnalogMin && AnalogRX <= AnalogMin){
    nullAnalogRX = true;
  } else {nullAnalogRX = false;}


  //Controles de direção e aceleração
  
    //4 Rodas para Frente (Analógico L para frente)
    if (AnalogLY > AnalogMin && nullAnalogRX == true && R2 == 0 && L2 == 0 ) {
      
      //Motor 1
      setPWM(0, AnalogLY, AnalogMin, AnalogMax, 0, maxPWM);
      setPWM(1, 0, 0, 0, 0, 0);

      //Motor 3
      setPWM(4, AnalogLY, AnalogMin, AnalogMax, 0, maxPWM);
      setPWM(5, 0, 0, 0, 0, 0);

      //Motor 2
      setPWM(2, AnalogLY, AnalogMin, AnalogMax, 0, maxPWM);
      setPWM(3, 0, 0, 0, 0, 0);

      //Motor 4
      setPWM(6, AnalogLY, AnalogMin, AnalogMax, 0, maxPWM);
      setPWM(7, 0, 0, 0, 0, 0);
    }

    //4 Rodas para trás (Analógico L para trás)
    if (AnalogLY < -AnalogMin && nullAnalogRX == true && R2 == 0 && L2 == 0){

      //Motor 1
      setPWM(0, 0, 0, 0, 0, 0);
      setPWM(1, -AnalogLY, AnalogMin, AnalogMax, 0, maxPWM);

      //Motor 3
      setPWM(4, 0, 0, 0, 0, 0);
      setPWM(5, -AnalogLY, AnalogMin, AnalogMax, 0, maxPWM);

      //Motor 2
      setPWM(2, 0, 0, 0, 0, 0);
      setPWM(3, -AnalogLY, AnalogMin, AnalogMax, 0, maxPWM);

      //Motor 4
      setPWM(6, 0, 0, 0, 0, 0);
      setPWM(7, -AnalogLY, AnalogMin, AnalogMax, 0, maxPWM);
    }

    //Vira pra Direita (Analógico R para Direita) - FRENTE
    if (AnalogRX > AnalogMin && AnalogLY > AnalogMin && R2 == 0 && L2 == 0){
    
      //Motor 1
      setPWM(0, AnalogLY, AnalogMin, AnalogMax, 0, maxPWM);
      setPWM(1, 0, 0, 0, 0, 0);

      //Motor 3
      setPWM(4, AnalogLY, AnalogMin, AnalogMax, 0, maxPWM);
      setPWM(5, 0, 0, 0, 0, 0);

      //Inverte PWM
      limit = map(AnalogLY, AnalogMin, AnalogMax, 0, maxPWM);
      invertedPWM = inverted_analog_input_scaler(AnalogRX, AnalogMin, AnalogMax, AnalogMin, AnalogMax);
      Serial.print("invertedPWM: ");
      Serial.println(invertedPWM);
      
      //Motor 2
      setPWM(2, invertedPWM, AnalogMin, AnalogMax, 0, limit);
      setPWM(3, 0, 0, 0, 0, 0);

      //Motor 4
      setPWM(6, invertedPWM, AnalogMin, AnalogMax, 0, limit);
      setPWM(7, 0, 0, 0, 0, 0);
    }

    //Vira pra Esquerda (Analógico R para esquerda) - FRENTE
    if (AnalogRX < -AnalogMin && AnalogLY > AnalogMin && R2 == 0 && L2 == 0){

      //Inverte PWM
      limit = map(AnalogLY, AnalogMin, AnalogMax, 0, maxPWM);
      invertedPWM = inverted_analog_input_scaler(-AnalogRX, AnalogMin, AnalogMax, AnalogMin, AnalogMax);
      Serial.print("invertedPWM: ");
      Serial.println(invertedPWM);
      
      //Motor 1
      setPWM(0, invertedPWM, AnalogMin, AnalogMax, 0, limit);
      setPWM(1, 0, 0, 0, 0, 0);

      //Motor 3
      setPWM(4, invertedPWM, AnalogMin, AnalogMax, 0, limit);
      setPWM(5, 0, 0, 0, 0, 0);

      //Motor 2
      setPWM(2, AnalogLY, AnalogMin, AnalogMax, 0, maxPWM);
      setPWM(3, 0, 0, 0, 0, 0);

      //Motor 4
      setPWM(6, AnalogLY, AnalogMin, AnalogMax, 0, maxPWM);
      setPWM(7, 0, 0, 0, 0, 0);
    }

    //Vira pra Direita (Analógico R para Direita) - RÉ
    if (AnalogRX > AnalogMin && AnalogLY < -AnalogMin && R2 == 0 && L2 == 0){
    
      //Motor 1
      setPWM(1, -AnalogLY, AnalogMin, AnalogMax, 0, maxPWM);
      setPWM(0, 0, 0, 0, 0, 0);

      //Motor 3
      setPWM(5, -AnalogLY, AnalogMin, AnalogMax, 0, maxPWM);
      setPWM(4, 0, 0, 0, 0, 0);

      //Inverte PWM
      limit = map(-AnalogLY, AnalogMin, AnalogMax, 0, maxPWM);
      invertedPWM = inverted_analog_input_scaler(AnalogRX, AnalogMin, AnalogMax, AnalogMin, AnalogMax);
      Serial.print("invertedPWM: ");
      Serial.println(invertedPWM);
      
      //Motor 2
      setPWM(3, invertedPWM, AnalogMin, AnalogMax, 0, limit);
      setPWM(2, 0, 0, 0, 0, 0);

      //Motor 4
      setPWM(7, invertedPWM, AnalogMin, AnalogMax, 0, limit);
      setPWM(6, 0, 0, 0, 0, 0);
    }

    //Vira pra Esquerda (Analógico R para esquerda) - RÉ
    if (AnalogRX < -AnalogMin && AnalogLY < -AnalogMin && R2 == 0 && L2 == 0){

      //Inverte PWM
      limit = map(-AnalogLY, AnalogMin, AnalogMax, 0, maxPWM);
      invertedPWM = inverted_analog_input_scaler(-AnalogRX, AnalogMin, AnalogMax, AnalogMin, AnalogMax);
      //Serial.print("invertedPWM: ");
      //Serial.println(invertedPWM);
      
      //Motor 1
      setPWM(1, invertedPWM, AnalogMin, AnalogMax, 0, limit);
      setPWM(0, 0, 0, 0, 0, 0);

      //Motor 3
      setPWM(5, invertedPWM, AnalogMin, AnalogMax, 0, limit);
      setPWM(4, 0, 0, 0, 0, 0);

      //Motor 2
      setPWM(3, -AnalogLY, AnalogMin, AnalogMax, 0, maxPWM);
      setPWM(2, 0, 0, 0, 0, 0);

      //Motor 4
      setPWM(7, -AnalogLY, AnalogMin, AnalogMax, 0, maxPWM);
      setPWM(6, 0, 0, 0, 0, 0);
    }

    //Giro 360 para direita (R2 + Analógico R para Direita)
    if (AnalogRX > AnalogMin && nullAnalogLY == true && nullAnalogRX == false && R2 == 1 && L2 == 0) {
      
      //Motor 1
      setPWM(0, AnalogRX, AnalogMin, AnalogMax, 0, maxPWM);
      setPWM(1, 0, 0, 0, 0, 0);

      //Motor 2
      setPWM(2, 0, 0, 0, 0, 0);
      setPWM(3, AnalogRX, AnalogMin, AnalogMax, 0, maxPWM);

      //Motor 3
      setPWM(4, AnalogRX, AnalogMin, AnalogMax, 0, maxPWM);
      setPWM(5, 0, 0, 0, 0, 0);

      //Motor 4
      setPWM(6, 0, 0, 0, 0, 0);
      setPWM(7, AnalogRX, AnalogMin, AnalogMax, 0, maxPWM);
    }

    //Giro 360 para esquerda (R2 + Analógico R para esquerda)
    if (AnalogRX < -AnalogMin && nullAnalogLY == true && nullAnalogRX == false && R2 == 1 && L2 == 0) {
      
      //Motor 1
      setPWM(0, 0, 0, 0, 0, 0);
      setPWM(1, -AnalogRX, AnalogMin, AnalogMax, 0, maxPWM);

      //Motor 2
      setPWM(2, -AnalogRX, AnalogMin, AnalogMax, 0, maxPWM);
      setPWM(3, 0, 0, 0, 0, 0);

      //Motor 3
      setPWM(4, 0, 0, 0, 0, 0);
      setPWM(5, -AnalogRX, AnalogMin, AnalogMax, 0, maxPWM);

      //Motor 4
      setPWM(6, -AnalogRX, AnalogMin, AnalogMax, 0, maxPWM);
      setPWM(7, 0, 0, 0, 0, 0);
    }
    
    //Detecta inatividade dos dois analógicos
    if (nullAnalogLY == true && nullAnalogRX == true)
    {
      nullDualAnalog = true;
    } else {
      nullDualAnalog = false;
    }

    //Anula movimento de giro 360 se analógicos inativos e R2 inativo
    if (nullDualAnalog == true && R2 == false)
    {
      nullGiro = true;
    } else {
      nullGiro = false;
    }

    //Freio
    if (L2 == 1 || nullDualAnalog == true || nullGiro == true){
      setPWM(0, 0, 0, 0, 0, 0);
      setPWM(1, 0, 0, 0, 0, 0);
      setPWM(2, 0, 0, 0, 0, 0);
      setPWM(3, 0, 0, 0, 0, 0);
      setPWM(4, 0, 0, 0, 0, 0);
      setPWM(5, 0, 0, 0, 0, 0);
      setPWM(6, 0, 0, 0, 0, 0);
      setPWM(7, 0, 0, 0, 0, 0);
    }

  //
}
