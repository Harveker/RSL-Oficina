#define TDIR 10  //trás direita
#define FDIR 11  //frente direita
#define TESQ 9   //trás esquerda
#define FESQ 6   //frente esquerda
#define LD A2
#define CD A3
#define CE A4
#define LE A5
#define BUT 7
#define LED 12

#include <Arduino.h>
#include <math.h>
#include "AcksenButton.h"

//ENUMERAÇÕES E STRUCTS

//Struct para controle das I/Os do robô
struct controlIO {
  enum Bobinas {TDIR_,FDIR_,TESQ_,FESQ_};   // enumerando para facilitar a identificação das bobinas
  enum Sensores {LD_,CD_,CE_,LE_};  // enumerando para facilitar a identificação dos sensores
  int bobinas[4]  = {TDIR_, FDIR_, TESQ_, FESQ_}; // array com as bobinas
  int sensores[4] = {LD_, CD_, CE_, LE_}; // array com os sensores
  char debugsensores[4][3] = {"LD","CD","CE","LE"}; //array com os nomes dos sensores para debug
  int readings[4];  // array para armazenar as leituras dos sensores
}typedef cIO;

//CONSTANTES GLOBAIS
const int DURACAO = 1000; //duração de um ciclo completo em ms
const int SENSIBILIDADE = 650; //sensibilidade do controle de linha

//VARIÁVEIS GLOBAIS
int acellCrescente = 0;
int acellDecrescente = 0;
int tempo = 0;
int tempoatual = millis()/1000;
int tempo_motor = 0;
int pwm_increment = 0;
int acell;
bool ligado;
const unsigned long debounce = 50; //debounce do botão em ms

//FUNÇÕES AUXILIARES
void direcao(int direitaFrente, int direitaTras, int esquerdaFrente, int esquerdaTras); //função para controlar a direção do robô

void leituras();

//INSTÂNCIAS
cIO robo; //instancia do struct de controle de I/O
AcksenButton button(BUT,ACKSEN_BUTTON_MODE_NORMAL,debounce,INPUT);  //instancia do botão

void setup() {
  Serial.begin(9600);
  // Inicializa as bobinas como saída e os sensores como entrada
  for (size_t i = 0; i < 4; i++)  //pode se criar uma função para isso algo como robo.init();
  {
    pinMode(robo.bobinas[i],OUTPUT);
    pinMode(robo.sensores[i],INPUT);
  //Botão
  pinMode(BUT,INPUT);  //Botão como entrada
  //LED
  pinMode(LED,OUTPUT);  //LED como saída
  //Inicializa variáveis
  digitalWrite(LED,LOW);
  }
}
void loop() {
  // put your main code here, to run repeatedly:
  leituras(); //função para ler os sensores
  button.refreshStatus(); //atualiza o estado do botão
  tempoatual = millis()/1000;

  if (tempoatual - tempo > 1) { // a cada 1 segundo 
    for (size_t i = 0; i < 4; i++) //loop para imprimir os valores dos sensores
    {
      tempo = tempoatual;
      Serial.print(robo.debugsensores[i]);  //imprime o nome do sensor
      Serial.print(": "); //imprime dois pontos
      Serial.println(robo.readings[i]); //imprime o valor do sensor
    }
  }

  //Cálculo das acelerações oscilatórias
  acellCrescente = ((0.5)*sin(tempo_motor)+0.5)*255;   //aceleração 1 é o inverso da aceleração 2
  acellDecrescente = ((0.5)*cos((M_PI/2)+(tempo_motor))+0.5)*255;   //aceleração 2 é o inverso da aceleração 1

  //Cálculo das acelerações lineares
  //int acell1 = ((tempoatual - tempo)/5)*255;
  //int acell2 = ((tempoatual - tempo)/5)*255;
  //limita a aceleração máxima
  if (acellCrescente > 255) {
    acellCrescente = 255;
  }
  if (acellDecrescente > 255) {
    acellDecrescente = 255;
  }

  if(button.onPressed()) {  //se o botão for pressionado
      direcao(0, 100, 0, 100); //anda para frente
      digitalWrite(LED,HIGH); //acende o LED
      ligado = true;
      tempo = millis();
  }
  if (ligado) {
    //se o sensor central direito detectar linha, gira para a esquerda
    robo.readings[cIO::CD_] < SENSIBILIDADE ? ((tempo_motor = tempoatual),direcao(acellCrescente,0,acellDecrescente,0)) : direcao(acellCrescente,0,acellCrescente,0);
    robo.readings[cIO::CE_] < SENSIBILIDADE ? ((tempo_motor = tempoatual),direcao(acellDecrescente,0,acellCrescente,0)): direcao(acellCrescente,0,acellCrescente,0);
    robo.readings[cIO::LD_] < SENSIBILIDADE ? ((tempo_motor = tempoatual),direcao(acellCrescente,0,-acellDecrescente,0)): direcao(acellCrescente,0,acellCrescente,0);
    robo.readings[cIO::LE_] < SENSIBILIDADE ? ((tempo_motor = tempoatual),direcao(-acellDecrescente,0,acellCrescente,0)): direcao(acellCrescente,0,acellCrescente,0);
  }
}

void direcao(int direitaFrente, int direitaTras, int esquerdaFrente, int esquerdaTras){
  analogWrite(FDIR, direitaFrente);
  analogWrite(TDIR, direitaTras);
  analogWrite(FESQ, esquerdaFrente);
  analogWrite(TESQ, esquerdaTras);
}
void leituras(){
  robo.readings[cIO::LD_] = analogRead(LD);
  robo.readings[cIO::CD_] = analogRead(CD);
  robo.readings[cIO::CE_] = analogRead(CE);
  robo.readings[cIO::LE_] = analogRead(LE);
}