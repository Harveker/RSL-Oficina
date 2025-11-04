#define MDB1 10
#define MDB2 11    //frente direit
#define MEB1 9
#define MEB2 6     //frente esquerd
#define LD A2
#define CD A3
#define CE A4
#define LE A5
#define BUT 7
#define LED 12

#include "math.h"

int ultEstadoBotao;
int botao;
int tempo = 0;
int tempoatual = millis()/100000;
int pwm_increment = 0;
int acell;
bool ligado;

int valorLE;
int valorCE;
int valorCD;
int valorLD;
  
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  //motor
  pinMode(MDB1,OUTPUT);
  pinMode(MDB2,OUTPUT);
  pinMode(MEB1,OUTPUT);
  pinMode(MEB2,OUTPUT);
  //sensor
  pinMode(LD,INPUT);
  pinMode(LE,INPUT);
  pinMode(CE,INPUT);
  pinMode(CD,INPUT);
  pinMode(BUT,INPUT);
  //LED
  pinMode(LED,OUTPUT);
  tempo = millis()/100000;
}

void loop() {
  // put your main code here, to run repeatedly:
  // Lê os valores dos sensores
  Serial.println(tempoatual);
  Serial.print(tempo);
     valorLE = analogRead(LE);
     valorCE = analogRead(CE);
     valorCD = analogRead(CD);
     valorLD = analogRead(LD);
  if (tempoatual - tempo > ) {
     Serial.print("LE: ");
     Serial.print(valorLE);
     Serial.print(" | CE: ");
     Serial.print(valorCE);
     Serial.print(" | CD: ");
     Serial.print(valorCD);
     Serial.print(" | LD: ");
     Serial.println(valorLD);
     tempo = millis();
  }
  // Imprime os valores no monitor serial
     
  /*
  if(button_release()){  
    analogWrite(MDB1,LOW);
    analogWrite(MDB2,HIGH);
    analogWrite(MEB1,LOW);
    analogWrite(MEB2,HIGH);
  }*/
  /*
  if (millis()-tempo == 1000){
    digitalWrite(MDB1,LOW);
    digitalWrite(MDB2,LOW);
    digitalWrite(MEB1,LOW);
    digitalWrite(MEB2,LOW);
  }*/
  tempo = millis();
  int acell = sin(2*M_PI*tempo)*255/1000;
  if(button_release()){
      analogWrite(MDB1,0);
      analogWrite(MDB2,acell);
      analogWrite(MEB1,0);
      analogWrite(MEB2,acell);
      ligado = true;
      tempo = millis();
  }
  if (ligado) {
  if ((analogRead(LE) <= 200) && (analogRead(CE) <= 200)) {
      analogWrite(MDB1,0);
      analogWrite(MDB2,127);
      analogWrite(MEB1,0);
      analogWrite(MEB2,0);
    } else if ((analogRead(LD) <= 200) && (analogRead(CD) <= 200)) {
      analogWrite(MDB1,0);
      analogWrite(MDB2,0);
      analogWrite(MEB1,0);
      analogWrite(MEB2,127);
    } else if ((analogRead(LD) <= 200) && (analogRead(CD) <= 200)) {
      analogWrite(MDB1,0);
      analogWrite(MDB2,127);
      analogWrite(MEB1,0);
      analogWrite(MEB2,0);
    } else if ((analogRead(CE) <= 200)) {
      analogWrite(MDB1, 0);
      analogWrite(MDB2, 50);
      analogWrite(MEB1, 0);
      analogWrite(MEB2, 0);
    } 
    else if ((analogRead(CD) <= 200)) {
      analogWrite(MDB1, 0);
      analogWrite(MDB2, 0);
      analogWrite(MEB1, 0);
      analogWrite(MEB2, 50);
    }
    else if ((analogRead(LD) <= 200) && (analogRead(CD) <= 200) && (analogRead(CE) <= 200) && (analogRead(LE) <= 200)) {
      analogWrite(MDB1, 0);
      analogWrite(MDB2, 0);
      analogWrite(MEB1, 0);
      analogWrite(MEB2, 0);
      ligado = false;
    } 
    
    else if ((analogRead(LD) >= 900) && (analogRead(CD) >= 900) && (analogRead(CE) > 900) && (analogRead(LE) <= 900)) {
      analogWrite(MDB1, acell);
      analogWrite(MDB2, 0);
      analogWrite(MEB1, acell);
      analogWrite(MEB2, 0);
      tempo= millis();
    } 
  }
}

bool button_release(){
   int botao = digitalRead(BUT);
   if(botao == LOW && ultEstadoBotao == HIGH){
     delay(50);
     return true;
   }
   ultEstadoBotao = digitalRead(BUT);
}
