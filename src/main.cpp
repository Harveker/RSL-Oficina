#define TDIR 10  // trás direita
#define FDIR 11  // frente direita
#define TESQ 9   // trás esquerda
#define FESQ 6   // frente esquerda
#define LD A2    // sensor lateral direito
#define CD A3    // sensor central direito
#define CE A4    // sensor central esquerdo
#define LE A5    // sensor lateral esquerdo
#define BUT 7    // botão
#define LED 12   // LED
#define TMDF 90  // porcentagem de trim do motor direito frente
#define TMDT 90  // porcentagem de trim do motor direito tras
#define TMEF 100 // porcentagem de trim do motor esquerdo frente
#define TMET 100 // porcentagem de trim do motor esquerdo tras
#include <Arduino.h>
#include <math.h>
#include "AcksenButton.h"

// ENUMERAÇÕES E STRUCTS-------------------------------------------------------------------------------------------------------------------------------

// Struct para controle das I/Os do robô
struct controlIO
{
  enum Bobinas // enumerando para facilitar a identificação das bobinas
  {
    TDIR_,
    FDIR_,
    TESQ_,
    FESQ_
  };

  enum Sensores // enumerando para facilitar a identificação dos sensores
  {
    LD_,
    CD_,
    CE_,
    LE_
  };
  const int trimMotores[4] = {TMDT, TMDF, TMET, TMEF}; // array com os trims dos motores
  int bobinas[4] = {TDIR_, FDIR_, TESQ_, FESQ_};       // array com as bobinas
  int sensores[4] = {LD_, CD_, CE_, LE_};              // array com os sensores
  char debugsensores[4][3] = {"LD", "CD", "CE", "LE"}; // array com os nomes dos sensores para debug
  int readings[4];                                     // array para armazenar as leituras dos sensores
} typedef cIO;

// CONSTANTES GLOBAIS-------------------------------------------------------------------------------------------------------------------------------
const int DURACAO = 1000;         // duração de um ciclo completo em ms
const int SENSIBILIDADE = 650;    // sensibilidade do controle de linha
const int LONGPRESS = 2000;       // tempo em ms para considerar um long press
const int VELOCIDADE = 255;       // velocidade do motor
const int VELOCIDADE_CURVA = 150; // velocidade do motor em curvas
const int VELOCIDADE_SUAVE = 120; // velocidade do motor em curvas suaves
const int VELOCIDADE_SUAVE_CURVA = 100; // velocidade do motor em curvas suaves

// VARIÁVEIS GLOBAIS-------------------------------------------------------------------------------------------------------------------------------
float acellCrescente = 0;
float acellDecrescente = 0;
unsigned long tempo = 0;
unsigned long tempoatual = millis();
unsigned long tempo_motor = 0;
int pwm_increment = 0;
bool ligado;
const unsigned long debounce = 50; // debounce do botão em ms
int estado = 0;
// FUNÇÕES AUXILIARES-------------------------------------------------------------------------------------------------------------------------------
// função para controlar a direção do robô
void controleDirecao(int direitaFrente, int direitaTras, int esquerdaFrente, int esquerdaTras);
// função para mover o robô para frente
void frente();
// função para mover o robô para trás
void tras();
// função para parar o robô
void parar();
// função para girar o robô para a esquerda
void esquerdaSuave();
// função para girar o robô para a direita
void direitaSuave();
// função para girar o robô para a esquerda
void esquerda();
// função para girar o robô para a direita
void direita();
// função para ler os sensores
void leituras();
// função para calibrar os motores com base na porcentagem de trim
void calibracaoMotores(int pwm, const int porcentagemTrim[4]);
// função para aplicar o trim ao valor de pwm
int trim(int pwm, int porcentagemTrim);
// função para verificar se apenas um sensor está detectando linha
bool apenasUmSensor(int indiceSensor);
// função para verificar se a parada foi sensoriada
bool paradaSensoriada();

int sensoriadoExtremo();
// INSTÂNCIAS-------------------------------------------------------------------------------------------------------------------------------

cIO robo;                                                                // instancia do struct de controle de I/O
AcksenButton button(BUT, ACKSEN_BUTTON_MODE_LONGPRESS, debounce, INPUT); // instancia do botão
//--------------------------------------------------------------------SETUP--------------------------------------------------------//-------------------------------
void setup()
{
  Serial.begin(115200); // inicializa a comunicação serial
  // Inicializa as bobinas como saída e os sensores como entrada
  for (size_t i = 0; i < 4; i++) // pode se criar uma função para isso algo como robo.init();
  {
    pinMode(robo.bobinas[i], OUTPUT);
    pinMode(robo.sensores[i], INPUT);
    // Botão
    pinMode(BUT, INPUT); // Botão como entrada
    // LED
    pinMode(LED, OUTPUT); // LED como saída
    // Inicializa variáveis
    digitalWrite(LED, LOW);
  }
  // Cálculo das acelerações oscilatórias - pode se colocar essas funções no geogebra para visualizar melhor o comportamento
  tempo_motor = (millis() % DURACAO) / (float)DURACAO;                       // tempo do motor varia de 0 a 1 em um ciclo de DURACAO ms
  acellCrescente = ((0.5) * sin(M_PI * tempo_motor) + 0.5);                  // aceleração 1 é o inverso da aceleração 2
  acellDecrescente = ((0.5) * cos((M_PI / 2) + (M_PI * tempo_motor)) + 0.5); // aceleração 2 é o inverso da aceleração 1
  // Cálculo das acelerações lineares
  // int acell1 = ((tempoatual - tempo)/5)*255;
  // int acell2 = ((tempoatual - tempo)/5)*255;
  button.setLongPressInterval(LONGPRESS); // define o intervalo de long press para 2 segundos
}
//--------------------------------------------------------------------LOOP--------------------------------------------------------//-------------------------------
void loop()
{
  // Atualizações gerais
  leituras();             // função para ler os sensores
  button.refreshStatus(); // atualiza o estado do botão
  tempoatual = millis();  // atualiza o tempo atual em milissegundos

  if (tempoatual - tempo > 1000)
  {                                // a cada 1 segundo
    for (size_t i = 0; i < 4; i++) // loop para imprimir os valores dos sensores
    {
      tempo = tempoatual;
      Serial.print(robo.debugsensores[i]); // imprime o nome do sensor
      Serial.print(": ");                  // imprime dois pontos
      Serial.println(robo.readings[i]);    // imprime o valor do sensor
    }
  }

  // limita a aceleração máxima
  if (acellCrescente > 255)
    acellCrescente = 255;
  if (acellDecrescente > 255)
    acellDecrescente = 255;

  //----------------------------------------------------------------CALIBRACAO--------------------------------------------------------//
  if (button.onLongPress()) // se o botão for pressionado por longo período
  {
    digitalWrite(LED, HIGH); // acende o LED
    Serial.println("Iniciando calibração dos motores...");
    // momento de calibração dos motores:
    calibracaoMotores(100, robo.trimMotores);
    delay(1500);
    controleDirecao(0, 0, 0, 0); // para o robô
    Serial.println("Robo calibrado e ligado!");
    button.setButtonOperatingMode(ACKSEN_BUTTON_MODE_NORMAL); // muda o modo do botão para NORMAL
    button.setLongPressInterval(LONGPRESS);                   // define o intervalo de long press para 2 segundos
  }

  //----------------------------------------------------------------CONTROLE DE LINHA--------------------------------------------------------//
  if (ligado || button.onPressed()) // se o botão estiver ligado e for pressionado por longo período
  {                                 // se o botão for pressionado por mais de 2 segundos
    ligado = true;                  // define o estado como ligado
    Serial.println("Robo modo linha ativado!");
    digitalWrite(LED, LOW);                                   // apaga o LED
    button.setButtonOperatingMode(ACKSEN_BUTTON_MODE_NORMAL); // muda o modo do botão para NORMAL
    frente();

    if (apenasUmSensor(cIO::LD_)) // se o sensor central direito detectar linha, gira para a direita
    {
      estado = 1;
      direita();
    }
    else if (apenasUmSensor(cIO::LE_)) // se o sensor central esquerdo detectar linha, gira para a esquerda
    {
      estado = 2;      
      esquerda();
    }
    else if (apenasUmSensor(cIO::CE_)) // se o sensor esquerdo detectar linha, faz uma curva leve para a esquerda
    {
      if (estado == 0 || estado == 3)
      {
        esquerdaSuave();
        estado = 4;
      }
      if (estado == 1)
      {
        esquerda();
        estado = 0;
      }
    }
    else if (apenasUmSensor(cIO::CD_)) // se o sensor direito detectar linha, faz uma curva leve para a direita
    {
      if (estado == 0 || estado == 4)
      {
        direitaSuave();
        estado = 3;
      }
      if (estado == 2)
      {
        direita();
        estado = 0;
      }
    }
    else if (sensoriadoExtremo() == 1)
    {
      direita();
      estado = 1;
    }
    else if (sensoriadoExtremo() == 2)
    {
      esquerda();
      estado = 2;
    }
    else if (paradaSensoriada()) // se todos os sensores detectarem linha, para o robô
    {
      estado = 0;
      parar();
      ligado = false; // define o estado como desligado
    }
  }
}
// FUNÇÕES AUXILIARES
int trim(int pwm, int porcentagemTrim) // Calcula o valor do PWM com base na porcentagem de trim
{
  long pwmCalibrado = (long)pwm * porcentagemTrim / 100;
  return constrain(pwmCalibrado, 0, 255);
}

void calibracaoMotores(int pwm, const int porcentagemTrim[4]) // Calibra os motores com base na porcentagem de trim
{
  for (size_t i = 0; i < 4; i++)
  {
    int pwmCalibrado = trim(pwm, porcentagemTrim[i]);
    analogWrite(robo.bobinas[i], pwmCalibrado);
  }
}

void controleDirecao(int direitaFrente, int direitaTras, int esquerdaFrente, int esquerdaTras)
{
  analogWrite(FDIR, direitaFrente);
  analogWrite(TDIR, direitaTras);
  analogWrite(FESQ, esquerdaFrente);
  analogWrite(TESQ, esquerdaTras);
}

// Funções de movimento - basicas, como otimizar?
// note que os valores de PWM vão de 0 a 255 - estamos usando valores menores.
// aqui também podemos aplicar o trim dos motores
void frente()
{
  // Serial.println("Movendo para frente");
  // controleDirecao(100, 0, 100,0 );
  controleDirecao(trim(VELOCIDADE, robo.trimMotores[0]), 0, trim(VELOCIDADE, robo.trimMotores[2]), 0); //-- exemplo com trim
}
void tras()
{
  // Serial.println("Movendo para tras");
  // controleDirecao(0, 100, 0, 100);
  controleDirecao(0, trim(VELOCIDADE, robo.trimMotores[1]), 0, trim(VELOCIDADE, robo.trimMotores[3]));
}
void parar()
{
  // Serial.println("Parando o robo");
  controleDirecao(0, 0, 0, 0);
}
void esquerda()
{
  // Serial.println("Movendo para esquerda");
  // controleDirecao(100, 0, 0, 100);
  controleDirecao(trim(VELOCIDADE, robo.trimMotores[0]), 0, 0, trim(VELOCIDADE_CURVA, robo.trimMotores[2]));
}
void esquerdaSuave()
{
  // Serial.println("Movendo para esquerda suave");

  // controleDirecao(175, 0, 100, 0);
  //controleDirecao(trim(VELOCIDADE, robo.trimMotores[0]), 0, 0, 0);
  controleDirecao(trim(VELOCIDADE, robo.trimMotores[0]), 0, trim(VELOCIDADE_CURVA, robo.trimMotores[2]), 0);
}
void direita()
{
  // Serial.println("Movendo para direita");
  // controleDirecao(0, 100, 100, 0);
  controleDirecao(0,trim(VELOCIDADE_CURVA, robo.trimMotores[2]), trim(VELOCIDADE, robo.trimMotores[2]), 0);
}
void direitaSuave()
{
  // Serial.println("Movendo para direita suave");
  // controleDirecao(100, 0, 100, 0);
  //controleDirecao(0, 0, trim(VELOCIDADE, robo.trimMotores[2]), 0);
  controleDirecao(trim(VELOCIDADE_CURVA, robo.trimMotores[0]), 0, trim(VELOCIDADE, robo.trimMotores[2]), 0);
}

bool paradaSensoriada()
{
  // se todos os sensores detectarem linha (valor abaixo da sensibilidade), retorna true
  if (robo.readings[cIO::LD_] < SENSIBILIDADE &&
      robo.readings[cIO::CD_] < SENSIBILIDADE &&
      robo.readings[cIO::CE_] < SENSIBILIDADE &&
      robo.readings[cIO::LE_] < SENSIBILIDADE)
  {
    return true;
  }
  else
  {
    return false;
  }
}
int sensoriadoExtremo(){
  if (robo.readings[cIO::LD_] < SENSIBILIDADE &&
      robo.readings[cIO::CD_] < SENSIBILIDADE &&
      robo.readings[cIO::CE_] > SENSIBILIDADE &&
      robo.readings[cIO::LE_] > SENSIBILIDADE)
  {
    return 1;  //direita extrema
  }else if (robo.readings[cIO::LD_] > SENSIBILIDADE &&
      robo.readings[cIO::CD_] > SENSIBILIDADE &&
      robo.readings[cIO::CE_] < SENSIBILIDADE &&
      robo.readings[cIO::LE_] < SENSIBILIDADE)
  {
    return 2;    //esquerd extrema
  }
  
  else
  {
    return 0;
  }
}
bool apenasUmSensor(int indiceSensor)
{
  for (int i = 0; i < 4; i++)
  {
    if (i == indiceSensor)
    {
      if (robo.readings[i] >= SENSIBILIDADE) // se for o sensor especificado
      {
        return false; // o sensor especificado não está detectando linha
      }
    }
    else
    {
      if (robo.readings[i] < SENSIBILIDADE) // se for outro sensor
      {
        return false; // outro sensor está detectando linha
      }
    }
  }
  return true; // apenas o sensor especificado está detectando linha
}

// Função para ler os sensores
void leituras()
{
  robo.readings[cIO::LD_] = analogRead(LD);
  robo.readings[cIO::CD_] = analogRead(CD);
  robo.readings[cIO::CE_] = analogRead(CE);
  robo.readings[cIO::LE_] = analogRead(LE);
}