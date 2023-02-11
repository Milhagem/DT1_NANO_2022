// Vamos calibrar o divisor de tensão e o sensor de corrente para o INA 226
// R5 = 20 kΩ
// R6 = 68 kΩ
// Divisor de tensão INA = R6/(R5 + R6) = 0,7727272... kΩ
// Divisor de tensão INA^-1 = (R5 + R6)/R6 = 1,29411764706
// fatorCorreçãoV = 1.01626

// Resistor do shunt = 1 mΩ
// FatorCorreçãoC = 0,482625

#include <INA.h> // INA Library (by Zanshin)
#include <SD.h>
#include <math.h>

// Portas definidas MEGA/DUE
#define pinoCS 10 // Pino Chip Select do datalogger
#define saida_led 6

#define sinal_corrente 8 // Permite ou proíbe o acionamento do motor
#define hall_motor 2
#define hall_roda 9

#define corrente_limite 20
//#define R5 20000
//#define R6 68000
//#define valorShunt 0.001
//#define fatorCorrecaoV 1.01626
//#define fatorCorrecaoC 0.482625

#define fatorMili 0.001
#define fatorMicro 0.000001

double tempo_escrita; // monitora o tempo para escrever no arquivo
double tempo_salvar;  // monitora o tempo para salvar o arquivo
File myFile;

// Variaveis do sensor INA226
INA_Class INA; // Construct a power monitor object names "ina"
int32_t rawCurrent = -666;
float corrente_motor_INA;
uint16_t rawVoltage = 666;
float tensao_bat_INA;
int32_t rawShunt = -666;
float shunt_motor_INA;

#define circunferencia_da_roda 1.596
#define circunferencia_do_motor 0.3267
#define imas_roda 3 // indica o numero de imas na roda
#define imas_motor 4

// Hall = Estado lógico do sensor // contagem = conta numero de voltas da roda (também conta o número de imãs que passa pelo sensor) //distancia_total [m]
long hall_valor_roda, contagem_roda;
long hall_valor_motor, contagem_motor;

// velocidade_instantanea e media [km/h]
float velocidade_instantanea_roda = 0;
float velocidade_instantanea_motor = 0;

// diferença_pulsos = diferença entre os pulsos atual e anterior =~[periodo]
volatile float diferenca_pulsos_roda = 0;
volatile float diferenca_pulsos_motor = 0;

// estado_ant = conta o estado lógico anterior do sensor Hall
int estado_ant_roda = 0;
int estado_ant_motor = 0;

// Variaveis de tempo:
unsigned long tempo_atual = 0; // Tempo em milisegundos medido no Loop atual.
volatile float tempo_anterior1_roda = 0, tempo_anterior_roda = 0;
volatile int conta_roda = LOW;
volatile float tempo_anterior1_motor = 0, tempo_anterior_motor = 0;
volatile int conta_motor = LOW;

void setup()
{
  tempo_atual = 0;
  contagem_roda = 0;
  tempo_anterior_roda = 0;
  velocidade_instantanea_roda = 0;
  estado_ant_roda = LOW;

  contagem_motor = 0;
  tempo_anterior_motor = 0;
  velocidade_instantanea_motor = 0;
  estado_ant_motor = LOW;

  attachInterrupt(digitalPinToInterrupt(2), H_motor, RISING);
  attachInterrupt(digitalPinToInterrupt(3), H_roda, RISING);
  pinMode(sinal_corrente, OUTPUT);
  pinMode(hall_motor, INPUT);
  pinMode(hall_roda, INPUT);

  tempo_escrita = 0.0;
  tempo_salvar = 0.0;

  SD.begin(pinoCS);
  myFile = SD.open("RamChu.txt", FILE_WRITE);
  myFile.println("Tensao, Corrente,Vel_roda,Vel_motor,Tempo");

  if (myFile)
  {
    digitalWrite(saida_led, HIGH);
  }
  else
  {
    digitalWrite(saida_led, LOW);
  }

  INA.begin(80, 1000, 0x40);             // Begin calibration for an expected ±1 Amps maximum current and for a 0.1Ohm resistor
  INA.setAveraging(10);                  // Average each reading n-times
  INA.setBusConversion(10000);           // Maximum conversion time 8.244ms
  INA.setShuntConversion(10000);         // Maximum conversion time 8.244ms
  INA.setMode(INA_MODE_CONTINUOUS_BOTH); // Bus/shunt measured continuously

  delay(20);
}

//___________________________________________________________________________________LOOP_________________________________________________________________________________________________________
void loop()
{
  valor_sensor();
  calculo_velocidade();
  datalogger();
}

//________________________________________________________________________________valor_sensor_______________________________________________________________________________________________________-
/*
 * Valor_sensor tem a função de calcular os valores de corrente e tensão recebidos do INA
 * Além disso, se o valor for muito alto, ele gera um valor no pino ligado à variável sinal_corrente.
 * Esse valor atua como um sistema de segurança que impede que o motor acione caso ultrapasse um limite
 */
void valor_sensor()
{
  /*___________________Sensor de Tensão INA___________________________*/

  float R5 = 20; // Resistência em kΩ
  float R6 = 68; // Resistência em kΩ
  double valorShunt = 0.001;
  double fatorCorrecaoV = 1.01626;
  double fatorCorrecaoC = 0.482625;

  rawVoltage = INA.getBusMilliVolts();
  tensao_bat_INA = (float)rawVoltage * ((R5 + R6) / R6) * fatorCorrecaoV * fatorMili; // Esse valor que está sendo multiplicado pelo valor da tensão tem a função de calibrar o sensor.
  // tensao_bat_INA = (float)rawVoltage * 0.00131516;
  //  Vbat = Vina *      1.29411764706       *    1.01626    *   0.001
  //  Vbat = Vina * fatorDivisorTensao * fatorCorreçãoV * fatorMili
  /*______________________________________________________________________*/

  /*__________________Sensor de Corrente INA______________________________*/

  rawCurrent = INA.getShuntMicroVolts();
  corrente_motor_INA = (float)rawCurrent * (fatorMicro / valorShunt) * fatorCorrecaoC;
  // Imotor = (Iina * 10^-6 / 0.001) * 0,482625
  // Imotor = (Iina * fatorMicroVolt / valorShunt) * fatorCorreçãoC
  /*______________________________________________________________________*/

  // Serial.println(corrente_motor_INA);

  if (corrente_motor_INA >= corrente_limite)
  {
    digitalWrite(sinal_corrente, HIGH); // A variável sinal_corrente representa uma saída que vai para no mega/due e atua como um dispositivo de segurança que permite ou proíbe o acionamento
  }
  else
  {
    digitalWrite(sinal_corrente, LOW);
  }
}

//________________________________________________________________________________Cálculo_velocidade_______________________________________________________________________________________-

/*
 * A função calculo_velocidade tem a função de calcular a velocidade do motor e da roda
 * Essa função utiliza os dados obtidos em H_roda e H_motor para realizar os cálculos
 */
void calculo_velocidade()
{
  /*____________________________Cálculo_Velocidade_Roda__________________________*/
  tempo_atual = millis();
  hall_valor_roda = digitalRead(hall_roda);
  if (conta_roda == HIGH)
  {
    // O coeficiente 3600000 representa: 1000000(converte us pra s) * 3,6(converte m/s para km/h)
    velocidade_instantanea_roda = ((float)circunferencia_da_roda / (diferenca_pulsos_roda * imas_roda)) * 3600000.0; // diferenca_pulsos em us
    tempo_anterior_roda = tempo_atual;
    conta_roda = LOW;
  }
  /*____________________________Cálculo_Velocidade_Motor__________________________*/

  tempo_atual = millis();
  hall_valor_motor = digitalRead(hall_motor);
  if (conta_motor == HIGH)
  {
    // O coeficiente 3600000 representa: 1000000(converte us pra s) * 3,6(converte m/s para km/h)
    velocidade_instantanea_motor = ((float)circunferencia_do_motor / (diferenca_pulsos_motor * imas_motor)) * 3600000.0; // diferenca_pulsos em us
    tempo_anterior_motor = tempo_atual;
    conta_motor = LOW;
  }
}
//____________________________________________________________________________________DATALOGGER____________________________________________________________________________________________________
/*
 * A função datalogger tem a função de armazenar os dados de velocidade instantânea, tensão da bateria e corrente no motor
 * Os dados ficam salvos em um cartão SD
 */
void datalogger()
{
  if ((millis() - tempo_escrita) > 100)
  {
    myFile.print(tensao_bat_INA);
    myFile.print(',');
    myFile.print(corrente_motor_INA);
    myFile.print(',');
    myFile.print(velocidade_instantanea_roda);
    myFile.print(',');
    myFile.print(velocidade_instantanea_motor);
    myFile.print(',');
    tempo_escrita = millis();
    myFile.println(tempo_escrita);
  }
  if ((millis() - tempo_salvar) > 100)
  {
    myFile.flush();
    tempo_salvar = millis();
  }
}

//_________________________________________________________________________________H_roda________________________________________________________________________________________________________
/*
 * A interrupção H_roda tem a função de fazer uma contagem de quantas vezes o sensor hall/encoder deu um pulso de sinal
 * Além disso, ela calcula a diferença no tempo dos pulsos
 * Esse valor da diferença no tempo dos pulsos é usado para o cálculo da velocidade
 */
void H_roda()
{
  contagem_roda++; // Faz a contagem dos pulsos
  if (contagem_roda % 2 == 0)
  {
    tempo_anterior1_roda = micros();                                    // Tempo atual
    diferenca_pulsos_roda = tempo_anterior1_roda - tempo_anterior_roda; // Cálculo do tempo
    conta_roda = HIGH;
  }
  else
  {
    tempo_anterior_roda = micros(); // Atualiza o tempo que será usado como o momento do último pulso quando a função for chamada novamente
  }
}

//________________________________________________________________________________H_motor_______________________________________________________________________________________________________
/*
 *A interrupção H_motor tem a função de fazer uma contagem de quantas vezes um dos sensores hall que ficam dento do motor
 * Além disso, ela calcula a diferença no tempo dos pulsos
 * Esse valor da diferença no tempo dos pulsos é usado para o cálculo da velocidade
 */
void H_motor()
{
  contagem_motor++; // Incremento da quantidade de pulsos
  if (contagem_motor % 2 == 0)
  {
    tempo_anterior1_motor = micros();                                      // Tempo atual
    diferenca_pulsos_motor = tempo_anterior1_motor - tempo_anterior_motor; // Cálculo do tempo da diferença dos pulsos
    conta_motor = HIGH;
  }
  else
  {
    tempo_anterior_motor = micros(); // Atualiza o tempo que será usado como o momento do último pulso quando a função for chamada novamente
  }
}
