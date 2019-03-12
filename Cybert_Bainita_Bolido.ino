/*
 *  Prueba 2 del Grupo Bainita
 */

#define DEBUG_CALIBRA

#define BAINITA_BOLIDO_UNO
#ifndef BAINITA_BOLIDO_UNO
  #define BAINITA_BOLIDO_DOS
#endif

#include <QTRSensors.h>

#ifdef BAINITA_BOLIDO_DOS 
  #include <Wire.h>
  #include <Adafruit_MotorShield.h>
#endif

/*
 * QTRSensorsRC se inicializa con los siguientes parametros:
 * 1- Array de caracteres que incluye el orden de los pines de los sensores de izquierda a derecha
 * 2- El número de sensores 
 * 3- El timeout de espera de descarga (máximo valor de oscuridad)
 * 4- El pin utilizado para controlar el dim de la luz led
 */
#ifdef        BAINITA_BOLIDO_UNO
  #define SENSORES_QTR (unsigned char[]) {2, 4, 5, 6, 7, 8, 9, 10}, numSensors, 2500, A0
#elif defined BAINITA_BOLIDO_DOS
  #define SENSORES_QTR (unsigned char[]) {10, 9, 8, 7, 6, 5, 4, 3}, numSensors, 2500, 2
#endif

const uint8_t numSensors = 8;
QTRSensorsRC qtrrc(SENSORES_QTR); 
unsigned int sensorValues[numSensors];
const int centerSens = (1000 * (numSensors - 1))/2;

/*
 *  Inicialización de MotorShield y motores
 */
#ifdef        BAINITA_BOLIDO_UNO
  const uint8_t pwm[2] = {3, 11};
  const uint8_t dir[2] = {12, 13};
#elif defined BAINITA_BOLIDO_DOS
  // Create the motor shield object with the default I2C address
  Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
  /*Adafruit_DCMotor *myMotorD = AFMS.getMotor(1);
  Adafruit_DCMotor *myMotorI = AFMS.getMotor(3);
  */
  Adafruit_DCMotor *myMotor[2] = {AFMS.getMotor(3), AFMS.getMotor(1)};
  
#endif

/*
 *  DEFINICIONES DE CONSTANTES PARA LOS DOS BÓLIDOS
 */

#ifdef        BAINITA_BOLIDO_UNO
  const uint8_t dEjes = 11; //distancia entre las dos ruedas motrices
  const int kPos = -2;      // constante P de PID
  const int kInt = 200;     // constante I de PID
  const int kDer = 200;     // constante D de PID
  const uint8_t targetSpeed = 40;
  const float minGiro = 16.5;
  const uint8_t delayLoop = 10;
#elif defined BAINITA_BOLIDO_DOS
  const uint8_t dEjes = 11; //distancia entre las dos ruedas motrices
  const int kPos = -2;      // constante P de PID
  const int kInt = 200;     // constante I de PID
  const int kDer = 200;     // constante D de PID
  const uint8_t targetSpeed = 40;
  const float minGiro = 16.5;
  const uint8_t delayLoop = 10;
#endif

int currentSpeed[2] = {0, 0} ;
// overflow?? const float maxFloat = 3.4028235E+38;
const float maxFloat = 3.4E+38;

void setup() {

  Serial.begin(9600);

  #ifdef        BAINITA_BOLIDO_UNO 
    pinMode(dir[0], OUTPUT);
    pinMode(dir[1], OUTPUT);
    analogWrite(pwm[0], 0);        // Velocidad 0 en motor izquierdo
    analogWrite(pwm[1], 0);        // Velocidad 0 en motor derecho
    digitalWrite(dir[0] , HIGH);  // Motor izquierdo adelante
    digitalWrite(dir[1] , HIGH);  // Motor derecho adelante 
  #elif defined BAINITA_BOLIDO_DOS
    AFMS.begin();
    for (int i = 0; i < 2; i++) {
      myMotor[i]->run(RELEASE);       // Parada
      myMotor[i]->setSpeed(0);        // Velocidad 0
      myMotor[i]->run(FORWARD);       // Empiezo adelante con velocidad 0
    }
  #endif
 
  CalibraQTR();
  
}

void loop() {

  // read calibrated sensor values and obtain a measure of the line position from 0 to 7000
  int posSens = qtrrc.readLine(sensorValues);
  int errorSens = posSens - centerSens;

  // print the sensor values as numbers from 0 to 1000, where 0 means maximum reflectance and
  // 1000 means minimum reflectance, followed by the line position
  for (unsigned char i = 0; i < numSensors; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println();

  
  Serial.print("Position / Error : ");
  Serial.print(posSens);
  Serial.print(" / ");
  Serial.println(errorSens);

  float radioGiro;
  if (errorSens != 0) radioGiro = ((minGiro * (float)centerSens) / (float)(kPos * errorSens));
  else radioGiro = maxFloat;

  if (isLine(posSens)) marcha(targetSpeed, radioGiro);
  else marcha(0, maxFloat);

  delay(delayLoop);
  
}

/*
 *  Funcion que calibra los sensores QTR
 */
void CalibraQTR() {

  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  delay(2000);
  digitalWrite(13, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode

  Serial.println("Empieza calibrado");
  
  for (int i = 0; i < 400; i++)  // make the calibration take about 10 seconds
  {
    qtrrc.calibrate();       // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)

    #ifdef DEBUG_CALIBRA
      Serial.println();
      Serial.print("Minimos :");
      for (int i = 0; i < numSensors; i++) {
        Serial.print(' ');
        Serial.print(qtrrc.calibratedMinimumOn[i]);
      }
      Serial.println();
      Serial.print("Maximos :");
      for (int i = 0; i < numSensors; i++) {
        Serial.print(' ');
        Serial.print(qtrrc.calibratedMaximumOn[i]);
      }
      Serial.println();
    #else 
      delay(10);
    #endif   
  }
  digitalWrite(13, LOW);     // turn off Arduino's LED to indicate we are through with calibratio

  // print the calibration minimum values measured when emitters were on
  Serial.println();
  Serial.print("Minimos final:");
  for (int i = 0; i < numSensors; i++)
  {
    Serial.print(' ');
    Serial.print(qtrrc.calibratedMinimumOn[i]);
  }
  Serial.println();
  
  Serial.println();
  Serial.print("Maximos final:");
  for (int i = 0; i < numSensors; i++)
  {
    Serial.print(' ');
    Serial.print(qtrrc.calibratedMaximumOn[i]);
  }
  Serial.println();
  Serial.println();
  Serial.println("Termina calibrado");
  Serial.println();

}

/*
 *  Funcion que hace marchar hacia delante o hacia atrás el bólido
 *  con un radio de giro
 */
void marcha(int speed, float radio) {

  int diferencial2 = (float)(speed * dEjes) / (float)(2 * radio);
  int speedM[2] = {(speed - diferencial2), (speed + diferencial2)};

  for (int i = 0; i < 2; i++) {
    if (speedM[i] >= 0) {
      if (currentSpeed[i] < 0)
      #ifdef        BAINITA_BOLIDO_UNO 
          digitalWrite(dir[i], HIGH);                     //adelante
        analogWrite (pwm[i], (uint8_t)speedM[i]);         //pongo la velocidad
      #elif defined BAINITA_BOLIDO_DOS
          myMotor[i]->run(FORWARD);                        //adelante
        myMotor[i]->setSpeed((uint8_t)speedM[i]);          //pongo la velocidad
      #endif   
    }
    else {
      if (currentSpeed[i] >= 0)
      #ifdef        BAINITA_BOLIDO_UNO 
          digitalWrite(dir[i], LOW);                      //atras
        analogWrite (pwm[i], (uint8_t)(-speedM[i]));      //pongo la velocidad
      #elif defined BAINITA_BOLIDO_DOS
          myMotor[i]->run(BACKWARD);                       //atras
        myMotor[i]->setSpeed((uint8_t)(-speedM[i]));       //pongo la velocidad
      #endif       
    }
    currentSpeed[i] = speedM[i];
  }  

    
  Serial.println();
  Serial.print("Radio / Mitad Diferencial : ");
  Serial.print(radio);
  Serial.print(" / ");
  Serial.println(diferencial2);
  Serial.println();
  Serial.print("Velocidad I   : ");
  Serial.println(speedM[0]);
  Serial.print("Velocidad D   : ");
  Serial.println(speedM[1]);     
}

/*
 *  Funcion que devuelve true si se encuentra línea bajo lops sensores
 */
boolean isLine(int position) {

  if (position == 0 || position == 7000) return (false);
  return (true);
     
}
