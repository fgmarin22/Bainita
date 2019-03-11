/*
 *  Prueba 2 del Grupo Bainita
 */

#include <QTRSensors.h>

#include <Wire.h>
#include <Adafruit_MotorShield.h>


// overflow?? const float maxFloat = 3.4028235E+38;
const float maxFloat = 3.4E+38;

/*
 * QTRSensorsRC se inicializa con los siguientes parametros:
 * 1- Array de caracteres que incluye el orden de los pines de los sensores de izquierda a derecha
 * 2- El número de sensores 
 * 3- El timeout de espera de descarga (máximo valor de oscuridad)
 * 4- El pin utilizado para controlar el dim de la luz led
 */
// Los sensores 0 a 7 se asignan a los pines 3 al 10, respectivamente
const uint8_t numSensors = 8;
QTRSensorsRC qtrrc((unsigned char[]) {10, 9, 8, 7, 6, 5, 4, 3}, numSensors, 2500, 2); 
unsigned int sensorValues[numSensors];
const int centerSens = (1000 * (numSensors - 1))/2;

/*
 *  Inicialización de MotorShield y motores
 */
// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

// Select which 'port' M1, M2, M3 or M4. In this case, M1 and M3
Adafruit_DCMotor *myMotorD = AFMS.getMotor(1);
Adafruit_DCMotor *myMotorI = AFMS.getMotor(3);

const uint8_t dEjes = 11;

const int kPos = -2;
const int kInt = 200;
const int kDer = 200;


uint8_t currentSpeed[2] = {0, 0} ;
uint8_t targetSpeed;
float minGiro;
uint8_t delayLoop;

void setup() {

  Serial.begin(9600);
  
  AFMS.begin();
  myMotorI->run(RELEASE);
  myMotorD->run(RELEASE);
  myMotorI->setSpeed(0);
  myMotorD->setSpeed(0);
  myMotorI->run(FORWARD);
  myMotorD->run(FORWARD);
  currentSpeed[0] = 0;
  currentSpeed[1] = 0;
  
  CalibraQTR(numSensors);

  targetSpeed = 40;
  minGiro = 16.5;
  delayLoop = 10;

}

void loop() {

  // read calibrated sensor values and obtain a measure of the line position from 0 to 7000
  int posSens = qtrrc.readLine(sensorValues);
  int errorSens = posSens - centerSens;

  Serial.println();
  Serial.print("Position / Error : ");
  Serial.print(posSens);
  Serial.print(" / ");
  Serial.println(errorSens);

  float radioGiro = maxFloat;
  if (errorSens != 0) radioGiro = ((minGiro * (float)centerSens) / (float)(kPos * errorSens));

  if (isLine(posSens)) marcha(targetSpeed, radioGiro);
  else marcha(0, maxFloat);

  delay(delayLoop);
  
}

/*
 *  Funcion que calibra los sensores QTR
 */
void CalibraQTR(uint8_t n) {

  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  delay(2000);
  digitalWrite(13, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode

  Serial.begin(9600);

  Serial.println("Empieza calibrado");

  
  for (int i = 0; i < 400; i++)  // make the calibration take about 10 seconds
  {
    qtrrc.calibrate();       // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
/*    
    Serial.println();
    Serial.print("Minimos :");
    for (int i = 0; i < numSensors; i++) {
      Serial.print(' ');
      Serial.print(qtrrc.calibratedMinimumOn[i]);
    }
    Serial.println();
    Serial.print("Maximos :");

    // print the calibration maximum values measured when emitters were on
    for (int i = 0; i < numSensors; i++) {
      Serial.print(' ');
      Serial.print(qtrrc.calibratedMaximumOn[i]);
    }
    Serial.println();
*/ 
  delay(10);   
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

  // print the calibration maximum values measured when emitters were on
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
 *  Funcion que hace marchar hacia delante o havia atrás el bólido
 *  con un radio de giro
 */
void marcha(int speed, float radio) {

  int diferencial2 = (speed * dEjes) / (2 * radio);

  int speedM[2] = {(speed - diferencial2), (speed + diferencial2)};

  Serial.println();
  Serial.print("Radio / Mitad Diferencial : ");
  Serial.print(radio);
  Serial.print(" / ");
  Serial.println(diferencial2);

  if (speedM[0] >= 0) {
    if (currentSpeed[0] < 0) myMotorI->run(FORWARD);
    myMotorI->setSpeed((uint8_t)speedM[0]);
  }
  else {
    if (currentSpeed[0] >= 0) myMotorI->run(BACKWARD);
    myMotorI->setSpeed((uint8_t)(-speedM[0]));    
  }

 currentSpeed[0] = speedM[0]; 

  if (speedM[1] >= 0) {
    if (currentSpeed[1] < 0) myMotorD->run(FORWARD);
    myMotorD->setSpeed((uint8_t)speedM[1]);
  }
  else {
    if (currentSpeed[1] >= 0) myMotorD->run(BACKWARD);
    myMotorD->setSpeed((uint8_t)(-speedM[1]));    
  }

  currentSpeed[1] = speedM[1];
  
  Serial.println();
  Serial.print("Velocidad I   : ");
  Serial.println(speedM[0]);
  Serial.print("Velocidad D   : ");
  Serial.println(speedM[1]);     
}

/*
 *  Funcion que devuelve true si se encuentra línea bajo lops sensores
 */
boolean isLine(unsigned int position) {

if (position == 0 || position == 7000) return (false);
return (true);
     
}
