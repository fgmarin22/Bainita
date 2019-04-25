/*
 *  PROGRAMA DE CONTROL DEL 
 *  ROBOT BAINITA
 */
/**************************************** 
 *  DEFINICIONES
 */
//#define BAINITA_BOLIDO_UNO
#define BAINITA_BOLIDO_DOS

//#define PRUEBA_SIGUELINEAS

//#define PRUEBA_ADELANTAMIENTO
  #if defined(PRUEBA_ADELANTAMIENTO)
    #define PRUEBA_ADELANTA_CARRIL -1               // CARRIL DE SALIDA: -1 CARRIL IZQUIERDO, +1 CARRIL DERECHO, SIN DEFINIR NO ADELANTA
  #endif

#define PRUEBA_LABERINTO

#define LAB_ALGORITMO 1                             // 0 = Laberinto conocido, +1- Regla derecha, -1- Regla izquierda
#define LAB_ALGO_CON_MAN 4,4,4                      // Definimos valores para array que describe el laberinto conocido
#define LAB_ALGO_CON_SEN 1,1,-1

#define PRUEBA_RASTRERINTO

//#define CALIBRA_MANUAL

#define VELO_PORCENT_LINEA 90                             //Porcentaje de la velocidad máxima a la que queremos ir                  

#define VELO_PORCENT_LABER 50                              //Porcentaje de la velocidad máxima a la que queremos ir                  
#define VELO_PORCENT_GIRO  40                             //Porcentaje de la velocidad máxima a la que queremos ir                  

//*****************************************
/*
 * Definiciones de Debugging y LOG
 * 
 */

#if defined(BAINITA_BOLIDO_DOS) 
  #define LOGGER_SHIELD
#endif


//#define SERIAL_CONN

#define DEBUG_DISTANCIA
#define DEBUG_LOOP
//#define DEBUG_CALIBRA

#if (defined(LOGGER_SHIELD) || defined(SERIAL_CONN))
  #define IFDEF_LOG_SERIAL(X) X
#else
  #define IFDEF_LOG_SERIAL(X)
#endif

#define LOG_WRT(D1, D2, D3, D4, D5, D6, D7, D8, D9, D10) IFDEF_LOG_SERIAL(logWrt(D1, D2, D3, D4, D5, D6, D7, D8, D9, D10))

/*
 * 
 * Definiciodes de CALIBRADO
 * 
 */
#ifndef CALIBRA_MANUAL

  #define CALIBRA_STORED_CONCURSO_REAL

  //#define CALIBRA_STORED_TELLI_COCINA
  //#define CALIBRA_STORED_PARQUET
  //#define CALIBRA_STORED_PAPEL
  //#define CALIBRA_STORED_TERRAZA
  //#define CALIBRA_STORED_CIRCUITO_PRUEBA
  //#define CALIBRA_STORED_INDU_GIMNASIO


#endif

#if (defined(CALIBRA_MANUAL) || defined(CALIBRA_MENEITO))              // Modos de calibrado ejecutados, no precargados
  #define CALIBRA_EXEC 
#endif

/*
 *  Definiciones de las luces LED
 *  
 *  Al estar el GREEN y el BLUE en los pines 0 y 1, que también
 *  utiliza la conexión serie, en caso de que el serial esté activo
 *  todo va al RED
 */
#if defined(BAINITA_BOLIDO_DOS)
  #define IFDEF_BAINITA_BOLIDO_DOS(X) X
#else
  #define IFDEF_BAINITA_BOLIDO_DOS(X)
#endif

#define RED 17
#define LED_ON_RED IFDEF_BAINITA_BOLIDO_DOS(digitalWrite(17, LOW))
#define LED_OFF_RED IFDEF_BAINITA_BOLIDO_DOS(digitalWrite(17, HIGH))
#if (!defined(SERIAL_CONN))
  #define GREEN 1
  #define BLUE 0
  #define LED_ON_GREEN IFDEF_BAINITA_BOLIDO_DOS(digitalWrite(1, LOW))
  #define LED_OFF_GREEN IFDEF_BAINITA_BOLIDO_DOS(digitalWrite(1, HIGH))
  #define LED_ON_BLUE IFDEF_BAINITA_BOLIDO_DOS(digitalWrite(0, LOW))
  #define LED_OFF_BLUE IFDEF_BAINITA_BOLIDO_DOS(digitalWrite(0, HIGH))
#else
  #define GREEN 17
  #define BLUE 17
  #define LED_ON_GREEN IFDEF_BAINITA_BOLIDO_DOS(digitalWrite(17, LOW))
  #define LED_OFF_GREEN IFDEF_BAINITA_BOLIDO_DOS(digitalWrite(17, HIGH))
  #define LED_ON_BLUE IFDEF_BAINITA_BOLIDO_DOS(digitalWrite(17, LOW))
  #define LED_OFF_BLUE IFDEF_BAINITA_BOLIDO_DOS(digitalWrite(17, HIGH))
#endif

#define PARAR targetSpeed= 0; difVelo = 0; difVeloP = 0; difVeloD = 0; marcha(0,0)


/*
 * 
 * INCLUDES
 * 
 */
#include <QTRSensors.h>
#include <limits.h>

#if defined(BAINITA_BOLIDO_DOS) 
  #include <Wire.h>
  #include <Adafruit_MotorShield.h>
  #if defined(LOGGER_SHIELD)  
    #include <SPI.h>
    #include <SD.h>
    #include "RTClib.h"  
  #endif
#endif


/*
 * QTRSensorsRC Inicializo el array de sensores de seguimiento de línea
 * Se inicializa con los siguientes parametros:
 * 1- Array de caracteres que incluye el orden de los pines de los sensores de izquierda a derecha
 * 2- El número de sensores 
 * 3- El timeout de espera de descarga (máximo valor de oscuridad)
 * 4- El pin utilizado para controlar el dim de la luz led
 */
#define QTR_NUM 8 

// Establezco el valor de la posición central del array de sensores de línea 
#define QTR_CENTRO (1000 * (QTR_NUM - 1))/2

#if defined(BAINITA_BOLIDO_UNO)
  #define QTR (unsigned char[]) {10, 9, 8, 7, 6, 5, 4, 2}, QTR_NUM, 1200, A0
#elif defined(BAINITA_BOLIDO_DOS)
  #define QTR (unsigned char[]) {9, 8, 7, 6, 5, 4, 3, 2}, QTR_NUM, 1200
#endif
QTRSensorsRC qtrrc(QTR);

/*
 *  
 *  MOTORES 
 *  
 *  Bólido uno (Arduino Motorshield)
 *  Bólido 2 (Adafruit Motor Shield V2)
 *  
 */
#if defined(BAINITA_BOLIDO_UNO)
  // Defino dos arrays que definen los pines de potenci(pwm) y dirección(dir) para los dos motores, izquierdo y derecho
  const uint8_t pwm[2] = {11, 3};
  const uint8_t dir[2] = {13, 12};
  
#elif defined(BAINITA_BOLIDO_DOS)
  // Create the motor shield object with the default I2C address
  Adafruit_MotorShield AFMS = Adafruit_MotorShield();
  // Creo los objetos DCMotor en un array, 0 el izquierdo y 1 el derecho 
  Adafruit_DCMotor *myMotor[2] = {AFMS.getMotor(3), AFMS.getMotor(1)}; 
#endif

/*
 *  DEFINICIONES DE CONSTANTES PARA LOS DOS BÓLIDOS
 */

#if defined(BAINITA_BOLIDO_UNO)
  #define K_POS 20                                                 // constante P de PID (Todos los uint8_t tienen que ser positivos)
  #define K_INT 10                                                 // constante I de PID (Todos los uint8_t tienen que ser positivos)
  #define K_DER 10                                                 // constante D de PID (Todos los uint8_t tienen que ser positivos)
  #define TARGET_SPEED_LINEA     (255 * VELO_PORCENT_LINEA)/100    // Máximo 255
  #define TARGET_SPEED_LABER     (255 * VELO_PORCENT_LABER)/100    // Máximo 255
  #define TARGET_SPEED_GIRO      (255 * VELO_PORCENT_GIRO)/50     // Máximo 255
  #define TARGET_SPEED_ADELANTA  (255 * VELO_PORCENT_LINEA)/100    // Máximo 255
#elif defined(BAINITA_BOLIDO_DOS) 
  #define K_POS 10                                                 // constante P de PID (Todos los uint8_t tienen que ser positivos)
  #define K_INT 10                                                 // constante I de PID (Todos los uint8_t tienen que ser positivos)
  #define K_DER 10                                                 // constante D de PID (Todos los uint8_t tienen que ser positivos)
  #define TARGET_SPEED_LINEA     (255 * VELO_PORCENT_LINEA)/100    // Máximo 255
  #define TARGET_SPEED_LABER     (255 * VELO_PORCENT_LABER)/100    // Máximo 255
  #define TARGET_SPEED_GIRO      (255 * VELO_PORCENT_GIRO)/100     // Máximo 255
  #define TARGET_SPEED_ADELANTA  (255 * VELO_PORCENT_LINEA)/100    // Máximo 255
#endif
#define QTR_ANCHO_DESVIO 1300                                   // Ancho de linea a partir del cual entendemos MARCA lateral en la linea
#define QTR_ANCHO_ANGULO 1000                                   // A veces el angulo recto solo pilla 1200. Ancho de linnea para considerar angulo recto
#define QTR_DERRAPE_LIMITE 2000                                 // Si sale de la línea y la última posición mayor interpretamos derrape

#define ANCHO_L_DESVIO 25                                                            // Distancia en cm en que se mantiene indicación de DESVIO 
#define ANCHO_T_DESVIO (255 * 1000 * ANCHO_L_DESVIO) / /70 * TARGET_SPEED_LINEA)     // Tiempo en mseg en que se mantiene indicación de DESVIO 
#define ANCHO_L_MARCA 7                                                              // Distancia en cm de la MARCA de DESVIO 
#define ANCHO_T_MARCA (255 * 1000 * ANCHO_L_MARCA) / (70 * TARGET_SPEED_LINEA)       // Tiempo en mseg de la MARCA de DESVIO 
#define ANCHO_L_ANGULO 4                                                             // Distancia en cm en que se mantiene indicación de ANGULO 
#define ANCHO_T_ANGULO (255 * 1000 * ANCHO_L_ANGULO) / (70 * TARGET_SPEED_LINEA)     // Tiempo en mseg en que se mantiene indicación de ANGULO 

#define VELO_REDUCTORA 70                       // Porcentaje de la velocidad normal a aplicar en situacion reductora

unsigned int velo_reductora = 100;              // Inicialmente, sin reduccion, al 100%

const unsigned int target_speed_linea = ((255 * VELO_PORCENT_LINEA)/100);
const unsigned int ancho_t_desvio = ((255L * 1000 * ANCHO_L_DESVIO) / (70 * target_speed_linea));
const unsigned int ancho_t_marca = ((255L * 1000 * ANCHO_L_MARCA) / (70 * target_speed_linea));
const unsigned int ancho_t_angulo = ((255L * 1000 * ANCHO_L_ANGULO) / (70 * target_speed_linea));

int8_t sentidoDesvio = 0;                                    // Sentido de marca-desvio +1 derecha -1 izquierda
int8_t sentidoAngulo = 0;                                    // Sentido del angulo recto +1 derecha -1 izquierda
                                   
unsigned long tiempoAngulo = 0;                              // Tiempo desde que se detectó anchura
#define VALLE_CONSECUTIVO_NUMERO 2
#define VALLE_CONSECUTIVO_VALOR  -700

/*
 * LABERINTO
 * 
 * SENSORES SHARP PROXIMIDAD, DISTANCIAS
 * 
 */

int8_t labAlgoritmo = LAB_ALGORITMO;

uint8_t labAlgoConNum = 0;

uint8_t labAlgoConMan[] = {LAB_ALGO_CON_MAN};
uint8_t labAlgoConSen[] = {LAB_ALGO_CON_MAN};

 
// Definiciones para los sensores de distancia SHARP

#define SHARP_PIN_I A0
#define SHARP_PIN_F A1
#define SHARP_PIN_D A2
#define SHARP_TIEMPO_RUIDO 20                     // Milisegundos que se tiene que mantener una detección de obstáculo para evitar ruido

  #define SHARP_41_MIN_D 40                          // Distancia mínima fiable en mm
  #define SHARP_41_MAX_D 300                         // Distancia máxima fiable en mm
  #define SHARP_41_FAC 20760                         // Factor multiplicador para calcular distancia en mm
  #define SHARP_41_SUB 11                            // Valor a sustraer a lectura para calcular distancia

  #define SHARP_21_MIN_D 80                          // Distancia mínima fiable en mm
  #define SHARP_21_MAX_D 800                         // Distancia máxima fiable en mm
  #define SHARP_21_FAC 48000                         // Factor multiplicador para calcular distancia en mm
  #define SHARP_21_SUB 20                            // Valor a sustraer a lectura para calcular distancia


#define ANCHO_LAB_PASILLO    300                              // Ancho del pasillo del laberinto
#define DIST_PARED_TARG      130                              // Distancia objetivo a la pared
#define DIST_PARED_HUECO_LATERAL     240  // POR ENCIMA DE 260 NO RECONOCE SEGUNDO GIRO EN LA L Distancia mínima para considerar un hueco lateral
#define DIST_PARED_HUECO_FRONTAL     220                              // Distancia mínima para considerar un hueco lateral
#define DIST_PARED_GIRO_COMPLETO  (DIST_PARED_TARG * 12)/10   // En giro, distancia de la pared a la que termina el giro y avanza

#define DIST_PARED_DESPEJADO_LATERAL (DIST_PARED_HUECO_LATERAL * 11)/10        // Distancia a la que consideramos que hay espacio delante
#define DIST_PARED_DESPEJADO_FRONTAL (DIST_PARED_HUECO_FRONTAL * 11)/10        // Distancia a la que consideramos que hay espacio delante

#define DIST_FRONT_STOP      140                               // Distancia mínima de un objeto frontal que hará parar al bólido

#define SHARP_41_LECT_MIN (SHARP_41_FAC / SHARP_41_MAX_D) + SHARP_41_SUB - 1
#define SHARP_41_LECT_MAX (SHARP_41_FAC / SHARP_41_MIN_D) + SHARP_41_SUB + 10
#define SHARP_21_LECT_MIN (SHARP_21_FAC / SHARP_21_MAX_D) + SHARP_21_SUB - 1
#define SHARP_21_LECT_MAX (SHARP_21_FAC / SHARP_21_MIN_D) + SHARP_21_SUB + 10

// Defino variables globales donde guardar las distancias ( 0- IZQUIERDA, 1 - FRONTAL, 2- DERECHA)

int            sharpDist[3]   = {0, 0, 0};
const uint8_t  sharpModel[3]  = {41, 41, 41};
const uint8_t  sharpMin[3]    = {SHARP_41_MIN_D, SHARP_41_MIN_D, SHARP_41_MIN_D};
const uint8_t  sharpMax[3]    = {SHARP_41_MAX_D, SHARP_41_MAX_D, SHARP_41_MAX_D};
const uint16_t sharpFac[3]    = {SHARP_41_FAC, SHARP_41_FAC, SHARP_41_FAC};
const uint8_t  sharpSub[3]    = {SHARP_41_SUB, SHARP_41_SUB, SHARP_41_SUB};

bool stopDist;

#define GIRO_MAX_MILISEC  4000
#define GIRO_REPOS 100                 // Posicion sobre la línea para dejar de girar al recolocarse

#if defined(LOGGER_SHIELD) 
  #define CHIP_SELECT 10
  File dataFile;
  bool loggerConnect = false; 
#endif

#if defined(SERIAL_CONN)
  bool serialConnect = false;
#endif

char buffer[150];                                     // buffer intermedio para formatear líneas para Serial

uint8_t programa;
uint8_t maniobra = 0;

// Variables para almacenar velocidades de los dos motores

int currentSpeed[2] = {0, 0} ;                       // Puede ser negativa
uint8_t targetSpeed = 0;
int difVeloP;                                        // Puede ser negativa
int difVeloD;                                        // Puede ser negativa
int difVelo;                                         // Puede ser negativa

int posicion[3] = {0, SHRT_MAX, SHRT_MAX};

/*
 * 
 * 
 *      SETUP  SETUP  SETUP  SETUP  SETUP  SETUP  SETUP  SETUP  SETUP  SETUP  SETUP  SETUP  SETUP
 * 
 * 
 * 
 */

void setup() {

#if defined(SERIAL_CONN)
  Serial.begin(9600);
  delay(100);
  if (Serial) serialConnect = true;
#endif

#if defined(LOGGER_SHIELD) 
  RTC_DS1307 rtc;
  rtc.begin();
  DateTime now = rtc.now();
  
  if (SD.begin(CHIP_SELECT)) {
    loggerConnect = true;
    sprintf_P(buffer, PSTR("/datoslog/%4d%02d%02d"), now.year(), now.month(), now.day()); 
    SD.mkdir(buffer);
    sprintf_P(buffer, PSTR("%s/%02d%02d%02d.csv"), buffer, now.hour(), now.minute(), now.second());
    dataFile = SD.open(buffer, FILE_WRITE);
    LOG_WRT(0, 0, now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second(), 0, 0); 
     
  }
  #if defined(SERIAL_CONN)
    else {
      if (serialConnect) Serial.println("** Error al inicializar el Data Logger");    
    }
  #endif
#endif 

// Inicio los motores hacia delante, velocidad 0
  #if defined(BAINITA_BOLIDO_UNO) 
    for (uint8_t i = 0; i < 2; i++) {
      pinMode(dir[i], OUTPUT);
      analogWrite(pwm[i], 0);        // Velocidad 0
      digitalWrite(dir[i] , HIGH);   // Empiezo adelante con velocidad 0
    }
  #elif defined(BAINITA_BOLIDO_DOS)
    AFMS.begin();
    for (uint8_t i = 0; i < 2; i++) {
      myMotor[i]->run(RELEASE);       // Parada
      myMotor[i]->setSpeed(0);        // Velocidad 0
      myMotor[i]->run(FORWARD);       // Empiezo adelante con velocidad 0
    }
  #endif

  #if defined(BAINITA_BOLIDO_DOS)
  pinMode(RED, OUTPUT);
  LED_ON_RED;
  LED_OFF_RED;
  #if (!defined(SERIAL_CONN))
    pinMode(GREEN, OUTPUT);  
    LED_ON_GREEN;
    LED_OFF_GREEN;
    pinMode(BLUE, OUTPUT);
    LED_ON_BLUE;
    LED_OFF_BLUE;
  #endif
#endif

  #if defined(PRUEBA_SIGUELINEAS) || defined(PRUEBA_ADELANTAMIENTO) || defined(PRUEBA_RASTRERINTO)
    programa = 1;
    // Calibro los sensores de línea
    if (programa == 1) CalibraQTR();
  #elif defined(PRUEBA_LABERINTO)
    programa = 2;
  #endif
  
}

/*
 * 
 * 
 *      LOOP  LOOP  LOOP  LOOP  LOOP  LOOP  LOOP  LOOP  LOOP  LOOP  LOOP  LOOP  LOOP  LOOP  LOOP  LOOP
 * 
 * 
 * 
 */

void loop() {

  /*
  * 
  *   COMUN A TODOS LOS PROGRAMAS Y OPERACIONES
  * 
  * 
  */
  #if defined(SERIAL_CONN)
    if (Serial) serialConnect = true; else serialConnect = false;
  #endif

  static unsigned long loopCnt = 0;                                         // Contador de pasadas por el loop
  #if defined(LOGGER_SHIELD)
    static unsigned long loopFlush = 0;                                     // Guardo el último loop en que se hizo flush al log
    #define LOOP_FLUSH 50
  #endif

  loopCnt = loopCnt + 1;                                                    // Incremento el contador de vueltas del loop

  #if (defined(SERIAL_CONN) && (defined(DEBUG_LOOP) || defined(DEBUG_DISTANCIA)))
      if (serialConnect) Serial.println();                                  // Separación de línea en cada loop   
  #endif
        
  #if defined(LOGGER_SHIELD)
    if (loggerConnect && ((targetSpeed == 0) || ((loopCnt-loopFlush)%LOOP_FLUSH == 0))) {
        dataFile.flush();
        LOG_WRT(91, loopCnt, loopFlush, targetSpeed, 0, 0, 0, 0, 0, 0);
        loopFlush = loopCnt;
    }
  #endif
  
  unsigned int sensorValues[QTR_NUM];                              // Array con los valores leídos en los sensores de línea  
  int anchuraLinea;
  bool negro;
  uint8_t tipoCruce = 0;
  #if defined(PRUEBA_ADELANTAMIENTO)
    static int8_t carrilAdelanta = PRUEBA_ADELANTA_CARRIL ;
  #endif
  static bool estoyAdelantando = false;
  bool giroRealizado = false; 
  static long tiempoCurva;
  uint8_t reenganche = 0;
  

  #if (defined(LOGGER_SHIELD) && !defined(SERIAL_CONN))
    static unsigned long loopMillis = 0;
    if ((millis()-loopMillis) > 20) {
      LOG_WRT(92, loopCnt, (millis()-loopMillis), loopMillis, 0, 0, 0, 0, 0, 0);
    }
    loopMillis = millis();
  #endif

  
  /*
  * DISTANCIA FRONTAL. Controlo si hay algún objeto delante. Solamente cada cierto número de repeticiones
  * En caso de que cada loop dure 5 ms, repitiendo cada 100 haría ese control cada 0,5 segundos
  */
    
  stopDist = DistanciaFrontalMide();


  // programa, maniobra, stopDist, sharpDist[1] 
  LOG_WRT(1, loopCnt, programa, maniobra, stopDist, sharpDist[1], posicion[0], targetSpeed, sentidoDesvio, sentidoAngulo);


  switch (programa) {

    /*
    *  PROGRAMA DE SEGUIMIENTO DE LÍNEA
    *  
    *  Actuo sobre el motor izquierdo y derecho, para que su diferencia de velocidad corrija
    *  el rumbo, adaptandose a los desplazamientos de la linea en los sensores
    *  
    *  La longitud del array de sensores es de unos 80 mm
    *  
    */     
    case 1:                                               // Rastreador de linea            
      switch (maniobra) {

        // Decision inicial
        case 0:
          anchuraLinea = 0;
          if (!stopDist) maniobra = 1;
          LOG_WRT(2, loopCnt, stopDist, sharpDist[1], maniobra, 0, 0, 0, 0, 0); 
        break;
        
        // Avanzar
        case 1:                                           // Avanzar

         if (stopDist) {                                     // Si lleva un tiempo detectando obstáculo
//       if (sharpDist[1] < 305) {
            maniobra = 2;
          } 
          else {
                 
            *posicion = addPosicion(posicion, sizeof(posicion), (qtrrc.readLine(sensorValues) - QTR_CENTRO));
       
            anchuraLinea = isLine(posicion[0], sensorValues);     

            if (sentidoDesvio == 0) {
               LED_OFF_RED; 
               LED_OFF_BLUE;
             }
            else if (sentidoDesvio == 1) LED_ON_BLUE;
            else LED_ON_RED;           
  
            if (anchuraLinea >= 0) {     // En esta caso, el bólido está sobre la línea

              if (sentidoDesvio == 0) velo_reductora = 100;
              else velo_reductora = VELO_REDUCTORA;

              difVeloCalculo(posicion, QTR_CENTRO, targetSpeed, ((TARGET_SPEED_LINEA * velo_reductora)/100));

              targetSpeed = ((TARGET_SPEED_LINEA * velo_reductora)/100);                    // A correr                                              
            
              marcha(targetSpeed, difVelo); 
            }
            else { PARAR; maniobra = 3;}
          }
          // Posicion, targetspeed, difVelo, difVeloP, difVeloD, anchuraLinea, sentidoDesvio 
          LOG_WRT(12, loopCnt, stopDist, posicion[0], targetSpeed, difVelo, difVeloP, difVeloD, anchuraLinea, sentidoDesvio);
        break;
        
        // ADELANTAMIENTO O PARADA  (La distancia entre carriles es de unos 40 cm)       

        
        case 2:                                       // Adelantamientos si prueba obstáculos          

          #ifdef PRUEBA_ADELANTAMIENTO 
            carrilAdelanta = PRUEBA_ADELANTA_CARRIL;
 
            *posicion = addPosicion(posicion, sizeof(posicion), (qtrrc.readLine(sensorValues) - QTR_CENTRO));
       
            negro = false;
            for(uint8_t i = 0; i < QTR_NUM; i++) {
              if (sensorValues[i] > 400) negro = true; 
            }
        
            
            if (abs(posicion[0]) >= QTR_CENTRO) estoyAdelantando = true;
         
            if  (estoyAdelantando && negro &&
              (((carrilAdelanta == 1)  && (posicion[0] > GIRO_REPOS) && (posicion[0] < QTR_CENTRO))  ||
               ((carrilAdelanta == -1) && (posicion[0] < -GIRO_REPOS) && (posicion[0] > -QTR_CENTRO)))  ) {   //Llego al otro carril                                    
          
              maniobra = 1;
              reenganche = 30;
              targetSpeed = (targetSpeed * 80) / 100;      
              carrilAdelanta = -carrilAdelanta;
              estoyAdelantando = false;
              giroRealizado = false;
            }
            else if  (estoyAdelantando && negro &&
              (abs(posicion[0]) >  QTR_CENTRO) ) {        //Llego al otro carril                                                                                
 
              difVeloCalculoDif(posicion, QTR_CENTRO, targetSpeed, ((targetSpeed * 80) / 100));
              targetSpeed = (targetSpeed * 80) / 100;                                            
              marcha(targetSpeed, difVelo); 

              }
            else if (sharpDist[1] < 305  && !giroRealizado) { 
                  tiempoCurva = millis();
              targetSpeed = 60;        
              difVelo = ((targetSpeed) * -(carrilAdelanta)); difVeloP = 0; difVeloD = 0;
              marcha(targetSpeed, difVelo);
              delay(700);                         // Para probar, a ver si este giro va bien para salvar el obstaculo
              giroRealizado = true;
            }
            else {
              targetSpeed = ((TARGET_SPEED_LINEA * 50) / 100);
    
              if ((millis() - tiempoCurva) < 1200) {
                difVelo = carrilAdelanta * ((targetSpeed * 20)/100); difVeloP = 0; difVeloD = 0;
              }
              else {
                tiempoCurva = 0;
                difVelo = 0;
              }
              marcha(targetSpeed, difVelo);
                        
            }

            if (abs(posicion[0]) >= QTR_CENTRO) estoyAdelantando = true;

            // Posicion, targetspeed, difVelo, difVeloP, difVeloD, carrilAdelanta, estoyAdelantando        
            LOG_WRT(22, loopCnt, stopDist, posicion[0], targetSpeed, difVelo, difVeloP, difVeloD, carrilAdelanta, estoyAdelantando);
          
          #else
            PARAR;
            anchuraLinea = 0;                    
            if  (!stopDist) maniobra = 1;                                        // Vuelvo a la marcha normal            
            // Posicion, targetspeed, difVelo, difVeloP, difVeloD, carrilAdelanta, estoyAdelantando        
            LOG_WRT(22, loopCnt, stopDist, posicion[0], targetSpeed, difVelo, difVeloP, difVeloD, 0, 0);
          #endif 
        break;
            
        // Fuera de la línea
        case 3:         // Fuera de la línea
        // RETORNO A LA LÍNEA POR ANGULO RECTO O DERRAPE o PARADA POR SALIDA DE LÍNEA
                     
          *posicion = addPosicion(posicion, sizeof(posicion), (qtrrc.readLine(sensorValues) - QTR_CENTRO));

          // Compruebo si realmente hay línea
          negro = false;
            for(uint8_t i = 0; i < QTR_NUM; i++) {
            if (sensorValues[i] > 400) negro = true; 
          }
          
           if (0) {}
           #if !defined(PRUEBA_ADELANTAMIENTO)
          
             //Salida por ANGULO RECTO  
             else if ((sentidoAngulo != 0) && (tiempoAngulo < ancho_t_angulo) &&
               ((abs(posicion[0]) >= GIRO_REPOS) || !negro )) {                 
     
               difVelo = (TARGET_SPEED_GIRO * sentidoDesvio); difVeloP = 0; difVeloD = 0;
               targetSpeed = (TARGET_SPEED_GIRO);
               marcha(targetSpeed, difVelo);
             }
           #endif
          
           //Salida DE PISTA POR DERRAPE
           else if ((sentidoDesvio == 0) && ((abs(posicion[1]) > QTR_DERRAPE_LIMITE) &&  (abs(posicion[1]) < QTR_CENTRO)) &&           // Salida en curva 
             ( (abs(posicion[0]) >= GIRO_REPOS) || !negro ) ) {
               
             difVelo = (TARGET_SPEED_GIRO * (posicion[1]/abs(posicion[1]))); difVeloP = 0; difVeloD = 0;
             targetSpeed = (TARGET_SPEED_GIRO);
             marcha(targetSpeed, difVelo);             
           }  
           else {         
             // FINAL DE PISTA, Paro y vuelvo a marcha por si aparece línea
             PARAR;
             sentidoDesvio = 0;
             sentidoAngulo = 0;
             velo_reductora = 100;
             maniobra = 1;
           } 

           // Posicion, targetspeed, difVelo , anchuraLinea, sentidoDesvio, sentidoAngulo, tiempoAngulo 

           #if defined(PRUEBA_RASTRERINTO) 
             DistanciaLateralMide();     
             tipoCruce = isParedes();              
             if (tipoCruce == 3) {
               programa = 2;
               maniobra = 0; 
             }      
           #endif         
                  
           LOG_WRT(32, loopCnt, stopDist, posicion[0], targetSpeed, difVelo, anchuraLinea, sentidoDesvio, sentidoAngulo, tiempoAngulo);                 
         break;       
       }

      
       LOG_WRT(13, loopCnt, sensorValues[0], sensorValues[1], sensorValues[2], sensorValues[3],
                            sensorValues[4], sensorValues[5], sensorValues[6], sensorValues[7]);

       
    break;
    
    case 2:                                              // Laberinto
    /*
    *  PROGRAMA DEL LABERINTO
    *  
    *  Actuo sobre las velocidades del motor izquierdo y derecho, para que su diferencia corrija el rumbo
    *  manteniendo una distancia determinada a las paredes
    *  
    */

      // Tomo distancias izquierda y derecha 

      DistanciaLateralMide(); tipoCruce = isParedes();

      uint8_t maniobraActual = maniobra;
    
      switch (maniobra) {
      /*
      * maniobra:
      * 0- Toma de decisión
      * 1- Avanzar por el centro del pasillo
      * 2- Avanzar manteniendo la distancia a pared lateral
      * 
      * 4- Girar en desvio lateral hasta tener la pared lateral a distancia razonable para avanzar
      *
      * 6- Girar 180 grados
      * 7- Parar
      */

        // Decision de maniobra siguiente a realizar 
        case 0:
          #if defined(LOGGER_SHIELD)
            dataFile.flush();
            LOG_WRT(91, loopCnt, loopFlush, targetSpeed, 0, 0, 0, 0, 0, 0);
            loopFlush = loopCnt;
          #endif

          switch (tipoCruce) {
      
         /* 0- Perdido. Libre delante, libre a los dos lados.
         *  1- Desvío derecha. Libre delante, libre derecha, pared izquierda 
         *  2- Desvío izquierda. Libre delante, libre izquierda, pared derecha
         *  3- Pasillo. Libre delante, paredes a los dos lados   
         *  4- T. Pared delante, libre a los dos lados
         *  5- L derecha. Pared delante, libre solo a la derecha
         *  6- L izquierda. Pared delante, libre solo a la izquierda
         *  7- Fondo. Pared delante, paredes a los dos lados
         */

            case 0: PARAR; maniobra = 0; break;                     // Perdido. Seguir adelante          
 
            case 1:                                                 // Desvio derecha. Decision giro derecha o adelante. 

              if (labAlgoritmo == 0)   {                            // Laberinto Conocido 
                maniobra = labAlgoConMan[labAlgoConNum];
                labAlgoConNum++;
              }                                 
              else if (labAlgoritmo == 1) { maniobra = 4; sentidoDesvio = +1;}   // Regla derecha, giro
              else                       { maniobra = 2; sentidoDesvio = -1;}   // Regla izquierda, voy adelante
            break;
            
            case 2:                                                 // Desvio izquierda. Decision giro izquierda o adelante. 
 
              if (labAlgoritmo == 0)   {                            // Laberinto Conocido 
                maniobra = labAlgoConMan[labAlgoConNum];
                labAlgoConNum++;
              }                                 
              else if (labAlgoritmo == 1) { maniobra = 2; sentidoDesvio = +1;}   // Regla derecha, voy adelante
              else                        { maniobra = 4; sentidoDesvio = -1;}   // Regla izquierda, giro
 
            break;
          
            case 3: maniobra = 1; break;                            // *Pasillo. Adelante equidistante de las paredes 

            case 4: maniobra = 4; sentidoDesvio = +1; break;        // T. Girar a la derecha o a la izquierda                                           

              if (labAlgoritmo == 0)   {                            // Laberinto Conocido 
                maniobra = labAlgoConMan[labAlgoConNum];
                labAlgoConNum++;
              }                                 
              else { maniobra = 4; sentidoDesvio = labAlgoritmo;}    // Regla derecha o izquierda, giro

            break;            

            case 5: maniobra = 4; sentidoDesvio = +1; break;         // *L derecha. Girar a la derecha
            case 6: maniobra = 4; sentidoDesvio = -1; break;         // *L izquierda. Girar a la izquierda
            case 7: maniobra = 6;                     break;         // *Fondo. Girar 180 grados
          }
        break;

        // Avanzar por el centro del pasillo
        case 1:
 
          if (tipoCruce != 3) {                                                   // He llegado a un cruce
            if (tipoCruce != 7) {
              targetSpeed = TARGET_SPEED_LABER;
              marcha(targetSpeed,0);                                                // Avanza 1/3
              delay((255L * 1000 * (ANCHO_LAB_PASILLO / 40)) / (70 * targetSpeed));           
            }
            PARAR; 
            maniobra = 0;
            }                             
          else {
            *posicion = addPosicion(posicion, sizeof(posicion), (sharpDist[2] - sharpDist[0]));
            difVeloCalculo(posicion, (DIST_PARED_TARG - sharpMin[0]), targetSpeed, TARGET_SPEED_LABER);  
            targetSpeed = TARGET_SPEED_LABER;
            marcha(targetSpeed, difVelo); 
          }             
        break;
      
        // Avanzar manteniendo distancia a una pared (indicada por sentidoDesvio)
        case 2:

          if ((tipoCruce != 1) && (tipoCruce != 2)) {
            targetSpeed = TARGET_SPEED_LABER;
            marcha(targetSpeed,0);                                                // Avanza 1/3
            delay((255L * 1000 * (ANCHO_LAB_PASILLO / 40)) / (70 * targetSpeed));           
            PARAR; 
            maniobra = 0;
          }
          else {
            *posicion = addPosicion(posicion, sizeof(posicion), 
              (-sentidoDesvio * (sharpDist[sentidoDesvio+1] - DIST_PARED_TARG)));
            difVeloCalculo(posicion, (DIST_PARED_TARG - sharpMin[0]), targetSpeed, TARGET_SPEED_LABER);  
            
            targetSpeed = TARGET_SPEED_LABER;
            marcha(targetSpeed, difVelo); 
          }             
        break;

        // Girar en desvio !! Cuidado que el frente está libre al inicio!!
        case 4:
          if (sentidoDesvio != 0) {                                                // He completado el giro       
            if ((sharpDist[sentidoDesvio+1] < DIST_PARED_GIRO_COMPLETO) &&
               (sharpDist[1] > DIST_PARED_DESPEJADO_FRONTAL)) {
               targetSpeed = TARGET_SPEED_LABER;
               marcha(targetSpeed,0);                                                // Avanza 1/3
               delay((255L * 1000 * (ANCHO_LAB_PASILLO / 40)) / (70 * targetSpeed));           
               PARAR; 
               maniobra = 0;
            }
            else if (sharpDist[sentidoDesvio+1] < DIST_PARED_TARG) {
              targetSpeed = TARGET_SPEED_GIRO; difVelo = 0; difVeloP = 0; difVeloD = 0;
              marcha(targetSpeed, difVelo);
            }                                           
            else {                       
              targetSpeed = TARGET_SPEED_GIRO; 
              difVelo = (sentidoDesvio * TARGET_SPEED_GIRO); difVeloP = 0; difVeloD = 0;
              marcha(targetSpeed, difVelo);
            }
          }
          else {PARAR; maniobra = 0;}          // Probable ERROR DE PROGRAMACION        
        break;

        // Girar en fondo de saco hasta encontrar despejado al frente
        case 6:
          if (sentidoDesvio == 0)  sentidoDesvio = ((random(0, 2) * 2) - 1); 
          if ((sharpDist[1] > DIST_PARED_DESPEJADO_FRONTAL) && ((sharpDist[0]+sharpDist[2]) < 380))
          {sentidoDesvio = 0; PARAR; maniobra = 0;}    // He completado el giro          
          else { 
            difVelo = (sentidoDesvio * TARGET_SPEED_GIRO); difVeloP = 0; difVeloD = 0;
            targetSpeed = (TARGET_SPEED_GIRO / 2);
            marcha(targetSpeed, difVelo);
          }                 
        break;
      
        // No veo paredes, avanzar recto 1 segundo
        case 7:
          targetSpeed = TARGET_SPEED_LABER;
          marcha(targetSpeed,0);                                                // Avanza 1/3
          delay((255L * 1000 * (ANCHO_LAB_PASILLO / 3)) / (70 * targetSpeed));           
          PARAR; 
          maniobra = 0;
        break;
      }
      LOG_WRT(51, loopCnt, maniobraActual, sentidoDesvio, tipoCruce,  posicion[0], targetSpeed, difVelo, 0, 0);
      LOG_WRT(52, loopCnt, sharpDist[1], sharpDist[0], sharpDist[2], (sharpDist[0]+sharpDist[2]), maniobra, ((255L * 1000 * (ANCHO_LAB_PASILLO / 30)) / (70 * TARGET_SPEED_LABER)), 0, 0);
    break;       
  }
}


/*
 *  MARCHA
 *  Funcion que hace marchar hacia delante o hacia atrás el bólido
 *  con un radio de giro
 */
 void marcha(uint8_t speed, int difVelo) {

   // ESTABLEZCO LAS VELOCIDADES PARA LOS DOS MOTORES
   int speedM[2] = {((int)speed + difVelo) , (int)speed};
   if ((speed * (long)difVelo) > 0) { 
     speedM[0] = (int)speed;
     speedM[1] = ((int)speed - difVelo);
   }

   LOG_WRT(5, 0, speedM[0], speedM[1], difVelo, 0, 0, 0, 0, 0);  

//  int speedM[2] = {((int)speed - (difVelo/2)), ((int)speed + (difVelo/2))};   de esta forma la diferencia se reparte entre los dos motores
  
   if ((speedM[0] == currentSpeed[0]) && (speedM[1] == currentSpeed[1])) return; // Si no han variado las velocidades, no hago nada

   // Establezco las nuevas velocidades de los motores y su dirección

   for (uint8_t i = 0; i < 2; i++) {
    if (speedM[i] >= 0) {
      #if defined(BAINITA_BOLIDO_UNO)
           digitalWrite(dir[i], HIGH);                      //adelante
          analogWrite (pwm[i], (uint8_t)speedM[i]);          //pongo la velocidad
      #elif defined(BAINITA_BOLIDO_DOS)
        if (currentSpeed[i] < 0)      
            myMotor[i]->run(FORWARD);                        //adelante
        if (speedM[i] != currentSpeed[i])
          myMotor[i]->setSpeed((uint8_t)speedM[i]);          //pongo la velocidad
      #endif   
    }
    else {
      #if defined(BAINITA_BOLIDO_UNO) 
          digitalWrite(dir[i], LOW);                       //atras
          analogWrite (pwm[i], (uint8_t)(-speedM[i]));       //pongo la velocidad
      #elif defined(BAINITA_BOLIDO_DOS)
        if (currentSpeed[i] >= 0)
          myMotor[i]->run(BACKWARD);                         //atras
        if (speedM[i] != currentSpeed[i])
          myMotor[i]->setSpeed((uint8_t)(-speedM[i]));       //pongo la velocidad
        #endif       
    }
    currentSpeed[i] = speedM[i];
  }     
}

/*
 *  CALIBRA
 *  
 *  Funcion que calibra los sensores QTR
 */
void CalibraQTR() {
#if defined(BAINITA_BOLIDO_DOS)
  LED_ON_BLUE;
#endif


  #if defined(SERIAL_CONN)
      if (serialConnect)  Serial.println(F("\nEMPIEZA CALIBRADO"));
  #endif

  #if defined(CALIBRA_MANUAL) // En este caso, se ejecuta un calibrado
  
    for (unsigned int i = 0; i < 400; i++)  // make the calibration take about 10 seconds
    {
      qtrrc.calibrate();       // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
      #if defined(DEBUG_CALIBRA) 
        calibraPrt(qtrrc.calibratedMinimumOn, qtrrc.calibratedMaximumOn);
      #else 
        delay(10);
      #endif   
  }

  #endif

  #if defined(CALIBRA_MENEITO)
    myMotor[0]->run(RELEASE);       
    myMotor[1]->run(RELEASE);       
    myMotor[0]->setSpeed(30);
    myMotor[1]->setSpeed(30); 
           
    for (uint8_t i = 0; i < 20; i++) {                // make the calibration take about 10 seconds 
      myMotor[i % 2]->run(FORWARD);
      myMotor[(i+1) % 2]->run(BACKWARD); 
      for (uint8_t j = 0; j < 20; j++) {              // make the calibration take about 0,5 seconds
        qtrrc.calibrate();                            // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
        #if defined(DEBUG_CALIBRA) 
          calibraPrt(qtrrc.calibratedMinimumOn, qtrrc.calibratedMaximumOn);
        #else 
          delay(10);
        #endif
      }  
      myMotor[i % 2]->run(BACKWARD);
      myMotor[(i+1) % 2]->run(FORWARD); 
      for (uint8_t j = 0; j < 20; j++)  {              // make the calibration take about 0,5 seconds
        qtrrc.calibrate();       // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
        #if defined(DEBUG_CALIBRA) 
          calibraPrt(qtrrc.calibratedMinimumOn, qtrrc.calibratedMaximumOn);
        #else  
          delay(10);
        #endif
      }   
    } 

    myMotor[0]->setSpeed(0);
    myMotor[1]->setSpeed(0); 
    myMotor[0]->run(RELEASE);       
    myMotor[1]->run(RELEASE);     
  #endif

  // Valores almacenados previamente para diferentes tipos de suelo
  #if defined(CALIBRA_STORED_CONCURSO_REAL)
    unsigned int calibrateMinStored[] = {112, 156, 156, 160, 160, 164, 212, 268};
    unsigned int calibrateMaxStored[] = {812, 640, 532, 592, 644, 636, 748, 1032};    
  #endif
  #if defined(CALIBRA_STORED_PAPEL)
    unsigned int calibrateMinStored[] = {164, 164, 180, 220, 220, 180, 164, 164};
    unsigned int calibrateMaxStored[] = {1100, 930, 880, 830, 830, 880, 930, 1100};
  #endif
  #if defined(CALIBRA_STORED_TERRAZA)
    unsigned int calibrateMinStored[] = {380, 400, 420, 440, 440, 420, 400, 380};
    unsigned int calibrateMaxStored[] = {1300, 1000, 900, 850, 850, 900, 1000, 1300};
  #endif
  #if defined(CALIBRA_STORED_PARQUET)
    unsigned int calibrateMinStored[] = {156,156,156,156,156,160,208,216};
    unsigned int calibrateMaxStored[] = {752,636,584,692,748,692,856,1032};
  #endif
  #if defined(CALIBRA_STORED_INDU_GIMNASIO)
    unsigned int calibrateMinStored[] = {560, 460, 564, 512, 512, 512, 512, 724};
    unsigned int calibrateMaxStored[] = {1200, 820, 1152, 984, 988, 984, 1096, 1200};
  #endif
  #if defined(CALIBRA_STORED_CIRCUITO_PRUEBA)
    unsigned int calibrateMinStored[] = {120, 160, 160, 160, 200, 210, 220, 270};
    unsigned int calibrateMaxStored[] = {940, 930, 700, 820, 880, 830, 830, 1300};
  #endif
  #if defined(CALIBRA_STORED_TELLI_COCINA)
    unsigned int calibrateMinStored[] = {256, 256, 208, 256, 256, 256, 308, 364};
    unsigned int calibrateMaxStored[] = {1176, 800, 684, 744, 800, 792, 1012, 1200};
  #endif

  #ifndef CALIBRA_EXEC
  // Esta parte se ejecuta cuando no se ejecuta el calibrado, sino que se dan valores previamente almacenados
  // *** Es necesario llamar a calibrate() antes de asignar valores, porque tiene que allocar espacio para las variables
    qtrrc.calibrate();       // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)    
    for (uint8_t i = 0; i < QTR_NUM; i++) {
      qtrrc.calibratedMinimumOn[i] = calibrateMinStored[i];
      qtrrc.calibratedMaximumOn[i] = calibrateMaxStored[i];
    }
  #endif

#if defined(BAINITA_BOLIDO_DOS)
  LED_OFF_BLUE;     // turn off Arduino's LED to indicate we are through with calibratio
#endif

#if defined(SERIAL_CONN) 
  if (serialConnect)  Serial.println(F("\nTERMINA CALIBRADO"));
  if (serialConnect) Serial.println(F("Valores Finales del CALIBRADO"));
#endif
#if (defined(LOGGER_SHIELD) || defined(SERIAL_CONN)) 
        calibraPrt(qtrrc.calibratedMinimumOn, qtrrc.calibratedMaximumOn);
#endif
}


/*
 * 
 *   Calculo del ajuste de velocidad PDI
 * 
 * 
 * 
 */

void difVeloCalculo(int error[], int errorRef, uint8_t speedAct, uint8_t speedNext) {

  difVeloP = ((long)error[0] * K_POS * (long)speedNext)/(10L * errorRef);                                            // Algoritmo proporcional con la desviación
  //int difVeloP = (sqrt((float)abs(error[0]) / (float)errorRef) * K_POS * speedNext)/10;                            // Algoritmo con funcion raiz cuadrada, corrige más fuerte las 
  //if (posicion[0] < 0) difVeloP = -difVeloP;                                                                       // desviaciones, pero creo que reduce un poco la velocidad  TRO);

  //Calculo parte diferencial del PID
  if (speedAct != 0) {                                                    
     difVeloD = -(K_DER * (speedNext * ((long)error[1] - (long)error[0])))/(10L * errorRef);
  }
  else {
    difVeloD = 0;                                                                                                    // Si estaba parado, no tengo en cuenta la diferencial                                              
  }
  // Evito que ningun motor vaya hacia atras
  difVelo = constrain((difVeloP + difVeloD), -speedNext, speedNext);  
}

/*
 *  isLine()   
 * 
 *  Funcion para comprobar si el bólido está sobre la línea
 *  Respuestas:
 *  >0 Sobre la línea, devuelve anchura de la línea
 *  -1- Se ha salido de la línea
 *  -2- En el aire
 */
  int isLine(int error, unsigned int sensor[QTR_NUM]) {

  static unsigned long primerTiempoDesvio = 0;
  static unsigned long primerTiempoAngulo = 0;
  unsigned long millisAhora = millis();  
  long ancho = 0;
  static int anchoAnt = 0;
  static int anchoRef = 0;
  
  long          error_2 = 0;
  unsigned int divisor = 0;
  int     valle[QTR_NUM];                                          // Esta variable ayuda a encontrar si hay una DESVIO
  uint8_t valleConsecutivo = 0;
  uint8_t valleConsecutivoMax = 0;
  int     valleMin = 0;
  bool    afterPeak = false;
  bool    hayDesvio = false;
  uint8_t valleMinPosRacha = 0;
  uint8_t valleMinPosFinal = 0;                                    // Guarda la posicion i mínima del valle
 
   // Si la posicion está fuera del rango, no está en la línea
   if (abs(error) >= QTR_CENTRO) return(-1);
   
/*   for (uint8_t i=0; i < QTR_NUM; i++) {
     if (sensor[i] > 200) {
       error_2+= (i*1000 * (long)sensor[i]);
       divisor+= sensor[i];           
     }     
   }
   error_2/= divisor; error_2-= QTR_CENTRO; */
  
   for (uint8_t i=0; i < QTR_NUM; i++) {
     if (sensor[i] > 200) {
       //ancho+= (abs(((i*1000)-QTR_CENTRO) - error_2) * (long)sensor[i]);    
       ancho+= (abs(((i*1000)-QTR_CENTRO) - error) * (long)sensor[i]);
       divisor+= sensor[i];    
     }  
   }
   ancho/=divisor;

// CALCULOS PARA DETERMINAR SI HAY ANGULO RECTO
  
   if ((ancho >= QTR_ANCHO_ANGULO) && (primerTiempoAngulo == 0)) {
    primerTiempoAngulo = millisAhora;
    sentidoAngulo = (error/abs(error));                       // +1 a la derecha, -1 a la izquierda, 0 no hay marca
   }
   else if (((millisAhora - primerTiempoAngulo) > ancho_t_angulo) && (ancho < QTR_ANCHO_ANGULO)) {
     primerTiempoAngulo = 0;
     sentidoAngulo = 0;
     tiempoAngulo = 0;
   }

   if (primerTiempoAngulo != 0) tiempoAngulo = millisAhora - primerTiempoAngulo;

// CALCULOS PARA AJUSTAR DIRECCION EN DESVIOS
   
   if ((ancho >= QTR_ANCHO_DESVIO) && (primerTiempoDesvio == 0)) {
    primerTiempoDesvio = millisAhora;
    sentidoDesvio = (error/abs(error));                       // +1 a la derecha, -1 a la izquierda, 0 no hay marca
    velo_reductora = VELO_REDUCTORA;
    anchoRef = anchoAnt;
   }
   else if (((millisAhora - primerTiempoDesvio) > ((ancho_t_desvio * 100) / velo_reductora)) && (ancho < QTR_ANCHO_DESVIO)) {
     primerTiempoDesvio = 0;
     sentidoDesvio = 0;
     velo_reductora = 100;
   }

   if (sentidoDesvio != 0) {
     if ((millisAhora - primerTiempoDesvio) > ((ancho_t_marca * 100) / velo_reductora)) {
       for (uint8_t i=0; i < (QTR_NUM); i++) {
         valle[i] = sensor[i];
         if ((i > 0) && ( i < (QTR_NUM-1))) valle[i] = 2 * sensor[i];
         else valle[i] = sensor[i];
         if (i < (QTR_NUM-2))   valle[i] = (valle[i] - ((sensor[i+1] + sensor[i+2])/2));
         if (i > 1)             valle[i] = (valle[i] - ((sensor[i-1] + sensor[i-2])/2));
         if (i == 1)            valle[i] = valle[i]  - sensor[i-1];     
         if (i == (QTR_NUM-2))  valle[i] = valle[i]  - sensor[i+1];

         if ((i > 0) && ( i < (QTR_NUM-1))) valle[i] /= 2;
       
       
         if ((valle[i] < 0) && (afterPeak)){
           valleConsecutivo++;         
           if (valle[i] < valleMin) {
             valleMin = valle[i];
             valleMinPosRacha = i;          
           } 
         }                   // Si encontramos un pico, dejamos de contar y guardamos valores
         else {
           if (valle[i] > 0) afterPeak = true;
           if (valleConsecutivo > valleConsecutivoMax) {
             valleConsecutivoMax = valleConsecutivo;
             valleMinPosFinal = valleMinPosRacha;
             if (valleMin < VALLE_CONSECUTIVO_VALOR) hayDesvio = true;
             else hayDesvio = false;
           }
           valleConsecutivo = 0;
         }        
       }
    
       if (valleConsecutivoMax < VALLE_CONSECUTIVO_NUMERO) hayDesvio = false;

       //  Si HAY DESVIO, recalculo la posicion, descartando la linea negra no deseada
       if (hayDesvio) {
         divisor = 0; error_2 = 0;
         for (uint8_t i=0; i < QTR_NUM; i++) {
           if ((sensor[i] > 50) && 
             ( ((sentidoDesvio == -1) && (i < valleMinPosFinal)) || ((sentidoDesvio == 1) && (i > valleMinPosFinal)) ) ) {
             error_2+= (i*1000 * (long)sensor[i]);
             divisor+= sensor[i];                     
           }     
         }
         error_2/= divisor; error_2-= QTR_CENTRO;
         posicion[0] = error_2;            
       }
       else {
         posicion[0] = posicion[0] + ( ( (sentidoDesvio * constrain(((int)ancho - anchoRef), 0, 3500 ) ) * 1) /1);
       }
       LOG_WRT(14, 0, valle[0], valle[1], valle[2], valle[3], valle[4], valle[5], valle[6], valle[7]);     
     }
     else {
       posicion[0] = posicion[0] - (((sentidoDesvio * constrain(((int)ancho - anchoRef), 0, 3500 )))/2);
     }
     LOG_WRT(15, 0, hayDesvio, sentidoDesvio, posicion[0], error, (millisAhora - primerTiempoDesvio), ((ancho_t_marca * 100) / velo_reductora), ((ancho_t_desvio * 100) / velo_reductora), ((ancho_t_angulo * 100) / velo_reductora));     
   }   
   anchoAnt = (int)ancho;                                       // Me guardo el ancho anterior
   return(ancho);     
 }

 /*
 *  isParedes()
 *  
 *  Funcion para situar las paredes en el laberinto
 *  Respuestas:
 *  0- Libre delante, libre a los dos lados
 *  1- Libre delante, lado libre solo a la derecha
 *  2- Libre delante, lado libre solo a la izquierda
 *  3- Libre delante, paredes a los dos lados   
 *  4- Pared delante, libre a los dos lados
 *  5- Pared delante, libre solo a la derecha
 *  6- Pared delante, libre solo a la izquierda
 *  7- Pared delante, paredes a los dos lados
 *   
 */
 uint8_t isParedes() {
  
   uint8_t retorno = 0;

   //Pared delante
   if (sharpDist[1] <= DIST_PARED_HUECO_FRONTAL) retorno= 4;
    
   //Pared a la izquierda
   if (sharpDist[0] <= DIST_PARED_HUECO_LATERAL) retorno++;

   // Pared a la derecha
   if (sharpDist[2] <= DIST_PARED_HUECO_LATERAL) retorno+=2;
  
   return(retorno);
 }

/*
 * logWrt()
 * 
 * Grabo un registro en el log
 * 
 */

void logWrt(int tipoReg, unsigned long loopCnt, int dato1, int dato2, int dato3, int dato4, int dato5, int dato6, int dato7, int dato8) {
#if defined(LOGGER_SHIELD)

  if (loggerConnect) {
    sprintf_P(buffer, PSTR("%d,%lu,%lu,%d,%d,%d,%d,%d,%d,%d,%d"), tipoReg, millis(), loopCnt, dato1, dato2, dato3, dato4, dato5, dato6, dato7, dato8);
    dataFile.println(buffer);
  }
#endif
#if defined(SERIAL_CONN)
    if (serialConnect) {
      switch (tipoReg) {
        #ifdef DEBUG_LOOP
          // Marcha - Velocidad izquierdo, velocidad derecho, diferencia        
          case 5:
            sprintf_P(buffer, PSTR("** %03d- T:%6lu L:%5lu Velocidad I:%4d D:%4d Dif:%4d  MARCHA"), tipoReg, millis(), loopCnt, dato1, dato2, dato3);       
          break;          
          // (StopDist, DistFrontal)     
          case 2:
             sprintf_P(buffer, PSTR("** %03d- T:%6lu L:%5lu StopD= %1d DisF:%5d Maniobra próxima:%1u "), tipoReg, millis(), loopCnt, dato1, dato2, dato3, dato4, dato5, dato6, dato7, dato8);       
          break;
          // (Posicion, targetspeed, difVelo, difVeloP, difVeloD, anchuraLinea, sentidoDesvio)     
          case 12:
          case 32:
             sprintf_P(buffer, PSTR("** %03d- T:%6lu L:%5lu StopD= %1d Error:%5d Velo:%4d difV:%4d difVP:%4d difVD:%4d Anchura:%5d Desv:%2d"), tipoReg, millis(), loopCnt, dato1, dato2, dato3, dato4, dato5, dato6, dato7, dato8);       
          break;
          // Sensores de línea
          case 13:
            sprintf_P(buffer, PSTR("** %03d- T:%6lu L:%5lu Sensor: %4d\t%4d\t%4d\t%4d\t%4d\t%4d\t%4d\t%4d"), tipoReg, millis(), loopCnt, dato1, dato2, dato3, dato4, dato5, dato6, dato7, dato8);       
          break;
          // Calculo de valles en el array por si bifurcacion
          case 14:
            sprintf_P(buffer, PSTR("** %03d- T:%6lu L:%5lu Valle : %4d\t%4d\t%4d\t%4d\t%4d\t%4d\t%4d\t%4d"), tipoReg, millis(), loopCnt, dato1, dato2, dato3, dato4, dato5, dato6, dato7, dato8);       
          break;                   
          case 15:
            sprintf_P(buffer, PSTR("** %03d- T:%6lu L:%5lu Haydesv:%1d Sentdesv:%1d ErroCorr:%5d ErroMed:%5d TdesdePDesv:%6u TMAR:%3d TDES:%3d"), tipoReg, millis(), loopCnt, dato1, dato2, dato3, dato4, dato5, dato6, dato7, dato8);       
          break;
          // (Posicion, targetspeed, difVelo, difVeloP, difVeloD, carrilAdelanta, estoyAdelantando)
          case 22:
             sprintf_P(buffer, PSTR("** %03d- T:%6lu L:%5lu StopD= %1d Error:%5d Velo:%4d difV:%4d difVP:%4d difVD:%4d Carril Ad:%2d EstoyAdelant:%1d"), tipoReg, millis(), loopCnt, dato1, dato2, dato3, dato4, dato5, dato6, dato7, dato8);       
          break;       
          // Laberinto. Maniobra, sentidoDesvio, Cruce, Error, Velo, difV, difVP, difVD
          case 51:
            sprintf_P(buffer, PSTR("** %03d- T:%6lu L:%5lu Mani:%2d Desv:%2d Cruce:%2d Error:%5d Velo:%4d difV:%4d"), tipoReg, millis(), loopCnt, dato1, dato2, dato3, dato4, dato5, dato6, dato7, dato8);       
          break;
          // Laberinto. disF, disI, disD, Suma I+D, proxima maniobra
          case 52: 
            sprintf_P(buffer, PSTR("** %03d- T:%6lu L:%5lu DisF:%5d DisI:%5d DisD:%5d AnchoCanal:%5d ManiProx:%2d"), tipoReg, millis(), loopCnt, dato1, dato2, dato3, dato4, dato5, dato6, dato7, dato8);       
          break;
        #endif
        #ifdef DEBUG_DISTANCIA
          // Distancia frontal - Milis Primera deteccion, Milis desde entonces, lectura sensor, distancia 
          case 81:
            sprintf_P(buffer, PSTR("** %03d- T:%6lu L:%5lu Retorno: %d DisF:%5d Lectura:%4d Pdo:%6u StopT:%6u"), tipoReg, millis(), loopCnt, dato1, dato2, dato3, dato4, dato5);    
          break;
          case 82:
            sprintf_P(buffer, PSTR("** %03d- T:%6lu L:%5lu Distancia I: %3d D: %3d"), tipoReg, millis(), loopCnt, dato1, dato2);
          break;       
        #endif
          // Loop inicio (programa, maniobra, stopDist, sharpDist[1], posicion[0], targetSpeed, 0, 0
          case 1:
            sprintf_P(buffer, PSTR("** %03d- T:%6lu L:%5lu Prog:%1u Man:%1u StopD= %1d DisF:%5d Error:%5d Velo %3d Desv:%2d"), tipoReg, millis(), loopCnt, dato1, dato2, dato3, dato4, dato5, dato6, dato7);
          break;
          // Flush del fichero log
          case 91:
            sprintf_P(buffer, PSTR("** %03d- T:%6lu L:%5lu LoopRef:%5u Velo: %3d"), tipoReg, millis(), loopCnt, dato1, dato2);
          break;
        //case 92:     /Indica duracion escesiva de un loop, solamente se ve en el Logger
          case 100:    // Para registrar cualquier cosa
            sprintf_P(buffer, PSTR("** %03d- T:%6lu L:%5lu 1:%d 2:%d 3:%d 4:%d 5:%d 6:%d 7:%d 8:%d"), tipoReg, millis(), loopCnt, dato1, dato2, dato3, dato4, dato5, dato6, dato7, dato8);       
          break;                

          default:
            buffer[0] = '\0';
          break;
      }
      if (buffer[0] != '\0') Serial.println(buffer);
    }
    
#endif 
}

/*
 *   Añade un elemento a la posición 0 de un array de int, y desplaza el resto
 */

int addPosicion(int pos[], uint8_t size, int add) {

  if (pos[1] != SHRT_MAX) {                                                         // Si no es la primera vez, desplazo y añado
    for (uint8_t i = 1; i < (size/sizeof(pos[0])); i++) pos[i] = pos[i-1];
    pos[0] = add; 
  }    
  else {                                                                         // Si primera vez, relleno todo con el valor añadido
    for (uint8_t i = 0; i < (size/sizeof(pos[0])); i++) pos[i] = add;    
  }
  return(*pos);
}

/*
 *  DistanciaFrontalMide()
 *  
 *  Medición de distancia al frente
 *  También guarda la hora de la primera detección consecutiva 
 *  de cercanía al obstáculo por debajo de un valor predefinido DIST_FRONT_STOP 
 *  
 */

bool DistanciaFrontalMide() {
  static unsigned long primerTiempObst = 1;
  unsigned long millisAhora = millis();
  int lectura = analogRead(SHARP_PIN_F);
  sharpDist[1] = sharpFac[1]/(constrain(lectura, SHARP_41_LECT_MIN, SHARP_41_LECT_MAX) - sharpSub[1]);
  
  if ((sharpDist[1] < DIST_FRONT_STOP) && (primerTiempObst == 0)) {    
    primerTiempObst = millisAhora;
  }
  else if (sharpDist[1] >= DIST_FRONT_STOP) primerTiempObst = 0;

  LOG_WRT(81, 0, ((sharpDist[1] < DIST_FRONT_STOP) && ((millisAhora - primerTiempObst) > SHARP_TIEMPO_RUIDO)), 
          sharpDist[1], lectura, primerTiempObst, (millisAhora - primerTiempObst), 0, 0, 0);    

  if ((sharpDist[1] < DIST_FRONT_STOP) && ((millisAhora - primerTiempObst) > SHARP_TIEMPO_RUIDO)) return(true);
  else return(false);
}

/*
 * DistanciaLateralMide()
 * 
 * Medición de distancias a los lados en laberinto
 * 
 */

void DistanciaLateralMide() {
  
  sharpDist[0] = sharpFac[0]/(constrain(analogRead(SHARP_PIN_I), SHARP_41_LECT_MIN, SHARP_41_LECT_MAX)-sharpSub[0]);

  sharpDist[2] = sharpFac[2]/(constrain(analogRead(SHARP_PIN_D), SHARP_41_LECT_MIN, SHARP_41_LECT_MAX)-sharpSub[2]);

  LOG_WRT(82, 0, sharpDist[0], sharpDist[2], 0, 0, 0, 0, 0, 0);

}

/*
 *  calibraPrt()
 *  
 *  Imprime resultados de calibrado
 * 
 */
 #if (defined(LOGGER_SHIELD) || defined(SERIAL_CONN))
 void calibraPrt(unsigned int calibratedMinimumOn[8], unsigned int calibratedMaximumOn[8]) {

   #if defined(LOGGER_SHIELD)
     if (loggerConnect) {
       sprintf_P(buffer, PSTR("%u,%lu,%d,%d,%d,%d,%d,%d,%d,%d,%d"), 200U, millis(), 0, calibratedMinimumOn[0], calibratedMinimumOn[1],
                                                                                      calibratedMinimumOn[2], calibratedMinimumOn[3],
                                                                                      calibratedMinimumOn[4], calibratedMinimumOn[5], 
                                                                                      calibratedMinimumOn[6], calibratedMinimumOn[7]);
       dataFile.println(buffer);
       sprintf_P(buffer, PSTR("%u,%lu,%d,%d,%d,%d,%d,%d,%d,%d,%d"), 201U, millis(), 0, calibratedMaximumOn[0], calibratedMaximumOn[1],
                                                                                      calibratedMaximumOn[2], calibratedMaximumOn[3],
                                                                                      calibratedMaximumOn[4], calibratedMaximumOn[5], 
                                                                                      calibratedMaximumOn[6], calibratedMaximumOn[7]);
       dataFile.println(buffer);
     }
   #endif
   #if defined(SERIAL_CONN)
     if (serialConnect) {
        
       sprintf_P(buffer, PSTR("\nMinimos  :\t%4d\t%4d\t%4d\t%4d\t%4d\t%4d\t%4d\t%4d"), calibratedMinimumOn[0], calibratedMinimumOn[1],
                                                                                       calibratedMinimumOn[2], calibratedMinimumOn[3],
                                                                                       calibratedMinimumOn[4], calibratedMinimumOn[5], 
                                                                                       calibratedMinimumOn[6], calibratedMinimumOn[7]);
       Serial.println(buffer);
       sprintf_P(buffer, PSTR("Máximos  :\t%4d\t%4d\t%4d\t%4d\t%4d\t%4d\t%4d\t%4d"), calibratedMaximumOn[0], calibratedMaximumOn[1],
                                                                                     calibratedMaximumOn[2], calibratedMaximumOn[3],
                                                                                     calibratedMaximumOn[4], calibratedMaximumOn[5], 
                                                                                     calibratedMaximumOn[6], calibratedMaximumOn[7]);
       Serial.println(buffer);
     }
     else delay(10);
   #endif  
   return;
 }
 #endif

        


  
