//Busqueda y Rescate
#include <math.h>
#include <stdio.h>
#define MAP_SIZE 4  // Tamaño del mapa (ajustar según la resolución y entorno)
//const int MAP_SIZE = MAP_SIZE_one;
const float CELDA = 15;  // = (Tamaño/2) Tamaño de la celda en centimetros (ejm 20cm*20cm) (ajustar según el tamaño del robot)
char mapa[MAP_SIZE][MAP_SIZE];  // Matriz del mapa de ocupación
int robotX, robotY;            // Posición actual del robot en el mapa
int inicioX, inicioY;          // Posición inicial (punto de origen)
int objetoX, objetoY; // Posición del objeto blanco (B)
int avanzo;

/* Ejemplo de la Matriz
 1 : ocupado (obstáculo)
 0 : libre
 - : desconocido / No explorado
"R": Posición actual de un robot
"B": Objeto blanco encontrado 

[-,  -,  0,  0]
[-,  -,  0,  1]
[-,  B,  0,  0]
[-,  R,  0,  0]


[0,  -,  -,  -] 
[0,  -,  -,  -] 
[0,  1,  -,  -] 
[0,  R,  -,  -]
*/

//Ultrasonic Sensor
const int Trigger = A0; // Pin Analogico 0 para el trigger del sensor 
const int Echo = A1; // Pin Analogico 1 para el echo del sensor

long duration; // Tiempo que dura en llegar el eco
long distance; // Distancia en cm 
long distanceInicial; // Distancia en cm 


// Qtr Sensor - Infrarrojo
// Numbers from 0 (maximum reflectance white) to 1000 (minimum reflectance black)
#include <QTRSensors.h>
QTRSensors qtr;

const uint8_t SensorCount = 2;
uint16_t sensorValues[SensorCount];

bool Obst = false; //Encontro objeto u obstaculo en la celda del frente false:no - true:si

// MPU6050 (Giroscopio-Acelerometro)
// Gyroscope readings from Adafruit MPU6050
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

// For calculate the degrees of the turn 
float deltaTime;
unsigned long lastTime;
float angle = 0; // Almacena el ángulo acumulado
float targetAngle = 0; // Ángulo objetivo (90 o 180 grados)

// Orientación actual en función del ángulo acumulado
int orientacion = 0;  // 0: Norte, 1: Este, 2: Sur, 3: Oeste (puntos cardinales relativos a la matriz)
float orientacionActual = 0;  // Ángulo acumulado del robot

// X positivo hacia el frente del robot
//Movimiento
float velX = 0;
float velY = 0;

float posX = 0;
float posY = 0;

// Calibration offsets
float accelOffsetX = 0, accelOffsetY = 0, accelOffsetZ = 0;
float gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0 ;

// Moving Average Filter
#define FILTER_SIZE 5  // Número de muestras para el promedio
float gyroBuffer[FILTER_SIZE]; // Buffer circular para lecturas
int bufferIndex = 0;           // Índice actual en el buffer
bool bufferInitialized = false; // Bandera de inicialización
int validValues = 0;           // Contador de valores válidos

//Motors
#include <AFMotor.h>
AF_DCMotor motori(1); //Left motor - connected to terminal 1
AF_DCMotor motord(2); //Right motor - connected to terminal 2

//Nodo Para A* 
struct Nodo {
    int x, y;        // Coordenadas en el mapa
    float costoG;    // Costo acumulado desde el inicio
    float costoH;    // Heurística estimada al objetivo
    float costoF;    // Costo total (G + H)
    Nodo* padre;     // Nodo previo en el camino
};

// Define a global array to store the path coordinates
const int MAX_PATH_LENGTH = MAP_SIZE * MAP_SIZE;
int path[MAX_PATH_LENGTH][2];
int pathLength = 0;
int objetivoX, objetivoY; // Posición del objetivo para A*

void setup() {
  Serial.begin(9600);

  // Matriz - Mapa
  inicioX = 0;
  inicioY = 0;
  robotX = inicioX;
  robotY = inicioY;
  inicializarMapa();
  avanzo = 0;

  //Ultrasonic Sensor
  pinMode(Trigger, OUTPUT); // Set trigger pin as an Output
  pinMode(Echo, INPUT); // Set echo pin as an Input

  //Configurando motores
  motori.setSpeed(80);
  motord.setSpeed(80);
  motori.run(RELEASE);
  motord.run(RELEASE);
  delay(100);
  
  // MPU Configuration
  if (!mpu.begin()){
    Serial.println("Failed to find MPU6050");
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

  lastTime = millis();

  // Perform calibration
  calibrateSensor();



  //Qtr
  // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A2, A3}, SensorCount);
  delay(500);
  // analogRead() takes about 0.1 ms on an AVR.
  // 0.1 ms per sensor * 4 samples per sensor read (default) * 6 sensors
  // * 10 reads per calibrate() call = ~24 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }

  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(100);

}

void loop() {
  delay(7);// To make sampling rate around 100hz
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  while (!Obst){
    busquedaZigZag();
  }

  stop();
  delay(1000);
}

void inicializarMapa(){
  // Inicializar cada celda del mapa
    for (int i = 0; i < MAP_SIZE; i++) {
        for (int j = 0; j < MAP_SIZE; j++) {
            // Marcar las celdas como no exploradas inicialmente
            mapa[i][j] = '-';
        }
    }
    
    // Si hay obstáculos conocidos de antemano, podemos inicializarlos en la matriz
    // Aquí sólo como ejemplo, se puede colocar un obstáculo en la posición (4,4)
    // mapa[4][4] = 1;  // 1 indica un obstáculo
    
    // Colocar el robot en su posición inicial
    mapa[inicioX][inicioY] = 'R';  // 'R' indica la posición del robot
    
    Serial.println("Mapa inicializado.");
    vermapa();
}

void vermapa(){
  for (int i = 0; i < MAP_SIZE; i++) {
        for (int j = 0; j < MAP_SIZE; j++) {
            Serial.print(mapa[i][j]);
            Serial.print("\t");
        }
        Serial.println();
    }

}

void unaCelda(){ // Moverse una sola celda hacia adelante
  Serial.println("Una celda");
  motori.run(RELEASE);
  motord.run(RELEASE);
  delay(200);
  avanzo = 1;
  velX = 0; // Reset velocity X
  velY = 0; // Reset velocity Y
  posX = 0; // Reiniciar distancia acumulada en X
  posY = 0; // Reiniciar distancia acumulada en X
  lastTime = millis(); // Reset the time

  while (posX <= CELDA){ // Avanzar hasta que haya pasado a la siguiente celda
    motori.run(BACKWARD); // adelante
    motord.run(BACKWARD);
    delay(5);

    Serial.println(posX);

    updateAccel();
  }

  if (posX > CELDA){
    actualizarPosicion();
  }
  Serial.println("posX y posY");
  Serial.println(posX);
  Serial.println(posY);
  

  motori.run(RELEASE);
  motord.run(RELEASE);
  delay(10);
}

void go(){
  motori.run(BACKWARD);
  motord.run(BACKWARD);
  delay(5); 
}

void stop(){
  motori.run(RELEASE);
  motord.run(RELEASE);
  delay(500); 
}

void uturn(){ // With the gyroscope measure a 180-degree turn 
  motori.run(RELEASE);
  motord.run(RELEASE);
  delay(10);
  angle = 0; // Reset the accumulated angle
  targetAngle = 180; // Set the target angle
  lastTime = millis(); // Reset the time

  while (angle < targetAngle){ // Turn until the target angle is reached
    motord.run(FORWARD); //retrocede
    motori.run(BACKWARD); // avanza
    delay(5);

    updateGyro();
  }
  delay(100);

  // Actualizar la orientación acumulada
  orientacionActual += 180;  // Sumar 180 grados al ángulo actual


  // stop the motors
  stop();
}

void right(){ // -90° Using the gyroscope
  Serial.println("Gira a la derecha");
  motori.run(RELEASE);
  motord.run(RELEASE);
  delay(10);
  angle = 0; // Reset the accumulated angle - angulo relativo de giro
  targetAngle = -90; // Set the target angle
  lastTime = millis(); // Reset the time

  while (angle > targetAngle){ // Turn until the target angle is reached
    motord.run(BACKWARD);
    motori.run(FORWARD);
    delay(5);

    updateGyro();
  }
  stop();
  delay(100);

  // Actualizar la orientación acumulada
  Serial.println("menos 90");
  orientacionActual -= 90;  // Restar 90 grados al ángulo actual

  // stop the motors
  stop();
}

void left(){ //All bifurcations will be at 90° - Using the gyroscope
  Serial.println("Gira a la izquierda");
  motori.run(RELEASE);
  motord.run(RELEASE);
  delay(100);
  angle = 0; // Reset the accumulated angle
  targetAngle = 90; // Set the target angle
  lastTime = millis(); // Reset the time

  while (angle < targetAngle){ // Turn until the target angle is reached

    motord.run(FORWARD);
    motori.run(BACKWARD);
    delay(5);

    updateGyro();
  }
  stop();
  delay(100);

  // Actualizar la orientación acumulada
  orientacionActual += 90;  // Sumar 90 grados al ángulo actual

  // stop the motors
  stop();
}

// Filtro Promedio Móvil (Moving Average Filter) (Reducir el ruido del giroscopio)
float filterMovingAverage(float rawGyro) {
  //Inicializar buffer
  if (!bufferInitialized) { // Solo inicializa si no se ha hecho antes
      float initialValue = rawGyro;
        for (int i = 0; i < FILTER_SIZE; i++) {
            gyroBuffer[i] = initialValue;
        }
        bufferInitialized = true; // Marca como inicializado
    }

  // Agrega la nueva lectura al buffer
  gyroBuffer[bufferIndex] = rawGyro;
  bufferIndex = (bufferIndex + 1) % FILTER_SIZE; // Avanza el índice circular

  // Incrementa el contador de valores válidos hasta llenar el buffer
    if (validValues < FILTER_SIZE) {
        validValues++;
    }

  // Calcula el promedio de los valores en el buffer
  float sum = 0;
  for (int i = 0; i < validValues; i++) {
    sum += gyroBuffer[i];
  }
  return sum / validValues; // Retorna el promedio
}


// Función para actualizar la aceleración angular y calcular el angulo
void updateGyro() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  unsigned long currentTime = millis();
  deltaTime = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;

  float gyroZ = (g.gyro.z - gyroOffsetZ) * (180 / PI); // rad/s a deg/s
  angle += gyroZ * deltaTime;
  Serial.print("Angulo sin filtro:");
  Serial.println(angle);

  float filteredGyroZ = filterMovingAverage(gyroZ);
  // Actualizar ángulo usando la señal filtrada
  angle += filteredGyroZ * deltaTime;
  Serial.print("Angulo con filtro:");
  Serial.println(angle);

}

// Función para actualizar la aceleración y calcular la distancia
void updateAccel() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);  // Obtener los datos del sensor


  // Obtener el tiempo transcurrido
  unsigned long currentTime = millis();
  deltaTime = (currentTime - lastTime) / 1000.0;  // Tiempo en segundos
  lastTime = currentTime;

  // Leer la aceleración en los ejes X e Y (m/s^2)
  // float accelX = a.acceleration.x * (100);  // Aceleración en el eje X * (m/s^2  a cm/s^2  1m = 100cm)
  // float accelY = a.acceleration.y * (100);  // Aceleración en el eje Y * (m/s^2  a cm/s^2)

  // Apply calibration offsets
  float accelX = (a.acceleration.x * 100);// - accelOffsetX;
  float accelY = (a.acceleration.y * 100);// - accelOffsetY
  float accelZ = (a.acceleration.z * 100) - accelOffsetZ;


  // Actualizar la velocidad en cada eje (v = v0 + a * t)
  velX += accelX * deltaTime; //cm/s
  velY += accelY * deltaTime;

  // Actualizar la posición en cada eje (d = d0 + v * t)
  posX += velX * deltaTime; //cm
  posY += velY * deltaTime;
  
}

void medirDistancia() {
  // Clears the trigPin
  digitalWrite(Trigger, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(Trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trigger, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(Echo, HIGH); //us = microsegundos
  // Calculating the distance
  distance = float(duration * 0.0343) / 2; //tiempo * velocodad del sonido / 2
  // Prints the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.println(distance);
}

void actualizarPosicion() {
  // Determinar la orientación actual en función del ángulo acumulado
  int orientacion;  // 0: Norte, 1: Este, 2: Sur, 3: Oeste  
  
  // Comparar el ángulo acumulado (orientacionActual) para determinar la dirección
  // Permitimos valores negativos para indicar orientación /////NO  ESTA FUNCIONANDO
  if (abs(orientacionActual - 0) < 15 || abs(orientacionActual - 360) < 15 || abs(orientacionActual + 360) < 15) {
    orientacion = 0; // Norte
  } else if (abs(orientacionActual - 90) < 15 || abs(orientacionActual + 270) < 15) {
    orientacion = 1; // Oeste 
  } else if (abs(orientacionActual - 180) < 15 || abs(orientacionActual + 180) < 15) {
    orientacion = 2; // Sur
  } else if (abs(orientacionActual - 270) < 15 || abs(orientacionActual + 90) < 15) {
    orientacion = 3; // Este
  }
  Serial.println("Orientacion");
  Serial.println(orientacion);
  Serial.println(orientacionActual);

  // Mover en función de la orientación actual
  switch (orientacion) {
    case 0: // Norte
      if (robotX >= 0 && robotX < MAP_SIZE) {  // Limitar el movimiento dentro de la matriz
        robotX++;  // Mover hacia el norte
      }
      break;
    case 1: // Este
      if (robotY >= 0 && robotY < MAP_SIZE) {
        robotY++;  // Mover hacia el oeste
      }
      break;
    case 2: // Sur
      if (robotX >= 0 && robotX < MAP_SIZE) {
        robotX--;  // Mover hacia el sur
      }
      break;
    case 3: // Oeste
      if (robotY >= 0 && robotY < MAP_SIZE) {
        robotY--;  // Mover hacia el este
      }
      break;
  }

  // Actualizar la posición del robot en el mapa
  for (int i = 0; i < MAP_SIZE; i++) {
    for (int j = 0; j < MAP_SIZE; j++) {
      if (mapa[i][j] == 'R') {
        mapa[i][j] = '0';  // Dejar la celda vacía una vez que el robot se mueve
      }
    }
  }
  mapa[robotX][robotY] = 'R';  // Marcar la nueva posición del robot
  Serial.println("Coordenadas actuales del robot:");
  Serial.println(robotX);
  Serial.println(robotY);

  Serial.println("Posición actualizada:");
  vermapa();  // Mostrar el mapa actualizado
}

void actualizarPosObjeto(char objeto) {
  // Determinar la orientación actual en función del ángulo acumulado
  int orientacion;  // 0: Norte, 1: Este, 2: Sur, 3: Oeste  
  
  // Comparar el ángulo acumulado (orientacionActual) para determinar la dirección
  // Permitimos valores negativos para indicar orientación
  if (abs(orientacionActual - 0) < 15 || abs(orientacionActual - 360) < 15 || abs(orientacionActual + 360) < 15) {
    orientacion = 0; // Norte
  } else if (abs(orientacionActual - 90) < 15 || abs(orientacionActual + 270) < 15) {
    orientacion = 1; // Este 
  } else if (abs(orientacionActual - 180) < 15 || abs(orientacionActual + 180) < 15) {
    orientacion = 2; // Sur
  } else if (abs(orientacionActual - 270) < 15 || abs(orientacionActual + 90) < 15) {
    orientacion = 3; // Oeste
  }
  Serial.println("Orientacion");
  Serial.println(orientacion);
  Serial.println(orientacionActual);

  // Mover en función de la orientación actual
  switch (orientacion) {
    case 0: // Norte
      if (robotX >= 0 && robotX < MAP_SIZE) {  // Limitar el movimiento dentro de la matriz
        objetoX = robotX + 1;  // El objeto u obstaculo esta 1 mas al norte que la posicion actual del robot
        objetoY = robotY;
      }
      break;
    case 1: // Este
      if (robotY >= 0 && robotY < MAP_SIZE) {
        objetoY = robotY + 1;  // El objeto u obstaculo esta 1 mas al oeste que el robot
        objetoX = robotX;
      }
      break;
    case 2: // Sur
      if (robotX >= 0 && robotX < MAP_SIZE) {
        objetoX = robotX - 1;;  // +1 sur
        objetoY = robotY;
      }
      break;
    case 3: // Oeste
      if (robotY >= 0 && robotY < MAP_SIZE) {
        objetoY = robotY - 1;  // +1 este
        objetoX = robotX;
      }
      break;
  }

  // Actualizar la posición del Objeto en el mapa

  mapa[objetoX][objetoY] = objeto;  // Marcar la nueva posición del robot
  Serial.println("Coordenadas actuales del Objeto:");
  Serial.println(objetoX);
  Serial.println(objetoY);

  Serial.println("Posición actualizada:");
  vermapa();  // Mostrar el mapa actualizado
}

// Función para detenerse y examinar el obstáculo con los sensores infrarrojos
void Examinar() {
  // Detener motores
  motori.run(RELEASE);
  motord.run(RELEASE);
  delay(300);

  posX = 0; // Reiniciar distancia acumulada en X
  lastTime = millis(); // Reset the time
  distanceInicial = distance;
  while (distance > 5){
    go();
    medirDistancia();
    updateAccel();
  }
  //Avanzo
  Serial.print("Avanzo: cm");
  Serial.println(posX);
  // Detener motores
  motori.run(RELEASE);
  motord.run(RELEASE);

  // Leer los sensores infrarrojos
  qtr.read(sensorValues);
  
  if (sensorValues[0] < 800 || sensorValues[1] < 800) {
    Serial.println("Objeto Blanco detectado."); 
    actualizarPosObjeto('B'); // Marca el objeto blanco en la matriz  
    if ((robotY % 2) == 0) {
        // Acción para columnas pares (Robot dandole la espalda a la meta - debe cambiar de posicion)
        Serial.println("Columna par");
        // Moverse dos celdas en la direccion hacia donde esta mirando y girar 90 grados a la izquierda
        // 0: Norte, 1: Este, 2: Sur, 3: Oeste  
        if (orientacion==0){
            objetivoX = robotX + 2;
            objetivoY = robotY;
            AEstrella(robotX, robotY, objetivoX, objetivoY);
            // TODO: Loop para ir revisando y reajustando el mapa mientras sigue el mapa dado por AEstrella
            // TODO: condiciones para controlar el movimiento segun las coordenadas

            //Revisar
            for (int i = pathLength - 1; i >= 0; i--) {
              int nextX = path[i][0];
              int nextY = path[i][1];

              // Move to the next cell
              while (robotX != nextX || robotY != nextY) {
                medirDistancia();
                if (distance <= CELDA) {
                  Serial.println("Encontro un obstaculo en el camino");
                  actualizarPosObjeto('1'); // Marca un obstáculo en la matriz
                  AEstrella(robotX, robotY, objetivoX, objetivoY); // Recalcular el camino
                  break; // Salir del bucle para recalcular el camino
                }

                if (robotX < nextX) {
                  if (orientacion != 0) {
                    while (orientacion != 0) {
                      right();
                    }
                  }
                  unaCelda();
                } else if (robotX > nextX) {
                  if (orientacion != 2) {
                    while (orientacion != 2) {
                      right();
                    }
                  }
                  unaCelda();
                } else if (robotY < nextY) {
                  if (orientacion != 1) {
                    while (orientacion != 1) {
                      right();
                    }
                  }
                  unaCelda();
                } else if (robotY > nextY) {
                  if (orientacion != 3) {
                    while (orientacion != 3) {
                      right();
                    }
                  }
                  unaCelda();
                }
              }
            }

            // Hasta aqui 
            left();
        }
        if (orientacion==1){
            objetivoX = robotX;
            objetivoY = robotY+2;
            AEstrella(robotX, robotY, objetivoX, objetivoY);
            left();
        }
    } else {
        // Acción para columnas impares (Robot de frente a la meta - solo debe avanzar)
        Serial.println("Columna impar");
        // Empujar hasta llegar a la coordenda x = 0  o  y = 0
        if (orientacion==2){
            objetivoX = 0;
            objetivoY = robotY;
            AEstrella(robotX, robotY, objetivoX, objetivoY);

        }
        if (orientacion==3){
            objetivoX = robotX;
            objetivoY = 0;
            AEstrella(robotX, robotY, objetivoX, objetivoY);

        }
    }

    Obst = true;
  } else if (sensorValues[0] > 800 && sensorValues[1] > 800) {
    Serial.println("Obstáculo detectado.");
    actualizarPosObjeto('1'); // Marca un obstáculo en la matriz
    Obst = true;

    if ((robotY % 2) == 0) {
        // Acción para columnas pares (Rodear obstaculo)
        Serial.println("Columna par");

    } else {
        // Acción para columnas impares (Rodear obstaculo)
        Serial.println("Columna impar");

    }
  }
  
  // Regresar lo que avanzo para acercarse
  while (distance < distanceInicial ) {  // Retrocede hasta que vuelva a la posición inicial
    motori.run(FORWARD);
    motord.run(FORWARD);
    delay(5);
    medirDistancia();
  }
}

void busquedaZigZag() {

  stop();
  medirDistancia();

  if(robotX == MAP_SIZE-1){
    left();
    if (distance > CELDA){
      unaCelda();
      left();
      if (distance > CELDA){
        unaCelda();}
    }
    else if (distance <= CELDA){
      stop();
      Serial.println("Encontro un obstaculo");
      Examinar();
    }
  }
  else if(robotX == 0 && avanzo == 1){
    right();
    if (distance > CELDA){
      unaCelda();
      right();
      if (distance > CELDA){
        unaCelda();}
    }
    else if (distance <= CELDA){
      stop();
      Serial.println("Encontro un obstaculo");
      Examinar();
    }
  }
  else if(distance <= CELDA){
    stop();
    Serial.println("Encontro un obstaculo");
    Examinar();

  }
  else if (distance > CELDA){
    Serial.println("Siga derecho");
    unaCelda();
  }

}


void calibrateSensor(){
  Serial.println("Calibrating sensor. Keep it stationary...");
  float sumGyroX = 0, sumGyroY = 0, sumGyroZ = 0;
  float sumX = 0, sumY = 0, sumZ = 0;
  int numSamples = 200;

  for (int i = 0; i < numSamples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // sumGyroX += g.gyro.x;
    // sumGyroY += g.gyro.y;
    // sumGyroZ += g.gyro.z;

    sumX += a.acceleration.x * 100; // Convert to cm/s²
    sumY += a.acceleration.y * 100;
    sumZ += a.acceleration.z * 100;

    delay(20);
  }

  // float gyroOffsetX = sumGyroX / numSamples;
  // float gyroOffsetY = sumGyroY / numSamples;
  // float gyroOffsetZ = sumGyroZ / numSamples;

  accelOffsetX = (sumX / numSamples);
  accelOffsetY = (sumY / numSamples);
  accelOffsetZ = (sumZ / numSamples)*100;

  Serial.println("Calibration complete.");
  Serial.print("Gyro offsets: X=");
  Serial.print(gyroOffsetX);
  Serial.print(", Y=");
  Serial.print(gyroOffsetY);
  Serial.print(", Z=");
  Serial.println(gyroOffsetZ);

  Serial.print("Offsets - X: ");
  Serial.print(accelOffsetX);
  Serial.print(", Y: ");
  Serial.print(accelOffsetY);
  Serial.print(", Z: ");
  Serial.println(accelOffsetZ);
}


bool esValido(int x, int y) {
    return x >= 0 && x < MAP_SIZE && y >= 0 && y < MAP_SIZE && mapa[x][y] != '1';
}

float heuristica(int x1, int y1, int x2, int y2) {
    // Distancia de Manhattan
    return abs(x1 - x2) + abs(y1 - y2);
}

void obtenerVecinos(int x, int y, int vecinos[][2], int& count) {
    // Movimientos posibles: arriba, abajo, izquierda, derecha
    int posiblesVecinos[4][2] = {{x - 1, y}, {x + 1, y}, {x, y - 1}, {x, y + 1}};
    count = 0;
    for (int i = 0; i < 4; ++i) {
        if (esValido(posiblesVecinos[i][0], posiblesVecinos[i][1])) {
            vecinos[count][0] = posiblesVecinos[i][0];
            vecinos[count][1] = posiblesVecinos[i][1];
            ++count;
        }
    }
}



// Function to store the path coordinates in an array
void almacenarInstrucciones(Nodo* objetivo) {
    Nodo* nodo = objetivo;
    pathLength = 0;
    while (nodo != nullptr && pathLength < MAX_PATH_LENGTH) {
        path[pathLength][0] = nodo->x;
        path[pathLength][1] = nodo->y;
        pathLength++;
        nodo = nodo->padre;
    }
}

void AEstrella(int inicioX, int inicioY, int objetivoX, int objetivoY) {
    // Inicializar estructuras
    Nodo* abierta[MAP_SIZE * MAP_SIZE];  // Abierta
    bool cerrada[MAP_SIZE][MAP_SIZE] = {false};  // Cerrada
    int abiertaCount = 0;

    // Nodo inicial
    Nodo* inicial = new Nodo{inicioX, inicioY, 0, heuristica(inicioX, inicioY, objetivoX, objetivoY), 0, nullptr};
    inicial->costoF = inicial->costoG + inicial->costoH;
    abierta[abiertaCount++] = inicial;

    while (abiertaCount > 0) {
        // Encontrar el nodo con el menor costoF
        int minIndex = 0;
        for (int i = 1; i < abiertaCount; ++i) {
            if (abierta[i]->costoF < abierta[minIndex]->costoF) {
                minIndex = i;
            }
        }
        Nodo* actual = abierta[minIndex];
        abierta[minIndex] = abierta[--abiertaCount];

        // Si llegamos al objetivo
        if (actual->x == objetivoX && actual->y == objetivoY) {
            Serial.println("Camino encontrado:");
            almacenarInstrucciones(actual);  // Call the function to store the path coordinates
            for (int i = pathLength - 1; i >= 0; i--) {
                Serial.print("(");
                Serial.print(path[i][0]);
                Serial.print(", ");
                Serial.print(path[i][1]);
                Serial.print(") ");
            }
            Serial.println();
            return;
        }

        // Marcar el nodo actual como cerrado
        cerrada[actual->x][actual->y] = true;

        // Obtener vecinos
        int vecinos[4][2];
        int vecinosCount;
        obtenerVecinos(actual->x, actual->y, vecinos, vecinosCount);
        for (int i = 0; i < vecinosCount; ++i) {
            int nx = vecinos[i][0];
            int ny = vecinos[i][1];

            if (cerrada[nx][ny]) continue;

            float nuevoCostoG = actual->costoG + 1;  // Costo de moverse a la celda vecina (puede ajustarse)
            float nuevoCostoH = heuristica(nx, ny, objetivoX, objetivoY);
            float nuevoCostoF = nuevoCostoG + nuevoCostoH;

            // Crear nodo vecino
            Nodo* vecinoNodo = new Nodo{nx, ny, nuevoCostoG, nuevoCostoH, nuevoCostoF, actual};

            // Insertar en abierta
            abierta[abiertaCount++] = vecinoNodo;
        }
    }
    Serial.println("No se encontró un camino.");
}