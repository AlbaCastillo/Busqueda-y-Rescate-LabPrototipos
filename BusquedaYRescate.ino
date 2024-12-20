//Busqueda y Rescate 
#include <math.h>
#include <stdio.h>
#define MAP_SIZE 4  // Tamaño del mapa (ajustar según la resolución y entorno)
//const int MAP_SIZE = MAP_SIZE_one;
const float CELDA = 10;  // Tamaño de la celda en centimetros (ejm 20cm*20cm) (ajustar según el tamaño del robot)
char mapa[MAP_SIZE][MAP_SIZE];  // Matriz del mapa de ocupación
int robotX, robotY;            // Posición actual del robot en el mapa
int inicioX, inicioY;          // Posición inicial (punto de origen)
int objetoX, objetoY; // Posición del objeto negro (B)

/* Ejemplo de la Matriz
 1 : ocupado (obstáculo)
 0 : libre
 - : desconocido / No explorado
"R": Posición actual de un robot
"B": Objeto negro encontrado 

[-1, -1,  0,  0]
[-1, -1,  0,  1]
[-1,  B,  0,  0]
[-1,  R,  0,  0]
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

//Motors
#include <AFMotor.h>
AF_DCMotor motori(1); //Left motor - connected to terminal 1
AF_DCMotor motord(2); //Right motor - connected to terminal 2

void setup() {
  Serial.begin(9600);

  // Matriz - Mapa
  inicioX = 0;
  inicioY = 0;
  robotX = inicioX;
  robotY = inicioY;
  inicializarMapa();

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
  
  posX = 0; // Reiniciar distancia acumulada en X
  posY = 0; // Reiniciar distancia acumulada en X
  lastTime = millis(); // Reset the time

  while (posX <= CELDA){ // Avanzar hasta que haya pasado a la siguiente celda
    motori.run(FORWARD);
    motord.run(FORWARD);
    delay(5);

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
  motori.run(FORWARD);
  motord.run(FORWARD);
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
    motori.run(BACKWARD);
    motord.run(FORWARD);
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

  while (angle < targetAngle){ // Turn until the target angle is reached
    motori.run(FORWARD);
    motord.run(BACKWARD);
    delay(5);

    updateGyro();
  }
  stop();
  delay(100);

  // Actualizar la orientación acumulada
  Serial.print("menos 90");
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
    motori.run(BACKWARD);
    motord.run(FORWARD);
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

// Función para actualizar la aceleración angular y calcular el angulo
void updateGyro() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  unsigned long currentTime = millis();
  deltaTime = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;

  float gyroZ = g.gyro.z * (180 / PI); // rad/s a deg/s
  angle += gyroZ * deltaTime;
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
  float accelX = a.acceleration.x * (100);  // Aceleración en el eje X * (m/s^2  a cm/s^2  1m = 100cm)
  float accelY = a.acceleration.y * (100);  // Aceleración en el eje Y * (m/s^2  a cm/s^2)

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
  
  if (sensorValues[0] > 800 || sensorValues[1] > 800) {
    Serial.println("Objeto negro detectado."); 
    actualizarPosObjeto('B'); // Marca el objeto negro en la matriz  
    Obst = true;
  } else if (sensorValues[0] < 800 && sensorValues[1] < 800) {
    Serial.println("Obstáculo detectado.");
    actualizarPosObjeto('1'); // Marca un obstáculo en la matriz
    Obst = true;
  }
  
  // Regresar lo que avanzo para acercarse
  while (distance < distanceInicial ) {  // Retrocede hasta que vuelva a la posición inicial
    motori.run(BACKWARD);
    motord.run(BACKWARD);
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
    }
    else if (distance <= CELDA){
      stop();
      Serial.println("Encontro un obstaculo");
      Examinar();
    }
  }
  else if((robotX == 0) && (orientacion == 2)){
    right();
    if (distance > CELDA){
      unaCelda();
      right();
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




