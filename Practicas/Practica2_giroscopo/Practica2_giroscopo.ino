#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h> 

Adafruit_MPU6050 mpu;


#define Frec_muestreo 100
unsigned long time1 = 0;
unsigned long time2 = 0;
int contador = 0;
float angulo_x0 = 0;
float angulo_x1 = 0;
float pi = 3.1415926;
float offset = 0.07;

void setup(void) {
	Serial.begin(115200);

	// Try to initialize!
	if (!mpu.begin()) {
		Serial.println("Failed to find MPU6050 chip");
		while (1) {
		  delay(10);
		}
	}
	Serial.println("MPU6050 Found!");

	// set accelerometer range to +-8G
	mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

	// set gyro range to +- 500 deg/s
	mpu.setGyroRange(MPU6050_RANGE_500_DEG);

	// set filter bandwidth to 5-10-21-44-94-184-260 Hz
	mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);

	delay(100);
}

void loop() {
	
  time1 = micros();

  //Como sacar el angulo X (usando giroscopos):
  //mido a 100Hz e integro la velocidad angular para sacar el grado
  //integro en discreto: 
  // tita_1 = tita_0 + w_0 * tiempo
  // tita_2 = ...
  // todo esto lo hago en un determinado ciclo y lo saco por serial print, a 100Hz.


 sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  angulo_x1 = angulo_x0 + (g.gyro.x + offset) * 1/(float)(Frec_muestreo);
  angulo_x0 = angulo_x1;
  float angulo_x1_deg = (angulo_x1 * 180) / pi; 
  
  if (contador == 100){
    Serial.println(angulo_x1_deg);
    contador = 0;
  }

  
	// Matlab send
  //matlab_send(d1,d2,d3);


  int aux = 1000000/Frec_muestreo;
  time2 = micros();
  contador++;
  delayMicroseconds(aux - (time2 - time1));  

}


void matlab_send(float dato1, float dato2, float dato3){
  Serial.write("abcd");
  byte * b = (byte *) &dato1;
  Serial.write(b,4);
  b = (byte *) &dato2;
  Serial.write(b,4);
  b = (byte *) &dato3;
  Serial.write(b,4);
}
