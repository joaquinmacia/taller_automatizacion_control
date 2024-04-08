#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h> 

Adafruit_MPU6050 mpu;


#define Frec_muestreo 100
unsigned long time1 = 0;
unsigned long time2 = 0;
int contador = 0;
float angulo_x0_giro = 0;
float angulo_x1_giro = 0;
float angulo_x0_acel = 0;
float angulo_filtro_complementario = 0;
float alfa = 0.02;
float angulo_filtro_complementario_deg = 0;
float pi = 3.1415926;

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

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  angulo_x1_giro = angulo_x0_giro + g.gyro.x * 1/(float)(Frec_muestreo);
  angulo_x0_giro = angulo_x1_giro;


  angulo_x0_acel = atan2(a.acceleration.y, a.acceleration.z);


  //junto las dos estimaciones haciendo una ponderacion:

  // tita = alfa * tita_acelerometro + (1-alfa) * tita_giroscopo

  //ej alfa = 0.02, le doy poca importancia en la medicion instantanea al acelerometro.

  angulo_filtro_complementario = alfa * (angulo_x0_acel) + (1-alfa) * angulo_x1_giro;

  angulo_filtro_complementario_deg = (angulo_filtro_complementario * 180) / pi; 
  
  if (contador == 100){
    Serial.println(angulo_filtro_complementario_deg);
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

