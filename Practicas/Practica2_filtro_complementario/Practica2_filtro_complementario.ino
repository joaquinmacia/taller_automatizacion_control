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
float alpha = 0.98;
float angulo_filtro_complementario_deg = 0;
float angulo_filtro_complementario_anterior;
float angulo_filtro_complementario_actual;
float pi = 3.1415926;

//Offset: Acceleration X: -0.64, Y: -0.02, Z: 7.86 m/s^2 (MEDIDO)
float offset_aceleracion_Y = 0.02;
float offset_aceleracion_Z = 9.8 - 7.86;

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
  angulo_filtro_complementario_anterior = 0;

	delay(100);
}

void loop() {
	
  time1 = micros();

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  angulo_x0_acel = atan2(a.acceleration.y + offset_aceleracion_Y, a.acceleration.z + offset_aceleracion_Z);

  angulo_filtro_complementario_actual = (1 - alpha) * (angulo_filtro_complementario_anterior + g.gyro.x * 1/(float)(Frec_muestreo)) + alpha * angulo_x0_acel;
  angulo_filtro_complementario_anterior = angulo_filtro_complementario_actual;

  angulo_filtro_complementario_deg = (angulo_filtro_complementario_actual * 180) / pi; 
  
  if (contador == 100){
    Serial.println(angulo_filtro_complementario_deg);
    contador = 0;
  }
  
  int aux = 1000000/Frec_muestreo;
  time2 = micros();
  contador++;
  delayMicroseconds(aux - (time2 - time1));  

}


