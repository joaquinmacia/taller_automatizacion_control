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


  //otra forma de obtener el anglo del eje X (con acelerometros):

  // Si lo dejo quieto, va a medir la aceleracion de la gravedad. El vector gravedad es siempre normal al piso.
  // Si muevo la placa (la inclino) el vector gracedad queda todo sobre otro eje. 
  // si esta a 45 grados: descomposicion con trigonometricas.
  // Ahi esta la relacion entre lo que mide cada acelerometro en cada eje y la inclinacion de la placa (arcotg del angulo con atan2()). 

  // tita_acelerometro = arctg(acelerometro_y / acelerometro_z)

  angulo_x0 = atan2(a.acceleration.y, a.acceleration.z);

  float angulo_x0_deg = (angulo_x0 * 180) / pi;
  
  if (contador == 100){
    Serial.println(angulo_x0_deg);
    contador = 0;
  }
  

  //junto las dos estimaciones haciendo una ponderacion:

  // tita = alfa * tita_acelerometro + (1-alfa) * tita_giroscopo

  //ej alfa = 0.02, le doy poca importancia en la medicion instantanea al acelerometro.

  
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
