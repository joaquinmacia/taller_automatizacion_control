#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
 
#define SensorPin A0
#define Frec_muestreo 100

Adafruit_MPU6050 mpu;

unsigned long time1 = 0;
unsigned long time2 = 0;
float angle;
float angle_map;
int lectura_pote;
float angulo_x0_giro = 0;
float angulo_x1_giro = 0;
float angulo_x0_acel = 0;
float angulo_filtro_complementario = 0;
float alpha = 0.98;
float angulo_filtro_complementario_deg = 0;
float angulo_filtro_complementario_anterior;
float angulo_filtro_complementario_actual;
float pi = 3.1415926;
float angle_pendulo = 0;
sensors_event_t a, g, temp;
float thita_medido = 0;
float phi_medido = 0;
float dthita_medido = 0; 
float phi_ref = 90;

//Observador
float thita_next = 0;
float thita_act = 0;
float dthita_next = 0;
float dthita_act = 0;
float phi_next = 0;
float phi_act = 0;
float dphi_next = 0;
float dphi_act = 0;



//Offset: Acceleration X: -0.64, Y: -0.02, Z: 7.86 m/s^2 (MEDIDO)
float offset_aceleracion_Y = 0.27;
float offset_aceleracion_Z = 9.8 - 7.9;
float offset_gyro_X = 0;

void setup() {
  Serial.begin(115200); //Inicializacion de puerto serie
  
    // Try to initialize!
	if (!mpu.begin()) {
		//Serial.println("Failed to find MPU6050 chip");
		while (1) {
		  delay(10);
		}
	}
	//Serial.println("MPU6050 Found!");

	// set accelerometer range to +-8G
	mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

	// set gyro range to +- 500 deg/s
	mpu.setGyroRange(MPU6050_RANGE_500_DEG);

	// set filter bandwidth to 5-10-21-44-94-184-260 Hz
	mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);
  angulo_filtro_complementario_anterior = 0;
  
  //Autocalibracion
  autocalibracion_IMU();

  //Configuracion e inicializacion del timer 1 para generar PWM 
  delay(1000);
}

void loop() {
  
  time1 = millis();
  /*
  while (Serial.available() > 0) {
    phi_ref = Serial.read();
  }*/
  

  angle = angle_IMU();
  Serial.println(angle);  

  //matlab_send(thita_medido, thita_next,dthita_medido,dthita_next,phi_medido,phi_next,dphi_next,phi_ref);
  
  int aux = 1000/Frec_muestreo;
  time2 = millis();
  
  delay(aux - (time2 - time1));   //Delay necesario para el muestreo a la frecuencia Frec_muestreo
}



void matlab_send(float dato1, float dato2, float dato3,float dato4,float dato5,float dato6,float dato7,float dato8){
  Serial.write("abcd");
  byte * b = (byte *) &dato1;
  Serial.write(b,4);
  b = (byte *) &dato2;
  Serial.write(b,4);
  b = (byte *) &dato3;
  Serial.write(b,4);
  b = (byte *) &dato4;
  Serial.write(b,4);
  b = (byte *) &dato5;
  Serial.write(b,4);
  b = (byte *) &dato6;
  Serial.write(b,4);
  b = (byte *) &dato7;
  Serial.write(b,4);
  b = (byte *) &dato8;
  Serial.write(b,4);
}

float angle_IMU(){
  
  mpu.getEvent(&a, &g, &temp);
  float acel_y = a.acceleration.y + offset_aceleracion_Y;
  float acel_z = a.acceleration.z + offset_aceleracion_Z;

  //Serial.println(acel_y);
  //Serial.println(acel_z);

  angulo_x0_acel = atan2(acel_y, acel_z);

  angulo_filtro_complementario_actual = alpha * (angulo_filtro_complementario_anterior + (g.gyro.x + offset_gyro_X) * 1/(float)(Frec_muestreo)) + (1 - alpha) * angulo_x0_acel;
  angulo_filtro_complementario_anterior = angulo_filtro_complementario_actual;

  return angulo_filtro_complementario_deg = (angulo_filtro_complementario_actual * 180) / pi;
}

float autocalibracion_IMU(){
  
  mpu.getEvent(&a, &g, &temp);
  float aux_z=0;
  float aux_y=0;
  float aux_x = 0;

  for (int i = 0; i < 10; i++){
    aux_z += a.acceleration.z;
    aux_y += a.acceleration.y;
    aux_x += g.gyro.x;
  }

  aux_z = aux_z / 10;
  aux_y = aux_y / 10;
  aux_x = aux_x / 10;

  offset_aceleracion_Z = 9.8 - aux_z;
  offset_aceleracion_Y = 0 - aux_y;
  offset_gyro_X = 0 - aux_x;
 
}
