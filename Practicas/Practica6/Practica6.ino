#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define PWMPin 9 
#define SensorPin A0
#define Lectura_pote_low  146
#define Lectura_pote_high 862
#define Frec_muestreo 100

Adafruit_MPU6050 mpu;

unsigned long time1 = 0;
unsigned long time2 = 0;
int val = 0;
float deg = 0;
int OCR1A_min = 1100;
int OCR1A_max = 4900;
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
float thita_next = 0;
float thita_act = 0;
float w_act = 0;
float w_next = 0;
float Ad[2][2] = {{1, 0.02},{-1.2250, 0.9475}};
float Cd[2] = {1,0};
float L[2] = {0.8475,6.5503};

//Offset: Acceleration X: -0.64, Y: -0.02, Z: 7.86 m/s^2 (MEDIDO)
float offset_aceleracion_Y = 0.27;
float offset_aceleracion_Z = 9.8 - 7.9;


void setup() {
  Serial.begin(115200); //Inicializacion de puerto serie

  TCNT1 = 0;

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
  

}

void loop() {
  
  time1 = millis();

  float thita_medido = angle_IMU();  
  float w = (g.gyro.x) * 180 / pi;    
  thita_next = Ad[0][0] * thita_act + Ad[0][1] * w_act + L[0] * (thita_medido - thita_act);
  w_next = Ad[1][0] * thita_act + Ad[1][1] * w_act + L[1] * (thita_medido - thita_act);
  w_act = w_next;  
  thita_act = thita_next;

  matlab_send(thita_medido, thita_next,w_next,w);
  
  int aux = 1000/Frec_muestreo;
  time2 = millis();
  
  delay(aux - (time2 - time1));   //Delay necesario para el muestreo a la frecuencia Frec_muestreo
}



void matlab_send(float dato1, float dato2, float dato3,float dato4){
  Serial.write("abcd");
  byte * b = (byte *) &dato1;
  Serial.write(b,4);
  b = (byte *) &dato2;
  Serial.write(b,4);
  b = (byte *) &dato3;
  Serial.write(b,4);
  b = (byte *) &dato4;
  Serial.write(b,4);
}

float angle_IMU(){
  
  mpu.getEvent(&a, &g, &temp);

  angulo_x0_acel = atan2(a.acceleration.y + offset_aceleracion_Y, a.acceleration.z + offset_aceleracion_Z);

  angulo_filtro_complementario_actual = alpha * (angulo_filtro_complementario_anterior + g.gyro.x * 1/(float)(Frec_muestreo)) + (1 - alpha) * angulo_x0_acel;
  angulo_filtro_complementario_anterior = angulo_filtro_complementario_actual;

  return angulo_filtro_complementario_deg = (angulo_filtro_complementario_actual * 180) / pi;
}
