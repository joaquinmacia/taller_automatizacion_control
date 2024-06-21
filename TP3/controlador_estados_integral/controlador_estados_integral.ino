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
float thita_medido = 0;
float phi_medido = 0;
float dthita_medido = 0;
float phi_eq = 90; 
float phi_ref = 90;
float thita_ref = 0;
float T = 1/Frec_muestreo;

//Observador
float thita_next = 0;
float thita_act = 0;
float dthita_next = 0;
float dthita_act = 0;
float phi_next = 0;
float phi_act = 0;
float dphi_next = 0;
float dphi_act = 0;

//Controlador por var. estado
float u_act = 0;
float q_next = 0;
float q_act = 0;
float Ad[4][4] = {{1.0000, 0.0100, 0.0000, 0.0000},{-0.6500, 0.9739, -2.0454, -0.2147},{0.0000, 0.0000, 1.0000, 0.0100},{0.0000, 0.0000, -2.9220, 0.6933}};
float Cd[2][4] = {{1,0,0,0},{0,0,1,0}};
float L[4][2] = {{1.5155, -0.2147},{54.7727, -27.9965},{0.0000, 1.2349},{0.0000, 18.5863}};
float K[5] = {-0.7152  ,  0.0879 ,   0.1525 ,  -0.0811 ,  0.0133};//{-0.8715  ,  0.0429  ,  0.5018 ,  -0.0393 ,  0.0093};//{-0.9757, 0.0129, 0.7346, -0.0115, -0.0053}; // {-0.9757, 0.0129, 0.7346, -0.0115};
float F[2] = {0.000, -0.2654}; 
float H[1] = {-0.1};
float Bd[4] = {0, 2.0454, 0, 2.9220};

//Offset: Acceleration X: -0.64, Y: -0.02, Z: 7.86 m/s^2 (MEDIDO)
float offset_aceleracion_Y = 0.27;
float offset_aceleracion_Z = 9.8 - 7.9;
float offset_gyro_X = 0;


void setup() {
  Serial.begin(115200); //Inicializacion de puerto serie
  pinMode(PWMPin, OUTPUT);  //Configuracion del pin 9 como salida

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
  
  PWM_50Hz();   //Configuracion e inicializacion del timer 1 para generar PWM 
  angle_2_servo(90);
  delay(1000);
}

void loop() {
  
  time1 = millis();
  /*
  while (Serial.available() > 0) {
    phi_ref = Serial.read();
  }*/
  
  thita_medido = angle_IMU();  
  phi_medido = pote_2_angle();
  dthita_medido = (g.gyro.x) * 180 / pi; 

  //Calcula los valores actuales
  thita_next = Ad[0][0] * thita_act + Ad[0][1] * dthita_act + Ad[0][2] * phi_act + Ad[0][3] * dphi_act  + L[0][0] * (thita_medido - thita_act) + L[0][1] * (phi_medido - phi_act) + Bd[0] * u_act;
  dthita_next = Ad[1][0] * thita_act + Ad[1][1] * dthita_act + Ad[1][2] * phi_act + Ad[1][3] * dphi_act  + L[1][0] * (thita_medido - thita_act) + L[1][1] * (phi_medido - phi_act) + Bd[1] * u_act;
  phi_next = Ad[2][0] * thita_act + Ad[2][1] * dthita_act + Ad[2][2] * phi_act + Ad[2][3] * dphi_act  + L[2][0] * (thita_medido - thita_act) + L[2][1] * (phi_medido - phi_act) + Bd[2] * u_act;  
  dphi_next = Ad[3][0] * thita_act + Ad[3][1] * dthita_act + Ad[3][2] * phi_act + Ad[3][3] * dphi_act  + L[3][0] * (thita_medido - thita_act) + L[3][1] * (phi_medido - phi_act) + Bd[3] * u_act;


  //Calculo accion control antes de actualizar los valores asi tengo los _act y los _next
  //act=k-1, next=k
  //float ek_thita_act = (thita_ref - thita_act);
  //float ek_thita_next = (thita_ref - thita_next);
  float ek_phi_next = (phi_ref - phi_next);
  q_next = q_act + ek_phi_next;
  //u_act = (K[0]*thita_next + K[1]*dthita_next + K[2]*(phi_next - phi_eq) + K[3]*dphi_next) + H[0]*(ek_thita_act + T*ek_thita_next) + H[1]*(ek_phi_act + T*ek_phi_next);
  
  u_act = (K[0]*thita_next + K[1]*dthita_next + K[2]*(phi_next - phi_eq) + K[3]*dphi_next) + K[4]*q_next;


  //Actualiza los valores
  thita_act = thita_next;
  dthita_act = dthita_next;
  phi_act = phi_next;
  dphi_act = dphi_next;
  q_act = q_next;

  //Aplico accion de control
  angle_2_servo(phi_eq + u_act);

  //matlab_send(thita_medido, thita_next,dthita_medido,dthita_next,phi_medido,phi_next,dphi_next,phi_ref);
  Serial.println(phi_next);
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

  angulo_x0_acel = atan2(a.acceleration.y + offset_aceleracion_Y, a.acceleration.z + offset_aceleracion_Z);

  angulo_filtro_complementario_actual = alpha * (angulo_filtro_complementario_anterior + (g.gyro.x + offset_gyro_X) * 1/(float)(Frec_muestreo)) + (1 - alpha) * angulo_x0_acel;
  angulo_filtro_complementario_anterior = angulo_filtro_complementario_actual;

  return angulo_filtro_complementario_deg = (angulo_filtro_complementario_actual * 180) / pi;
}

float pote_2_angle (){

  float lectura_pote = analogRead(SensorPin);
      //Se limita el rango de valores del potenciometro para un rango de entre -90° y 90°
  //if (lectura_pote <= Lectura_pote_low)
    //lectura_pote = Lectura_pote_low;
  //if (lectura_pote >= Lectura_pote_high)
    //lectura_pote = Lectura_pote_high; 

  float aux2 = map(lectura_pote, Lectura_pote_low, Lectura_pote_high, 0, 180);  

  return aux2;

}

//Retorna el valor de OICRA
void angle_2_servo(float angle){
    if (angle <= 30){
      angle = 30;
    }
    else if (angle >=150){
      angle = 150;
    }
    OCR1A = map(angle, 0, 180, OCR1A_max, OCR1A_min); 
}

void PWM_50Hz(){
  //Configuracion de los registros para un Fast PWM de 50Hz

  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1A = (1 << COM1A1) | (1 << WGM11) ;
  TCCR1B = (1 << CS11) | (1 << WGM13) | (1 << WGM12) ;
  ICR1 = 39999;
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