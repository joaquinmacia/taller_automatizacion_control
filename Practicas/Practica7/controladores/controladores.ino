#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h> 

#define PWMPin 9 
#define SensorPin A0
#define Lectura_pote_low  146
#define Lectura_pote_high 862
#define Frec_muestreo 100

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
float thita_ref = 0;
float phi_ref = 90;
float thita_control = 0;
float u = 0;
float accion_control_ant = 90;
float error[2] = {0,0};
float D[2] = {0,0};
float I[2] = {0,0};
float error2[2] = {0,0};
float D2[2] = {0,0};
float I2[2] = {0,0};

//Controlador proporcional kp = 0.6
//Controlador proporcional integrativo kp = 0.6 ki = 10
//Controlador proporcional derivativo kp = 0.6 ki = 0.0001
float kp = 0.6; 
float ki = 0;
float kd = 0.0001;
float kp2 = 0.6; 
float ki2 = 0;
float kd2 = 0.0001;


float T = 0.01;

//Offset: Acceleration X: -0.64, Y: -0.02, Z: 7.86 m/s^2 (MEDIDO)
float offset_aceleracion_Y = 0.27;
float offset_aceleracion_Z = 9.8 - 7.9;

Adafruit_MPU6050 mpu;

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
  
  //Mediciones de los angulos
  float angle_measure = pote_2_angle();
  angle_pendulo = angle_IMU();

  //Controlador 2 
  //Calculo el error y actualizo el anterior
  error2[1] = error[0];
  error2[0] = phi_ref - angle_measure;

  float swap = I2[0];
  I2[0] = I2[1] + T/2 * error2[0] + T/2 * error2[1];
  I2[1] = swap;
  swap = D2[0];
  D2[0] = 2 * (error2[0] - error2[1])/T - D2[1];
  D2[1] = swap;

  //PID por Tustin
  thita_control = kp2 * error2[0] + ki2 * (I2[1] + T/2 * error2[0] + T/2 * error2[1]) + kd2 * (2 * (error2[0] - error2[1])/T - D2[1]); 

  //Controlador 1
  //Calculo el error y actualizo el anterior
  error[1] = error[0];
  error[0] = thita_control - angle_pendulo;

 //Calculo y actualizo valores de las derivadas e integrales;
  swap = I[0];
  I[0] = I[1] + T/2 * error[0] + T/2 * error[1];
  I[1] = swap;
  swap = D[0];
  D[0] = 2 * (error[0] - error[1])/T - D[1];
  D[1] = swap;

  //PID por Tustin
  u = kp * error[0] + ki * (I[1] + T/2 * error[0] + T/2 * error[1]) + kd * (2 * (error[0] - error[1])/T - D[1]);
  
  //Aplico la accion de control
  angle_2_servo(phi_ref + u);

  int aux = 1000/Frec_muestreo;
  time2 = millis();
  

  matlab_send(u, angle_measure, angle_pendulo);

  delay(aux - (time2 - time1));   //Delay necesario para el muestreo a la frecuencia Frec_muestreo
}


void PWM_50Hz(){
  //Configuracion de los registros para un Fast PWM de 50Hz

  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1A = (1 << COM1A1) | (1 << WGM11) ;
  TCCR1B = (1 << CS11) | (1 << WGM13) | (1 << WGM12) ;
  ICR1 = 39999;
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

void matlab_send(float dato1, float dato2, float dato3){
  Serial.write("abcd");
  byte * b = (byte *) &dato1;
  Serial.write(b,4);
  b = (byte *) &dato2;
  Serial.write(b,4);
  b = (byte *) &dato3;
  Serial.write(b,4);
}


float angle_IMU(){
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  angulo_x0_acel = atan2(a.acceleration.y + offset_aceleracion_Y, a.acceleration.z + offset_aceleracion_Z);

  angulo_filtro_complementario_actual = alpha * (angulo_filtro_complementario_anterior + g.gyro.x * 1/(float)(Frec_muestreo)) + (1 - alpha) * angulo_x0_acel;
  angulo_filtro_complementario_anterior = angulo_filtro_complementario_actual;

  return angulo_filtro_complementario_deg = (angulo_filtro_complementario_actual * 180) / pi;
}
