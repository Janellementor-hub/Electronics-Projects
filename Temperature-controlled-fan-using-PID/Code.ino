#include <LiquidCrystal.h> 
#include <math.h> 

 
//Pin Assignments 
const int analogPin = A0; //Reads temperature from thermistor 
const int fanPin = 3; //Controls the fan via PWM 

//Variables for PID controller 
double setpoint = 24.0; //desired value  

//Define PID parameters 
double Kp = 10.0; //proportional gain 
double Ki = 0.0; //integral gain 
double Kd = 0.1; //derivative gain  

 
float previous_error = 0; //Used to compute the derivative term 
float integral = 0; // Initializes the integral term 
unsigned long last_time = 0; //stores last loop timestamp in ms 


//LCD pin connections RS, E, D4, D5, D6, D7 
LiquidCrystal lcd(6, 7, 8, 9, 10, 11); 

 
//Declare variables to store values fixed resistor, VCC, max ADC value, thermistor, 25degrees in Kelvin, BETA value for thermistor 
const float fixed_resistor = 9960.0; //measure this actual value 
const float VCC = 5.04; 
const float MAX_ADC = 1023.0; // ADC resolution (0...1023) 
const float thermistor_resistor = 10910.0; //measure this actual value 
const float Kelvin_value = 298.15; // 25 degrees in Kelvin (25 + 273.15) 
const float BETA = 3981.51; //Beta constant computed from datasheet (0 degrees, 32650 ohms to 150 degrees, 185.97ohms) 

 
//average samples 
const int SAMPLES = 20; 
const int Samples_delay_ms = 10; 

 
void setup(){ 
pinMode(fanPin, OUTPUT); 
Serial.begin(9600); 
Serial.println("Temperature: "); 
Serial.print(temp); 

lcd.begin(16,2); 
last_time = millis(); 
} 

void loop(){ 

//read all the raw analog values 
long sum = 0; 
for(int i = 0; i < SAMPLES; i++){ 
sum+= analogRead(analogPin); // reads the analogPin for the analog voltages 
delay(Samples_delay_ms); 
} 

//store the raw values/analog voltage 
float raw_ADC_values = sum / (float)SAMPLES;  

//convert the raw analog voltage values to a digital voltage 
float voltage = raw_ADC_values * (VCC/MAX_ADC); 

 //error message 

if(voltage <= 0.0001){ 
Serial.println("Sensor error "); 
lcd.setCursor(0,0); 
lcd.print("Sensor Error."); 
delay(1000); 
return; 
} 

//compute thermistor resistance 
float thermistor_resistance = fixed_resistor * (VCC/voltage - 1.0); 

//convert thermistor resistance to temperature in Celcius 
float InRatio = log(thermistor_resistance/thermistor_resistor); 
float invT = (1.0/Kelvin_value) + (InRatio/BETA); 
float temp_in_Kelvin = 1.0/invT; 
float temp_in_Celcius = temp_in_Kelvin - 273.15;  

//Time step calculation 
unsigned long now = millis(); 
float time_difference = (now - last_time) / 1000.0; //Convert ms to seconds; dt is the time difference between current and previous loop iteration 
if(time_difference <= 0.00){ 
time_difference = 0.001; // this is to prevent dividing by 0 
} 

last_time = now; 

//PID error terms 
float error = temp_in_Celcius - setpoint;  
integral += error * time_difference; // integral accumulates error over time 
//anti wind up 
if(integral > 300) integral = 300; 
if(integral < -300) integral = -300; 
 

float derivative = (error - previous_error) / time_difference; // rate of change of error  

// PID output 
float output = Kp * error + Ki * integral + Kd * derivative; 
output = constrain(output, 0, 255); 
int pwm_output = (int)output; 

//Apply to the fan 
analogWrite(fanPin, pwm_output); 

//Update the error 
previous_error = error; 


Serial.print(" TempC: "); 
Serial.println(temp_in_Celcius, 2); 
Serial.print( "Fan: ";
Serial.println(pwm_output);

//Display on LCD 
lcd.setCursor(0, 0); 
lcd.print("Temp: "); 
lcd.print(temp_in_Celcius, 1); 
lcd.setCursor(0, 1); 
lcd.print("Fan: "); // clear old digits 
lcd.setCursor(5, 1); 
lcd.print(pwm_output); 

delay(500); 

} 
