#include <SPI.h>
#include <Pixy.h>

//Variables for motor controller
#define pwm 4
#define pwm2 5

#define dir1 2
#define dir2 3

#define poke_pin 10




//Variables for Pixy
Pixy pixy;
float x,y,x1,y1,x2,y2;
static int i = 0;
int j;
float PWM_output;
uint16_t blocks;

int buffer_count_main = 0;
int buffer_time=80;
int buffer_count = buffer_time/2;
int mode_oscillation;
int count_oscillation;


//Variables for PID function
float pid_p_gain = 2;                                 
float pid_i_gain = 0.02;                      
float pid_d_gain = 0.15;


float scaling_factor = 0.6375; //0.4375;

float pid_error, pid_i_mem, pid_output, pid_last_d_error, x_predicted_coordinate, x_predicted_distance;

float pid_setpoint = 163;

int strike_distance_coordinate = 120;
int pull_distance_coordinate = 190;
int poke_counter=0;
int poke_loc=0;
int sweep_loc=1;
int sweep_count=0;



void setup()
{
 //Serial.begin(115200);
  //Serial.print("Starting...\n");

  pinMode(dir1,OUTPUT);
  pinMode(pwm,OUTPUT);
  pinMode(pwm2,OUTPUT);

  pinMode(dir2,OUTPUT);
  pinMode(poke_pin, OUTPUT);
  digitalWrite(poke_pin, HIGH);
  

   
  pixy.init();


 //Serial.begin(115200);
  analogWrite(pwm, LOW);
  analogWrite(pwm2,LOW);
  
  delay(1000);
  //Serial.println("ON");
  
}

void loop()
{ 

         delay(20);
         Center() ;

                    if(x > 128 && x < 198 && sweep_count > 1 && sweep_loc == 0){
          if(x > pid_setpoint){
          digitalWrite(dir1,HIGH);
          digitalWrite(dir2,HIGH);
          analogWrite(pwm, 150);
          analogWrite(pwm2,150);
          delay(130);
          digitalWrite(dir1,LOW);
          digitalWrite(dir2,LOW);
          delay(260);
          analogWrite(pwm, 0);
          analogWrite(pwm2,0);
          }
          else{
          digitalWrite(dir1,LOW);
          digitalWrite(dir2,LOW);
          analogWrite(pwm, 150);
          analogWrite(pwm2,150);
          delay(130);
          digitalWrite(dir1,HIGH);
          digitalWrite(dir2,HIGH);
          delay(260);
          analogWrite(pwm, 0);
          analogWrite(pwm2,0); 
          }  
          sweep_count = 0; 
          sweep_loc = 1;
        }
       if (sweep_loc == 0){
        sweep_count++;
       }
        buffer();
        
        //Serial.print(x);
       // Serial.print(",");
       // Serial.println(y);

//65,114,82
   if (poke_counter>15){
    if (poke_loc==0){
      if (y>65 && y<90){
        poke();

        sweep_loc=0;
        poke_loc=1;

      }
      
      }else{
        if (y>110||y<45){
          pull();
          sweep_loc=1;
          poke_loc=0;
          sweep_count = 0;
        }
    }
   poke_counter=0;   
   }
   poke_counter++;


}



//Obtain the current X,Y coordinates of the puck from PixyCam
void GetCoordinates()  {
    blocks = pixy.getBlocks();
   // Serial.println(blocks);
      if (blocks) 
      { 
        for (j=0; j<blocks; j++)
        {
    
          x = pixy.blocks[j].x;
          y = pixy.blocks[j].y;
        }       
      }
       else 
       {
        x = -1;
        y = -1;
       }
  return ;
}


//Standard PID function to control the DC motors
void PID(){

  pid_error = x - pid_setpoint ;

  pid_i_mem += pid_i_gain * pid_error;                                 //Calculate the I-controller value and add it to the pid_i_mem variable
  if(pid_i_mem > 400)pid_i_mem = 400;                                       //Limit the I-controller to the maximum controller output
  else if(pid_i_mem < -400)pid_i_mem = -400;
  //Calculate the PID output value
  
  pid_output = pid_p_gain * pid_error + pid_i_mem + pid_d_gain * (pid_error - pid_last_d_error);
  
  if(pid_output > 400)pid_output = 400;    
  
  //Limit the PI-controller to the maximum controller output
  else if(pid_output < -400)pid_output = -400;


   return ;
}


//Control the motors to keep the robot's hockey stick pointed directly at the puck
void  Center(){
GetCoordinates();
//Serial.println("Coord");

if (x != -1 && y != -1){
//Serial.print("True");
//predict();
PID();

PWM_output = pid_output * scaling_factor;
//Serial.println(PWM_output);
if (PWM_output > 0) {
  digitalWrite(dir1, HIGH);
  digitalWrite(dir2, HIGH);
}
else {
  PWM_output *= -1;
  digitalWrite(dir1, LOW);
  digitalWrite(dir2, LOW);
}

  analogWrite(pwm, PWM_output);
  analogWrite(pwm2, PWM_output);
}

else {
  analogWrite(pwm, 0);
  analogWrite(pwm2, 0);

}


  return ;
}







void poke(){
  digitalWrite(poke_pin, LOW);
 Serial.println("POKE");
}




void pull(){
  digitalWrite(poke_pin, HIGH);
  Serial.println("PULL");
}

void sweep1(){
 
  digitalWrite(dir1, HIGH);
  digitalWrite(dir2, HIGH);
  analogWrite(pwm, 255);
  analogWrite(pwm2, 255);

}


void sweep2(){

  
  digitalWrite(dir1, LOW);
  digitalWrite(dir2, LOW);
  analogWrite(pwm, 255);
  analogWrite(pwm2, 255);
}


void predict(){
  x_predicted_distance = x-x1;
  x_predicted_coordinate = x + x_predicted_distance;
  
return;
}


void buffer(){
         //Serial.print("Center");
      
      if(x == -1 && y == -1){
        if(buffer_count_main > 20){
          if (poke_loc=1){
           pull();
          poke_loc=0;
        }
       
        
        if(buffer_count > 0){
          digitalWrite(dir1, HIGH);
          digitalWrite(dir2, HIGH);
          analogWrite(pwm, 50);
          analogWrite(pwm2, 50);
          if(buffer_count > buffer_time){
            buffer_count=-buffer_time;
          }
          
        }
        else{
          digitalWrite(dir1, LOW);
          digitalWrite(dir2, LOW);
          analogWrite(pwm, 50);
          analogWrite(pwm2, 50);
        }
       buffer_count++;
        

        }
        buffer_count_main++;
      }
      else{
           buffer_count_main = 0;
      }
      return;
}
