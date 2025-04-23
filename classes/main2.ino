

  //This is working without the need to open the serial monitor. However, I am not sure i am able to adjust 
//the pwm timers.
// action = [Bx, By, Bz, alpha, gamma, freq]
//potential issues
// - if  (myTransfer.available()) may cuz bugs because actions maybe empty or something
// - may need a better way to toggle the enable pin on all 6 drivers
// - there is no hard zero option as I got rid of typ
#include <AD9850.h>
#include "SerialTransfer.h"
SerialTransfer myTransfer;
float action[9]; //an array to store incoming data from python

#define PI 3.1415926535897932384626433832795

//actions
float Bx;
float By;
float Bz;
float alpha;
float gamma;
float rolling_frequency;
float psi;
float gradient_status;
float equal_field_status;
float acoustic_frequency;

float T_switch;
float phase_switch;
float t2;



int phase = 0; 



//other field vals
float Bx_roll;
float By_roll;
float Bz_roll;

float Bx_uniform;
float By_uniform;
float Bz_uniform;

float BxPer;
float ByPer;
float BzPer;
float c; 
float magnitude;


//other constants
float tim;
float t;
float omega;

float Bx_final;
float By_final;
float Bz_final;

float z0;
float x0;




//Coil 1 : +Y Brown
const int Coil1_PWMR = 2;
const int Coil1_PWML = 3;
const int Coil1_ENR = 26; //26
const int Coil1_ENL = 27; //27                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            333

//Coil 2 : +X Purple
const int Coil2_PWMR = 4;
const int Coil2_PWML = 5;
const int Coil2_ENR = 24; //24
const int Coil2_ENL = 25; //25

//Coil 3 : -Y Green
const int Coil3_PWMR = 6;
const int Coil3_PWML = 7;
const int Coil3_ENR = 22;
const int Coil3_ENL = 23;

//Coil 4: -X Blue
const int Coil4_PWMR = 8;
const int Coil4_PWML = 9;
const int Coil4_ENR = 32;
const int Coil4_ENL = 33;

//Coil 5 : +Z  Yellow
const int Coil5_PWMR = 10;
const int Coil5_PWML = 11;
const int Coil5_ENR = 30;
const int Coil5_ENL = 31;

//Coil 6 : -Z Orange
const int Coil6_PWMR = 12;
const int Coil6_PWML = 13;
const int Coil6_ENR = 28;
const int Coil6_ENL = 29;


//AD9850 Acoustic Module
const int W_CLK_PIN = 34;
const int FQ_UD_PIN = 36;
const int DATA_PIN = 38;
const int RESET_PIN = 40;


  
void setup()
{
  DDS.begin(W_CLK_PIN, FQ_UD_PIN, DATA_PIN, RESET_PIN);
  DDS.calibrate(124999500);

  cli();
  TCCR0B = (TCCR0B & 0b11111000) | 0x02; //7.81250[kHz] pin 13,4
  TCCR1B = (TCCR1B & 0b11111000) | 0x01; //31.37255 [kHz] pin 12,11
  TCCR2B = (TCCR2B & 0b11111000) | 0x01; //31.37255 [kHz] pin 10,9
  TCCR3B = (TCCR3B & 0b11111000) | 0x01; //31.37255 [kHz] pin 5,3,2
  TCCR4B = (TCCR4B & 0b11111000) | 0x01; //31.37255 [kHz] pin 8,7,6 
  sei();
  
  Serial.begin(115200);
  myTransfer.begin(Serial);


   //Coil1 Ouptut
  pinMode(Coil1_PWMR, OUTPUT);  
  pinMode(Coil1_PWML, OUTPUT);
  pinMode(Coil1_ENR, OUTPUT);  
  pinMode(Coil1_ENL, OUTPUT); 
  


  //Coil2 Output
  pinMode(Coil2_PWMR, OUTPUT);  
  pinMode(Coil2_PWML, OUTPUT);
  pinMode(Coil2_ENR, OUTPUT);  
  pinMode(Coil2_ENL, OUTPUT); 
  


  //Coil3 Output
  pinMode(Coil3_PWMR, OUTPUT);  
  pinMode(Coil3_PWML, OUTPUT);
  pinMode(Coil3_ENR, OUTPUT);  
  pinMode(Coil3_ENL, OUTPUT); 


  //Coil4 Output
  pinMode(Coil4_PWMR, OUTPUT);  
  pinMode(Coil4_PWML, OUTPUT);
  pinMode(Coil4_ENR, OUTPUT);  
  pinMode(Coil4_ENL, OUTPUT); 


  //Coil5 Output
  pinMode(Coil5_PWMR, OUTPUT);  
  pinMode(Coil5_PWML, OUTPUT);
  pinMode(Coil5_ENR, OUTPUT);  
  pinMode(Coil5_ENL, OUTPUT);
 

  //Coil6 Output
  pinMode(Coil6_PWMR, OUTPUT);  
  pinMode(Coil6_PWML, OUTPUT);
  pinMode(Coil6_ENR, OUTPUT);  
  pinMode(Coil6_ENL, OUTPUT);
  
  
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void set1(float DC1){
  digitalWrite(Coil1_ENR,HIGH);
  digitalWrite(Coil1_ENL,HIGH);

  if (DC1 > 0){
    analogWrite(Coil1_PWMR,abs(DC1)*255);
    analogWrite(Coil1_PWML,0);
  }
  else if (DC1 < 0){
    analogWrite(Coil1_PWMR,0);
    analogWrite(Coil1_PWML,abs(DC1)*255);
  }
  else {
    analogWrite(Coil1_PWMR,0);
    analogWrite(Coil1_PWML,0);
    
  }



}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void set2(float DC2){
  digitalWrite(Coil2_ENR,HIGH);
  digitalWrite(Coil2_ENL,HIGH);

  if (DC2 > 0){
    analogWrite(Coil2_PWMR,abs(DC2)*255);
    analogWrite(Coil2_PWML,0);
  }
  else if (DC2 < 0){
    analogWrite(Coil2_PWMR,0);
    analogWrite(Coil2_PWML,abs(DC2)*255);
  }
  else {
    analogWrite(Coil2_PWMR,0);
    analogWrite(Coil2_PWML,0);
    
  }
  

  
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void set3(float DC3){
  digitalWrite(Coil3_ENR,HIGH);
  digitalWrite(Coil3_ENL,HIGH);

  if (DC3 > 0){
    analogWrite(Coil3_PWMR,abs(DC3)*255);
    analogWrite(Coil3_PWML,0);
  }
  else if (DC3 < 0){
    analogWrite(Coil3_PWMR,0);
    analogWrite(Coil3_PWML,abs(DC3)*255);
  }
  else {
    analogWrite(Coil3_PWMR,0);
    analogWrite(Coil3_PWML,0);
    
  }

  

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void set4(float DC4){
  digitalWrite(Coil4_ENR,HIGH);
  digitalWrite(Coil4_ENL,HIGH);

  if (DC4 > 0){
    analogWrite(Coil4_PWMR,abs(DC4)*255);
    analogWrite(Coil4_PWML,0);
  }
  else if (DC4 < 0){
    analogWrite(Coil4_PWMR,0);
    analogWrite(Coil4_PWML,abs(DC4)*255);
  }
  else {
    analogWrite(Coil4_PWMR,0);
    analogWrite(Coil4_PWML,0);
   
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void set5(float DC5){
  digitalWrite(Coil5_ENR,HIGH);
  digitalWrite(Coil5_ENL,HIGH);

  if (DC5 > 0){
    analogWrite(Coil5_PWMR,abs(DC5)*255);
    analogWrite(Coil5_PWML,0);
  }
  else if (DC5 < 0){
    analogWrite(Coil5_PWMR,0);
    analogWrite(Coil5_PWML,abs(DC5)*255);
  }
  else {
    analogWrite(Coil5_PWMR,0);
    analogWrite(Coil5_PWML,0);
    
  }


 
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void set6(float DC6){
  digitalWrite(Coil6_ENR,HIGH);
  digitalWrite(Coil6_ENL,HIGH);

  if (DC6 > 0){
    analogWrite(Coil6_PWMR,abs(DC6)*255);
    analogWrite(Coil6_PWML,0);
  }
  else if (DC6 < 0){
    analogWrite(Coil6_PWMR,0);
    analogWrite(Coil6_PWML,abs(DC6)*255);
  }
  else {
    analogWrite(Coil6_PWMR,0);
    analogWrite(Coil6_PWML,0);

  }
  
 

}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



void loop()
{
    if  (myTransfer.available()){ // THIS IF STATEMENT MIGHT MESS UP EVERYTHING BELOW
      
              uint16_t message = 0;
              message = myTransfer.rxObj(action,message);               
    }

   
   //NEW LOGIC
   Bx_uniform = action[0];
   By_uniform = action[1];
   Bz_uniform = action[2];   
   alpha = action[3];
   gamma = action[4];
   rolling_frequency = action[5]; 
   psi = action[6]; 
   gradient_status = action[7];
   equal_field_status = action[8];
   acoustic_frequency = action[9];
   

   if (acoustic_frequency != 0){
      DDS.setfreq(acoustic_frequency, phase);
   }
   else{
      DDS.down();
   }
   
   
   omega = 2*PI*rolling_frequency;
   
   tim = micros() % 7812500;
   t = tim / 7812500;
   
   if (omega == 0){
       Bx_roll = 0;
       By_roll = 0;
       Bz_roll = 0;
      }
   else {
      //correct equations 7/8/23
      //Bx_roll = (-sin(alpha) * sin(omega*t)) + (-cos(alpha) * cos(gamma)  * cos(omega*t)); 
      //By_roll =  (cos(alpha) * sin(omega*t)) + (-sin(alpha) * cos(gamma) *  cos(omega*t)); 
      //Bz_roll = sin(gamma) * cos(omega*t);
      
      T_switch = 1/(4*omega/(2*PI));

      phase_switch = (sin(2*PI*t/T_switch )>= 0)? 1 : -1;
      float modTime = fmod(t,T_switch);
      t2 = abs(modTime  - T_switch/2)*2;
      //if (sin(2*PI*omega*t/T_switch) < 0 ){
         // phase_switch = -1;

        // }
      //else{
       // phase_switch =1;
       // }
      //t2 = phase_switch*t;

      //Bx_roll = (-sin(alpha) * sin(omega*t2))+ (-cos(alpha) * cos(gamma)  * cos(omega*t2));  
      //By_roll =  (cos(alpha) * sin(omega*t2))+ (-sin(alpha) * cos(gamma) *  cos(omega*t2)); 
      //Bz_roll = sin(gamma) * cos(omega*t2);

      //Bx_roll = (sin(omega*t2));  
      //By_roll =  0.5*(cos(omega*t2)); 
      //Bz_roll = 0.25;

      //this one is just like rolling but there's a constant field applied perpendicularly to the rotation axis and a tilt field applied along one direction
      // (takes a vector ((1-x0)*sin(wt)+x0, cos(wt), z0) and rotates it in 3D)
      z0 = 1;
      x0 = 0;
      Bx_roll = (-sin(alpha+(PI/2)*sin(omega*t2)) * (x0 + (1-x0)*sin(omega*t2))) + (-cos(alpha+(PI/2)*sin(omega*t2)) * cos(gamma)  * cos(omega*t2)) + z0*cos(alpha+(PI/2)*sin(omega*t2))*sin(gamma); 
      By_roll =  (cos(alpha+(PI/2)*sin(omega*t2)) * (x0 + (1-x0)*sin(omega*t2))) + (-sin(alpha+(PI/2)*sin(omega*t2)) * cos(gamma) *  cos(omega*t2)) + z0*sin(alpha+(PI/2)*sin(omega*t2))*sin(gamma); 
      Bz_roll = sin(gamma) * cos(omega*t2) + z0*cos(gamma);

      //this is for tilting back and forth (takes a vector (0, cos(wt),sin(wt)) and rotates it in 3D)
      //Bx_roll = -cos(alpha) * cos(gamma)  * cos(omega*t2) + cos(alpha)*sin(gamma)*sin(omega*t2); 
      //By_roll =  -sin(alpha) * cos(gamma) *  cos(omega*t2) + sin(alpha)*sin(gamma)*sin(omega*t2); 
      //Bz_roll = sin(gamma) * cos(omega*t2) + cos(gamma)* sin(omega*t2);
      
      
       // condition for perpendicular field (psi cannot be 90)
       // condition for perpendicular field (psi cannot be 90)
      if (psi < PI/2){
          c = 1/tan(psi);
          BxPer = c* cos(alpha) * sin(gamma);
          ByPer = tan(alpha) * BxPer;
          BzPer = BxPer * (1/cos(alpha)) * (1/tan(gamma));  
          }
      else{
          c = 0;
          BxPer = 0;
          ByPer = 0;
          BzPer = 0;
      }
       
       // superimpose the rolling field with the perpendicular field
      Bx_roll = (Bx_roll + BxPer) / (1+c);
      By_roll = (By_roll + ByPer) / (1+c);
      Bz_roll = (Bz_roll + BzPer) / (1+c);
      
      }
   //need to add unform field with rotating field and normalize
   //cc = sqrt(Bx_roll*Bx_roll + By_roll*By_roll + Bz_roll*Bz_roll)
   Bx = (Bx_uniform + Bx_roll); /// (1+cc);
   By = (By_uniform + By_roll); /// (1+cc);
   Bz = (Bz_uniform + Bz_roll); //// (1+cc); 
    

   // condition to prevent divide by zero error when total Bx, By, Bz are off aka zeroed
   if (Bx == 0 and By == 0 and Bz ==0){
        Bx_final = 0;
        By_final = 0;
        Bz_final = 0;
   }
   // otherwise I need to normalize the superpoistion of the rotating field with the uniform field
   else{
       magnitude = max(sqrt(Bx_uniform*Bx_uniform + By_uniform*By_uniform + Bz_uniform*Bz_uniform), 
                       sqrt(Bx_roll*Bx_roll + By_roll*By_roll + Bz_roll*Bz_roll));


       Bx_final = magnitude * (Bx / sqrt(Bx*Bx + By*By + Bz*Bz));
       By_final = magnitude * (By / sqrt(Bx*Bx + By*By + Bz*Bz));
       Bz_final = magnitude * (Bz / sqrt(Bx*Bx + By*By + Bz*Bz));
       
       if (equal_field_status == 1){
           Bx_final = Bx_final;
           By_final = By_final * .6;
           Bz_final = Bz_final * .3;
       }
          
       
   }
   

  // if gradient status = 1: output the the corresponding gradient field
   if (gradient_status != 0){
      //y gradient
      if (By_final > 0){
        set1(By_final);
        
      }
      else if (By_final < 0){
        set3(By_final);
      }
      else{ //if By==0
        set1(0);
        set3(0);
      }

      //x gradient
      if (Bx_final > 0){
        set2(Bx_final);
      }
      else if (Bx_final < 0){
        set4(Bx_final);
      }
      else{ //if Bx==0
        set2(0);
        set4(0);
      }

      //z gradient
      if (Bz_final > 0){
        set5(Bz_final);
      }
      else if (Bz_final < 0){
        set6(Bz_final);
      }
      else{ //if Bz==0
        set5(0);
        set6(0);
      }
   }

   // if gradient status = 0: output the correspending uniform field
   else {
      set1(By_final);
      set2(Bx_final);
      set3(-By_final);
      set4(-Bx_final);
      set5(Bz_final);
      set6(-Bz_final);
   }


    }
  
