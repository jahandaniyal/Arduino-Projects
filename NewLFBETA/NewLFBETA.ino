/********************************************************
 * PID Basic Example
 * Reading analog Input 0 to control analog PWM Output 3
 ********************************************************/

#include <PID_v1.h>

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint,2,0,0, DIRECT);


int P[] ={A0,A1,A2,A3,A4,A5};
#define MR1 11
#define MR2 6
#define ML1 10
#define ML2 9
//#ifndef PID_H_
#define PID_H_

int start,End,dif=0;
 int Max[6],Min[6],avg[6],S[6],b[6],dv;

void setup()
{
  Serial.begin(9600);
  //initialize the variables we're linked to
  Setpoint = 0;
  myPID.SetOutputLimits(0, 255);
  myPID.SetSampleTime(1);
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  pinMode(ML1, OUTPUT);
  pinMode(ML2, OUTPUT);
  pinMode(MR1, OUTPUT);
  pinMode(MR2, OUTPUT);
  pinMode(2, INPUT);
  stp();
  calibrate();
  Align();
}

void loop()
{ 
  rd_Sensor();
 int mspeed;
 switch(dv)
  {
    case -1  :  Input = 1;
                myPID.Compute();
                forward(Output);
                      break;
    case  0  :  Input = 1; 
                myPID.Compute();
                forward(Output);
                      break;
    case  1  :  Input = 1;
                myPID.Compute();
                forward(Output);
                 break;
                 
  case 10  :  Input = 2; 
                myPID.Compute();
                SRight(Output);
                    break;
      case 11  :  Input = 2; 
                myPID.Compute();
                SRight(Output);
                  break;      
      case -10  :  Input = 2; 
                myPID.Compute();
                SLeft(Output);
                  break;      
    case -11  :  Input = 2; 
                myPID.Compute();
                SLeft(Output); 
                      break;
    case 110 :  Input = 3; 
                myPID.Compute();
                HRight(Output);
                 break;
    
      
    case -110 :  Input = 3; 
                myPID.Compute();
                HLeft(Output);
                 break;
    case 100 :  Input = 3;
                myPID.Compute();
                HRight(Output);
                 break;
    case 111 :  Input = 4; 
                myPID.Compute();
                HRight(Output);
                 break;   
    case -100 :  Input = 3; 
                myPID.Compute();
                HLeft(Output);
                 break; 
    case -111 :  Input = 4;
                myPID.Compute();
                XHLeft(Output); 
                 break;
                 
    case 9   :  Input = 2;
                myPID.Compute();
                SRight(Output);
               break;
            
   case 99  : Input = 3;
                myPID.Compute();
                SRight(Output);
                    break;
    case 90  : Input = 3; 
                myPID.Compute();
                SRight(Output);
                   break;
      case 101 :  Input = 3;
                myPID.Compute();
                SRight(Output);
                    break;                
            
               case -99  : Input = 3;
                myPID.Compute();
                {SLeft(Output);
                break;}
    case -90  :{ Input = 3;
                myPID.Compute();
                SLeft(Output);
                break;}
    case -101 : { Input = 3; 
                myPID.Compute();
                SLeft(Output);
                break;}
   
    case -9   : Input = 2;
                myPID.Compute();
                SLeft(Output);
                break;
              
  
    case 5 : stp();  
 default : stp();    
  }
  myPID.Compute();
}

void rd_Sensor()
 {/// start = micros();
 for(int i =0;i<6;i++)
{
   S[i]=analogRead(P[i]);
   if(S[i]>=((Max[i]+Min[i])/2)) b[i]=1;
   else if(S[i]<((Max[i]+Min[i])/2)) b[i]=0;
}
  
  dv = (b[0]*100  + b[1]*10 + b[2]) - (b[3] + b[4]*10 + b[5]*100);
  if(((b[0]||b[1]||b[2]||b[3]||b[4]||b[5])==0)||((b[0]&&b[1]&&b[2]&&b[3]&&b[4]&&b[5])==1)) dv=5;
 /// End = micros();
 /// dif = End-start;
  }
 
 void forward(float spd)
 { 
   analogWrite(ML2,0);
   analogWrite(MR2,0);
   analogWrite(MR1,spd);
   analogWrite(ML1,spd);
   //Serial.print("  spd SLEFT    ");
   //Serial.print(spd);
 }
 
void SRight(float spd)
 {
   analogWrite(ML2,0);
   analogWrite(MR2,0);
   analogWrite(MR1,00);
   analogWrite(ML1,spd);
   //Serial.print("  spd SLEFT    ");
   //Serial.print(spd);
 }
 void HRight(float spd)
 {//reverse();
   analogWrite(ML2,0);
   analogWrite(MR2,spd);
   analogWrite(MR1,0);
   analogWrite(ML1,spd);
   //Serial.print("  spd SLEFT    ");
   //Serial.print(spd);
 }
 void SLeft(float spd)
 {
   analogWrite(ML2,0);
   analogWrite(MR2,0);
   analogWrite(MR1,spd);
   analogWrite(ML1,0);
   //Serial.print("  spd SLEFT    ");
  // Serial.print(spd);
 }
 void HLeft(float spd)
 {//stp();
 //reverse();
   analogWrite(ML2,spd);
   analogWrite(MR2,0);
   analogWrite(MR1,spd);
   analogWrite(ML1,0);
   //Serial.print("  spd SLEFT    ");
   //Serial.print(spd);
 }
 void stp()
 {
   analogWrite(ML2,255);
   analogWrite(MR2,255);
   analogWrite(MR1,255);
   analogWrite(ML1,255);
 }
 
 void XHRight(float spd)
 {  
  /// reverse();
  ///stp();
   //delay(10);
   int flag2 =0 ,count = 0;
   while(flag2!=1 && count <30)
   { count++;
  // Serial.print("   countRight :    ");
   //Serial.println(count);
   rd_Sensor();   
   if(dv==0||dv==1||dv==-1 || dv==11 || dv ==10)
   {
     flag2 =1;
     break;
   }
   else if(flag2==0)
   {
     rd_Sensor();   
   analogWrite(ML2,0);
   analogWrite(MR2,spd);
   analogWrite(MR1,0);
   analogWrite(ML1,spd);
   }
   }
   stp();
 }
  void XHLeft(float spd)
 { ///reverse();
  ///stp();
 ///delay(10);
   int flag1 = 0, count =0;
   while(flag1!=1 && count < 30)
   { count++;
  //  Serial.print("   countLeft :    ");
   //Serial.println(count);
     rd_Sensor();   
   if(dv==0||dv==1||dv==-1 || dv == -11 || dv==-10)
   {
     flag1 =1;
     break;
   }
   else if(flag1==0)
   {
     rd_Sensor();
   analogWrite(ML2,spd);
   analogWrite(MR2,0);
   analogWrite(MR1,spd);
   analogWrite(ML1,0);
   }
   }
   stp();
 }
 
 void reverse()
 {
   analogWrite(ML2,255);
   analogWrite(MR2,255);
   analogWrite(MR1,0);
   analogWrite(ML1,0);
  // delay(1);
 }
 
 
 void calibrate()
 {
  
   for(int i=0;i<6;i++) 
   {
     Max[i] = Min[i] = analogRead(P[i]);
   }
   int m1,m2;
   for(int g=0;g<2;g++)
   for(int k=0;k<2;k++)
   {  
     if(k==0)
     {
       m1=70;
       m2=0;
     }
     else if(k==1)
     {
       m1=0;
       m2=70;
     }
     for(int j=0;j<10;j++)
     {
       for(int L=0;L<6;L++)
       {
         S[L]=analogRead(P[L]);
     analogWrite(MR1,m1);
     analogWrite(MR2,m2);
     analogWrite(ML1,m2);
     analogWrite(ML2,m1);
     if(S[L]>Max[L]) Max[L]=S[L];
     if(S[L]<Min[L]) Min[L]=S[L];
    
      Serial.print(S[0]);
  Serial.print("   ");
    Serial.print(S[1]);
  Serial.print("   ");

  Serial.print(S[2]);
  Serial.print("   ");

  Serial.print(S[3]);
  Serial.print("   ");

  Serial.print(S[4]);
  Serial.print("   ");

  Serial.print(S[5]);
  Serial.println("   ");
  
     }
     stp();
     }
   }
   }
   
   void Align()
   {
     Serial.print("Aligning");
     int flag3=0;
   while(flag3!=1)
   {
   rd_Sensor();   
     if(dv==10||dv==11) 
     {
     flag3=1;
     //Serial.println("Exit While Loop");
     break;
     }
     else
     {
       //Serial.println("runnning");
       analogWrite(MR1,40);
     analogWrite(MR2,0);
     analogWrite(ML1,0);
     analogWrite(ML2,40);
     }
   }
  stp();
   }
 
   
 void serialprint()
 {
   Serial.println();
   for(int i=0;i<6;i++)
  {
    Serial.print("AVG");
    Serial.print(i);
    Serial.print("   ");
    Serial.print((Max[i]+Min[i])/2);
    Serial.print("   ");
    Serial.print("Value");
    Serial.print(i);
    Serial.print("   ");
  Serial.print(S[i]);
  Serial.print("   ");
  }
    delay(300);
  Serial.println(" ");
   
 }
 

