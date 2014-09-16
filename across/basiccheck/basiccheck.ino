//Auto-Calibrate Routine

int P[] ={A0,A1,A2,A3,A4,A5};
#define MR1 11
#define MR2 6
#define ML1 10
#define ML2 9
//#ifndef PID_H_
#define PID_H_

//Define parameter
#define epsilon 0.01
#define dt 0.001             //5ms loop time
#define MAX  90                  //For Current Saturation
#define MIN 50
#define Kp  9
#define Kd  0.0015//0.01
#define Ki  10//0.005
#define LEFT -1
#define RIGHT 1
#define BOTH 90
int whitestate=3,prestate=3,state=99;
int start,End,dif=0;
float integral=0;
 int Max[6],Min[6],avg[6],S[6],b[6],dv;
int stack[3],stackD[3],top=0,topD=0,LastLWS = LOW,LastRWS = LOW; 
int iturn,dir,flag=LOW,flag1=LOW,flag2=LOW,LPS,RPS,LWS,RWS;
int PINLPS=3,PINRPS=4;
int PINLWS=12,PINRWS=13;


 //###################################################

 //PID. Returns float
 float PIDcal(float setpoint,float actual_position)
{
	static float pre_error = 0;
	//static float integral = 0;
        if(state==0) integral=0;
        else if(prestate!=state) integral=0;
	float error;
	float derivative;
	float output;

	//Caculate P,I,D
	error = setpoint - actual_position;

	//In case of error too small then stop intergration
        
	if(abs(error) > epsilon)
	{
		integral = integral + error*dt;
          Serial.println(integral);
	}
       
	derivative = (error - pre_error)/dt;
	output = abs(Kp*error + Ki*integral + Kd*derivative);

	//Saturation Filter
	if(output > MAX)
	{
		output = MAX;
	}
	else if(output < MIN)
	{
		output = MIN;
	}
        //Update error
        pre_error = error;
      //  Serial.println(output);
      prestate=state;
 return output;
}

 //##################################################
void wallread()
    { 
      readwall();
      ///if(LWS!=LastLWS && RWS !=LastRWS)
      ///{
     if(RWS==HIGH && LWS == HIGH)
     {
       ///flag = HIGH;
          dir=BOTH;
          intersection();
     }
     if(RWS==LOW && LWS==LOW)// && flag==HIGH)
      {  
        
        //stp();
        intersection();
        //flag=LOW;
      }
      if(LWS==HIGH && RWS==LOW)
       {
       dir=LEFT;
       intersection();
       //if(LWS==LOW) left();
       }
    
      if(RWS==HIGH && LWS==LOW)
      {
      dir=RIGHT;
      intersection();
      //if(RWS==LOW) right();
      }
    }
     ///}
     
 //####################################################     
     void intersection()
      { 
        readwall();
        if(LWS==LOW && RWS == LOW)
        switch(dir)
        { 
          case RIGHT : XHRight(PIDcal(0,10));
                        break;
          case LEFT : XHLeft(PIDcal(0,10));
                      break;
          case BOTH : wallforward();
                       waiting();
                      break;
        }
        else if(LWS==HIGH && RWS==LOW) forward(PIDcal(0,1));
        else if(LWS==LOW && RWS==HIGH) forward(PIDcal(0,1));
        } 
 //#################################################
        /*
        if(LWS==LOW && RWS==LOW)
        {
          stp(); 
        if(LPS==HIGH) 
        {
          dir=LEFT;
          flag1=HIGH;
        }
        if(RPS==HIGH)
       {
         dir=RIGHT;
         flag2=HIGH;
       }
       if(flag1==HIGH && flag2==HIGH)
       {
         forward();
         delay(100);
       }
       flag1=flag2=LOW;
        }
       while(LWS==HIGH && RWS==HIGH)
         forward(PIDcal(0,1));
       }
      }    
*/
 //#############################################3
void setup()
{
  Serial.begin(9600);
  ///start = micros();
  pinMode(ML1, OUTPUT);
  pinMode(ML2, OUTPUT);
  pinMode(MR1, OUTPUT);
  pinMode(MR2, OUTPUT);
  pinMode(PINRWS, INPUT);
  pinMode(PINLWS,INPUT);
  pinMode(PINRPS,INPUT);
  pinMode(PINLPS,INPUT);  
  stp();
  //calibrate();
  //Align();
 /// Serial.begin(9600);
}


 //###############################################
void loop()
{
  //serialprint();
  
 /* if LWS HIGH >>> Set buffer TO LEFT
    when LWS LOW again >>> Set TopOfStack to LEFT
    left();
  if RWS HIGH >>> Set buffer TO RIGHT 
    when rWS LOW again >>> Set TopOfStack to RIGHT 
    right();
  if LWS HIGH && RWS HIGH >>> Set buffer TO Decision 
    when LWS && RWS both LOW again >>> Set TopOfStack to DIRECTION TAKEN
   */ 
  rd_Sensor();
    //readwall();
switch(dv)
  {
    case -11 :
    case -1  :   state=0; 
                HRight(PIDcal(0,10));
                      break;
    case  0  :   
                  if((b[2]==1 && b[3]==1) && (b[0]||b[5]))    
                  {
                  state=1;
                  forward(PIDcal(0,1));
                  }
                  else 
                  {
                   // stp();
                   // wallread();
                   XHRight(PIDcal(0,10));
                  }
                      break;
    case 11 :                  
    case  1  : state=0;
                
                HLeft(PIDcal(0,10));
                 break;
    case -10: ///stp();
                  ///wallread();
                  XHRight(PIDcal(0,10));
              break;
    case 10:    ///stp();
                ///wallread();
                XHLeft(PIDcal(0,10));
                break;
                
                
                
    case -11: ///stp();
                  ///wallread();
                  XHRight(PIDcal(0,10));
              break;
    case 11:    ///stp();
                ///wallread();
                XHLeft(PIDcal(0,10));
                break;
    
     case 5 : if((b[2]&&b[3])&&(b[0]==0)&&(b[5]==0))
                forward;
               else if(b[2]&&b[3]&&((b[1]&&[b0])||(b[4]&&b[5])))
                XHLeft(PIDcal(0,10));
                     
                
                
      case 7 : /*stp();
                wallread();*/
                XHRight(PIDcal(0,10));
               break;
 default : stp();    
  /*               
  case 10  : state=0;
  SRight(PIDcal(0,11));
                    break;
      case 11  :
     state=2;
     SRight(PIDcal(0,11));
                  break;      
      case -10  : state=0;
      SLeft(PIDcal(0,11));
                  break;      
    case -11  : state=0;
    SLeft(PIDcal(0,11)); 
                      break;
    case 110 : state=0;
    SRight(PIDcal(0,13));
                 break;
    
      
    case -110 : state=0;
    SLeft(PIDcal(0,13)); //H
                 break;
    case 100 :state=0;
    XHRight(PIDcal(0,13));
                 break;
    case 111 : state=0;
    SRight(PIDcal(0,11.5));
                 break;   
    case -100 :state=0;
    XHLeft(PIDcal(0,13));
                 break; 
    case -111 :state=0;
    SLeft(PIDcal(0,11.5)); 
                 break; 
    */                
    
    
       /*  
               
    case 9   : SRight(PIDcal(0,9));
               break;
             
                   
   
   case 99  :SRight(PIDcal(0,99));
                    break;
    case 90  :SRight(PIDcal(0,90));
                   break;
      case 101 : SRight(PIDcal(0,101));
                    break;                
            
               case -99  :{SLeft(PIDcal(0,99));
                break;}
    case -90  :{SLeft(PIDcal(0,90));
                break;}
    case -101 : {SLeft(PIDcal(0,101));
                break;}
   
    case -9   : SLeft(PIDcal(0,9));
                break;
            */  
  //
    //case 5 : stp();  
      //      break;

  }
  //End = micros();
  //dif = End - start;
  //stop();
  //buttonstate=0;
}

void readwall()
{
  RWS=digitalRead(PINRWS);
  LWS=digitalRead(PINLWS);
}
//##################################################
void waiting()
{
 stp();
 int flagw=0;
 while(flagw!=1)
 {
   readtopbot();
   if(LPS==HIGH && RPS==HIGH)
   {
     flagw=1;
     reverse();
     delay(1000);
     stp();
     //forward();
     break;
   }
   else if(LPS==HIGH) dir=LEFT;
   else if(RPS ==HIGH) dir=RIGHT;
 }
}
 //##############################################
 void readtopbot()
 {
   
  RPS=digitalRead(PINRPS);
  LPS=digitalRead(PINLPS);
 }
//################################################
void wallforward()
 { //reverse();
 //stp();
 //delay(10);
   int flag1 = 0;
   while(flag1!=1)
   { 
  //  Serial.print("   countLeft :    ");
   //Serial.println(count);
     //rd_Sensor();   
     readwall();
   if(RWS==LOW && LWS==LOW)//||dv==-111)
   {
     flag1 =1;
     stp();
     break;
   }
   else if(flag1==0)
   {
   analogWrite(ML2,100);
   analogWrite(MR2,0);
   analogWrite(MR1,100);
   analogWrite(ML1,0);
   }
   }
   stp();
 }
 
//#################################################
void rd_Sensor()
 {/// start = micros();
 for(int i =0;i<6;i++)
{
   S[i]=analogRead(P[i]);
   if(S[i]>=((Max[i]+Min[i])/2 + 30)) b[i]=1;
   else if(S[i]<((Max[i]+Min[i])/2 + 30)) b[i]=0;
}
  
  dv = (b[0]*10  + b[1]) - (b[4] + b[5]*10);
  //if(b[2]&&b[1]==1) dv=13;
  if(((b[0]||b[1]||b[4]||b[5])==0)) dv=5;
  else if((b[0]&&b[1]&&b[4]&&b[5])==1) dv=7;
  
 /// End = micros();
 /// dif = End-start;
  }
  //##################################################
/* void stopforever(float spd)
 {
  int flag1 = 0, count =0;
   while(flag1==3||dv==7)
   { count++;
  //  Serial.print("   countLeft :    ");
   //Serial.println(count);
     rd_Sensor();   
   if(dv!=7 )//||dv==-111)
   {
     flag1 =1;
     stp();
     
     break;
   }
   else if(flag1==0||dv==7)
   {
     rd_Sensor();
   revers(80);
   }
   }
   stp();
 }
*/
//############################
 void forward(float spd)
 {
   analogWrite(ML2,0);
   analogWrite(MR2,0);
   analogWrite(MR1,spd);
   analogWrite(ML1,spd);
   //Serial.print("  spd SLEFT    ");
   //Serial.print(spd);
 }
//############################
 void stp()
 {
   analogWrite(ML2,255);
   analogWrite(MR2,255);
   analogWrite(MR1,255);
   analogWrite(ML1,255);
 }
 //############################
 void reverse()
 {
   analogWrite(ML2,0);
   analogWrite(MR2,0);
   analogWrite(MR1,100);
   analogWrite(ML1,100);
   //Serial.print("  spd SLEFT    ");
   //Serial.print(spd);
 }

 //###########################
 void XHLeft(float spd)
 { //reverse();
 //stp();
 //delay(10);
   int flag1 = 0, count =0;
   while(flag1!=1 && count < 200)
   { count++;
  //  Serial.print("   countLeft :    ");
   //Serial.println(count);
     rd_Sensor();   
   if(dv == -11 || dv==-10 || dv==-1 )//||dv==-111)
   {
     flag1 =1;
     HRight(90);
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
 
 //########################### 
 void XHRight(float spd)
 {  
   //reverse();
  ///stp();
   //delay(10);
   int flag2 =0 ,count = 0;
   while(flag2!=1 && count <200)
   { count++;
  // Serial.print("   countRight :    ");
   //Serial.println(count);
   rd_Sensor();   
   if(dv==11 || dv ==10 || dv==1)
   {
     flag2 =1;
     HLeft(90);
     break;
   }
   else if(flag2==0)
   {
     //rd_Sensor();  
     
   analogWrite(ML2,0);
   analogWrite(MR2,spd);
   analogWrite(MR1,0);
   analogWrite(ML1,spd);
   }
   }
   stp();
 }
 //###########################
 void HLeft(float spd)
 {
   analogWrite(ML2,spd);
   analogWrite(MR2,0);
   analogWrite(MR1,spd);
   analogWrite(ML1,0);
   //Serial.print("  spd SLEFT    ");
   //Serial.print(spd);
 }
 //###########################
 void HRight(float spd)
 {
   analogWrite(ML2,0);
   analogWrite(MR2,spd);
   analogWrite(MR1,0);
   analogWrite(ML1,spd);
   //Serial.print("  spd SLEFT    ");
   //Serial.print(spd);
 }
 //########################################
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

 //###########################################
   void Align()
   {
     Serial.print("Aligning");
     int flag3=0;
   while(flag3!=1)
   {
   rd_Sensor();   
     if(dv==1||dv==11) 
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
 
 //#############################################   
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

