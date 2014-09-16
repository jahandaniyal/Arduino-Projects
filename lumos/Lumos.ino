
//Auto-Calibrate Routine

int P[] ={A0,A1,A2};
#define MR1 11
#define MR2 6
#define ML1 10
#define ML2 9
 int Max,S[3],state;
 float spd,val=80;
 
void setup() {
  Serial.begin(9600);
  pinMode(ML1, OUTPUT);
  pinMode(ML2, OUTPUT);
  pinMode(MR1, OUTPUT);
  pinMode(MR2, OUTPUT);
   rdvalues();
  search();
  /*calibrate();
  Align();
  for(int i=0;i<6;i++)
  {
  Serial.print(Max[i]);
  Serial.print("   ");
  Serial.print(Min[i]);
  Serial.println("   ");
  } */
  //delay(1000);
}

void loop() {
 for(int i =0;i<3;i++)
{
   S[i]=analogRead(P[i]);
}
if (S[0]<20&&(S[1]-100)<35&&S[2]<20)
{
  //stp();
  reverse();
  delay(500);
  stp();
  sek();
}
 else if(S[1]-100>S[0] && S[1]-100>S[2])
 {  if(state==1) val=val+2.3;
    else if(state==0) val = 90;
    if(val > 250)
     val = 255;
   frwd(val);
   Serial.println(val);
   state=1;
 }
 else if(S[0]>S[1]-100 && S[0]>S[2])
 { state=0;
   left();
 }
 
 else if(S[2]>S[1]-100 && S[2]>S[0])
 { state=0;
   right();
 }
 
 /*
  Serial.print(S[0]);
  Serial.print("   ");
    Serial.print(S[1]);
  Serial.print("   ");

  Serial.print(S[2]);
  Serial.print("   ");
  Serial.println("   ");
  delay(100);
  */
}

 void rdvalues()
 {
  int m1=70,m2;
     Max = analogRead(A1); 
     for(int j=0;j<200;j++)
     {
         S[1]=analogRead(A1);
     analogWrite(MR1,m1);
     analogWrite(MR2,m2);
     analogWrite(ML1,m2);
     analogWrite(ML2,m1);
     if(S[1]>Max) Max=S[1];
      Serial.print(S[0]);
  Serial.print("   ");
        Serial.print(Max);
  Serial.println("   ");
     }
     stp();
    Serial.print(Max);
  Serial.println("   ");
 }
   void search()
   {  int count=0;
     int flag3=0;
   while(flag3!=1 && count<300)
   {
   S[1]=analogRead(A1);
   if(S[1]>=(Max-20))  
   {
     stp();
     break;
   }
     else  
     { count++;
        analogWrite(MR1,0);
     analogWrite(MR2,80);
     analogWrite(ML1,80);
     analogWrite(ML2,0);
     }
     Serial.println("searching");
   }
   stp();
   }
 void sek()
   {  int count=0;
   int val=60;
     int flag3=0;
   while(flag3!=1 && count<300)
   {
   S[1]=analogRead(A1);
   if(S[1]>=(val))  
   {
     stp();
     break;
   }
     else  
     { count++;
        analogWrite(MR1,0);
     analogWrite(MR2,80);
     analogWrite(ML1,80);
     analogWrite(ML2,0);
     }
     Serial.println("searching");
   }
   stp();
   }
   
 void frwd(int spd)
 {
   analogWrite(MR1,spd);
     analogWrite(MR2,0);
     analogWrite(ML1,spd);
     analogWrite(ML2,0);
 }
 
 void stp()
 {
   analogWrite(MR1,0);
     analogWrite(MR2,0);
     analogWrite(ML1,0);
     analogWrite(ML2,0);
 }
 
 void left()
 {
   analogWrite(MR1,80);
     analogWrite(MR2,0);
     analogWrite(ML1,0);
     analogWrite(ML2,80);
 }
 
 void right()
 {
   analogWrite(MR1,0);
     analogWrite(MR2,80);
     analogWrite(ML1,80);
     analogWrite(ML2,0);
 }
 
 void reverse()
 {
   analogWrite(MR1,0);
     analogWrite(MR2,200);
     analogWrite(ML1,0);
     analogWrite(ML2,200);
 }
