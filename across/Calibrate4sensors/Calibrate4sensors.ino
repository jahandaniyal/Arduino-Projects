
//Auto-Calibrate Routine

int P[] ={A0,A1,A2,A3,A4,A5};
#define MR1 11
#define MR2 6
#define ML1 10
#define ML2 9
 int Max[6],Min[6],avg[6],S[6],b[6],dv;
 
void setup() {
  Serial.begin(9600);
  pinMode(ML1, OUTPUT);
  pinMode(ML2, OUTPUT);
  pinMode(MR1, OUTPUT);
  pinMode(MR2, OUTPUT);
  calibrate();
  Align();
  for(int i=0;i<6;i++)
  {
  Serial.print(Max[i]);
  Serial.print("   ");
  Serial.print(Min[i]);
  Serial.println("   ");
  }
  delay(10000);
}

void loop() {
 for(int i =0;i<6;i++)
{
   S[i]=analogRead(P[i]);
   if(S[i]>=((Max[i]+Min[i])/2)) b[i]=1;
   else if(S[i]<((Max[i]+Min[i])/2)) b[i]=0;
}
  //if((b[0]||b[1]||b[2]||b[3]||b[4]||b[5])!=0)
  dv = (b[0]*10  + b[1]) - (b[4] + b[5]*10);
         if((b[0]&&b[1]&&b[4]&&b[5])==1) dv=7;
 else if(((b[0]||b[1]||b[4]||b[5])==0)||((b[0]&&b[1]&&b[4]&&b[5])==1)) dv=5;
 else if(b[2]==1&&b[3]==1 &&((b[1]&&b[4])==0)==1) dv=19;
  
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
  Serial.print("   ");
  Serial.print("DEV : ");
  Serial.print(dv);
  Serial.println("   ");
  delay(1110);
  
}

 void calibrate()
 {
  int m1,m2;
   for(int i=0;i<6;i++) 
   {
     Max[i] = Min[i] = analogRead(P[i]);
   }
   
   for(int g=0;g<2;g++)
   {
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
     analogWrite(MR1,0);
     analogWrite(MR2,0);
     analogWrite(ML1,0);
     analogWrite(ML2,0);
     
     }
   }
   }
 }
   void Align()
   {
     Serial.print("Aligning");
     int flag3=0;
   while(flag3!=1)
   {
   for(int i =0;i<6;i++)
   {
   S[i]=analogRead(P[i]);
   if(S[i]>=((Max[i]+Min[i])/2)) b[i]=1;
   else if(S[i]<((Max[i]+Min[i])/2)) b[i]=0;
   }
  
    dv = (b[0]*10  + b[1]) - (b[4] + b[5]*10);
      if(((b[0]||b[1]||b[4]||b[5])==0)) dv=5;
        else if((b[0]&&b[1]&&b[4]&&b[5])==1) dv=7;
  // Serial.print("   ");
   //Serial.println(dv);
   
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
   analogWrite(MR1,0);
     analogWrite(MR2,0);
     analogWrite(ML1,0);
     analogWrite(ML2,0);
   }
 
