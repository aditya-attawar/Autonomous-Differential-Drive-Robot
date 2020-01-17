#include "fis_header.h"
// Number of inputs to the fuzzy inference system
const int fis_gcI = 1;
// Number of outputs to the fuzzy inference system
const int fis_gcO = 1;
// Number of rules to the fuzzy inference system
const int fis_gcR = 3;
FIS_TYPE g_fisInput[fis_gcI];
FIS_TYPE g_fisOutput[fis_gcO];
const int trig_M = 23;
const int echo_M = 25;
const int trig_L = 28;
const int echo_L = 32;
const int trig_R = 24;
const int echo_R = 22;
const int FL1 = 38;
const int FL2 = 40;
const int EN_FL = 8;
const int FR1 = 31;
const int FR2 = 33;
const int EN_FR = 5;
const int RL1 = 42;
const int RL2 = 44;
const int EN_RL = 10;
const int RR1 = 37;
const int RR2 = 39;
const int EN_RR = 4;
const int hall = 48;
const float r = 3.5;
const float pi = 3.14;
const int led = 52;
const int bluetooth = 50;
int project_done;
long dura;
int dist;
int ultrasonic(int,int);
long pulse;
int check;
void rotations();
int rot_en;
int p_yp;
int p_yn;
int p_xp;
int p_xn;
int a;
int b;
int y_d;
int x_d;
int turn;
int comp;
int state;
float x;
float y;
float x_dest;
float y_dest;
float x_final;
float y_final;
void straight();
void right(int,int);
void left(int,int);
void stop_motors();
void updatea();
void checkxy();
void location();
void opposite_d();
void compensation();
void setup()
{
  pinMode(trig_M, OUTPUT);
  pinMode(echo_M, INPUT);
  pinMode(trig_L, OUTPUT);
  pinMode(echo_L, INPUT);
  pinMode(trig_R, OUTPUT);
  pinMode(echo_R, INPUT);
  pinMode(FL1, OUTPUT);
  pinMode(FL2, OUTPUT);
  pinMode(EN_FL, OUTPUT);
  pinMode(RL1, OUTPUT);
  pinMode(RL2, OUTPUT);
  pinMode(EN_RL, OUTPUT);
  pinMode(FR1, OUTPUT);
  pinMode(FR2, OUTPUT);
  pinMode(EN_FR, OUTPUT);
  pinMode(RR1, OUTPUT);
  pinMode(RR2, OUTPUT);
  pinMode(EN_RR, OUTPUT);
  pinMode(hall, INPUT);
  pinMode(led, OUTPUT);
  digitalWrite(led, LOW);
   pinMode(bluetooth, INPUT);
  digitalWrite(bluetooth, LOW);
  Serial.begin(9600);
  delay(150);
  check=1;
  rot_en=1;
  a=1;/*Y Axis*/
  b=1;/*Positive direction*/
  turn=0;
  x=0;
  y=0;
  x_final=0;
  y_final=0;
  x_dest=60;
  y_dest=60;
  p_yp=0;
  p_yn=0;
  p_xp=0;
  p_xn=0;
  project_done=0;
  comp = 0;
  state=0;
}

void loop()
{
   state=digitalRead(bluetooth);
   g_fisInput[0] = ultrasonic(trig_M,echo_M);
    g_fisOutput[0] = 0;
    fis_evaluate();
  if(project_done == 0)
  {  
    digitalWrite(led, LOW);
    if(ultrasonic(trig_M,echo_M) >= 2 && ultrasonic(trig_R,echo_R) >= 2 && ultrasonic(trig_L,echo_L) >= 2)
    {
    checkxy();
    location();
    if(a==1)
    {
      if(b==1)
      {
      if((ultrasonic(trig_M,echo_M)>25 && y_final==0) || ((y_dest - y) < ultrasonic(trig_M,echo_M) && project_done == 0))
      {
        location();
        checkxy();
        opposite_d();
        straight();
        }
      else if((ultrasonic(trig_L,echo_L)<=20) && (ultrasonic(trig_R,echo_R)<=20))
      {
        stop_motors();
      }
      else if(x == x_dest &&  y_final == 0)
      {
        if(ultrasonic(trig_L,echo_L) > ultrasonic(trig_R,echo_R))
        {
          left(a,b);
        }
        else
        {
          right(a,b);
        }
      }
      else if((x_dest>x && (ultrasonic(trig_R,echo_R)>40) && project_done == 0) ||  (ultrasonic(trig_L,echo_L)<=20 && (project_done==0)) || ((x_dest - x) < ultrasonic(trig_R,echo_R) && project_done == 0))
      {
        right(a,b);   
      }
      else if((x_dest<x && (ultrasonic(trig_L,echo_L)>40) && project_done == 0) ||  (ultrasonic(trig_R,echo_R)<=20 && (project_done==0)) || ((x - x_dest) < ultrasonic(trig_L,echo_L) && project_done == 0))
      {
        left(a,b);
      }
      else
      {
        stop_motors();
      }
      }
      if(b==0)
      {
      if((ultrasonic(trig_M,echo_M)>25 && y_final==0) || ((y - y_dest) < ultrasonic(trig_M,echo_M) && project_done == 0))
      {
        location();
        checkxy();
        opposite_d();
        straight();
        }
      
      else if((ultrasonic(trig_L,echo_L)<=20) && (ultrasonic(trig_R,echo_R)<=20))
      {
        stop_motors();
      }
      else if(x == x_dest &&  y_final == 0)
      {
        if(ultrasonic(trig_L,echo_L) > ultrasonic(trig_R,echo_R))
        {
          left(a,b);
        }
        else
        {
          right(a,b);
        }
      }
      else if((x_dest>x && (ultrasonic(trig_L,echo_L)>40) && project_done == 0) ||  (ultrasonic(trig_R,echo_R)<=20 && (project_done==0)) || ((x_dest - x) < ultrasonic(trig_L,echo_L) && project_done == 0))
      {
        left(a,b);
      }
      else if((x_dest<x && (ultrasonic(trig_R,echo_R)>40) && project_done == 0) ||  (ultrasonic(trig_L,echo_L)<=20 && (project_done==0)) || ((x - x_dest) < ultrasonic(trig_R,echo_R) && project_done == 0))
      {
        right(a,b);
      }
      else
      {
        stop_motors();
      }
      }
    }
    if(a==0)
    {
      if(b==1)
      {
      if((ultrasonic(trig_M,echo_M)>25 && x_final==0) || ((x_dest - x) < ultrasonic(trig_M,echo_M) && project_done == 0))
      {
        location();
        checkxy();
        opposite_d();
        straight();
      }
      else if((ultrasonic(trig_L,echo_L)<=20) && (ultrasonic(trig_R,echo_R)<=20))
      {
        stop_motors();
      }
      else if(y == y_dest &&  x_final == 0)
      {
        if(ultrasonic(trig_L,echo_L) > ultrasonic(trig_R,echo_R))
        {
          left(a,b);
        }
        else
        {
          right(a,b);
        }
      }
      else if((y_dest>y && (ultrasonic(trig_L,echo_L)>40) && project_done == 0) ||  (ultrasonic(trig_R,echo_R)<=20 && (project_done==0)) || ((y_dest - y) < ultrasonic(trig_L,echo_L) && project_done == 0))
      {
        left(a,b);
      }
      else if((y_dest<y && (ultrasonic(trig_R,echo_R)>40) && project_done == 0) ||  (ultrasonic(trig_L,echo_L)<=20 && (project_done==0)) || ((y - y_dest) < ultrasonic(trig_R,echo_R) && project_done == 0))
      {
        right(a,b);
      }
      else
      {
        stop_motors();
      }
      }
      if(b==0)
      {
      if((ultrasonic(trig_M,echo_M)>25 && x_final==0) || ((x - x_dest) < ultrasonic(trig_M,echo_M) && project_done == 0))
      {
        location();
        checkxy();
        opposite_d();
        straight();
      }
      else if((ultrasonic(trig_L,echo_L)<=20) && (ultrasonic(trig_R,echo_R)<=20) && project_done==0)
      {
        stop_motors();
      }
      else if(y == y_dest &&  x_final == 0)
      {
        if(ultrasonic(trig_L,echo_L) > ultrasonic(trig_R,echo_R))
        {
          left(a,b);
        }
        else
        {
          right(a,b);
        }
      }
      else if((y_dest>y && ultrasonic(trig_R,echo_R)>40 && project_done==0) ||  (ultrasonic(trig_L,echo_L)<=20 && project_done==0) || ((y_dest - y) < ultrasonic(trig_R,echo_R) && project_done == 0))
      {
        right(a,b);
      }
      else if((y_dest<y && ultrasonic(trig_L,echo_L)>40 && project_done==0) ||  (ultrasonic(trig_R,echo_R)<=20 && project_done==0) || ((y - y_dest) < ultrasonic(trig_L,echo_L) && project_done == 0))
      {
        left(a,b);
      }
      else {
        stop_motors();
      }
    }
    }
  }
  }
  else
  {
    stop_motors();
  }
}
void straight()
{
  if(g_fisOutput[0] > 100)
  {
  rot_en=1;
  digitalWrite(FL1, LOW);
  digitalWrite(FL2, HIGH);
  analogWrite(EN_FL , g_fisOutput[0]*0.80);
  digitalWrite(FR1, HIGH);
  digitalWrite(FR2, LOW);
  analogWrite(EN_FR , g_fisOutput[0]*0.80);
  digitalWrite(RL1, LOW);
  digitalWrite(RL2, HIGH);
  analogWrite(EN_RL , g_fisOutput[0]*0.80);
  digitalWrite(RR1, HIGH);
  digitalWrite(RR2, LOW);
  analogWrite(EN_RR , g_fisOutput[0]*0.80);
}
else
{
  rot_en=1;
  digitalWrite(FL1, LOW);
  digitalWrite(FL2, HIGH);
  analogWrite(EN_FL , g_fisOutput[0]);
  digitalWrite(FR1, HIGH);
  digitalWrite(FR2, LOW);
  analogWrite(EN_FR , g_fisOutput[0]);
  digitalWrite(RL1, LOW);
  digitalWrite(RL2, HIGH);
  analogWrite(EN_RL , g_fisOutput[0]);
  digitalWrite(RR1, HIGH);
  digitalWrite(RR2, LOW);
  analogWrite(EN_RR , g_fisOutput[0]);
}
}A
void right(int a,int b)
{
  rot_en=0;
  turn=1;
  stop_motors();
  digitalWrite(FL1, LOW);
  digitalWrite(FL2, HIGH);
  analogWrite(EN_FL, 250);
  digitalWrite(FR1, LOW);
  digitalWrite(FR2, LOW);
  digitalWrite(RL1, LOW);
  digitalWrite(RL2, HIGH);
  analogWrite(EN_RL, 250);
  digitalWrite(RR1, LOW);
  digitalWrite(RR2, LOW);
  delay(1100);
  stop_motors();
  updatea();
  rot_en=1;
}
void left(int a,int b)
{
  rot_en=0;
  turn=2;
  stop_motors();
  digitalWrite(FL1, LOW);
  digitalWrite(FL2, LOW);
  digitalWrite(FR1, HIGH);
  digitalWrite(FR2, LOW);
  analogWrite(EN_FR, 250);
  digitalWrite(RL1, LOW);
  digitalWrite(RL2, LOW);
  digitalWrite(RR1, HIGH);
  digitalWrite(RR2, LOW);
  analogWrite(EN_RR, 250);
  delay(1100);
  stop_motors();
  updatea();
  rot_en=1;
}

void stop_motors()
{
  digitalWrite(FL1, LOW);
  digitalWrite(FL2, LOW);
  digitalWrite(FR1, LOW);
  digitalWrite(FR2, LOW);
  digitalWrite(RL1, LOW);
  digitalWrite(RL2, LOW);
  digitalWrite(RR1, LOW);
  digitalWrite(RR2, LOW);
}

void updatea()
{
  if(turn==1)
  {
    if(a==1)
    {
      a=0;
    }
    else
    {
      a=1;
      if(b==1)
      {
        b=0;
      }
      else
      {
        b=1;
      }
    }
  }
  else
  {
    if(a==0)
    {
      a=1;
    }
    else
    {
      a=0;
      if(b==1)
      {
        b=0;
      }
      else
      {
        b=1;
      }
    }
  }
  turn=0;
}

int ultrasonic(int trig,int echo)
{
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  dura = pulseIn(echo, HIGH);
  dist = dura*0.034/2;
  return(dist);
}


void location()
{
 if(rot_en==1)
 {
  if(a==1)
  {
    if(b==1)
    {
    pulse=digitalRead(hall);
  if(pulse==0)
  {
    if(check==1)
    {
      p_yp = p_yp+1;
      check=0;
    }
  }
  else if(pulse==1)
  {
    if(check==0)
    {
      p_yp = p_yp+1;
      check=1;
    }  
  }
    }
    else
    {
    pulse=digitalRead(hall);
  if(pulse==0)
  {
    if(check==1)
    {
      p_yn = p_yn+1;
      check=0;
    }
  }
  else if(pulse==1)
  {
    if(check==0)
    {
      p_yn = p_yn+1;
      check=1;
    }  
  }
    }
  }
  else
  {
    if(b==1)
    {
     pulse=digitalRead(hall);
  if(pulse==0)
  {
    if(check==1)
    {
      p_xp = p_xp+1;
      check=0;
    }
  }
  else if(pulse==1)
  {
    if(check==0)
    {
      p_xp = p_xp+1;
      check=1;
    }  
  }
    }
    else
    {
    pulse=digitalRead(hall);
  if(pulse==0)
  {
    if(check==1)
    {
      p_xn = p_xn+1;
      check=0;
    }
  }
  else if(pulse==1)
  {
    if(check==0)
    {
      p_xn = p_xn+1;
      check=1;
    }  
  }
    }
  }
}
y=(p_yp - p_yn)*2*pi*r/6;
x=(p_xp - p_xn)*2*pi*r/6;
Serial.println("Y=");
Serial.println(y);
Serial.println("X=");
Serial.println(x);
}
void checkxy()
{
if(abs(y-y_dest)<=3.66)
    {
      y_final=1;
      stop_motors();
    }
    else
    {
      y_final=0;
    }
 if(abs(x-x_dest)<=3.66)
    {
       x_final=1;
       stop_motors();
    }
    else
    {
      x_final=0;
    } 
if(x_final==1 && y_final==1)
{
  project_done=1;
  digitalWrite(led, HIGH);
}
}
void opposite_d()
{
  if(project_done == 0)
  {
  if(y_dest-y >=0)
  {
   y_d=1; 
  }
  else
  {
    y_d=0;
  }
  if(x_dest-x>=0)
  {
    x_d=1;
  }
  else
  {
    x_d=0;
  }
  if(a==1)
  {
    if(b==1)
    {
    if (y_d==0)
    {
     while(project_done == 0)
     {
     
      if((x_dest > x && ultrasonic(trig_R,echo_R)>40) || ((x > x_dest) && ultrasonic(trig_M,echo_M) < 25 && ultrasonic(trig_L,echo_L) < 40))
      {
        right(a,b);
        break;
      }
      if((x_dest < x && ultrasonic(trig_L,echo_L)>40) || ((x_dest > x) && ultrasonic(trig_M,echo_M) < 25 && ultrasonic(trig_R,echo_R) < 40))
     {
      left(a,b);
      break;
     }
      straight();
     }
    }
    }
    else
    {
      if(y_d == 1)
      {
       while(project_done == 0)
     {
      straight();
      if((x_dest > x && ultrasonic(trig_L,echo_L)>40) || ((x > x_dest) && ultrasonic(trig_M,echo_M) < 25 && ultrasonic(trig_R,echo_R) < 40))
      {
        left(a,b);
        break;
      }
      if((x_dest < x && ultrasonic(trig_R,echo_R)>40) || ((x_dest > x) && ultrasonic(trig_M,echo_M) < 25 && ultrasonic(trig_L,echo_L) < 40))
     {
      right(a,b);
      break;
     }
     }  
      }
    }
  }
  else
  {
    if(b==1)
    {
     if(x_d == 0)
     {
      while(project_done == 0)
     {
      straight();
      if((y_dest > y && ultrasonic(trig_L,echo_L)>40) || ((y > y_dest) && ultrasonic(trig_M,echo_M) < 25 && ultrasonic(trig_R,echo_R) < 40))
      {
        left(a,b);
        break;
      }
      if((y_dest < y && ultrasonic(trig_R,echo_R)>40) || ((y_dest > y) && ultrasonic(trig_M,echo_M) < 25 && ultrasonic(trig_L,echo_L) < 40))
     {
      right(a,b);
      break;
     }
     }
     }
    }
    else
    {
      if(x_d == 1)
      {
      while(project_done == 0)
     {
      straight();
      if((y_dest > y && ultrasonic(trig_R,echo_R)>40) || ((y > y_dest) && ultrasonic(trig_M,echo_M) < 25 && ultrasonic(trig_L,echo_L) < 40))
      {
        right(a,b);
        break;
      }
      if((y_dest < y && ultrasonic(trig_L,echo_L)>40) || ((y_dest > y) && ultrasonic(trig_M,echo_M) < 25 && ultrasonic(trig_R,echo_R) < 40))
     {
      left(a,b);
      break;
     }
     }  
      }
    }
   
  }
}
}
void compensation()
{
  if (comp == 1)
  {
  if(a == 1)
  {
    if(b == 1)
    {
     if(turn == 1)
     {
      x = x + 23;
     }
     if(turn ==2)
     {
      x = x - 23; 
     }
    }
    else
    {
      if(turn == 1)
     {
       x = x - 23;
     }
     if(turn ==2)
     {
      x = x + 23;
     }
    }
  }
    else
    {
      if(b ==1)
    {
      if(turn == 1)
     {
       y = y - 23;
     }
     if(turn ==2)
     {
      y = y + 23;
     }
    }
    else
    {
      if(turn == 1)
     {
       y = y + 23;  
     }
     if(turn ==2)
     {
        y = y - 23;
     }
    }
    }
  }
  else if(comp == 0)
  {
     if(a == 1)
  {
    if(b == 1)
    {
     if(turn == 1)
     {
      y = y + 18;
      x = x + 23;
     }
     if(turn ==2)
     {
      y = y + 18;
      x = x - 23; 
     }
    }
    else
    {
      if(turn == 1)
     {
      y = y - 18;
       x = x - 23;
     }
     if(turn ==2)
     {
      y = y - 18;
      x = x + 23;
     }
    }
  }
    else
    {
      if(b ==1)
    {
      if(turn == 1)
     {
      x = x + 18;
       y = y - 23;
     }
     if(turn ==2)
     {
      x = x +18;
      y = y + 23;
     }
    }
    else
    {
      if(turn == 1)
     {
      x = x - 18;
       y = y + 23;  
     }
     if(turn ==2)
     {
      x = x - 18;
        y = y - 23;
     }
    }
    }
  }
  comp = 0;
}
//***********************************************************************
// Support functions for Fuzzy Inference System                          
//***********************************************************************
// Triangular Member Function
FIS_TYPE fis_trimf(FIS_TYPE x, FIS_TYPE* p)
{
    FIS_TYPE a = p[0], b = p[1], c = p[2];
    FIS_TYPE t1 = (x - a) / (b - a);
    FIS_TYPE t2 = (c - x) / (c - b);
    if ((a == b) && (b == c)) return (FIS_TYPE) (x == a);
    if (a == b) return (FIS_TYPE) (t2*(b <= x)*(x <= c));
    if (b == c) return (FIS_TYPE) (t1*(a <= x)*(x <= b));
    t1 = min(t1, t2);
    return (FIS_TYPE) max(t1, 0);
}

FIS_TYPE fis_min(FIS_TYPE a, FIS_TYPE b)
{
    return min(a, b);
}

FIS_TYPE fis_max(FIS_TYPE a, FIS_TYPE b)
{
    return max(a, b);
}

FIS_TYPE fis_array_operation(FIS_TYPE *array, int size, _FIS_ARR_OP pfnOp)
{
    int i;
    FIS_TYPE ret = 0;

    if (size == 0) return ret;
    if (size == 1) return array[0];

    ret = array[0];
    for (i = 1; i < size; i++)
    {
        ret = (*pfnOp)(ret, array[i]);
    }

    return ret;
}


//***********************************************************************
// Data for Fuzzy Inference System                                       
//***********************************************************************
// Pointers to the implementations of member functions
_FIS_MF fis_gMF[] =
{
    fis_trimf
};

// Count of member function for each Input
int fis_gIMFCount[] = { 3 };

// Count of member function for each Output 
int fis_gOMFCount[] = { 3 };

// Coefficients for the Input Member Functions
FIS_TYPE fis_gMFI0Coeff1[] = { -9999999, 10, 25 };
FIS_TYPE fis_gMFI0Coeff2[] = { 20, 40, 60 };
FIS_TYPE fis_gMFI0Coeff3[] = { 40, 60, 9999999 };
FIS_TYPE* fis_gMFI0Coeff[] = { fis_gMFI0Coeff1, fis_gMFI0Coeff2, fis_gMFI0Coeff3 };
FIS_TYPE** fis_gMFICoeff[] = { fis_gMFI0Coeff };

// Coefficients for the Output Member Functions
FIS_TYPE fis_gMFO0Coeff1[] = { -104.2, 0, 104.2 };
FIS_TYPE fis_gMFO0Coeff2[] = { 20.83, 135, 250 };
FIS_TYPE fis_gMFO0Coeff3[] = { 250, 250, 354.2 };
FIS_TYPE* fis_gMFO0Coeff[] = { fis_gMFO0Coeff1, fis_gMFO0Coeff2, fis_gMFO0Coeff3 };
FIS_TYPE** fis_gMFOCoeff[] = { fis_gMFO0Coeff };

// Input membership function set
int fis_gMFI0[] = { 0, 0, 0 };
int* fis_gMFI[] = { fis_gMFI0};

// Output membership function set
int fis_gMFO0[] = { 0, 0, 0 };
int* fis_gMFO[] = { fis_gMFO0};

// Rule Weights
FIS_TYPE fis_gRWeight[] = { 1, 1, 1 };

// Rule Type
int fis_gRType[] = { 1, 1, 1 };

// Rule Inputs
int fis_gRI0[] = { 1 };
int fis_gRI1[] = { 2 };
int fis_gRI2[] = { 3 };
int* fis_gRI[] = { fis_gRI0, fis_gRI1, fis_gRI2 };

// Rule Outputs
int fis_gRO0[] = { 1 };
int fis_gRO1[] = { 2 };
int fis_gRO2[] = { 3 };
int* fis_gRO[] = { fis_gRO0, fis_gRO1, fis_gRO2 };

// Input range Min
FIS_TYPE fis_gIMin[] = { 0 };

// Input range Max
FIS_TYPE fis_gIMax[] = { 600 };

// Output range Min
FIS_TYPE fis_gOMin[] = { 0 };

// Output range Max
FIS_TYPE fis_gOMax[] = { 250 };

//***********************************************************************
// Data dependent support functions for Fuzzy Inference System           
//***********************************************************************
FIS_TYPE fis_MF_out(FIS_TYPE** fuzzyRuleSet, FIS_TYPE x, int o)
{
    FIS_TYPE mfOut;
    int r;

    for (r = 0; r < fis_gcR; ++r)
    {
        int index = fis_gRO[r][o];
        if (index > 0)
        {
            index = index - 1;
            mfOut = (fis_gMF[fis_gMFO[o][index]])(x, fis_gMFOCoeff[o][index]);
        }
        else if (index < 0)
        {
            index = -index - 1;
            mfOut = 1 - (fis_gMF[fis_gMFO[o][index]])(x, fis_gMFOCoeff[o][index]);
        }
        else
        {
            mfOut = 0;
        }

        fuzzyRuleSet[0][r] = fis_min(mfOut, fuzzyRuleSet[1][r]);
    }
    return fis_array_operation(fuzzyRuleSet[0], fis_gcR, fis_max);
}

FIS_TYPE fis_defuzz_centroid(FIS_TYPE** fuzzyRuleSet, int o)
{
    FIS_TYPE step = (fis_gOMax[o] - fis_gOMin[o]) / (FIS_RESOLUSION - 1);
    FIS_TYPE area = 0;
    FIS_TYPE momentum = 0;
    FIS_TYPE dist, slice;
    int i;

    // calculate the area under the curve formed by the MF outputs
    for (i = 0; i < FIS_RESOLUSION; ++i){
        dist = fis_gOMin[o] + (step * i);
        slice = step * fis_MF_out(fuzzyRuleSet, dist, o);
        area += slice;
        momentum += slice*dist;
    }

    return ((area == 0) ? ((fis_gOMax[o] + fis_gOMin[o]) / 2) : (momentum / area));
}

//***********************************************************************
// Fuzzy Inference System                                                
//***********************************************************************
void fis_evaluate()
{
    FIS_TYPE fuzzyInput0[] = { 0, 0, 0 };
    FIS_TYPE* fuzzyInput[fis_gcI] = { fuzzyInput0, };
    FIS_TYPE fuzzyOutput0[] = { 0, 0, 0 };
    FIS_TYPE* fuzzyOutput[fis_gcO] = { fuzzyOutput0, };
    FIS_TYPE fuzzyRules[fis_gcR] = { 0 };
    FIS_TYPE fuzzyFires[fis_gcR] = { 0 };
    FIS_TYPE* fuzzyRuleSet[] = { fuzzyRules, fuzzyFires };
    FIS_TYPE sW = 0;

    // Transforming input to fuzzy Input
    int i, j, r, o;
    for (i = 0; i < fis_gcI; ++i)
    {
        for (j = 0; j < fis_gIMFCount[i]; ++j)
        {
            fuzzyInput[i][j] =
                (fis_gMF[fis_gMFI[i][j]])(g_fisInput[i], fis_gMFICoeff[i][j]);
        }
    }

    int index = 0;
    for (r = 0; r < fis_gcR; ++r)
    {
        if (fis_gRType[r] == 1)
        {
            fuzzyFires[r] = FIS_MAX;
            for (i = 0; i < fis_gcI; ++i)
            {
                index = fis_gRI[r][i];
                if (index > 0)
                    fuzzyFires[r] = fis_min(fuzzyFires[r], fuzzyInput[i][index - 1]);
                else if (index < 0)
                    fuzzyFires[r] = fis_min(fuzzyFires[r], 1 - fuzzyInput[i][-index - 1]);
                else
                    fuzzyFires[r] = fis_min(fuzzyFires[r], 1);
            }
        }
        else
        {
            fuzzyFires[r] = FIS_MIN;
            for (i = 0; i < fis_gcI; ++i)
            {
                index = fis_gRI[r][i];
                if (index > 0)
                    fuzzyFires[r] = fis_max(fuzzyFires[r], fuzzyInput[i][index - 1]);
                else if (index < 0)
                    fuzzyFires[r] = fis_max(fuzzyFires[r], 1 - fuzzyInput[i][-index - 1]);
                else
                    fuzzyFires[r] = fis_max(fuzzyFires[r], 0);
            }
        }

        fuzzyFires[r] = fis_gRWeight[r] * fuzzyFires[r];
        sW += fuzzyFires[r];
    }

    if (sW == 0)
    {
        for (o = 0; o < fis_gcO; ++o)
        {
            g_fisOutput[o] = ((fis_gOMax[o] + fis_gOMin[o]) / 2);
        }
    }
    else
    {
        for (o = 0; o < fis_gcO; ++o)
        {
            g_fisOutput[o] = fis_defuzz_centroid(fuzzyRuleSet, o);
        }
    }
}
