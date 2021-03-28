#define trig1 10
#define echo1 11
#define trig2 13
#define echo2 12
#define trig3 2
#define echo3 3
#define l_wheel 9
#define r_wheel 5
#define dir_right 6
//#define dir2right 7
#define dir_left 7
//#define dir2left 8
#define normal_s 50
double duration1;
double distance_1;
double duration2;
double distance_2;
double duration3;
double distance_3;
double ideal = 40;
double ini_orient_error = 0;
double d_error=0;
double prev_dis_error=0;
double p_error2 = 0;
double d_error2 = 0;
double sum_error2 = 0;
void more_l_orient_l();
void more_l_orient_r();
void less_l_orient_l();
void less_l_orient_r();
void turn_r();
void perfect();
void orient_r();
void orient_l();
void turn_l();
int p=120;
double kp = 1.5;
double kd = 0.3;
double ki = 0;
double kd2 = 6;
double ki2 = 0.00;
double kp2 = 12;
double k=0.6;

void setup() {

  pinMode(trig1, OUTPUT);
  pinMode(echo1, INPUT);
  pinMode(trig2, OUTPUT);
  pinMode(echo2, INPUT);
  pinMode(trig3, OUTPUT);
  pinMode(echo3, INPUT);
  pinMode(l_wheel, OUTPUT);
  pinMode(r_wheel, OUTPUT);
  pinMode(dir_left, OUTPUT);
  //pinMode(dir2left,OUTPUT);
  pinMode(dir_right, OUTPUT);
  //pinMode(dir2right,OUTPUT);
  Serial.begin(9600);

}
void loop() {
  /*digitalWrite(trig1,LOW);
    delayMicroseconds(2);
    digitalWrite(trig1,HIGH);
    delayMicroseconds(10);
    digitalWrite(trig1,LOW);

    duration1=pulseIn(echo1,HIGH);
    distance_1=duration1/58.2;
    digitalWrite(trig2,LOW);
    delayMicroseconds(2);*/

  calculate_distance();
  /*Serial.println("READ1");
    Serial.println(distance_1);*/
  Serial.print(" READ2 ");
  Serial.print(distance_2);
  Serial.print(" READ3 ");
  Serial.print(distance_3);
  //delay(500);

  double p_error = ideal - ((distance_2 + distance_3) / 2.0);
  d_error=p_error-prev_dis_error;
  double p_error2 = distance_3 - distance_2;
  d_error2 = p_error2 - ini_orient_error;
  sum_error2 = 0;

  if (abs(p_error2) < 0.25)sum_error2 = 0;
  if (abs(sum_error2) >= 65)sum_error2 = 0;
  double correction_dis = kp * p_error;+kd*d_error;//+ki*i_error;
  double correction_orient = kp2 * p_error2 + kd2 * d_error2 + ki2 * sum_error2;
  ini_orient_error = p_error2;
  prev_dis_error=p_error;
  Serial.print(" P_ERROR ");Serial.print(p_error);
  //Serial.print(" C_DISC ");Serial.print(correction_dis);

  //Serial.print(" C_orient ");Serial.print(correction_orient);
 /* double tot_err2 = p_error2 + d_error2 + sum_error2;
  Serial.print(" Orient_error "); Serial.print(tot_err2);
  Serial.print(" dis_err "); Serial.print(p_error);
  if (p_error <= -7) {
    if (2 < abs(abs(p_error) - abs(p_error2)) < 35)
    {
      correction_orient = -7;
    }
  }
  Serial.print(" corr_orient "); Serial.print(correction_orient);
  Serial.print(" C_DISC "); Serial.print(correction_dis);*/

  /*if(20<abs(abs(correction_dis)-abs(correction_orient))<50)
    {digitalWrite(dir_left,HIGH);
     analogWrite(l_wheel,60);
    digitalWrite(dir_right,HIGH);
     analogWrite(r_wheel,60);
  */
  double output = correction_dis + k*(correction_orient);

  int out_l_final = normal_s + constrain(output, -100, 100);
  int out_l_final2 = constrain(out_l_final, -100, 100);
  if (distance_2 < 2000 && distance_3 < 2000)
  { if (out_l_final2 >= 0)
    {
      digitalWrite(dir_left, HIGH);
      //digitalWrite(dir2left,LOW);
      analogWrite(l_wheel, out_l_final + p);
      Serial.print(" LEFT ");
      Serial.print(out_l_final2 + p);

    }
    else
    {
      digitalWrite(dir_left, LOW);
      //digitalWrite(dir2left,HIGH);
      analogWrite(l_wheel, abs(out_l_final2 + p));
      Serial.print(" LEFT ");
      Serial.print(out_l_final2 - p);

    }
    int out_r_final = normal_s - constrain(output, -100, 100);
    int out_r_final2 = constrain(out_r_final, -100, 100);
    if (out_r_final2 >= 0)
    {
      digitalWrite(dir_right, HIGH);
      //digitalWrite(dir2right,LOW);
      analogWrite(r_wheel, out_r_final2 + p);
      Serial.print(" RIGHT ");
      Serial.print(out_r_final2 + p);
    }
    else
    {
      digitalWrite(dir_right, LOW);
      //digitalWrite(dir2right,HIGH);
      analogWrite(r_wheel, abs(out_r_final2) + p);
      Serial.print(" RIGHT ");
      Serial.print(out_r_final2 - p);
    }
  }
  if (distance_3 > 2000)
  {
    digitalWrite(dir_right, HIGH);
    analogWrite(r_wheel, 160);
    digitalWrite(dir_left, HIGH);
    analogWrite(l_wheel, 50);
    Serial.print(" LEFT 70 RIGHT 100");


  }
  if (distance_2 > 2000)
  {
    digitalWrite(dir_right, HIGH);
    analogWrite(r_wheel, 50);
    digitalWrite(dir_left, HIGH);
    analogWrite(l_wheel, 160);
    Serial.print(" LEFT 100 RIGHT 70");


  }

  //delay(500);
  Serial.println();

}

void calculate_distance()
{
  digitalWrite(trig2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig2, LOW);

  duration2 = pulseIn(echo2, HIGH);
  distance_2 = (duration2 / 58.2) + 1.0;
  digitalWrite(trig3, LOW);
  delayMicroseconds(2);

  digitalWrite(trig3, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig3, LOW);

  duration3 = pulseIn(echo3, HIGH);
  distance_3 = duration3 / 58.2;
}
