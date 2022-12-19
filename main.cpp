#include "mbed.h"
#include "bbcar.h"
  #include "PwmIn.h"
  #include <iostream>
  #include "uLCD_4DGL.h"

  #define CENTER_BASE 1500
  #define unitsFC 360                          // Units in a full circle
  #define dutyScale 1000                       // Scale duty cycle to 1/000ths
  #define dcMin 29                             // Minimum duty cycle
  #define dcMax 971                            // Maximum duty cycle
  #define q2min unitsFC/4                      // For checking if in 1st uadrant
  #define q3max q2min * 3                      // For checking if in 4th uadrant

Ticker servo_ticker;
Ticker servo_feedback_ticker;

PwmOut pin11(D11), pin13(D13);
PwmIn pin10(D10), pin12(D12);
BBCar car(pin11, pin10, pin13, pin12, servo_ticker, servo_feedback_ticker);
BusInOut qti_pin(D4,D5,D6,D7);
uLCD_4DGL uLCD(D1, D0, D2);


BufferedSerial pc(USBTX, USBRX);
DigitalInOut ping(D9);
Timer t;

  volatile int angle1, targetAngle1 = 3;              // Global shared ariables
  volatile int Kp1 = 1;                          // Proportional constant
  volatile float tCycle1;
  volatile int theta1;
  volatile int thetaP1;
  volatile int turns1 = 0;
  volatile int angle2, targetAngle2 = 3;              // Global shared ariables
  volatile int Kp2 = 1;                          // Proportional constant
  volatile float tCycle2;
  volatile int theta2;
  volatile int thetaP2;
  volatile int turns2 = 0;

  void feedback360(){                           // Position monitoring
     tCycle1 = pin10.period();
     int dc1 = dutyScale * pin10.dutycycle();
     theta1 = (unitsFC - 1) -                   // Calculate angle
              ((dc1 - dcMin) * unitsFC)
              / (dcMax - dcMin + 1);
     if(theta1 < 0)                             // Keep theta valid
        theta1 = 0;
     else if(theta1 > (unitsFC - 1))
        theta1 = unitsFC - 1;

     // If transition from quadrant 4 to
     // quadrant 1, increase turns count.
     if((theta1 < q2min) && (thetaP1 > q3max))
        turns1++;
     // If transition from quadrant 1 to
     // quadrant 4, decrease turns count.
     else if((thetaP1 < q2min) && (theta1 > q3max))
        turns1 --;

     // Construct the angle measurement from the turns count and
     // current theta value.
     if(turns1 >= 0)
        angle1 = (turns1 * unitsFC) + theta1;
     else if(turns1 <  0)
        angle1 = ((turns1 + 1) * unitsFC) - (unitsFC - theta1);

     thetaP1 = theta1;                           // Theta previous for next rep


     tCycle2 = pin12.period();
     int dc2 = dutyScale * pin12.dutycycle();
     theta2 = (unitsFC - 1) -                   // Calculate angle
              ((dc2 - dcMin) * unitsFC)
              / (dcMax - dcMin + 1);
     if(theta2 < 0)                             // Keep theta valid
        theta2 = 0;
     else if(theta2 > (unitsFC - 1))
        theta2 = unitsFC - 1;

     // If transition from quadrant 4 to
     // quadrant 1, increase turns count.
     if((theta2 < q2min) && (thetaP2 > q3max))
        turns2++;
     // If transition from quadrant 1 to
     // quadrant 4, decrease turns count.
     else if((thetaP2 < q2min) && (theta2 > q3max))
        turns2 --;

     // Construct the angle measurement from the turns count and
     // current theta value.
     if(turns2 >= 0)
        angle2 = (turns2 * unitsFC) + theta2;
     else if(turns2 <  0)
        angle2 = ((turns2 + 1) * unitsFC) - (unitsFC - theta2);

     thetaP2 = theta2;                           // Theta previous for next rep
  }


int main() {

   servo_feedback_ticker.attach(&feedback360, 5ms);

   float val;
   pc.set_baud(9600);
   int distance_store[1000] = {0};
   int count = 0;
   int object = 0;
   int object_distance[2] = {0};
   int object_angle[2] = {0};
   int distance_second = 0;

   while(1) {
      ping.output();
      ping = 0; wait_us(200);
      ping = 1; wait_us(5);
      ping = 0; wait_us(5);

      ping.input();
      while(ping.read() == 0);
      t.start();
      while(ping.read() == 1);
      val = t.read();
      //printf("Ping = %lf\r\n", val*17700.4f);
      //printf("Laser Ping = %lf\r\n", val*17150.0f); //Laser Ping's distance
      if (object < 2) car.turn(-30, 0.3);
      uLCD.text_width(1); // 4X size text
      uLCD.text_height(1);
      uLCD.color(RED);
      uLCD.locate(1, 2);
      int distance_uLCD =  val*17150.0f;
      uLCD.printf("distance = %3d\n", distance_uLCD);
      distance_store[count] = distance_uLCD;
      if ((count != 0 && count != 1 && count != 2 && object == 1
          && distance_store[count] + 10 < distance_store[count - 1]) ||
          (count != 0 && count != 1 && count != 2 && object == 0
          && distance_store[count - 1] + 10 < distance_store[count])) {

          if (object == 0) {
              object_distance[object] = distance_store[count - 1];
              object_angle[object] = (angle1 + angle2);
          }
          else if (object == 1) {
              object_distance[object] = distance_store[count];
              object_angle[object] = (angle1 + angle2);
          }
          object++;
          car.stop();
          ThisThread::sleep_for(1s);
      }

      count++;

      if (object == 1) {
          uLCD.text_width(1); // 4X size text
          uLCD.text_height(1);
          uLCD.color(RED);
          uLCD.locate(1, 3);
          uLCD.printf("object1_d = %3d\n", object_distance[0]);
          uLCD.text_width(1); // 4X size text
          uLCD.text_height(1);
          uLCD.color(RED);
          uLCD.locate(1, 4);
          uLCD.printf("object1_a = %4d\n", object_angle[0]);
      } else if (object == 2) {
          uLCD.text_width(1); // 4X size text
          uLCD.text_height(1);
          uLCD.color(RED);
          uLCD.locate(1, 5);
          uLCD.printf("object2_d = %3d\n", object_distance[1]);
          uLCD.text_width(1); // 4X size text
          uLCD.text_height(1);
          uLCD.color(RED);
          uLCD.locate(1, 6);
          uLCD.printf("object2_a = %4d\n", object_angle[1]);
          uLCD.text_width(1); // 4X size text
          uLCD.text_height(1);
          uLCD.color(RED);
          uLCD.locate(1, 7);
          int total_a = ((object_angle[0] - object_angle[1]) * 3.3 / 10.9);
          uLCD.printf("total_a = %4d\n", total_a);
          int total_d = sqrt(((object_distance[0])*(object_distance[0]) + object_distance[1]*object_distance[1] - 2 * object_distance[0] * object_distance[1] * cos(3.1416 * total_a / 180)));
          uLCD.text_width(1); // 4X size text
          uLCD.text_height(1);
          uLCD.color(RED);
          uLCD.locate(1, 8);
          uLCD.printf("total_d = %3d\n", total_d);
      }

      t.stop();
      t.reset();
      ThisThread::sleep_for(50ms);

      if (object >= 2) {
          car.stop();
          ThisThread::sleep_for(80s);
      }
   }
}