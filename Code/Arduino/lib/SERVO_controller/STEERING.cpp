#include <Arduino.h>
#include <STEERING.h>

int SERVO_PIN = 10;

double newAngle; // Initial servo angle

double oldAngle; // Previous servo angle

double BASE_ANGLE = 90; // Previous servo angle

Servo myServo; // Create a servo object

 void updateServo(String newTargetAngleFactor) {
   if (newTargetAngleFactor.toInt() >= -1 && newTargetAngleFactor.toInt() <= 1)
   {

      newAngle = 180 * newTargetAngleFactor.toInt();
      Serial.print("New Servo angle target: ");
      Serial.println(newAngle);

      myServo.write(newAngle);
      
   }
   
}

//void updateServo(String newTargetAngle) {
   // oldAngle = newAngle; // Store the previous angle
   // newAngle = newTargetAngle.toInt();
   // if(newAngle > oldAngle)
   // {
    //    for(int x = 0; x < newAngle ;x++)
      //  {
      //      myServo.write(x);//back to 'num' degrees(0 to 180)
       //     delay(10);//control servo speed
       // }
    //}
    //else if(newAngle < oldAngle)
    //{
      //  for(int x = oldAngle; x > newAngle ;x--)
      //  {
      //      myServo.write(x);//back to 'num' degrees(0 to 180)
       //     delay(10);//control servo speed
       // }
    //}
    //else
    //{
     //   myServo.write(newAngle);
    //}

    //Serial.print("New Servo angle target: ");
    //Serial.println(newAngle);

    //myServo.write(newAngle);
//}