#define dirPin 2
#define stepPin 3

#include <Servo.h>

Servo servoUpDown;  // Create a servo object
int servoPin = 6;  // Replace with your servo's actual pin

bool checked = false;
float totalAngle = 0;

void setup() {
  // Declare pins as output:
  // pinMode(stepPin, OUTPUT);
  // pinMode(dirPin, OUTPUT);
  // digitalWrite(dirPin, HIGH);
  // stepper.setMaxSpeed(5000); // Adjust the maximum speed
  // stepper.setAcceleration(500); // Adjust the acceleration

  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  servoUpDown.attach(servoPin);  // Attach the servo to the specified pin
  servoUpDown.write(90);
  //Set the spinning direction CW/CCW:
  // digitalWrite(dirPin, HIGH);
  Serial.begin(115200);

}

void moveLeftToRight(int degrees){
    int steps = int((degrees/360.0) * 3200);
    if(steps > 0){
      digitalWrite(dirPin, HIGH);
      for (int i = 0; i < steps; i++) {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(500);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(500);
      }
    }
    if(steps < 0){
      digitalWrite(dirPin, LOW);
      for (int i = steps; i < 0; i++) {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(500);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(500);
      }
    }
    checked = true;
}


void loop() {
  // moveLeftToRight(10);
  // delay(1000);
  // int moveUpDownAngle = 75; // (subangle value from CV)
  // Ensure the angle is within the allowed range (0 to 90 degrees)
  //moveUpDownAngle = constrain(moveUpDownAngle, 0, 90);
  // Move the servo to the desired angle
  //servoUpDown.write(10);

  if(Serial.available() > 0){
    // String coord_str = Serial.readString();
    // int commaIndex = coord_str.indexOf(",");
    // int newlineIndex = coord_str.indexOf("\n");

    // String angleStr = coord_str.substring(0, commaIndex);
    // String yAngleStr = coord_str.substring(commaIndex + 1, newlineIndex);

    // int angle = angleStr.toInt();
    // int yAngle = yAngleStr.toInt();
    float angle = Serial.readString().toInt();
    for (int i = 0; i <= angle; i += 1) {
      servoUpDown.write(angle);
      delay(15);
    }

      // totalAngle += angle;
      // moveLeftToRight(totalAngle);
      // if(checked){
      //   totalAngle = 0;
      //   checked = false;
      // }
      //Serial.print(coord_str);
    // String input = Serial.readStringUntil('\n');  // Read the incoming data until a newline character is received
    // int commaIndex = input.indexOf(',');
    //     if (commaIndex != -1) {
    //         String x_str = input.substring(0, commaIndex);
    //         String y_str = input.substring(commaIndex + 1);
    //         int x_coord = x_str.toInt();
    //         int y_coord = y_str.toInt();
            
    //         // Process the coordinates as needed
    //         Serial.print("Received X: ");
    //         Serial.print(x_coord);
    //         Serial.print(", Y: ");
    //         Serial.println(y_coord);
    //     }
   
  }

  // stepper.run();
  // stepper.runToPosition();
  // int stepsToMove = 200 * 90 / 360; // Calculate the number of steps needed for the desired degrees
  
}