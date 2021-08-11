#include <Arduino.h>

#include "ESC.h"
#define LED_PIN (13)              // Arduino Nano Pin for the ON-BOARD LED 
#define SPEED_MIN (1000)          // Set the Minimum Speed in microseconds - 5% duty cycle
#define SPEED_MAX (2000)          // Set the Minimum Speed in microseconds - 10% duty cycle
#define SPEED_STOP (1500)         // The STOP setting in mircoseconds for bi-directional ESC - 7.5% duty
#define SPEED_REV_HIGH (1100)     // Reverse HIGH = 3.5 kg F = 3.5 x 9.81 N (force)
#define SPEED_FWD_HIGH (1900)     // Forward HIGH = 4.5 kg F = 4.5 x 9.81 N (force)
#define SPEED_REV_LOW (1400)      // Reverse LOW = 0.5 kg F = 0.5 x 9.81 N (force)
#define SPEED_FWD_LOW (1600)      // Forward LOW = 0.5 kg F = 0.5 x 9.91 N (force)

ESC ESC_Left  (5, SPEED_MIN, SPEED_MAX, 500);   // ESC_Name (ESC PIN, Minimum Value, Maximum Value, Default Speed, Arm Value)
ESC ESC_Right (6, SPEED_MIN, SPEED_MAX, 500);   // ESC_Name (ESC PIN, Minimum Value, Maximum Value, Default Speed, Arm Value)
ESC ESC_Depth (9, SPEED_MIN, SPEED_MAX, 500);   // ESC_Name (ESC PIN, Minimum Value, Maximum Value, Default Speed, Arm Value)


// int oESC;                                      // Variable for the speed sent to the ESC

void setup() {
  // Serial.begin(9600);
  pinMode(LED_PIN, OUTPUT);           // LED Visual Output
  digitalWrite(LED_PIN, LOW);
  ESC_Left.arm();                     // Send the Arm value so the left ESC will be ready to take commands
  ESC_Right.arm();                    // Arm the right ESC
  ESC_Depth.arm();                    // Arm the depth ESC
  delay(5000);                        // Wait for a while
  digitalWrite(LED_PIN, HIGH);        // LED High Once Armed
}

void loop() {
  // STOP Left, Right , Depth Thrusters
  ESC_Left.speed(SPEED_STOP);
  ESC_Right.speed(SPEED_STOP);
  ESC_Depth.speed(SPEED_STOP);
  delay(7000);
  digitalWrite(LED_PIN, LOW);        // LED OFF (3 secs)
  delay(3000);
  digitalWrite(LED_PIN, HIGH);        // LED ON (thruster will start)

  // TEST LEFT - FWD LOW-HIGH-LOW
  /*  
  ESC_Left.speed(SPEED_FWD_LOW);
  delay(3000);
  ESC_Left.speed(SPEED_FWD_HIGH);
  delay(3000);
  ESC_Left.speed(SPEED_FWD_LOW);
  delay(3000);
  ESC_Left.speed(SPEED_STOP);
  delay(3000);
  */

  // TEST DEPTH - FWD-UP LOW-HIGH-LOW
  ESC_Depth.speed(SPEED_FWD_LOW);
  delay(3000);
  ESC_Depth.speed(SPEED_FWD_HIGH);
  delay(3000);
  ESC_Depth.speed(SPEED_FWD_LOW);
  delay(3000);
  ESC_Depth.speed(SPEED_STOP);
  delay(3000);

  // TEST RIGHT - FWD LOW-HIGH-LOW
  ESC_Right.speed(SPEED_FWD_LOW);
  delay(3000);
  ESC_Right.speed(SPEED_FWD_HIGH);
  delay(3000);
  ESC_Right.speed(SPEED_FWD_LOW);
  delay(3000);
  ESC_Right.speed(SPEED_STOP);
  delay(3000);

  /*
  for (oESC = SPEED_MIN; oESC <= SPEED_MAX; oESC += 5) {  // goes from 1000 microseconds to 2000 microseconds
    myESC.speed(oESC);                                   // tell ESC to go to the oESC speed value
    Serial.println(oESC);
    delay(50);                                            // waits 10ms for the ESC to reach speed
  }
  delay(1000);
  for (oESC = SPEED_MAX; oESC >= SPEED_MIN; oESC -= 5) {  // goes from 2000 microseconds to 1000 microseconds
    myESC.speed(oESC);                                    // tell ESC to go to the oESC speed value
    Serial.println(oESC);
    delay(50);                                            // waits 10ms for the ESC to reach speed  
   }
  delay(1000);                                            // Wait for a while befor restart
  */
}