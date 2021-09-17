#include <ESC.h>
#include <gyroPinDef.h>
#define LED_PIN (13)                                      // Pin for the LED 
#define SPEED_MIN (1000)                                  // Set the Minimum Speed in microseconds
#define SPEED_MAX (2000)                                  // Set the Minimum Speed in microseconds

ESC flywheel(flywheelPin, SPEED_MIN, SPEED_MAX, 500);                 // ESC_Name (ESC PIN, Minimum Value, Maximum Value, Default Speed, Arm Value)
int oESC;                                                 // Variable for the speed sent to the ESC

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);                               // LED Visual Output
  flywheel.arm();                                            // Send the Arm value so the ESC will be ready to take commands
  digitalWrite(LED_PIN, HIGH);                            // LED High Once Armed
  delay(5000);                                            // Wait for a while
  Serial.println("end of setup");
}

void loop() {

  if (Serial.available()) {
    oESC = Serial.parseInt();
    flywheel.speed(oESC);                                    // tell ESC to go to the oESC speed value
    Serial.println(oESC);
  }
  /**
   * @brief this allow a ramp up and down of the ESC speed based on the Min and Max values.
   *
   */
   //   for (oESC = SPEED_MIN; oESC <= SPEED_MAX; oESC += 1) {  // goes from 1000 microseconds to 2000 microseconds
   //     flywheel.speed(oESC);                                    // tell ESC to go to the oESC speed value
   //     Serial.println(oESC);
   //     delay(10);                                            // waits 10ms for the ESC to reach speed
   //   }
   //   delay(1000);
   //   for (oESC = SPEED_MAX; oESC >= SPEED_MIN; oESC -= 1) {  // goes from 2000 microseconds to 1000 microseconds
   //     flywheel.speed(oESC);                                    // tell ESC to go to the oESC speed value
   //     Serial.println(oESC);
   //     delay(10);                                            // waits 10ms for the ESC to reach speed  
   //    }
   //   delay(5000);                                            // Wait for a while befor restart
}