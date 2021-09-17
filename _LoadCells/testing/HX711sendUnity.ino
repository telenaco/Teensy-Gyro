/**
 * @brief calibrated load cell send data to unity for recording it
 */
#include "HX711.h"

 // HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 7;
const int LOADCELL_SCK_PIN = 8;

HX711 scale;

void setup() {
    Serial.begin(115200);
    scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
    scale.set_scale(43564);
    scale.tare();

}

void loop() {
    if (scale.is_ready())
        Serial.println(scale.get_units(5));
}





