/**
 * @brief calibrated load cell send data to unity for recording it
 */
#include "HX711.h"

 // HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 7;
const int LOADCELL_SCK_PIN = 8 ;

HX711 scale;
int32_t val;

void setup() {
    while (!Serial);
    Serial.begin(115200);
    scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
    // set the offset value that the load cell is reading with no load
    //loadcell.set_offset(LOADCELL_OFFSET);
    scale.set_scale();
    scale.tare();

}

void loop() {
    if (scale.is_ready()) {
        val = scale.get_value(1);
        Serial.println(val);
    }
    delay(10);
}

// ## How to calibrate your load cell
// 1. Call `set_scale()` with no parameter.
// 2. Call `tare()` with no parameter.
// 3. Place a known weight on the scale and call `get_units(10)`.
// 4. Divide the result in step 3 to your known weight. You should
//    get about the parameter you need to pass to `set_scale()`.
// 5. Adjust the parameter in step 4 until you get an accurate reading.
