#include <TFT_eSPI.h>
#include <cstdint>
#include <ExtFlashLoader.h>

TFT_eSPI tft;

void setup() {
    tft.begin();
    tft.setRotation(3);
    tft.fillScreen(0);

    pinMode(WIO_KEY_A, INPUT_PULLUP);
    if( digitalRead(WIO_KEY_A) == LOW) {
      tft.printf("Launching QSPI application\r\n");
      ExtFlashLoader::ExtFlashLoader loader;
    }
    
    tft.printf("Normal flash application\r\n");

    Serial.begin(115200);
    while(!Serial);
}
void loop() {

}