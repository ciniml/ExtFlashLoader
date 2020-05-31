#include <TFT_eSPI.h>
#include <cstdint>
#include <ExtFlashLoader.h>

TFT_eSPI tft;
ExtFlashLoader::QSPIFlash qspiFlash;
ExtFlashLoader::SerialDownloader downloader(qspiFlash);

void setup() {
    tft.begin();
    tft.setRotation(3);
    tft.fillScreen(0);

    qspiFlash.initialize();    
    qspiFlash.reset();
    auto id = qspiFlash.readId();
    tft.printf("Flash detected ID: %02x, %04x\r\n", id.manufacturer, id.product);

    tft.printf("Waiting for serial connection...\r\n");

    Serial.begin(115200);
    while(!Serial);
}
void loop() {
    downloader.run();
}
