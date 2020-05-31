#include <TFT_eSPI.h>
#include <cstdint>
#include <ExtFlashLoader.h>
#include "menu_data.h"

TFT_eSPI tft;
ExtFlashLoader::QSPIFlash qspiFlash;

void setup() {
    tft.begin();
    tft.setRotation(3);
    tft.fillScreen(0);

    qspiFlash.initialize();    
    qspiFlash.reset();
    auto id = qspiFlash.readId();
    tft.printf("Flash detected ID: %02x, %04x\r\n", id.manufacturer, id.product);

    tft.printf("Updating external flash...\r\n");
    
    auto result = ExtFlashLoader::writeExternalFlash(qspiFlash, 0, menu_data, menu_data_len, [](std::size_t bytes_processed, std::size_t bytes_total, bool verifying){
        tft.drawCentreString(verifying ? "Verifying..." : "Writing...", 160, 100, 0);
        tft.fillRect(40, 120, 240*bytes_processed/bytes_total, 32, tft.color565(0, verifying?255:0, 255));
        tft.drawRect(40-1, 120-1, 240+2, 32+2, tft.color565(255, 255, 255));
        return true;
    });
    if( result ) {
        tft.printf("Launching menu...");
        ExtFlashLoader::runQSPIApplication(qspiFlash, 0x04000000);
    }
    else {
        tft.printf("Failed to load menu app into the external flash");
    }
}
void loop() {
}
