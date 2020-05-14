#include <TFT_eSPI.h>
#include <cstdint>
unsigned char boot_bin[] = {
  0x00, 0x00, 0x03, 0x20, 0x09, 0x00, 0x00, 0x04, 0x80, 0xb5, 0x00, 0xaf,
  0x88, 0xb0, 0x00, 0xf0, 0x4d, 0xf8, 0x07, 0x90, 0x07, 0x98, 0x01, 0x21,
  0xca, 0x03, 0x82, 0x60, 0x03, 0x91, 0x00, 0xf0, 0x4d, 0xf8, 0x06, 0x90,
  0x06, 0x98, 0x03, 0x99, 0xca, 0x02, 0x82, 0x60, 0x06, 0x98, 0x82, 0x61,
  0xff, 0xe7, 0x07, 0x98, 0x01, 0x21, 0xca, 0x03, 0xc2, 0x61, 0x05, 0x91,
  0xff, 0xe7, 0x05, 0x98, 0x15, 0x49, 0x88, 0x42, 0x18, 0xd8, 0xff, 0xe7,
  0x06, 0x98, 0x01, 0x21, 0xc9, 0x02, 0xc1, 0x61, 0x00, 0x20, 0x04, 0x90,
  0xff, 0xe7, 0x04, 0x98, 0x05, 0x99, 0x88, 0x42, 0x06, 0xd2, 0xff, 0xe7,
  0x04, 0x98, 0x41, 0x1c, 0x81, 0x42, 0x02, 0x91, 0x07, 0xd3, 0x0a, 0xe0,
  0x05, 0x98, 0x41, 0x1c, 0x81, 0x42, 0x01, 0x91, 0x08, 0xd3, 0x0b, 0xe0,
  0xdb, 0xe7, 0x08, 0x48, 0x00, 0x21, 0x00, 0xf0, 0x0f, 0xf8, 0x02, 0x98,
  0x04, 0x90, 0xe6, 0xe7, 0x04, 0x48, 0x00, 0x21, 0x00, 0xf0, 0x08, 0xf8,
  0x01, 0x98, 0x05, 0x90, 0xd3, 0xe7, 0xc0, 0x46, 0x1f, 0x4e, 0x00, 0x00,
  0x34, 0x01, 0x00, 0x04, 0x81, 0xb0, 0x00, 0x91, 0xff, 0xe7, 0xfe, 0xde,
  0xfd, 0xe7, 0xc0, 0x46, 0x81, 0xb0, 0x02, 0x48, 0x00, 0x90, 0x00, 0x98,
  0x01, 0xb0, 0x70, 0x47, 0x00, 0x80, 0x00, 0x41, 0x81, 0xb0, 0x02, 0x48,
  0x00, 0x90, 0x00, 0x98, 0x01, 0xb0, 0x70, 0x47, 0x80, 0x81, 0x00, 0x41,
  0x82, 0xb0, 0x01, 0x00, 0x3f, 0x22, 0x10, 0x40, 0x6b, 0x46, 0x18, 0x70,
  0x00, 0x98, 0x12, 0x30, 0x10, 0x40, 0x02, 0x28, 0x04, 0xd3, 0xff, 0xe7,
  0x01, 0xa8, 0x00, 0x21, 0x01, 0x70, 0x03, 0xe0, 0x01, 0xa8, 0x01, 0x21,
  0x01, 0x70, 0xff, 0xe7, 0x01, 0x98, 0x02, 0xb0, 0x70, 0x47, 0xd4, 0xd4,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x1c, 0x00, 0x00, 0x00, 0xa1, 0x00, 0x00, 0x04, 0x3c, 0x01, 0x00, 0x04,
  0x10, 0x00, 0x00, 0x00, 0x69, 0x6e, 0x74, 0x65, 0x67, 0x65, 0x72, 0x20,
  0x6f, 0x76, 0x65, 0x72, 0x66, 0x6c, 0x6f, 0x77, 0x00, 0xd4, 0xd4, 0xd4
};
unsigned int boot_bin_len = 336;

struct GPIORegs
{
    std::uint32_t DIR   ;
    std::uint32_t DIRCLR;
    std::uint32_t DIRSET;
    std::uint32_t DIRTGL;
    std::uint32_t OUT   ;
    std::uint32_t OUTCLR;
    std::uint32_t OUTSET;
    std::uint32_t OUTTGL;
    std::uint32_t IN    ;
    std::uint32_t CTRL  ;
    std::uint32_t WRCONFIG;
    std::uint32_t EVCTRL;
    std::uint8_t  PMUX[16];
    std::uint8_t  PINCFG[32]; 
};

static volatile GPIORegs* const PORT_GROUP0 = reinterpret_cast<volatile GPIORegs* const>(0x41008000 + 0x80*0);
static volatile GPIORegs* const PORT_GROUP1 = reinterpret_cast<volatile GPIORegs* const>(0x41008000 + 0x80*1);
static volatile GPIORegs* const PORT_GROUP2 = reinterpret_cast<volatile GPIORegs* const>(0x41008000 + 0x80*2);
static volatile GPIORegs* const PORT_GROUP3 = reinterpret_cast<volatile GPIORegs* const>(0x41008000 + 0x80*3);

class QSPIFlash
{
private:
    static constexpr const std::uintptr_t baseAddress = 0x42003400;
    struct Regs
    {
        std::uint32_t CTRLA;
        std::uint32_t CTRLB;
        std::uint32_t BAUD;
        std::uint32_t RXDATA;
        std::uint32_t TXDATA;
        std::uint32_t INTENCLR;
        std::uint32_t INTENSET;
        std::uint32_t INTFLAG;
        std::uint32_t STATUS;
        std::uint32_t _Reserved0[3];
        std::uint32_t INSTRADDR;
        std::uint32_t INSTRCTRL;
        std::uint32_t INSTRFRAME;
        std::uint32_t _Reserved1;
        std::uint32_t SCRAMBCTRL;
        std::uint32_t SCRAMBKEY;
    };
    volatile Regs* const regs = reinterpret_cast<volatile Regs* const>(baseAddress);

    void waitDataRegisterEmpty()
    {
        while( !(regs->INTFLAG & (1u << 1)) );
    }
    void waitTransferComplete()
    {
        while( !(regs->INTFLAG & (1u << 2) ));
    }
    void waitReceiveDataFull()
    {
        while( !(regs->INTFLAG & (1u << 0) ));
    }
    uint8_t transmit(std::uint8_t value, bool isFirst, bool deassertCS)
    {
        if( !isFirst ) {
            this->waitReceiveDataFull();
        }
        std::uint8_t rxdata = regs->RXDATA;
        this->waitDataRegisterEmpty();
        regs->TXDATA = value;
        this->setLastTransfer(deassertCS);
        return rxdata;
    }
    void setLastTransfer(bool isLastTransfer)
    {
        regs->CTRLA = ((isLastTransfer ? (1u << 24) : 0) | (1u << 1));
    }
    void transmit(const std::uint8_t* txData, std::uint8_t* rxBuffer, std::size_t bytesToTransfer, bool keepCSAsserted = false)
    {
        if( bytesToTransfer == 0 ) {
            return;
        }
        bytesToTransfer--;
        transmit(*txData++, true, bytesToTransfer == 0 && !keepCSAsserted);
        if( bytesToTransfer > 0 ) {
            if( rxBuffer == nullptr ) {
                for(std::size_t bytesTransferred = 0; bytesTransferred < bytesToTransfer - 1; bytesTransferred++ ) {
                    this->transmit(*txData++, false, false);
                }
                this->transmit(*txData++, false, !keepCSAsserted);
            }
            else {
                for(std::size_t bytesTransferred = 0; bytesTransferred < bytesToTransfer - 1; bytesTransferred++ ) {
                    *(rxBuffer++) = this->transmit(*txData++, false, false);
                }
                *(rxBuffer++) = this->transmit(*txData++, false, !keepCSAsserted);
            }
        }
        this->waitTransferComplete();
        this->waitReceiveDataFull();
        if( rxBuffer != nullptr ) {
            *rxBuffer = regs->RXDATA;
        }
        else {
            volatile uint8_t dummy = regs->RXDATA;
        }
    }
    void transmitSingleCommand(std::uint8_t command)
    {
        this->transmit(&command, nullptr, 1);
    }

public:
    struct FlashID
    {
        std::uint16_t product;
        std::uint8_t  manufacturer;
    };
    void initialize()
    {
        PORT_GROUP0->PINCFG[ 8] = 0x01;
        PORT_GROUP0->PINCFG[ 9] = 0x01;
        PORT_GROUP0->PINCFG[10] = 0x01;
        PORT_GROUP0->PINCFG[11] = 0x01;
        PORT_GROUP1->PINCFG[10] = 0x01;
        PORT_GROUP1->PINCFG[11] = 0x01;

        PORT_GROUP0->PMUX[ 8 >> 1] = 0x77;
        PORT_GROUP0->PMUX[10 >> 1] = 0x77;
        PORT_GROUP1->PMUX[10 >> 1] = 0x77;

        regs->CTRLA = (1u << 0); // SWRST
        regs->BAUD = (9u << 8) | (10u << 24);
        regs->CTRLB = (0b01u << 4); // CSMODE=0b01 (LASTXFER)
        regs->CTRLA = (1u << 1);
        while( !(regs->STATUS & (1u << 1)) );   // Wait until the QSPI is enabled.
    }

    void reset()
    {        
        // Reset
        this->transmitSingleCommand(0x66);  // Enable Reset
        this->transmitSingleCommand(0x99);  // Reset
        delayMicroseconds(100);

    }
    FlashID readId()
    {
        std::uint8_t buffer[6];
        memset(buffer, 0xff, sizeof(buffer));

        // Release Power-down
        buffer[0] = 0xab;
        transmit(buffer, nullptr, 5);
        // JEDEC ID
        buffer[0] = 0x9f;
        transmit(buffer, buffer, 4);

        FlashID flashId = {
            (buffer[2] << 8) | buffer[3],
            buffer[1]
        };
        return flashId;
    }
    void writeEnable()
    {
        this->transmitSingleCommand(0x06);
    }
    void writeDisable()
    {
        this->transmitSingleCommand(0x04);
    }

    static constexpr const std::uint8_t STATUS1 = 0x00;
    static constexpr const std::uint8_t STATUS2 = 0x30;
    static constexpr const std::uint8_t STATUS3 = 0x10;
    std::uint8_t readStatus(std::uint8_t statusReg)
    {
        std::uint8_t buffer[] = {statusReg | 0x05, 0x00};
        this->transmit(buffer, buffer, sizeof(buffer));
        return buffer[1];
    }
    void writeStatus(std::uint8_t statusReg, std::uint8_t value)
    {
        std::uint8_t buffer[] = {statusReg | 0x01, value};
        this->transmit(buffer, nullptr, sizeof(buffer));
    }

    void enableQuad()
    {
        std::uint8_t status = this->readStatus(STATUS1);
        this->writeStatus(QSPIFlash::STATUS2, status | (1u << 1));
    }
    void eraseSector(std::uint32_t address)
    {
        std::uint8_t command[] = {
            0x20,
            static_cast<std::uint8_t>(address >> 16), 
            static_cast<std::uint8_t>(address >>  8),
            static_cast<std::uint8_t>(address >>  0),
        };
        this->transmit(command, nullptr, sizeof(command));
    }
    void programPage(std::uint32_t address, const std::uint8_t* data, std::size_t bytesToWrite)
    {
        if(bytesToWrite > 256 ) {
            return;
        }
        std::uint8_t command[] = {
            0x02,
            static_cast<std::uint8_t>(address >> 16), 
            static_cast<std::uint8_t>(address >>  8),
            static_cast<std::uint8_t>(address >>  0),
        };
        this->transmit(command, nullptr, sizeof(command), true);
        this->transmit(data, nullptr, bytesToWrite, false);
    }
    void readData(std::uint32_t address, std::uint8_t* buffer, std::size_t bytesToRead)
    {
        std::uint8_t command[] = {
            0x03,
            static_cast<std::uint8_t>(address >> 16), 
            static_cast<std::uint8_t>(address >>  8),
            static_cast<std::uint8_t>(address >>  0),
        };
        this->transmit(command, nullptr, sizeof(command), true);
        this->transmit(buffer, buffer, bytesToRead, false);
    }
    
    bool waitProgram(std::uint32_t timeout)
    {
        while( this->readStatus(STATUS1) & 0x01 );  // support timeout
        return true;
    }

    void enterToMemoryMode()
    {
        this->enableQuad(); // Enable Quad IO in flash.
        regs->CTRLA = 0;
        while(regs->STATUS & (1u << 1));
        regs->CTRLB = (1u << 0);    // Enable MEMORY mode.
        regs->BAUD = (1u << 8) | (10u << 24);   // BAUD=1, CSDMY=10
        regs->INSTRCTRL = 0xeb | (0xf0u << 16); // Fast QUAD IO Read, Option word = 0xf0 for continuous read.
        // WIDTH=Quad IO, INSTREN, ADDREN, OPTCODEEN, DATAEN, OPTCODELEN=8bits, ADDRLEN=24bits, TFRTYPE=READMEMORY, CRMODE, DUMMYLEN=4
        regs->INSTRFRAME = 0x04 | (1u << 4) | (1u << 5) | (1u << 6) | (1u << 7) | (0x03u << 8) | (0x01 << 12) | (1u << 14) | (4u << 16);
        regs->CTRLA = (1u << 1);
        while( !(regs->STATUS & (1u << 1)) );
    }
};

TFT_eSPI tft;
QSPIFlash qspiFlash;

void setup() {
    qspiFlash.initialize();
    Serial.begin(115200);
    while(!Serial);

    tft.begin();
    tft.setRotation(3);

    qspiFlash.reset();
    auto id = qspiFlash.readId();
    Serial.printf("ID: %02x, %04x\r\n", id.manufacturer, id.product);

    Serial.println("Erasing sector...");
    qspiFlash.writeEnable();
    qspiFlash.eraseSector(0);
    qspiFlash.waitProgram(0);

    Serial.println("Programming page...");
    
    std::uint8_t pageData[256];
    for(std::size_t bytesProgrammed = 0; bytesProgrammed < boot_bin_len; bytesProgrammed += sizeof(pageData)) {
        qspiFlash.writeEnable();
        if( boot_bin_len - bytesProgrammed >= sizeof(pageData) ) {
            qspiFlash.programPage(bytesProgrammed, boot_bin + bytesProgrammed, sizeof(pageData));
        } else {
            memset(pageData, 0xff, sizeof(pageData));
            memcpy(pageData, boot_bin + bytesProgrammed, boot_bin_len - bytesProgrammed);
            qspiFlash.programPage(bytesProgrammed, pageData, sizeof(pageData));
        }
        qspiFlash.waitProgram(0);
    }
    Serial.println("Done!");
    Serial.println("Initializing QSPI with memory mode");
    qspiFlash.enterToMemoryMode();

    Serial.println("Verify via mapped area...");
    volatile std::uint8_t* mapped = reinterpret_cast<volatile std::uint8_t*>(0x04000000);
    for(std::uint_fast16_t i = 0; i < boot_bin_len; i++) {
        if( boot_bin[i] != mapped[i]) {
            Serial.printf("%04x: %02x != %02x\r\n", i, mapped[i], boot_bin[i]);
        }
    }
    Serial.println("Done!");
    
    Serial.println("Jump to flash code");
    std::uintptr_t resetVectorAddress = *reinterpret_cast<volatile std::uint32_t*>(mapped + 4);
    ((void (*)())resetVectorAddress)();
}
void loop() {
    tft.fillScreen(0xffff);
    
    delay(1000);
}
