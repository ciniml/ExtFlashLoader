#include <TFT_eSPI.h>
#include <cstdint>

#include "sam.h"
#include "variant.h"

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
    volatile std::uint32_t* const CMCC_MAINT0 = reinterpret_cast<volatile std::uint32_t* const>(0x41006000 + 0x20);

    bool isMemoryMode = false;

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
    void setBaudRateRegister()
    {
        regs->BAUD = (1u << 8) | (10u << 24);
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
        
        this->setBaudRateRegister();
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
        
        auto wasMemoryMode = this->isMemoryMode;
        this->exitFromMemoryMode();

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

        if( wasMemoryMode ) {
            this->enterToMemoryMode();
        }
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
    void disableQuad()
    {
        std::uint8_t status = this->readStatus(STATUS1);
        this->writeStatus(QSPIFlash::STATUS2, status & ~(1u << 1));
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

    bool getIsMemoryMode() const { return this->isMemoryMode; }

    void enterToMemoryMode()
    {
        if( this->isMemoryMode ) {
            return;
        }
        this->enableQuad(); // Enable Quad IO in flash.
        regs->CTRLA = 0;
        while(regs->STATUS & (1u << 1));
        regs->CTRLB = (1u << 0);    // Enable MEMORY mode.
        this->setBaudRateRegister();
        regs->INSTRCTRL = 0xeb | (0x00u << 16); // Fast QUAD IO Read, Option word = 0xe0 for continuous read.
        // WIDTH=Quad IO, INSTREN, ADDREN, OPTCODEEN, DATAEN, OPTCODELEN=8bits, ADDRLEN=24bits, TFRTYPE=READMEMORY, CRMODE, DUMMYLEN=4
        regs->INSTRFRAME = 0x04 | (1u << 4) | (1u << 5) | (1u << 6) | (1u << 7) | (0x03u << 8) | (0x01 << 12) | /*(1u << 14) | */ (4u << 16);
        regs->CTRLA = (1u << 1);
        while( !(regs->STATUS & (1u << 1)) );

        *CMCC_MAINT0 = 1;   // Invalidate all caches.
        __ISB();
        __DSB();
        this->isMemoryMode = true;
    }

    void exitFromMemoryMode()
    {
        if( !this->isMemoryMode ) {
            return;
        }
        regs->CTRLA = (1u << 0);
        while(regs->STATUS & (1u << 1));
        this->setBaudRateRegister();
        regs->CTRLB = (0b01u << 4); // CSMODE=0b01 (LASTXFER), Disable memory mode.
        regs->CTRLA = (1u << 1);
        while(!(regs->STATUS & (1u << 1)));
        this->disableQuad(); // Disable Quad IO in flash.

        *CMCC_MAINT0 = 1;   // Invalidate all caches.
        __ISB();
        __DSB();
        this->isMemoryMode = false;
    }
};


class SerialDownloader
{
private:
    enum class State
    {
        Idle,
        Response,
        Read,
        Write,
    };

    State state = State::Idle;
    std::uint8_t buffer[256];

    enum class CommandType : std::uint8_t {
        Identify,
        BeginRead,
        BeginWrite,
        Erase,
        Read,
        Write,
        Run,
    };
    struct __attribute__((packed)) CommandHeader
    {
        CommandType command;
        std::uint8_t reserved0;
        std::uint16_t length;
        std::uint32_t address;
    };

    CommandHeader command;
    std::uintptr_t address;
    std::uint16_t bytes_remaining;

    QSPIFlash& qspi;
    volatile std::uint8_t* const USB_BASE = reinterpret_cast<volatile std::uint8_t* const>(0x41000000);

    static constexpr std::uint8_t HOST_MARKER = 0xa5;
    static constexpr std::uint8_t DEVICE_MARKER = 0x5a;

public:
    SerialDownloader(QSPIFlash& qspi) : qspi(qspi) {}

    void setupResponse()
    {
        this->address = reinterpret_cast<std::uintptr_t>(this->buffer);
        this->bytes_remaining = 2;
        this->buffer[0] = DEVICE_MARKER;
        this->buffer[1] = static_cast<std::uint8_t>(this->command.command);
        this->state = State::Response;
    }

    void run()
    {
        switch(this->state) {
            case State::Idle:
                if( Serial.available() >= sizeof(this->command) + 1) {
                    std::uint8_t marker;
                    Serial.readBytes(reinterpret_cast<char*>(&marker), 1);
                    if( marker == HOST_MARKER ) {
                        Serial.readBytes(reinterpret_cast<char*>(&this->command), sizeof(this->command));
                        switch(this->command.command)
                        {
                            case CommandType::Identify: {
                                this->bytes_remaining = sizeof(QSPIFlash::FlashID) + 2;
                                this->buffer[0] = DEVICE_MARKER;
                                this->buffer[1] = static_cast<std::uint8_t>(this->command.command);
                                auto id = this->qspi.readId();
                                memcpy(this->buffer + 2, &id, sizeof(id));
                                this->address = reinterpret_cast<std::uintptr_t>(this->buffer);
                                this->state = State::Response;
                                break;
                            }
                            case CommandType::Read:
                                this->address = this->command.address;
                                this->bytes_remaining = this->command.length;
                                this->state = State::Read;
                                Serial.write(DEVICE_MARKER);
                                Serial.write(static_cast<std::uint8_t>(this->command.command));
                                this->qspi.enterToMemoryMode();
                                break;
                            case CommandType::Write:
                                this->address = this->command.address;
                                this->bytes_remaining = this->command.length;
                                this->state = State::Write;
                                this->qspi.exitFromMemoryMode();
                                break;
                            case CommandType::BeginRead:
                                this->qspi.enterToMemoryMode();
                                this->setupResponse();
                                break;
                            case CommandType::BeginWrite:
                                this->qspi.exitFromMemoryMode();
                                this->setupResponse();
                                break;
                            case CommandType::Erase:
                                this->address = this->command.address;
                                this->qspi.exitFromMemoryMode();
                                this->qspi.writeEnable();
                                this->qspi.eraseSector(this->address);
                                this->qspi.waitProgram(0);
                                
                                this->setupResponse();
                                break;
                            case CommandType::Run: {
                                this->qspi.enterToMemoryMode();
                                __disable_irq();
                                Serial.end();
                                // Disable USB
                                *(USB_BASE + 0x00) = 1; // SWRST
                                while(*(USB_BASE + 0x02) != 0); // SWRST busy wait
                                // Revert GCLK0 to DFLL
                                GCLK->GENCTRL[0].reg = GCLK_GENCTRL_SRC(GCLK_GENCTRL_SRC_DFLL) | GCLK_GENCTRL_GENEN;
                                while ( GCLK->SYNCBUSY.reg & GCLK_SYNCBUSY_GENCTRL0 );
                                std::uint32_t* vector_top = reinterpret_cast<std::uint32_t*>(this->command.address);
                                auto reset_vector = reinterpret_cast<void (*)()>(*(vector_top + 1));
                                SCB->VTOR = this->command.address;
                                __DSB();
                                auto stack_top = *vector_top;
                                __asm__("mov r13, %[stack_top]":: [stack_top] "r" (stack_top));
                                reset_vector();
                                break;
                            }
                        }
                    }
                }
                break;
            case State::Response:
            case State::Read:
                while(this->bytes_remaining > 0) {
                    std::uint16_t bytes_written = Serial.write(reinterpret_cast<char*>(this->address), this->bytes_remaining);
                    if( bytes_written == 0 ) {
                        return;
                    }
                    this->bytes_remaining -= bytes_written;
                    this->address += bytes_written;
                }
                this->state = State::Idle;
                break;
            case State::Write:
                while(this->bytes_remaining > 0) {
                    std::uint16_t bytes_read = Serial.readBytes(reinterpret_cast<char*>(this->buffer + sizeof(this->buffer) - this->bytes_remaining), this->bytes_remaining);
                    if( bytes_read == 0 ) {
                        return;
                    }
                    this->bytes_remaining -= bytes_read;
                }
                this->qspi.writeEnable();
                this->qspi.programPage(this->address, this->buffer, sizeof(this->buffer));
                this->qspi.waitProgram(0);

                this->setupResponse();
                break;
        }
    }
};

TFT_eSPI tft;
QSPIFlash qspiFlash;
SerialDownloader downloader(qspiFlash);

void setup() {
    qspiFlash.initialize();
    Serial.begin(115200);
    while(!Serial);

    tft.begin();
    tft.setRotation(3);

    qspiFlash.reset();
    auto id = qspiFlash.readId();
    tft.fillScreen(0);
    tft.printf("ID: %02x, %04x\r\n", id.manufacturer, id.product);
}
void loop() {
    downloader.run();
}
