#ifndef spi_rfm95_h
#define i2c_rfm95_h

#include <SPI.h>

/** ######################################################################

Driver for the RFM95W LongRange Tranceiver (Samtech sx1276)

 CONSUMPTION:
    sleep       0.2-1 µA
    idle        1.5 µA
    standby     1.8 µA
    RX          12 mA
    TX          20 (7dBm) - 120mA (20dBm)

 Details:

TODO:
- uses digitalWrite-Stuff
-

########################################################################  */



class RFM95
{

#define RFM_FIFO_SIZE               256
#define RFM_FXOSC                   32000000.0
#define RFM_FSTEP                   (RFM_FXOSC / 524288)

#define SPI_WRITE(a)                (a|0x80)
#define SPI_READ(a)                 (a&0x7F)


    /** ######### Register-Map ################################################################# */

#define REG_FIFO                    0x00    // cleared when in SleepMode
#define REG_OPMODE                  0x01
#define     MSK_OPMODE_LORA         (B10000000) // settable only in sleep mode
#define     MSK_OPMODE_ACCSHARED    (B01000000)
#define     MSK_OPMODE_LOFREQ       (B00001000)
#define     MSK_OPMODE_MODE         (B00000111)
#define         VAL_MODE_SLEEP      0x00    // only mode to allow switching FSK/OOK/LORA
#define         VAL_MODE_STANDBY    0x01
#define         VAL_MODE_FSTX       0x02
#define         VAL_MODE_TX         0x03
#define         VAL_MODE_FSRX       0x04
#define         VAL_MODE_RX_CONT    0x05
#define         VAL_MODE_RX_SINGLE  0x06
#define         VAL_MODE_CAD        0x07

//#define REG_BITRATEMSB              0x02    // bit rate setting
//#define REG_BITRATELSB              0x03

//#define REG_FDEVMSB                 0x04    // Freq Deviation setting
//#define REG_FDEVLSB                 0x05

#define REG_REGFRFMSB               0x06    // RF Carrier Freq --> access in sleep/stdby
#define REG_REGFRFMID               0x07
#define REG_REGFRFLSB               0x08

#define REG_PA_CONFIG               0x09    // PA selection, output power control
#define     MSK_PA_SELECT           (B10000000)  // 0:RFO, 1:BOOST
#define     MSK_PA_MAX_POWER        (B01110000)  // pmax = 10.8+0.6*REG dBm
#define     MSK_PA_OUT_POWER        (B00001111)  // pout = pmax - (15-REG) OR (boost) pout = 2 + REG
#define REG_PA_RAMP                 0x0A    // low phase noise
//#define     MSK_PA_SHAPING          (B01100000)
#define     MSK_PA_RAMP             (B00001111)
#define REG_OCP                     0x0B    // Over current Protection
#define     MSK_OCP_ON              (B00100000)
#define     MSK_OCP_TRIM            (B00011111) // max 240mA, default: 100mA
#define REG_LNA                     0x0C    // LNA settings
#define     MSK_LNA_GAIN            (B11100000) // possible: 1-6 from 0 to -48dB
#define     MSK_LNA_BOOST_LF        (B00011000) // do not change
#define     MSK_LNA_BOOST_HF        (B00000011)
#define         VAL_LNA_BOOST_HF_ON (B00000011) // boost 150%
#define         VAL_LNA_BOOST_HF_DE (B00000000)

#define REG_FIFO_ADDR_PTR           0x0D    // write position to read/write (autoinc)
#define REG_FIFO_TX_BASE_AD         0x0E    // startup: 0x80
#define REG_FIFO_RX_BASE_AD         0x0F    // startup: 0x00
#define REG_FIFO_RX_CURRENT_ADDR    0x10    // start of last packet received

#define REG_IRQ_FLAGS_MASK          0x11
#define REG_IRQ_FLAGS               0x12
#define     MSK_IRQ_RX_TIMEOUT      (B10000000)
#define     MSK_IRQ_RX_DONE         (B01000000)
#define     MSK_IRQ_PAYLOAD_CRC_ERR (B00100000)
#define     MSK_IRQ_VALID_HEADER    (B00010000)
#define     MSK_IRQ_TX_DONE         (B00001000)
#define     MSK_IRQ_CAD_DONE        (B00000100)
#define     MSK_IRQ_FHSS_CHANGE     (B00000010)
#define     MSK_IRQ_CAT_DETECTED    (B00000001)


#define REG_RX_NB_BYTES             0x13    // successful receive will write this bytecount

#define REG_RX_HEADER_CNT_MSB       0x14
#define REG_RX_HEADER_CNT_LSB       0x15
#define REG_RX_PACKET_CNT_MSB       0x16
#define REG_RX_PACKET_CNT_LSB       0x17

#define REG_MODEM_STAT              0x18
#define     MSK_MODEM_RX_CODINGRATE (B11100000)
#define     MSK_MODEM_STATUS        (B00011111)
#define     MSK_MODEM_STATUS_CLEAR  (B00010000)
#define     MSK_MODEM_STATUS_HDRVAL (B00001000)
#define     MSK_MODEM_STATUS_RXON   (B00000100)
#define     MSK_MODEM_STATUS_SYNC   (B00000010)
#define     MSK_MODEM_STATUS_SIGDET (B00000001)

#define REG_PKT_SNR_VALUE           0x19    // last Packets SNR dB = int8_REG/4
#define REG_PKT_RSSI_VALUE          0x1A    // last Packets RSSI dBm = REG - 127
#define REG_RSSI_VALUE              0x1B    // current RSSI dBm = REG - 127

#define REG_HOP_CHANNEL             0x1C
#define     MSK_HOP_PLL_TIMEOUT     (B10000000)
#define     MSK_HOP_RX_CRCON        (B01000000)
#define     MSK_HOP_FHSS_CHANNEL    (B00111111)

#define REG_MODEM_CONFIG1           0x1D        ///
#define     MSK_MODEM_BW            (B11110000)
#define         VAL_MODEM_BW008     (B00000000) // in kHz
#define         VAL_MODEM_BW010     (B00010000)  // in kHz
#define         VAL_MODEM_BW016     (B00100000)  // in kHz
#define         VAL_MODEM_BW021     (B00110000)  // in kHz
#define         VAL_MODEM_BW031     (B01000000)  // in kHz
#define         VAL_MODEM_BW042     (B01010000)  // in kHz
#define         VAL_MODEM_BW063     (B01100000)  // in kHz
#define         VAL_MODEM_BW125     (B01110000)  // in kHz
#define         VAL_MODEM_BW250     (B10000000)  // in kHz
#define         VAL_MODEM_BW500     (B10010000)  // in kHz
#define     MSK_MODEM_CR            (B00001110)
#define         VAL_MODEM_CR1       (B00000010) // Coding Rate 4/5
#define         VAL_MODEM_CR2       (B00000100) // Coding Rate 4/6
#define         VAL_MODEM_CR3       (B00000110) // Coding Rate 4/7
#define         VAL_MODEM_CR4       (B00001000) // Coding Rate 4/8
#define     MSK_MODEM_IMPLICITHDR   (B00000001) // implicit header stores length

#define REG_MODEM_CONFIG2           0x1E
SF06 // 64 chips/symbol --> SNR -5 dB
SF07 // 128 chips/symbol --> SNR -7.5 dB
SF08 // 256 chips/symbol --> SNR -10 dB
SF09 // 512 chips/symbol --> SNR -12.5 dB
SF10 // 1024 chips/symbol --> SNR -15 dB
SF11 // 2048 chips/symbol --> SNR -17.5 dB
SF12 // 4096 chips/symbol --> SNR -20 dB

#define REG_PREAMBLE_DETECT         0x1F
#define REG_PREAMBLE_MSB            0x20
#define REG_PREAMBLE_LSB            0x21

#define REG_PAYLOAD_LENGTH          0x22    // size of bytes to be transmitted
#define REG_MAX_PAYLOAD_LENGTH      0x23

#define REG_HOP_PERIOD              0x24

#define REG_FIFO_RX_BYTE_ADDR       0x25    // Addr of last byte written

#define REG_MODEM_CONFIG3           0x26

/// 0x27 - 3F not for LORA

#define REG_DIO_MAPPING_1           0x40
#define REG_DIO_MAPPING_2           0x41
#define REG_VERSION                 0x42
#define     VAL_V1B                 0x12 // has errors --> extra document (errata)

#define REG_TCXO                    0x4B    // TCXO or XTAL input
#define REG_PA_DAC                  0x4D    // Power setting of PA
#define REG_FORMER_TEMP             0x5B

#define REG_AGC_REF                 0x61
#define REG_AGC_THRESH1             0x62
#define REG_AGC_THRESH2             0x63
#define REG_AGC_THRESH3             0x64
#define REG_PLL                     0x70



#define PAYLOAD_LENGTH              0x0A
#define IMPLICIT_MODE               0x0C

// LOW NOISE AMPLIFIER
#define REG_LNA                     0x0C
#define LNA_MAX_GAIN                0x23 // 0010 0011
#define LNA_OFF_GAIN                0x00

/// static configuration:
// CRCon
// Variable packet: preamble B, SyncWord/NW-ID 1B, length 1B, addr 1B, Data , CRC 2B
// AGC: automatic gain control

private:


    /** ######### function definition ################################################################# */

public:

    RFM95(void)
    {
        //_address = MPL_ADDRESS;
    };

// write();
// writeByte();
// read()
// readByte();
// readFlag();

    void exchange(uint8_t register, uint8_t& buffer[], uint8_t length=1) // uint8_t CS,
{
    if (!length) return;
    digitalWrite(CS, LOW);

    SPI.transfer(buffer[counter]);

    for (uint8_t counter=0; counter < length; counter++)
    {
        buffer[counter] = SPI.transfer(buffer[counter]);
    }
    digitalWrite(CS, HIGH);
};

// setLoRaMode()
// setMode()
// setRXTimeout()

    void sendData()
{
    // goto Standby
    // Set FifoPtrAddr to FifoTxPtrBase.
    // Write PayloadLength bytes to the FIFO (RegFifo)
};

    void receiveDataSingle()
{
    //1 Set FifoAddrPtr to FifoRxBaseAddr.
    //2 Static configuration register device can be written in either Sleep mode, Stand-by mode or FSRX mode.
    //3 A single packet receive operation is initiated by selecting the operating mode RXSINGLE.
    //4 The receiver will then await the reception of a valid preamble. Once received, the gain of the receive chain is set. Following the ensuing reception of a valid header, indicated by the ValidHeader interrupt in explicit mode. The packet reception process commences. Once the reception process is complete the RxDone interrupt is set. The radio then returns automatically to Stand-by mode to reduce power consumption.
    //5 The receiver status register PayloadCrcError should be checked for packet payload integrity.
    //6 If a valid packet payload has been received then the FIFO should be read (See Payload Data Extraction below). Should a subsequent single packet reception need to be triggered, then the RXSINGLE operating mode must be re-selected to launch the receive process again - taking care to reset the SPI pointer (FifoAddrPtr) to the base location in memory (FifoRxBaseAddr).
};

    void receiveDataCont()
    {
    //1 Whilst in Sleep or Stand-by mode select RXCONT mode.
    //2 Upon reception of a valid header CRC the RxDone interrupt is set. The radio remains in RXCONT mode waiting for the
    //next RX LoRaTM packet.
    //3 The PayloadCrcError flag should be checked for packet integrity.
    //4 If packet has been correctly received the FIFO data buffer can be read (see below).
    //5 The reception process (steps 2 - 4) can be repeated or receiver operating mode exited as desired.
    };

    void extractFifo()
    {
       // ValidHeader, PayloadCrcError, RxDone and RxTimeout should not be set
       // RegRxNbBytes Indicates the number of bytes that have been received thus far.
       // RegFifoAddrPtr is a dynamic pointer that indicates precisely where the Lora modem received data has been written up
       // to.
       // Set RegFifoAddrPtr to RegFifoRxCurrentAddr. This sets the FIFO pointer to the location of the last packet received in
       // the FIFO. The payload can then be extracted by reading the register RegFifo, RegRxNbBytes times.
       // Alternatively, it is possible to manually point to the location of the last packet received, from the start of the current
       // packet, by setting RegFifoAddrPtr to RegFifoRxByteAddr minus RegRxNbBytes. The payload bytes can then be read
       // from the FIFO by reading the RegFifo address RegRxNbBytes times.
    };




