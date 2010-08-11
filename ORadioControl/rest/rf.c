// Phase transition time = 0 
// Base frequency = 2432.999908 
// Carrier frequency = 2432.999908 
// Channel number = 0 
// Carrier frequency = 2432.999908 
// Modulated = true 
// Modulation format = MSK 
// Manchester enable = false 
// Sync word qualifier mode = 30/32 sync word bits detected 
// Preamble count = 8 
// Channel spacing = 199.951172 
// Carrier frequency = 2432.999908 
// Data rate = 499.878 
// RX filter BW = 812.500000 
// Data format = Normal mode 
// Length config = Variable packet length mode. Packet length configured by the first byte after sync word 
// CRC enable = true 
// Packet length = 255 
// Device address = 0 
// Address config = No address check 
// CRC autoflush = false 
//  = false 
// TX power = 0 
RF_SETTINGS code rfSettings = {
    0x0C,  // FSCTRL1       Frequency Synthesizer Control 
    0x06,  // IOCFG0        GDO0Output Pin Configuration 
    0x00,  // FSCTRL0       Frequency Synthesizer Control 
    0x5D,  // FREQ2         Frequency Control Word, High Byte 
    0x93,  // FREQ1         Frequency Control Word, Middle Byte 
    0xB1,  // FREQ0         Frequency Control Word, Low Byte 
    0x0E,  // MDMCFG4       Modem Configuration 
    0x3B,  // MDMCFG3       Modem Configuration 
    0x73,  // MDMCFG2       Modem Configuration
    0x42,  // MDMCFG1       Modem Configuration
    0xF8,  // MDMCFG0       Modem Configuration 
    0x00,  // CHANNR        Channel Number 
    0x00,  // DEVIATN       Modem Deviation Setting 
    0xB6,  // FREND1        Front End RX Configuration 
    0x10,  // FREND0        Front End TX configuration 
    0x18,  // MCSM0         Main Radio Control State Machine Configuration 
    0x1D,  // FOCCFG        Frequency Offset Compensation Configuration
    0x1C,  // BSCFG         Bit Synchronization Configuration
    0xC7,  // AGCCTRL2      AGC Control
    0x40,  // AGCCTRL1      AGC Control
    0xB0,  // AGCCTRL0      AGC Control
    0xEA,  // FSCAL3        Frequency Synthesizer Calibration 
    0x0A,  // FSCAL2        Frequency Synthesizer Calibration 
    0x00,  // FSCAL1        Frequency Synthesizer Calibration 
    0x19,  // FSCAL0        Frequency Synthesizer Calibration 
    0x59,  // FSTEST        Frequency Synthesizer Calibration Control 
    0x88,  // TEST2         Various Test Settings 
    0x31,  // TEST1         Various Test Settings 
    0x0B,  // TEST0         Various Test Settings 
    0x07,  // FIFOTHR       RX FIFO and TX FIFO Thresholds
    0x29,  // IOCFG2        GDO2Output Pin Configuration 
    0x04,  // PKTCTRL1      Packet Automation Control
    0x05,  // PKTCTRL0      Packet Automation Control
    0x00,  // ADDR          Device Address 
    0xFF,  // PKTLEN        Packet Length 
};