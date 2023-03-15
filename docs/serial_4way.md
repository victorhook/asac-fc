UART both RX and TX
BAUD: 19200
1 start
8 data
1 stop
No parity

#define CMD_RUN             0x00
#define CMD_PROG_FLASH      0x01
#define CMD_ERASE_FLASH     0x02
#define CMD_READ_FLASH_SIL  0x03
#define CMD_VERIFY_FLASH    0x03
#define CMD_VERIFY_FLASH_ARM 0x04
#define CMD_READ_EEPROM     0x04
#define CMD_PROG_EEPROM     0x05
#define CMD_READ_SRAM       0x06
#define CMD_READ_FLASH_ATM  0x07
#define CMD_KEEP_ALIVE      0xFD
#define CMD_SET_ADDRESS     0xFF
#define CMD_SET_BUFFER      0xFE

#define CMD_BOOTINIT        0x07
#define CMD_BOOTSIGN        0x08

// Bootloader result codes
#define brSUCCESS           0x30
#define brERRORVERIFY       0xC0
#define brERRORCOMMAND      0xC1
#define brERRORCRC          0xC2
#define brNONE              0xFF


# Initialize bootloader
### BOOT_SIGN:	DB	"BLHeli"
DOWN: `0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 'B', 'L', 'H', 'e', 'l', 'i', 0xF4, 0x7D`
### UP: BOOT_MSG, BOOT_INFO
#### BOOT_MSG:  "471d"		; Interface-MCU_BootlaoderRevision
#### BOOT_INFO:	SIGNATURE_001, SIGNATURE_002, BOOT_VERSION, BOOT_PAGES
UP: `'4', '7', '1', 'c', 0xE8 0xF0, 0x06, 0x01, 0x30`


READ

## Set Address
TX: 0xFF, 0x00, ADDR_H, ADDR_L
RX: ACK



 | CMD, NUM_BYTES