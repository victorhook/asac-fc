#include "msp.h"

#include "stdbool.h"
#include "string.h"

/*
msp_state_t msp_usb;

void uart_tx(const uint8_t* data, const uint8_t len) {
    // Sent data through UART
}

void uart_rx() {
    uint8_t byte;
    msp_process_byte(&msp_usb, byte);
}

void setup() {
    msp_set_response_callback(&msp_usb, uart_tx);
}

*/


#define DEBUG_PRINTF(msg, ...)

typedef struct {
    uint8_t version_multiwii; // version of MultiWii
    uint8_t multitype; // TRI/QUADP,QUADX,BI,GIMBAL,Y6,HEX6,FLYING_WING,Y4,HEX6X,OCTOX8, OCTOFLATP,OCTOFLATX,AIRPLANE,HELI_120,HELI_90,VTAIL4,HEX6H,SINGLECOPTER,DUALCOPTER
    uint8_t version_msp; // not used currently
    uint32_t capability; // Currently, BIND button is used on first bit, DYNBAL on second, FLAP on third
}__attribute__((packed)) msp_packet_ident_t;

typedef struct {
    int16_t accx;
    int16_t accy;
    int16_t accz;
    int16_t gyrx;  // For MPU6050, 1 unit = 1/4.096 deg/s
    int16_t gyry;
    int16_t gyrz;
    int16_t magx;
    int16_t magy;
    int16_t magz;
}__attribute__((packed)) msp_packet_raw_imu_t;

typedef struct {
    uint16_t m[8];
}__attribute__((packed)) msp_packet_motor_t;

typedef struct {
    int16_t angx;
    int16_t angy;
    int16_t heading;
}__attribute__((packed)) msp_packet_atitude_t;

typedef struct {
    uint8_t p[8];
    uint8_t i[8];
    uint8_t d[8];
}__attribute__((packed)) msp_packet_pid_t;


typedef struct {
  uint8_t protocol_version;
  uint8_t api_version[2];
}__attribute__((packed)) msp_packet_api_version_t;


typedef struct {
  char variant[4];
}__attribute__((packed)) msp_packet_fc_variant_t;


typedef struct {
  uint8_t version[3];
}__attribute__((packed)) msp_packet_fc_version_t;


typedef struct {
  char board_info[4];
  uint8_t board_version[2];
}__attribute__((packed)) msp_packet_board_info_t;


typedef struct {
  char date[11];
  char time[8];
}__attribute__((packed)) msp_packet_build_info_t;


// CUSTOM

typedef struct {
    uint32_t address;
    uint8_t size;
}__attribute__((packed)) msp_packet_read_mem_address_t;

typedef struct {
    uint8_t size;
    union {
        uint8_t  u8;
        uint16_t u16;
        uint32_t u32;
        uint64_t u64;
    } value;
}__attribute__((packed)) msp_packet_read_mem_address_resp_t;

typedef struct {
    uint32_t address;
    uint8_t size;
    union {
        uint8_t  u8;
        uint16_t u16;
        uint32_t u32;
        uint64_t u64;
    } value;
}__attribute__((packed)) msp_packet_write_mem_address_t;


static uint8_t calculate_crc(const msp_packet_t* rx_pkt) {
    uint8_t crc = rx_pkt->size ^ rx_pkt->command;
    for (int i = 0; i < rx_pkt->size; i++) {
        crc ^= rx_pkt->data[i];
    }
    return crc;
}

/* Returns true if packet is OK */
static bool validate_packet(const msp_packet_t* rx_pkt) {
    uint8_t crc = calculate_crc(rx_pkt);
    if (crc != rx_pkt->crc) {
        return false;
    }

    return true;
}

#define assert_expected(actual, expected) \
    if (actual != expected) { \
        DEBUG_PRINTF("Got: %d, expected: %d", actual, expected); \
        header_bytes_read = 0; \
        data_bytes_read = 0; \
        packet_errors++; \
        return; \
    }


// Constants
static uint16_t packet_errors = 0;
static uint8_t header_bytes_read = 0;
static uint8_t data_bytes_read = 0;
static msp_packet_t rx_pkt;
#define MSP_HEADER_SIZE_NO_CRC 5


static void handle_packet(msp_state_t* packet);


void msp_process_byte(msp_state_t* state, const uint8_t byte) {

    switch (header_bytes_read) {
        case 0:
            assert_expected('$', byte);
        case 1:
            assert_expected('M', byte);
            state->rx_pkt.preamble[header_bytes_read] = byte;
            header_bytes_read++;
            break;
        case 2:
            assert_expected('<', byte);
            state->rx_pkt.direction = byte;
            header_bytes_read++;
            break;
        case 3:
            state->rx_pkt.direction = byte;
            header_bytes_read++;
            break;
        case 4:
            state->rx_pkt.command = byte;
            header_bytes_read++;
            break;
        default:
            if (data_bytes_read >= state->rx_pkt.size) {
                // Incoming crc
                state->rx_pkt.crc = byte;
                bool packet_ok = validate_packet(&state->rx_pkt);

                if (packet_ok) {
                    handle_packet(state);
                } else {
                    // TOOD
                }

                // Reset state machine
                header_bytes_read = 0;
                data_bytes_read = 0;

            } else {
                // Incoming data
                state->rx_pkt.data[data_bytes_read] = byte;
                data_bytes_read++;
            }
    }

}


void msp_set_response_callback(msp_state_t* state, response_callback callback) {

}

static msp_packet_t* apply_msp_header(msp_packet_t* tx_pkt, const msp_command_t command, const uint8_t data_len) {
    tx_pkt->preamble[0] = '$';
    tx_pkt->preamble[1] = 'M';
    tx_pkt->direction = '>';
    tx_pkt->command = command;
    tx_pkt->size = data_len;
    return tx_pkt;
}

static void handle_packet(msp_state_t* state) {
    bool send_response = true;
    uint8_t* tx_data = &state->tx_buf[MSP_HEADER_SIZE_NO_CRC];
    uint8_t tx_data_len = 0;
    msp_command_t tx_command;

    switch (state->rx_pkt.command) {
        case MSP_API_VERSION:
            msp_packet_api_version_t* api_version = (msp_packet_api_version_t*) tx_data;
            api_version->protocol_version = 2;
            api_version->api_version[2] = 3;
            tx_data_len = sizeof(msp_packet_api_version_t);
            break;
        case MSP_FC_VARIANT:
            msp_packet_fc_variant_t* fc_variant = (msp_packet_fc_variant_t*) tx_data;
            memcpy(fc_variant->variant, INFO_FC_VARIANT, 4);
            tx_data_len = sizeof(msp_packet_fc_variant_t);
            break;
        case MSP_FC_VERSION:
            msp_packet_fc_version_t* fc_version = (msp_packet_fc_version_t*) tx_data;
            fc_version->version[0] = INFO_FC_VERSION;
            tx_data_len = sizeof(msp_packet_fc_version_t);
            break;
        case MSP_BOARD_INFO:
            msp_packet_board_info_t* board_info = (msp_packet_board_info_t*) tx_data;
            memcpy(board_info->board_info, INFO_BOARD_INFO, 4);
            board_info->board_version[0] = INFO_BOARD_VERSION;
            tx_data_len = sizeof(msp_packet_board_info_t);
            break;
        case MSP_BUILD_INFO:
            msp_packet_build_info_t* build_info = (msp_packet_build_info_t*) tx_data;
            memcpy(build_info->date, INFO_BUILD_DATE, 11);
            memcpy(build_info->time, INFO_BUILD_TIME, 8);
            tx_data_len = sizeof(msp_packet_build_info_t);
            break;
        case MSP_IDENT:
            msp_packet_ident_t* ident = (msp_packet_ident_t*) tx_data;
            ident->version_multiwii = 0;
            ident->multitype = 0;
            ident->capability = 0;
            tx_data_len = sizeof(msp_packet_ident_t);
            break;
        case MSP_RAW_IMU:
            msp_packet_raw_imu_t* raw_imu = (msp_packet_raw_imu_t*) tx_data;
            raw_imu->accx = 1;
            raw_imu->accy = 2;
            raw_imu->accz = 3;
            raw_imu->gyrx = 4;
            raw_imu->gyry = 5;
            raw_imu->gyrz = 6;
            tx_data_len = sizeof(msp_packet_raw_imu_t);
            break;
        case MSP_MOTOR:
            msp_packet_motor_t* motor = (msp_packet_motor_t*) tx_data;
            motor->m[0] = 1;
            motor->m[1] = 2;
            motor->m[2] = 3;
            motor->m[3] = 4;
            tx_data_len = sizeof(msp_packet_motor_t);
            break;
        case MSP_SET_MOTOR:
            break;
        case MSP_ATTITUDE:
            msp_packet_atitude_t* atitude = (msp_packet_atitude_t*) tx_data;
            atitude->angx = 1;
            atitude->angy = 2;
            atitude->heading = 3;
            tx_data_len = sizeof(msp_packet_atitude_t);
            break;
        case MSP_PID:
            msp_packet_pid_t* pid = (msp_packet_pid_t*) tx_data;
            tx_data_len = sizeof(msp_packet_pid_t);
            break;
        case MSP_SET_PID:
            msp_packet_pid_t* set_pid = (msp_packet_pid_t*) tx_data;
            tx_data_len = sizeof(msp_packet_pid_t);
            break;
        case MSP_READ_MEM_ADDRESS:
            msp_packet_read_mem_address_t* read_mem_address = (msp_packet_read_mem_address_t*) state->rx_pkt.data;
            msp_packet_read_mem_address_resp_t* read_mem_address_resp = (msp_packet_read_mem_address_resp_t*) tx_data;
            // Copy data from the memory address to tx buffer
            memcpy(read_mem_address_resp, &(read_mem_address->address), read_mem_address->size);
            // Set size of parameter
            read_mem_address_resp->size = read_mem_address->size;
            tx_data_len = sizeof(msp_packet_read_mem_address_resp_t);
            break;
        case MSP_WRITE_MEM_ADDRESS:
            msp_packet_write_mem_address_t* write_mem_address = (msp_packet_write_mem_address_t*) state->rx_pkt.data;
            memcpy(&(write_mem_address->address), &write_mem_address->value, write_mem_address->size);
            send_response = false;
            break;
    }

    if (send_response) {
        msp_packet_t* tx_pkt = (msp_packet_t*) state->tx_buf;
        // Apply header to TX packet
        apply_msp_header(tx_pkt, tx_command, tx_data_len);
        // Appropiate data should already be set in the switch-statement above.
        // Set CRC
        uint8_t* tx_crc = &state->tx_buf[MSP_HEADER_SIZE_NO_CRC + tx_data_len];
        *tx_crc = calculate_crc(tx_pkt);

        // Call callback function to send response
        state->callback(state->tx_buf, state->tx_len);
    }

}