#include "stdio.h"
#include "vtx/smartaudio.h"




static void test_msg(const uint8_t msg[], const uint8_t len)
{
    printf("Testing %d bytes: ", len);
    for (int i = 0; i < len; i++)
    {
        printf("%02x ", msg[i]);
    }
    printf("\n");

    for (int i = 0; i < len; i++)
    {
        if (smart_audio_parse_byte(msg[i]))
        {
            printf("NEW FRAME!\n");
        }
    }
    printf("\n");
}


int main()
{
    smart_audio_init();

    // [REQ] Get settings
    // 0xAA 0x55
    // 0x03(modified Command see Host to VTX)
    // 0x00 Length
    // 0x9F (CRC)
    uint8_t msg[] = {0xAA, 0x55, 0x03, 0x00, 0x9F};
    test_msg(msg, sizeof(msg));

    // [RESP] Settings
    // 0xAA 0x55
    // 0x09 (Version/Command)
    // 0x05 (Length)
    // 0x01 (Channel)
    // 0x00 (Power Level)
    // 0x1A (Operation Mode)
    // 0x16 0xE9(Current Frequency 5865)
    // 0xDA(CRC8)
    uint8_t resp_v2[] = {0xAA, 0x55, 0x09, 0x05, 0x01, 0x00, 0x1A, 0x16, 0xE9, 0x82};
    test_msg(resp_v2, sizeof(resp_v2));

    // [REQ] SET Power
    // 0xAA 0x55
    // 0x07(Command 3)
    // 0x01(Length)
    // 0x00 (All 40 Channels 0-40)
    // 0xB8(CRC8)
    uint8_t req_set_power[] = {0xAA, 0x55, 0x07, 0x01, 0x00, 0xB8};
    test_msg(req_set_power, sizeof(req_set_power));

    // [RESP] SET Power
    // 0xAA 0x55
    // 0x02 (Command)
    // 0x03 (Length)
    // 0x01 (Channel)
    // 0x00 (Power Level)
    // 0x01 (reserved)
    // 0x0F (CRC8)
    uint8_t resp_set_power[] = {0xAA, 0x55, 0x02, 0x03, 0x01, 0x00, 0x0F};
    test_msg(resp_set_power, sizeof(resp_set_power));




    return 0;
}