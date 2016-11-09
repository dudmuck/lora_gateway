#include <time.h>
#define SHM_KEY     1234

#define N_UPLINK_PACKETS        4
struct uplink_struct {
    uint32_t    freq_hz;        /*!> RF center frequency */
    uint8_t payload[256];
    uint16_t size; /*!> payload size in bytes */
    uint16_t bw_khz;
    uint8_t ppg;
    uint32_t tx_end_count_us;
    uint8_t sf;
    uint8_t cr;
    int8_t dBm_tx_power;
    bool has_packet;
};

typedef struct {
    uint32_t    freq_hz;        /*!> RF center frequency */
    int8_t gw_tx_power;
    uint8_t modulation;
    uint16_t bw_khz;
    uint8_t sf;
    uint8_t cr;
    uint8_t iq_invert;
    uint8_t f_dev;
    uint16_t preamble_length;
    bool no_crc;
    bool no_header;
    uint8_t payload_size;
    uint8_t payload[256];

    struct timespec tx_preamble_start_time, tx_preamble_end_time;
    uint8_t ppg;

    bool transmitting;
    struct timespec tx_done_time;
    uint32_t tx_cnt;
    int8_t tx_dBm;
} downlink_t;

struct lora_shm_struct {
    uint16_t size;  /* size of this struct */
    struct timespec gw_start_ts;

    struct uplink_struct uplink_packets[N_UPLINK_PACKETS];

    downlink_t downlink;
};

