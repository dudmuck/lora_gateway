#include <stdio.h>
#include <limits.h>
#include "loragw_hal.h"
#include "loragw_reg.h"
#include "lora_shm.h"
#include <sys/ipc.h>
#include <sys/shm.h>
#include <stdlib.h>
#include <pthread.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <math.h>

#define MIN_LORA_PREAMBLE   4
#define STD_LORA_PREAMBLE   6

const char lgw_version_string[] = "Version: " LIBLORAGW_VERSION ";";

void *shared_memory1_pointer = (void *)0;
int shared_memory1_id;
struct lora_shm_struct *shared_memory1;

pthread_t thrid_fakegps, thrid_send;
extern bool hal_run;

uint8_t ppg;

void thread_fakegps(void);  // from loragw_gps.shm.c
extern bool gps_tx_trigger; // from loragw_gps.shm.c
struct lgw_pkt_tx_s g_pkt_data;

#define MAX_IF_PER_RADIO       5
typedef struct {
    uint32_t center_freq;
    uint32_t rx_freqs[MAX_IF_PER_RADIO];
    uint8_t bws[MAX_IF_PER_RADIO];
    uint8_t drs[MAX_IF_PER_RADIO];
    bool valid[MAX_IF_PER_RADIO];
    uint8_t if_chain_num[MAX_IF_PER_RADIO];
    uint8_t n_ifs;
} radio_t;
radio_t radio[2];
bool initialized_radio = false;

#define POLL_DELAY_NS      1e8
//uint32_t cnt = 0;
void thread_send(void)
{
    int ret;
    struct timespec now, dly, rem;

    while (hal_run) {
        if (clock_gettime (CLOCK_MONOTONIC, &now) == -1)
            perror ("clock_gettime");

        dly.tv_sec = now.tv_sec;
        dly.tv_nsec = now.tv_nsec + POLL_DELAY_NS;
        if (dly.tv_nsec > 1e9) {
            dly.tv_nsec -= 1e9;
            dly.tv_sec++;
        }
        //printf("(%d) cnt:%d\n", sizeof(now.tv_nsec), cnt++);
        //printf("dly.tv_nsec:%lu\n", dly.tv_nsec);

        ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &dly, &rem);
        if (ret == EFAULT) {
            hal_run = false;
            printf("EFAULT\n");
        } else if (ret == EINTR) {
            printf("EINTR\n");
        } else if (ret == EINVAL) {
            printf("EINVAL\n");
            printf("dly.tv_nsec:%lu\n", dly.tv_nsec);
            printf("dly.tv_sec:%lu\n", dly.tv_sec);
            hal_run = false;
        }
    } // ..while (hal_run)
}

static void radio_init(void)
{
    int rf_chain, if_chain;
    for (rf_chain = 0; rf_chain < 2; rf_chain++) {
        radio[rf_chain].n_ifs = 0;
        for (if_chain = 0; if_chain < MAX_IF_PER_RADIO; if_chain++) {
            radio[rf_chain].valid[if_chain] = false;
            radio[rf_chain].rx_freqs[if_chain] = 0;
        }
    }
    initialized_radio = true;
}

int lgw_board_setconf(struct lgw_conf_board_s conf)
{
    if (!initialized_radio)
        radio_init();

    if (conf.lorawan_public)
        ppg = 0x34;
    else
        ppg = 0x12;

    printf("ppg:0x%02x\n", ppg);
    return LGW_HAL_SUCCESS;
}

int lgw_rxrf_setconf(uint8_t rf_chain, struct lgw_conf_rxrf_s conf)
{
    if (!initialized_radio)
        radio_init();

    if (conf.enable) {
        radio[rf_chain].center_freq = conf.freq_hz;
    }
    return LGW_HAL_SUCCESS;
}

int lgw_rxif_setconf(uint8_t if_chain, struct lgw_conf_rxif_s conf)
{
    if (!initialized_radio)
        radio_init();

    if (conf.enable) {
        uint8_t idx = radio[conf.rf_chain].n_ifs;
        radio[conf.rf_chain].rx_freqs[idx] = radio[conf.rf_chain].center_freq + conf.freq_hz;
        radio[conf.rf_chain].bws[idx] = conf.bandwidth;
        printf("if chain bw:%d\n", conf.bandwidth);
        radio[conf.rf_chain].drs[idx] = conf.datarate;
        radio[conf.rf_chain].if_chain_num[idx] = if_chain;
        radio[conf.rf_chain].valid[idx] = true;

        radio[conf.rf_chain].n_ifs++;
    }
    return LGW_HAL_SUCCESS;
}

uint32_t my_round(uint32_t hz)
{
    /* round to 10KHz */
    float i, fp;
    uint32_t ri;
    float ten_khz = (float)hz / (float)10000;
    //printf("hz:%u ten_khz:%f\n", hz, ten_khz);
    fp = modff(ten_khz, &i);
    //printf("ipart:%f, fp:%f\n", i, fp);

    if (fp >= 0.5)
        ri = (uint32_t)ceilf(ten_khz);
    else
        ri = (uint32_t)floorf(ten_khz);

    return ri * 10000;
}

/*void
test_round()
{
    uint32_t hz = 902899963;
    printf("test: %u\n", my_round(hz));
}*/

int lgw_start(void)
{
    int ret;

    /*test_round();
    return LGW_HAL_ERROR;*/

    if (!initialized_radio)
        radio_init();

    shared_memory1_id = shmget((key_t)SHM_KEY, sizeof(struct lora_shm_struct), 0666 | IPC_CREAT);	
    if (shared_memory1_id < 0) {
        perror("shmget");
        return LGW_HAL_ERROR;
    }

	shared_memory1_pointer = shmat(shared_memory1_id, (void *)0, 0);
	if (shared_memory1_pointer < 0)
	{
        perror("shmat");
        return LGW_HAL_ERROR;
    }

    shared_memory1 = (struct lora_shm_struct *)shared_memory1_pointer;

    if (clock_gettime (CLOCK_MONOTONIC, &shared_memory1->gw_start_ts) == -1) {
        perror ("clock_gettime");
        return LGW_HAL_ERROR;
    }
    printf("gw_start_ts: %lu, %lu\n",
        shared_memory1->gw_start_ts.tv_sec,
        shared_memory1->gw_start_ts.tv_nsec
    );
    /////////////////////////////////////////////////////////
    shared_memory1->size = sizeof(struct lora_shm_struct);
    /* clear any stale packets */
    for (ret = 0; ret < N_UPLINK_PACKETS; ret++)
        shared_memory1->uplink_packets[ret].has_packet = false;

    /////////////////////////////////////////////////////////
    hal_run = true;
    ret = pthread_create(&thrid_fakegps, NULL, (void * (*)(void *))thread_fakegps, NULL);
    if (ret != 0) {
        perror("pthread_create");
        return LGW_HAL_ERROR;
    }

    ret = pthread_create(&thrid_send, NULL, (void * (*)(void *))thread_send, NULL);
    if (ret != 0) {
        perror("pthread_create");
        return LGW_HAL_ERROR;
    }

    return LGW_HAL_SUCCESS;
}

int lgw_stop(void)
{
    hal_run = false;
    pthread_join(thrid_fakegps, NULL);
    pthread_join(thrid_send, NULL);

    if (shmdt(shared_memory1_pointer) < 0) {
        perror("shmdt");
        return LGW_HAL_ERROR;
    }

    if (shmctl(shared_memory1_id, IPC_RMID, 0) < 0) {
        perror("shmctl");
        return LGW_HAL_ERROR;
    }

    return LGW_HAL_SUCCESS;
}

int
rx_find_uplink_channel(uint16_t txer_bw_khz, uint32_t end_node_tx_freq_hz, uint8_t* rf_chain, uint8_t* if_chain)
{
    int _rf, _if;
    unsigned int smallest_abs_diff = INT_MAX;
    //printf("\n");
    /* did end-node transmit into one of our IF chains? */
    for (_rf = 0; _rf < 2; _rf++) {
        //printf("_rf:%d n_ifs:%d\n", _rf, radio[_rf].n_ifs);
        for (_if = 0; _if < radio[_rf].n_ifs; _if++) {
            //printf("_rf:%d _if:%d valid:%d\n", _rf, _if, radio[_rf].valid[_if]);
            if (radio[_rf].valid[_if]) {
                int abs_diff_hz = abs(radio[_rf].rx_freqs[_if] - end_node_tx_freq_hz);
                //printf("%d,%d diff:%u\n", _rf, _if, abs_diff_hz);
                if (abs_diff_hz < smallest_abs_diff) {
                    uint16_t rx_bw_khz;
                    if (radio[_rf].drs[_if] == DR_UNDEFINED)
                        rx_bw_khz = 125;    // multi-SF is always bw=125khz
                    else {
                        switch (radio[_rf].bws[_if]) {
                            case BW_500KHZ: rx_bw_khz = 500; break;
                            case BW_250KHZ: rx_bw_khz = 250; break;
                            case BW_125KHZ: rx_bw_khz = 125; break;
                            case BW_UNDEFINED:  rx_bw_khz = 0; break;
                        }
                    }
                    //printf("rxbw:%d txbw:%d\n", rx_bw_khz, txer_bw_khz);
                    *rf_chain = _rf;
                    *if_chain = radio[_rf].if_chain_num[_if];
                    //printf("smallest ");
                    smallest_abs_diff = abs_diff_hz;
                }
            }
            //printf("\n");
        }
    }

    return smallest_abs_diff;
}

int lgw_receive(uint8_t max_pkt, struct lgw_pkt_rx_s *pkt_data)
{
    int nb_pkt_fetch = 0; /* loop variable and return value */
    unsigned int uplink_idx;

    for (uplink_idx = 0; uplink_idx < N_UPLINK_PACKETS; uplink_idx++) {
        uint32_t hz_error_limit;
        struct uplink_struct* shm_uplink;
        unsigned int hz_freq_error;
        if (!shared_memory1->uplink_packets[uplink_idx].has_packet)
            continue;
        shm_uplink = &shared_memory1->uplink_packets[uplink_idx];
        struct lgw_pkt_rx_s *p = &pkt_data[nb_pkt_fetch];

        printf("%dhz bw:%u ", shm_uplink->freq_hz, shm_uplink->bw_khz);

        hz_freq_error = rx_find_uplink_channel(shm_uplink->bw_khz, shm_uplink->freq_hz, &p->rf_chain, &p->if_chain);
        hz_error_limit = (shm_uplink->bw_khz * 1000) / 4;
        if (hz_freq_error > hz_error_limit) {
            printf("freq_hz out of range: err=%u, limit=%u\n", hz_freq_error, hz_error_limit);
            shm_uplink->has_packet = false; // mark this uplink packet as taken
            continue;
        }
        p->freq_hz = my_round(shm_uplink->freq_hz);
        if (shm_uplink->ppg != ppg) {
            printf("tx-ppg:%02x, rx-ppg:%02x ", shm_uplink->ppg, ppg);
            shm_uplink->has_packet = false; // mark this uplink packet as taken
            continue;
        }
        printf("freq ok (%d) rf:%d, if:%d ", hz_freq_error, p->rf_chain, p->if_chain);

        switch (shm_uplink->bw_khz) {
            case 8: p->bandwidth = BW_7K8HZ; break;
            case 15: p->bandwidth = BW_15K6HZ; break;
            case 31: p->bandwidth = BW_31K2HZ; break;
            case 62: p->bandwidth = BW_62K5HZ; break;
            case 125: p->bandwidth = BW_125KHZ; break;
            case 250: p->bandwidth = BW_250KHZ; break;
            case 500: p->bandwidth = BW_500KHZ; break;
            default: p->bandwidth = BW_UNDEFINED; break;
        }

        printf("lgw_receive: size:%d ", shm_uplink->size);
        printf("count_us:%u ", shm_uplink->tx_end_count_us);

        p->status = STAT_CRC_OK; /* STAT_CRC_BAD, STAT_NO_CRC */
        p->size = shm_uplink->size;
        p->count_us = shm_uplink->tx_end_count_us;
        p->modulation = MOD_LORA;
        printf("sf:%d ", shm_uplink->sf);
        switch (shm_uplink->sf) {
            case 7: p->datarate = DR_LORA_SF7; break;
            case 8: p->datarate = DR_LORA_SF8; break;
            case 9: p->datarate = DR_LORA_SF9; break;
            case 10: p->datarate = DR_LORA_SF10; break;
            case 11: p->datarate = DR_LORA_SF11; break;
            case 12: p->datarate = DR_LORA_SF12; break;
            default: p->datarate = DR_UNDEFINED; break;
        }

        switch (shm_uplink->cr) {
            case 0: p->coderate = CR_UNDEFINED; break;
            case 1: p->coderate = CR_LORA_4_5; break;
            case 2: p->coderate = CR_LORA_4_6; break;
            case 3: p->coderate = CR_LORA_4_7; break;
            case 4: p->coderate = CR_LORA_4_8; break;
        }

        p->rssi = -50;  //  todo: relative to end-node tx power + path loss
        p->snr = 8.0;   // lora signal quality
        p->snr_min = -10;
        p->snr_max = 10;
        p->crc = 0xbeef;

        memcpy(p->payload, shm_uplink->payload, p->size);

        printf("\n");
        shm_uplink->has_packet = false; // mark this uplink packet as taken
        if (++nb_pkt_fetch == max_pkt)
            break;
    } // ..for (uplink_idx..)

    return nb_pkt_fetch;
}


int lgw_lbt_setconf(struct lgw_conf_lbt_s conf)
{
    return LGW_HAL_SUCCESS;
}

int lgw_txgain_setconf(struct lgw_tx_gain_lut_s *conf)
{
    return LGW_HAL_SUCCESS;
}

/* return current sx1301 time */
int lgw_get_trigcnt(uint32_t* trig_cnt_us)
{
	uint64_t ret64;
    struct timespec tp;

    if (clock_gettime (CLOCK_MONOTONIC, &tp) == -1) {
        perror ("clock_gettime");
        return LGW_HAL_ERROR;
    }

    tp.tv_sec -= shared_memory1->gw_start_ts.tv_sec;
    tp.tv_nsec -= shared_memory1->gw_start_ts.tv_nsec;
    if (tp.tv_nsec < 0) {
        tp.tv_nsec += 1000000000;
        tp.tv_sec--;
    }

	// convert to microseconds
	ret64 = tp.tv_sec * 1e6;
	ret64 += tp.tv_nsec / 1000;

    //printf("%u = lgw_get_trigcnt()\n", (uint32_t)ret64);
    *trig_cnt_us = (uint32_t)ret64;

    return LGW_HAL_SUCCESS;
}

int lgw_status(uint8_t select, uint8_t *code)
{
    if (select == TX_STATUS) {
        if (gps_tx_trigger)
            *code = TX_SCHEDULED;
        else
            *code = TX_FREE;
    } else if (select == RX_STATUS)
        *code = RX_STATUS_UNKNOWN; /* todo */
    else
        return LGW_HAL_ERROR;
    
    return LGW_HAL_SUCCESS;
}

//struct timespec tx_start_time, tx_end_time;

static double difftimespec(struct timespec end, struct timespec beginning) {
    double x;

    x = 1E-9 * (double)(end.tv_nsec - beginning.tv_nsec);
    x += (double)(end.tv_sec - beginning.tv_sec);

    return x;
}

void
parse_lgw_pkt_tx_s(struct lgw_pkt_tx_s* pkt_data)
{
    struct timespec ts;
    float symbol_period_seconds;
    uint32_t symbol_period_ns;
    downlink_t* dl = &shared_memory1->downlink;

    switch (pkt_data->bandwidth) {
        case BW_125KHZ: dl->bw_khz = 125; break;
        case BW_250KHZ: dl->bw_khz = 250; break;
        case BW_500KHZ: dl->bw_khz = 500; break;
        default: dl->bw_khz = 0; break;
    }

    switch (pkt_data->datarate) {
        case DR_LORA_SF7: dl->sf = 7; break;
        case DR_LORA_SF8: dl->sf = 8; break;
        case DR_LORA_SF9: dl->sf = 9; break;
        case DR_LORA_SF10: dl->sf = 10; break;
        case DR_LORA_SF11: dl->sf = 11; break;
        case DR_LORA_SF12: dl->sf = 12; break;
        default: dl->sf = 0; break;
    }

    if (pkt_data->preamble == 0) { /* if not explicit, use recommended LoRa preamble size */
        pkt_data->preamble = STD_LORA_PREAMBLE;
    } else if (pkt_data->preamble < MIN_LORA_PREAMBLE) { /* enforce minimum preamble size */
        pkt_data->preamble = MIN_LORA_PREAMBLE;
        printf("Note: preamble length adjusted to respect minimum LoRa preamble size\n");
    }

    symbol_period_seconds = (1 << dl->sf) / (float)(dl->bw_khz * 1000);
    //printf("symbol_period_seconds:%f\n", symbol_period_seconds);
    symbol_period_ns = symbol_period_seconds * 1000000000;

    ts.tv_sec = dl->tx_preamble_start_time.tv_sec;
    ts.tv_nsec = dl->tx_preamble_start_time.tv_nsec + (symbol_period_ns * pkt_data->preamble);
    if (ts.tv_nsec > 1000000000) {
        ts.tv_nsec -= 1000000000;
        ts.tv_sec++;
    }
    dl->tx_preamble_end_time.tv_sec = ts.tv_sec;
    dl->tx_preamble_end_time.tv_nsec = ts.tv_nsec;

    dl->freq_hz = pkt_data->freq_hz;
    dl->gw_tx_power = pkt_data->rf_power;
    dl->modulation = pkt_data->modulation;

    switch (pkt_data->coderate) {
        case CR_LORA_4_5: dl->cr = 1; break;
        case CR_LORA_4_6: dl->cr = 2; break;
        case CR_LORA_4_7: dl->cr = 3; break;
        case CR_LORA_4_8: dl->cr = 4; break;
        default: dl->cr = 0; break;
    }
    dl->iq_invert = pkt_data->invert_pol;
    dl->f_dev = pkt_data->f_dev;
    dl->preamble_length = pkt_data->preamble;
    dl->no_crc = pkt_data->no_crc;
    dl->no_header = pkt_data->no_header;
    dl->payload_size = pkt_data->size;
    memcpy(dl->payload, pkt_data->payload, pkt_data->size);
    dl->ppg = ppg;
}

void
tx_to_shared(bool timestamped_tx)
{
    int ret;
    struct timespec rem;
    downlink_t* dl = &shared_memory1->downlink;
    float symbol_period_seconds;

    if (timestamped_tx) {
        /* sleep until time to transmit */
        ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &dl->tx_preamble_start_time, &rem);
    }
    dl->transmitting = true;
    /* sleep for preamble duration */

    ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &dl->tx_preamble_end_time, &rem);
    dl->transmitting = false;

    symbol_period_seconds = (1 << dl->sf) / (float)(dl->bw_khz * 1000);
    /* wait some coarse period before clearing downlink */
    usleep((symbol_period_seconds * dl->payload_size * 8) * 1000000);
    
    dl->preamble_length = 0;
    memset(dl->payload, 0, dl->payload_size);
    dl->payload_size = 0;
    dl->sf = 0;
    dl->bw_khz = 0;
    dl->freq_hz = 0;
    dl->modulation = MOD_UNDEFINED;


    printf("tx-done\n");
}

/* from gps trigger: */
void
pps_tx(struct timespec* now)
{
    shared_memory1->downlink.tx_preamble_start_time.tv_nsec = now->tv_nsec;
    shared_memory1->downlink.tx_preamble_start_time.tv_sec = now->tv_sec;

    parse_lgw_pkt_tx_s(&g_pkt_data);
    tx_to_shared(false);
}

void print_now()
{
    struct timespec now;

    if (clock_gettime (CLOCK_MONOTONIC, &now) == -1)
        perror ("clock_gettime");
    printf("raw-now:%lu, %lu\n", now.tv_sec, now.tv_nsec);
}

int lgw_send(struct lgw_pkt_tx_s pkt_data)
{
    struct timespec now, ts;
    double seconds_to_tx;

    switch (pkt_data.tx_mode) {
        case TIMESTAMPED:
            if (gps_tx_trigger)
                return LGW_HAL_ERROR;
            // count_us is microseconds, convert to struct timespec

            if (clock_gettime (CLOCK_MONOTONIC, &now) == -1)
                perror ("clock_gettime");
            //print_now();
            uint32_t us_part = pkt_data.count_us % 1000000;    // get microsecond part
            ts.tv_nsec = us_part * 1000; // to nanoseconds
            pkt_data.count_us -= us_part;
            ts.tv_sec = pkt_data.count_us / 1000000;

            // restore using startup-offset
            ts.tv_sec += shared_memory1->gw_start_ts.tv_sec;
            ts.tv_nsec += shared_memory1->gw_start_ts.tv_nsec;
            if (ts.tv_nsec > 1000000000) {
                ts.tv_nsec -= 1000000000;
                ts.tv_sec++;
            }
            seconds_to_tx = difftimespec(ts, now);
            printf("TIMESTAMPED seconds_to_tx:%f\n", seconds_to_tx);

            shared_memory1->downlink.tx_preamble_start_time.tv_nsec = ts.tv_nsec;
            shared_memory1->downlink.tx_preamble_start_time.tv_sec = ts.tv_sec;

            parse_lgw_pkt_tx_s(&pkt_data);
            tx_to_shared(true);
            break;
        case ON_GPS:
            //printf("todo ON_GPS\n");
            memcpy(&g_pkt_data, &pkt_data, sizeof(struct lgw_pkt_tx_s));
            gps_tx_trigger = true;
            break;
        case IMMEDIATE:
            //return LGW_HAL_ERROR;
            if (clock_gettime (CLOCK_MONOTONIC, &now) == -1)
                perror ("clock_gettime");

            shared_memory1->downlink.tx_preamble_start_time.tv_nsec = now.tv_nsec;
            shared_memory1->downlink.tx_preamble_start_time.tv_sec = now.tv_sec;

            parse_lgw_pkt_tx_s(&pkt_data);
            tx_to_shared(false);
            break;
    }

    return LGW_HAL_SUCCESS;
}

int
lgw_reg_w(uint16_t register_id, int32_t reg_value)
{
    if (register_id != LGW_GPS_EN) {
        /* unhandled register write */
        fprintf(stderr, "TODO lgw_reg_w(%d)\n", register_id);
        exit(EXIT_FAILURE);
    }
}

const char* lgw_version_info() {
    return lgw_version_string;
}

