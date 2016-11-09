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

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#if DEBUG_HAL == 1
    #define DEBUG_MSG(str)                fprintf(stderr, str)
    #define DEBUG_PRINTF(fmt, args...)    fprintf(stderr,"%s:%d: "fmt, __FUNCTION__, __LINE__, args)
    #define DEBUG_ARRAY(a,b,c)            for(a=0;a<b;++a) fprintf(stderr,"%x.",c[a]);fprintf(stderr,"end\n")
    #define CHECK_NULL(a)                 if(a==NULL){fprintf(stderr,"%s:%d: ERROR: NULL POINTER AS ARGUMENT\n", __FUNCTION__, __LINE__);return LGW_HAL_ERROR;}
#else
    #define DEBUG_MSG(str)
    #define DEBUG_PRINTF(fmt, args...)
    #define DEBUG_ARRAY(a,b,c)            for(a=0;a!=0;){}
    #define CHECK_NULL(a)                 if(a==NULL){return LGW_HAL_ERROR;}
#endif

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

uint32_t count_us_overflows; // overflows every 2^32 microseconds

static uint8_t fsk_sync_word_size = 3; /* default number of bytes for FSK sync word */

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
        if (clock_gettime (CLOCK_REALTIME, &now) == -1)
            perror ("clock_gettime");

        dly.tv_sec = now.tv_sec;
        dly.tv_nsec = now.tv_nsec + POLL_DELAY_NS;
        if (dly.tv_nsec > 1e9) {
            dly.tv_nsec -= 1e9;
            dly.tv_sec++;
        }
        //printf("(%d) cnt:%d\n", sizeof(now.tv_nsec), cnt++);
        //printf("dly.tv_nsec:%lu\n", dly.tv_nsec);

        ret = clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &dly, &rem);
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

        /* TODO: IF_FSK_STD */
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

    count_us_overflows = 0;
    if (clock_gettime (CLOCK_REALTIME, &shared_memory1->gw_start_ts) == -1) {
        perror ("clock_gettime");
        return LGW_HAL_ERROR;
    }
    printf("gw_start_ts: %lu.%lu\n",
        shared_memory1->gw_start_ts.tv_sec,
        shared_memory1->gw_start_ts.tv_nsec
    );
    /////////////////////////////////////////////////////////
    shared_memory1->size = sizeof(struct lora_shm_struct);
    shared_memory1->downlink.tx_cnt = 0;
    /* clear any stale packets */
    for (ret = 0; ret < N_UPLINK_PACKETS; ret++)
        shared_memory1->uplink_packets[ret].has_packet = false;

    if (clock_gettime (CLOCK_REALTIME, &shared_memory1->downlink.tx_done_time) == -1)
        perror ("clock_gettime");
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

    if (clock_gettime (CLOCK_REALTIME, &tp) == -1) {
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

int32_t lgw_bw_getval(int x) {
    switch (x) {
        case BW_500KHZ: return 500000;
        case BW_250KHZ: return 250000;
        case BW_125KHZ: return 125000;
        case BW_62K5HZ: return 62500;
        case BW_31K2HZ: return 31200;
        case BW_15K6HZ: return 15600;
        case BW_7K8HZ : return 7800;
        default: return -1;
    }
}

int32_t lgw_sf_getval(int x) {
    switch (x) {
        case DR_LORA_SF7: return 7;
        case DR_LORA_SF8: return 8;
        case DR_LORA_SF9: return 9;
        case DR_LORA_SF10: return 10;
        case DR_LORA_SF11: return 11;
        case DR_LORA_SF12: return 12;
        default: return -1;
    }
}

#define MSG_CYAN(...) printf("[46m" __VA_ARGS__ ); /* 46:bg-CYAN */ \
                    printf("[0m") 


uint32_t lgw_time_on_air_us(struct lgw_pkt_tx_s *packet)
{
    int32_t val;
    uint8_t SF, H, DE;
    float BW;
    uint32_t payloadSymbNb, Tpacket;
    double Tsym, Tpreamble, Tpayload, Tfsk;

    if (packet == NULL) {
        fprintf(stderr, "ERROR: Failed to compute time on air, wrong parameter\n");
        return 0;
    }

    if (packet->modulation == MOD_LORA) {
        /* Get bandwidth */
        val = lgw_bw_getval(packet->bandwidth);
        //printf("%d = lgw_bw_getval()\n", val);
        if (val != -1) {
            BW = val / (float)1e6;
            MSG_CYAN("BW:%f ", BW);
        } else {
            fprintf(stderr, "ERROR: Cannot compute time on air for this packet, unsupported bandwidth (0x%02X)\n", packet->bandwidth);
            return 0;
        }

        /* Get datarate */
        val = lgw_sf_getval(packet->datarate);
        if (val != -1) {
            SF = (uint8_t)val;
            MSG_CYAN("SF:%u ", SF);
        } else {
            fprintf(stderr, "ERROR: Cannot compute time on air for this packet, unsupported datarate (0x%02X)\n", packet->datarate);
            return 0;
        }

        /* Duration of 1 symbol */
        Tsym = pow(2, SF) / BW;
        //printf("Tsym:%f = %f / %f\n", Tsym, pow(2, SF), BW);

        /* Duration of preamble */
        //Tpreamble = (8 + 4.25) * Tsym; /* 8 programmed symbols in preamble */
        Tpreamble = (packet->preamble + 4.25) * Tsym; /* 8 programmed symbols in preamble */

        /* Duration of payload */
        //H = (packet->no_header==false) ? 0 : 1; /* header is always enabled, except for beacons */
        H = packet->no_header ? 1 : 0; /* header is always enabled, except for beacons */
        DE = (SF >= 11) ? 1 : 0; /* Low datarate optimization enabled for SF11 and SF12 */
        uint8_t crcOn = packet->no_crc ? 0: 1;
        MSG_CYAN("Tsym:%f Tpreamble:%f(%usymbs) H:%d(%d) DE:%d crc:%u ", Tsym, Tpreamble, packet->preamble, H, packet->no_header, DE, crcOn);

        payloadSymbNb = 8 + (ceil((double)(8*packet->size - 4*SF + 28 + 16*crcOn - 20*H) / (double)(4*(SF - 2*DE))) * (packet->coderate + 4)); /* Explicitely cast to double to keep precision of the division */

        Tpayload = payloadSymbNb * Tsym;
        MSG_CYAN(" (%ubytes) Tpayload:%f payloadSymbNb :%f ", packet->size, Tpayload, payloadSymbNb);

        /* Duration of packet */
        Tpacket = Tpreamble + Tpayload;
    } else if (packet->modulation == MOD_FSK) {
        /* PREAMBLE + SYNC_WORD + PKT_LEN + PKT_PAYLOAD + CRC
                PREAMBLE: default 5 bytes
                SYNC_WORD: default 3 bytes
                PKT_LEN: 1 byte (variable length mode)
                PKT_PAYLOAD: x bytes
                CRC: 0 or 2 bytes
        */
        Tfsk = (8 * (double)(packet->preamble + fsk_sync_word_size + 1 + packet->size + ((packet->no_crc == true) ? 0 : 2)) / (double)packet->datarate);

        /* Duration of packet */
        Tpacket = (uint32_t)Tfsk + 1; /* add margin for rounding */
    } else {
        Tpacket = 0;
        fprintf(stderr, "ERROR: Cannot compute time on air for this packet, unsupported modulation (0x%02X)\n", packet->modulation);
    }

    return Tpacket;
}

int
parse_lgw_pkt_tx_s(struct lgw_pkt_tx_s* pkt_data)
{
    struct timespec ts;
    float symbol_period_seconds;
    uint32_t symbol_period_ns;
    downlink_t* dl = &shared_memory1->downlink;
    uint32_t toa_us;

    if (pkt_data->modulation == MOD_LORA) {
        switch (pkt_data->bandwidth) {
            case BW_125KHZ: dl->bw_khz = 125; break;
            case BW_250KHZ: dl->bw_khz = 250; break;
            case BW_500KHZ: dl->bw_khz = 500; break;
            default: dl->bw_khz = 0; printf("bw:%d\n", pkt_data->bandwidth); return -1;
        }

        switch (pkt_data->datarate) {
            case DR_LORA_SF7: dl->sf = 7; break;
            case DR_LORA_SF8: dl->sf = 8; break;
            case DR_LORA_SF9: dl->sf = 9; break;
            case DR_LORA_SF10: dl->sf = 10; break;
            case DR_LORA_SF11: dl->sf = 11; break;
            case DR_LORA_SF12: dl->sf = 12; break;
            default: dl->sf = 0; printf("sf:%d\n", pkt_data->datarate); return -1;
        }

        switch (pkt_data->coderate) {
            case CR_LORA_4_5: dl->cr = 1; break;
            case CR_LORA_4_6: dl->cr = 2; break;
            case CR_LORA_4_7: dl->cr = 3; break;
            case CR_LORA_4_8: dl->cr = 4; break;
            default: dl->cr = 0; printf("cr:%d\n", pkt_data->coderate); return -1;
        }

        if (pkt_data->preamble == 0) { /* if not explicit, use recommended LoRa preamble size */
            pkt_data->preamble = STD_LORA_PREAMBLE;
        } else if (pkt_data->preamble < MIN_LORA_PREAMBLE) { /* enforce minimum preamble size */
            pkt_data->preamble = MIN_LORA_PREAMBLE;
            printf("Note: preamble length adjusted to respect minimum LoRa preamble size\n");
        }

        toa_us = lgw_time_on_air_us(pkt_data);
        printf("toa_us:%u\n", toa_us);

        symbol_period_seconds = (1 << dl->sf) / (float)(dl->bw_khz * 1000);
        //printf("symbol_period_seconds:%f\n", symbol_period_seconds);
        symbol_period_ns = symbol_period_seconds * 1000000000;
    } else {
        printf("todo tx-modulation\n");
        return -1;
    }

    ts.tv_sec = dl->tx_preamble_start_time.tv_sec;
    ts.tv_nsec = dl->tx_preamble_start_time.tv_nsec + (symbol_period_ns * pkt_data->preamble);
    if (ts.tv_nsec > 1000000000) {
        ts.tv_nsec -= 1000000000;
        ts.tv_sec++;
    }
    dl->tx_preamble_end_time.tv_sec = ts.tv_sec;
    dl->tx_preamble_end_time.tv_nsec = ts.tv_nsec;

    dl->tx_done_time.tv_sec = dl->tx_preamble_start_time.tv_sec;
    while (toa_us > 1000000) {
        dl->tx_done_time.tv_sec++;
        toa_us -= 1000000;
    }
    dl->tx_done_time.tv_nsec = dl->tx_preamble_start_time.tv_nsec + (toa_us * 1000);
    if (dl->tx_done_time.tv_nsec > 1000000000) {
        dl->tx_done_time.tv_sec++;
        dl->tx_done_time.tv_nsec -= 1000000000;
    }
    //printf("tx_done_time:%u.%u\n", dl->tx_done_time.tv_sec, dl->tx_done_time.tv_nsec);

    dl->freq_hz = pkt_data->freq_hz;
    dl->gw_tx_power = pkt_data->rf_power;
    dl->modulation = pkt_data->modulation;

    dl->iq_invert = pkt_data->invert_pol;
    dl->f_dev = pkt_data->f_dev;
    dl->preamble_length = pkt_data->preamble;
    dl->no_crc = pkt_data->no_crc;
    dl->no_header = pkt_data->no_header;
    dl->payload_size = pkt_data->size;
    memcpy(dl->payload, pkt_data->payload, pkt_data->size);
    dl->ppg = ppg;

    return 0;
}

void
tx_to_shared(bool timestamped_tx)
{
    int ret;
    struct timespec rem;
    downlink_t* dl = &shared_memory1->downlink;
    //float symbol_period_seconds;

    if (timestamped_tx) {
        /* sleep until time to transmit */
        ret = clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &dl->tx_preamble_start_time, &rem);
    }
    dl->transmitting = true;
    dl->tx_cnt++;
    printf("gwtx at %uhz sf%dbw%u\n", dl->freq_hz, dl->sf, dl->bw_khz);

    /* sleep for preamble duration 
    ret = clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &dl->tx_preamble_end_time, &rem);*/
    ret = clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &dl->tx_done_time, &rem);
    dl->transmitting = false;

#if 0
    symbol_period_seconds = (1 << dl->sf) / (float)(dl->bw_khz * 1000);
    /* wait some coarse period before clearing downlink */
    //usleep((symbol_period_seconds * dl->payload_size * 8) * 1000000);
    usleep(symbol_period_seconds * 1000000 * 8);
#endif /* #if 0 */

    
    dl->preamble_length = 0;
    memset(dl->payload, 0, dl->payload_size);
    dl->payload_size = 0;
    dl->sf = 0;
    dl->bw_khz = 0;
    dl->freq_hz = 0;
    dl->modulation = MOD_UNDEFINED;

    //printf("tx-done\n");
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

    if (clock_gettime (CLOCK_REALTIME, &now) == -1)
        perror ("clock_gettime");
    printf("raw-now:%lu, %lu\n", now.tv_sec, now.tv_nsec);
}

int lgw_send(struct lgw_pkt_tx_s pkt_data)
{
    struct timespec now, ts;
    double seconds_to_tx;
    uint64_t count_us;
    uint32_t trigcnt;

    switch (pkt_data.tx_mode) {
        case TIMESTAMPED:
            lgw_get_trigcnt(&trigcnt);
/*            if (trigcnt >= pkt_data.count_us) {
                printf("[31mlgw_send() count_us in past: %u >= %u[0m\n", trigcnt, pkt_data.count_us);
                return LGW_HAL_ERROR;
            }*/
            if ((pkt_data.count_us - trigcnt) > 128000000) {
                printf("[31mlgw_send() count_us too far in future:%u[0m\n", pkt_data.count_us - trigcnt);
                return LGW_HAL_ERROR;
            }
            printf("lgw_send() TIMESTAMPED count_us:%u diff:%u\n", pkt_data.count_us, pkt_data.count_us - trigcnt);
            if (gps_tx_trigger) {
                printf("[31mlgw_send() gps_tx_trigger set[0m\n");
                return LGW_HAL_ERROR;
            }
            //count_us_overflows

            if (clock_gettime (CLOCK_REALTIME, &now) == -1)
                perror ("clock_gettime");
            //print_now();
            do {
                // count_us is microseconds, convert to struct timespec
                uint64_t ovfl = count_us_overflows;
                uint32_t us_part;
                count_us = ovfl << 32;
                count_us += pkt_data.count_us;
                us_part = count_us % 1000000;    // get microsecond part
                ts.tv_nsec = us_part * 1000; // to nanoseconds
                count_us -= us_part;
                ts.tv_sec = count_us / 1000000;

                // restore using startup-offset
                ts.tv_sec += shared_memory1->gw_start_ts.tv_sec;
                ts.tv_nsec += shared_memory1->gw_start_ts.tv_nsec;
                if (ts.tv_nsec > 1000000000) {
                    ts.tv_nsec -= 1000000000;
                    ts.tv_sec++;
                }
                seconds_to_tx = difftimespec(ts, now);
                if (seconds_to_tx < 0) {
                    count_us_overflows++;
                    printf("count_us overflow %f, %u\n", seconds_to_tx, count_us_overflows);
                }
            } while (seconds_to_tx < 0);
            //printf("TIMESTAMPED seconds_to_tx:%f\n", seconds_to_tx);

            if (difftimespec(ts, shared_memory1->downlink.tx_done_time) < 0) {
                printf("[31mtx begin is before previous tx end[0m\n");
                return LGW_HAL_ERROR;
            }

            shared_memory1->downlink.tx_preamble_start_time.tv_nsec = ts.tv_nsec;
            shared_memory1->downlink.tx_preamble_start_time.tv_sec = ts.tv_sec;

            if (parse_lgw_pkt_tx_s(&pkt_data) < 0) {
                printf("[31mparse_lgw_pkt_tx_s() failed[0m\n");
                return LGW_HAL_ERROR;
            }
            tx_to_shared(true);
            break;
        case ON_GPS:
            printf("lgw_send() ON_GPS\n");
            memcpy(&g_pkt_data, &pkt_data, sizeof(struct lgw_pkt_tx_s));
            gps_tx_trigger = true;
            break;
        case IMMEDIATE:
            printf("lgw_send() IMMEDIATE\n");
            if (clock_gettime (CLOCK_REALTIME, &now) == -1)
                perror ("clock_gettime");

            //printf("         now:%u.%u\n", now.tv_sec, now.tv_nsec);
            //printf("prev tx_done:%u.%u\n", shared_memory1->downlink.tx_done_time.tv_sec, shared_memory1->downlink.tx_done_time.tv_nsec);
            if (difftimespec(now, shared_memory1->downlink.tx_done_time) < 0) {
                printf("tx immediate: already transmitting\n");
                return LGW_HAL_ERROR;
            }

            shared_memory1->downlink.tx_preamble_start_time.tv_nsec = now.tv_nsec;
            shared_memory1->downlink.tx_preamble_start_time.tv_sec = now.tv_sec;

            if (parse_lgw_pkt_tx_s(&pkt_data) < 0)
                return LGW_HAL_ERROR;
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

uint32_t lgw_time_on_air(struct lgw_pkt_tx_s *packet) {
    int32_t val;
    uint8_t SF, H, DE;
    uint16_t BW;
    uint32_t payloadSymbNb, Tpacket;
    double Tsym, Tpreamble, Tpayload, Tfsk;

    if (packet == NULL) {
        DEBUG_MSG("ERROR: Failed to compute time on air, wrong parameter\n");
        return 0;
    }

    if (packet->modulation == MOD_LORA) {
        /* Get bandwidth */
        val = lgw_bw_getval(packet->bandwidth);
        if (val != -1) {
            BW = (uint16_t)(val / 1E3);
        } else {
            DEBUG_PRINTF("ERROR: Cannot compute time on air for this packet, unsupported bandwidth (0x%02X)\n", packet->bandwidth);
            return 0;
        }

        /* Get datarate */
        val = lgw_sf_getval(packet->datarate);
        if (val != -1) {
            SF = (uint8_t)val;
        } else {
            DEBUG_PRINTF("ERROR: Cannot compute time on air for this packet, unsupported datarate (0x%02X)\n", packet->datarate);
            return 0;
        }

        /* Duration of 1 symbol */
        Tsym = pow(2, SF) / BW;

        /* Duration of preamble */
        Tpreamble = (8 + 4.25) * Tsym; /* 8 programmed symbols in preamble */

        /* Duration of payload */
        H = (packet->no_header==false) ? 0 : 1; /* header is always enabled, except for beacons */
        DE = (SF >= 11) ? 1 : 0; /* Low datarate optimization enabled for SF11 and SF12 */

        payloadSymbNb = 8 + (ceil((double)(8*packet->size - 4*SF + 28 + 16 - 20*H) / (double)(4*(SF - 2*DE))) * (packet->coderate + 4)); /* Explicitely cast to double to keep precision of the division */

        Tpayload = payloadSymbNb * Tsym;

        /* Duration of packet */
        Tpacket = Tpreamble + Tpayload;
    } else if (packet->modulation == MOD_FSK) {
        /* PREAMBLE + SYNC_WORD + PKT_LEN + PKT_PAYLOAD + CRC
                PREAMBLE: default 5 bytes
                SYNC_WORD: default 3 bytes
                PKT_LEN: 1 byte (variable length mode)
                PKT_PAYLOAD: x bytes
                CRC: 0 or 2 bytes
        */
        Tfsk = (8 * (double)(packet->preamble + fsk_sync_word_size + 1 + packet->size + ((packet->no_crc == true) ? 0 : 2)) / (double)packet->datarate) * 1E3;

        /* Duration of packet */
        Tpacket = (uint32_t)Tfsk + 1; /* add margin for rounding */
    } else {
        Tpacket = 0;
        DEBUG_PRINTF("ERROR: Cannot compute time on air for this packet, unsupported modulation (0x%02X)\n", packet->modulation);
    }

    return Tpacket;
}

