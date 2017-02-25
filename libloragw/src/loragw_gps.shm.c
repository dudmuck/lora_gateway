#include <stdio.h>      /* printf fprintf */
#include <unistd.h>      /* pipe */
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include "loragw_gps.h"
#include "lora_shm.h"

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#if DEBUG_GPS == 1
    #define DEBUG_MSG(args...)  fprintf(stderr, args)
    #define DEBUG_ARRAY(a,b,c)  for(a=0;a<b;++a) fprintf(stderr,"%x.",c[a]);fprintf(stderr,"end\n")
    #define CHECK_NULL(a)       if(a==NULL){fprintf(stderr,"%s:%d: ERROR: NULL POINTER AS ARGUMENT\n", __FUNCTION__, __LINE__);return LGW_GPS_ERROR;}
#else
    #define DEBUG_MSG(args...)
    #define DEBUG_ARRAY(a,b,c)  for(a=0;a!=0;){}
    #define CHECK_NULL(a)       if(a==NULL){return LGW_GPS_ERROR;}
#endif
#define TRACE()         fprintf(stderr, "@ %s %d\n", __FUNCTION__, __LINE__);

#define TS_CPS              1E6 /* count-per-second of the timestamp counter */
#define PLUS_10PPM          1.00001
#define MINUS_10PPM         0.99999

#define PIPE_MSG_SIZE       2
/********************************************************/
#define UTC_GPS_EPOCH_DIFF      315964781   /* observed difference */

#define PFDS_WRITE_IDX      1
#define PFDS_READ_IDX       0
int pfds[2];

bool hal_run;
bool pipe_open = false;

void pps_tx(struct timespec*);
extern struct lora_shm_struct *shared_memory1;  // from loragw_hal.shm.c

double _difftimespec(struct timespec end, struct timespec beginning);
void timespec_diff(struct timespec *start, struct timespec *stop,
                   struct timespec *result);

void
thread_fakegps(void)
{
    struct timespec dly, rem;
    struct timespec now;

    if (clock_gettime (CLOCK_REALTIME, &now) == -1)
        perror ("clock_gettime");

    dly.tv_sec = now.tv_sec;
    dly.tv_nsec = 5e8;

    while (hal_run) {
        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &dly, &rem);

        /* simulate NMEA RMC */
        if (pipe_open) {
            /* cause read() to unblock: lgw_gps_get() will be called */
            uint8_t buf[PIPE_MSG_SIZE];
            buf[0] = LGW_GPS_UBX_SYNC_CHAR;
            buf[1] = 0;
            write(pfds[PFDS_WRITE_IDX], buf, PIPE_MSG_SIZE);
        }

        dly.tv_sec++;
    }
}

int lgw_cnt2utc(struct tref ref, uint32_t count_us, struct timespec *utc)
{
    double delta_sec;
    double intpart, fractpart;
    long tmp;

    CHECK_NULL(utc);
    if ((ref.systime == 0) || (ref.xtal_err > PLUS_10PPM) || (ref.xtal_err < MINUS_10PPM)) {
        DEBUG_MSG("ERROR: INVALID REFERENCE FOR CNT -> UTC CONVERSION\n");
        return LGW_GPS_ERROR;
    }

    /* calculate delta in seconds between reference count_us and target count_us */
    delta_sec = (double)(count_us - ref.count_us) / (TS_CPS * ref.xtal_err);

    /* now add that delta to reference UTC time */
    fractpart = modf (delta_sec , &intpart);
    tmp = ref.utc.tv_nsec + (long)(fractpart * 1E9);
    if (tmp < (long)1E9) { /* the nanosecond part doesn't overflow */
        utc->tv_sec = ref.utc.tv_sec + (time_t)intpart;
        utc->tv_nsec = tmp;
    } else { /* must carry one second */
        utc->tv_sec = ref.utc.tv_sec + (time_t)intpart + 1;
        utc->tv_nsec = tmp - (long)1E9;
    }

    return LGW_GPS_SUCCESS;
}

int lgw_utc2cnt(struct tref ref, struct timespec utc, uint32_t *count_us)
{
    double delta_sec;

    CHECK_NULL(count_us);
    if ((ref.systime == 0) || (ref.xtal_err > PLUS_10PPM) || (ref.xtal_err < MINUS_10PPM)) {
        DEBUG_MSG("ERROR: INVALID REFERENCE FOR UTC -> CNT CONVERSION\n");
        return LGW_GPS_ERROR;
    }

    /* calculate delta in seconds between reference utc and target utc */
    delta_sec = (double)(utc.tv_sec - ref.utc.tv_sec);
    delta_sec += 1E-9 * (double)(utc.tv_nsec - ref.utc.tv_nsec);

    /* now convert that to internal counter tics and add that to reference counter value */
    *count_us = ref.count_us + (uint32_t)(delta_sec * TS_CPS * ref.xtal_err);

    return LGW_GPS_SUCCESS;
}

enum gps_msg lgw_parse_nmea(char* serial_buff, int buff_size)
{
    /* this function is called by 1PPS write to pipe from thread_fakegps() */
    return NMEA_RMC;
}

#ifdef ENABLE_HAL_UBX
int lgw_gps_get(struct timespec *utc, struct timespec *gps_time, struct coord_s *loc, struct coord_s *err)
#else
int lgw_gps_get(struct timespec *utc, struct coord_s *loc, struct coord_s *err)
#endif
{
    struct timespec now;
    if (clock_gettime (CLOCK_REALTIME, &now) == -1) {
        perror ("clock_gettime");
        return LGW_GPS_ERROR;
    }
    if (utc != NULL) {
        utc->tv_sec = now.tv_sec;
        utc->tv_nsec = 0;
    }

#ifdef ENABLE_HAL_UBX
    if (gps_time != NULL) {
        /* epoch difference between UTC and GPS time */
        gps_time->tv_sec = now.tv_sec - UTC_GPS_EPOCH_DIFF;
        gps_time->tv_nsec = 0;
    }
#endif

    return LGW_GPS_SUCCESS;
}

#ifdef ENABLE_HAL_UBX
int lgw_gps_sync(struct tref *ref, uint32_t count_us, struct timespec utc, struct timespec gps_time)
#else
int lgw_gps_sync(struct tref *ref, uint32_t count_us, struct timespec utc)
#endif
{
    ref->systime = time(NULL) - 1;  // -1 for gps_ref_age
    ref->count_us = count_us;
    ref->utc = utc;
#ifdef ENABLE_HAL_UBX
    ref->gps = gps_time;
#endif
    ref->xtal_err = 1.0;

    return LGW_GPS_SUCCESS;
}

int lgw_gps_enable(char *tty_path, char *gps_familly, speed_t target_brate, int *fd_ptr)
{
    if (pipe(pfds) < 0) {
        perror("pipe");
        return LGW_GPS_ERROR;
    } 

    *fd_ptr = pfds[PFDS_READ_IDX];  // give read end of pipe
    pipe_open = true;

    return LGW_GPS_SUCCESS;
}

int
lgw_cnt2gps(struct tref ref, uint32_t count_us, struct timespec* result)
{
    count_us_to_timespec(count_us, result);

#ifdef ENABLE_HAL_UBX
    result->tv_sec -= UTC_GPS_EPOCH_DIFF;
#endif

    return LGW_GPS_SUCCESS;
}

int lgw_gps2cnt(struct tref ref, struct timespec gps_time, uint32_t* count_us)
{
    uint64_t ret64;
    struct timespec result;

#ifdef ENABLE_HAL_UBX
    gps_time.tv_sec += UTC_GPS_EPOCH_DIFF;
#endif

    timespec_diff(&shared_memory1->gw_start_ts, &gps_time, &result);

    // convert to microseconds
    ret64 = result.tv_sec * 1e6;
    ret64 += result.tv_nsec / 1000;

    *count_us = (uint32_t)ret64;

    return LGW_GPS_SUCCESS;
}

enum gps_msg
lgw_parse_ubx(const char* serial_buff, size_t buff_size, size_t *msg_size)
{
    *msg_size = PIPE_MSG_SIZE;
    return UBX_NAV_TIMEGPS;
}

int lgw_gps_disable(int fd)
{
    return LGW_GPS_SUCCESS;
}
