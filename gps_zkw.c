#include <errno.h>
#include <pthread.h>
#include <fcntl.h>
#include <sys/epoll.h>
#include <math.h>
#include <time.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netdb.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <semaphore.h>

#include <termios.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>

#define LOCATION_NLP_FIX "/data/misc/gps/LOCATION.DAT"
#define C_INVALID_FD -1
#define LOG_TAG  "gps_zkw"
#include <cutils/log.h>
#include <cutils/sockets.h>
#include <cutils/properties.h>
#ifdef HAVE_LIBC_SYSTEM_PROPERTIES
#define _REALLY_INCLUDE_SYS__SYSTEM_PROPERTIES_H_
#include <sys/_system_properties.h>
#endif
#include <hardware/gps.h>

#define MAJOR_NO	9
#define MINOR_NO	7
#define UNUSED(x) (void)(x)
#define MEASUREMENT_SUPPLY      0
/* the name of the controlled socket */
#define GPS_CHANNEL_NAME        "/dev/ttyAMA3"
//#define TTY_BAUD                B9600
#define TTY_BAUD                B115200

#define REDUCE_SV_FREQ		0	// 0 - OFF, 1 - When gps_state_start(), 2 - When tty payload is high.
#define TTY_BOOST		0

#define SVID_PLUS_GLONASS       64
#define SVID_PLUS_GALILEO       100
#define SVID_PLUS_BEIDOU        200

#define  GPS_DEBUG  1
#define  NMEA_DEBUG 1   /*the flag works if GPS_DEBUG is defined*/
#if GPS_DEBUG
#define TRC(f)      ALOGD("%s", __func__)
#define ERR(f, ...) ALOGE("%s: line = %d, " f, __func__, __LINE__, ##__VA_ARGS__)
#define WAN(f, ...) ALOGW("%s: line = %d, " f, __func__, __LINE__, ##__VA_ARGS__)
#define DBG(f, ...) ALOGD("%s: line = %d, " f, __func__, __LINE__, ##__VA_ARGS__)
#define VER(f, ...) ALOGD("%s: line = %d, " f, __func__, __LINE__, ##__VA_ARGS__)
//#define VER(f, ...) ((void)0)    // ((void)0)   //
#else
#  define DBG(...)    ((void)0)
#  define VER(...)    ((void)0)
#  define ERR(...)    ((void)0)
#endif
static GpsStatus gps_status;
static int flag_unlock = 0;
const char* gps_native_thread = "GPS NATIVE THREAD";
static GpsCallbacks callback_backup;
static GpsMeasurementCallbacks measurement_callbacks;
static int is_measurement = 0;
static float report_time_interval = 0;
static int started = 0;
/*****************************************************************************/

/*****************************************************************/
/*****************************************************************/
/*****                                                       *****/
/*****       C O N N E C T I O N   S T A T E                 *****/
/*****                                                       *****/
/*****************************************************************/
/*****************************************************************/

/* commands sent to the gps thread */
enum {
        CMD_QUIT  = 0,
        CMD_START = 1,
        CMD_STOP  = 2,
        CMD_RESTART = 3,
        CMD_DOWNLOAD = 4,

        CMD_TEST_START = 10,
        CMD_TEST_STOP = 11,
        CMD_TEST_SMS_NO_RESULT = 12,
};

static int gps_nmea_end_tag = 0;

/*****************************************************************/
/*****************************************************************/
/*****                                                       *****/
/*****       N M E A   T O K E N I Z E R                     *****/
/*****                                                       *****/
/*****************************************************************/
/*****************************************************************/

typedef struct {
        const char*  p;
        const char*  end;
} Token;

#define  MAX_NMEA_TOKENS  24

typedef struct {
        int     count;
        Token   tokens[ MAX_NMEA_TOKENS ];
} NmeaTokenizer;

static int
nmea_tokenizer_init(NmeaTokenizer*  t, const char*  p, const char*  end)
{
        int    count = 0;
        //char*  q;

        //  the initial '$' is optional
        if (p < end && p[0] == '$')
                p += 1;

        //  remove trailing newline
        if (end > p && (*(end-1) == '\n')) {
                end -= 1;
                if (end > p && (*(end-1) == '\r'))
                        end -= 1;
        }

        //  get rid of checksum at the end of the sentecne
        if (end >= p+3 && (*(end-3) == '*')) {
                end -= 3;
        }

        while (p < end) {
                const char*  q = p;

                q = memchr(p, ',', end-p);
                if (q == NULL)
                        q = end;

                if (q >= p) {
                        if (count < MAX_NMEA_TOKENS) {
                                t->tokens[count].p   = p;
                                t->tokens[count].end = q;
                                count += 1;
                        }
                }
                if (q < end)
                        q += 1;

                p = q;
        }

        t->count = count;
        return count;
}

static Token
nmea_tokenizer_get(NmeaTokenizer*  t, int  index)
{
        Token  tok;
        static const char*  dummy = "";

        if (index < 0 || index >= t->count) {
                tok.p = tok.end = dummy;
        } else
                tok = t->tokens[index];

        return tok;
}


static int
str2int(const char*  p, const char*  end)
{
        int   result = 0;
        int   len    = end - p;
        int   sign = 1;

        if (*p == '-')
        {
                sign = -1;
                p++;
                len = end - p;
        }

        for (; len > 0; len--, p++)
        {
                int  c;

                if (p >= end)
                        goto Fail;

                c = *p - '0';
                if ((unsigned)c >= 10)
                        goto Fail;

                result = result*10 + c;
        }
        return  sign*result;

Fail:
        return -1;
}

static double
str2float(const char*  p, const char*  end)
{
        //int   result = 0;
        int   len    = end - p;
        char  temp[16];

        if (len >= (int)sizeof(temp))
                return 0.;

        memcpy(temp, p, len);
        temp[len] = 0;
        return strtod(temp, NULL);
}

/*****************************************************************/
/*****************************************************************/
/*****                                                       *****/
/*****       N M E A   P A R S E R                           *****/
/*****                                                       *****/
/*****************************************************************/
/*****************************************************************/

/*
struct svinfo_raw {
        short constell;
        short prn;
        short azim;
        short elev;
        unsigned char cn0;
        unsigned char is_used; 
};
*/

#define HAS_CARRIER_FREQUENCY(p)        ((p) >= '1' && (p) <= '9')
#define MAX_GNSS_SVID   320
#define  NMEA_MAX_SIZE  256
/*maximum number of SV information in GPGSV*/
#define  NMEA_MAX_SV_INFO 4
#define  LOC_FIXED(pNmeaReader) ((pNmeaReader->fix_mode == 2) || (pNmeaReader->fix_mode ==3))
typedef struct {
        int     pos;
        int     overflow;
        int     utc_year;
        int     utc_mon;
        int     utc_day;
        int     utc_diff;
        GpsLocation  fix;

        /*
         * The fix flag extracted from GPGSA setence: 1: No fix; 2 = 2D; 3 = 3D
         * if the fix mode is 0, no location will be reported via callback
         * otherwise, the location will be reported via callback
         */
        int             fix_mode;
        /*
         * Indicate that the status of callback handling.
         * The flag is used to report GPS_STATUS_SESSION_BEGIN or GPS_STATUS_SESSION_END:
         * (0) The flag will be set as true when callback setting is changed via nmea_reader_set_callback
         * (1) GPS_STATUS_SESSION_BEGIN: receive location fix + flag set + callback is set
         * (2) GPS_STATUS_SESSION_END:   receive location fix + flag set + callback is null
         */
        int             cb_status_changed;
        //GpsSvStatus     sv_status_gps;
        GnssSvStatus    sv_status_gnss;
        GpsCallbacks    callbacks;
        GnssData        gnss_data;
        char            in[ NMEA_MAX_SIZE+1 ];
        int             sv_status_can_report;
        int             location_can_report;
        char            sv_used_in_fix[MAX_GNSS_SVID];
	double 		hdop;
        //GnssSvInfo      svlst[MAX_GNSS_SVID];
} NmeaReader;


static void
nmea_reader_update_utc_diff(NmeaReader* const r)
{
        time_t         now = time(NULL);
        struct tm      tm_local;
        struct tm      tm_utc;
        unsigned long  time_local, time_utc;

        gmtime_r(&now, &tm_utc);
        localtime_r(&now, &tm_local);


        time_local = mktime(&tm_local);


        time_utc = mktime(&tm_utc);

        r->utc_diff = time_utc - time_local;
}


static void
nmea_reader_init(NmeaReader* const r)
{
        memset(r, 0, sizeof(*r));

        r->pos      = 0;
        r->overflow = 0;
        r->utc_year = -1;
        r->utc_mon  = -1;
        r->utc_day  = -1;
        r->utc_diff = 0;
        r->callbacks.location_cb= NULL;
        r->callbacks.status_cb= NULL;
        r->callbacks.sv_status_cb= NULL;
        //r->sv_count = 0;
        r->fix_mode = 0;    /*no fix*/
        r->cb_status_changed = 0;
	r->hdop = 99.0;
        memset((void*)&r->sv_status_gnss, 0x00, sizeof(r->sv_status_gnss));
        memset((void*)&r->in, 0x00, sizeof(r->in));

        nmea_reader_update_utc_diff(r);
}

static void
nmea_reader_set_callback(NmeaReader* const r, GpsCallbacks* const cbs)
{
        if (!r) {           /*this should not happen*/
                return;
        } else if (!cbs) {  /*unregister the callback */
                return;
        } else {/*register the callback*/
                r->fix.flags = 0;
                //r->sv_count = 0;
                //r->sv_status_gps.num_svs = 0;
                r->sv_status_gnss.num_svs = 0;
                // memset(r->svlst, 0, sizeof(r->svlst));
        }
}


static int
nmea_reader_update_time(NmeaReader* const r, Token  tok)
{
        int        hour, minute;
        double     seconds;
        struct tm  tm;
        struct tm  tm_local;
        time_t     fix_time;

        if (tok.p + 6 > tok.end)
                return -1;

        memset((void*)&tm, 0x00, sizeof(tm));
        if (r->utc_year < 0) {
                //  no date yet, get current one
                time_t  now = time(NULL);
                gmtime_r(&now, &tm);
                r->utc_year = tm.tm_year + 1900;
                r->utc_mon  = tm.tm_mon + 1;
                r->utc_day  = tm.tm_mday;
        }

        hour    = str2int(tok.p,   tok.p+2);
        minute  = str2int(tok.p+2, tok.p+4);
        seconds = str2float(tok.p+4, tok.end);

        tm.tm_hour = hour;
        tm.tm_min  = minute;
        tm.tm_sec  = (int) seconds;
        tm.tm_year = r->utc_year - 1900;
        tm.tm_mon  = r->utc_mon - 1;
        tm.tm_mday = r->utc_day;
        tm.tm_isdst = -1;

        if (mktime(&tm) == (time_t)-1)
                ERR("mktime error: %d %s\n", errno, strerror(errno));

        fix_time = mktime(&tm);
        localtime_r(&fix_time, &tm_local);

        // fix_time += tm_local.tm_gmtoff;
        // DBG("fix_time: %d\n", (int)fix_time);
        r->fix.timestamp = (long long)fix_time * 1000;
        return 0;
}

static int
nmea_reader_update_date(NmeaReader* const r, Token  date, Token  time)
{
        Token  tok = date;
        int    day, mon, year;

        if (tok.p + 6 != tok.end) {
                ERR("date not properly formatted: '%.*s'", (int)(tok.end-tok.p), tok.p);
                return -1;
        }
        day  = str2int(tok.p, tok.p+2);
        mon  = str2int(tok.p+2, tok.p+4);
        year = str2int(tok.p+4, tok.p+6) + 2000;

        if ((day|mon|year) < 0) {
                ERR("date not properly formatted: '%.*s'", (int)(tok.end-tok.p), tok.p);
                return -1;
        }

        r->utc_year  = year;
        r->utc_mon   = mon;
        r->utc_day   = day;

        return nmea_reader_update_time(r, time);
}


static double
convert_from_hhmm(Token  tok)
{
        double  val     = str2float(tok.p, tok.end);
        int     degrees = (int)(floor(val) / 100);
        double  minutes = val - degrees*100.;
        double  dcoord  = degrees + minutes / 60.0;
        return dcoord;
}


static int
nmea_reader_update_latlong(NmeaReader* const r,
                           Token        latitude,
                           char         latitudeHemi,
                           Token        longitude,
                           char         longitudeHemi)
{
        double   lat, lon;
        Token    tok;

        tok = latitude;
        if (tok.p + 6 > tok.end) {
                ERR("latitude is too short: '%.*s'", (int)(tok.end-tok.p), tok.p);
                return -1;
        }
        lat = convert_from_hhmm(tok);
        if (latitudeHemi == 'S')
                lat = -lat;

        tok = longitude;
        if (tok.p + 6 > tok.end) {
                ERR("longitude is too short: '%.*s'", (int)(tok.end-tok.p), tok.p);
                return -1;
        }
        lon = convert_from_hhmm(tok);
        if (longitudeHemi == 'W')
                lon = -lon;

        r->fix.flags    |= GPS_LOCATION_HAS_LAT_LONG;
        r->fix.latitude  = lat;
        r->fix.longitude = lon;
        return 0;
}
/* this is the state of our connection to the daemon */
typedef struct {
        int                     init;
        int                     fd;
        GpsCallbacks            callbacks;
        pthread_t               thread;
        int                     control[2];
        //int                     fake_fd;
        int                     epoll_fd;
        int                     flag;
        int                     start_flag;
        //   int                     thread_exit_flag;
} GpsState;

static GpsState  _gps_state[1];

static int
nmea_reader_update_altitude(NmeaReader* const r,
                            Token        altitude,
                            Token        units)
{
	UNUSED(units); //double  alt;
        Token   tok = altitude;
        if (tok.p >= tok.end)
                return -1;

        r->fix.flags   |= GPS_LOCATION_HAS_ALTITUDE;
        r->fix.altitude = str2float(tok.p, tok.end);
        return 0;
}


static int
nmea_reader_update_bearing(NmeaReader* const r,
                           Token        bearing)
{
        //double  alt;
        Token   tok = bearing;

        if (tok.p >= tok.end)
                return -1;

        r->fix.flags   |= GPS_LOCATION_HAS_BEARING;
        r->fix.bearing  = str2float(tok.p, tok.end);
        return 0;
}


static int
nmea_reader_update_speed(NmeaReader* const r,
                         Token        speed)
{
        //double  alt;
        Token   tok = speed;

        if (tok.p >= tok.end)
                return -1;

        r->fix.flags   |= GPS_LOCATION_HAS_SPEED;

        // knot to m/s
        r->fix.speed = str2float(tok.p, tok.end) / 1.942795467;
        return 0;
}

// Add by LCH for accuracy
static void
nmea_reader_update_accuracy(NmeaReader* const r, double hdop)
{
        r->fix.flags   |= GPS_LOCATION_HAS_ACCURACY;
        r->fix.accuracy = hdop;
}

/*
static int
nmea_reader_update_sv_status_gps(NmeaReader* r, int sv_index,
                              int id, Token elevation,
                              Token azimuth, Token snr)
{
       // int prn = str2int(id.p, id.end);
    int prn = id;
    if ((prn <= 0) || (prn < 65 && prn > GPS_MAX_SVS)|| (prn > 96) || (r->sv_count >= GPS_MAX_SVS)) {
        VER("sv_status_gps: ignore (%d)", prn);
        return 0;
    }
    sv_index = r->sv_count+r->sv_status_gps.num_svs;
    if (GPS_MAX_SVS <= sv_index) {
        ERR("ERR: sv_index=[%d] is larger than GPS_MAX_SVS.\n", sv_index);
        return 0;
    }
    r->sv_status_gps.sv_list[sv_index].prn = prn;
    r->sv_status_gps.sv_list[sv_index].snr = str2float(snr.p, snr.end);
    r->sv_status_gps.sv_list[sv_index].elevation = str2int(elevation.p, elevation.end);
    r->sv_status_gps.sv_list[sv_index].azimuth = str2int(azimuth.p, azimuth.end);
    r->sv_count++;
    VER("sv_status_gps(%2d): %2d, %2f, %3f, %2f", sv_index,
       r->sv_status_gps.sv_list[sv_index].prn, r->sv_status_gps.sv_list[sv_index].elevation,
       r->sv_status_gps.sv_list[sv_index].azimuth, r->sv_status_gps.sv_list[sv_index].snr);
    return 0;
}
*/


/*
 * GPS's prn: 1-32
 * GLN's prn: 65-96
 * BDS's prn: 201-237
 */
static int
prn2svid(int prn, int constell)
{
        if (prn <= 0)
                return 0;

        switch (constell) {
        case GNSS_CONSTELLATION_GPS:
                break;
        case GNSS_CONSTELLATION_GLONASS:
                if (prn > 0 && prn <= 32)
                        prn += SVID_PLUS_GLONASS;
                break;
        case GNSS_CONSTELLATION_GALILEO:
                if (prn > 0 && prn <= 32)
                        prn += SVID_PLUS_GALILEO;
                break;
        case GNSS_CONSTELLATION_BEIDOU:
                if (prn > 0 && prn <= 64)
                        prn += SVID_PLUS_BEIDOU;
                break;
        default:
                break;
        }
        return prn;
}

/*
static int 
get_svid(int prn, int sv_type)
{
        if (sv_type == GNSS_CONSTELLATION_GLONASS && prn >= 1 && prn <= 32)
                return prn + 64;
        else if (sv_type == GNSS_CONSTELLATION_BEIDOU && prn >= 1 && prn <= 32)
                return prn + 200;

        return prn;
}
*/


static float
get_carrier_frequency_hz(char frq, int sv_type)
{
        float mhz = 0;
        switch (sv_type) {
        case GNSS_CONSTELLATION_GPS:
                if (frq == '1')
                        mhz = 1575.42f;
                else if (frq == '7')
                        mhz = 1176.45f;
                break;
        case GNSS_CONSTELLATION_BEIDOU:
                if (frq == '1')
                        mhz = 1561.098f;
                else if (frq == '7')
                        mhz = 1575.42f;
                else if (frq == '8')
                        mhz = 1176.45f;
                break;
        case GNSS_CONSTELLATION_GLONASS:
                break;
        default:
                break;
        }
        
        return mhz * 1000000.0;
}

static int
nmea_reader_update_gnss_measurement(NmeaReader*r, int sv_type, int prn, Token frq)
{
        int n = r->gnss_data.measurement_count;
        if (GNSS_MAX_MEASUREMENT <= n) {
                ERR("ERR: n=[%d] is larger than GNSS_MAX_MEASUREMENT.\n", n);
                return 1;
        }
        r->gnss_data.measurements[n].size = sizeof(GnssMeasurement);
        r->gnss_data.measurements[n].svid = prn;
        r->gnss_data.measurements[n].constellation = sv_type;
        r->gnss_data.measurements[n].flags |= GNSS_MEASUREMENT_HAS_CARRIER_FREQUENCY; 
        r->gnss_data.measurements[n].carrier_frequency_hz = get_carrier_frequency_hz(frq.p[0], sv_type);

        DBG("Update measurement: prn=%d typ=%d cfrq=%f", prn, sv_type, r->gnss_data.measurements[n].carrier_frequency_hz);

        r->gnss_data.measurement_count++;
        return 0;
}


static int
nmea_reader_update_sv_status_gnss(NmeaReader* r, int sv_type, int prn, Token elevation, Token azimuth, Token snr)
{
        int sv_index = r->sv_status_gnss.num_svs;
        if (GNSS_MAX_SVS <= sv_index) {
                ERR("ERR: sv_index=[%d] is larger than GNSS_MAX_SVS.\n", sv_index);
                return 1;
        }
	if (sv_type == GNSS_CONSTELLATION_GLONASS && prn > SVID_PLUS_GLONASS)
		prn -= SVID_PLUS_GLONASS;
        r->sv_status_gnss.gnss_sv_list[sv_index].svid = prn;
        r->sv_status_gnss.gnss_sv_list[sv_index].constellation = sv_type;

        r->sv_status_gnss.gnss_sv_list[sv_index].c_n0_dbhz = str2float(snr.p, snr.end);
        r->sv_status_gnss.gnss_sv_list[sv_index].elevation = str2int(elevation.p, elevation.end);
        r->sv_status_gnss.gnss_sv_list[sv_index].azimuth = str2int(azimuth.p, azimuth.end);
        r->sv_status_gnss.gnss_sv_list[sv_index].flags = 0;
        if (r->sv_used_in_fix[prn2svid(prn, sv_type)])
                r->sv_status_gnss.gnss_sv_list[sv_index].flags |= GNSS_SV_FLAGS_USED_IN_FIX;

        DBG("Update SV: prn=%d typ=%d use=%d\n", prn, sv_type, r->sv_used_in_fix[prn2svid(prn, sv_type)]); 

        //r->sv_count++;
        r->sv_status_gnss.num_svs++;
        return 0;
 }

static void
nmea_reader_parse(NmeaReader* const r)
{
        /* we received a complete sentence, now parse it to generate
         * a new GPS fix...
         */
        NmeaTokenizer  tzer[1];
        Token          tok;
        GnssConstellationType sv_type = GNSS_CONSTELLATION_GPS;


#if NMEA_DEBUG
        DBG("Received: '%.*s'", r->pos, r->in);
#endif
        if (r->pos < 9) {
                ERR("Too short. discarded. '%.*s'", r->pos, r->in);
                return;
        }
        if (r->pos < (int)sizeof(r->in)) {
                nmea_tokenizer_init(tzer, r->in, r->in + r->pos);
        }
#if NMEA_DEBUG
        {
                int  n;
                DBG("Found %d tokens", tzer->count);
                for (n = 0; n < tzer->count; n++) {
                        Token  tok = nmea_tokenizer_get(tzer, n);
                        DBG("%2d: '%.*s'", n, (int)(tok.end-tok.p), tok.p);
                }
        }
#endif

        tok = nmea_tokenizer_get(tzer, 0);
        if (tok.p + 5 > tok.end) {
                ERR("sentence id '%.*s' too short, ignored.", (int)(tok.end-tok.p), tok.p);
                return;
        }

        //  ignore first two characters.
        if (!memcmp(tok.p, "BD", 2)) {
                sv_type = GNSS_CONSTELLATION_BEIDOU;
                DBG("BDS SV type");
        } else if (!memcmp(tok.p, "GL", 2)) {
                sv_type = GNSS_CONSTELLATION_GLONASS;
                DBG("GLN SV type");
        } else if (!memcmp(tok.p, "GA", 2)) {
                sv_type = GNSS_CONSTELLATION_GALILEO;
                DBG("GAL SV type");
        }

        tok.p += 2;
        if (!memcmp(tok.p, "GGA", 3)) {
                //  GPS fix
                Token  tok_time          = nmea_tokenizer_get(tzer, 1);
                Token  tok_latitude      = nmea_tokenizer_get(tzer, 2);
                Token  tok_latitudeHemi  = nmea_tokenizer_get(tzer, 3);
                Token  tok_longitude     = nmea_tokenizer_get(tzer, 4);
                Token  tok_longitudeHemi = nmea_tokenizer_get(tzer, 5);
		Token  tok_hdop 	 = nmea_tokenizer_get(tzer, 8); // hdop
                Token  tok_altitude      = nmea_tokenizer_get(tzer, 9);
                Token  tok_altitudeUnits = nmea_tokenizer_get(tzer, 10);

                nmea_reader_update_time(r, tok_time);
                nmea_reader_update_latlong(r, tok_latitude,
                                           tok_latitudeHemi.p[0],
                                           tok_longitude,
                                           tok_longitudeHemi.p[0]);
                nmea_reader_update_altitude(r, tok_altitude, tok_altitudeUnits);
		r->hdop = str2float(tok_hdop.p, tok_hdop.end);
		// nmea_reader_update_accuracy(r, tok_accuracy);

        }
        else if (!memcmp(tok.p, "GSA", 3)) {
                Token tok_fix = nmea_tokenizer_get(tzer, 2);
                Token tok_svs = nmea_tokenizer_get(tzer, 18);
                switch(tok_svs.p[0]) {
                case '1':
                        sv_type = GNSS_CONSTELLATION_GPS;
                        break;
                case '2':
                        sv_type = GNSS_CONSTELLATION_GLONASS;
                        break;
                case '3':
                        sv_type = GNSS_CONSTELLATION_GALILEO;
                        break;
                case '4':
                        sv_type = GNSS_CONSTELLATION_BEIDOU;
                        break;
                default:
                        break;
                }
                int idx, max = 12;  /*the number of satellites in GPGSA*/

                r->fix_mode = str2int(tok_fix.p, tok_fix.end);

                if (LOC_FIXED(r)) {  /* 1: No fix; 2: 2D; 3: 3D*/

                        for (idx = 0; idx < max; idx++) {
                                Token tok_satellite = nmea_tokenizer_get(tzer, idx+3);
                                if (tok_satellite.p == tok_satellite.end) {
                                        DBG("GSA: found %d active satellites\n", idx);
                                        break;
                                }
                                int prn = str2int(tok_satellite.p, tok_satellite.end);
                                int svid = prn2svid(prn, sv_type);
                                if (svid >= 0 && svid < MAX_GNSS_SVID)
                                        r->sv_used_in_fix[svid] = 1;
                                //        r->svlst[svid].flags |= GNSS_SV_FLAGS_USED_IN_FIX;
                        }
                }
        }
        else if (!memcmp(tok.p, "RMC", 3)) {
                Token  tok_time          = nmea_tokenizer_get(tzer, 1);
                Token  tok_fixStatus     = nmea_tokenizer_get(tzer, 2);
                Token  tok_latitude      = nmea_tokenizer_get(tzer, 3);
                Token  tok_latitudeHemi  = nmea_tokenizer_get(tzer, 4);
                Token  tok_longitude     = nmea_tokenizer_get(tzer, 5);
                Token  tok_longitudeHemi = nmea_tokenizer_get(tzer, 6);
                Token  tok_speed         = nmea_tokenizer_get(tzer, 7);
                Token  tok_bearing       = nmea_tokenizer_get(tzer, 8);
                Token  tok_date          = nmea_tokenizer_get(tzer, 9);

                VER("in RMC, fixStatus=%c", tok_fixStatus.p[0]);
                if (tok_fixStatus.p[0] == 'A')
                {
                        nmea_reader_update_date(r, tok_date, tok_time);

                        nmea_reader_update_latlong(r, tok_latitude,
                                                   tok_latitudeHemi.p[0],
                                                   tok_longitude,
                                                   tok_longitudeHemi.p[0]);

                        nmea_reader_update_bearing(r, tok_bearing);
                        nmea_reader_update_speed(r, tok_speed);
			nmea_reader_update_accuracy(r, r->hdop);
                        r->location_can_report = 1;
                }
                r->sv_status_can_report = 1;
        } else if (!memcmp(tok.p, "GSV", 3)) {
                Token tok_num = nmea_tokenizer_get(tzer, 1);    // number of messages
                Token tok_seq = nmea_tokenizer_get(tzer, 2);    // sequence number
                Token tok_cnt = nmea_tokenizer_get(tzer, 3);    // Satellites in view
                int num = str2int(tok_num.p, tok_num.end);
                int seq = str2int(tok_seq.p, tok_seq.end);
                int cnt = str2int(tok_cnt.p, tok_cnt.end);
                int sv_base = (seq - 1) * NMEA_MAX_SV_INFO;
                int sv_num = cnt - sv_base;
                int idx, base = 4, base_idx;

                if (sv_num > NMEA_MAX_SV_INFO)
                        sv_num = NMEA_MAX_SV_INFO;

                Token tok_frq = nmea_tokenizer_get(tzer, base + sv_num * 4);

                /*
                if (seq == 1)  
                        r->sv_count = 0;
                */

                for (idx = 0; idx < sv_num; idx++) {
                        base_idx = base*(idx+1);
                        Token tok_id  = nmea_tokenizer_get(tzer, base_idx+0);
                        int prn = str2int(tok_id.p, tok_id.end);
                        //int svid = prn2svid(prn, sv_type);

                        Token tok_ele = nmea_tokenizer_get(tzer, base_idx+1);
                        Token tok_azi = nmea_tokenizer_get(tzer, base_idx+2);
                        Token tok_snr = nmea_tokenizer_get(tzer, base_idx+3);
                        /*
                        sv_arr[idx].c_n0_dbhz = str2int(tok_snr.p, tok_snr.end);
                        sv_arr[idx].elevation = str2int(tok_ele.p, tok_ele.end);
                        sv_arr[idx].azimuth = str2int(tok_azi.p, tok_azi.end);
                        sv_arr[idx].svid = prn;
                        sv_arr[idx].constellation = sv_type;
                        */
                        nmea_reader_update_sv_status_gnss(r, sv_type, prn, tok_ele, tok_azi, tok_snr);
                        if (HAS_CARRIER_FREQUENCY(tok_frq.p[0]))
                                nmea_reader_update_gnss_measurement(r, sv_type, prn, tok_frq);
                }
                if (seq == num) {
                /*
                        if (r->sv_count <= cnt) {
                                DBG("r->sv_count = %d", r->sv_count);
                                r->sv_status_gnss.num_svs += r->sv_count;


                        } else {
                                ERR("GPGSV incomplete (%d/%d), ignored!", r->sv_count, cnt);
                                r->sv_count = r->sv_status_gnss.num_svs = 0;
                        }
                */
                }
        }
        // Add for Accuracy
	/* else if (!memcmp(tok.p, "ACCURACY", 8)) {
                if ((r->fix_mode == 3) || (r->fix_mode == 2)) {
                        Token  tok_accuracy = nmea_tokenizer_get(tzer, 1);
                        nmea_reader_update_accuracy(r, tok_accuracy);
                        DBG("GPS get accuracy from driver:%f\n", r->fix.accuracy);
                }
                else {
                        DBG("GPS get accuracy failed, fix mode:%d\n", r->fix_mode);
                }
        }*/
        else {
                tok.p -= 2;
                VER("unknown sentence '%.*s'", (int)(tok.end-tok.p), tok.p);
        }
        //if (!LOC_FIXED(r)) {
        //    VER("Location is not fixed, ignored callback\n");
        //} else if (r->fix.flags != 0 && gps_nmea_end_tag) {
        if (r->location_can_report) {
#if NMEA_DEBUG
                char   temp[256];
                char*  p   = temp;
                char*  end = p + sizeof(temp);
                struct tm   utc;

                p += snprintf(p, end-p, "sending fix");
                if (r->fix.flags & GPS_LOCATION_HAS_LAT_LONG) {
                        p += snprintf(p, end-p, " lat=%g lon=%g", r->fix.latitude, r->fix.longitude);
                }
                if (r->fix.flags & GPS_LOCATION_HAS_ALTITUDE) {
                        p += snprintf(p, end-p, " altitude=%g", r->fix.altitude);
                }
                if (r->fix.flags & GPS_LOCATION_HAS_SPEED) {
                        p += snprintf(p, end-p, " speed=%g", r->fix.speed);
                }
                if (r->fix.flags & GPS_LOCATION_HAS_BEARING) {
                        p += snprintf(p, end-p, " bearing=%g", r->fix.bearing);
                }
                if (r->fix.flags & GPS_LOCATION_HAS_ACCURACY) {
                        p += snprintf(p, end-p, " accuracy=%g", r->fix.accuracy);
                        DBG("GPS accuracy=%g\n", r->fix.accuracy);
                }
                gmtime_r((time_t*) &r->fix.timestamp, &utc);
                p += snprintf(p, end-p, " time=%s", asctime(&utc));
                VER("%s", temp);
#endif

                callback_backup.location_cb(&r->fix);
                r->fix.flags = 0;
                r->location_can_report = 0;
        }

        DBG("gnss_num = %d, gps_nmea_end_tag = %d, sv_status_can_report = %d", 
                r->sv_status_gnss.num_svs, gps_nmea_end_tag, r->sv_status_can_report);
        if (r->sv_status_can_report) {
                if (r->sv_status_gnss.num_svs > 0) {
                	DBG("Report sv status, num = %d, cb=%p.", r->sv_status_gnss.num_svs, callback_backup.gnss_sv_status_cb);
                        r->sv_status_gnss.size = sizeof(GnssSvStatus);
                        callback_backup.gnss_sv_status_cb(&r->sv_status_gnss);
                        //r->sv_count = r->sv_status_gnss.num_svs = 0;
                        memset(r->sv_used_in_fix, 0, sizeof(r->sv_used_in_fix));
                        memset(&r->sv_status_gnss, 0, sizeof(r->sv_status_gnss));
                }
		
                DBG("Report gnss measurements, num = %d, measure_on = %d", 
		(int)r->gnss_data.measurement_count, is_measurement);
                if (r->gnss_data.measurement_count > 0) {
                        r->gnss_data.size = sizeof(GnssData);
                        if (is_measurement)
                                measurement_callbacks.gnss_measurement_callback(&r->gnss_data);
                        memset(&r->gnss_data, 0, sizeof(r->gnss_data));
                }
                /*
                DBG("sv list --> sv status gnss");
                int sv_count = 0;
                for (int i = 0; i < MAX_GNSS_SVID; i++) {
                        if (r->svlst[i].svid == 0)
                                continue;
                        if (sv_count >=0 && sv_count < GNSS_MAX_SVS)
                                r->sv_status_gnss.gnss_sv_list[sv_count++] = r->svlst[i];
                }
                r->sv_status_gnss.num_svs = sv_count;
                r->sv_status_gnss.size = sizeof(GnssSvStatus);

                DBG("report %d sv status", sv_count);
                if (r->sv_status_gnss.num_svs > 0)
                        callback_backup.gnss_sv_status_cb(&r->sv_status_gnss);

                memset(r->svlst, 0, sizeof(r->svlst));
                */
                r->sv_status_can_report = 0;
        }
}

static void
reduce_sv_freq()
{
	// Set GSA and GSV outputs once per 2 seconds
	char msg[] = "$PCAS03,,,2,2,,,,,,,,,,*02\r\n";

	write(_gps_state->fd, msg, strlen(msg));
	DBG("Reduce GSA and GSV outputs freq.");
}


static void
auto_reduce_sv_freq2(const NmeaReader *r)
{
        const int MAX_NBYTES = 960 * 2 * 0.85;

        static int nbytes = 0;
        static int rmc_counter = 0;

        if (REDUCE_SV_FREQ != 2) {
                //DBG("Reduce sv freq: OFF.");
                return;
        }

        if (TTY_BAUD > B9600) {
                DBG("High speed baudrate, no need to reduce sv freq.");
                return;
        }

        nbytes += r->pos;
        DBG("TTY payload rate=%d/%d.", nbytes, MAX_NBYTES);
        if (nbytes > MAX_NBYTES) 
		reduce_sv_freq();

        // reset nbytes to 0 every 2 rmc
        if (memcmp(r->in + 3, "RMC", 3) == 0) 
                if (++rmc_counter % 2 == 0)
                        nbytes = 0;
}

static void
nmea_reader_addc(NmeaReader* const r, int  c)
{
        if (r->overflow) {
                r->overflow = (c != '\n');
                return;
        }

        if ((r->pos >= (int) sizeof(r->in)-1 ) || (r->pos < 0)) {
                r->overflow = 1;
                r->pos      = 0;
                DBG("nmea sentence overflow\n");
                return;
        }

        r->in[r->pos] = (char)c;
        r->pos       += 1;

        if (c == '\n') {
		auto_reduce_sv_freq2(r);
                nmea_reader_parse(r);

                DBG("start nmea_cb\n");
		r->in[r->pos] = 0;
                callback_backup.nmea_cb(r->fix.timestamp, r->in, r->pos);
                r->pos = 0;
        }
}


static void
gps_state_done(GpsState*  s)
{
        char   cmd = CMD_QUIT;
	void *dummy;

        write(s->control[0], &cmd, 1);
	pthread_join(s->thread, &dummy);


        close(s->control[0]);
        s->control[0] = -1;
        close(s->control[1]);
        s->control[1] = -1;
        close(s->fd);
        s->fd = -1;
	// close(s->fake_fd);
        // s->fake_fd = -1;
        close(s->epoll_fd);
        s->epoll_fd = -1;
        s->init = 0;
        return;
}

static void
try_boost_gnsstty(int tty_fd, int baud)
{
	const char msg[] = "$PCAS01,5*19\r\n";

	if (TTY_BOOST == 0) {
		DBG("TTY Boost OFF.");
		return;
	}

	if (baud > B9600) {
		DBG("No need to boost baudrate.");
		return;
	}

        struct termios cfg;
        tcgetattr(tty_fd, &cfg);
        cfmakeraw(&cfg);
        cfsetispeed(&cfg, baud);
        cfsetospeed(&cfg, baud);
        tcsetattr(tty_fd, TCSANOW, &cfg);

	write(tty_fd, msg, strlen(msg));

	usleep(200 * 1000);	// sleep 200ms to make sure msg is send at TTY_BAUD

	// Upgrade tty's baudrate to 115200
        cfsetispeed(&cfg, B115200);
        cfsetospeed(&cfg, B115200);
        tcsetattr(tty_fd, TCSANOW, &cfg);

	DBG("Boost baudrate to 115200.");
}

static void
gps_state_start(GpsState*  s)
{
        char  cmd = CMD_START;
        int   ret;

        do {
                ret = write(s->control[0], &cmd, 1);
        }
        while (ret < 0 && errno == EINTR);

        if (ret != 1)
                ERR("%s: could not send CMD_START command: ret=%d: %s",
                    __FUNCTION__, ret, strerror(errno));

	// Try Boost baudrate
	try_boost_gnsstty(s->fd, TTY_BAUD);

	if (REDUCE_SV_FREQ == 1)
		reduce_sv_freq();
}

static void
gps_state_stop(GpsState*  s)
{
        char  cmd = CMD_STOP;
        int   ret;

        do {
                ret = write(s->control[0], &cmd, 1);
        }
        while (ret < 0 && errno == EINTR);

        if (ret != 1)
                ERR("%s: could not send CMD_STOP command: ret=%d: %s",
                    __FUNCTION__, ret, strerror(errno));
}
/*
static void
gps_state_restart(GpsState*  s)
{
        char  cmd = CMD_RESTART;
        int   ret;

        do {
                ret = write(s->control[0], &cmd, 1);
        }
        while (ret < 0 && errno == EINTR);

        if (ret != 1)
                ERR("%s: could not send CMD_RESTART command: ret=%d: %s",
                    __FUNCTION__, ret, strerror(errno));
}
*/

static int
epoll_register(int  epoll_fd, int  fd)
{
        struct epoll_event  ev;
        int                 ret, flags;

        // important: make the fd non-blocking 
        flags = fcntl(fd, F_GETFL);
        fcntl(fd, F_SETFL, flags | O_NONBLOCK);

        ev.events  = EPOLLIN;
        ev.data.fd = fd;
        do {
                ret = epoll_ctl(epoll_fd, EPOLL_CTL_ADD, fd, &ev);
        } while (ret < 0 && errno == EINTR);
        if (ret < 0)
                ERR("epoll ctl error, fd = %d, error num is %d, message is %s\n", fd, errno, strerror(errno));
        return ret;
 }

/*
static int
epoll_deregister(int  epoll_fd, int  fd)
{
        int  ret;
        do {
                ret = epoll_ctl(epoll_fd, EPOLL_CTL_DEL, fd, NULL);
        } while (ret < 0 && errno == EINTR);
        return ret;
}
*/

/*for reducing the function call to get data from kernel*/
static char buff[2048];
/* this is the main thread, it waits for commands from gps_state_start/stop and,
 * when started, messages from the GPS daemon. these are simple NMEA sentences
 * that must be parsed to be converted into GPS fixes sent to the framework
 */
void
gps_state_thread(void*  arg)
{
        static float count = 0;
        GpsState*   state = (GpsState*) arg;
        //   state->thread_exit_flag=0;
        NmeaReader  reader[1];
        int         gps_fd     = state->fd;
        int         control_fd = state->control[1];

        int epoll_fd = state->epoll_fd;

        nmea_reader_init(reader);

        if (control_fd < 0 || gps_fd < 0) {
                ERR("control_fd = %d, gps_fd = %d", control_fd, gps_fd);
                return;
        }

        //  register control file descriptors for polling
        if (epoll_register(epoll_fd, control_fd) < 0) {
                ERR("epoll register control_fd error, error num is %d, message is %s\n", errno, strerror(errno));
		return;
	}

        if (epoll_register(epoll_fd, gps_fd) < 0) {
                ERR("epoll register gps_fd error, error num is %d, message is %s\n", errno, strerror(errno));
		return;
	}

        DBG("gps thread(v%d.%d) running: PPID[%d], PID[%d], EPOLL_FD[%d], GPSFD[%d]\n", MAJOR_NO, MINOR_NO, getppid(), getpid(), epoll_fd, gps_fd);
        DBG("HAL thread is ready, realease lock, and CMD_START can be handled\n");
        //  now loop
        for (;;) {
                struct epoll_event   events[4];
                int                  ne, nevents;
		// DBG("Call epoll wait, epoll_fd=%d", epoll_fd);
                nevents = epoll_wait(epoll_fd, events, 2, -1);
                if (nevents < 0) {
                        if (errno != EINTR) {
                                ERR("epoll_wait() unexpected error: %s.", strerror(errno));
				if (errno == EINVAL)
					goto Exit;
			}
                        continue;
                }
                VER("gps thread received %d events", nevents);
                for (ne = 0; ne < nevents; ne++) {
                        if ((events[ne].events & (EPOLLERR|EPOLLHUP)) != 0) {
                                ERR("EPOLLERR or EPOLLHUP after epoll_wait() !?");
                                goto Exit;
                        }
                        if ((events[ne].events & EPOLLIN) != 0) {
                                int  fd = events[ne].data.fd;

                                if (fd == control_fd) {
                                        char  cmd = 255;
                                        int   ret;
                                        DBG("gps control fd event");
                                        do {
                                                ret = read(fd, &cmd, 1);
                                        } while (ret < 0 && errno == EINTR);

                                        if (cmd == CMD_QUIT) {
                                                DBG("gps thread quitting on demand");
                                                goto Exit;
                                        }
                                        else if (cmd == CMD_START) {
                                                if (!started) {
                                                        DBG("gps thread starting  location_cb=%p", &callback_backup);
                                                        started = 1;
                                                        nmea_reader_set_callback(reader, &state->callbacks);
                                                }
                                        }
                                        else if (cmd == CMD_STOP) {
                                                if (started) {
                                                        DBG("gps thread stopping");
                                                        started = 0;
                                                        nmea_reader_set_callback(reader, NULL);
                                                        DBG("CMD_STOP has been receiving from HAL thread, release lock so can handle CLEAN_UP\n");
                                                }
                                        }
                                        else if (cmd == CMD_RESTART) {
                                                reader->fix_mode = 0;
                                        }
                                }
                                else if (fd == gps_fd) {
                                        if (!flag_unlock) {
                                                flag_unlock = 1;
                                                DBG("got first NMEA sentence, release lock to set state ENGINE ON, SESSION BEGIN");
                                        }
                                        VER("gps fd event");
                                        if (report_time_interval > ++count) {
                                                DBG("[trace]count is %f\n", count);
                                                read(fd, buff, sizeof(buff));
                                                continue;
                                        }
                                        count = 0;
                                        for (;;) {
                                                int  nn, ret;
                                                ret = read(fd, buff, sizeof(buff));
                                                if (ret < 0) {
                                                        if (errno == EINTR)
                                                                continue;
                                                        if (errno != EWOULDBLOCK)
                                                                ERR("error while reading from gps daemon socket: %s: %p", strerror(errno), buff);
                                                        break;
                                                }
                                                //DBG("received %d bytes.\n", ret);
						DBG("gps fd received: %.*s bytes: %d\n", ret, buff, ret);
                                                gps_nmea_end_tag = 0;
                                                for (nn = 0; nn < ret; nn++)
                                                {
                                                        if (nn == (ret-1))
                                                                gps_nmea_end_tag = 1;

                                                        nmea_reader_addc(reader, buff[nn]);
                                                }
                                        }
                                        VER("gps fd event end");
                                }
                                else
                                {
                                        ERR("epoll_wait() returned unkown fd %d ?", fd);
                                }
                        }
                }
        }
Exit:
        DBG("HAL thread is exiting, release lock to clean resources\n");
        return;
}


static void
gps_state_init(GpsState*  state)
{
        state->control[0] = -1;
        state->control[1] = -1;
        state->fd         = -1;
	state->epoll_fd   = -1;


        DBG("Try open gps hardware:  %s", GPS_CHANNEL_NAME);
        //state->fd = open(GPS_CHANNEL_NAME, O_RDONLY | O_NONBLOCK | O_NOCTTY);
        state->fd = open(GPS_CHANNEL_NAME, O_RDWR | O_NONBLOCK | O_NOCTTY);

        if (state->fd < 0) {
                ERR("no gps hardware detected: %s:%d, %s", GPS_CHANNEL_NAME, state->fd, strerror(errno));
                return;
        }

        struct termios cfg;
        tcgetattr(state->fd, &cfg);
        cfmakeraw(&cfg);
        cfsetispeed(&cfg, TTY_BAUD);
        cfsetospeed(&cfg, TTY_BAUD);
        tcsetattr(state->fd, TCSANOW, &cfg);

        DBG("Open gps hardware succeed: %s, gps_fd=%d", GPS_CHANNEL_NAME, state->fd);

        //int epoll_fd   = epoll_create(2);
        int epoll_fd   = epoll_create1(0);
        state->epoll_fd = epoll_fd;
	DBG("Create epoll fd: %d, %s", epoll_fd, strerror(errno));

        if (socketpair(AF_LOCAL, SOCK_STREAM, 0, state->control) < 0) {
                ERR("could not create thread control socket pair: %s", strerror(errno));
                goto Fail;
        }
        state->thread = callback_backup.create_thread_cb(gps_native_thread, gps_state_thread, state);
        if (!state->thread) {
                ERR("could not create gps thread: %s", strerror(errno));
                goto Fail;
        }

        DBG("gps state initialized, the thread is %d", (int)state->thread);
	DBG("TTY_BOOST=%d, REDUCE_SV_FREQ=%d.", TTY_BAUD, REDUCE_SV_FREQ);

        return;

Fail:
        gps_state_done(state);
}


/*****************************************************************/
/*****************************************************************/
/*****                                                       *****/
/*****       I N T E R F A C E                               *****/
/*****                                                       *****/
/*****************************************************************/
/*****************************************************************/
static int
zkw_gps_init(GpsCallbacks* callbacks)
{
        GpsState*  s = _gps_state;
        if (s->init)
                return 0;

        if ( callbacks != NULL && callbacks->size == sizeof(GpsCallbacks)) {
                s->callbacks = *callbacks;
                callback_backup = *callbacks;
                gps_state_init(s);
        } else {
                ERR("Invalid callbacks.");
        }

        if (s->fd < 0) {
                return -1;
        }
        DBG("Set GPS_CAPABILITY_SCHEDULING \n");
        callback_backup.set_capabilities_cb(GPS_CAPABILITY_SCHEDULING);
        s->init = 1;
        return 0;
}

static void
zkw_gps_cleanup(void)
{
        GpsState*  s = _gps_state;

        char cmd = CMD_STOP;
        int ret;

        do {
                ret = write(s->control[0], &cmd, 1);
        } while(ret < 0 && errno == EINTR);

        /*
        if (s->init)
                gps_state_done(s);
        */
        DBG("zkw_gps_cleanup done");
        //     return NULL;
}

int
zkw_gps_start()
{
        GpsState*  s = _gps_state;

        if (!s->init) {
                ERR("%s: called with uninitialized state !!", __FUNCTION__);
                return -1;
        }

        DBG("HAL thread has initialiazed\n");
        gps_state_start(s);

        gps_status.status = GPS_STATUS_ENGINE_ON;
        DBG("gps_status = GPS_STATUS_ENGINE_ON\n");
        callback_backup.status_cb(&gps_status);
        gps_status.status = GPS_STATUS_SESSION_BEGIN;
        DBG("gps_status = GPS_STATUS_SESSION_BEGIN\n");
        callback_backup.status_cb(&gps_status);
        callback_backup.acquire_wakelock_cb();
        s->start_flag = 1;
        DBG("s->start_flag = 1\n");
        return 0;
}
int
zkw_gps_stop()
{
        GpsState*  s = _gps_state;

        if (!s->init) {
                ERR("%s: called with uninitialized state !!", __FUNCTION__);
                return -1;
        }

        gps_state_stop(s);

        gps_status.status = GPS_STATUS_SESSION_END;
        callback_backup.status_cb(&gps_status);
        DBG("gps_status = GPS_STATUS_SESSION_END\n");
        gps_status.status = GPS_STATUS_ENGINE_OFF;
        DBG("gps_status = GPS_STATUS_ENGINE_OFF\n");
        callback_backup.status_cb(&gps_status);
        callback_backup.release_wakelock_cb();
        s->start_flag = 0;
        DBG("s->start_flag = 0\n");
        return 0;
}
static int
zkw_gps_inject_time(GpsUtcTime time, int64_t timeReference, int uncertainty)
{
	UNUSED(time);
	UNUSED(timeReference);
	UNUSED(uncertainty);
        return 0;
}

static int
zkw_gps_inject_location(double latitude, double longitude, float accuracy)
{
	UNUSED(latitude);
	UNUSED(longitude);
	UNUSED(accuracy);
        return 0;
}

static void
zkw_gps_delete_aiding_data(GpsAidingData flags)
{
	UNUSED(flags);
        return;
}

static int
zkw_gps_set_position_mode(GpsPositionMode mode, GpsPositionRecurrence recurrence, uint32_t min_interval, uint32_t preferred_accuracy, uint32_t preferred_time)
{
        // FIXME - support fix_frequency
	UNUSED(mode);
	UNUSED(recurrence);
	UNUSED(min_interval);
	UNUSED(preferred_accuracy);
	UNUSED(preferred_time);

        return 0;
}

static int
zkw_gps_measurement_init(GpsMeasurementCallbacks *callbacks)
{
        measurement_callbacks = *callbacks;
        is_measurement = 1;
        DBG("GPS measurement init.");
        return 0;
}

static void
zkw_gps_measurement_close()
{
        is_measurement = 0;
        DBG("GPS measurement closed.");
}

static const GpsMeasurementInterface zkwGpsMeasurementInterface = {
        sizeof(GpsMeasurementInterface),
        zkw_gps_measurement_init,
        zkw_gps_measurement_close,
};

static const void*
zkw_gps_get_extension(const char* name)
{
        DBG("zkw_gps_get_extension name=[%s]\n", name);
        /*
            TRC();
            if (strncmp(name, "agps", strlen(name)) == 0) {
                return &zkwAGpsInterface;
            }
            if (strncmp(name, "gps-ni", strlen(name)) == 0) {
                return &zkwGpsNiInterface;
            }
            if (strncmp(name, "agps_ril", strlen(name)) == 0) {
                return &zkwAGpsRilInterface;
            }
            if (strncmp(name, "supl-certificate", strlen(name)) == 0) {
               return &zkwSuplCertificateInterface;
            }
            if (strncmp(name, GPS_NAVIGATION_MESSAGE_INTERFACE, strlen(name)) == 0) {
               return &zkwGpsNavigationMessageInterface;
            }
        */
        if (MEASUREMENT_SUPPLY && strncmp(name, GPS_MEASUREMENT_INTERFACE, strlen(name)) == 0)
                return &zkwGpsMeasurementInterface;
        return NULL;
}

static const GpsInterface  zkwGpsInterface = {
        sizeof(GpsInterface),
        zkw_gps_init,
        zkw_gps_start,
        zkw_gps_stop,
        zkw_gps_cleanup,
        zkw_gps_inject_time,
        zkw_gps_inject_location,
        zkw_gps_delete_aiding_data,
        zkw_gps_set_position_mode,
        zkw_gps_get_extension,
};


const GpsInterface* gps__get_gps_interface(struct gps_device_t* dev)
{
        DBG("gps__get_gps_interface HAL\n");
	UNUSED(dev);

        return &zkwGpsInterface;
}

static int open_gps(const struct hw_module_t* module, char const* name,
                    struct hw_device_t** device) {
        DBG("open_gps HAL 1: name=%s ver=%d.%d.\n", name, MAJOR_NO, MINOR_NO);
        struct gps_device_t *dev = malloc(sizeof(struct gps_device_t));
        if (dev != NULL) {
                memset(dev, 0, sizeof(*dev));

                dev->common.tag = HARDWARE_DEVICE_TAG;
                dev->common.version = 0;
                dev->common.module = (struct hw_module_t*)module;
                //   dev->common.close = (int (*)(struct hw_device_t*))close_lights;
                DBG("open_gps HAL 2\n");
                dev->get_gps_interface = gps__get_gps_interface;
                DBG("open_gps HAL 3\n");
                *device = (struct hw_device_t*)dev;
        } else {
                DBG("malloc failed dev = NULL!\n");
        }
        return 0;
}


static struct hw_module_methods_t gps_module_methods = {
        .open = open_gps
};


struct hw_module_t HAL_MODULE_INFO_SYM = {
        .tag = HARDWARE_MODULE_TAG,
        .version_major = MAJOR_NO,
        .version_minor = MINOR_NO,
        .id = GPS_HARDWARE_MODULE_ID,
        .name = "Hardware GPS Module",
        .author = "",
        .methods = &gps_module_methods,
};
