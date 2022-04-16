#include "nrf_gps.h"

#include "minmea.h"

#define NMEA_MAX_DATA_LEN MINMEA_MAX_LENGTH + 3

static void parse_nmea_sentence(char* line)
{
    switch (minmea_sentence_id(line, false)) {
        case MINMEA_SENTENCE_RMC: {
            struct minmea_sentence_rmc frame;
            if (minmea_parse_rmc(&frame, line)) {
                Debug("$xxRMC: raw coordinates and speed: (%d/%d,%d/%d) %d/%d\n",
                        frame.latitude.value, frame.latitude.scale,
                        frame.longitude.value, frame.longitude.scale,
                        frame.speed.value, frame.speed.scale);
                Debug("$xxRMC fixed-point coordinates and speed scaled to three decimal places: (%d,%d) %d\n",
                        minmea_rescale(&frame.latitude, 1000),
                        minmea_rescale(&frame.longitude, 1000),
                        minmea_rescale(&frame.speed, 1000));
                Debug("$xxRMC floating point degree coordinates and speed: (%f,%f) %f\n",
                        minmea_tocoord(&frame.latitude),
                        minmea_tocoord(&frame.longitude),
                        minmea_tofloat(&frame.speed));
            }
            else {
                Debug("$xxRMC sentence is not parsed\n");
            }
        } break;

        case MINMEA_SENTENCE_GGA: {
            struct minmea_sentence_gga frame;
            if (minmea_parse_gga(&frame, line)) {
                Debug("$xxGGA: fix quality: %d\n", frame.fix_quality);
            }
            else {
                Debug("$xxGGA sentence is not parsed\n");
            }
        } break;

        case MINMEA_SENTENCE_GST: {
            struct minmea_sentence_gst frame;
            if (minmea_parse_gst(&frame, line)) {
                Debug("$xxGST: raw latitude,longitude and altitude error deviation: (%d/%d,%d/%d,%d/%d)\n",
                        frame.latitude_error_deviation.value, frame.latitude_error_deviation.scale,
                        frame.longitude_error_deviation.value, frame.longitude_error_deviation.scale,
                        frame.altitude_error_deviation.value, frame.altitude_error_deviation.scale);
                Debug("$xxGST fixed point latitude,longitude and altitude error deviation"
                       " scaled to one decimal place: (%d,%d,%d)\n",
                        minmea_rescale(&frame.latitude_error_deviation, 10),
                        minmea_rescale(&frame.longitude_error_deviation, 10),
                        minmea_rescale(&frame.altitude_error_deviation, 10));
                Debug("$xxGST floating point degree latitude, longitude and altitude error deviation: (%f,%f,%f)",
                        minmea_tofloat(&frame.latitude_error_deviation),
                        minmea_tofloat(&frame.longitude_error_deviation),
                        minmea_tofloat(&frame.altitude_error_deviation));
            }
            else {
                Debug("$xxGST sentence is not parsed\n");
            }
        } break;

        case MINMEA_SENTENCE_GSV: {
            struct minmea_sentence_gsv frame;
            if (minmea_parse_gsv(&frame, line)) {
                Debug("$xxGSV: message %d of %d\n", frame.msg_nr, frame.total_msgs);
                Debug("$xxGSV: sattelites in view: %d\n", frame.total_sats);
                for (int i = 0; i < 4; i++)
                    Debug("$xxGSV: sat nr %d, elevation: %d, azimuth: %d, snr: %d dbm\n",
                        frame.sats[i].nr,
                        frame.sats[i].elevation,
                        frame.sats[i].azimuth,
                        frame.sats[i].snr);
            }
            else {
                Debug("$xxGSV sentence is not parsed\n");
            }
        } break;

        case MINMEA_SENTENCE_VTG: {
           struct minmea_sentence_vtg frame;
           if (minmea_parse_vtg(&frame, line)) {
                Debug("$xxVTG: true track degrees = %f\n",
                       minmea_tofloat(&frame.true_track_degrees));
                Debug("        magnetic track degrees = %f\n",
                       minmea_tofloat(&frame.magnetic_track_degrees));
                Debug("        speed knots = %f\n",
                        minmea_tofloat(&frame.speed_knots));
                Debug("        speed kph = %f\n",
                        minmea_tofloat(&frame.speed_kph));
           }
           else {
                Debug("$xxVTG sentence is not parsed\n");
           }
        } break;

        case MINMEA_SENTENCE_ZDA: {
            struct minmea_sentence_zda frame;
            if (minmea_parse_zda(&frame, line)) {
//                Debug("$xxZDA: %d:%d:%d %02d.%02d.%d UTC%+03d:%02d\n",
//                       frame.time.hours,
//                       frame.time.minutes,
//                       frame.time.seconds,
//                       frame.date.day,
//                       frame.date.month,
//                       frame.date.year,
//                       frame.hour_offset,
//                       frame.minute_offset);
                Debug("$xxZDA: %d:%d:%d",frame.time.hours, frame.time.minutes, frame.time.seconds);
            }
            else {
                Debug("$xxZDA sentence is not parsed\n");
            }
        } break;

        case MINMEA_INVALID: {
            Debug("$xxxxx sentence is not valid\n");
        } break;

        default: {
            Debug("$xxxxx sentence is not parsed\n");
        } break;
    }

}

void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[NMEA_MAX_DATA_LEN];
    static uint16_t index = 0;
    uint32_t       err_code;

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));

            if ((data_array[index] == '\n') || (index == (NMEA_MAX_DATA_LEN - 1)))
            {
                data_array[index+1] = 0;
                //NRF_LOG_HEXDUMP_INFO(data_array, index);
                parse_nmea_sentence((char*)data_array);
                
                index = 0;
            } else 
            {
                index++;
            }
            break;

        case APP_UART_COMMUNICATION_ERROR:
            //APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            //APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}

void gps_begin(void)
{

    uart_init();
    


}
