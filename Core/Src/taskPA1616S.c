/*
 * taskPA1616S.c
 *
 *  Created on: Oct 26, 2024
 *      Author: liamt
 */

#include <taskShared.h>
#include <taskPA1616S.h>
#include <minmea.h>

/******************************/
/*          GLOBALS           */
/******************************/

uint8_t pa1616s_buf[PA1616S_BUF_SIZE] = {0};
RingBuffer * local_pa1616s_ringbuffer;
osSemaphoreId_t * local_pa1616s_semaphore;

/******************************/
/*      HELPER FUNCTIONS      */
/******************************/

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t size)
{
    // Persistent variables to track DMA transactions across callback events.
    static uint16_t current_size   = 0;
    static uint16_t current_index  = 0;
    static uint16_t starting_index = 0;

    // In case of double callback, return without doing anything.
    if(size == current_index) return;

    // Increment the current size and update the current index.
    current_size = size > current_index
                 ? current_size + (size - current_index)
                 : current_size + (PA1616S_BUF_SIZE - current_index) + size;

    //current_size += (size - current_index);
    current_index = (starting_index + current_size) % PA1616S_BUF_SIZE;

    // Only if the last character written is '\n' do we want to
    //  process information. Otherwise, we do nothing until next callback.
    if(pa1616s_buf[size - 1] == '\n') {
        // Update storage used to pass information about data burst.
        RingBuffer_t const item = {
                .type = RB_GPS_IDLE,
                .pa1616s_idle = (PA1616S_IDLE){
                    .size = current_size,
                    .index = starting_index,
                    .buf = pa1616s_buf}
        };

        RingBuffer_enqueue(local_pa1616s_ringbuffer, item);

        // Notify process of data burst completion and reset current size
        //  and update starting index to get ready for next data burst.
        osSemaphoreRelease(*local_pa1616s_semaphore);
        current_size = 0;
        starting_index = current_index;
    }
}

static inline PA1616S_STATUS nmea_parse(char * const sentence,
                                        struct minmea_sentence_gga * const gga,
                                        struct minmea_sentence_rmc * const rmc)
{
    switch(minmea_sentence_id(sentence, false)) {
    // If the NMEA output sentence is GGA data:
    case MINMEA_SENTENCE_GGA:
        minmea_parse_gga(gga, sentence);
        break;
    // If the NMEA output sentence is RMC data:
    case MINMEA_SENTENCE_RMC:
        minmea_parse_rmc(rmc, sentence);
        break;
    // Else, return failure.
    default:
        return PA1616S_FAILURE;
    }

    return PA1616S_SUCCESS;
}

/******************************/
/*       CORE FUNCTIONS       */
/******************************/

void StartPA1616S(void *argument)
{
    TASK_ARGS const args = *(TASK_ARGS*)argument;

    // Set local pointers for idle callback function.
    local_pa1616s_ringbuffer = args.pa1616s_ringbuffer;
    local_pa1616s_semaphore = args.pa1616s_semaphore;

    // Configure UART to interrupt on idle line.
    if(PA1616S_Setup(args.huart_pa1616s, pa1616s_buf) != PA1616S_SUCCESS) {
        task_report_error(args, "GPS SETUP ERROR");
        vTaskSuspend(NULL);
    }

    RingBuffer_t update;
    uint16_t temp, end1, end2;
    char sentence[90] = {0};
    // Run indefinitely.
    for(;;) {
        // Wait for idle callback to notify of completed DMA transfer.
        osSemaphoreAcquire(*local_pa1616s_semaphore, osWaitForever);

        // Retrieve latest idle notification from DMA idle callback.
        if(RingBuffer_dequeue(args.pa1616s_ringbuffer, &update)
                != RB_SUCCESS) {
            task_report_error(args, "GPS RINGBUFFER DEQUEUE ERROR");
            vTaskSuspend(NULL);
        }

        // Setup to loop through DMA buffer.
        temp = update.pa1616s_idle.index + update.pa1616s_idle.size;
        end1 = temp > PA1616S_BUF_SIZE ? PA1616S_BUF_SIZE : temp;
        end2 = temp > PA1616S_BUF_SIZE ? temp - PA1616S_BUF_SIZE : 0;

        // Temporary storage for output NMEA sentences.
        struct minmea_sentence_gga gga;
        struct minmea_sentence_rmc rmc;

        uint16_t count = 0;
        for(int i = update.pa1616s_idle.index; i < end1; i++) {
            // Place the next character into the sentence buffer.
            sentence[count++] = pa1616s_buf[i];

            // If character is the end of a sentence:
            if(pa1616s_buf[i] == '\n') {
                // Parse the sentence using the open-source minmea library.
                nmea_parse(sentence, &gga, &rmc);

                // Reset the sentence buffer.
                memset(sentence, 0x00, sizeof(sentence) / sizeof(sentence[0]));
                count = 0;
            }
        }

        // If the DMA data burst does not wrap around the length of the
        //  buffer, then this for loop will be skipped.
        for(int i = 0; i < end2; i++) {
            // Place the next character into the sentence buffer.
            sentence[count++] = pa1616s_buf[i];

            // If character is the end of a sentence:
            if(pa1616s_buf[i] == '\n') {
                // Parse the sentence using the open-source minmea library.
                nmea_parse(sentence, &gga, &rmc);

                // Reset the sentence buffer.
                memset(sentence, 0x00, sizeof(sentence) / sizeof(sentence[0]));
                count = 0;
            }
        }

        // If reporting no GPS fix, then wait for the next DMA data burst.
        if(gga.fix_quality != 1) continue;

        // Send an update to the logger.
        RingBuffer_t const pa1616s = {
                .type = RB_GPS,
                .pa1616s = (PA1616S){
                    .longitude = minmea_tocoord(&rmc.longitude),
                    .latitude = minmea_tocoord(&rmc.latitude),
                    .altitude = minmea_tofloat(&gga.altitude),
                    .speed = minmea_tofloat(&rmc.speed),
                    .heading = minmea_tofloat(&rmc.course)}
        };

        task_send_update_log(args, pa1616s);
    }
}
