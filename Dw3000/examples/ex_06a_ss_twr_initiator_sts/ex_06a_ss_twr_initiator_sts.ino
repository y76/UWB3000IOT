#include "dw3000.h"

#define APP_NAME "SS TWR INIT STS v1.0"

// connection pins
const uint8_t PIN_RST = 27; // reset pin
const uint8_t PIN_IRQ = 34; // irq pin
const uint8_t PIN_SS = 4; // spi select pin

/* Inter-ranging delay period, in milliseconds. */
#define RNG_DELAY_MS 1000

/* Default antenna delay values for 64 MHz PRF. See NOTE 2 below. */
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

/* Frames used in the ranging process. See NOTE 3 below. */
static uint8_t tx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0, 0};
static uint8_t rx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

/* Length of the common part of the message (up to and including the function code, see NOTE 3 below). */
#define ALL_MSG_COMMON_LEN 10
/* Indexes to access some of the fields in the frames defined above. */
#define ALL_MSG_SN_IDX 2
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
#define RESP_MSG_TS_LEN 4
/* Frame sequence number, incremented after each transmission. */
static uint8_t frame_seq_nb = 0;

/* Buffer to store received response message.
 * Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN 24
static uint8_t rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32_t status_reg = 0;

/* Delay between frames, in UWB microseconds. See NOTE 1 below. */
#define POLL_TX_TO_RESP_RX_DLY_UUS (300 + CPU_COMP)
/* Receive response timeout. See NOTE 5 below. */
#define RESP_RX_TIMEOUT_UUS 700

/* Hold copies of computed time of flight and distance here for reference so that it can be examined at a debug breakpoint. */
static double tof;
static double distance;

/* Hold the amount of errors that have occurred */
static uint32_t errors[23] = {0};

extern dwt_config_t config_options;
extern dwt_txconfig_t txconfig_options;
extern dwt_txconfig_t txconfig_options_ch9;

/*
 * 128-bit STS key to be programmed into CP_KEY register.
 *
 * This key needs to be known and programmed the same at both units performing the SS-TWR.
 * In a real application for security this would be private and unique to the two communicating units
 * and chosen/assigned in a secure manner lasting just for the period of their association.
 *
 * Here we use a default KEY as specified in the IEEE 802.15.4z annex
 */
static dwt_sts_cp_key_t cp_key =
{
        0x14EB220F,0xF86050A8,0xD1D336AA,0x14148674
};

/*
 * 128-bit initial value for the nonce to be programmed into the CP_IV register.
 *
 * The IV, like the key, needs to be known and programmed the same at both units performing the SS-TWR.
 * It can be considered as an extension of the KEY. The low 32 bits of the IV is the counter.
 * In a real application for any particular key the value of the IV including the count should not be reused,
 * i.e. if the counter value wraps the upper 96-bits of the IV should be changed, e.g. incremented.
 *
 * Here we use a default IV as specified in the IEEE 802.15.4z annex
 */
static dwt_sts_cp_iv_t cp_iv =
{
        0x1F9A3DE4,0xD37EC3CA,0xC44FA8FB,0x362EEB34
};

/*
 * The 'poll' message initiating the ranging exchange includes a 32-bit counter which is part
 * of the IV used to generate the scrambled timestamp sequence (STS) in the transmitted packet.
 */
static void send_tx_poll_msg(void)
{
    /* Write frame data to DW IC and prepare transmission. See NOTE 7 below. */
    tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
    dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0); /* Zero offset in TX buffer. */
    dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1); /* Zero offset in TX buffer, ranging. */

    /* Start transmission. */
    dwt_starttx(DWT_START_TX_IMMEDIATE);

    /* Poll DW IC until TX frame sent event set. See NOTE 8 below. */
    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK))
    { };

    /* Clear TXFRS event. */
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
}


int16_t stsQual; /* This will contain STS quality index and status */
int goodSts = 0; /* Used for checking STS quality in received signal */
uint8_t firstLoopFlag = 0; /* Used to track if the program has gone through the first loop or not. */


void setup() {
  UART_init();
  test_run_info((unsigned char *)APP_NAME);

  /* Configure SPI rate, DW3000 supports up to 38 MHz */
  /* Reset DW IC */
  spiBegin(PIN_IRQ, PIN_RST);
  spiSelect(PIN_SS);

  delay(2); // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC, or could wait for SPIRDY event)

  while (!dwt_checkidlerc()) // Need to make sure DW IC is in IDLE_RC before proceeding 
  {
    UART_puts("IDLE FAILED\r\n");
    while (1) ;
  }

  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
  {
    UART_puts("INIT FAILED\r\n");
    while (1) ;
  }

  // Enabling LEDs here for debug so that for each TX the D1 LED will flash on DW3000 red eval-shield boards.
  dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

  /* Configure DW IC. See NOTE 15 below. */
    if(dwt_configure(&config_options)) /* if the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device */
    {
        test_run_info((unsigned char *)"CONFIG FAILED     ");
        while (1)
        { };
    }

    /* Configure the TX spectrum parameters (power, PG delay and PG count) */
    if(config_options.chan == 5)
    {
        dwt_configuretxrf(&txconfig_options);
    }
    else
    {
        dwt_configuretxrf(&txconfig_options_ch9);
    }

    /* Apply default antenna delay value. See NOTE 2 below. */
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);

    /* Set expected response's timeout. See NOTE 1 and 5 below.
     * As this example only handles one incoming frame with always the same delay, this value can be set here once for all. */
    set_resp_rx_timeout(RESP_RX_TIMEOUT_UUS, &config_options);

    /* Next can enable TX/RX states output on GPIOs 5 and 6 to help diagnostics, and also TX/RX LEDs */
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
}

static uint32_t message_count = 0;
static uint32_t successful_ranges = 0;
static uint32_t failed_ranges = 0;
static double distance_sum = 0;
static double last_distance = 0;
static uint32_t sts_failures = 0;
static uint32_t last_report_time = 0;

void print_message_content(const uint8_t* msg, size_t len, const char* prefix) {
    Serial.print(prefix);
    Serial.print(" [");
    Serial.print(len);
    Serial.print(" bytes]: ");
    for (size_t i = 0; i < len; i++) {
        if (msg[i] < 0x10) Serial.print("0");
        Serial.print(msg[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
}

void print_status_report() {
    const uint32_t REPORT_INTERVAL = 5000; // Print stats every 5 seconds
    uint32_t current_time = millis();
    
    if (current_time - last_report_time >= REPORT_INTERVAL) {
        Serial.println("\n=== STS Initiator Status Report ===");
        Serial.print("Messages sent: "); Serial.println(message_count);
        Serial.print("Successful ranges: "); Serial.println(successful_ranges);
        Serial.print("Failed ranges: "); Serial.println(failed_ranges);
        Serial.print("STS failures: "); Serial.println(sts_failures);
        Serial.print("Success rate: ");
        Serial.print(message_count > 0 ? (float)successful_ranges / message_count * 100 : 0);
        Serial.println("%");
        
        if (successful_ranges > 0) {
            Serial.print("Last distance: ");
            Serial.print(last_distance, 2);
            Serial.println(" m");
            
            Serial.print("Average distance: ");
            Serial.print(distance_sum / successful_ranges, 2);
            Serial.println(" m");
        }
        
        Serial.println("================================\n");
        last_report_time = current_time;
    }
}

// Modify the loop() function to include debugging
void loop() {
    message_count++;
    
    // Reload STS IV
    if (!firstLoopFlag) {
        dwt_configurestskey(&cp_key);
        dwt_configurestsiv(&cp_iv);
        dwt_configurestsloadiv();
        firstLoopFlag = 1;
        delay(1000);
        Serial.println("start");
    } else {
        dwt_writetodevice(STS_IV0_ID, 0, 4, (uint8_t *)&cp_iv);
        dwt_configurestsloadiv();
    }

    /* Write frame data to DW IC and prepare transmission. */
    tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
    dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
    dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1);

    print_message_content(tx_poll_msg, sizeof(tx_poll_msg), "TX Poll");
    
    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
uint32_t wait_start = millis();

    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
{ 
    if (millis() - wait_start > 1000) {  // 1 second max wait
        Serial.println("Forcing timeout - wait too long");
        break;
    }
    print_status_report();
}

    frame_seq_nb++;

    if (status_reg & SYS_STATUS_RXFCG_BIT_MASK)
    {
        uint32_t frame_len;
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
        frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
        
        if (frame_len <= sizeof(rx_buffer))
        {
            dwt_readrxdata(rx_buffer, frame_len, 0);
            print_message_content(rx_buffer, frame_len, "RX Response");

            // Check STS quality
            int16_t stsQual;
            int goodSts = dwt_readstsquality(&stsQual);
            
            if (goodSts < 0) {
                sts_failures++;
                Serial.println("STS Quality Check Failed");
                failed_ranges++;
            } else {
                rx_buffer[ALL_MSG_SN_IDX] = 0;
                if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0)
                {
                    uint32_t poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
                    int32_t rtd_init, rtd_resp;
                    float clockOffsetRatio;

                    poll_tx_ts = dwt_readtxtimestamplo32();
                    resp_rx_ts = dwt_readrxtimestamplo32();
                    clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1 << 26);

                    resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &poll_rx_ts);
                    resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &resp_tx_ts);

                    rtd_init = resp_rx_ts - poll_tx_ts;
                    rtd_resp = resp_tx_ts - poll_rx_ts;

                    tof = ((rtd_init - rtd_resp * (1 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;
                    distance = tof * SPEED_OF_LIGHT;
                    
                    // Update statistics
                    successful_ranges++;
                    last_distance = distance;
                    distance_sum += distance;

                    Serial.print("Distance: "); 
                    Serial.print(distance, 2);
                    Serial.println(" m");
                }
                else
                {
                    Serial.println("Frame validation failed");
                    failed_ranges++;
                }
            }
        }
    }
    else
    {
        failed_ranges++;
        Serial.print("RX Error/Timeout - Status: 0x");
        Serial.println(status_reg, HEX);
        
        if (status_reg & SYS_STATUS_ALL_RX_TO) Serial.println("TIMEOUT");
     //   if (status_reg & SYS_STATUS_RXPHE) Serial.println("PHY Header Error");
      //  if (status_reg & SYS_STATUS_RXFCE) Serial.println("Frame Check Error");
     //   if (status_reg & SYS_STATUS_RXRFSL) Serial.println("Reed Solomon Error");
        
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
    }

    Sleep(RNG_DELAY_MS);
}