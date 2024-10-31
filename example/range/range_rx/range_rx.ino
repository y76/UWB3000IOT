#include "dw3000.h"

#define PIN_RST 27
#define PIN_IRQ 34
#define PIN_SS 4

#define RNG_DELAY_MS 50
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385
#define ALL_MSG_COMMON_LEN 10
#define ALL_MSG_SN_IDX 2
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
#define RESP_MSG_TS_LEN 4
#define POLL_TX_TO_RESP_RX_DLY_UUS 300
#define RESP_RX_TIMEOUT_UUS 600

// Debug tracking variables
static uint32_t message_count = 0;
static uint32_t last_tx_time = 0;
static uint32_t successful_ranges = 0;
static uint32_t failed_ranges = 0;
static double distance_sum = 0;
static double last_distance = 0;

/* Default communication configuration. We use default non-STS DW mode. */
static dwt_config_t config = {
    9,                /* Channel number. */
    DWT_PLEN_128,     /* Preamble length. Used in TX only. */
    DWT_PAC8,         /* Preamble acquisition chunk size. Used in RX only. */
    9,                /* TX preamble code. Used in TX only. */
    9,                /* RX preamble code. Used in RX only. */
    1,                /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,       /* Data rate. */
    DWT_PHRMODE_STD,  /* PHY header mode. */
    DWT_PHRRATE_STD,  /* PHY header rate. */
    (129 + 8 - 8),    /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_OFF, /* STS disabled */
    DWT_STS_LEN_64,   /* STS length see allowed values in Enum dwt_sts_lengths_e */
    DWT_PDOA_M0       /* PDOA mode off */
};

static uint8_t tx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0, 0};
static uint8_t rx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8_t frame_seq_nb = 0;
static uint8_t rx_buffer[20];
static uint32_t status_reg = 0;
static double tof;
static double distance;

extern dwt_txconfig_t txconfig_options;

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
    //Serial.println();
}

void print_status_report() {
    static uint32_t last_report = 0;
    const uint32_t REPORT_INTERVAL = 5000; // Print stats every 5 seconds
    
    if (millis() - last_report >= REPORT_INTERVAL) {
        Serial.println("\n=== Status Report ===");
        Serial.print("Messages sent: "); Serial.println(message_count);
        Serial.print("Successful ranges: "); Serial.println(successful_ranges);
        Serial.print("Failed ranges: "); Serial.println(failed_ranges);
        Serial.print("Success rate: ");
        Serial.print(message_count > 0 ? (float)successful_ranges / message_count * 100 : 0);
        Serial.println("%");
        
        // Add distance information
        Serial.print("Last distance: ");
        Serial.print(last_distance, 2);
        Serial.println(" m");
        
        Serial.print("Average distance: ");
        if (successful_ranges > 0) {
            Serial.print(distance_sum / successful_ranges, 2);
            Serial.println(" m");
        } else {
            Serial.println("N/A");
        }
        
        Serial.println("===================\n");
        last_report = millis();
    }
}
void setup()
{
    Serial.begin(115200);
    UART_init();

    spiBegin(PIN_IRQ, PIN_RST);
    spiSelect(PIN_SS);

    delay(2);

    while (!dwt_checkidlerc())
    {
        Serial.println("IDLE FAILED");
        while (1);
    }

    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
    {
        Serial.println("INIT FAILED");
        while (1);
    }

    dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

    if (dwt_configure(&config))
    {
        Serial.println("CONFIG FAILED");
        while (1);
    }

    dwt_configuretxrf(&txconfig_options);
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);
    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

    Serial.println("Range Initiator Ready");
}

void loop()
{
    message_count++;
    last_tx_time = millis();
    
    /* Write frame data to DW IC and prepare transmission. */
    tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
    dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
    dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1);

  //  print_message_content(tx_poll_msg, sizeof(tx_poll_msg), "TX Poll");
    
    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
    {
        print_status_report();  // Print periodic status while waiting
    };

    frame_seq_nb++;

    if (status_reg & SYS_STATUS_RXFCG_BIT_MASK)
    {
        uint32_t frame_len;

        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

        frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
        if (frame_len <= sizeof(rx_buffer))
        {
            dwt_readrxdata(rx_buffer, frame_len, 0);
           // print_message_content(rx_buffer, frame_len, "RX Response");

            rx_buffer[ALL_MSG_SN_IDX] = 0;
            if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0)
            {
                uint32_t poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
                int32_t rtd_init, rtd_resp;
                float clockOffsetRatio;

                /* Retrieve timestamps */
                poll_tx_ts = dwt_readtxtimestamplo32();
                resp_rx_ts = dwt_readrxtimestamplo32();
                clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1 << 26);

                /* Get timestamps embedded in response message. */
                resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &poll_rx_ts);
                resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &resp_tx_ts);

                /* Compute time of flight and distance */
                rtd_init = resp_rx_ts - poll_tx_ts;
                rtd_resp = resp_tx_ts - poll_rx_ts;

                tof = ((rtd_init - rtd_resp * (1 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;
                distance = tof * SPEED_OF_LIGHT;
                last_distance = distance;
                distance_sum += distance;

                /* Display computed distance */
               // snprintf(dist_str, sizeof(dist_str), "DIST: %3.2f m", distance);
                //test_run_info((unsigned char *)dist_str);
                successful_ranges++;
            }
        }
    }
    else
    {
        failed_ranges++;
        // Print specific timeout/error information
        //Serial.print("RX Error/Timeout (");
       // if (status_reg & SYS_STATUS_ALL_RX_TO) Serial.print("TIMEOUT");
       // else if (status_reg & SYS_STATUS_RXPHE_BIT_MASK) Serial.print("PHE");
      //  else if (status_reg & SYS_STATUS_RXFCE_BIT_MASK) Serial.print("FCE");
      //  else if (status_reg & SYS_STATUS_RXFSL_BIT_MASK) Serial.print("RFSL");  // Fixed this line
      // Serial.println(")");
        
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
    }

    /* Execute a delay between ranging exchanges. */
    Sleep(RNG_DELAY_MS);
}