/*! ----------------------------------------------------------------------------
 * @file    main.c
 * @brief   This is the a variant of implementation of the Slotted TWR PDoA Node 
 *          on Nordic nRF52840 platform with FreeRTOS
 *
 * @author  Decawave Applications
 *
 * @attention Copyright 2017-2019 (c) DecaWave Ltd, Dublin, Ireland.
 *            All rights reserved.
 *
 */
/* Includes ------------------------------------------------------------------*/

#include "port.h"
#include "deca_device_api.h"
#include "deca_dbg.h"
#include "deca_probe_interface.h"
#include "crc16.h"

#include "app.h"

#include "nrf_drv_wdt.h"

#include <config.h>
#include <port.h>

#include <node.h>
#include <tag_list.h>
#include <task_node.h>
#include <task_usb2spi.h>
#include <task_tcfm.h>
#include <task_tcwm.h>
#include <task_flush.h>
#include <task_tag.h>

#include "nrf_uart.h"

#define USB_DRV_UPDATE_MS   200

/* Private variables ---------------------------------------------------------*/

osThreadId        defaultTaskHandle;
gDiagPrintFStr_t  gDiagPrintFStr;
app_t             app;    /**< All global variables are in the "app" structure */

//-----------------------------------------------------------------------------
//NRF port
uint32_t gRTC_SF_PERIOD = 3276;


extern nrf_drv_wdt_channel_id m_channel_id;
extern int gRangingStart;

extern void trilat_helper(void const *argument);
extern void trilat_terminate_tasks(void);

void StartDefaultTask(void const * argument);
void CtrlTask(void const * arg);
void deca_usb_init(void);

extern void trilat_terminate();
extern void listener_terminate();
extern void listener_helper();

/**
 * @brief Function for application main entry.
 */
 
/* Example application name */
#define APP_NAME "SIMPLE TX v1.0"

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
    DWT_STS_MODE_OFF, /* No STS mode enabled (STS Mode 0). */
    DWT_STS_LEN_64,   /* STS length, see allowed values in Enum dwt_sts_lengths_e */
    DWT_PDOA_M0       /* PDOA mode off */
};

/* The frame sent in this example is an 802.15.4e standard blink. It is a 12-byte frame composed of the following fields:
 *     - byte 0: frame type (0xC5 for a blink).
 *     - byte 1: sequence number, incremented for each new frame.
 *     - byte 2 -> 9: device ID, see NOTE 1 below.
 */
static uint8_t tx_msg[] = { 0xC5, 0, 'D', 'E', 'C', 'A', 'W', 'A', 'V', 'E' };
/* Index to access to sequence number of the blink frame in the tx_msg array. */
#define BLINK_FRAME_SN_IDX 1

#define FRAME_LENGTH (sizeof(tx_msg) + FCS_LEN) // The real length that is going to be transmitted

/* Inter-frame delay period, in milliseconds. */
#define TX_DELAY_MS 1//500

dwt_txconfig_t txconfig_options_ch9 = {
    0x34,       /* PG delay. */
    0xcacacaca, /* TX power. */
    0x0         /*PG count*/
};
void test_run_info(unsigned char *data)
{
    //printf("%s\n", data);
}

void dwt_while(void)
{
        //while (1){        
        /* Write frame data to DW IC and prepare transmission. See NOTE 3 below.*/
        dwt_writetxdata(FRAME_LENGTH - FCS_LEN, tx_msg, 0); /* Zero offset in TX buffer. */

        /* In this example since the length of the transmitted frame does not change,
         * nor the other parameters of the dwt_writetxfctrl function, the
         * dwt_writetxfctrl call could be outside the main while(1) loop.
         */
        dwt_writetxfctrl(FRAME_LENGTH, 0, 0); /* Zero offset in TX buffer, no ranging. */

        /* Start transmission. */
        dwt_starttx(DWT_START_TX_IMMEDIATE);
        /* Poll DW IC until TX frame sent event set. See NOTE 4 below.
         * STATUS register is 4 bytes long but, as the event we are looking
         * at is in the first byte of the register, we can use this simplest
         * API function to access it.*/
        waitforsysstatus(NULL, NULL, DWT_INT_TXFRS_BIT_MASK, 0);

        /* Clear TX frame sent event. */
        dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);

        //test_run_info((unsigned char *)"TX Frame Sent");

        /* Execute a delay between transmissions. */
        nrf_delay_ms(TX_DELAY_MS);

        /* Increment the blink frame sequence number (modulo 256). */
        //tx_msg[BLINK_FRAME_SN_IDX]++;
    //}
}
#ifdef ENABLE_LOOPBACK_TEST
/* Use flow control in loopback test. */
#define UART_HWFC APP_UART_FLOW_CONTROL_ENABLED

/** @brief Function for setting the @ref ERROR_PIN high, and then enter an infinite loop.
 */
static void show_error(void)
{

    bsp_board_leds_on();
    while (true)
    {
        // Do nothing.
    }
}


/** @brief Function for testing UART loop back.
 *  @details Transmitts one character at a time to check if the data received from the loopback is same as the transmitted data.
 *  @note  @ref TX_PIN_NUMBER must be connected to @ref RX_PIN_NUMBER)
 */
static void uart_loopback_test()
{
    uint8_t * tx_data = (uint8_t *)("\r\nLOOPBACK_TEST\r\n");
    uint8_t   rx_data;

    // Start sending one byte and see if you get the same
    for (uint32_t i = 0; i < MAX_TEST_DATA_BYTES; i++)
    {
        uint32_t err_code;
        while (app_uart_put(tx_data[i]) != NRF_SUCCESS);

        nrf_delay_ms(10);
        err_code = app_uart_get(&rx_data);

        if ((rx_data != tx_data[i]) || (err_code != NRF_SUCCESS))
        {
            show_error();
        }
    }
    return;
}
#else
/* When UART is used for communication with the host do not use flow control.*/
#define UART_HWFC APP_UART_FLOW_CONTROL_DISABLED
#endif

/* Inter-ranging delay period, in milliseconds. */
#define RNG_DELAY_MS 20//1000

/* Frames used in the ranging process. See NOTE 2 below. */
static uint8_t rx_poll_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x21, 0, 0 };
static uint8_t tx_resp_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0x10, 0x02, 0, 0, 0, 0 };
static uint8_t rx_final_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
/* Length of the common part of the message (up to and including the function code, see NOTE 2 below). */
#define ALL_MSG_COMMON_LEN 10
/* Index to access some of the fields in the frames involved in the process. */
#define ALL_MSG_SN_IDX            2
#define FINAL_MSG_POLL_TX_TS_IDX  10
#define FINAL_MSG_RESP_RX_TS_IDX  14
#define FINAL_MSG_FINAL_TX_TS_IDX 18
/* Frame sequence number, incremented after each transmission. */
static uint8_t frame_seq_nb = 0;

/* Buffer to store received messages.
 * Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN 24
static uint8_t rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32_t status_reg = 0;

/* Delay between frames, in UWB microseconds. See NOTE 4 below. */
/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW IC's delayed TX function. This includes the
 * frame length of approximately 190 us with above configuration. */
#define POLL_RX_TO_RESP_TX_DLY_UUS 900
/* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW IC's wait for response feature. */
#define RESP_TX_TO_FINAL_RX_DLY_UUS 500
/* Receive final timeout. See NOTE 5 below. */
#define FINAL_RX_TIMEOUT_UUS 220
/* Preamble timeout, in multiple of PAC size. See NOTE 6 below. */
#define PRE_TIMEOUT 5

/* Timestamps of frames transmission/reception. */
static uint64_t poll_rx_ts;
static uint64_t resp_tx_ts;
static uint64_t final_rx_ts;

/* Hold copies of computed time of flight and distance here for reference so that it can be examined at a debug breakpoint. */
static double tof;
static double distance;
char dist_str_temp[32];
/* String used to display measured distance on LCD screen (16 characters maximum). */
char dist_str[16] = { 0 };
#if 0
/* Buffer to store received response message.
 * Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN 20
static uint8_t rx_buffer[RX_BUF_LEN];
/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32_t status_reg = 0;
#define CPU_PROCESSING_TIME 400
/* Delay between frames, in UWB microseconds. See NOTE 4 below. */
/* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW IC's wait for response feature. */
#define POLL_TX_TO_RESP_RX_DLY_UUS (300 + CPU_PROCESSING_TIME)
/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW IC's delayed TX function.
 * This value is required to be larger than POLL_TX_TO_RESP_RX_DLY_UUS. Please see NOTE 4 for more details. */
#define RESP_RX_TO_FINAL_TX_DLY_UUS (300 + CPU_PROCESSING_TIME)
/* Receive response timeout. See NOTE 5 below. */
#define RESP_RX_TIMEOUT_UUS 300
/* Preamble timeout, in multiple of PAC size. See NOTE 7 below. */
#define PRE_TIMEOUT 5

/* Time-stamps of frames transmission/reception, expressed in device time units. */
static uint64_t poll_tx_ts;
static uint64_t resp_rx_ts;
static uint64_t final_tx_ts;

/* Frames used in the ranging process. See NOTE 2 below. */
static uint8_t tx_poll_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x21 };
static uint8_t rx_resp_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0x10, 0x02, 0, 0 };
static uint8_t tx_final_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
/* Length of the common part of the message (up to and including the function code, see NOTE 2 below). */
#define ALL_MSG_COMMON_LEN 10
/* Indexes to access some of the fields in the frames defined above. */
#define ALL_MSG_SN_IDX            2
#define FINAL_MSG_POLL_TX_TS_IDX  10
#define FINAL_MSG_RESP_RX_TS_IDX  14
#define FINAL_MSG_FINAL_TX_TS_IDX 18
/* Frame sequence number, incremented after each transmission. */
static uint8_t frame_seq_nb = 0;
#endif
/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
 * 1 uus = 512 / 499.2 祍 and 1 祍 = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 63898

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_tx_timestamp_u64()
 *
 * @brief Get the TX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
uint64_t get_tx_timestamp_u64(void)
{
    uint8_t ts_tab[5];
    uint64_t ts = 0;
    int8_t i;
    dwt_readtxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_rx_timestamp_u64()
 *
 * @brief Get the RX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
uint64_t get_rx_timestamp_u64(void)
{
    uint8_t ts_tab[5];
    uint64_t ts = 0;
    int8_t i;
    dwt_readrxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}
#define FINAL_MSG_TS_LEN 4
/*! ------------------------------------------------------------------------------------------------------------------
 * @fn final_msg_get_ts()
 *
 * @brief Read a given timestamp value from the final message. In the timestamp fields of the final message, the least
 *        significant byte is at the lower address.
 *
 * @param  ts_field  pointer on the first byte of the timestamp field to read
 *         ts  timestamp value
 *
 * @return none
 */
void final_msg_get_ts(const uint8_t *ts_field, uint32_t *ts)
{
    uint8_t i;
    *ts = 0;
    for (i = 0; i < FINAL_MSG_TS_LEN; i++)
    {
        *ts += ((uint32_t)ts_field[i] << (i * 8));
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn final_msg_set_ts()
 *
 * @brief Fill a given timestamp field in the final message with the given value. In the timestamp fields of the final
 *        message, the least significant byte is at the lower address.
 *
 * @param  ts_field  pointer on the first byte of the timestamp field to fill
 *         ts  timestamp value
 *
 * @return none
 */
void final_msg_set_ts(uint8_t *ts_field, uint64_t ts)
{
    uint8_t i;
    for (i = 0; i < FINAL_MSG_TS_LEN; i++)
    {
        ts_field[i] = (uint8_t)ts;
        ts >>= 8;
    }
}
volatile double dist_str_d=0;
volatile uint32_t dist_str_num=0;
volatile uint8_t dist_str_flag=0;
volatile uint8_t event=0;
#define IDLE 0
#define TEST_SIMPLE_TX 1
#define TEST_DS_TWR_INITIATOR 2
#define TEST_DS_TWR_RESPONDER 3
#define RTB_STATUS 4
uint32_t waitforsysstatus_timeout=0;
void waitforsysstatus(uint32_t *lo_result, uint32_t *hi_result, uint32_t lo_mask, uint32_t hi_mask)
{
    uint32_t lo_result_tmp = 0;
    uint32_t hi_result_tmp = 0;

    // If a mask has been passed into the function for the system status register (lower 32-bits)
    if (lo_mask)
    {
        while (!((lo_result_tmp = dwt_readsysstatuslo()) & (lo_mask)))
        {
            waitforsysstatus_timeout++;
            if(waitforsysstatus_timeout>200000)
            {
              waitforsysstatus_timeout=0;
              break;
            }
            // If a mask value is set for the system status register (higher 32-bits)
            if (hi_mask)
            {
                // If mask value for the system status register (higher 32-bits) is found
                if ((hi_result_tmp = dwt_readsysstatushi()) & hi_mask)
                {
                    waitforsysstatus_timeout=0;
                    break;
                }
            }
        }
    }
    // if only a mask value for the system status register (higher 32-bits) is set
    else if (hi_mask)
    {
        while (!((hi_result_tmp = dwt_readsysstatushi()) & (hi_mask))) {};
    }

    if (lo_result != NULL)
    {
        *lo_result = lo_result_tmp;
    }

    if (hi_result != NULL)
    {
        *hi_result = hi_result_tmp;
    }
}
char dat[64];
uint16_t dat_num=0;
volatile uint32_t rxdelay=16391;//16385;
volatile uint32_t txdelay=16391;//16385;
void uart_send(char *str,uint32_t str_len)
{
    for(uint32_t i=0;i<str_len;i++)
    app_uart_put(str[i]);
}
static TaskHandle_t m_UART_task;
void uart_error_handle(app_uart_evt_t * p_event)
{
    
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
    else if (p_event->evt_type == APP_UART_DATA_READY)
    {
        uint8_t cr;
        app_uart_get(&cr);
        //app_uart_put(cr);
        dat[dat_num]=cr;
        dat_num++;
        if(dat[dat_num-2]=='\r'&& dat[dat_num-1]=='\n')
        {
            if(strcmp(dat, "rtb antenna delay model\r\n")==0)//dut antenna delay model
            {
                //event=TEST_DS_TWR_INITIATOR;
                event=TEST_DS_TWR_RESPONDER;
            }
            else if(strcmp(dat, "rtb status?\r\n")==0)//dut antenna delay model
            {
                event=RTB_STATUS;
            }
            else if(strcmp(dat, "rtb sensitivity start\r\n")==0)//dut antenna delay model
            {
                event=TEST_SIMPLE_TX;
            }
            else if (strcmp(dat, "rtb sensitivity stop\r\n")==0)
            {
              uart_send("ok\r\n",strlen("ok\r\n"));
              event=IDLE;
            }
            else
            {
              uart_send("err",strlen("err"));
              event=IDLE;
            }
            osThreadResume(m_UART_task);
            dat_num=0;
            memset(dat, 0, sizeof(dat));
        }
    }
    else if (p_event->evt_type == APP_UART_TX_EMPTY) 
    {//发送完成
            //发送完成不知道要做什么的,可以点个灯提醒
            //nrf_gpio_pin_toggle(LED0);
            
    }
}

static void uart_task(void * parm)
{
    
    for(;;)
    {
      switch (event)
      {
          case IDLE:
          osThreadSuspend(m_UART_task);
          break;
          case RTB_STATUS:
          uart_send("ok\r\n",strlen("ok\r\n"));
          event=IDLE;
          break;
          case TEST_SIMPLE_TX:
          uart_send("ok\r\n",strlen("ok\r\n"));
          //osDelay(1000);        /**< small pause to startup */

          reset_DW3000(); //this will reset DW device
          
          if (dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf))
          {
            error_handler(1, _ERR_INIT);
          }
          int32_t dev_id = dwt_readdevid();

            while (!dwt_checkidlerc()) /* Need to make sure DW IC is in IDLE_RC before proceeding */ { };
          /* This initialization is added to avoid crashing the board when calling APIs that writes inside local data
          like setxtaltrim */
          if (dwt_initialise(DWT_DW_INIT) != DWT_SUCCESS)
          {
            error_handler(1, _ERR_INIT);
          }
         /* Enabling LEDs here for debug so that for each TX the D1 LED will flash on DW3000 red eval-shield boards. */
          dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

          /* Configure DW IC. See NOTE 5 below. */
          /* if the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device */
          if (dwt_configure(&config))
          {
              //test_run_info((unsigned char *)"CONFIG FAILED     ");
              //while (1) { };
          }

          /* Configure the TX spectrum parameters (power PG delay and PG Count) */
          dwt_configuretxrf(&txconfig_options_ch9);
          
          while (1){        
          if(event!=TEST_SIMPLE_TX)
            break;
          /* Write frame data to DW IC and prepare transmission. See NOTE 3 below.*/
          dwt_writetxdata(FRAME_LENGTH - FCS_LEN, tx_msg, 0); /* Zero offset in TX buffer. */

          /* In this example since the length of the transmitted frame does not change,
           * nor the other parameters of the dwt_writetxfctrl function, the
           * dwt_writetxfctrl call could be outside the main while(1) loop.
           */
          dwt_writetxfctrl(FRAME_LENGTH, 0, 0); /* Zero offset in TX buffer, no ranging. */

          /* Start transmission. */
          dwt_starttx(DWT_START_TX_IMMEDIATE);
          /* Poll DW IC until TX frame sent event set. See NOTE 4 below.
           * STATUS register is 4 bytes long but, as the event we are looking
           * at is in the first byte of the register, we can use this simplest
           * API function to access it.*/
          waitforsysstatus(NULL, NULL, DWT_INT_TXFRS_BIT_MASK, 0);

          /* Clear TX frame sent event. */
          dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);

          //test_run_info((unsigned char *)"TX Frame Sent");
          

          /* Execute a delay between transmissions. */
          osDelay(TX_DELAY_MS);

          /* Increment the blink frame sequence number (modulo 256). */
          //tx_msg[BLINK_FRAME_SN_IDX]++;
          }
          event=IDLE;
          break;
          #if 0
          case TEST_DS_TWR_INITIATOR:
          reset_DW3000(); //this will reset DW device

          /* Probe for the correct device driver. */
          dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf);

          while (!dwt_checkidlerc()) /* Need to make sure DW IC is in IDLE_RC before proceeding */ { };

          if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
          {
              test_run_info((unsigned char *)"INIT FAILED     ");
              while (1) { };
          }

          /* Configure DW IC. See NOTE 2 below. */
          /* if the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device */
          if (dwt_configure(&config))
          {
              test_run_info((unsigned char *)"CONFIG FAILED     ");
              while (1) { };
          }

          /* Configure the TX spectrum parameters (power, PG delay and PG count) */
          dwt_configuretxrf(&txconfig_options_ch9);

          /* Apply default antenna delay value. See NOTE 1 below. */
          dwt_setrxantennadelay(RX_ANT_DLY);
          dwt_settxantennadelay(TX_ANT_DLY);

          /* Set expected response's delay and timeout. See NOTE 4, 5 and 7 below.
           * As this example only handles one incoming frame with always the same delay and timeout, those values can be set here once for all. */
          dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
          dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
          dwt_setpreambledetecttimeout(PRE_TIMEOUT);

          /* Next can enable TX/RX states output on GPIOs 5 and 6 to help debug, and also TX/RX LEDs
           * Note, in real low power applications the LEDs should not be used. */
          dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
          // dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

          /* Loop forever initiating ranging exchanges. */
          while (1)
          {
              /* Write frame data to DW IC and prepare transmission. See NOTE 9 below. */
              tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
              dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);  /* Zero offset in TX buffer. */
              dwt_writetxfctrl(sizeof(tx_poll_msg) + FCS_LEN, 0, 1); /* Zero offset in TX buffer, ranging. */

              /* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
               * set by dwt_setrxaftertxdelay() has elapsed. */
              dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

              /* We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout. See NOTE 10 below. */
              waitforsysstatus(&status_reg, NULL, (DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR), 0);

              /* Increment frame sequence number after transmission of the poll message (modulo 256). */
              frame_seq_nb++;

              if (status_reg & DWT_INT_RXFCG_BIT_MASK)
              {
                  uint16_t frame_len;

                  /* Clear good RX frame event and TX frame sent in the DW IC status register. */
                  dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK | DWT_INT_TXFRS_BIT_MASK);

                  /* A frame has been received, read it into the local buffer. */
                  frame_len = dwt_getframelength();
                  if (frame_len <= RX_BUF_LEN)
                  {
                      dwt_readrxdata(rx_buffer, frame_len, 0);
                  }

                  /* Check that the frame is the expected response from the companion "DS TWR responder" example.
                   * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
                  rx_buffer[ALL_MSG_SN_IDX] = 0;
                  if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0)
                  {
                      uint32_t final_tx_time;
                      int ret;

                      /* Retrieve poll transmission and response reception timestamp. */
                      poll_tx_ts = get_tx_timestamp_u64();
                      resp_rx_ts = get_rx_timestamp_u64();

                      /* Compute final message transmission time. See NOTE 11 below. */
                      final_tx_time = (resp_rx_ts + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
                      dwt_setdelayedtrxtime(final_tx_time);

                      /* Final TX timestamp is the transmission time we programmed plus the TX antenna delay. */
                      final_tx_ts = (((uint64_t)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

                      /* Write all timestamps in the final message. See NOTE 12 below. */
                      final_msg_set_ts(&tx_final_msg[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts);
                      final_msg_set_ts(&tx_final_msg[FINAL_MSG_RESP_RX_TS_IDX], resp_rx_ts);
                      final_msg_set_ts(&tx_final_msg[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts);

                      /* Write and send final message. See NOTE 9 below. */
                      tx_final_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
                      dwt_writetxdata(sizeof(tx_final_msg), tx_final_msg, 0); /* Zero offset in TX buffer. */
                      dwt_writetxfctrl(sizeof(tx_final_msg) + FCS_LEN, 0, 1); /* Zero offset in TX buffer, ranging bit set. */

                      ret = dwt_starttx(DWT_START_TX_DELAYED);
                      /* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. See NOTE 13 below. */
                      if (ret == DWT_SUCCESS)
                      {
                          /* Poll DW IC until TX frame sent event set. See NOTE 10 below. */
                          waitforsysstatus(NULL, NULL, DWT_INT_TXFRS_BIT_MASK, 0);

                          /* Clear TXFRS event. */
                          dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);

                          /* Increment frame sequence number after transmission of the final message (modulo 256). */
                          frame_seq_nb++;
                      }
                  }
              }
              else
              {
                  /* Clear RX error/timeout events in the DW IC status register. */
                  dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR | DWT_INT_TXFRS_BIT_MASK);
              }
              if(event!=TEST_DS_TWR_INITIATOR)
                break;

              /* Execute a delay between ranging exchanges. */
              osDelay(RNG_DELAY_MS);
          }
          event=IDLE;
          break;
          #endif
          case TEST_DS_TWR_RESPONDER:
          uart_send("ok\r\n",strlen("ok\r\n"));
          dist_str_num=0;
          dist_str_d=0;
          reset_DW3000(); //this will reset DW device

           /* Probe for the correct device driver. */
          dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf);

          while (!dwt_checkidlerc()) /* Need to make sure DW IC is in IDLE_RC before proceeding */ { };

          if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
          {
              /*test_run_info((unsigned char *)"INIT FAILED     ");
              while (1) { };*/
          }

          /* Configure DW IC. See NOTE 15 below. */
          /* if the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device */
          if (dwt_configure(&config))
          {
              /*test_run_info((unsigned char *)"CONFIG FAILED     ");
              while (1) { };*/
          }

          /* Configure the TX spectrum parameters (power, PG delay and PG count) */
          dwt_configuretxrf(&txconfig_options_ch9);

          /* Apply default antenna delay value. See NOTE 1 below. */

          dwt_setrxantennadelay(rxdelay);
          dwt_settxantennadelay(txdelay);

          /* Next can enable TX/RX states output on GPIOs 5 and 6 to help debug, and also TX/RX LEDs
           * Note, in real low power applications the LEDs should not be used. */
          dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

          /* Loop forever responding to ranging requests. */
          while (1)
          {
              dwt_setpreambledetecttimeout(0);
              /* Clear reception timeout to start next ranging process. */
              dwt_setrxtimeout(0);

              /* Activate reception immediately. */
              dwt_rxenable(DWT_START_RX_IMMEDIATE);

              /* Poll for reception of a frame or error/timeout. See NOTE 8 below. */
              waitforsysstatus(&status_reg, NULL, (DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR), 0);

              if (status_reg & DWT_INT_RXFCG_BIT_MASK)
              {
                  uint16_t frame_len;

                  /* Clear good RX frame event in the DW IC status register. */
                  dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);

                  /* A frame has been received, read it into the local buffer. */
                  frame_len = dwt_getframelength();
                  if (frame_len <= RX_BUF_LEN)
                  {
                      dwt_readrxdata(rx_buffer, frame_len, 0);
                  }

                  /* Check that the frame is a poll sent by "DS TWR initiator" example.
                   * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
                  rx_buffer[ALL_MSG_SN_IDX] = 0;
                  if (memcmp(rx_buffer, rx_poll_msg, ALL_MSG_COMMON_LEN) == 0)
                  {
                      uint32_t resp_tx_time;
                      int ret;

                      /* Retrieve poll reception timestamp. */
                      poll_rx_ts = get_rx_timestamp_u64();

                      /* Set send time for response. See NOTE 9 below. */
                      resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
                      dwt_setdelayedtrxtime(resp_tx_time);

                      /* Set expected delay and timeout for final message reception. See NOTE 4 and 5 below. */
                      dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS);
                      dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);
                      /* Set preamble timeout for expected frames. See NOTE 6 below. */
                      dwt_setpreambledetecttimeout(PRE_TIMEOUT);

                      /* Write and send the response message. See NOTE 10 below.*/
                      tx_resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
                      dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0); /* Zero offset in TX buffer. */
                      dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1);          /* Zero offset in TX buffer, ranging. */
                      ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);

                      /* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. See NOTE 11 below. */
                      if (ret == DWT_ERROR)
                      {
                          continue;
                      }

                      /* Poll for reception of expected "final" frame or error/timeout. See NOTE 8 below. */
                      waitforsysstatus(&status_reg, NULL, (DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR), 0);

                      /* Increment frame sequence number after transmission of the response message (modulo 256). */
                      frame_seq_nb++;

                      if (status_reg & DWT_INT_RXFCG_BIT_MASK)
                      {
                          /* Clear good RX frame event and TX frame sent in the DW IC status register. */
                          dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK | DWT_INT_TXFRS_BIT_MASK);

                          /* A frame has been received, read it into the local buffer. */
                          frame_len = dwt_getframelength();
                          if (frame_len <= RX_BUF_LEN)
                          {
                              dwt_readrxdata(rx_buffer, frame_len, 0);
                          }

                          /* Check that the frame is a final message sent by "DS TWR initiator" example.
                           * As the sequence number field of the frame is not used in this example, it can be zeroed to ease the validation of the frame. */
                          rx_buffer[ALL_MSG_SN_IDX] = 0;
                          if (memcmp(rx_buffer, rx_final_msg, ALL_MSG_COMMON_LEN) == 0)
                          {
                              uint32_t poll_tx_ts, resp_rx_ts, final_tx_ts;
                              uint32_t poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;
                              double Ra, Rb, Da, Db;
                              int64_t tof_dtu;

                              /* Retrieve response transmission and final reception timestamps. */
                              resp_tx_ts = get_tx_timestamp_u64();
                              final_rx_ts = get_rx_timestamp_u64();

                              /* Get timestamps embedded in the final message. */
                              final_msg_get_ts(&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX], &poll_tx_ts);
                              final_msg_get_ts(&rx_buffer[FINAL_MSG_RESP_RX_TS_IDX], &resp_rx_ts);
                              final_msg_get_ts(&rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts);

                              /* Compute time of flight. 32-bit subtractions give correct answers even if clock has wrapped. See NOTE 12 below. */
                              poll_rx_ts_32 = (uint32_t)poll_rx_ts;
                              resp_tx_ts_32 = (uint32_t)resp_tx_ts;
                              final_rx_ts_32 = (uint32_t)final_rx_ts;
                              Ra = (double)(resp_rx_ts - poll_tx_ts);
                              Rb = (double)(final_rx_ts_32 - resp_tx_ts_32);
                              Da = (double)(final_tx_ts - resp_rx_ts);
                              Db = (double)(resp_tx_ts_32 - poll_rx_ts_32);
                              tof_dtu = (int64_t)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));

                              tof = tof_dtu * DWT_TIME_UNITS;
                              distance = tof * SPEED_OF_LIGHT;
                              /* Display computed distance on LCD. */
                              sprintf(dist_str, "DIST: %3.2f m \n", distance);
                              //uart_send("dist_str",strlen("dist_str"));
                              //test_run_info((unsigned char *)dist_str);
                              dist_str_flag++;
                              dist_str_d+=distance;
                              if(dist_str_flag>99)
                              {
                                
                                dist_str_num=(dist_str_d/dist_str_flag)*dist_str_flag;
                                dist_str_flag=0;
                                memset(dist_str_temp,0,sizeof(dist_str_temp));
                                sprintf(dist_str_temp, "%d cm", dist_str_num);
                                uart_send("rtb test distance:",strlen("rtb test distance:"));
                                uart_send(dist_str_temp,strlen(dist_str_temp));
                                uart_send("\r\n",strlen("\r\n"));
                                //dist_str_num=0;
                                break;
                              }
                              /*if(event!=IDLE)
                              {
                                break;
                              }*/
                              /* as DS-TWR initiator is waiting for RNG_DELAY_MS before next poll transmission
                               * we can add a delay here before RX is re-enabled again
                               */
                              osDelay(RNG_DELAY_MS - 10); // start couple of ms earlier
                          }
                      }
                      else
                      {
                          /* Clear RX error/timeout events in the DW IC status register. */
                          dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
                      }
                  }
              }
              else
              {
                  /* Clear RX error/timeout events in the DW IC status register. */
                  dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
                  uart_send("rtb test distance:0 cm\r\n",strlen("rtb test distance:0 cm\r\n"));
                  break;
              }
          }
          event=IDLE;
          break;
          default:
          break;
      }
    }
}
int main(void)
{
    peripherals_init();
    port_init_dw_chip();
    dw_irq_init();

uint32_t err_code;
  const app_uart_comm_params_t comm_params =
    {
          8,
          6,
          5,
          7,
          UART_HWFC,
          false,
#if defined (UART_PRESENT)
          NRF_UART_BAUDRATE_115200
#else
          NRF_UARTE_BAUDRATE_115200
#endif
    };
    APP_UART_FIFO_INIT(&comm_params,
                         256,
                         256,
                         uart_error_handle,
                         APP_IRQ_PRIORITY_LOWEST,
                         err_code);

/*
app.xStartTaskEvent = xEventGroupCreate(); 
osMailQDef(rxPcktPool_q, RX_MAIL_QUEUE_SIZE, rx_mail_t);
    app.rxPcktPool_q_id = osMailCreate(osMailQ(rxPcktPool_q), NULL);

    if(!app.rxPcktPool_q_id)
    {
        error_handler(1, _ERR_Cannot_Alloc_Mail);
    }
  osThreadDef(defaultTask, StartDefaultTask, PRIO_StartDefaultTask, 0, configMINIMAL_STACK_SIZE*2);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);*/
BaseType_t xReturned = xTaskCreate(uart_task,
                                       "UART",
                                       512,
                                       NULL,
                                       0,
                                       &m_UART_task);
if (xReturned != pdPASS)
{
    //NRF_LOG_ERROR("uart_task not created.");
    APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
}

osKernelStart();
    for (;;) {
        APP_ERROR_HANDLER(NRF_ERROR_FORBIDDEN);
    }
#if 0
    int devID = 0, delayCnt = 0, status = 0;
    ret_code_t err_code;

    gRangingStart = 0;

    init_crc16();

//    bsp_board_init(BSP_INIT_LEDS|BSP_INIT_BUTTONS);
    peripherals_init();
    port_init_dw_chip();
    dw_irq_init();

    memset(&app,0,sizeof(app));
    memset(&gDiagPrintFStr, 0, sizeof(gDiagPrintFStr));

    load_bssConfig();                 /**< load the RAM Configuration parameters from NVM block */
    app.pConfig = get_pbssConfig();
    if (app.pConfig->s.uartEn)
    {
        deca_uart_init();
    }

    app.xStartTaskEvent = xEventGroupCreate(); /**< xStartTaskEvent indicates which tasks to be started */

    /*for(int i=0; i<6; i++)
    {
        bsp_board_led_invert(BSP_BOARD_LED_0);
        bsp_board_led_invert(BSP_BOARD_LED_1);
        bsp_board_led_invert(BSP_BOARD_LED_2);
        bsp_board_led_invert(BSP_BOARD_LED_3);
        nrf_delay_ms(250);
    }*/

#ifdef ENABLE_USB_PRINT
    deca_usb_init();
#endif

    nrf_delay_ms(1000);        /**< small pause to startup */

  reset_DW3000(); //this will reset DW device

  if (dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf))
  {
    error_handler(1, _ERR_INIT);
  }
  int32_t dev_id = dwt_readdevid();

    while (!dwt_checkidlerc()) /* Need to make sure DW IC is in IDLE_RC before proceeding */ { };
  /* This initialization is added to avoid crashing the board when calling APIs that writes inside local data
  like setxtaltrim */
  if (dwt_initialise(DWT_DW_INIT) != DWT_SUCCESS)
  {
    error_handler(1, _ERR_INIT);
  }
 /* Enabling LEDs here for debug so that for each TX the D1 LED will flash on DW3000 red eval-shield boards. */
    dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

    /* Configure DW IC. See NOTE 5 below. */
    /* if the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device */
    if (dwt_configure(&config))
    {
        //test_run_info((unsigned char *)"CONFIG FAILED     ");
        //while (1) { };
    }

    /* Configure the TX spectrum parameters (power PG delay and PG Count) */
    dwt_configuretxrf(&txconfig_options_ch9);
    while(1){
        dwt_while();}
    #endif
    #if 1
  /* initialize inter-task communication mail queue for Node :
   *
   * The RxTask need to send the rxPckt to the CalcTask.
   *
   * TODO: rxPcktPool_q_id should be a part of NodeInfo, but
   * FreeRTOS cannot free resources efficiently on task deletion.
   *
   * Current code has an implementation where NodeInfo is statically defined
   * and rxPcktPool_q is a part of FreeRtos Heap.
   *
   * Note, the debug accumulator & diagnostics readings are a part of
   * mail queue. Every rx_mail_t has a size of ~6kB.
   *
   * */
   #if 0
    osMailQDef(rxPcktPool_q, RX_MAIL_QUEUE_SIZE, rx_mail_t);
    app.rxPcktPool_q_id = osMailCreate(osMailQ(rxPcktPool_q), NULL);

    if(!app.rxPcktPool_q_id)
    {
        error_handler(1, _ERR_Cannot_Alloc_Mail);
    }

    /* Create the thread(s) */
    /* definition and creation of defaultTask */
    /* Note. The DefaultTask is responsible for starting & stopping of TOP Level applications. */
  osThreadDef(defaultTask, StartDefaultTask, PRIO_StartDefaultTask, 0, configMINIMAL_STACK_SIZE*2);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* FlushTask is always working and flushing the output buffer to uart/usb */
  osThreadDef(flushTask, FlushTask, PRIO_FlushTask, 0, configMINIMAL_STACK_SIZE);
  app.flushTask.Handle = osThreadCreate(osThread(flushTask), NULL);

  /* ctrlTask is always working serving rx from uart/usb */
  //2K for CTRL task: it needs a lot of memory: it uses mallocs(512), sscanf(212bytes)
  osThreadDef(ctrlTask, CtrlTask, PRIO_CtrlTask, 0, 512);
  app.ctrlTask.Handle = osThreadCreate(osThread(ctrlTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  
  if( !defaultTaskHandle | !app.flushTask.Handle | !app.ctrlTask.Handle )
  {
      error_handler(1, _ERR_Create_Task_Bad);
  }

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  while (1)
  {
  }
  /* Loop forever sending frames periodically. */
    #endif
  #endif
}

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{
#if 0
    const EventBits_t bitsWaitForAny = (Ev_Node_Task | Ev_Tag_Task | Ev_Trilat_N_Task |
                                        Ev_Usb2spi_Task | Ev_Tcwm_Task | Ev_Tcfm_Task | Ev_Listener_Task | Ev_Stop_All);

    EventBits_t    uxBits;

    uxBits = check_the_user_button();

    xEventGroupSetBits(app.xStartTaskEvent, uxBits);

    /* Infinite loop */
    while(1)
    {
        uxBits = xEventGroupWaitBits(app.xStartTaskEvent,   bitsWaitForAny,
                                                            pdTRUE, pdFALSE,
                                                            USB_DRV_UPDATE_MS/portTICK_PERIOD_MS );

        nrf_drv_wdt_channel_feed(m_channel_id);    //WDG_Refresh

      uxBits &= bitsWaitForAny;

      if(uxBits)
      {
          app.lastErrorCode = _NO_ERR;

         /*   need to switch off DW chip's RX and IRQ before killing tasks */
          if(app.mode != mIDLE)
          {
              disable_dw3000_irq();
              reset_DW3000();
              app_apptimer_stop();                //cancel application slow timer
              dwt_setcallbacks(NULL, NULL, NULL, NULL, NULL, NULL, NULL);//DW_IRQ is disabled: safe to cancel all user call-backs
          }

          /* Event to start/stop task received */
          /* 1. free the resources: kill all user threads and timers */
          tag_terminate();
          node_terminate();
          usb2spi_terminate();
          tcfm_terminate();
          tcwm_terminate();
          trilat_terminate();
          listener_terminate();

          FlushTask_reset();

          app.lastErrorCode = _NO_ERR;

          //incoming Events are slow, and usually User-events, i.e. from a slow I/O,
          //however the Ev_Stop_All can be generated internally and may OR with other event,
          //because they can be received asynchronously.
          //Ev_Stop_All event should be tracked separately.
          app.mode = mIDLE;
          uxBits &= ~Ev_Stop_All;
      }

        nrf_drv_wdt_channel_feed(m_channel_id);    //WDG_Refresh

        osThreadYield(); //force switch of context 

        taskENTER_CRITICAL();

      /* 2. Start appropriate RTOS top-level application or run a usb_vbus_driver() if a dummy loop */
      switch (uxBits)
      {
      case Ev_Listener_Task:
          app.mode = mLISTENER;
          listener_helper(NULL); /* call Listener helper function which will setup sub-tasks for Listener process */
          break;

#if (PDOA_TAG == 1)
      /* PDoA */
      case Ev_Tag_Task:
          app.mode = mPTAG;
          tag_helper(NULL); /* call Tag helper function which will setup sub-tasks for Tag process */
          break;
#endif

#if (PDOA_NODE == 1)
      case Ev_Node_Task:
          app.mode = mPNODE;
          node_helper(NULL); /* call Node helper function which will setup sub-tasks for Node process */
          break;
#endif

#if (CFG_LE_TRILAT == 1)
      case Ev_Trilat_N_Task:
          app.mode = mTRILAT_N;
          trilat_helper(NULL); /* call Trilat helper function which will setup sub-tasks for Node process & Trilat */
          break;
#endif

      /* Service apps */
      case Ev_Usb2spi_Task:
          /* Setup a Usb2Spi task : 8K of stack is required to this task */
          app.mode = mUSB2SPI;
          usb2spi_helper(NULL);
          break;

      case Ev_Tcfm_Task:
          /* Setup a TCFM task */
          app.mode = mTCFM;
          tcfm_helper(NULL);    /* call tcfm helper function which will setup sub-tasks for Power test */
          break;

      case Ev_Tcwm_Task:
          /* Setup a TCWM task */
          app.mode = mTCWM;
          tcwm_helper(NULL);    /* call tcwm helper function which will setup sub-tasks for Power test */
          break;

      case Ev_Stop_All:
          app.mode = mIDLE;
        break;

        default:
            //usb_vbus_driver();    
            /**< connection / disconnection of USB interface :
            on connection activate flush_task & usb_rx_process  */
            nrf_drv_wdt_channel_feed(m_channel_id);    //WDG_Refresh
            break;
        }
        taskEXIT_CRITICAL();    //ready to switch to a created task

        osThreadYield();
    }
    #endif
    /*while(1){
    dwt_while();}*/

}

/** @} */