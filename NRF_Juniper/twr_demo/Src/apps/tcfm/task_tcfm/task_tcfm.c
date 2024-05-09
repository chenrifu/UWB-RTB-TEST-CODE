/*
 * @file      task_tcfm.c
 *
 * @brief     Task for an extended TCFM application
 *
 * @author    Decawave
 *
 * @attention Copyright 2017-2020 (c) Decawave Ltd, Dublin, Ireland.
 *            All rights reserved.
 *
 */

#include "task_tcfm.h"

#include "port.h"
#include "tcfm.h"
#include "deca_dbg.h"


#if (DEBUG)
#define FTCFM_PRINTF diag_printf
#else
#define FTCFM_PRINTF(...) {}
#endif

//-----------------------------------------------------------------------------

/*
 * @brief TCFMTask
 *
 * */
static void
tcfmTask(void const * argument)
{
    osMutexDef(tcfmMutex);
    app.tcfmTask.MutexId = osMutexCreate(osMutex(tcfmMutex));

    FTCFM_PRINTF("tcfmTask: irq enabled %d %d %d\n", app.tcfm_info.nframes, app.tcfm_info.period_ms, app.tcfm_info.bytes);

    taskENTER_CRITICAL();

    enable_dw3000_irq();    /**< IRQ is enabled and we can receive TX IRQ immediately after this point */

    dwt_starttx(DWT_START_TX_IMMEDIATE);    /**< First frame is sent immediately */

    taskEXIT_CRITICAL();

    while (1)
    {
        osSignalWait(app.tcfmTask.Signal, osWaitForever);
        osMutexWait(app.tcfmTask.MutexId, 0);

        tcfm_process_run();

        osMutexRelease(app.tcfmTask.MutexId);
        osThreadYield();
    }
}


/* @brief Terminate all tcfmTask related functionality, if any.
 *        DW1000's RX and IRQ shall be switched off before task termination,
 *        that IRQ will not produce unexpected Signal
 * */
void
tcfm_terminate(void)
{
    tcfm_process_terminate();

    TERMINATE_STD_TASK(app.tcfmTask);
}


/* @fn         tcfm_helper
 * @brief      this is a service function which starts the
 *             TX/TCFM applicaiton
 * @param      argument is a pointer to the tcfm_info_t structure
 *
 * */
void tcfm_helper(void const *argument)
{
    error_e ret;
    port_disable_dw_irq_and_reset(1);

    ret = tcfm_process_init((tcfm_info_t*)&app.tcfm_info);

    if(ret != _NO_ERR)
    {
        error_handler(1, ret);

    }

    set_dw_spi_fast_rate();

    /* "RTOS-based" : setup (not start) all necessary tasks for the power test operation. */
    osThreadDef(TCFMf, tcfmTask, PRIO_TcfmTask, 0, configMINIMAL_STACK_SIZE *4);
    app.tcfmTask.Handle = osThreadCreate (osThread(TCFMf), NULL);

}
