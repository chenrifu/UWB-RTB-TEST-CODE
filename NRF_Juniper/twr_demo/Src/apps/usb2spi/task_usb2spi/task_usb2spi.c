/*
 * @file      task_usb2spi.c
 *
 * @brief     usb2spi implementation
 *
 * @author    Decawave
 *
 * @attention Copyright 2017-2019 (c) Decawave Ltd, Dublin, Ireland.
 *            All rights reserved.
 *
 */

#include "task_usb2spi.h"

#include "port.h"
#include "usb_uart_rx.h"
#include "usb2spi.h"
#include "error.h"
#include "deca_dbg.h"


//-----------------------------------------------------------------------------
/* @fn        Usb2SpiTask
 * @brief     this starts the usb2spi functionality.
 *
 *            Note: Previous tasks which can call shared resources must be killed.
 *            This task needs the RAM size of at least usb2spi_t
 *
 * */
void Usb2SpiTask(void const *argument)
{
    osMutexDef(u2sMutex);
    app.usb2spiTask.MutexId = osMutexCreate(osMutex(u2sMutex));

    while(1)
    {
        osMutexRelease(app.usb2spiTask.MutexId);

        osSignalWait(app.usb2spiTask.Signal , osWaitForever);

        osMutexWait(app.usb2spiTask.MutexId, 0);

        usb2spi_process_run();    //app.local_buff has a Usb2Spi protocol sequence
    }
}


/* @brief
 * Kill all tasks and timers related to usb2spi if any
 *
 * */
void usb2spi_terminate(void)
{
    TERMINATE_STD_TASK(app.usb2spiTask);

    usb2spi_process_terminate();
}



/* @fn         usb2spi_helper
 * @brief      this is a service function which starts the
 *             Usb2Spi test functionality
 * */
void usb2spi_helper(void const * arg)
{
    error_e        tmp;

    port_disable_dw_irq_and_reset(0);

    osThreadDef(u2sTask, Usb2SpiTask, PRIO_Usb2SpiTask, 0, 4096);
    app.usb2spiTask.Handle = osThreadCreate(osThread(u2sTask), (void*)arg);

    if(!app.usb2spiTask.Handle)
    {
        error_handler(1, _ERR_Create_Task_Bad);
    }

    //set_dw_spi_fast_rate();

    tmp = usb2spi_process_init();

    if(tmp != _NO_ERR)
    {
        error_handler(1, tmp);
    }

    diag_printf("usb2spi: irq enabled\r\n");
    UNUSED(arg);
}
