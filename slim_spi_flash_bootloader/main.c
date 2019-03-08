/**
 * Copyright (c) 2016 - 2018, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 *
 * @defgroup bootloader_secure_ble main.c
 * @{
 * @ingroup dfu_bootloader_api
 * @brief Bootloader project main file for secure DFU.
 *
 */

#include <stdint.h>
#include "boards.h"
#include "nrf_mbr.h"
#include "nrf_bootloader_slim_spi.h"
#include "nrf_bootloader_app_start.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "app_error.h"
#include "nrf_bootloader_info.h"
#include "nrf_delay.h"

static void on_error(void)
{
        NRF_LOG_FINAL_FLUSH();

#if NRF_MODULE_ENABLED(NRF_LOG_BACKEND_RTT)
        // To allow the buffer to be flushed by the host.
        nrf_delay_ms(100);
#endif
#ifdef NRF_DFU_DEBUG_VERSION
        NRF_BREAKPOINT_COND;
#endif
        NVIC_SystemReset();
}


void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
        NRF_LOG_ERROR("%s:%d", p_file_name, line_num);
        on_error();
}

void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
        NRF_LOG_ERROR("Received a fault! id: 0x%08x, pc: 0x%08x, info: 0x%08x", id, pc, info);
        on_error();
}

void app_error_handler_bare(uint32_t error_code)
{
        NRF_LOG_ERROR("Received an error: 0x%08x!", error_code);
        on_error();
}

/**@brief Function for application main entry. */
int main(void)
{
        uint32_t ret_val;

        (void) NRF_LOG_INIT(NULL);
        NRF_LOG_DEFAULT_BACKENDS_INIT();

        NRF_LOG_INFO("Inside main");

        ret_val = nrf_bootloader_slim_spi_init();
        APP_ERROR_CHECK(ret_val);

//        ret_val = nrf_bootloader_init(NULL);//dfu_observer);
//        APP_ERROR_CHECK(ret_val);
//
//        // Either there was no DFU functionality enabled in this project or the DFU module detected
//        // no ongoing DFU operation and found a valid main application.
//        // Boot the main application.
//        nrf_bootloader_app_start();

        // Should never be reached.
        NRF_LOG_INFO("After main");
}

/**
 * @}
 */
