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

#include <stddef.h>
#include <stdint.h>
#include "sdk_config.h"
#include "crc32.h"
#include "spi_flash_misc.h"


static uint32_t m_dat_addr = APP_CFG_INIT_START_ADDR;
static uint32_t m_bin_addr = APP_CFG_FW_START_ADDR;


ret_code_t spi_flash_use_image_addr_2(void)
{
    m_dat_addr = APP_CFG_INIT_START_ADDR2;
    m_bin_addr = APP_CFG_FW_START_ADDR2;

    return NRF_SUCCESS;
}

uint32_t spi_flash_get_fw_dat_addr(void)
{
    return m_dat_addr;
}

uint32_t spi_flash_get_fw_bin_addr(void)
{
    return m_bin_addr;
}

uint32_t spi_flash_verify_init(const spi_flash_init_cmd_t *p_init)
{
    uint32_t err_code;
    uint32_t crc32;

    crc32 = crc32_compute((uint8_t const *) p_init, offsetof(spi_flash_init_cmd_t, cmd_crc), NULL);

    err_code = (p_init->app_size > 0 && p_init->cmd_crc == crc32) ? 
               NRF_SUCCESS : 
               NRF_ERROR_INVALID_DATA;

    return err_code;
}
