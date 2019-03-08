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

#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "nrf_gpio.h"
#include "app_error.h"
#include "spi_flash.h"
#include "sdk_config.h"
#include "boards.h"

// local buffer sizes
#define TX_BUFF_SIZE    4

// MX25R6435F command opcodes
#define SF25_OP_READ_DATA       0x03
#define SF25_OP_WRITE_DATA      0x02
#define SF25_OP_READ_STAT       0x05
#define SF25_OP_WRITE_ENABLE    0x06
#define SF25_OP_WRITE_DISABLE   0x04
#define SF25_OP_ERASE_SECTOR    0x20
#define SF25_OP_ERASE_BLOCK     0x52
#define SF25_OP_READ_ID         0x9F
#define SF25_OP_NOP             0x00

#if defined(BOARD_PCA10056)
#define SF25_CSN_PIN            BSP_QSPI_CSN_PIN  //P0.17
#define SF25_SCK_PIN            BSP_QSPI_SCK_PIN  //P0.19
#define SF25_MOSI_PIN           BSP_QSPI_IO0_PIN  //P0.20
#define SF25_MISO_PIN           BSP_QSPI_IO1_PIN  //P0.21
#else
#define SF25_CSN_PIN            17  //P0.17
#define SF25_SCK_PIN            19  //P0.19
#define SF25_MOSI_PIN           20  //P0.20
#define SF25_MISO_PIN           21  //P0.21

#endif

//#define HIGH_SCK nrf_gpio_pin_set(SF25_SCK_PIN)
//#define LOW_SCK     nrf_gpio_pin_clear(SF25_SCK_PIN);

static bool m_is_init = false;

static uint8_t m_spi_flash_tx_buff[TX_BUFF_SIZE];

ret_code_t spi_flash_init(void)
{
    ret_code_t err_code = NRF_SUCCESS;

    if (m_is_init)
    {
        err_code = NRF_ERROR_INVALID_STATE;
    }
    else
    {
        // pull high CS pin
        nrf_gpio_pin_set(SF25_CSN_PIN);
        nrf_gpio_cfg_output(SF25_CSN_PIN);

        // initialize SPI
        nrf_gpio_pin_clear(SF25_SCK_PIN);
        nrf_gpio_cfg_output(SF25_SCK_PIN);
        nrf_gpio_pin_clear(SF25_MOSI_PIN);
        nrf_gpio_cfg_output(SF25_MOSI_PIN);
        nrf_gpio_cfg_input(SF25_MISO_PIN, NRF_GPIO_PIN_PULLDOWN);

//        err_code = NRF_SUCCESS;

        m_is_init = true;
    }

    return err_code;
}

ret_code_t spi_flash_uninit(void)
{
    ret_code_t err_code = NRF_SUCCESS;
    
    if (m_is_init)
    {
        nrf_gpio_cfg_default(SF25_SCK_PIN);
        nrf_gpio_cfg_default(SF25_MOSI_PIN);
        nrf_gpio_cfg_default(SF25_MISO_PIN);
//      nrf_gpio_cfg_default(SF25_CSN_PIN);

        m_is_init = false;

//        err_code = NRF_SUCCESS;
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
    }
    
    return err_code;    
}

static void spi_flash_write_bytes(const uint8_t *p_data, uint8_t len)
{
//     n;
    
    for (uint8_t n = 0; n < len; n++)
    {
        uint8_t val = *(p_data + n);
        // uint8_t nb;

        // ORDER = MsbFirst
        for (uint8_t nb = 0x80; nb > 0; nb >>= 1)
        {
            if (val & nb)
                nrf_gpio_pin_set(SF25_MOSI_PIN);
            else
                nrf_gpio_pin_clear(SF25_MOSI_PIN);

            __NOP();
            __NOP();

            // SPI mode 0, CPHA = Leading, CPOL = ActiveHigh
            nrf_gpio_pin_set(SF25_SCK_PIN);
            __NOP();
            __NOP();
            nrf_gpio_pin_clear(SF25_SCK_PIN);
        }
    }
}

static void spi_flash_read_bytes(uint8_t *p_data, uint32_t len)
{
    uint32_t n;
    
    for (n = 0; n < len; n++)
    {
        uint8_t val = 0;
        uint8_t nb;

        // ORDER = MsbFirst
        for (nb = 0x80; nb > 0; nb >>= 1)
        {
            __NOP();
            __NOP();

            // SPI mode 0, CPHA = Leading, CPOL = ActiveHigh
            nrf_gpio_pin_set(SF25_SCK_PIN);
            val = (val << 1) | nrf_gpio_pin_read(SF25_MISO_PIN);
//          __NOP();
//          __NOP();
            nrf_gpio_pin_clear(SF25_SCK_PIN);
        }

        *(p_data + n) = val;
    }
}

static ret_code_t spi_flash_start_op_4(uint8_t opcode, uint32_t addr)
{
    ret_code_t err_code;

    if (addr >= FLASH_SIZE_BYTE)
        err_code = NRF_ERROR_INVALID_PARAM;
    else
    {
        m_spi_flash_tx_buff[0] = opcode;
        m_spi_flash_tx_buff[1] = (uint8_t)((addr >> 16) & 0xFF);
        m_spi_flash_tx_buff[2] = (uint8_t)((addr >>  8) & 0xFF);
        m_spi_flash_tx_buff[3] = (uint8_t)((addr >>  0) & 0xFF);

        nrf_gpio_pin_clear(SF25_CSN_PIN);
        spi_flash_write_bytes(m_spi_flash_tx_buff, 4);

        err_code = NRF_SUCCESS;
    }

    return err_code;
}

ret_code_t spi_flash_read_data(uint32_t addr)
{
    if (!m_is_init)
        return NRF_ERROR_INVALID_STATE;

//    ret_code_t err_code;
//
//    // write SPI flash opcode
//    err_code = spi_flash_start_op_4(SF25_OP_READ_DATA, addr);

    return spi_flash_start_op_4(SF25_OP_READ_DATA, addr);
}

ret_code_t spi_flash_data_get(uint8_t *buff, uint32_t len)
{
    if (!m_is_init)
        return NRF_ERROR_INVALID_STATE;

    // read SPI flash data
    nrf_gpio_pin_clear(SF25_CSN_PIN);
    spi_flash_read_bytes(buff, len);

    // pull high CS pin
    nrf_gpio_pin_set(SF25_CSN_PIN);

    return NRF_SUCCESS;
}
