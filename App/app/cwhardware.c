 /* Copyright 2026 NR7Y
 * https://github.com/briand
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 *     Unless required by applicable law or agreed to in writing, software
 *     distributed under the License is distributed on an "AS IS" BASIS,
 *     WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *     See the License for the specific language governing permissions and
 *     limitations under the License.
 */

 // Hardware input helpers for CW keyer (port config, debounced reads, etc.)
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "misc.h"
#include "app/cwhardware.h"
#include "settings.h"
#include "py32f071_ll_dma.h"
#include "py32f071_ll_gpio.h"
#include "py32f071_ll_bus.h"
#include "py32f071_ll_rcc.h"
#include "py32f071_ll_usart.h"
#include "py32f071_ll_adc.h"
#include "driver/gpio.h"
#include "driver/systick.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "driver/adc.h"
#include "driver/millis.h"
#include "external/printf/printf.h"

// Local debug toggle for CW hardware reads
#ifndef ENABLE_CW_HARDWARE_DEBUG
#define ENABLE_CW_HARDWARE_DEBUG 0
#endif

#define ENABLE_CEC_KEYER_DEBUG 0

// Local state for last sampled paddles (edge detection)
static bool     s_last_dit   = false;
static bool     s_last_dah   = false;
static uint32_t s_dit_count  = 0;  // consecutive raw-true reads for dit
static uint32_t s_dah_count  = 0;  // consecutive raw-true reads for dah

// Read button ring input (SIDE1)
static void CW_ReadSideButton(bool *ring_out)
{
#if ENABLE_CW_HARDWARE_DEBUG
    char dbg_buf[80];
#endif

    // The keyboard matrix lives on GPIOB (columns on PB3..PB6, rows on PB12..PB15).
    // KEY_SIDE1 is in the "zero" column, row 0 => PB15.
    const uint32_t cols_mask = LL_GPIO_PIN_3 | LL_GPIO_PIN_4 | LL_GPIO_PIN_5 | LL_GPIO_PIN_6; // PB3..PB6
    const uint32_t side1_row  = LL_GPIO_PIN_15; // PB15

    // Drive columns high (same as keyboard scan initial state)
    LL_GPIO_SetOutputPin(GPIOB, cols_mask);

    bool ring = false;
    uint32_t reg = 0, reg2 = 0;
    uint32_t match_count = 0;

    // Debounce: take several samples with short delays and require consecutive matching reads
    for (unsigned int k = 0; k < 8; k++) {
        SYSTICK_DelayUs(10);
        reg2 = LL_GPIO_ReadInputPort(GPIOB) & side1_row;

        if (reg2 != reg) {
            match_count = 0;
            reg = reg2;
        } else {
            match_count++;
        }

        if (match_count >= 2) {
            break;
        }
    }

    if (match_count >= 2) {
        // Stable reading achieved - active low when pressed
        ring = !(reg);
    }

    static bool last_reported = false;
    if (ring != last_reported) {
#if ENABLE_CW_HARDWARE_DEBUG
        sprintf_(dbg_buf, "CW_ReadSideButton: stable=%u reg=0x%08X match=%u ring=%u\r\n", (unsigned)(match_count>=2), (unsigned)reg, (unsigned)match_count, (unsigned)ring);
        UART_Send(dbg_buf, strlen(dbg_buf));
#endif
        last_reported = ring;
    }

    // Cleanup: leave columns high as keyboard does
    LL_GPIO_SetOutputPin(GPIOB, cols_mask);

    *ring_out = ring;
}


// Generic GPIO deglitch function - reads with de-noise
// Returns true if pin is active (low), false if inactive (high)
static bool CW_ReadGpioDeglitched(GPIO_TypeDef *gpio_port, uint32_t pin_mask, bool heavy)
{
    bool result = false;
    uint16_t reg = 0, reg2;
    unsigned int i, k;
    uint32_t limit = heavy ? 500 : 100; // more samples for heavy de-noise
    uint32_t goal = heavy ? 300 : 60;  // need this many stable samples

    for (i = 0, k = 0, reg = 0; i < goal && k < limit; i++, k++) {
        SYSTICK_DelayUs(1);
        // Read using LL helper: returns non-zero if pin input is set
        reg2 = LL_GPIO_IsInputPinSet(gpio_port, pin_mask) ? pin_mask : 0;
        i *= (reg == reg2);  // Reset i if readings differ
        reg = reg2;
    }

    if (i >= goal) {
        // Stable reading achieved - active low
        result = !reg;
    }

    return result;
}

// Read the PTT/tip with de-noise
static void CW_ReadPtt(bool *ptt_out)
{
    // TODO: add back the de-glitch routine
    //*ptt_out = GPIO_IsPttPressed();
    *ptt_out = CW_ReadGpioDeglitched(GPIOB, LL_GPIO_PIN_10, false);

}

// static uint16_t ReadCH3()
// {
//     // OLD SINGLE SAMPLE CODE (keep for reference):
//     // Trigger ADC conversion
//     (*(volatile uint32_t *)0x400BA004U) = 0x1U;  // SARADC_START
    
//     // Wait for CH3 end of conversion (0x400BA028 = CH0 + 3*sizeof(ADC_Channel_t))
//     while (!(*(volatile uint32_t *)0x400BA028U & 0x1)) {}
    
//     // Clear interrupt flag for CH3
//     (*(volatile uint32_t *)0x400BA00CU) = (1U << 3);
    
//     // 12-bit data (0x400BA02C = CH3 DATA register)
//     return (uint16_t)((*(volatile uint32_t *)0x400BA02CU) & 0xFFFU);
// }

// Read raw paddle inputs for a specific mode
// Returns true if mode is valid, false otherwise
bool CW_ReadKeysForMode(uint8_t mode, bool *dit_out, bool *dah_out)
{
    // Check if keyer is disabled (handkey modes)
    if (mode & CW_KEY_FLAG_NO_KEYER && !(mode & CW_KEY_FLAG_PORT_GROUND)) {
        return false;
    }

    // Read PTT (PC5) as ring/dah - shared across button and port configs
    bool hw_ring = false;
    bool hw_tip = false;
    CW_ReadPtt(&hw_ring);

    // Read button ring input if enabled
    if (mode & CW_KEY_FLAG_SIDE1) {
        CW_ReadSideButton(&hw_tip);
    }
    
    // Read port ring input if enabled and OR with button ring.
    // Short-circuit: if SD1 already resolved true, skip the expensive heavy deglitch —
    // the OR result is the same and we avoid ~500us of sampling overhead on every poll.
    if ((mode & CW_KEY_FLAG_PORT_RING) && !hw_ring) {
        // New hardware: port-ring is on PA13 (SWDIO when not used). Use deglitch helper on GPIOA bit 13.
        hw_ring = CW_ReadGpioDeglitched(GPIOA, LL_GPIO_PIN_13, true);
    }
    
    // Determine if keys are reversed
    bool reverse = (mode & CW_KEY_FLAG_REVERSED);

    // Map tip/ring to dit/dah based on reversed flag
    *dit_out = reverse ? hw_ring : hw_tip;
    *dah_out = reverse ? hw_tip : hw_ring;

    return true;
}

// Read GPIO inputs based on configured mode
void CW_ReadKeys(CW_Input *in)
{
    bool n_dit = false;
    bool n_dah = false;

    // Read inputs using helper function
    if (!CW_ReadKeysForMode(gEeprom.CW_KEY_INPUT, &n_dit, &n_dah)) {
        // Handkey mode or invalid - no keyer input
        n_dit = false;
        n_dah = false;
    }

    // Three-strike debounce: increment counter while raw line is active, reset on inactive.
    if (n_dit) s_dit_count++; else s_dit_count = 0;
    if (n_dah) s_dah_count++; else s_dah_count = 0;

    // Debounced state: line is considered active only after 3 consecutive hits.
    bool deb_dit = (s_dit_count >= 3);
    bool deb_dah = (s_dah_count >= 3);

    // Edges computed against previous debounced state.
    in->dit_rise = (!s_last_dit && deb_dit);
    in->dah_rise = (!s_last_dah && deb_dah);
    in->dit = deb_dit;
    in->dah = deb_dah;

    s_last_dit = deb_dit;
    s_last_dah = deb_dah;
}

// Configure port ground pin (PA10) for tip/ring paddle input
// When enabled: PA10 becomes GPIO output low (acts as ground for paddle port)
// When disabled: restore UART1 RX functionality (call UART_Init to reconfigure)
void CW_ConfigurePortGround(bool enable)
{
    // Use LL drivers to reconfigure PA10 (USART1 RX on AF1) to GPIO output low
    if (enable) {
        // Disable USART1 so the pin can be used as GPIO
        LL_USART_Disable(USART1);

        // Disable RX DMA channel used by USART1 receive (configured in UART_Init)
        LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);

        // Ensure GPIOA clock is enabled then configure PA10 as push-pull output and drive low
        //LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
        LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_10, LL_GPIO_MODE_OUTPUT);
        LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_10, LL_GPIO_OUTPUT_PUSHPULL);
        LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_10, LL_GPIO_PULL_DOWN);
        LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_10); // drive low (ground)
    } else {
        // Restore UART configuration which will reassign PA10 to AF1 (USART1_RX)
        UART_Init();
    }
#if ENABLE_KEYER_DEBUG
    char buf[50];
    sprintf_(buf, "Port Ground %s\r\n", enable ? "Enabled" : "Disabled");
    UART_Send(buf, strlen(buf));
#endif
}

// FM Radio is disabled on this firmware, we *always* configure
// PB15 as an input, because the radio might have the line reworked
// onto the mic input, and we don't want to affect that.
void CW_ConfigurePortRing(bool enable)
{
    // On new hardware the port-ring signal is on PA13 (shared with SWDIO).
    // When enabling we configure PA13 as a GPIO input with pull-up so
    // it can be sampled. When disabling we leave PA13 alone so the
    // debugger (SWD) continues to work — do not enable port-ring if
    // you need SWD.
    if (enable) {
        LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
        LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_13, LL_GPIO_MODE_INPUT);
        LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_13, LL_GPIO_PULL_UP);
    } else {
        // Intentionally do nothing — leave PA13 as SWDIO/default
    }
#if ENABLE_KEYER_DEBUG
    char buf[50];
    sprintf_(buf, "Port Ring %s\r\n", enable ? "Enabled" : "Disabled");
    UART_Send(buf, strlen(buf));
#endif
}

// Reset sampled key states (used from keyer init)
void CW_HW_ResetKeySamples(void)
{
    s_last_dit = false;
    s_last_dah = false;
}


