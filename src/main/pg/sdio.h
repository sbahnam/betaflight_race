/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "drivers/dma_reqmap.h"
#include "drivers/io_types.h"
#include "pg/pg.h"

typedef struct sdioConfig_s {
    uint8_t clockBypass;
    uint8_t useCache;
    uint8_t use4BitWidth;
    int8_t dmaopt;
    uint8_t device;
} sdioConfig_t;

PG_DECLARE(sdioConfig_t, sdioConfig);

typedef struct sdioPinConfig_s {
    ioTag_t CKPin;
    ioTag_t CMDPin;
    ioTag_t D0Pin;
    ioTag_t D1Pin;
    ioTag_t D2Pin;
    ioTag_t D3Pin;
} sdioPinConfig_t;

PG_DECLARE(sdioPinConfig_t, sdioPinConfig);
