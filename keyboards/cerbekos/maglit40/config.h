// Copyright 2023 cerbekos (@cerbekos)
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

/*
 * Feature disable options
 *  These options are also useful to firmware size reduction.
 */

/* disable debug print */
//#define NO_DEBUG

/* disable print */
//#define NO_PRINT

/* disable action features */
//#define NO_ACTION_LAYER
//#define NO_ACTION_TAPPING
//#define NO_ACTION_ONESHOT
#define DYNAMIC_KEYMAP_LAYER_COUNT 6
#define MATRIX_MASKED

#define BOOTMAGIC_LITE_ROW 0
#define BOOTMAGIC_LITE_COLUMN 1
#define BOOTMAGIC_LITE_ROW_RIGHT 6
#define BOOTMAGIC_LITE_COLUMN_RIGHT 1

#define SPLIT_HAND_PIN GP3

#define SPLIT_USB_TIMEOUT 5000
#define SPLIT_USB_TIMEOUT_POLL 25
#define SPLIT_MAX_CONNECTION_ERRORS 10

#define WS2812_DI_PIN GP26