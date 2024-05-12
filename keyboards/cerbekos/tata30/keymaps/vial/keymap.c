/* Copyright 2020 noroadsleft
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include QMK_KEYBOARD_H
#include "keymap_japanese.h"
#include "quantum.h"  
#include <stdio.h> 

/* keymap */
#define MS_DOT MT(MOD_RSFT,KC_DOT)
#define MC_Q MT(MOD_LCTL,KC_Q)
#define MS_Z MT(MOD_LSFT,KC_Z)
#define MA_X MT(MOD_LALT,KC_X)
#define MG_V MT(MOD_LGUI,KC_V)

#define MS_EQL MT(MOD_LSFT,KC_EQL)
#define MA_PPLS MT(MOD_LALT,KC_PPLS)
#define MG_PAST MT(MOD_LGUI,KC_PAST)

#define L1_ENT LT(1,KC_ENT)
#define L2_P LT(2,KC_P)
#define L3_L LT(3,KC_L)

#define S_SLSH S(KC_SLSH)
#define S_SCLN S(KC_SCLN)

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
    [0] = LAYOUT(
        TD(1),   KC_W,    KC_E,    KC_R,    KC_T,             KC_Y,    KC_U,    KC_I,    KC_O,    TD(4),
        KC_A,    KC_S,    KC_D,    KC_F,    KC_G,             KC_H,    KC_J,    KC_K,    L3_L,    TD(0),
        MS_Z,    MA_X,    KC_C,    MG_V,    KC_B,    KC_F24,  KC_N,    KC_M,    TD(2),   TD(3)
    ),
    [1] = LAYOUT(
        KC_1,    KC_2,    KC_3,    KC_4,    KC_5,             KC_END,  KC_PGUP, KC_UP,   KC_PGDN, _______,
        KC_6,    KC_7,    KC_8,    KC_9,    KC_0,             KC_HOME, KC_LEFT, KC_DOWN, KC_RGHT, _______,
        MS_EQL,  MA_PPLS, KC_PMNS, MG_PAST, KC_PSLS, KC_MUTE, KC_BSPC, KC_DEL,  S(KC_TAB),  KC_TAB
    ),
    [2] = LAYOUT(
        S(KC_1), S(KC_2), S(KC_3), S(KC_4), S(KC_5),          KC_SLSH, S_SLSH,  KC_INT3, _______, _______,
        S(KC_6), S(KC_7), S(KC_8), S(KC_9), S(KC_0),          KC_LBRC, KC_RBRC, KC_BSLS, _______, _______,
        KC_LSFT, KC_CAPS, _______, _______, _______, QK_BOOT, KC_APP,  KC_SLSH, _______, _______
    ),
    [3] = LAYOUT(
        KC_F1,   KC_F2,   KC_F3,   KC_F4,   KC_F5,            KC_SCLN, KC_QUOT, KC_NUHS, _______, _______,
        KC_F6,   KC_F7,   KC_F8,   KC_F9,   KC_F10,           KC_MINS, KC_TAB,  KC_ESC, _______, _______,
        KC_LSFT, KC_F11,  KC_PSCR, KC_SCRL, KC_PAUS, DT_PRNT, KC_F24,  _______, _______, _______
    )
};

// RGBLayer setting
const rgblight_segment_t PROGMEM my_layer1_layer[] = RGBLIGHT_LAYER_SEGMENTS({0, 1, HSV_RED});
const rgblight_segment_t PROGMEM my_layer2_layer[] = RGBLIGHT_LAYER_SEGMENTS({0, 1, HSV_BLUE});
const rgblight_segment_t PROGMEM my_layer3_layer[] = RGBLIGHT_LAYER_SEGMENTS({0, 1, HSV_GREEN});
const rgblight_segment_t PROGMEM my_layerCL_layer[] = RGBLIGHT_LAYER_SEGMENTS({0, 1, HSV_YELLOW});
const rgblight_segment_t PROGMEM my_layerNL_layer[] = RGBLIGHT_LAYER_SEGMENTS({0, 1, HSV_MAGENTA});
const rgblight_segment_t PROGMEM my_layerOFF_layer[] = RGBLIGHT_LAYER_SEGMENTS({0, 1, HSV_OFF});

const rgblight_segment_t * const PROGMEM my_rgb_layers[] = RGBLIGHT_LAYERS_LIST(
    my_layerNL_layer,
    my_layerCL_layer,
    my_layer3_layer,
    my_layer2_layer,
    my_layer1_layer,
    my_layerOFF_layer
);

void keyboard_post_init_user(void) {
    rgblight_layers = my_rgb_layers;
    rgblight_sethsv_noeeprom(0, 0, 0);
    
    // vial tap dance
    vial_tap_dance_entry_t td0 = { KC_SPC,  MO(1),   KC_ENT,  KC_SPC,  200}; dynamic_keymap_set_tap_dance(0, &td0);
    vial_tap_dance_entry_t td1 = { KC_Q,    KC_LCTL, KC_ESC,  KC_Q,    180}; dynamic_keymap_set_tap_dance(1, &td1);
    vial_tap_dance_entry_t td2 = { KC_COMM, KC_RCTL, KC_SCLN, KC_SCLN, 180}; dynamic_keymap_set_tap_dance(2, &td2);
    vial_tap_dance_entry_t td3 = { KC_DOT,  KC_RSFT, KC_SLSH, KC_SLSH, 180}; dynamic_keymap_set_tap_dance(3, &td3);
    vial_tap_dance_entry_t td4 = { KC_P,    MO(2),   KC_MINS, KC_MINS, 180}; dynamic_keymap_set_tap_dance(4, &td4);
};

// LayerIndicator 
layer_state_t layer_state_set_user(layer_state_t state) {
    rgblight_set_layer_state(5, layer_state_cmp(state, 0));
    rgblight_set_layer_state(4, layer_state_cmp(state, 1));
    rgblight_set_layer_state(3, layer_state_cmp(state, 2));
    rgblight_set_layer_state(2, layer_state_cmp(state, 3));

    return state;
};

// LockIndicator
bool led_update_user(led_t led_state) {
    rgblight_set_layer_state(1, led_state.caps_lock);
    return true;
};

// CAPS WORD Indicator
void caps_word_set_user(bool active) {
    if (active) {
        rgblight_set_layer_state(1, active);
    }
}

// ENCODER
#if defined(ENCODER_MAP_ENABLE)
const uint16_t PROGMEM encoder_map[][NUM_ENCODERS][2] = {
    [0] = { ENCODER_CCW_CW(KC_INT5, KC_INT4)},
    [1] = { ENCODER_CCW_CW(KC_TAB,  S(KC_TAB))},
    [2] = { ENCODER_CCW_CW(KC_VOLU, KC_VOLD)},
    [3] = { ENCODER_CCW_CW(DT_UP, DT_DOWN)},
};
#endif

// OLED
#ifdef OLED_ENABLE
oled_rotation_t oled_init_kb(oled_rotation_t rotation) {
    return OLED_ROTATION_270;
}

static const char PROGMEM tata30_logo[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0xf0, 0xf8, 0xfc, 0x0e, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0xfc, 0xd8, 0xe0, 0xf0, 0x70, 0x38, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xff, 0xff, 0xff, 0xfe, 0xf0, 0xe0, 0xc0, 
    0x80, 0x00, 0x00, 0x80, 0x40, 0x29, 0x1f, 0xff, 0x1f, 0x0f, 0x08, 0x02, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x81, 0x83, 0x8f, 0x8f, 0x1f, 0x1f, 0xbf, 
    0x3f, 0x3f, 0xbf, 0xbe, 0xbe, 0xbe, 0xbe, 0x1e, 0x1c, 0x8e, 0x06, 0x02, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x1f, 0x00, 0x00, 0x18, 0xc7, 0x01, 
    0xc7, 0x98, 0x00, 0x00, 0x9f, 0x40, 0x40, 0x98, 0x0e, 0x01, 0x07, 0x1c, 0x10, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x12, 
    0x13, 0x0c, 0x00, 0x07, 0x0f, 0x10, 0x10, 0x0f, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static unsigned int type_count = 0;
void count_type(void) {
    type_count++;
}

void oled_write_type_count(void) {
    static char type_count_str[7];
    oled_write_P(PSTR("Type:"), false);
    itoa(type_count, type_count_str, 10);
    oled_write_ln(type_count_str, false);
}

bool oled_task_user(void) {

    // lender logo
    oled_set_cursor(0, 1);
    oled_write_raw_P(tata30_logo, sizeof(tata30_logo));
    
    // Host Keyboard Layer Status
    oled_set_cursor(0, 7);
    oled_write_P(PSTR("Lay:"), false);

    switch (get_highest_layer(layer_state)) {
        case 1:
            oled_write_P(PSTR("1"), false);
            break;
        case 2:
            oled_write_P(PSTR("2"), false);
            break;
        case 3:
            oled_write_P(PSTR("3"), false);
            break;
        case 4:
            oled_write_P(PSTR("4"), false);
            break;
        case 5:
            oled_write_P(PSTR("5"), false);
            break;
        default:
            oled_write_ln_P(PSTR("0"), false);
    }

    // Host Keyboard LED Status 
    uint8_t mod_state;
    mod_state = get_mods();
    oled_set_cursor(0, 9);
    oled_write_P((mod_state & MOD_MASK_CTRL) ? PSTR("C") : PSTR(" "), false);
    oled_write_P((mod_state & MOD_MASK_SHIFT) ? PSTR("S") : PSTR(" "), false);
    oled_write_P((mod_state & MOD_MASK_ALT) ? PSTR("A") : PSTR(" "), false);
    oled_write_P((mod_state & MOD_MASK_GUI) ? PSTR("G") : PSTR(" "), false);
    oled_write_P((is_caps_word_on()) ? PSTR("W") : PSTR(" "), false);
    
    // type counat
    oled_set_cursor(0, 11);
    oled_write_type_count();
    // tapping term
    oled_set_cursor(0, 14);
    oled_write_P(PSTR("Term:"), false);
    char buf1[6];
    snprintf(buf1, 6, "%3d", g_tapping_term); 
    oled_write(buf1, false);

    return false;
}
#endif

// Keep mod
bool process_record_user(uint16_t keycode, keyrecord_t *record) {

    if (record->event.pressed) {
        count_type();

        uint8_t mod_state;
        mod_state = get_mods();
        
        switch (keycode) {
        // capsword mode switching
        case KC_F23:
            if (mod_state & MOD_MASK_CTRL) {
                del_mods(MOD_MASK_CTRL);
            } else {
                add_mods(MOD_MASK_CTRL);
            } 
            return false;
        case KC_F22:
            if (mod_state & MOD_MASK_SHIFT) {
                del_mods(MOD_MASK_SHIFT);
            } else {
                add_mods(MOD_MASK_SHIFT);
            } 
            return false;
        default:
            return true;
        }

    } else  {
        switch (keycode) {
        // capsword mode switching
        case KC_F24:
            if (is_caps_word_on()) {
                caps_word_off();
            } else {
                caps_word_on();
            }
            return false;
        default:
            return true;
        }
        
    }
}

// CAPS WORD
bool caps_word_press_user(uint16_t keycode) {
    switch (keycode) {
        // Keycodes that continue Caps Word, with shift applied.
        case KC_A ... KC_Z:
        case KC_MINS:
        case TD(0):
        case TD(1):
        case TD(2):
        case TD(3):
        case TD(4):
            add_weak_mods(MOD_BIT(KC_LSFT));  // Apply shift to next key.
            return true;

        // Keycodes that continue Caps Word, without shifting.
        case KC_1 ... KC_0:
        case KC_BSPC:
        case KC_DEL:
        case KC_UNDS:
        case KC_ESC:
            return true;

        default:
            return false;  // Deactivate Caps Word.
    }
}
