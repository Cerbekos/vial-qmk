#include QMK_KEYBOARD_H
#include "keymap_japanese.h"
#include "quantum.h"  

/* keymap */
#define MC_TAB MT(MOD_LCTL,KC_TAB)
#define MS_Z MT(MOD_LSFT,KC_Z)
#define MC_MINS MT(MOD_LCTL,KC_MINS)
#define MS_P4 MT(MOD_LSFT,KC_P4)
#define MA_CAPS MT(MOD_LALT,KC_CAPS)

#define L1_SPC LT(1,KC_SPC)
#define L2_ENT LT(2,KC_ENT)
#define L4_SPC LT(4,KC_SPC)
#define L5_ENT LT(5,KC_ENT)
#define L7_SPC LT(7,KC_SPC)
#define L8_ENT LT(8,KC_ENT)
#define L10_SPC LT(10,KC_SPC)
#define L11_ENT LT(11,KC_ENT)
#define L12_NUM LT(12,KC_NUM)

#define S_SLSH MT(MOD_RSFT,KC_SLSH)
#define S_SCLN S(KC_SCLN)

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
    [0] = LAYOUT(
        KC_ESC,  KC_Q,    KC_W,    KC_E,    KC_R,    KC_T,    KC_QUOT, KC_Y,    KC_U,    KC_I,    KC_O,    KC_P,    KC_BSPC,
        MC_TAB,  KC_A,    KC_S,    KC_D,    KC_F,    KC_G,    KC_LBRC, KC_H,    KC_J,    KC_K,    KC_L,    MC_MINS, KC_ENT,
        KC_LSFT, KC_Z,    KC_X,    KC_C,    KC_V,    KC_B,    KC_RBRC, KC_N,    KC_M,    TD(1),   TD(2),KC_SLSH, KC_RSFT,
        KC_LCTL, KC_LGUI, KC_LALT, MA_CAPS, L1_SPC,  KC_DEL,  MO(12),  KC_BSPC, L2_ENT,  KC_LEFT, KC_DOWN, KC_UP,   KC_RGHT
    ),
    [1] = LAYOUT(
        _______, KC_F1,   KC_F2,   KC_F3,   KC_F4,   KC_F5,   _______, _______, KC_PGUP, KC_UP,   KC_PGDN, KC_INT3, _______,
        KC_LALT, KC_F6,   KC_F7,   KC_F8,   KC_F9,   KC_F10,  _______, KC_HOME, KC_LEFT, KC_DOWN, KC_RGHT, KC_BSPC, _______,
        _______, _______, _______, KC_PSCR, KC_SCRL, KC_PAUS, _______, KC_END,  _______, KC_SCLN, KC_QUOT, KC_APP,  _______,
        _______, _______, _______, _______, _______, _______, _______, _______, _______, KC_HOME, KC_DOWN, KC_PGUP, KC_END
    ),
    [2] = LAYOUT(
        _______, S(KC_1), S(KC_2), S(KC_3), S(KC_4), S(KC_5), _______, S(KC_6), S(KC_7), S(KC_8), S(KC_9), S(KC_0), _______,
        _______, KC_1,    KC_2,    KC_3,    KC_4,    KC_5,    _______, KC_6,    KC_7,    KC_8,    KC_9,    KC_0,    _______,
        _______, _______, _______, _______, S_SCLN,  KC_EQL,  _______, KC_PPLS, KC_PMNS, KC_PAST, KC_PSLS, _______, _______,
        _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______
    ),
    [3] = LAYOUT(
        TD(0),   KC_W,    KC_E,    KC_R,    KC_T,    KC_Y,    KC_U,    KC_I,    KC_O,    KC_P,    KC_P7,   KC_P8,   KC_P9,
        KC_A,    KC_S,    KC_D,    KC_F,    KC_G,    KC_H,    KC_J,    KC_K,    KC_L,    MC_MINS, MS_P4,   KC_P5,   KC_P6,   
        MS_Z,    KC_X,    KC_C,    KC_V,    KC_B,    KC_N,    KC_M,    TD(1),   TD(2),KC_UP,   KC_P1,   KC_P2,   KC_P3,
        KC_LCTL, KC_LGUI, MA_CAPS, L4_SPC,  KC_DEL,  KC_BSPC, L5_ENT,  L12_NUM, KC_LEFT, KC_DOWN, KC_RGHT, KC_P0,   KC_PDOT
    ),
    [4] = LAYOUT(
        KC_F1,   KC_F2,   KC_F3,   KC_F4,   KC_F5,   _______, KC_PGUP, KC_UP,   KC_PGDN, KC_INT3, KC_LBRC, KC_RBRC, _______,
        KC_F6,   KC_F7,   KC_F8,   KC_F9,   KC_F10,  KC_HOME, KC_LEFT, KC_DOWN, KC_RGHT, KC_BSPC, _______, _______, _______,
        KC_TAB,  _______, KC_PSCR, KC_SCRL, KC_PAUS, KC_END,  _______, KC_SCLN, KC_QUOT, S_SLSH,  _______, _______, _______,
        _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______
    ),
    [5] = LAYOUT(
        S(KC_1), S(KC_2), S(KC_3), S(KC_4), S(KC_5),  S(KC_6), S(KC_7), S(KC_8), S(KC_9), S(KC_0), _______, _______, _______,
        KC_1,    KC_2,    KC_3,    KC_4,    KC_5,     KC_6,    KC_7,    KC_8,    KC_9,    KC_0,    _______, _______, _______,
        _______, _______, _______, S_SCLN,  KC_EQL,   KC_PPLS, KC_PMNS, KC_PAST, KC_PSLS, _______, _______, _______, _______,
        _______, _______, _______, _______, _______,  _______, _______, _______, _______, _______, _______, _______, _______
    ),
    [6] = LAYOUT(
        TD(0),   KC_W,    KC_E,    KC_R,    KC_T,    KC_P7,   KC_P8,   KC_P9,   KC_Y,    KC_U,    KC_I,    KC_O,    KC_P,
        KC_A,    KC_S,    KC_D,    KC_F,    KC_G,    KC_P4,   KC_P5,   KC_P6,   KC_H,    KC_J,    KC_K,    KC_L,    MC_MINS,
        MS_Z,    KC_X,    KC_C,    KC_V,    KC_B,    KC_P1,   KC_P2,   KC_P3,   KC_N,    KC_M,    TD(1),   TD(2),S_SLSH,
        KC_LCTL, KC_LGUI, MA_CAPS, L7_SPC,  KC_DEL,  KC_P0,   KC_PDOT, L12_NUM, KC_BSPC, L8_ENT,  KC_TAB,  KC_APP,  KC_RCTL
    ),
    [7] = LAYOUT(
        KC_F1,   KC_F2,   KC_F3,   KC_F4,   KC_F5,   _______, _______, _______, _______, KC_PGUP, KC_UP,   KC_PGDN, KC_INT3,
        KC_F6,   KC_F7,   KC_F8,   KC_F9,   KC_F10,  KC_LBRC, _______, KC_RBRC, KC_HOME, KC_LEFT, KC_DOWN, KC_RGHT, KC_BSPC,
        _______, _______, KC_PSCR, KC_SCRL, KC_PAUS, _______, _______, _______, KC_END,  _______, KC_SCLN, KC_QUOT, KC_APP,
        _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______
    ),
    [8] = LAYOUT(
        S(KC_1), S(KC_2), S(KC_3), S(KC_4), S(KC_5), _______, _______, _______, S(KC_6), S(KC_7), S(KC_8), S(KC_9), S(KC_0),
        KC_1,    KC_2,    KC_3,    KC_4,    KC_5,    _______, _______, _______, KC_6,    KC_7,    KC_8,    KC_9,    KC_0,   
        _______, _______, _______, S_SCLN,  KC_EQL,  _______, _______, _______, KC_PPLS, KC_PMNS, KC_PAST, KC_PSLS, _______,
        _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______
    ),
    [9] = LAYOUT(
        KC_P7,   KC_P8,   KC_P9,   TD(0),   KC_W,    KC_E,    KC_R,    KC_T,    KC_Y,    KC_U,    KC_I,    KC_O,    KC_P,    
        KC_P4,   KC_P5,   KC_P6,   KC_A,    KC_S,    KC_D,    KC_F,    KC_G,    KC_H,    KC_J,    KC_K,    KC_L,    MC_MINS, 
        KC_P1,   KC_P2,   KC_P3,   MS_Z,    KC_X,    KC_C,    KC_V,    KC_B,    KC_N,    KC_M,    TD(1),   TD(2),S_SLSH,
        KC_P0,   KC_PDOT, L12_NUM, KC_LCTL, KC_LGUI, MA_CAPS, L10_SPC, KC_DEL,  KC_BSPC, L11_ENT, KC_TAB,  KC_APP,  KC_RCTL
    ),
    [10] = LAYOUT(
        _______, _______, _______, KC_F1,   KC_F2,   KC_F3,   KC_F4,   KC_F5,   _______, KC_PGUP, KC_UP,   KC_PGDN, KC_INT3,
        _______, _______, _______, KC_F6,   KC_F7,   KC_F8,   KC_F9,   KC_F10,  KC_HOME, KC_LEFT, KC_DOWN, KC_RGHT, KC_BSPC,
        _______, _______, _______, KC_LSFT, KC_F11,  KC_PSCR, KC_SCRL, KC_PAUS, KC_END,  _______, KC_SCLN, KC_QUOT, _______,
        _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______
    ),
    [11] = LAYOUT(
        _______, _______, _______, S(KC_1), S(KC_2), S(KC_3), S(KC_4), S(KC_5),  S(KC_6), S(KC_7), S(KC_8), S(KC_9), S(KC_0),
        _______, _______, _______, KC_1,    KC_2,    KC_3,    KC_4,    KC_5,     KC_6,    KC_7,    KC_8,    KC_9,    KC_0,   
        _______, _______, _______, _______, _______, _______, S_SCLN,  KC_EQL,   KC_PPLS, KC_PMNS, KC_PAST, KC_PSLS, _______,
        _______, _______, _______, _______, _______, _______, _______, _______,  _______, _______, _______, _______, _______
    ),
    [12] = LAYOUT(
        KC_F24,  KC_F23,  KC_F22,  KC_F21,  _______, RGB_HUI, _______, RGB_HUD, _______, _______, _______, _______, QK_BOOT,
        _______, _______, _______, _______, _______, RGB_MOD, _______, RGB_RMOD,_______, _______, _______, _______, _______,
        _______, _______, _______, _______, _______, RGB_SPI, _______, RGB_SPD, _______, _______, _______, _______, _______,
        _______, _______, _______, _______, _______, RGB_TOG, _______, _______, _______, _______, _______, _______, _______
    )
};

// TapDance
void keyboard_post_init_user(void) {
    // vial tap dance
    vial_tap_dance_entry_t td0 = { KC_Q,    KC_LCTL, KC_ESC,  KC_Q,    180}; dynamic_keymap_set_tap_dance(0, &td0);
    vial_tap_dance_entry_t td1 = { KC_COMM, KC_RCTL, KC_SCLN, KC_SCLN, 180}; dynamic_keymap_set_tap_dance(1, &td1);
    vial_tap_dance_entry_t td2 = { KC_DOT,  KC_RALT, KC_COLN, KC_COLN, 180}; dynamic_keymap_set_tap_dance(2, &td2);
    vial_tap_dance_entry_t td3 = { KC_DOT,  MO(1),   KC_COLN, KC_COLN, 180}; dynamic_keymap_set_tap_dance(3, &td3);
    vial_tap_dance_entry_t td4 = { KC_PLUS, MO(2),   KC_PAST, KC_PAST, 180}; dynamic_keymap_set_tap_dance(4, &td4);
    vial_tap_dance_entry_t td5 = { KC_PMNS, MO(3),   KC_PSLS, KC_PSLS, 180}; dynamic_keymap_set_tap_dance(5, &td5);        
};
/*
// RGBLayer setting
bool rgb_matrix_indicators_advanced_user(uint8_t led_min, uint8_t led_max) {
    for (uint8_t i = led_min; i < led_max; i++) {
        switch(get_highest_layer(layer_state)) {
            case 12: if (HAS_FLAGS(g_led_config.flags[i], 0x08)) rgb_matrix_set_color(i, RGB_MAGENTA); break;
            case 11: if (HAS_FLAGS(g_led_config.flags[i], 0x08)) rgb_matrix_set_color(i, RGB_GREEN); break;
            case 10: if (HAS_FLAGS(g_led_config.flags[i], 0x08)) rgb_matrix_set_color(i, RGB_BLUE); break;
            case 8: if (HAS_FLAGS(g_led_config.flags[i], 0x08)) rgb_matrix_set_color(i, RGB_GREEN); break;
            case 7: if (HAS_FLAGS(g_led_config.flags[i], 0x08)) rgb_matrix_set_color(i, RGB_BLUE); break;
            case 5: if (HAS_FLAGS(g_led_config.flags[i], 0x08)) rgb_matrix_set_color(i, RGB_GREEN); break;
            case 4: if (HAS_FLAGS(g_led_config.flags[i], 0x08)) rgb_matrix_set_color(i, RGB_BLUE); break;
            case 2: if (HAS_FLAGS(g_led_config.flags[i], 0x08)) rgb_matrix_set_color(i, RGB_GREEN); break;
            case 1: if (HAS_FLAGS(g_led_config.flags[i], 0x08)) rgb_matrix_set_color(i, RGB_BLUE); break;
            default: break;
        }
        if (host_keyboard_led_state().caps_lock) {
            if (HAS_FLAGS(g_led_config.flags[i], 0x08)) rgb_matrix_set_color(i, RGB_YELLOW);
        }
        if (is_caps_word_on()){
            if (HAS_FLAGS(g_led_config.flags[i], 0x08)) rgb_matrix_set_color(i, RGB_YELLOW);
        }
    }
    return false;
}
*/
// CAPS WORD
bool caps_word_press_user(uint16_t keycode) {
    switch (keycode) {
        // Keycodes that continue Caps Word, with shift applied.
        case KC_A ... KC_Z:
        case KC_MINS:
        case TD(0):
        case TD(1):
        case TD(2):
            add_weak_mods(MOD_BIT(KC_LSFT));  // Apply shift to next key.
            return true;

        // Keycodes that continue Caps Word, without shifting.
        case KC_1 ... KC_0:
        case KC_BSPC:
        case KC_DEL:
        case KC_UNDS:
            return true;

        default:
            return false;  // Deactivate Caps Word.
    }
}

// Defalut lyout change
bool process_record_user(uint16_t keycode, keyrecord_t *record) {
  switch (keycode) {
    case KC_F24:
      if (record->event.pressed) {
        return false;
      } else {
        rgb_matrix_mode(RGB_MATRIX_CUSTOM_myef_mod_func);
        set_single_persistent_default_layer(0);
        return false;
      }
    case KC_F23:
      if (record->event.pressed) {
        return false;
      } else {
        rgb_matrix_mode(RGB_MATRIX_CUSTOM_myef_right_numpad);
        set_single_persistent_default_layer(3);
        return false;
      }
    case KC_F22:
      if (record->event.pressed) {
        return false;
      } else {
        rgb_matrix_mode(RGB_MATRIX_CUSTOM_myef_center_numpad);
        set_single_persistent_default_layer(6);
        return false;
      }
    case KC_F21:
      if (record->event.pressed) {
        return false;
      } else {
        rgb_matrix_mode(RGB_MATRIX_CUSTOM_myef_left_numpad);
        set_single_persistent_default_layer(9);
        return false;
      }
    
    default:
      return true;
  }
};
