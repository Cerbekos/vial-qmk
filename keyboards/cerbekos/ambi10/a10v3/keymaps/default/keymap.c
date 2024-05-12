#include QMK_KEYBOARD_H
#include "keymap_japanese.h"

// ***combos***//
 enum combo_events {
    KA,KI /*,KU,KE,KO,
    SA,SI,SU,SE,SO,
    TA,TI,TU,TE,TO,
    NA,NI,NU,NE,NO,
    HA,HI,HU,HE,HO,
    MA,MI,MU,ME,MO,
    YA,YU,YO
    RA,RI,RU,RE,RO,
    WA,WO,NN,
    GA,GI,GU,GE,GO,
    ZA,ZI,ZU,ZE,ZO,
    DA,DI,DU,DE,DO,
    BA,BI,BU,BE,BO,
    PA,PI,PU,PE,PO,
    KYA,KYI,KYU,KYE,KYO,
    SYA,SYI,SYU,SYE,SYO,
    TYA,TYI,TYU,TYE,TYO,
    HYA,HYI,HYU,HYE,HYO,
    MYA,MYI,MYU,MYE,MYO,
    RYA,RYI,RYU,RYE,RYO,
    GYA,GYI,GYU,GYE,GYO,
    ZYA,ZYI,ZYU,ZYE,ZYO,
    DYA,DYI,DYU,DYE,DYO,
    BYA,BYI,BYU,BYE,BYO */
};

const uint16_t PROGMEM KA[] = {KC_ESC, KC_A, COMBO_END};
const uint16_t PROGMEM KI[] = {KC_ESC, KC_I, COMBO_END};

combo_t key_combos[] = {
  [KA] = COMBO_ACTION(KA_COMBO),
  [KI] = COMBO_ACTION(KI_COMBO),
};
 
const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
  [0] = LAYOUT(
    KC_ESC,  TO(5),   TO(6),
    KC_TAB,  KC_A,    KC_U,   KC_O,
    KC_CAPS, KC_I,    KC_E,   KC_MINS,
             L2_SPC,  L1_ENT
  ),
  [1] = LAYOUT(
    KC_K,    KC_N,    KC_M, 
    KC_S,    _______, _______, _______,
    KC_T,    _______, _______, KC_H,
             KC_Y,    XXXXXXX
  ),
  [2] = LAYOUT(
    KC_G,    KC_R,    KC_P, 
    KC_Z,    _______, KC_W,    KC_Q,
    KC_D,    KC_C,    KC_X,    KC_B,
             KC_V,    XXXXXXX
  ),
  [3] = LAYOUT(
    _______, _______, _______, 
    _______, _______, _______, XXXXXXX,
    _______, RGB_HUI, RGB_HUD, RGB_TOG,
             RGB_MOD, RGB_RMOD                 
    )
};

//RGBLayer setting
const rgblight_segment_t PROGMEM my_layer1_layer[] = RGBLIGHT_LAYER_SEGMENTS({0, 15, HSV_RED});
const rgblight_segment_t PROGMEM my_layer2_layer[] = RGBLIGHT_LAYER_SEGMENTS({0, 15, HSV_BLUE});
const rgblight_segment_t PROGMEM my_layer3_layer[] = RGBLIGHT_LAYER_SEGMENTS({0, 15, HSV_GREEN});
const rgblight_segment_t PROGMEM my_layerCL_layer[] = RGBLIGHT_LAYER_SEGMENTS({0, 15, HSV_YELLOW});
const rgblight_segment_t PROGMEM my_layerNL_layer[] = RGBLIGHT_LAYER_SEGMENTS({0, 15, HSV_MAGENTA});
const rgblight_segment_t PROGMEM my_layerBlink_layer[] = RGBLIGHT_LAYER_SEGMENTS({0, 15, HSV_WHITE});

const rgblight_segment_t * const PROGMEM my_rgb_layers[] = RGBLIGHT_LAYERS_LIST(
    my_layerNL_layer,
    my_layerCL_layer,
    my_layer3_layer,
    my_layer2_layer,
    my_layer1_layer,
    my_layerBlink_layer
);

void keyboard_post_init_user(void) {
    rgblight_layers = my_rgb_layers;
};

// LayerIndicator setting
layer_state_t layer_state_set_user(layer_state_t state) {
      rgblight_set_layer_state(4, layer_state_cmp(state, 1));
      rgblight_set_layer_state(3, layer_state_cmp(state, 2));
      rgblight_set_layer_state(2, layer_state_cmp(state, 3));
    return state;
};

bool led_update_kb(led_t led_state) {
    bool res = led_update_user(led_state);
    if(res) {
        //writePin(D2, !led_state.num_lock);
        rgblight_set_layer_state(0, !led_state.num_lock);
        rgblight_set_layer_state(1, led_state.caps_lock);
    }
    return res;
};

// LED setting
bool process_record_user(uint16_t keycode, keyrecord_t *record) {
  switch (keycode) {
    // RGBLighting switching
    case KC_F24:
      if (record->event.pressed) {
        return false;
      } else {
        layer_state_set(0);
        if (rgblight_get_val() > 0) {
          // RGB OFF
          rgblight_mode_noeeprom(RGBLIGHT_MODE_STATIC_LIGHT);
          rgblight_sethsv_noeeprom(0, 0, 0);
          rgblight_blink_layer_repeat(5, 200, 2);
          return false;
        } else {
          // RGB reload from eeprom
          rgblight_reload_from_eeprom();
          rgblight_blink_layer_repeat(5, 200, 2);
          return false;
        }
      }
    default:
      return true;
    }
  };

// COMBO
void process_combo_event(uint16_t combo_index, bool pressed) {
  switch(combo_index) {
    case KA_COMBO:
      if (pressed) {
        tap_code16(KC_K);
        tap_code16(KC_A);
      }
      break;
    case KI_COMBO:
      if (pressed) {
        tap_code16(KC_K);
        tap_code16(KC_I);
      }
      break;
  }
}