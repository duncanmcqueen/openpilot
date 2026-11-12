#pragma once

#include "selfdrive/pandad/aol_protocol.h"

// Native profiles for Mazda (safety model 13), implemented in opendbc/safety/modes/mazda.h:
// 0 = stock GEN1, 2 = GEN2 (FLAG_GEN2), 9 = GEN1 Torque Interceptor (FLAG_GEN1 | FLAG_TORQUE_INTERCEPTOR).
// Each grants lateral only.
inline bool mazda_aol_param(uint16_t param) {
  return (param == 0U) || (param == 2U) || (param == 9U);
}

inline constexpr AolSafetyProfile MAZDA_AOL_PROFILE{13U, mazda_aol_param, true};
