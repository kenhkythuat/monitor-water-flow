#pragma once
#include "gateway_state.h"

void gateway_core_init(void);

void gateway_set_state(gateway_state_t s);
gateway_state_t gateway_get_state(void);
const char *gateway_state_str(gateway_state_t s);
