#pragma once

#if true

bool setup_gpio();

void set_gpio_to_output(int pin);
void set_gpio_to_input(int pin);
void set_gpio(int pin, int state);
int get_gpio(int pin);

#endif
