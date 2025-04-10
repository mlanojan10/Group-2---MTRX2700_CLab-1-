#ifndef BUTTON_H
#define BUTTON_H

#include <stdint.h>

// Function pointer type for button press callbacks
typedef void (*button_callback_t)(void);

// Call this to initialize the button interrupt, pass in your callback
void button_init(button_callback_t callback);

#endif // BUTTON_H
