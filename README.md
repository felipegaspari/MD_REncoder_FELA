# MD_REncoder_fela - Rotary Encoder Library (FELA fork)

Fork of MajicDesigns [MD_REncoder](https://github.com/MajicDesigns/MD_REncoder) for DCO Input: A/B bits come from a mux snapshot instead of `digitalRead()`.

```cpp
#include <MD_REncoder_fela.h>

MD_REncoder enc(50, 50);   // dummy pins when using a mux
enc.begin();               // does not call pinMode()
uint8_t dir = enc.read(muxA, muxB);   // DIR_NONE / DIR_CW / DIR_CCW
uint16_t spd = enc.speed();           // ClickCount * (400 / period), default period 60 ms
```

Class name remains `MD_REncoder`. Arduino library name is `MD_REncoder_fela`.

## FELA changes (1.1.0)

* `read(uint8_t valueA, uint8_t valueB)` — pass mux (or GPIO) bits; decoder does not sample pins
* `begin()` does not call `pinMode` (caller / mux owns GPIO; dummy pins are safe)
* `DEFAULT_PERIOD` 60 ms; speed gain `_count * (400 / _period)` for synth-panel acceleration

## Upstream features

This is an adaptation of Ben Buxton's excellent [rotary library](http://www.buxtronix.net/2011/10/rotary-encoders-done-properly.html) and implements additional features for encoder rotation speed.

* Debounce handling with support for high rotation speeds
* Correctly handles direction changes mid-step
* Checks for valid state changes for more robust counting and noise immunity
* Interrupt based or polling in loop()
* Counts full-steps (default) or half-steps
* Calculates speed of rotation

If you like and use the original library please consider making a small donation using [PayPal](https://paypal.me/MajicDesigns/4USD)
