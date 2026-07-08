// Minimal Arduino stub for host-side simulation of ak10_mit.h
#ifndef ARDUINO_STUB_H
#define ARDUINO_STUB_H
#include <cstdint>
#include <cstdlib>

// Fake clock, controllable from the test. millis() ticks 1 ms per call so
// busy-wait loops both terminate and advance simulated time; delay() jumps.
extern uint32_t fake_now;
inline uint32_t millis() { return ++fake_now; }
inline void delay(uint32_t ms) { fake_now += ms; }

template <typename T>
T constrain(T v, T lo, T hi) { return v < lo ? lo : (v > hi ? hi : v); }

#endif
