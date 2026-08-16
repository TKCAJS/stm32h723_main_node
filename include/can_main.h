#pragma once
#include <stdint.h>
#include <stdbool.h>

// FDCAN1 on PD0/PD1, 500 kbit/s classic, 29-bit extended IDs.
//
// Bit timing, filter setup and the data[0]=seq / data[1]=status payload
// convention are lifted verbatim from stm32h723_transmitter_node — same board,
// same bus, already proven against the rear node.
//
// Nothing here blocks: TX queues into the hardware FIFO and returns, RX drains
// whatever has arrived. There is no wait-for-ACK anywhere in the shift path.

bool can_init();

// Drain the RX FIFO. Call every loop.
void can_poll();

// Periodic traffic (heartbeat, RPM broadcast). Call every loop; self-rates.
void can_tick(uint16_t rpm, bool manualMode);

// --- Commands to the rear node (it owns the relays and the ignition cut) ---
void can_send_shift_up(uint16_t shiftMs, uint16_t ignCutMs, uint8_t targetGear);
void can_send_shift_down(uint16_t shiftMs, uint8_t targetGear);
void can_send_shift_stack(uint8_t targetGear);

// --- Rear node state ---
uint8_t  can_gear();            // GEAR_UNKNOWN until the rear node reports
bool     can_gear_valid();      // false once the report goes stale
uint32_t can_gear_age_ms();
uint32_t can_rx_count();
bool     can_bus_live();        // heard at least one frame — the bus is up and ACKing
