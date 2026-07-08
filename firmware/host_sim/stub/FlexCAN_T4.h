// Minimal FlexCAN_T4 stub for host-side simulation of ak10_mit.h.
// read()/write() delegate to hooks the test installs, so the test can model
// the motors on the other end of the bus.
#ifndef FLEXCAN_T4_STUB_H
#define FLEXCAN_T4_STUB_H
#include <cstdint>

enum CAN_DEV_TABLE { CAN1, CAN2, CAN3 };
enum FLEXCAN_RXQUEUE_TABLE { RX_SIZE_256 = 256 };
enum FLEXCAN_TXQUEUE_TABLE { TX_SIZE_16 = 16 };

struct CAN_message_t
{
    uint32_t id = 0;
    uint8_t len = 8;
    uint8_t buf[8] = {0};
    struct { bool extended = false; } flags;
};

extern bool (*flexcan_read_hook)(CAN_message_t &);
extern void (*flexcan_write_hook)(const CAN_message_t &);

template <CAN_DEV_TABLE Bus, FLEXCAN_RXQUEUE_TABLE Rx, FLEXCAN_TXQUEUE_TABLE Tx>
class FlexCAN_T4
{
public:
    void begin() {}
    void setBaudRate(uint32_t) {}
    void setMaxMB(int) {}
    void enableFIFO() {}
    bool read(CAN_message_t &msg) { return flexcan_read_hook && flexcan_read_hook(msg); }
    bool write(const CAN_message_t &msg)
    {
        if (flexcan_write_hook) flexcan_write_hook(msg);
        return true;
    }
};

#endif
