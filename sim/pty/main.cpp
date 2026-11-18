// Runs the logic analyzer simulation behind a pseudo terminal so that
// sigrok-cli / PulseView ("ols" driver) can talk to it.
//   ./Vvtop <symlink-path> [trigger_cycle]
// Probes are driven with a counter that increments every clock (bits 0-14);
// probe 15 goes high at trigger_cycle (if given).
#include "Vvtop.h"
#include "verilated.h"
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>
#include <deque>

static const int CPB = 8;    // clocks per UART bit (100 MHz / 12.5 Mbaud)

int main(int argc, char** argv) {
    if (argc < 2) { fprintf(stderr, "usage: %s <link> [trigger_cycle]\n", argv[0]); return 1; }
    long long trig_cycle = argc > 2 ? atoll(argv[2]) : -1;

    int m = posix_openpt(O_RDWR | O_NOCTTY);
    grantpt(m); unlockpt(m);
    struct termios t; tcgetattr(m, &t); cfmakeraw(&t); tcsetattr(m, TCSANOW, &t);
    unlink(argv[1]);
    if (symlink(ptsname(m), argv[1])) { perror("symlink"); return 1; }
    // keep a slave fd open so the master doesn't see EOF/EIO between opens
    int keep = open(ptsname(m), O_RDWR | O_NOCTTY);
    fprintf(stderr, "[sim] serial port: %s -> %s\n", argv[1], ptsname(m));
    int fl = fcntl(m, F_GETFL); fcntl(m, F_SETFL, fl | O_NONBLOCK);

    auto* ctx = new VerilatedContext; ctx->commandArgs(argc, argv);
    auto* top = new Vvtop{ctx};
    top->uart_rx = 1; top->probes = 0;

    std::deque<uint8_t> inq;
    int tx_state = -1, tx_cnt = 0; uint16_t tx_shift = 0;     // host -> FPGA
    int rx_state = -1, rx_cnt = 0; uint8_t rx_byte = 0;       // FPGA -> host
    long long cyc = 0; int prev_clk = 0;
    unsigned long bytes_out = 0;

    while (!ctx->gotFinish()) {
        top->eval();
        if (top->clk && !prev_clk) {           // rising edge
            cyc++;
            uint16_t p = (uint16_t)(cyc & 0x7FFF);
            if (trig_cycle >= 0 && cyc >= trig_cycle) p |= 0x8000;
            top->probes = p;

            if ((cyc & 255) == 0 && inq.size() < 16) {
                uint8_t buf[64];
                ssize_t n = read(m, buf, sizeof buf);
                for (ssize_t i = 0; i < n; i++) inq.push_back(buf[i]);
            }
            // UART towards the FPGA
            if (tx_state < 0 && !inq.empty()) {
                tx_shift = (uint16_t)((1 << 9) | (inq.front() << 1));
                inq.pop_front(); tx_state = 0; tx_cnt = 0;
            }
            if (tx_state >= 0) {
                top->uart_rx = (tx_shift >> tx_state) & 1;
                if (++tx_cnt == CPB) { tx_cnt = 0; if (++tx_state == 10) tx_state = -1; }
            }
            // UART from the FPGA
            int line = top->uart_tx;
            if (rx_state < 0) {
                if (!line) { rx_state = 0; rx_cnt = CPB + CPB / 2; rx_byte = 0; }
            } else if (--rx_cnt == 0) {
                if (rx_state < 8) { rx_byte |= line << rx_state; rx_state++; rx_cnt = CPB; }
                else {
                    // blocking write: simulated time waits for the host
                    fcntl(m, F_SETFL, fl);
                    if (write(m, &rx_byte, 1) != 1) perror("write");
                    fcntl(m, F_SETFL, fl | O_NONBLOCK);
                    bytes_out++;
                    rx_state = -1;
                }
            }
        }
        prev_clk = top->clk;
        if (!top->eventsPending()) break;
        ctx->time(top->nextTimeSlot());
    }
    close(keep);
    delete top; delete ctx;
    return 0;
}
