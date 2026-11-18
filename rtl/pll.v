// 27 MHz -> 100.286 MHz sample/system clock for the Tang Nano 20K (GW2AR-18C).
//
// libsigrok's OLS driver assumes a 100 MHz base clock (samplerate =
// 100 MHz / (divider + 1)). An exact 100 MHz is not reachable from 27 MHz
// with a phase-detector frequency >= 3 MHz; the closest valid setting
// (computed with apycula's gowin_pll) is 27 * 26 / 7 = 100.286 MHz, i.e. all
// time bases are 0.29% fast - irrelevant for protocol decoding.
//   PFD = 27 / (IDIV_SEL+1) = 3.857 MHz
//   CLKOUT = PFD * (FBDIV_SEL+1) = 100.286 MHz
//   VCO = CLKOUT * ODIV_SEL = 802.3 MHz (500..1250 MHz)
module pll (
    input  wire clkin,
    output wire clkout,
    output wire clkoutp,     // same frequency, 180 degrees (SDRAM clock)
    output wire lock
);
    rPLL #(
        .FCLKIN("27"),
        .IDIV_SEL(6),
        .FBDIV_SEL(25),
        .ODIV_SEL(8),
        .DYN_IDIV_SEL("false"),
        .DYN_FBDIV_SEL("false"),
        .DYN_ODIV_SEL("false"),
        .DYN_SDIV_SEL(2),
        .DYN_DA_EN("false"),
        .PSDA_SEL("1000"),         // CLKOUTP phase: 8 x 22.5 = 180 degrees,
        .DUTYDA_SEL("1000"),
        .CLKFB_SEL("internal"),
        .CLKOUT_BYPASS("false"),
        .CLKOUTP_BYPASS("false"),
        .CLKOUTD_BYPASS("false"),
        .CLKOUTD_SRC("CLKOUT"),
        .CLKOUTD3_SRC("CLKOUT"),
        .DEVICE("GW2AR-18C")
    ) u_pll (
        .CLKIN(clkin),
        .CLKFB(1'b0),
        .RESET(1'b0),
        .RESET_P(1'b0),
        .FBDSEL(6'b0),
        .IDSEL(6'b0),
        .ODSEL(6'b0),
        .PSDA(4'b0),
        .DUTYDA(4'b0),
        .FDLY(4'b0),
        .CLKOUT(clkout),
        .LOCK(lock),
        .CLKOUTP(clkoutp),
        .CLKOUTD(),
        .CLKOUTD3()
    );
endmodule
