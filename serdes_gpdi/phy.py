from migen import *
from migen.genlib.cdc import *



__all__ = ["LatticeECP5PCIeSERDES"]


class LatticeECP5PCIeSERDES(Module):
    """
    Lattice ECP5 DCU configured in PCIe mode. Assumes 100 MHz reference clock on SERDES clock
    input pair. Uses 1:2 gearing. Receiver Detection runs in TX clock domain. Only provides
    a single lane.

    Parameters
    ----------
    ref_clk : Signal
        100 MHz SERDES reference clock.

    rx_clk_o : Signal
        125 MHz clock recovered from received data.
    rx_clk_i : Signal
        125 MHz clock for the receive FIFO.

    tx_clk_o : Signal
        125 MHz clock generated by transmit PLL.
    tx_clk_i : Signal
        125 MHz clock for the transmit FIFO.
    """

    def __init__(self, pins):
        self.ref_clk    = Signal()

        self.tx_clk_o   = Signal()
        self.tx_clk_i   = Signal()
        self.tx_bus     = Signal(24)

        self.clock_domains.cd_tx = ClockDomain(reset_less=True)
        self.comb += self.cd_tx.clk.eq(self.tx_clk_i)

        tx_lol   = Signal()
        tx_lol_s = Signal()
        self.specials += [
            MultiReg(tx_lol, tx_lol_s, odomain="tx")
        ]

        self.specials.dcu0 = Instance("DCUA",
            #============================ DCU
            # DCU — power management
            p_D_MACROPDB            = "0b1",
            p_D_IB_PWDNB            = "0b1",    # undocumented (required for RX)
            p_D_TXPLL_PWDNB         = "0b1",
            i_D_FFC_MACROPDB        = 1,

            # DCU — reset
            i_D_FFC_MACRO_RST       = ResetSignal("phy"),
            i_D_FFC_DUAL_RST        = ResetSignal("phy"),
            i_D_FFC_TRST            = ResetSignal("phy"),

            # DCU — clocking
            i_D_REFCLKI             = self.ref_clk,
            o_D_FFS_PLOL            = tx_lol,
            p_D_BUS8BIT_SEL = 0b0,
            p_D_REFCK_MODE          = {
                25: "0b100",
                20: "0b000",
                16: "0b010",
                10: "0b001",
                 8: "0b011"}[10],
            p_D_TX_MAX_RATE         = "0.160",    # 2.5 Gbps
            p_D_TX_VCO_CK_DIV       = {
                32: "0b111",
                16: "0b110",
                 8: "0b101",
                 4: "0b100",
                 2: "0b010",
                 1: "0b000"}[1],                # DIV/1
            p_D_BITCLK_LOCAL_EN     = "0b1",    # Use local PLL clock

            # DCU ­— unknown
            p_D_CMUSETBIASI         = "0b00",   # begin undocumented (PCIe sample code used)
            p_D_CMUSETI4CPP         = "0d4",
            p_D_CMUSETI4CPZ         = "0d3",
            p_D_CMUSETI4VCO         = "0b00",
            p_D_CMUSETICP4P         = "0b01",
            p_D_CMUSETICP4Z         = "0b101",
            p_D_CMUSETINITVCT       = "0b00",
            p_D_CMUSETISCL4VCO      = "0b000",
            p_D_CMUSETP1GM          = "0b000",
            p_D_CMUSETP2AGM         = "0b000",
            p_D_CMUSETZGM           = "0b100",
            p_D_SETIRPOLY_AUX       = "0b10",
            p_D_SETICONST_AUX       = "0b01",
            p_D_SETIRPOLY_CH        = "0b10",
            p_D_SETICONST_CH        = "0b10",
            p_D_SETPLLRC            = "0d1",
            p_D_RG_EN               = "0b1",
            p_D_RG_SET              = "0b00",   # end undocumented

            # DCU — FIFOs
            p_D_LOW_MARK            = "0d4",
            p_D_HIGH_MARK           = "0d12",

            #============================ CH0 common
            # CH0 — protocol
            p_CH0_PROTOCOL          = "10BSER",
            p_CH0_PCIE_MODE         = "0b0",

            p_CH0_ENC_BYPASS        = "0b1",    # Bypass the 8b10b encoder
            p_CH0_DEC_BYPASS        = "0b1",    # Bypass the 8b10b decoder

            #============================ CH0 transmit
            # CH0 TX — power management
            p_CH0_TPWDNB            = "0b1",
            i_CH0_FFC_TXPWDNB       = 1,

            # CH0 TX ­— reset
            i_CH0_FFC_LANE_TX_RST   = 0,

            # CH0 TX ­— output
            o_CH0_HDOUTP            = pins.tx_p,
            o_CH0_HDOUTN            = pins.tx_n,

            p_CH0_TXAMPLITUDE       = "0d1000", # 1000 mV
            p_CH0_RTERM_TX          = "0d19",   # 50 Ohm

            p_CH0_TDRV_SLICE0_CUR   = "0b011",  # 400 uA
            p_CH0_TDRV_SLICE0_SEL   = "0b01",   # main data
            p_CH0_TDRV_SLICE1_CUR   = "0b000",  # 100 uA
            p_CH0_TDRV_SLICE1_SEL   = "0b00",   # power down
            p_CH0_TDRV_SLICE2_CUR   = "0b11",   # 3200 uA
            p_CH0_TDRV_SLICE2_SEL   = "0b01",   # main data
            p_CH0_TDRV_SLICE3_CUR   = "0b11",   # 3200 uA
            p_CH0_TDRV_SLICE3_SEL   = "0b01",   # main data
            p_CH0_TDRV_SLICE4_CUR   = "0b11",   # 3200 uA
            p_CH0_TDRV_SLICE4_SEL   = "0b01",   # main data
            p_CH0_TDRV_SLICE5_CUR   = "0b00",   # 800 uA
            p_CH0_TDRV_SLICE5_SEL   = "0b00",   # power down

            # CH0 TX ­— clocking
            o_CH0_FF_TX_PCLK        = self.tx_clk_o,
            i_CH0_FF_TXI_CLK        = self.tx_clk_i,

            p_CH0_TX_GEAR_MODE      = "0b0",    # 1:2 gearbox
            p_CH0_FF_TX_H_CLK_EN    = "0b1",    # enable  DIV/2 output clock
            p_CH0_FF_TX_F_CLK_DIS   = "0b1",    # disable DIV/1 output clock

            # CH0 TX — data
            **{"o_CH0_FF_TX_D_%d" % n: self.tx_bus[n] for n in range(self.tx_bus.nbits)},

            # CH0 DET
            #i_CH0_FFC_PCIE_DET_EN   = 1,
            #i_CH0_FFC_PCIE_CT       = 1,
            #o_CH0_FFS_PCIE_DONE     = pcie_done,
            #o_CH0_FFS_PCIE_CON      = pcie_con,
        )
        self.dcu0.attr.add(("LOC", "DCU0"))
        self.dcu0.attr.add(("CHAN", "CH0"))
        self.dcu0.attr.add(("BEL", "X42/Y71/DCU"))
