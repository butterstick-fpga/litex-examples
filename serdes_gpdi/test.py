#!/usr/bin/env python3

#
# This file is part of LiteX-Boards.
#
# Copyright (c) 2021 Florent Kermarrec <florent@enjoy-digital.fr>
# Copyright (c) 2021 Greg Davill <greg.davill@gmail.com>
# SPDX-License-Identifier: BSD-2-Clause

# Build/Use:
# ./gsd_butterstick.py --uart-name=crossover --with-etherbone --csr-csv=csr.csv --build --load
# litex_server --udp
# litex_term crossover

from signal import signal
from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

import platform as gsd_butterstick

from litex.build.lattice.trellis import trellis_args, trellis_argdict

from litex.soc.cores.clock import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *
from litex.soc.cores.led import LedChaser
from litex.soc.cores.gpio import GPIOTristate

from litedram.modules import MT41K64M16,MT41K128M16,MT41K256M16,MT41K512M16
from litedram.phy import ECP5DDRPHY

from liteeth.phy.ecp5rgmii import LiteEthPHYRGMII
from phy import LatticeECP5PCIeSERDES

# CRG ---------------------------------------------------------------------------------------------

class _CRG(Module):
    def __init__(self, platform, sys_clk_freq):
        self.rst = Signal()
        self.clock_domains.cd_init    = ClockDomain()
        self.clock_domains.cd_por     = ClockDomain()
        self.clock_domains.cd_sys     = ClockDomain()
        self.clock_domains.cd_phy     = ClockDomain()

        # # #

        self.stop  = Signal()
        self.reset = Signal()

        # Clk / Rst
        clk30 = platform.request("clk30")
        rst_n = platform.request("user_btn", 0)

        reset_soc = platform.request("fgpa_reset", 0)
        self.comb += reset_soc.eq(rst_n)

        # Power on reset
        por_count = Signal(16, reset=2**16-1)
        por_done  = Signal()
        self.comb += self.cd_por.clk.eq(clk30)
        self.comb += por_done.eq(por_count == 0)
        self.sync.por += If(~por_done, por_count.eq(por_count - 1))

        # PLL
        self.submodules.pll = pll = ECP5PLL()
        self.comb += pll.reset.eq(~por_done | ~rst_n | self.rst)
        pll.register_clkin(clk30, 30e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq)
        pll.create_clkout(self.cd_init,   25e6)
        pll.create_clkout(self.cd_phy,   8e6)
        

# BaseSoC ------------------------------------------------------------------------------------------

class BaseSoC(SoCCore):
    def __init__(self, revision="1.0", device="85F", sdram_device="MT41K64M16", sys_clk_freq=int(60e6), 
        toolchain="trellis", with_ethernet=False, with_etherbone=False, eth_ip="192.168.1.50", 
        eth_dynamic_ip   = False,
        with_spi_flash   = False,
        with_led_chaser  = True,
        with_syzygy_gpio = True,
        **kwargs)       :
        platform = gsd_butterstick.Platform(revision=revision, device=device ,toolchain=toolchain)

        # CRG --------------------------------------------------------------------------------------
        self.submodules.crg = _CRG(platform, sys_clk_freq)
        kwargs["cpu_type"] = None
        kwargs["uart_name"] = 'stub'
        SoCCore.__init__(self, platform, sys_clk_freq, ident="LiteX SoC on ButterStick", **kwargs)

        # Leds -------------------------------------------------------------------------------------
        if with_led_chaser:
            self.comb += platform.request("user_led_color").eq(0b010) # Blue.
            self.submodules.leds = LedChaser(
                pads         = platform.request_all("user_led"),
                sys_clk_freq = sys_clk_freq)

        self.submodules.phy = phy = LatticeECP5PCIeSERDES(platform.request("serdes"))

        value = Signal(10, reset=0)
        btn = self.platform.request("user_btn",1)
        btn_r = Signal()

        shift = Signal(20)

        self.comb += [
            phy.tx_bus[0:10].eq(value),
            #phy.tx_bus[12:22].eq(value),

            phy.tx_clk_i.eq(ClockSignal("phy")),
            phy.ref_clk.eq(ClockSignal("phy")),
        ]

        self.clock_domains.cd_phy_div = ClockDomain()
        self.comb += self.cd_phy_div.clk.eq(phy.tx_clk_o)

        self.sync.phy_div += [

            btn_r.eq(btn),
            If(shift > 10,
            shift.eq(0),
            value.eq(value ^ 0b0101010101),
            ).Else(

            shift.eq(shift+1),
            ),

            If(~btn & btn_r,
                value.eq(~value),
            )
            
            
        ]

# Build --------------------------------------------------------------------------------------------

def main():
    from litex.soc.integration.soc import LiteXSoCArgumentParser
    parser = LiteXSoCArgumentParser(description="LiteX SoC on ButterStick")
    target_group = parser.add_argument_group(title="Target options")
    target_group.add_argument("--build",           action="store_true",    help="Build design.")
    target_group.add_argument("--load",            action="store_true",    help="Load bitstream.")
    target_group.add_argument("--toolchain",       default="trellis",      help="FPGA toolchain (trellis or diamond).")
    target_group.add_argument("--sys-clk-freq",    default=75e6,           help="System clock frequency.")
    target_group.add_argument("--revision",        default="1.0",          help="Board Revision (1.0).")
    target_group.add_argument("--device",          default="85F",          help="ECP5 device (25F, 45F, 85F).")
    target_group.add_argument("--sdram-device",    default="MT41K64M16",   help="SDRAM device (MT41K64M16, MT41K128M16, MT41K256M16 or MT41K512M16).")
    ethopts = target_group.add_mutually_exclusive_group()
    ethopts.add_argument("--with-ethernet",  action="store_true",    help="Add Ethernet.")
    ethopts.add_argument("--with-etherbone", action="store_true",    help="Add EtherBone.")
    target_group.add_argument("--eth-ip",          default="192.168.1.50", help="Ethernet/Etherbone IP address.")
    target_group.add_argument("--eth-dynamic-ip",  action="store_true",    help="Enable dynamic Ethernet IP addresses setting.")
    target_group.add_argument("--with-spi-flash",  action="store_true",    help="Enable SPI Flash (MMAPed).")
    sdopts = target_group.add_mutually_exclusive_group()
    sdopts.add_argument("--with-spi-sdcard", action="store_true", help="Enable SPI-mode SDCard support.")
    sdopts.add_argument("--with-sdcard",     action="store_true", help="Enable SDCard support.")
    target_group.add_argument("--with-syzygy-gpio",action="store_true", help="Enable GPIOs through SYZYGY Breakout on Port-A.")
    builder_args(parser)
    soc_core_args(parser)
    trellis_args(parser)
    args = parser.parse_args()

    assert not (args.with_etherbone and args.eth_dynamic_ip)

    soc = BaseSoC(
        toolchain        = args.toolchain,
        revision         = args.revision,
        device           = args.device,
        sdram_device     = args.sdram_device,
        sys_clk_freq     = int(float(args.sys_clk_freq)),
        with_ethernet    = args.with_ethernet,
        with_etherbone   = args.with_etherbone,
        eth_ip           = args.eth_ip,
        eth_dynamic_ip   = args.eth_dynamic_ip,
        with_spi_flash   = args.with_spi_flash,
        with_syzygy_gpio = args.with_syzygy_gpio,
        **soc_core_argdict(args))
    if args.with_spi_sdcard:
        soc.add_spi_sdcard()
    if args.with_sdcard:
        soc.add_sdcard()
    builder = Builder(soc, **builder_argdict(args))
    builder_kargs = trellis_argdict(args) if args.toolchain == "trellis" else {}
    if args.build:
        builder.build(**builder_kargs)

    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(builder.get_bitstream_filename(mode="sram"))

if __name__ == "__main__":
    main()
