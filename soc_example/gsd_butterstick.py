#!/usr/bin/env python3

#
# This file is part of LiteX-Boards.
#
# Copyright (c) 2021 Florent Kermarrec <florent@enjoy-digital.fr>
# Copyright (c) 2021 Greg Davill <greg.davill@gmail.com>
# SPDX-License-Identifier: BSD-2-Clause

# Build/Use:
# ./gsd_butterstick.py --sdram-device MT41K64M16 --build --load
# litex_term /dev/ttyACM0

import os
import sys
import argparse

from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex_boards.platforms import butterstick

from litex.build.lattice.trellis import trellis_args, trellis_argdict

from litex.soc.cores.clock import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *
from litex.soc.cores.led import LedChaser

from litedram.modules import MT41K64M16,MT41K128M16,MT41K256M16,MT41K512M16
from litedram.phy import ECP5DDRPHY

from liteeth.phy.ecp5rgmii import LiteEthPHYRGMII


from litex.build.generic_platform import *

serial_syzygy0 = [
    ("serial", 0,
        Subsignal("tx", Pins("SYZYGY0:S0"), IOStandard("LVCMOS33")),
        Subsignal("rx", Pins("SYZYGY0:S1"), IOStandard("LVCMOS33"))
    )
]


# CRG ---------------------------------------------------------------------------------------------

class _CRG(Module):
    def __init__(self, platform, sys_clk_freq):
        self.rst = Signal()
        self.clock_domains.cd_init    = ClockDomain()
        self.clock_domains.cd_por     = ClockDomain(reset_less=True)
        self.clock_domains.cd_sys     = ClockDomain()
        self.clock_domains.cd_sys2x   = ClockDomain()
        self.clock_domains.cd_sys2x_i = ClockDomain(reset_less=True)
        self.clock_domains.cd_usb     = ClockDomain()

        # # #

        self.stop  = Signal()
        self.reset = Signal()

        # Clk / Rst
        clk30 = platform.request("clk30")
        rst_n = platform.request("user_btn", 0)

        # Power on reset
        por_count = Signal(16, reset=2**16-1)
        por_done  = Signal()
        self.comb += self.cd_por.clk.eq(clk30)
        self.comb += por_done.eq(por_count == 0)
        self.sync.por += If(~por_done, por_count.eq(por_count - 1))

        # PLL
        self.submodules.pll = pll = ECP5PLL()
        self.comb += pll.reset.eq(~por_done | ~rst_n)
        pll.register_clkin(clk30, 30e6)
        pll.create_clkout(self.cd_sys2x_i, 2*sys_clk_freq)
        pll.create_clkout(self.cd_init,   25e6)
        self.specials += [
            Instance("ECLKSYNCB",
                i_ECLKI = self.cd_sys2x_i.clk,
                i_STOP  = self.stop,
                o_ECLKO = self.cd_sys2x.clk),
            Instance("CLKDIVF",
                p_DIV     = "2.0",
                i_ALIGNWD = 0,
                i_CLKI    = self.cd_sys2x.clk,
                i_RST     = self.reset,
                o_CDIVX   = self.cd_sys.clk),
            AsyncResetSynchronizer(self.cd_sys,    ~pll.locked | self.reset),
            AsyncResetSynchronizer(self.cd_usb,    ~pll.locked),
            AsyncResetSynchronizer(self.cd_sys2x,  ~pll.locked | self.reset),
        ]

        self.comb += self.cd_usb.clk.eq(self.cd_sys.clk)

# BaseSoC ------------------------------------------------------------------------------------------

class BaseSoC(SoCCore):
    def __init__(self, revision="1.0", device="85F", sdram_device="MT41K64M16", sys_clk_freq=int(60e6), 
        toolchain="trellis", with_ethernet=False, with_etherbone=False, eth_ip="192.168.1.50", 
        eth_dynamic_ip=False,
        with_spi_flash=False,
        with_led_chaser=True,
        **kwargs)       :
        platform = butterstick.Platform(revision=revision, device=device ,toolchain=toolchain)

        platform.add_extension(serial_syzygy0)

        vccio_ctrl = platform.request("vccio_ctrl")

        self.sync.por += [
            vccio_ctrl.pdm.eq(~vccio_ctrl.pdm),
            vccio_ctrl.en.eq(1),
        ]

        kwargs["uart_name"] = "stream"
        
        # SoCCore ----------------------------------------------------------------------------------
        SoCCore.__init__(self, platform, sys_clk_freq,
            ident          = "LiteX SoC on ButterStick",
            ident_version  = True,
            **kwargs)


        # Serial -----------------------------------------------------------------------------------
        # Defaults to USB ACM through ValentyUSB.
        os.system("git clone https://github.com/gregdavill/luna-usb-serial-acm.git")
        sys.path.append("luna-usb-serial-acm")
        from USBSerialDevice import USBSerialDevice 
        self.submodules.usb_acm = usb_acm = USBSerialDevice(platform, platform.request('ulpi'))
        self.comb += [
            usb_acm.usb_rx.connect(self.uart.sink),
            self.uart.source.connect(usb_acm.usb_tx, omit=['last']),
            usb_acm.usb_tx.last.eq(1)
        ]

        
        # CRG --------------------------------------------------------------------------------------
        self.submodules.crg = _CRG(platform, sys_clk_freq)

        # DDR3 SDRAM -------------------------------------------------------------------------------
        if not self.integrated_main_ram_size:
            available_sdram_modules = {
                "MT41K64M16":  MT41K64M16,
                "MT41K128M16": MT41K128M16,
                "MT41K256M16": MT41K256M16,
                "MT41K512M16": MT41K512M16,
            }
            sdram_module = available_sdram_modules.get(sdram_device)

            self.submodules.ddrphy = ECP5DDRPHY(
                platform.request("ddram"),
                sys_clk_freq=sys_clk_freq)
            self.comb += self.crg.stop.eq(self.ddrphy.init.stop)
            self.comb += self.crg.reset.eq(self.ddrphy.init.reset)
            self.add_sdram("sdram",
                phy           = self.ddrphy,
                module        = sdram_module(sys_clk_freq, "1:2"),
                l2_cache_size = kwargs.get("l2_size", 8192),
            )

        # Ethernet / Etherbone ---------------------------------------------------------------------
        if with_ethernet or with_etherbone:
            self.submodules.ethphy = LiteEthPHYRGMII(
                clock_pads = self.platform.request("eth_clocks"),
                pads       = self.platform.request("eth"))
            if with_ethernet:
                self.add_ethernet(phy=self.ethphy, dynamic_ip=eth_dynamic_ip)
            if with_etherbone:
                self.add_etherbone(phy=self.ethphy, ip_address=eth_ip)

        # SPI Flash --------------------------------------------------------------------------------
        if with_spi_flash:
            from litespi.modules import W25Q128JV
            from litespi.opcodes import SpiNorFlashOpCodes as Codes
            self.add_spi_flash(mode="4x", module=W25Q128JV(Codes.READ_1_1_4), with_master=False)

        # Leds -------------------------------------------------------------------------------------
        if with_led_chaser:
            self.comb += platform.request("user_led_color").eq(0b010) # Blue.
            self.submodules.leds = LedChaser(
                pads         = platform.request_all("user_led"),
                sys_clk_freq = sys_clk_freq)

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="LiteX SoC on ButterStick")
    parser.add_argument("--build",           action="store_true",    help="Build bitstream")
    parser.add_argument("--load",            action="store_true",    help="Load bitstream")
    parser.add_argument("--toolchain",       default="trellis",      help="FPGA  use, trellis (default) or diamond")
    parser.add_argument("--sys-clk-freq",    default=60e6,           help="System clock frequency (default: 75MHz)")
    parser.add_argument("--revision",        default="1.0",          help="Board Revision: 1.0 (default)")
    parser.add_argument("--device",          default="85F",          help="ECP5 device (default: 85F)")
    parser.add_argument("--sdram-device",    default="MT41K64M16",   help="SDRAM device (default: MT41K64M16)")
    ethopts = parser.add_mutually_exclusive_group()
    ethopts.add_argument("--with-ethernet",  action="store_true",    help="Add Ethernet")
    ethopts.add_argument("--with-etherbone", action="store_true",    help="Add EtherBone")
    parser.add_argument("--eth-ip",          default="192.168.1.50", help="Ethernet/Etherbone IP address")
    parser.add_argument("--eth-dynamic-ip",  action="store_true",    help="Enable dynamic Ethernet IP addresses setting")
    parser.add_argument("--with-spi-flash",  action="store_true",    help="Enable SPI Flash (MMAPed)")
    sdopts = parser.add_mutually_exclusive_group()
    sdopts.add_argument("--with-spi-sdcard", action="store_true", help="Enable SPI-mode SDCard support")
    sdopts.add_argument("--with-sdcard",     action="store_true", help="Enable SDCard support")
    builder_args(parser)
    soc_core_args(parser)
    trellis_args(parser)
    args = parser.parse_args()

    assert not (args.with_etherbone and args.eth_dynamic_ip)

    soc = BaseSoC(
        toolchain      = args.toolchain,
        revision       = args.revision,
        device         = args.device,
        sdram_device   = args.sdram_device,
        sys_clk_freq   = int(float(args.sys_clk_freq)),
        with_ethernet  = args.with_ethernet,
        with_etherbone = args.with_etherbone,
        eth_ip         = args.eth_ip,
        eth_dynamic_ip = args.eth_dynamic_ip,
        with_spi_flash = args.with_spi_flash,
        **soc_core_argdict(args))
    if args.with_spi_sdcard:
        soc.add_spi_sdcard()
    if args.with_sdcard:
        soc.add_sdcard()
    builder = Builder(soc, **builder_argdict(args))
    builder_kargs = trellis_argdict(args) if args.toolchain == "trellis" else {}
    builder.build(**builder_kargs, run=args.build)

    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(os.path.join(builder.gateware_dir, soc.build_name + ".bit"))

    config = os.path.join(builder.gateware_dir, soc.build_name + ".config")
    output_bitstream = os.path.join(builder.gateware_dir, soc.build_name + ".bit")
    
    os.system(f"ecppack --freq 38.8 --compress --input {config} --bit {output_bitstream}")

    dfu_file = os.path.join(builder.gateware_dir, f"{soc.platform.name}.dfu")
    shutil.copyfile(output_bitstream, dfu_file)
    os.system(f"dfu-suffix -v 1209 -p 5bf0 -a {dfu_file}")

# 

if __name__ == "__main__":
    main()
