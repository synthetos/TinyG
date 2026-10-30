"""Board description ("device tree") loaded from tests/hil/boards/<name>.json.

Everything specific to a board revision - MCU and PDI device context, port register layout, output and
switch pin maps, firmware symbols the harness reads - lives in that file, not in the code.
"""
import json
import os
import pathlib
import struct

BOARDS = pathlib.Path(__file__).resolve().parent.parent / "boards"
DEFAULT_BOARD = os.environ.get("TINYG_BOARD", "tinyg-v8")


def _int(v):
    return int(v, 0) if isinstance(v, str) else int(v)


class Board:
    def __init__(self, data: dict, path=None):
        self.data, self.path = data, path
        self.name = data["name"]
        mcu = data["mcu"]
        self.ports = {k: _int(v) for k, v in mcu["ports"].items()}
        self.regs = {k: _int(v) for k, v in mcu["port_registers"].items()}
        self.opc = {k: _int(v) for k, v in mcu["pinctrl_opc"].items()}
        self.sram_symbol_base = _int(mcu["sram_symbol_base"])
        self.outputs = data.get("outputs", {})
        self.switches = data.get("switches", {})
        self.symbols = data.get("firmware_symbols", {})

    # ---- registers --------------------------------------------------------------------

    def port_reg(self, port, reg):
        return self.ports[port] + self.regs[reg]

    def pinctrl(self, port, bit):
        return self.ports[port] + self.regs["PINCTRL"] + bit

    def output_pin(self, name):
        o = self.outputs[name]
        return o["port"], o["bit"]

    def switch_pin(self, axis, end):
        s = self.switches[axis]
        return s["port"], s[end]

    def switch_input(self, axis, end):
        """Firmware input number ($inN) for a switch."""
        return self.switches[axis][f"input_{end}"]

    def axis_index(self, axis):
        return self.symbols["axis_index"][axis]

    # ---- programmer -------------------------------------------------------------------

    def pdi_device_data(self) -> bytearray:
        """AVR8 generic protocol XMEGA device context (field order fixed by the protocol)."""
        c = {k: _int(v) for k, v in self.data["mcu"]["pdi_device_context"].items()}
        le32 = lambda v: bytearray(struct.pack("<I", v))
        le16 = lambda v: bytearray(struct.pack("<H", v))
        return (le32(c["appl_base"]) + le32(c["boot_base"]) + le32(c["eeprom_base"]) + le32(c["fuse_base"]) +
                le32(c["lockbit_base"]) + le32(c["user_sig_base"]) + le32(c["prod_sig_base"]) + le32(c["data_base"]) +
                le32(c["appl_bytes"]) + le16(c["boot_bytes"]) + le16(c["flash_page_bytes"]) + le16(c["eeprom_bytes"]) +
                bytearray([c["eeprom_page_bytes"]]) + le16(c["nvm_offset"]) + le16(c["signature_offset"]))


def load_board(name_or_path=None) -> Board:
    ref = name_or_path or DEFAULT_BOARD
    path = pathlib.Path(ref)
    if not path.suffix:
        path = BOARDS / f"{ref}.json"
    return Board(json.loads(path.read_text()), path)
