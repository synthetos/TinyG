"""Atmel-ICE on the TinyG's PDI header: peek/poke RAM by symbol, program counter, EEPROM, flash.

On an XMEGA, PDI memory access needs the CPU halted, so every read is halt -> read -> resume
(a few milliseconds; steppers and serial pause for that long). Symbols come from the ELF of the
build that is on the board; struct members need DWARF (the Makefile builds with -g).

    with Ice(elf="firmware/tinyg/build/tinyg.elf") as ice:
        ice.peek("mb.buffers_available")
        ice.peek("cm.hold_state")
        ice.poke("cs.null", 0)
        ice.where()                  # function the CPU is executing

Holding the board lock is the caller's job (pytest's tg fixture already holds it; the CLI takes it).
"""
import pathlib
import struct
import time
from contextlib import contextmanager

SRAM_SYMBOL_BASE = 0x800000         # avr-gcc puts data-space symbols at 0x800000 + address


def _le32(v):
    return bytearray(struct.pack("<I", v))


def _le16(v):
    return bytearray(struct.pack("<H", v))


# ATxmega192A3 device context for the AVR8 generic protocol (XMEGA layout)
DEVICE_DATA = (_le32(0x00800000) + _le32(0x00830000) + _le32(0x008C0000) + _le32(0x008F0020) +
               _le32(0x008F0027) + _le32(0x008E0400) + _le32(0x008E0200) + _le32(0x01000000) +
               _le32(0x00030000) + _le16(0x2000) + _le16(512) + _le16(2048) + bytearray([32]) +
               _le16(0x01C0) + _le16(0x0090))


class Symbols:
    """Addresses, sizes and (with DWARF) member offsets and base types from an ELF."""

    def __init__(self, elf_path):
        from elftools.elf.elffile import ELFFile
        self.path = elf_path
        self._f = open(elf_path, "rb")
        self.elf = ELFFile(self._f)
        self.sym = {}
        self.funcs = []
        for s in self.elf.get_section_by_name(".symtab").iter_symbols():
            kind = s["st_info"]["type"]
            if kind == "STT_OBJECT":
                self.sym[s.name] = (s["st_value"], s["st_size"])
            elif kind == "STT_FUNC" and s["st_size"]:
                self.funcs.append((s["st_value"], s["st_size"], s.name))
        self.funcs.sort()
        self.types = self._dwarf_variables() if self.elf.has_dwarf_info() else {}

    def _dwarf_variables(self):
        """Map global variable name -> DIE of its type."""
        out = {}
        for cu in self.elf.get_dwarf_info().iter_CUs():
            for die in cu.iter_DIEs():
                if die.tag == "DW_TAG_variable" and "DW_AT_name" in die.attributes and "DW_AT_type" in die.attributes:
                    name = die.attributes["DW_AT_name"].value.decode()
                    if name not in out or out[name].tag == "DW_TAG_base_type":
                        out[name] = die.get_DIE_from_attribute("DW_AT_type")
        return out

    @staticmethod
    def _strip(die):
        while die.tag in ("DW_TAG_typedef", "DW_TAG_volatile_type", "DW_TAG_const_type"):
            die = die.get_DIE_from_attribute("DW_AT_type")
        return die

    def resolve(self, expr):
        """'name', 'name.member[.member]', 'name[3]' or 'name+offset' -> (sram address, size, kind)."""
        offset = 0
        if "+" in expr:
            expr, extra = expr.split("+", 1)
            offset = int(extra, 0)
        parts = expr.replace("]", "").replace("[", ".[").split(".")
        name = parts[0]
        if name not in self.sym:
            raise KeyError(f"no data symbol {name!r} in {self.path}")
        addr, size = self.sym[name]
        addr = addr - SRAM_SYMBOL_BASE + offset
        kind = "bytes"
        die = self._strip(self.types[name]) if name in self.types else None
        for part in parts[1:]:
            if die is None:
                raise KeyError(f"{expr}: member access needs DWARF (build with -g)")
            if part.startswith("["):
                index = int(part[1:], 0)
                elem = self._strip(die.get_DIE_from_attribute("DW_AT_type"))
                esize = elem.attributes["DW_AT_byte_size"].value
                addr, size, die = addr + index * esize, esize, elem
                continue
            for child in die.iter_children():
                if child.tag == "DW_TAG_member" and child.attributes.get("DW_AT_name") and \
                        child.attributes["DW_AT_name"].value.decode() == part:
                    loc = child.attributes["DW_AT_data_member_location"].value
                    addr += loc if isinstance(loc, int) else loc[-1]
                    die = self._strip(child.get_DIE_from_attribute("DW_AT_type"))
                    size = die.attributes["DW_AT_byte_size"].value if "DW_AT_byte_size" in die.attributes else size
                    break
            else:
                raise KeyError(f"{expr}: no member {part!r}")
        if die is not None and die.tag == "DW_TAG_base_type":
            enc = die.attributes["DW_AT_encoding"].value
            kind = {4: "float", 5: "int", 6: "int", 7: "uint", 8: "uint", 2: "uint"}.get(enc, "bytes")
        elif die is not None and die.tag in ("DW_TAG_enumeration_type", "DW_TAG_pointer_type"):
            kind = "uint"
        return addr, size, kind

    def function_at(self, byte_addr):
        for start, size, name in self.funcs:
            if start <= byte_addr < start + size:
                return f"{name}+0x{byte_addr - start:x}"
        return f"0x{byte_addr:x}"


def decode(raw, kind):
    if kind == "float" and len(raw) == 4:
        return struct.unpack("<f", raw)[0]
    if kind in ("int", "uint") and len(raw) in (1, 2, 4):
        return int.from_bytes(raw, "little", signed=(kind == "int"))
    return bytes(raw)


def encode(value, size, kind):
    if kind == "float":
        return bytearray(struct.pack("<f", float(value)))
    if isinstance(value, (bytes, bytearray)):
        return bytearray(value)
    return bytearray(int(value).to_bytes(size, "little", signed=(kind == "int" and int(value) < 0)))


class Ice:
    def __init__(self, elf=None, serial=None):
        self.symbols = Symbols(elf) if elf else None
        self.serial = serial
        self.transport = self.target = self.protocol = None
        self._halt_depth = 0

    # ---- session ----------------------------------------------------------------------

    def open(self):
        from pyedbglib.hidtransport.hidtransportfactory import hid_transport
        from pymcuprog.avr8target import XmegaAvrTarget
        self.transport = hid_transport()
        if not self.transport.connect(serial_number=self.serial, product="atmel-ice"):
            raise RuntimeError("Atmel-ICE not found on USB")
        self.target = XmegaAvrTarget(self.transport)
        self.protocol = self.target.protocol
        for stale in (self.protocol.detach, self.target.deactivate_physical):   # left by an aborted session
            try:
                stale()
            except Exception:  # noqa: BLE001
                pass
        self.target.setup_debug_session()
        self.protocol.write_device_data(DEVICE_DATA)
        self.target.activate_physical()
        self.protocol.attach(do_break=False)
        return self

    def close(self):
        for step in (self.protocol.run, self.protocol.detach, self.target.deactivate_physical, self.transport.disconnect):
            try:
                step()
            except Exception:  # noqa: BLE001 - always try every step so the target keeps running
                pass

    def __enter__(self):
        return self.open()

    def __exit__(self, *exc):
        self.close()

    # ---- run control ------------------------------------------------------------------

    @contextmanager
    def halted(self):
        """Keep the CPU stopped for several accesses; nests."""
        if self._halt_depth == 0:
            self.protocol.stop()
        self._halt_depth += 1
        try:
            yield self
        finally:
            self._halt_depth -= 1
            if self._halt_depth == 0:
                self.protocol.run()

    def _retry(self, fn, *args):
        from pyedbglib.protocols.jtagice3protocol import Jtagice3ResponseError
        end = time.monotonic() + 0.5
        while True:
            try:
                return fn(*args)
            except Jtagice3ResponseError as e:
                if "ILLEGAL_STATE" not in str(e) or time.monotonic() > end:
                    raise
                time.sleep(0.001)          # stop() returns before the CPU has actually halted

    # ---- memory -----------------------------------------------------------------------

    def read(self, addr, n, memtype=None):
        from pyedbglib.protocols.avr8protocol import Avr8Protocol
        memtype = Avr8Protocol.AVR8_MEMTYPE_SRAM if memtype is None else memtype
        with self.halted():
            return bytes(self._retry(self.protocol.memory_read, memtype, addr, n))

    def write(self, addr, data, memtype=None):
        from pyedbglib.protocols.avr8protocol import Avr8Protocol
        memtype = Avr8Protocol.AVR8_MEMTYPE_SRAM if memtype is None else memtype
        with self.halted():
            self._retry(self.protocol.memory_write, memtype, addr, bytearray(data))

    def peek(self, expr):
        addr, size, kind = self.symbols.resolve(expr)
        return decode(self.read(addr, size), kind)

    def peek_many(self, exprs):
        """Several values from one halt (consistent snapshot)."""
        with self.halted():
            return {e: self.peek(e) for e in exprs}

    def poke(self, expr, value):
        addr, size, kind = self.symbols.resolve(expr)
        self.write(addr, encode(value, size, kind))

    def pc(self):
        with self.halted():
            return self._retry(self.protocol.program_counter_read) * 2      # word -> byte address

    def where(self):
        return self.symbols.function_at(self.pc()) if self.symbols else hex(self.pc())

    def eeprom_read(self, addr=0, n=2048):
        from pyedbglib.protocols.avr8protocol import Avr8Protocol
        return self.read(addr, n, Avr8Protocol.AVR8_MEMTYPE_EEPROM)



def flashed_elf(port):
    """ELF recorded by tinyg_hil.flash for the build last flashed on this port, or None."""
    f = pathlib.Path("/tmp") / f"tinyg-hil-{pathlib.Path(port).name}.flashed"
    return f.read_text().strip() if f.exists() else None


def main(argv=None):
    import argparse
    import json
    from .lock import BoardLock
    p = argparse.ArgumentParser(prog="tinyg_hil.ice", description="Atmel-ICE peek/poke on the TinyG")
    p.add_argument("command", choices=["peek", "poke", "watch", "where", "eeprom"])
    p.add_argument("args", nargs="*", help="peek: EXPR...; poke: EXPR VALUE; watch: EXPR...; eeprom: [OUTFILE]")
    p.add_argument("--port", default="/dev/cu.usbserial-BENCH1", help="serial port whose board lock to take")
    p.add_argument("--elf", help="ELF of the flashed build (default: the one recorded at flash time)")
    p.add_argument("--interval", type=float, default=0.25)
    p.add_argument("--count", type=int, default=0, help="watch: stop after N samples (0 = until Ctrl-C)")
    a = p.parse_args(argv)
    elf = a.elf or flashed_elf(a.port)
    if a.command in ("peek", "poke", "watch") and not elf:
        p.error("no ELF: pass --elf or flash through tinyg_hil first")
    lock = BoardLock(a.port).acquire(f"ice {a.command} {' '.join(a.args)}"[:120], wait=60)
    try:
        with Ice(elf=elf) as ice:
            if a.command == "peek":
                print(json.dumps(ice.peek_many(a.args), default=lambda b: b.hex(" ")))
            elif a.command == "poke":
                ice.poke(a.args[0], float(a.args[1]) if "." in a.args[1] else int(a.args[1], 0))
                print(json.dumps({a.args[0]: ice.peek(a.args[0])}, default=lambda b: b.hex(" ")))
            elif a.command == "where":
                print(ice.where())
            elif a.command == "eeprom":
                data = ice.eeprom_read()
                if a.args:
                    pathlib.Path(a.args[0]).write_bytes(data)
                    print(f"wrote {len(data)} bytes to {a.args[0]}")
                else:
                    print(data.hex(" "))
            elif a.command == "watch":
                n = 0
                while a.count == 0 or n < a.count:
                    t = time.monotonic()
                    print(json.dumps({"t": round(t, 3), **ice.peek_many(a.args)}, default=lambda b: b.hex(" ")),
                          flush=True)
                    n += 1
                    time.sleep(max(0.0, a.interval - (time.monotonic() - t)))
    finally:
        lock.release()


if __name__ == "__main__":
    main()
