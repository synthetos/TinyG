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
import threading
import time
from contextlib import contextmanager

from .board import Board, load_board


class Symbols:
    """Addresses, sizes and (with DWARF) member offsets and base types from an ELF."""

    def __init__(self, elf_path, sram_symbol_base=0x800000):
        from elftools.elf.elffile import ELFFile
        self.path = elf_path
        self.sram_symbol_base = sram_symbol_base     # avr-gcc puts data-space symbols at 0x800000 + address
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
        addr = addr - self.sram_symbol_base + offset
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
    def __init__(self, elf=None, serial=None, board: Board | None = None, wire=None):
        self.board = board or load_board()
        self.symbols = Symbols(elf, self.board.sram_symbol_base) if elf else None
        self.serial = serial
        self.wire = wire                    # the serial Transport.wire lock: never halt while bytes are inbound
        self.transport = self.target = self.protocol = None
        self._halt_depth = 0
        self._mutex = threading.RLock()     # one thread at a time drives the debugger (mounted switches poll)

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
        if self.board.data.get("ice", {}).get("run_timers_when_halted"):
            from pyedbglib.protocols.avr8protocol import Avr8Protocol
            self.protocol.set_byte(Avr8Protocol.AVR8_CTXT_OPTIONS, Avr8Protocol.AVR8_OPT_RUN_TIMERS, 1)
        self.protocol.write_device_data(self.board.pdi_device_data())
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
        """Keep the CPU stopped for several accesses; nests. Waits for any serial line in flight first."""
        with self._mutex:
            if self._halt_depth == 0:
                if self.wire:
                    self.wire.acquire()
                try:
                    self.protocol.stop()
                except BaseException:
                    if self.wire:
                        self.wire.release()
                    raise
            self._halt_depth += 1
            try:
                yield self
            finally:
                self._halt_depth -= 1
                if self._halt_depth == 0:
                    try:
                        self.protocol.run()
                    finally:
                        if self.wire:
                            self.wire.release()

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

    def port_out(self, port):
        """PORTx.OUT register value."""
        return self.read(self.board.port_reg(port, "OUT"), 1)[0]

    def output(self, name):
        """State (0/1) of a board output pin by name (board file "outputs")."""
        port, bit = self.board.output_pin(name)
        return (self.port_out(port) >> bit) & 1

    def outputs(self):
        with self.halted():
            return {name: self.output(name) for name in self.board.outputs}

    def switch_pin(self, axis, end):
        """Electrical level (0/1) of a switch input: PORTx.IN bit."""
        port, bit = self.board.switch_pin(axis, end)
        return (self.read(self.board.port_reg(port, "IN"), 1)[0] >> bit) & 1

    def set_switch_level(self, axis, end, level):
        """Drive a switch input low, or release it to its idle level (board pull-up -> high).

        "drive-low" (board file switch_drive.method): low = pin as output driving 0 (input buffer and
        pin-change interrupt keep working), high = pin back to input. "pull" instead swaps the
        internal pull-up/pull-down, for boards without external pull-ups. Either way the firmware's
        own pin-change ISR sees a real edge.
        """
        port, bit = self.board.switch_pin(axis, end)
        if self._uses_hook():
            self._switch_via_hook(axis, bit, level)
            return
        mask = bytes([1 << bit])
        method = self.board.data.get("switch_drive", {}).get("method", "pull")
        with self.halted():
            if method == "drive-low":
                if level:
                    self.write(self.board.port_reg(port, "DIRCLR"), mask)
                else:
                    self.write(self.board.port_reg(port, "OUTCLR"), mask)
                    self.write(self.board.port_reg(port, "DIRSET"), mask)
                return
            addr, opc = self.board.pinctrl(port, bit), self.board.opc
            ctrl = self.read(addr, 1)[0]
            self.write(addr, bytes([(ctrl & ~opc["mask"] & 0xFF) | (opc["pullup"] if level else opc["pulldown"])]))

    HOOK_RELEASE, HOOK_DRIVE_LOW, HOOK_INVERT, HOOK_NORMAL = 0, 1, 2, 3

    def _uses_hook(self):
        return bool(self.symbols) and "test_hook" in self.symbols.sym

    def _switch_via_hook(self, axis, bit, level, timeout=1.0):
        """Test builds (TINYG_TEST_HOOKS): the running firmware applies the change, so the port's edge
        detector and the switch ISR see it (a change made while the CPU is halted is not detected)."""
        if self._halt_depth:
            raise RuntimeError("switch hook requested while the CPU is held halted; the firmware cannot apply it")
        if self.peek("test_hook.magic") != 0x5A:
            raise RuntimeError("test_hook present but not initialized (firmware not running?)")
        index = list(self.board.switches).index(axis)
        action = self.HOOK_RELEASE if level else self.HOOK_DRIVE_LOW
        with self.halted():
            self.poke("test_hook.axis", index)
            self.poke("test_hook.bit", bit)
            self.poke("test_hook.action", action)
            self.poke("test_hook.pending", 1)                   # last: the firmware acts on this
        end = time.monotonic() + timeout
        while True:
            time.sleep(0.01)                    # each check halts the CPU; give the main loop room to run
            if not self.peek("test_hook.pending"):
                return
            if time.monotonic() > end:
                raise RuntimeError("firmware did not apply the switch hook (main loop blocked?)")

    def set_switch(self, axis, end, closed, normally_closed=False):
        """Simulate a switch: NO switches read low when closed, NC switches read high when open (tripped)."""
        tripped_level = 1 if normally_closed else 0
        self.set_switch_level(axis, end, tripped_level if closed else 1 - tripped_level)

    def release_all_switches(self):
        """Restore every switch input to the firmware's pull-up configuration."""
        if self._uses_hook():
            for axis in self.board.switches:             # the CPU must run to service each hook
                for end in ("min", "max"):
                    self.set_switch_level(axis, end, 1)
            return
        with self.halted():
            for axis in self.board.switches:
                for end in ("min", "max"):
                    self.set_switch_level(axis, end, 1)

    def pc(self):
        with self.halted():
            return self._retry(self.protocol.program_counter_read) * 2      # word -> byte address

    def where(self):
        return self.symbols.function_at(self.pc()) if self.symbols else hex(self.pc())

    def eeprom_read(self, addr=0, n=2048):
        from pyedbglib.protocols.avr8protocol import Avr8Protocol
        return self.read(addr, n, Avr8Protocol.AVR8_MEMTYPE_EEPROM)



def flashed_elf(port):
    """ELF recorded by tinyg_hil.flash for the build last flashed on this port, or None.

    Refuses an ELF whose contents changed since the flash (rebuilt tree), since its symbols would lie.
    """
    import hashlib
    from .flash import read_provenance
    prov = read_provenance(port)
    elf = prov.get("elf")
    if not elf or not pathlib.Path(elf).exists():
        return None
    if prov.get("elf_sha256") and hashlib.sha256(pathlib.Path(elf).read_bytes()).hexdigest() != prov["elf_sha256"]:
        raise RuntimeError(f"{elf} was rebuilt after flashing; reflash or pass --elf for the build on the board")
    return elf


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
