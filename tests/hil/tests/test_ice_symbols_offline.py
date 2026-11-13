"""ICE symbol resolution of bitfield members (the firmware builds GCodeState_t with __BITFIELDS); no board needed.

Compiles a packed struct with avr-gcc for both DWARF 2 (DW_AT_bit_offset, from the storage unit's MSB)
and DWARF 4 (DW_AT_data_bit_offset), including a field that straddles a byte, then checks that
resolve() + decode()/encode() see each field's own bits.
"""
import shutil
import subprocess

import pytest

import threading

from tinyg_hil.ice import Ice, Symbols, decode, encode

SOURCE = r'''
#include <stdint.h>
struct s { uint8_t lead; uint8_t a : 3; uint8_t b : 2; uint8_t c : 2; uint8_t d : 2; uint8_t e : 7; uint16_t w; };
volatile struct s v = {0x11, 5, 2, 1, 3, 100, 0x1234};
int main(void) { return v.a; }
'''
EXPECTED = {"lead": 0x11, "a": 5, "b": 2, "c": 1, "d": 3, "e": 100, "w": 0x1234}


@pytest.mark.parametrize("dwarf", ["-gdwarf-2", "-gdwarf-4"])
def test_bitfield_members_decode(tmp_path, dwarf):
    cc = shutil.which("avr-gcc")
    if not cc:
        pytest.skip("avr-gcc not installed")
    (tmp_path / "s.c").write_text(SOURCE)
    elf = tmp_path / "s.elf"
    subprocess.run([cc, "-mmcu=atxmega192a3", "-Os", "-fpack-struct", dwarf, str(tmp_path / "s.c"), "-o", str(elf)],
                   check=True)
    syms = Symbols(str(elf))
    section = syms.elf.get_section_by_name(".data")      # v's initial bytes stand in for SRAM
    start = section["sh_addr"] - syms.sram_symbol_base
    sram = {start + i: b for i, b in enumerate(section.data())}
    for field, value in EXPECTED.items():
        addr, n, kind = syms.resolve(f"v.{field}")
        raw = bytes(sram[a] for a in range(addr, addr + n))
        assert decode(raw, kind) == value, (field, kind, raw.hex())
        if kind.startswith("bits:"):             # writing one field leaves its neighbours alone
            other = encode(0, n, kind, raw)
            assert decode(bytes(other), kind) == 0
            assert decode(bytes(encode(value, n, kind, other)), kind) == value
            assert bytes(encode(value, n, kind, other)) == raw


def test_bitfield_poke_is_one_halt():
    """The read-modify-write of a bitfield must not let the CPU run between the read and the write."""
    events, memory = [], {0x10: 0xE7}

    class Protocol:
        def stop(self): events.append("stop")
        def run(self): events.append("run")
        def get_byte(self, context, offset): return 1
        def memory_read(self, memtype, addr, n):
            events.append("read")
            return bytearray(memory[addr + i] for i in range(n))
        def memory_write(self, memtype, addr, data):
            events.append("write")
            memory.update({addr + i: b for i, b in enumerate(data)})

    class Syms:
        def resolve(self, expr): return 0x10, 1, "bits:3:2"

    ice = Ice.__new__(Ice)
    ice.protocol, ice.symbols, ice.wire = Protocol(), Syms(), None
    ice._mutex, ice._halt_depth = threading.RLock(), 0
    ice.poke("v.b", 2)
    assert events == ["stop", "read", "write", "run"]
    assert memory[0x10] == 0xF7
