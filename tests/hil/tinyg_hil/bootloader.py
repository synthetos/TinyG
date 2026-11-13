"""Keep application traffic away from an AVR109 command interpreter."""
import json
import os
import time

import serial

from .state import port_path


class BootloaderActive(RuntimeError):
    pass


def marker(port):
    return port_path(port, 'flash-unsafe')


def quarantine(port, reason):
    path = marker(port)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', encoding='utf-8') as stream:
        json.dump({'reason': reason, 'time': time.time()}, stream)
        stream.flush()
        os.fsync(stream.fileno())


def require_application(port):
    if marker(port).exists():
        raise BootloaderActive(f'{port}: flash state unsafe; application traffic blocked. '
                               'Recover with the AVR109 flash helper or the ICE; '
                               f'guard: {marker(port)}')


def inspect_port(port, baud=115200):
    """Open without a reader thread; send only ESC/S and look for XBoot++."""
    wire = serial.Serial(port, baud, timeout=.25, xonxoff=False, rtscts=False, write_timeout=5)
    try:
        from .transport import restart_output
        restart_output(wire)  # local tty state only; sends no XON byte
        # Resetting the host RX queue sends nothing to the bootloader.
        wire.reset_input_buffer()
        wire.write(b'\x1bS')
        answer = wire.read(256)
        return wire, b'XBoot++' in answer
    except BaseException:
        wire.close()
        raise


def exit_and_inspect(wire):
    """Try AVR109 exit once, drain its ACK/banner to quiet, then probe again."""
    wire.write(b'E')
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline:
        if not wire.read(256):
            break
    wire.write(b'\x1bS')
    return b'XBoot++' in wire.read(256)
