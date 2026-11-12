"""Board utilities.

  uv run python -m tinyg_hil info     --port P
  uv run python -m tinyg_hil snapshot --port P [--out DIR]
  uv run python -m tinyg_hil restore  --port P SNAPSHOT.json
  uv run python -m tinyg_hil flash    --port P (--hex FILE | --build CHECKOUT) [--restore SNAPSHOT.json]

flash always takes a snapshot first (to --out) and, unless --no-restore, writes it back
afterwards, so a flash that changes fb (and therefore loads defaults) keeps the settings.
"""
import argparse
import os
import json
import pathlib
import sys

from . import flash as flasher
from . import snapshot
from . import perf_snapshot
from .driver import TinyG
from .lock import BoardLock

DEFAULT_OUT = pathlib.Path(__file__).resolve().parent.parent / ".snapshots"


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    if argv and argv[0] == "evidence":
        from .evidence import main as evidence_main
        return evidence_main(argv[1:])
    p = argparse.ArgumentParser(prog="tinyg_hil")
    p.add_argument("command", choices=["info", "snapshot", "restore", "flash", "text", "json"])
    p.add_argument("file", nargs="?", help="snapshot file for restore; the line to send for text/json")
    p.add_argument("--port", required=True)
    p.add_argument("--out", default=str(DEFAULT_OUT))
    p.add_argument("--hex")
    p.add_argument("--build", help="checkout to build with make (firmware/tinyg)")
    p.add_argument("--make", action="append", default=[], help="extra make arguments, e.g. --make TEST_HOOKS=1")
    p.add_argument("--restore", help="snapshot to restore after flashing (default: the one just taken)")
    p.add_argument("--no-snapshot", action="store_true",
                   help="flash without reading settings first (board in PANIC); needs --restore FILE")
    p.add_argument("--no-restore", action="store_true")
    p.add_argument("--perf-snapshot", default=os.environ.get("PERF_SNAPSHOT"),
                   help="fixed A/B settings file: restore and verify after every flash")
    p.add_argument("--transcript", help="write a timestamped raw serial transcript here")
    args = p.parse_args(argv)

    fixed = None
    if args.command == 'flash' and (args.perf_snapshot or os.environ.get('FIRMWARE_TAG') in ('A','B')):
        if not args.perf_snapshot:p.error('A/B flash requires --perf-snapshot or PERF_SNAPSHOT')
        if args.no_restore:p.error('A/B flash cannot use --no-restore')
        if args.restore and pathlib.Path(args.restore).resolve()!=pathlib.Path(args.perf_snapshot).resolve():
            p.error('--restore and --perf-snapshot must name the same fixed file')
        fixed, fixed_meta = perf_snapshot.load(args.perf_snapshot)
        args.restore = args.perf_snapshot

    lock = BoardLock(args.port).acquire(f"tinyg_hil {args.command}", wait=600)
    tg = TinyG(args.port, transcript=open(args.transcript, "w") if args.transcript else None)
    tg.connect()
    print(f"board: {json.dumps(tg.identity)} dialect {tg.dialect.name}")
    if args.command == "info":
        pass
    elif args.command == "text":
        for line in tg.send_text(args.file):
            print(line)
    elif args.command == "json":
        print(tg.send_json(args.file).raw)
    elif args.command == "snapshot":
        print("saved", snapshot.save(snapshot.take(tg), args.out))
    elif args.command == "restore":
        print(snapshot.restore(tg, snapshot.load(args.file)))
    elif args.command == "flash":
        if fixed and fixed.get('identity',{}).get('id') not in (None,tg.identity.get('id')):
            tg.close();lock.release()
            p.error('fixed snapshot belongs to another board')
        expect = None
        hex_path = args.hex
        if args.build:
            hex_path = flasher.build(args.build, args.make)
            expect = flasher.source_build(args.build)
            print(f"built {hex_path} (fb {expect})")
        if not hex_path:
            p.error("flash needs --hex or --build")
        snap = None
        if args.no_snapshot:            # a board in PANIC rejects the snapshot's commands; recover from a file
            if not args.restore and not args.no_restore:
                p.error("--no-snapshot needs --restore FILE (or --no-restore)")
        else:
            snap = snapshot.take(tg)
            print("snapshot", snapshot.save(snap, args.out))
        ident = flasher.flash(tg, hex_path, expect_fb=expect)
        print(f"flashed: {json.dumps(ident)} dialect {tg.dialect.name}")
        if not args.no_restore:
            source = fixed if fixed is not None else (snapshot.load(args.restore) if args.restore else snap)
            if fixed is not None:
                try:
                    print('fixed restore:', perf_snapshot.restore_verified(tg, fixed))
                    print('fixed snapshot:', json.dumps(fixed_meta))
                except Exception:
                    tg.close();lock.release();raise
            else:
                print("restore:", snapshot.restore(tg, source))
    tg.close()
    lock.release()


if __name__ == "__main__":
    sys.exit(main())
