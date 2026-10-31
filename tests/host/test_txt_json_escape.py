"""E137: text printed inside {"txt":"..."} is escaped exactly once, so r.txt decodes to the original text.

The real xio_putc_usb() filter (xio/xio_usb.c) and text_finalize_message() (text_parser.c) are extracted;
the raw writer is replaced by a capture buffer. Direct output (as help.c prints it) and a
text_finalize_message() report both carry a real newline, quote, backslash and tab, and the captured
bytes, wrapped in quotes, must decode with Python's JSON parser to exactly that text.
"""
import json
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import unittest

from source_extract import function

ROOT = Path(__file__).resolve().parents[2]

PRELUDE = r'''
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#define XIO_OK 0
#define JSON_MODE 1
#define JSON_MODE_TXT_OVERRIDE 2
struct { uint8_t json_escape; } xio;
struct { int comm_mode; } cs;
static char out[512]; static size_t n;
static int _putc_usb_raw(const char c) { out[n++] = c; return XIO_OK; }
'''

EXERCISE = r'''
static int capture_putc(char c, FILE *f) { return xio_putc_usb(c, f); }
static void emit(const char *s) { for (; *s; s++) capture_putc(*s, NULL); }
#define fputs(s, f) emit(s)
'''

MAIN = r'''
int main(void) {
    char report[] = "Line \"one\"\twith \\ slash\nLine two\n";
    xio.json_escape = 1; cs.comm_mode = JSON_MODE_TXT_OVERRIDE;
    emit("help: \"q\" \\ \tx\nnext\n");             /* direct printf-style output, as help.c */
    out[n++] = '|';
    text_finalize_message(report);                    /* a $-report through text_finalize_message */
    xio.json_escape = 0;
    out[n] = 0; fputs_stdout: printf("%s", out);
    return 0;
}
'''


class TxtJsonEscape(unittest.TestCase):
    def test_txt_contents_decode_to_original_text(self):
        cc = shutil.which("cc")
        if not cc:
            self.skipTest("host C compiler not installed")
        usb = (ROOT / "firmware/tinyg/xio/xio_usb.c").read_text()
        text = (ROOT / "firmware/tinyg/text_parser.c").read_text()
        putc = function(usb, r"int xio_putc_usb\([^;{}]*\)(?=\s*\{)")
        code = PRELUDE + putc + "\n" + EXERCISE
        code += function(text, r"void text_finalize_message\([^;{}]*\)(?=\s*\{)") + "\n"
        code += MAIN.replace("fputs_stdout: ", "")
        with tempfile.TemporaryDirectory(prefix="tinyg-e137-") as tmp:
            c, exe = Path(tmp) / "esc.c", Path(tmp) / "esc"
            c.write_text(code)
            subprocess.run([cc, "-std=c99", "-Wall", "-Wno-unused-function", "-fsanitize=address,undefined",
                            str(c), "-o", str(exe)], check=True)
            raw = subprocess.run([str(exe)], capture_output=True, text=True, check=True).stdout
        direct, report = raw.split("|")
        self.assertEqual(json.loads('"' + direct + '"'), 'help: "q" \\ \tx\nnext\n')
        self.assertEqual(json.loads('"' + report + '"'), 'Line "one"\twith \\ slash\nLine two\n')


if __name__ == "__main__":
    unittest.main()
