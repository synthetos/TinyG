"""Compile the firmware's input classifier for host-only regression checks."""
import ctypes
from pathlib import Path
import shutil
import subprocess

import pytest

CTRL, DATA = 1, 2


@pytest.fixture(scope="module")
def classify(tmp_path_factory):
    cc = shutil.which("cc")
    if cc is None:
        pytest.skip("host C compiler required")
    source = (Path(__file__).resolve().parents[3] / "firmware/tinyg/xio.c").read_text()
    start = source.index("static bool _json_space(")
    end = source.index("/*\n * _next_buffer_to_process()", start)
    directory = tmp_path_factory.mktemp("xio-classifier")
    c_file, library = directory / "classifier.c", directory / "classifier.so"
    c_file.write_text('''
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#define NUL 0
#define PSTR(s) (s)
#define pgm_read_byte(p) (*(const unsigned char *)(p))
#define strchr_P strchr
typedef enum { DEV_IS_CTRL = 1, DEV_IS_DATA = 2 } devflags_t;
''' + source[start:end] + '''
int classify(const char *line, uint16_t size) { return _classify_line(line, size); }
''')
    subprocess.run([cc, "-shared", "-fPIC", "-std=c99", "-funsigned-char", "-Wall", "-Wextra", "-Werror",
                    str(c_file), "-o", str(library)], check=True, capture_output=True, text=True)
    function = ctypes.CDLL(str(library)).classify
    function.argtypes = [ctypes.c_char_p, ctypes.c_uint16]
    function.restype = ctypes.c_int
    return function


@pytest.mark.parametrize("line, expected", [
    ('G4 P1', DATA),
    ('  G4 P1', DATA),
    ('?', CTRL),
    ('$fb', CTRL),
    ('H', CTRL),
    ('h', CTRL),
    ('!', CTRL),
    ('~', CTRL),
    ('%', CTRL),
    ('{"gc":"G4 P1"}', DATA),
    ('{gc:"G4 P1"}', DATA),
    ('{GC:"G4 P1"}', DATA),
    ('{g"c:"G4 P1"}', DATA),  # tokenizer strips key quotes even in relaxed input
    ('{"gc:"G4 P1"}', DATA),
    ('{ " G C "\t:\x7f"G4 P1" }', DATA),
    ('{tid:1,gc:"G4 P1"}', DATA),
    ('{gc:"G4 P1",tid:1}', DATA),
    ('{gc:null}', CTRL),
    ('{GC:NULL}', CTRL),
    ('{gc:"null"}', DATA),
    ('{gc:false}', DATA),
    ('{gc:0}', DATA),
    ('{gc:null,txt:"G4 P1"}', DATA),
    ('{txt:"G4 P1"}', DATA),
    ('{ T X T \t: "G4 P1"}', DATA),
    ('{txt:" G4 P1"}', DATA),
    ('{txt:"$fb"}', CTRL),
    ('{txt:"?"}', CTRL),
    ('{txt:"H"}', CTRL),
    ('{txt:"h"}', CTRL),
    ('{txt:""}', CTRL),
    ('{txt:null}', CTRL),
    ('{txt:"$fb",gc:"G4 P1"}', DATA),
    ('{"txt":"{\\"gc\\":\\"G4 P1\\"}"}', DATA),
    ('{"txt":"{\\"sr\\":null}"}', DATA),  # conservative nested-container policy
    ('{"sr":null}', CTRL),
    ('{"sr":null,"msg":"literal {gc:hello}"}', CTRL),
    ('{"sr":{"gc":false}}', CTRL),
    ('{"sr":{"txt":"G4 P1"}}', CTRL),
    ('{"msg":"literal \\" ,gc:hello", "sr":null}', CTRL),
    ('{"msg":"literal \\" ,gc:hello", "gc":"G4 P1"}', DATA),
    ('{sr:{gc:false},gc:"G4 P1"}', DATA),
    ('{msg:[{gc:false}],sr:null}', CTRL),
    ('{msg:[{gc:false}],txt:"G4 P1"}', DATA),
    ('{longgc:"G4 P1"}', CTRL),
    ('{gcx:"G4 P1"}', CTRL),
    ('{txtlong:"G4 P1"}', CTRL),
    ('{sr:null}\0{gc:"G4 P1"}', CTRL),
])
def test_classifier(classify, line, expected):
    encoded = line.encode()
    assert classify(encoded, len(encoded) + 1) == expected


@pytest.mark.parametrize("line, size", [
    (b'{gc:"G1"}', 0),
    (b'{gc:"G1"}', 1),
    (b'{gc:"G1"}', 4),
    (b'{txt:"G1"}', 5),
    (b'{txt:"G1"}', 6),
    (b'   G1', 3),
])
def test_classifier_respects_bound(classify, line, size):
    assert classify(line, size) == CTRL
