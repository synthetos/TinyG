"""N92: the master 440.20/440.21 table in config_master.h stays usable for the migration.

config_master.h holds one string per master cfgArray row (396 rows): the token if master loaded it at boot, else
empty. Every such token must still name a persisted edge setting, or be one edge deliberately does not keep, so
that an edge table change cannot silently drop a master setting from the migration.
"""
import pathlib
import re
import unittest

TINYG = pathlib.Path(__file__).resolve().parents[2] / "firmware" / "tinyg"

# persisted on master, not kept by edge: identity, line numbers, G92/G28/G30 state, the network and motor-timeout rows
NOT_KEPT = {"fv", "n", "line", "net", "ma", "fd"} | {f"g{c}{a}" for c in ("92", "28", "30") for a in "xyzabc"}


def master_rows():
    text = (TINYG / "config_master.h").read_text()
    body = text[text.index("master_tokens[] PROGMEM ="):text.index(";", text.index("master_tokens[] PROGMEM ="))]
    joined = "".join(re.findall(r'"((?:[^"\\]|\\.)*)"', body))
    raw = joined.encode().decode("unicode_escape")
    rows = raw.split("\0")
    assert rows[-1] == "", "the last row must be NUL terminated"
    return rows[:-1], int(re.search(r"#define MASTER_NV_ROWS (\d+)", text).group(1))


def edge_persisted():
    """Tokens whose flag macro loads them at boot and persists them (F_INITIALIZE and F_PERSIST, from config.h)."""
    macros = {name for name, body in re.findall(r"^#define (_\w+)\s+\(([^)]*)\)", (TINYG / "config.h").read_text(), re.M)
              if "F_INITIALIZE" in body and "F_PERSIST" in body}
    rows = re.findall(r'\{\s*"[^"]*"\s*,\s*"([^"]+)"\s*,\s*(_\w+)', (TINYG / "config_app.c").read_text())
    return {token for token, flags in rows if flags in macros}


class MasterTable(unittest.TestCase):
    def test_row_count_matches_master(self):
        rows, declared = master_rows()
        self.assertEqual(declared, 396)
        self.assertEqual(len(rows), declared)
        self.assertEqual(rows[0], "", "record 0 is master's build number, not a setting")

    def test_every_master_setting_has_an_edge_home(self):
        rows, _ = master_rows()
        kept = edge_persisted()
        self.assertGreater(len(kept), 150)
        lost = [t for t in rows if t and t not in kept and t not in NOT_KEPT]
        self.assertEqual(lost, [], "master settings the migration would drop")

    def test_a_token_may_start_with_a_digit(self):
        rows, _ = master_rows()
        self.assertIn("1mi", rows)
        self.assertIn("4tr", rows)


if __name__ == "__main__":
    unittest.main()
