# Switchbox reference documents

Hardware references for the Adafruit Feather HUZZAH ESP8266 (product 2821) and its parts. The per-pin analysis drawn from them is in [pins.md](pins.md).

All files were fetched on **2026-10-31** (22:24 UTC). The source is the Adafruit guide's downloads page, https://learn.adafruit.com/adafruit-feather-huzzah-esp8266/downloads, plus the files it links to. The one exception is the Espressif ESP8266EX datasheet, fetched from espressif.com because the spec sheet the guide links (2013) has no pin or boot-strap table. Files are kept byte-for-byte as downloaded.

## Committed: `adafruit-pcb/`

Adafruit's EagleCAD design, from https://github.com/adafruit/Adafruit-Feather-ESP8266-HUZZAH-PCB (last pushed 2020-10-30). It is licensed **CC BY-SA 3.0**: the full text is in `adafruit-pcb/license.txt`. It was designed by Limor Fried/Ladyada for Adafruit Industries.

| file | SHA-256 |
|---|---|
| `Adafruit ESP8266 Feather rev G.sch` (schematic, Eagle XML; the netlist in pins.md comes from it) | `633e53d91e8a7637001416031415361166549e9e1301e35cc1ac0442756e482f` |
| `license.txt` (upstream licence) | `075dad5e5fc96c27014fabc269f4f5732909cffd178a486f546d982b6cf86b74` |
| `README.md` (upstream readme) | `0a44d505f87f5261f48d02f2f7a4cdb2ae4d0040d46856a216b094cb52b42125` |

## Local only: `local/` (git-ignored; `sh fetch-local.sh` re-downloads and checks them)

These are either proprietary datasheets or Learn-guide assets published without a stated licence, so they are not committed.

| file | source | why local |
|---|---|---|
| `Huzzah_ESP8266_Pinout_v1.2.pdf` | https://cdn-learn.adafruit.com/assets/assets/000/046/211/original/Huzzah_ESP8266_Pinout_v1.2.pdf | no licence stated on the guide |
| `adafruit_products_schem.png` (Rev G schematic image) | https://cdn-learn.adafruit.com/assets/assets/000/096/603/original/adafruit_products_schem.png | no licence stated |
| `adafruit_products_Feather_ESP8266_HUZZAH_sch.png` (schematic, large) | https://cdn-learn.adafruit.com/assets/assets/000/088/838/original/adafruit_products_Feather_ESP8266_HUZZAH_sch.png | no licence stated |
| `adafruit_products_espbrd.png` (fabrication print) | https://cdn-learn.adafruit.com/assets/assets/000/028/672/original/adafruit_products_espbrd.png | no licence stated |
| `ESP8266_Specifications_English.pdf` (Espressif, 2013) | http://www.adafruit.com/datasheets/ESP8266_Specifications_English.pdf | Espressif copyright, no licence granted |
| `esp8266ex_datasheet_en.pdf` (Espressif, v7.1) | https://www.espressif.com/sites/default/files/documentation/0a-esp8266ex_datasheet_en.pdf | Espressif copyright, no licence granted |
| `cp2104.pdf` (Silicon Labs CP2104) | http://www.adafruit.com/datasheets/cp2104.pdf | Silicon Labs copyright |
| `Adafruit ESP8266 Feather rev G.brd` (board layout, 0.9 MB) | the Adafruit PCB repo above | CC BY-SA, but large and not needed for the pin analysis |

**Not fetched:**
- the guide's FCC and CE test reports for the module (they are not datasheets);
- the AP2112 and MCP73831 datasheets (power parts);
- the Fritzing part (it lives in the Adafruit Fritzing library repository, not as a single small file).

The guide links no ESP-12 *module* datasheet. The module pinout comes from the Eagle footprint (`ESP-12_DOIT_ESP8266`, package `ESP-12F`) and from the pinout PDF, which names the module as Ai-Thinker ESP8266MOD.

**Handling downloads:** the files were first saved to a fresh, empty directory outside the repository and inspected there; any parsing used `python3 -I`. To extract the netlist from the `.sch`:

```
python3 -I - "adafruit-pcb/Adafruit ESP8266 Feather rev G.sch" <<'EOF'
import sys, xml.etree.ElementTree as ET
root = ET.parse(sys.argv[1]).getroot()
for net in root.iter("net"):
    print(net.get("name"), sorted({f"{p.get('part')}.{p.get('gate')}:{p.get('pin')}" for p in net.iter("pinref")}))
EOF
```
