#!/bin/sh
# Re-download the reference documents that are not redistributable (or carry no licence) into
# docs/local/ (git-ignored), and check them against the SHA-256 recorded when they were first
# fetched (2026-10-31). A mismatch means upstream changed the file; compare before trusting it.
set -eu
cd "$(dirname "$0")"
mkdir -p local
get() {
    curl -sSL -A "Mozilla/5.0" -o "local/$1" "$2"
}
get Huzzah_ESP8266_Pinout_v1.2.pdf "https://cdn-learn.adafruit.com/assets/assets/000/046/211/original/Huzzah_ESP8266_Pinout_v1.2.pdf"
get adafruit_products_schem.png "https://cdn-learn.adafruit.com/assets/assets/000/096/603/original/adafruit_products_schem.png"
get adafruit_products_Feather_ESP8266_HUZZAH_sch.png "https://cdn-learn.adafruit.com/assets/assets/000/088/838/original/adafruit_products_Feather_ESP8266_HUZZAH_sch.png"
get adafruit_products_espbrd.png "https://cdn-learn.adafruit.com/assets/assets/000/028/672/original/adafruit_products_espbrd.png"
get ESP8266_Specifications_English.pdf "http://www.adafruit.com/datasheets/ESP8266_Specifications_English.pdf"
get cp2104.pdf "http://www.adafruit.com/datasheets/cp2104.pdf"
get esp8266ex_datasheet_en.pdf "https://www.espressif.com/sites/default/files/documentation/0a-esp8266ex_datasheet_en.pdf"
get "Adafruit ESP8266 Feather rev G.brd" "https://raw.githubusercontent.com/adafruit/Adafruit-Feather-ESP8266-HUZZAH-PCB/master/Adafruit%20ESP8266%20Feather%20rev%20G.brd"
shasum -a 256 -c <<'EOF'
0ca7a793a76d8252753f0b9d0fb6088244c4b7b65a7407940ae7996ca5ff3541  local/Huzzah_ESP8266_Pinout_v1.2.pdf
34c79fb2bb9ede57371d43dd01814f345aed644d76cfe68ad2a13d1d7034d5b9  local/adafruit_products_schem.png
ebf62134af868fa2da7032937cd76898e7bf5a709fd50769ad25b4fa1b4b213c  local/adafruit_products_Feather_ESP8266_HUZZAH_sch.png
17acca963a5b967c12c326bf9a8043497df6a7191907e18525b2b172a1d3e519  local/adafruit_products_espbrd.png
7d2da8ea29f45053605991b33c31ec030420725ce21f2c96c307ad8272e90363  local/ESP8266_Specifications_English.pdf
3373ea163f3dbcecc4709a0a9a1d348913141814b3a565db35f58815f7680ff3  local/cp2104.pdf
3a038e2fd3040ad06d95d41b16afc8292090339e9f8e728ce46a83db439cecf1  local/esp8266ex_datasheet_en.pdf
c7f249dfa2b899d480b497107b002c45218093275dbc5b5e70952b957d8ad9f3  local/Adafruit ESP8266 Feather rev G.brd
EOF
