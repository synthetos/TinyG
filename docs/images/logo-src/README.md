# edge-next logo (2026 update)

Sources for `docs/images/tinyg-edge-next-2026.png` (1200x630 banner, social-card size) and
`docs/images/tinyg-edge-next-2026-icon.png` (512x512). The mark is a NEMA 17 stepper motor face with a
jerk-limited S-curve speed profile across it; accent colour `#facc15`. Fonts: Big Shoulders Display and
IBM Plex Mono (Google Fonts, loaded by the pages).

Render at 2x with headless Chrome:

    chrome --headless=new --hide-scrollbars --force-device-scale-factor=2 --window-size=1200,630 \
      --virtual-time-budget=8000 --screenshot=tinyg-edge-next-2026.png tinyg-edge-next-2026.html
    chrome --headless=new --hide-scrollbars --force-device-scale-factor=2 --window-size=512,512 \
      --virtual-time-budget=8000 --screenshot=tinyg-edge-next-2026-icon.png tinyg-edge-next-2026-icon.html
