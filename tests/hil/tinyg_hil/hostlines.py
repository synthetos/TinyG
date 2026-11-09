"""What a careful host does to a G-code file before streaming it to TinyG (N67, N101).

- ! and ~ are removed anywhere: the board takes them as feedhold and resume even inside a comment.
- Then the cleaned, trimmed line is classified. It is dropped when it is a command rather than G-code:
  a leading $ (GRBL posts emit $H, $HZ, $X), ? or h/H puts the board in text mode, so a JSON host loses every reply;
  a lone % (a program delimiter in many files) is a queue flush to the board. A line left empty is dropped too.
The original text of every changed or dropped line is kept for the run record.
"""
TEXT_MODE_LEADS = ("$", "?", "h", "H")
REALTIME = ("!", "~")


def host_safe(lines, raw=False):
    """Return (lines to send, record). raw=True sends the file unchanged (SOAK_RAW=1)."""
    if raw:
        return list(lines), {"raw": True}
    out, dropped, cleaned = [], [], []
    for n, line in enumerate(lines, 1):
        clean = line
        for ch in REALTIME:
            clean = clean.replace(ch, "")
        if clean != line:
            cleaned.append((n, line[:60]))
        head = clean.strip()
        if not head or head == "%" or head[:1] in TEXT_MODE_LEADS:
            dropped.append((n, line[:60]))
            continue
        out.append(clean)
    return out, {"dropped": dropped[:20], "dropped_count": len(dropped),
                 "realtime_removed": cleaned[:20], "realtime_removed_count": len(cleaned)}
