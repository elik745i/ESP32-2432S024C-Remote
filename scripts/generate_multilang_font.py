from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

try:
    Import("env")
    PROJECT_DIR = Path(env["PROJECT_DIR"])
except NameError:
    PROJECT_DIR = Path(__file__).resolve().parents[1]

MAIN_CPP = PROJECT_DIR / "src" / "main.cpp"
APP_DIR = PROJECT_DIR / "src" / "app"
OUT_DIR = PROJECT_DIR / "src" / "generated"
OUT_FILE = OUT_DIR / "ui_multilang_font_16.c"

NODE_DIR = Path(r"C:\Program Files\nodejs")
LV_FONT_CONV = Path(os.environ.get("APPDATA", "")) / "npm" / "lv_font_conv.cmd"

FONT_SOURCES = {
    "default": Path(r"C:\Windows\Fonts\segoeui.ttf"),
    "cyrillic": Path(r"C:\Windows\Fonts\segoeui.ttf"),
    "han": PROJECT_DIR / "fonts" / "NotoSansCJKsc-Regular.otf",
    "kana": PROJECT_DIR / "fonts" / "NotoSansCJKjp-Regular.otf",
    "hangul": PROJECT_DIR / "fonts" / "NotoSansCJKkr-Regular.otf",
}


def collect_non_ascii_codepoints() -> list[int]:
    files = [MAIN_CPP]
    if APP_DIR.exists():
        files.extend(sorted(APP_DIR.rglob("*.inc")))

    chars: set[int] = set()
    for path in files:
        text = path.read_text(encoding="utf-8")
        for ch in text:
            cp = ord(ch)
            if cp <= 127:
                continue
            # Skip BOM and C1 control garbage that occasionally sneaks into edited text.
            if cp == 0xFEFF or 0x80 <= cp <= 0x9F:
                continue
            chars.add(cp)
    return sorted(chars)


def build_range_arg(codepoints: list[int]) -> str:
    ranges: list[str] = []
    start = end = codepoints[0]

    for cp in codepoints[1:]:
        if cp == end + 1:
            end = cp
            continue
        ranges.append(f"0x{start:X}" if start == end else f"0x{start:X}-0x{end:X}")
        start = end = cp

    ranges.append(f"0x{start:X}" if start == end else f"0x{start:X}-0x{end:X}")
    return ",".join(ranges)


def ensure_tooling() -> None:
    if not NODE_DIR.exists():
        raise RuntimeError("Node.js is not installed at C:\\Program Files\\nodejs")
    if not LV_FONT_CONV.exists():
        raise RuntimeError(f"lv_font_conv not found at {LV_FONT_CONV}")


def build_font() -> None:
    codepoints = collect_non_ascii_codepoints()
    if not codepoints:
        raise RuntimeError("No non-ASCII glyphs found in extracted UI sources")

    missing_fonts = [str(path) for path in FONT_SOURCES.values() if not path.exists()]
    if missing_fonts:
        raise RuntimeError("Missing font files: " + ", ".join(missing_fonts))

    OUT_DIR.mkdir(parents=True, exist_ok=True)

    cmd = [
        str(LV_FONT_CONV),
        "--size",
        "16",
        "--bpp",
        "4",
        "--format",
        "lvgl",
        "--no-compress",
        "--no-prefilter",
        "--force-fast-kern-format",
        "--lv-include",
        "lvgl.h",
        "--lv-font-name",
        "ui_multilang_font_16",
        "--lv-fallback",
        "lv_font_montserrat_16",
        "--output",
        str(OUT_FILE),
    ]

    groups = {
        "default": [cp for cp in codepoints if cp not in range(0x3040, 0x3100)
                    and cp not in range(0x4E00, 0xA000)
                    and cp not in range(0xAC00, 0xD7B0)
                    and cp < 0xF000],
        "cyrillic": [cp for cp in codepoints if 0x0400 <= cp <= 0x04FF],
        "kana": [cp for cp in codepoints if 0x3040 <= cp <= 0x30FF],
        "han": [cp for cp in codepoints if 0x4E00 <= cp <= 0x9FFF],
        "hangul": [cp for cp in codepoints if 0xAC00 <= cp <= 0xD7AF],
    }

    for key, cps in groups.items():
        if not cps:
            continue
        cmd.extend(["--font", str(FONT_SOURCES[key]), "--range", build_range_arg(cps)])

    env_vars = os.environ.copy()
    env_vars["PATH"] = str(NODE_DIR) + os.pathsep + env_vars.get("PATH", "")

    subprocess.run(cmd, check=True, env=env_vars, cwd=str(PROJECT_DIR))


try:
    ensure_tooling()
    build_font()
except Exception as exc:
    print(f"[font] {exc}", file=sys.stderr)
    raise
