#!/usr/bin/env python3
"""Headless-browser check of the console's new path / serial-port pickers."""
import sys
from playwright.sync_api import sync_playwright

import os
URL = os.environ.get("CONSOLE_URL", "http://127.0.0.1:8090/")
SHOT = os.environ.get("SHOT_DIR", "/tmp")
results = []


def ok(name, cond, extra=""):
    results.append((cond, name, extra))
    print(("PASS  " if cond else "FAIL  ") + name + (f"  [{extra}]" if extra else ""))


with sync_playwright() as p:
    b = p.chromium.launch()
    pg = b.new_page(viewport={"width": 1280, "height": 900})
    pg.goto(URL, wait_until="networkidle")
    ok("console page loads", "Linorobot2 Console" in pg.title())

    # ---- 1. dir picker on the Install tab (install-workspace) ----
    pg.click('button.tab-btn[data-tab="install"]')
    pg.click('.pick-btn[data-target="install-workspace"]')
    pg.wait_for_selector("#picker-overlay.open", timeout=4000)
    ok("dir picker modal opens", pg.is_visible("#picker-overlay.open"))
    ok("modal title = folder", pg.inner_text("#picker-title").lower().find("folder") >= 0)
    rows = pg.query_selector_all("#picker-list .pk-row")
    ok("dir listing populated", len(rows) >= 1, f"{len(rows)} rows")
    cwd0 = pg.inner_text("#picker-cwd")
    # descend into the first folder row
    folder_rows = [r for r in rows if "📂" in (r.inner_text() or "")]
    ok("has folders to click", len(folder_rows) > 0)
    if folder_rows:
        folder_rows[0].click()
        pg.wait_for_timeout(400)
        ok("clicking a folder navigates", pg.inner_text("#picker-cwd") != cwd0,
           pg.inner_text("#picker-cwd"))
    # go up
    pg.click("#picker-up")
    pg.wait_for_timeout(400)
    ok("up button navigates", pg.inner_text("#picker-cwd") == cwd0, pg.inner_text("#picker-cwd"))
    pg.screenshot(path=f"{SHOT}/picker_dir.png")
    # use this folder
    pg.click("#picker-use")
    pg.wait_for_function("!document.querySelector('#picker-overlay').classList.contains('open')", timeout=3000)
    val = pg.input_value("#install-workspace")
    ok("'Use this folder' fills the input", val == cwd0, val)

    # ---- 2. file picker on SLAM & Nav (nav-params-file, .yaml) ----
    pg.click('button.tab-btn[data-tab="slam-nav"]')
    CFG = os.path.abspath(os.path.join(os.path.dirname(__file__), "config"))
    pg.evaluate(f"document.getElementById('nav-params-file').value = {CFG!r}")
    pg.click('.pick-btn[data-target="nav-params-file"]')
    pg.wait_for_selector("#picker-overlay.open", timeout=4000)
    pg.wait_for_timeout(500)
    cwd = pg.inner_text("#picker-cwd")
    ok("file picker seeds from the input's current value", cwd == CFG, cwd)
    file_rows = [r for r in pg.query_selector_all("#picker-list .pk-row")
                 if "📄" in (r.inner_text() or "")]
    ok("only .yaml files listed (exts filter)",
       len(file_rows) > 0 and all(".yaml" in (r.inner_text() or "") for r in file_rows),
       f"{len(file_rows)} files")
    pg.screenshot(path=f"{SHOT}/picker_file.png")
    file_rows[0].click()
    pg.wait_for_function("!document.querySelector('#picker-overlay').classList.contains('open')", timeout=3000)
    fv = pg.input_value("#nav-params-file")
    ok("picking a file fills the input", fv.endswith(".yaml") and fv.startswith(CFG), fv)

    # ---- 3. serial-port picker (Settings > agent device) ----
    pg.click('button.tab-btn[data-tab="settings"]')
    pg.click('.pick-btn[data-target="cfg-agent-device"]')
    pg.wait_for_selector("#picker-overlay.open", timeout=4000)
    ok("serial picker modal opens", pg.is_visible("#picker-overlay.open"))
    ok("serial modal title", "serial port" in pg.inner_text("#picker-title").lower())
    ok("path bar hidden for serial", pg.get_attribute("#picker-path", "hidden") is not None)
    pg.wait_for_timeout(600)
    srows = pg.query_selector_all("#picker-list .pk-row")
    ok("serial list rendered", len(srows) >= 1, pg.inner_text("#picker-list")[:80])
    pg.screenshot(path=f"{SHOT}/picker_serial.png")
    txt0 = srows[0].inner_text()
    if "No USB serial" not in txt0:
        srows[0].click()
        pg.wait_for_function("!document.querySelector('#picker-overlay').classList.contains('open')", timeout=3000)
        sv = pg.input_value("#cfg-agent-device")
        ok("picking a port fills the device field", sv.startswith("/dev/"), sv)
    else:
        ok("serial list handled empty case", True, "no devices -> friendly message")
        pg.click("#picker-close")

    b.close()

fails = [r for r in results if not r[0]]
print(f"\n{len(results)-len(fails)}/{len(results)} passed")
sys.exit(1 if fails else 0)
