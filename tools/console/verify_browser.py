#!/usr/bin/env python3
"""Broad headless-browser walkthrough of the console UI."""
import sys
from playwright.sync_api import sync_playwright

import os
URL = os.environ.get("CONSOLE_URL", "http://127.0.0.1:8090/")
SHOT = os.environ.get("SHOT_DIR", "/tmp")
R = []


def ok(name, cond, extra=""):
    R.append(cond)
    print(("PASS  " if cond else "FAIL  ") + name + (f"  [{extra}]" if extra else ""))


with sync_playwright() as p:
    b = p.chromium.launch()
    pg = b.new_page(viewport={"width": 1360, "height": 950})
    errs = []
    pg.on("pageerror", lambda e: errs.append(str(e)))
    pg.on("console", lambda m: errs.append(m.text) if m.type == "error" else None)
    pg.goto(URL, wait_until="networkidle")

    ok("page title", "Linorobot2 Console" in pg.title())
    ok("distro selector = jazzy/lyrical/rolling (no humble)",
       [o.get_attribute("value") for o in pg.query_selector_all("#hdr-distro-select option")]
       == ["jazzy", "lyrical", "rolling"])

    tabs = ["install", "bringup", "teleop", "slam-nav", "calibration", "lidar", "settings"]
    for t in tabs:
        pg.click(f'button.tab-btn[data-tab="{t}"]')
        pg.wait_for_timeout(120)
        ok(f"tab '{t}' shows", pg.is_visible(f"#tab-{t}"))

    # sensor registry drove the dropdowns
    pg.click('button.tab-btn[data-tab="install"]')
    laser_opts = [o.inner_text() for o in pg.query_selector_all("#install-laser option") if o.get_attribute("value")]
    ok("Install laser dropdown populated from /api/sensors", len(laser_opts) >= 3, ",".join(laser_opts))
    pg.click('button.tab-btn[data-tab="bringup"]')
    bl = [o.get_attribute("value") for o in pg.query_selector_all("#bringup-laser-sensor option") if o.get_attribute("value")]
    ok("Bringup laser model codes populated", "ld19" in bl and "a1" in bl, ",".join(bl[:6]))

    # SLAM & Nav — Nav2 editor + AI tuning + export/merge panel
    pg.click('button.tab-btn[data-tab="slam-nav"]')
    pg.wait_for_timeout(300)
    ok("AI Tuning Studio card present", pg.is_visible("#btn-ai-tune-apply") or pg.query_selector("#btn-ai-tune-apply") is not None)
    ok("Custom Robot Builder card present", pg.query_selector("#btn-ai-robot-generate") is not None)
    # open the YAML editors
    if pg.query_selector("#btn-nav2-toggle"):
        pg.click("#btn-nav2-toggle")
        pg.wait_for_timeout(200)
    tgl = pg.query_selector("summary")
    # the Export & Merge panel controls exist
    for cid in ["btn-params-export", "btn-params-merge", "btn-params-merge-dry", "btn-params-promote"]:
        ok(f"export/merge control #{cid}", pg.query_selector("#" + cid) is not None)

    # AI tune round-trip through the real endpoint
    pg.click('button.tab-btn[data-tab="slam-nav"]')
    pg.fill("#ai-tune-prompt", "robot overshoots the goal and blows past, late braking")
    pg.click("#btn-ai-tune-ask")
    pg.wait_for_selector("#ai-tune-diagnosis:not(:empty)", timeout=8000)
    diag = pg.inner_text("#ai-tune-diagnosis")
    ok("AI tune returns a diagnosis", "overshoot" in diag.lower() or "brak" in diag.lower(), diag[:90])

    # pickers (condensed)
    pg.click('button.tab-btn[data-tab="install"]')
    pg.click('.pick-btn[data-target="install-workspace"]')
    pg.wait_for_selector("#picker-overlay.open", timeout=4000)
    pg.wait_for_timeout(400)
    ok("path picker opens + lists", len(pg.query_selector_all("#picker-list .pk-row")) >= 1)
    pg.click("#picker-use")
    pg.wait_for_function("!document.querySelector('#picker-overlay').classList.contains('open')")
    ok("path picker filled install-workspace", pg.input_value("#install-workspace").startswith("/"))

    pg.click('button.tab-btn[data-tab="settings"]')
    pg.click('.pick-btn[data-target="cfg-agent-device"]')
    pg.wait_for_selector("#picker-overlay.open", timeout=4000)
    pg.wait_for_timeout(600)
    ok("serial picker path bar hidden", not pg.is_visible("#picker-path"))
    rows = pg.query_selector_all("#picker-list .pk-row")
    ok("serial picker lists devices/message", len(rows) >= 1)
    if rows and "No USB serial" not in rows[0].inner_text():
        rows[0].click()
        pg.wait_for_function("!document.querySelector('#picker-overlay').classList.contains('open')")
        ok("serial pick fills device (by-path)", pg.input_value("#cfg-agent-device").startswith("/dev/"))
    else:
        pg.click("#picker-close")

    pg.screenshot(path=f"{SHOT}/walkthrough_final.png", full_page=True)
    ok("no uncaught JS errors during walkthrough", len(errs) == 0, "; ".join(errs[:3]))
    b.close()

print(f"\n{sum(R)}/{len(R)} passed")
sys.exit(0 if all(R) else 1)
