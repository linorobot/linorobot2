#!/usr/bin/env python3
"""Headless-browser check of the console's Robot Name + Branch header row.

Assumes a console server is already running:  python3 web/server.py 8098
Usage: python3 verify_robot_header.py [port]

NOTE: the '= name' check only fills the branch field and then triggers a real
`git checkout -b <robot>`; run this against a scratch checkout, or delete the
created branch afterwards.
"""
import sys
from playwright.sync_api import sync_playwright

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8098
URL = f"http://localhost:{PORT}/"

passed, failed = 0, 0


def menu_names(page, menu_id):
    """The names in an open picker menu, read off data-name (the visible text
    also carries the 'current' badge)."""
    return page.eval_on_selector_all(
        f"#{menu_id} .branch-item", "els => els.map(e => e.dataset.name)")


def check(name, cond, detail=""):
    global passed, failed
    if cond:
        passed += 1
        print(f"  PASS  {name}")
    else:
        failed += 1
        print(f"  FAIL  {name}  {detail}")


with sync_playwright() as p:
    browser = p.chromium.launch()
    page = browser.new_page()
    errors = []
    page.on("pageerror", lambda e: errors.append(str(e)))
    page.goto(URL, wait_until="networkidle")
    page.wait_for_timeout(1200)

    check("no JS page errors", not errors, str(errors))

    # 1. header row renders
    check("robot name input present", page.locator("#hdr-robot-name").count() == 1)
    check("branch input present", page.locator("#hdr-git-branch").count() == 1)
    check("robot menu caret present", page.locator("#btn-robot-menu").count() == 1)
    check("branch menu caret present", page.locator("#btn-branch-menu").count() == 1)
    check("'= name' button present", page.locator("#btn-branch-to-name").count() == 1)

    # 2. populated from /api/status + /api/gitinfo
    robot = page.input_value("#hdr-robot-name")
    branch = page.input_value("#hdr-git-branch")
    check("robot name populated", bool(robot), f"got {robot!r}")
    check("branch populated", bool(branch), f"got {branch!r}")
    print(f"        robot={robot!r} branch={branch!r}")

    # 3. robot dropdown lists saved configs
    page.click("#btn-robot-menu")
    page.wait_for_timeout(300)
    check("robot menu opens", not page.locator("#robot-menu").is_hidden())
    items = menu_names(page, "robot-menu")
    check("robot menu lists active robot", robot in items, f"items={items}")
    page.click("body")
    page.wait_for_timeout(200)
    check("robot menu closes on outside click", page.locator("#robot-menu").is_hidden())

    # 4. branch dropdown lists local git branches
    page.click("#btn-branch-menu")
    page.wait_for_timeout(500)
    check("branch menu opens", not page.locator("#branch-menu").is_hidden())
    branches = menu_names(page, "branch-menu")
    check("branch menu lists current branch", branch in branches, f"branches={branches}")
    check("current branch is first (pinned)", branches and branches[0] == branch, f"{branches[:3]}")
    page.click("body")
    page.wait_for_timeout(200)

    # 5. switching robots via the input persists and reloads config
    page.fill("#hdr-robot-name", "browsertest_bot")
    page.locator("#hdr-robot-name").press("Enter")
    page.wait_for_timeout(1200)
    log = page.inner_text("#console-pane")
    check("robot switch logged", "browsertest_bot" in log, log[-200:])
    api = page.evaluate("() => fetch('/api/robots').then(r => r.json())")
    names = [r["name"] for r in api["robots"]]
    check("new robot config created", "browsertest_bot" in names, str(names))
    check("new robot is active", api["active"] == "browsertest_bot", api["active"])

    # 6. header input reflects the active robot after a status poll
    page.wait_for_timeout(4500)
    check("header shows switched robot", page.input_value("#hdr-robot-name") == "browsertest_bot")

    # 7. invalid robot name is rejected client-side and the field reverts
    page.fill("#hdr-robot-name", "Bad Name!")
    page.locator("#hdr-robot-name").press("Enter")
    page.wait_for_timeout(600)
    log = page.inner_text("#console-pane")
    check("invalid robot name rejected", "invalid robot name" in log, log[-200:])
    check("field reverted to active robot",
          page.input_value("#hdr-robot-name") == "browsertest_bot",
          page.input_value("#hdr-robot-name"))

    # 8. Escape closes an open menu; typing filters the list (config-engine UX)
    page.click("#hdr-git-branch")
    page.wait_for_timeout(500)
    check("clicking the branch input opens the menu",
          not page.locator("#branch-menu").is_hidden())
    check("current branch is marked with the dot",
          page.locator("#branch-menu .branch-item.is-current").count() == 1)
    page.fill("#hdr-git-branch", "zzz_no_such_branch")
    page.wait_for_timeout(300)
    visible = [b for b in page.locator("#branch-menu .branch-item").all()
               if b.is_visible()]
    check("typing filters the branch list to nothing", len(visible) == 0, str(len(visible)))
    page.keyboard.press("Escape")
    page.wait_for_timeout(200)
    check("Escape closes the branch menu", page.locator("#branch-menu").is_hidden())

    # 9. '= name' copies the robot name into the branch field
    page.click("#btn-branch-to-name")
    page.wait_for_timeout(400)
    check("'= name' fills branch field",
          page.input_value("#hdr-git-branch") == "browsertest_bot",
          page.input_value("#hdr-git-branch"))

    # restore
    page.fill("#hdr-robot-name", "linorobot2")
    page.locator("#hdr-robot-name").press("Enter")
    page.wait_for_timeout(1000)
    api = page.evaluate("() => fetch('/api/robots').then(r => r.json())")
    check("restored to linorobot2", api["active"] == "linorobot2", api["active"])

    browser.close()

print(f"\n{passed} passed, {failed} failed")
sys.exit(1 if failed else 0)
