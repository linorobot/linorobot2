#!/usr/bin/env python3
# Copyright (c) 2026 Linorobot contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Comment-preserving, value-level deep merge for the Nav2 / EKF / SLAM YAML.

Zero external dependencies (no pyyaml / ruamel) -- these configs are heavily
commented and pyyaml would strip every comment on a round-trip. Instead this
walks both files line by line, keys each scalar `key: value` mapping line by
its full indent-path, and rewrites the target's value in place wherever the
source sets the same path. Full-line comments, blank lines, trailing inline
comments and key order in the target are all left exactly as they were.

Scope: block-style mappings with 2-space indent steps and scalar / inline-list
(`[...]`) values -- which is everything the linorobot2 Nav2 configs use. Block
sequences (`- item`) are treated as opaque and never merged into.
"""

import re

_KV = re.compile(
    r'^(?P<indent>[ \t]*)'
    r'(?P<key>[A-Za-z0-9_.:/+~-]+):'
    r'(?P<sp>[ \t]*)'
    r'(?P<val>(?:"[^"]*"|\'[^\']*\'|[^#\n])*?)'
    r'(?P<comment>[ \t]*#.*)?$'
)
_LIST_ITEM = re.compile(r'^[ \t]*-\s')
_BLANK_OR_COMMENT = re.compile(r'^[ \t]*(#.*)?$')


def scalar_paths(text):
    """full key-path tuple -> (line_index, value_str) for every scalar mapping line."""
    out = {}
    stack = []  # list of (indent, key) for the current ancestor chain
    for i, line in enumerate(text.splitlines()):
        if _BLANK_OR_COMMENT.match(line) or _LIST_ITEM.match(line):
            continue
        m = _KV.match(line)
        if not m:
            continue
        indent = len(m.group('indent').expandtabs(1))
        while stack and stack[-1][0] >= indent:
            stack.pop()
        key = m.group('key')
        path = tuple(k for _, k in stack) + (key,)
        val = (m.group('val') or '').rstrip()
        if val == '':
            stack.append((indent, key))          # a parent mapping key
        else:
            out[path] = (i, val)
    return out


def merge_yaml(target_text, source_text):
    """Overlay source's scalar values onto target, keeping target's layout/comments.

    Returns (merged_text, report) where report is
    {"changed": [path,...], "unchanged": n, "source_only": [path,...]}.
    Paths present only in source are reported, not written (keeping the file
    structurally safe); the caller can surface them.
    """
    lines = target_text.splitlines()
    tgt = scalar_paths(target_text)
    src = scalar_paths(source_text)

    changed, source_only = [], []
    for path, (_, sval) in src.items():
        if path not in tgt:
            source_only.append(path)
            continue
        ti, tval = tgt[path]
        if tval == sval:
            continue
        m = _KV.match(lines[ti])
        sp = m.group('sp') or ' '
        comment = m.group('comment') or ''
        lines[ti] = f"{m.group('indent')}{m.group('key')}:{sp}{sval}{comment}"
        changed.append(path)

    merged = "\n".join(lines)
    if target_text.endswith("\n"):
        merged += "\n"
    return merged, {
        "changed": ["/".join(p) for p in changed],
        "unchanged": len(src) - len(changed) - len(source_only),
        "source_only": ["/".join(p) for p in source_only],
    }


def diff_summary(a_text, b_text):
    """Human-readable list of scalar paths whose value differs between two configs."""
    a, b = scalar_paths(a_text), scalar_paths(b_text)
    rows = []
    for path in sorted(set(a) | set(b)):
        av = a.get(path, (None, "<absent>"))[1]
        bv = b.get(path, (None, "<absent>"))[1]
        if av != bv:
            rows.append(f"{'/'.join(path)}: {av}  ->  {bv}")
    return rows
