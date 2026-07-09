## graphify

This project has a knowledge graph at graphify-out/ with god nodes, community structure, and cross-file relationships.

The `graphify` CLI is only installed on the robot's Jetson (`/home/jetson1/.local/bin/graphify`). **First check it is available** (`command -v graphify`); if it is not (e.g. remote/web sessions), skip these rules and browse the source directly — do not report the missing command as an error.

Rules (only when `graphify` is installed):
- For codebase questions, first run `graphify query "<question>"` when graphify-out/graph.json exists. Use `graphify path "<A>" "<B>"` for relationships and `graphify explain "<concept>"` for focused concepts. These return a scoped subgraph, usually much smaller than GRAPH_REPORT.md or raw grep output.
- If graphify-out/wiki/index.md exists, use it for broad navigation instead of raw source browsing.
- Read graphify-out/GRAPH_REPORT.md only for broad architecture review or when query/path/explain do not surface enough context.
- After modifying code, run `graphify update .` to keep the graph current (AST-only, no API cost).
