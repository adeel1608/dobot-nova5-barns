# BARNS Sequence Flowchart Pack

This pack documents the current sequence code with Mermaid flowcharts generated from the Python source AST.

## Open first

- Start with the per-file Markdown files in `docs/`.
- Mermaid diagrams render in GitHub, many IDE Markdown previews, and Mermaid-compatible viewers.
- `metadata/function_catalog.csv` lists parameter keys, branch conditions, and robot calls per function.

## Coverage

| Source file | Sequence/exported | Helper/support | Doc |
|---|---:|---:|---|
| `cleaning.py` | 7 | 6 | [docs/cleaning.md](docs/cleaning.md) |
| `computer_vision.py` | 1 | 0 | [docs/computer_vision.md](docs/computer_vision.md) |
| `espresso.py` | 47 | 25 | [docs/espresso.md](docs/espresso.md) |
| `home.py` | 10 | 5 | [docs/home.md](docs/home.md) |
| `milk_frothing.py` | 10 | 6 | [docs/milk_frothing.md](docs/milk_frothing.md) |
| `paper_cups.py` | 17 | 5 | [docs/paper_cups.md](docs/paper_cups.md) |
| `plastic_cups.py` | 10 | 7 | [docs/plastic_cups.md](docs/plastic_cups.md) |
| `slush.py` | 2 | 3 | [docs/slush.md](docs/slush.md) |
| `manipulate_node.py` | 5 | 1 | [docs/manipulate_node.md](docs/manipulate_node.md) |

## Notes

- The generator emphasizes robot motion calls, retries, cache branches, parameter normalization, detection checks, and returns.
- Trivial assignments are omitted to keep diagrams readable.
- Very large functions are capped with a visible truncation node; use the source file for low-level continuation.
