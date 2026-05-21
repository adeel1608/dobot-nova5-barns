# How to read this flowchart pack

1. Start at `README.md`.
2. Open a file under `docs/`, for example `docs/espresso.md`.
3. Each function has:
   - parameter scenarios observed from `params.get(...)` and extractor helpers,
   - branch/decision conditions,
   - a Mermaid flowchart.
4. For a browser view, open `viewer.html`; it uses Mermaid from a CDN.

Notes:
- Diagrams are generated from source code and are intended for debugging/documentation.
- Very large functions may show a truncation node to keep the diagram readable.
- The Python source remains the source of truth for deployment behavior.
