"""
gui/params_editor.py

Read and update values inside oms_v1/params.py without disturbing comments,
formatting, or unrelated bindings.

Strategy
--------
1. Parse the file into an AST.
2. Locate the assignment for the requested top-level name (e.g.
   ``GRAB_PAPER_CUP_PARAMS``) and walk into nested keys (e.g.
   ``GRAB_PAPER_CUP_PARAMS["7oz"]["approach"]``) until the target value node
   is found. Tuple/list elements can also be addressed via integer indices.
3. Use the value node's `lineno`, `col_offset`, `end_lineno`, `end_col_offset`
   to splice a freshly formatted Python literal back into the source. This
   preserves the original line's inline comment and surrounding whitespace.
4. Snapshot a backup, then write atomically (tempfile + os.replace), and
   finally validate with ``compile()`` to make sure we did not corrupt the
   module. On any failure the backup is restored.

Only Python literals (numbers, strings, booleans, None, and tuples/lists/
dicts of those) are accepted as new values.
"""

from __future__ import annotations

import ast
import io
import os
import shutil
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, List, Optional, Sequence, Tuple, Union

from .trace import gui_log, gui_log_error


KeyPart = Union[str, int]


@dataclass
class ResolvedTarget:
    name: str
    parts: Tuple[KeyPart, ...]
    value_node: ast.AST
    current_value: Any


@dataclass
class WriteResult:
    success: bool
    backup_path: Optional[Path]
    target_file: Path
    message: str


def default_params_path() -> Path:
    """Return ``oms_v1/params.py`` resolved relative to this file."""
    here = Path(__file__).resolve()
    return here.parent.parent / "params.py"


def backup_dir(params_path: Optional[Path] = None) -> Path:
    p = params_path or default_params_path()
    return p.parent / "params.py.bak"


def _safe_literal(node: ast.AST) -> Any:
    """ast.literal_eval but never raises -- returns ``None`` on failure."""
    try:
        return ast.literal_eval(node)
    except Exception:
        return None


def _walk_target(
    module_node: ast.Module, name: str, parts: Sequence[KeyPart]
) -> Optional[ast.AST]:
    """Locate the AST node for ``name[parts...]`` inside the module."""
    target_value: Optional[ast.AST] = None
    for stmt in module_node.body:
        if isinstance(stmt, ast.Assign):
            for tgt in stmt.targets:
                if isinstance(tgt, ast.Name) and tgt.id == name:
                    target_value = stmt.value
                    break
        if target_value is not None:
            break
    if target_value is None:
        return None

    node: ast.AST = target_value
    for part in parts:
        if isinstance(node, ast.Dict):
            found = None
            for key_node, val_node in zip(node.keys, node.values):
                key_val = _safe_literal(key_node) if key_node is not None else None
                if key_val == part:
                    found = val_node
                    break
            if found is None:
                return None
            node = found
        elif isinstance(node, (ast.Tuple, ast.List)):
            try:
                idx = int(part)
            except (TypeError, ValueError):
                return None
            if idx < 0 or idx >= len(node.elts):
                return None
            node = node.elts[idx]
        else:
            return None
    return node


def resolve_target(
    name: str,
    parts: Sequence[KeyPart],
    params_path: Optional[Path] = None,
) -> Optional[ResolvedTarget]:
    """Return the current value + AST node for ``name[parts...]``.

    Returns ``None`` if the name or any part cannot be located.
    """
    path = params_path or default_params_path()
    try:
        source = path.read_text(encoding="utf-8")
        module = ast.parse(source, filename=str(path))
    except Exception as exc:
        gui_log_error("GUI-PARAMS", f"failed to parse {path}", exc)
        return None
    node = _walk_target(module, name, tuple(parts))
    if node is None:
        return None
    current_value = _safe_literal(node)
    return ResolvedTarget(
        name=name,
        parts=tuple(parts),
        value_node=node,
        current_value=current_value,
    )


def get_value(
    name: str,
    parts: Sequence[KeyPart] = (),
    params_path: Optional[Path] = None,
) -> Any:
    target = resolve_target(name, parts, params_path)
    return None if target is None else target.current_value


def get_top_level_dict(
    name: str,
    params_path: Optional[Path] = None,
) -> Optional[dict]:
    target = resolve_target(name, (), params_path)
    if target is None:
        return None
    if isinstance(target.current_value, dict):
        return target.current_value
    return None


def format_key_path(name: str, parts: Sequence[KeyPart]) -> str:
    """Render ``name[parts...]`` as Python source for display."""
    out = name
    for part in parts:
        if isinstance(part, str):
            out += f"[{part!r}]"
        else:
            out += f"[{part}]"
    return out


def _is_pure_literal(value: Any) -> bool:
    """Return True if ``value`` only consists of types ast.literal_eval handles."""
    if value is None or isinstance(value, (bool, int, float, str, bytes)):
        return True
    if isinstance(value, (list, tuple, set, frozenset)):
        return all(_is_pure_literal(v) for v in value)
    if isinstance(value, dict):
        return all(_is_pure_literal(k) and _is_pure_literal(v) for k, v in value.items())
    return False


def _format_value(value: Any) -> str:
    """Render ``value`` as a Python literal that ast.literal_eval can read.

    Tuples are emitted with parentheses (and a trailing comma when single-
    element) so they stay tuples. Floats use ``repr`` for round-trip
    fidelity.
    """
    if isinstance(value, float):
        return repr(value)
    if isinstance(value, tuple):
        if len(value) == 1:
            return f"({_format_value(value[0])},)"
        return "(" + ", ".join(_format_value(v) for v in value) + ")"
    if isinstance(value, list):
        return "[" + ", ".join(_format_value(v) for v in value) + "]"
    if isinstance(value, dict):
        items = ", ".join(
            f"{_format_value(k)}: {_format_value(v)}" for k, v in value.items()
        )
        return "{" + items + "}"
    return repr(value)


def _splice_source(
    source: str, node: ast.AST, replacement: str
) -> Optional[str]:
    """Replace the source range of ``node`` with ``replacement``."""
    if not all(
        hasattr(node, attr)
        for attr in ("lineno", "col_offset", "end_lineno", "end_col_offset")
    ):
        return None
    lines = source.splitlines(keepends=True)
    start_line = node.lineno - 1
    end_line = node.end_lineno - 1
    start_col = node.col_offset
    end_col = node.end_col_offset

    if start_line == end_line:
        line = lines[start_line]
        new_line = line[:start_col] + replacement + line[end_col:]
        lines[start_line] = new_line
    else:
        prefix = lines[start_line][:start_col]
        suffix = lines[end_line][end_col:]
        new_block = prefix + replacement + suffix
        del lines[start_line + 1 : end_line + 1]
        lines[start_line] = new_block
    return "".join(lines)


def _ensure_backup_dir(params_path: Path) -> Path:
    bak = backup_dir(params_path)
    bak.mkdir(parents=True, exist_ok=True)
    return bak


def make_backup(params_path: Optional[Path] = None) -> Path:
    """Copy the current params.py to ``params.py.bak/<utc_iso>.py``."""
    path = params_path or default_params_path()
    bak_dir = _ensure_backup_dir(path)
    stamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
    target = bak_dir / f"{stamp}.py"
    counter = 0
    while target.exists():
        counter += 1
        target = bak_dir / f"{stamp}_{counter}.py"
    shutil.copy2(path, target)
    gui_log("GUI-PARAMS", f"backup created path={target}")
    return target


def _atomic_write(path: Path, contents: str) -> None:
    parent = path.parent
    fd, tmp_path = tempfile.mkstemp(prefix=path.name + ".", dir=str(parent))
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as fh:
            fh.write(contents)
        os.replace(tmp_path, path)
    except Exception:
        try:
            os.unlink(tmp_path)
        except FileNotFoundError:
            pass
        raise


def update_value(
    name: str,
    parts: Sequence[KeyPart],
    new_value: Any,
    params_path: Optional[Path] = None,
) -> WriteResult:
    """Update ``name[parts...]`` to ``new_value`` in params.py.

    Performs:
        - backup snapshot
        - AST-located source splice
        - atomic write
        - compile() validation
        - automatic restore on failure
    """
    path = params_path or default_params_path()
    if not _is_pure_literal(new_value):
        return WriteResult(
            success=False,
            backup_path=None,
            target_file=path,
            message="rejected: new value is not a pure Python literal",
        )

    try:
        source = path.read_text(encoding="utf-8")
    except Exception as exc:
        gui_log_error("GUI-PARAMS", f"failed to read {path}", exc)
        return WriteResult(False, None, path, f"read failed: {exc}")

    try:
        module = ast.parse(source, filename=str(path))
    except SyntaxError as exc:
        gui_log_error("GUI-PARAMS", "params.py already has a SyntaxError", exc)
        return WriteResult(False, None, path, f"existing syntax error: {exc}")

    node = _walk_target(module, name, tuple(parts))
    if node is None:
        return WriteResult(
            False,
            None,
            path,
            f"target not found: {format_key_path(name, parts)}",
        )

    replacement = _format_value(new_value)
    new_source = _splice_source(source, node, replacement)
    if new_source is None:
        return WriteResult(
            False, None, path, "splice failed: AST node missing position info"
        )

    backup = make_backup(path)
    try:
        _atomic_write(path, new_source)
    except Exception as exc:
        gui_log_error("GUI-PARAMS", "atomic write failed", exc)
        try:
            shutil.copy2(backup, path)
        except Exception as restore_exc:
            gui_log_error("GUI-PARAMS", "restore failed", restore_exc)
        return WriteResult(False, backup, path, f"write failed: {exc}")

    try:
        compile(path.read_text(encoding="utf-8"), str(path), "exec")
    except SyntaxError as exc:
        gui_log_error("GUI-PARAMS", "post-write compile failed", exc)
        try:
            shutil.copy2(backup, path)
        except Exception as restore_exc:
            gui_log_error("GUI-PARAMS", "restore failed", restore_exc)
            return WriteResult(
                False,
                backup,
                path,
                f"compile failed AND restore failed: {restore_exc}",
            )
        return WriteResult(
            False,
            backup,
            path,
            f"post-write compile failed -- restored from backup: {exc}",
        )

    gui_log(
        "GUI-PARAMS",
        f"updated {format_key_path(name, parts)} -> {replacement[:200]}",
    )
    return WriteResult(
        True,
        backup,
        path,
        f"updated {format_key_path(name, parts)}",
    )


def list_backups(params_path: Optional[Path] = None) -> List[Path]:
    bak_dir = backup_dir(params_path)
    if not bak_dir.exists():
        return []
    return sorted(bak_dir.glob("*.py"))


def restore_backup(
    backup_file: Path, params_path: Optional[Path] = None
) -> WriteResult:
    """Restore params.py from a previously-taken backup."""
    path = params_path or default_params_path()
    backup_file = Path(backup_file)
    if not backup_file.exists():
        return WriteResult(False, None, path, f"backup not found: {backup_file}")
    pre_restore = make_backup(path)
    try:
        shutil.copy2(backup_file, path)
    except Exception as exc:
        gui_log_error("GUI-PARAMS", "restore copy failed", exc)
        return WriteResult(False, pre_restore, path, f"restore failed: {exc}")
    try:
        compile(path.read_text(encoding="utf-8"), str(path), "exec")
    except SyntaxError as exc:
        try:
            shutil.copy2(pre_restore, path)
        except Exception:
            pass
        return WriteResult(
            False,
            pre_restore,
            path,
            f"restored file has syntax error -- reverted: {exc}",
        )
    return WriteResult(True, pre_restore, path, f"restored from {backup_file.name}")
