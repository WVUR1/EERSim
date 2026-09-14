"""Conservative, read-only discovery of PROS motor tuning targets.

This is a source scanner, not a C++ compiler or a hardware inventory. It resolves
literal integer ports, const/constexpr integer aliases, object-like #defines,
literal port vectors, PROS Motor/MotorGroup constructors and EERKit TankDrive /
XDrivetrain group references. It never executes the workspace. Conditional code
(except a conventional header guard), computed ports, factories, templates,
arbitrary wrappers, runtime reversal and preprocessor expansion are not inferred.
Resolved targets are ``verified`` against source syntax only; a runtime mapping
check is still required before commanding motors. Unresolved targets produce
warnings rather than guessed ports. Namespace and local scopes remain distinct.

Axes contain coefficients in logical signed-port order. X-drive order is EERKit
front-left, front-right, back-left, back-right. Manually entered H-drive order is
left, right, strafe; larger groups must supply explicit coefficient arrays.
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import re
from typing import Any


_KINDS = {"motor", "motor_group", "tank", "h_drive", "x_drive"}
_SUFFIXES = {".c", ".cc", ".cpp", ".cxx", ".h", ".hh", ".hpp", ".hxx"}
_SKIP = {".git", ".sim-tools", ".pros", "build", "bin", "dist", "node_modules",
         "__pycache__", "venv", ".venv", "pros", "okapi", "third_party", "vendor"}
_LEX = re.compile(
    r'//[^\n]*|/\*[\s\S]*?\*/|R"(?P<raw>[^\s()\\]{0,16})\([\s\S]*?\)(?P=raw)"'
    r'|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\''
    r'|[A-Za-z_]\w*|0[xX][0-9a-fA-F]+[uUlL]*|0[bB][01]+[uUlL]*|\d+[uUlL]*|::|[^\s]')
_IDENT = re.compile(r"[A-Za-z_]\w*\Z")
_MOTOR_ENUM = re.compile(r"(?:(?:pros::)?(?:v5::)?(?:MotorGearset|MotorGears|MotorUnits)::[A-Za-z_]\w*|(?:pros::)?E_MOTOR_(?:GEARSET|ENCODER)_[A-Za-z_0-9]+)\Z")


def make_target(name: str, kind: str, ports: list[int],
                axes: dict[str, list[float]] | None = None,
                source: str = "manual", verified: bool = False) -> dict[str, Any]:
    """Validate a target without touching code, hardware or simulation state."""
    if not isinstance(name, str) or not name.strip():
        raise ValueError("A tuning target needs a name")
    if kind not in _KINDS:
        raise ValueError(f"Unsupported target kind: {kind}")
    ports = list(ports)
    if not ports or any(isinstance(p, bool) or not isinstance(p, int)
                        or not 1 <= abs(p) <= 21 for p in ports):
        raise ValueError("Ports must be signed PROS integers from -21 to 21, excluding zero")
    if len({abs(p) for p in ports}) != len(ports):
        raise ValueError("A target cannot contain the same physical motor port twice")
    if kind == "motor" and len(ports) != 1:
        raise ValueError("A single motor target must contain exactly one port")
    if axes is None:
        if kind in {"motor", "motor_group"}:
            axes = {"shaft": [1.0] * len(ports)}
        elif kind == "tank" and len(ports) == 2:
            axes = {"forward": [1, 1], "turn": [1, -1]}
        elif kind == "h_drive" and len(ports) == 3:
            axes = {"forward": [1, 1, 0], "turn": [1, -1, 0], "strafe": [0, 0, 1]}
        elif kind == "x_drive" and len(ports) == 4:
            axes = {"forward": [1, 1, 1, 1], "turn": [1, -1, 1, -1],
                    "strafe": [1, -1, -1, 1]}
        else:
            raise ValueError("Supply explicit axis coefficients for a drivetrain with grouped motors")
    if not isinstance(axes, dict) or not axes:
        raise ValueError("At least one tuning axis is required")
    clean_axes = {}
    for axis, values in axes.items():
        if not isinstance(axis, str) or not axis.strip() or not isinstance(values, (list, tuple)):
            raise ValueError("Axes need names and lists of coefficients")
        if len(values) != len(ports):
            raise ValueError("Each axis must have one coefficient per port")
        try:
            coefficients = [float(value) for value in values]
        except (TypeError, ValueError) as exc:
            raise ValueError("Axis coefficients must be finite numbers") from exc
        if not all(math.isfinite(value) and abs(value) <= 1 for value in coefficients):
            raise ValueError("Axis coefficients must be finite and between -1 and 1")
        if not any(coefficients):
            raise ValueError("Each axis must move at least one motor")
        clean_axes[axis.strip()] = coefficients
    identity = json.dumps([source, name.strip(), kind, ports, clean_axes], sort_keys=True)
    return {"id": hashlib.sha256(identity.encode()).hexdigest()[:20], "name": name.strip(),
            "kind": kind, "ports": ports, "axes": clean_axes, "source": str(source),
            "verified": bool(verified), "notes": []}


@dataclass
class _Token:
    text: str
    pos: int
    scope: tuple[str, ...] = ()


@dataclass
class _Symbol:
    name: str
    scope: tuple[str, ...]
    file: str
    line: int
    kind: str
    args: list[list[str]]
    style: str = "("
    value: Any = None
    error: str = ""
    internal: bool = False

    @property
    def source(self) -> str:
        return f"{self.file}:{self.line}"

    @property
    def label(self) -> str:
        names = [part for part in self.scope if not part.startswith("@")]
        return "::".join([*names, self.name])


def _lex(text: str) -> list[_Token]:
    return [_Token(m.group(), m.start()) for m in _LEX.finditer(text)
            if not m.group().startswith(('//', '/*', '"', "'", 'R"'))]


def _split(tokens: list[str]) -> list[list[str]]:
    result, start, stack = [], 0, []
    for index, token in enumerate(tokens):
        if token in {"(", "{", "[", "<"}:
            stack.append(token)
        elif token in {")", "}", "]", ">"} and stack:
            stack.pop()
        elif token == "," and not stack:
            result.append(tokens[start:index])
            start = index + 1
    if tokens[start:]:
        result.append(tokens[start:])
    return result


def _closing(tokens: list[str], start: int) -> int:
    closer = {"(": ")", "{": "}", "[": "]", "<": ">"}[tokens[start]]
    depth = 1
    for index in range(start + 1, len(tokens)):
        if tokens[index] == tokens[start]:
            depth += 1
        elif tokens[index] == closer:
            depth -= 1
            if depth == 0:
                return index
    raise ValueError("Unclosed initializer")


def _prepare(text: str, file: str, warnings: list[str]) -> tuple[str, list[_Symbol]]:
    """Mask directives and conditional branches; preserve offsets and line numbers."""
    lines = text.splitlines(keepends=True)
    # Directives inside comments / raw string literals are not C++ directives.
    view = list(text)
    for match in _LEX.finditer(text):
        if match.group().startswith(('//', '/*', '"', "'", 'R"')):
            view[match.start():match.end()] = ["\n" if char == "\n" else " " for char in match.group()]
    visible_lines = "".join(view).splitlines(keepends=True)
    directive_lines = [(i, re.match(r"\s*#\s*(\w+)\s*(.*)", line))
                       for i, line in enumerate(visible_lines)]
    directives = [(i, m.group(1), m.group(2).strip()) for i, m in directive_lines if m
                  and not (m.group(1) == "pragma" and m.group(2).strip() == "once")]
    guard = None
    if len(directives) >= 2:
        first, second = directives[:2]
        if first[1] == "ifndef" and second[1:] == ("define", first[2]):
            guard = first[2]
    active, macros, warned = [], [], False
    for index, line in enumerate(lines):
        match = re.match(r"\s*#\s*(\w+)\s*(.*)", visible_lines[index])
        masked = any(active)
        if match:
            directive, body = match.groups()
            if directive in {"if", "ifdef", "ifndef"}:
                active.append(not (directive == "ifndef" and body.strip() == guard and not active))
            elif directive == "endif" and active:
                active.pop()
            elif directive in {"else", "elif"} and active:
                # A header guard's else arm is not part of its declarations.
                active[-1] = True
            elif directive == "define" and not masked:
                define = re.fullmatch(r"([A-Za-z_]\w*)(\s+.*)?", body.strip())
                if define and define.group(2):
                    value = [t.text for t in _lex(define.group(2))]
                    macros.append(_Symbol(define.group(1), (), file, index + 1, "constant", [value]))
            lines[index] = "".join("\n" if char == "\n" else " " for char in line)
        elif masked:
            if re.search(r"\b(?:Motor|MotorGroup|XDrivetrain|TankDrive|HDrive|HDrivetrain)\b", line) and not warned:
                warnings.append(f"{file}:{index + 1}: conditional motor declarations were skipped; build flags are not evaluated")
                warned = True
            lines[index] = "".join("\n" if char == "\n" else " " for char in line)
    return "".join(lines), macros


def _scope_tokens(tokens: list[_Token], file: str) -> None:
    scope: tuple[str, ...] = ()
    stack, segment = [], 0
    for index, token in enumerate(tokens):
        token.scope = scope
        if token.text == "{":
            preceding = [t.text for t in tokens[segment:index]]
            stack.append(scope)
            if "namespace" in preceding:
                tail = preceding[preceding.index("namespace") + 1:]
                names = [item for item in tail if _IDENT.fullmatch(item)]
                scope += tuple(names) if names else (f"@anonymous:{file}",)
            else:
                scope += (f"@{file}:{token.pos}",)
            segment = index + 1
        elif token.text == "}":
            scope = stack.pop() if stack else ()
            segment = index + 1
        elif token.text == ";":
            segment = index + 1


def _symbols(text: str, file: str, warnings: list[str]) -> list[_Symbol]:
    text, symbols = _prepare(text, file, warnings)
    tokens = _lex(text)
    _scope_tokens(tokens, file)
    words = [token.text for token in tokens]
    type_specs = {"pros::Motor": "motor", "pros::v5::Motor": "motor",
                  "pros::MotorGroup": "motor_group", "pros::v5::MotorGroup": "motor_group",
                  "eerkit::TankDrive": "tank", "eerkit::XDrivetrain": "x_drive",
                  "eerkit::HDrive": "h_drive", "eerkit::HDrivetrain": "h_drive"}
    imports = []
    for position, word in enumerate(words):
        if word != "using":
            continue
        end = position + 1
        while end < min(position + 12, len(words)) and words[end] != ";":
            end += 1
        imports.append((tokens[position].scope, "".join(words[position + 1:end])))
    index = 0
    while index < len(tokens):
        token = tokens[index]
        # Mutable port vectors are common PROS constructor arguments. Discover
        # their initial values but never claim to track subsequent mutations.
        if words[index:index + 4] in (["std", "::", "vector", "<"], ["std", "::", "array", "<"]):
            try:
                end = _closing(words, index + 3) + 1
                if end + 1 < len(words) and _IDENT.fullmatch(words[end]) and words[end + 1] in {"=", "{"}:
                    start = end + 1 + (words[end + 1] == "=")
                    if words[start] == "{":
                        finish = _closing(words, start)
                        symbols.append(_Symbol(words[end], token.scope, file, text.count("\n", 0, token.pos) + 1,
                                               "constant", [words[start:finish + 1]]))
            except (ValueError, IndexError):
                pass
        if token.text in {"constexpr", "const"}:
            end = index + 1
            while end < min(index + 150, len(words)) and words[end] not in {";", "=" , "{"}:
                end += 1
            if end < len(words) and words[end] in {"=", "{"} and end > index + 1:
                name = words[end - 1]
                prefix = words[index + 1:end - 1]
                scalar = bool(prefix) and all(p in {"const", "constexpr", "static", "inline", "unsigned", "signed",
                            "int", "long", "short", "char", "bool", "auto", "std", "::", "int8_t", "uint8_t",
                            "int16_t", "uint16_t", "int32_t", "uint32_t", "size_t"} for p in prefix)
                vector = "vector" in prefix or "array" in prefix
                if _IDENT.fullmatch(name) and (scalar or vector):
                    try:
                        if words[end] == "{":
                            finish = _closing(words, end)
                            expr = words[end:finish + 1] if vector else words[end + 1:finish]
                        else:
                            finish = end + 1
                            while finish < len(words) and words[finish] != ";":
                                finish += 1
                            expr = words[end + 1:finish]
                        symbols.append(_Symbol(name, token.scope, file, text.count("\n", 0, token.pos) + 1,
                                               "constant", [expr]))
                    except ValueError:
                        pass
        if index and words[index - 1] == "::":
            index += 1
            continue
        matched = None
        available_types = dict(type_specs)
        for qualified, kind in type_specs.items():
            namespace, basename = qualified.rsplit("::", 1)
            if any(token.scope[:len(scope)] == scope and imported in {qualified, "namespace" + namespace}
                   for scope, imported in imports):
                available_types[basename] = kind
        for typename, kind in available_types.items():
            pattern = typename.replace("::", " :: ").split()
            if words[index:index + len(pattern)] == pattern:
                matched = (kind, index + len(pattern))
                break
        if matched:
            kind, cursor = matched
            if cursor < len(words) and _IDENT.fullmatch(words[cursor]):
                name, cursor = words[cursor], cursor + 1
                if cursor < len(words) and words[cursor] == "=":
                    cursor += 1
                    # Copy construction / ``= pros::Motor(...)``.
                    while cursor < len(words) and words[cursor] not in {"(", "{", ";"}:
                        cursor += 1
                if cursor < len(words) and words[cursor] in {"(", "{"}:
                    try:
                        finish = _closing(words, cursor)
                        # Function definitions and constructor declarations are not objects.
                        if finish + 1 < len(words) and words[finish + 1] in {";", ","}:
                            symbols.append(_Symbol(name, token.scope, file, text.count("\n", 0, token.pos) + 1,
                                                   kind, _split(words[cursor + 1:finish]), words[cursor]))
                            index = finish
                    except ValueError as exc:
                        warnings.append(f"{file}: {name}: {exc}")
        index += 1
    # A constexpr const declaration is scanned twice; retain its one identity.
    unique = {}
    for symbol in symbols:
        # Explicit static declarations have file-local linkage. This prevents
        # another translation unit from borrowing unrelated private motors.
        prefix = text.splitlines()[symbol.line - 1].split(symbol.name, 1)[0]
        symbol.internal = bool(re.search(r"\bstatic\b", prefix))
        unique[(symbol.file, symbol.scope, symbol.name, symbol.line, symbol.kind)] = symbol
    return list(unique.values())


class _Resolver:
    def __init__(self, symbols: list[_Symbol]):
        self.symbols = symbols

    def lookup(self, expr: list[str], owner: _Symbol, trail: set[int]) -> Any:
        name = "".join(expr).removeprefix("&")
        if not re.fullmatch(r"(?:::)?[A-Za-z_]\w*(?:::[A-Za-z_]\w*)*", name):
            raise ValueError("computed expressions and factories are not inferred")
        parts = name.removeprefix("::").split("::")
        prefixes = [()] if name.startswith("::") else [owner.scope[:i] for i in range(len(owner.scope), -1, -1)]
        for prefix in prefixes:
            scope, basename = prefix + tuple(parts[:-1]), parts[-1]
            candidates = [s for s in self.symbols if s.name == basename and s.scope == scope]
            local = [s for s in candidates if s.file == owner.file]
            candidates = local or [s for s in candidates if not s.internal and not any(p.startswith("@") for p in s.scope)]
            if not candidates:
                continue
            values = [self.resolve(s, trail) for s in candidates]
            if any(value != values[0] for value in values[1:]):
                raise ValueError(f"ambiguous definition of {name}")
            return values[0]
        raise ValueError(f"unresolved reference {name}")

    def expression(self, expr: list[str], owner: _Symbol, trail: set[int]) -> Any:
        expr = list(expr)
        while len(expr) >= 2 and expr[0] == "(" and _closing(expr, 0) == len(expr) - 1:
            expr = expr[1:-1]
        if not expr:
            raise ValueError("empty port initializer")
        if expr[0] == "{" and _closing(expr, 0) == len(expr) - 1:
            values = [self.expression(part, owner, trail) for part in _split(expr[1:-1])]
            return values[0] if len(values) == 1 and isinstance(values[0], list) else values
        if expr[0] in {"+", "-"}:
            value = self.expression(expr[1:], owner, trail)
            if isinstance(value, bool) or not isinstance(value, int):
                raise ValueError("port sign requires an integer")
            return value if expr[0] == "+" else -value
        if len(expr) == 1:
            if expr[0] in {"true", "false"}:
                return expr[0] == "true"
            literal = re.fullmatch(r"(0[xX][0-9a-fA-F]+|0[bB][01]+|\d+)[uUlL]*", expr[0])
            if literal:
                value = literal.group(1)
                return int(value, 8 if len(value) > 1 and value.startswith("0") and value[1].isdigit() else 0 if value.startswith(("0x", "0X", "0b", "0B")) else 10)
        # Explicit vector temporary: std::vector<std::int8_t>{1, -2}.
        if "vector" in expr or "array" in expr:
            if "{" in expr:
                return self.expression(expr[expr.index("{"):], owner, trail)
        return self.lookup(expr, owner, trail)

    def resolve(self, symbol: _Symbol, trail: set[int] | None = None) -> Any:
        if symbol.value is not None:
            return symbol.value
        trail = set(trail or ())
        if id(symbol) in trail:
            raise ValueError(f"cyclic definition of {symbol.name}")
        trail.add(id(symbol))
        if symbol.kind == "constant":
            symbol.value = self.expression(symbol.args[0], symbol, trail)
            return symbol.value
        if not symbol.args:
            raise ValueError("no constructor arguments")
        if symbol.kind in {"motor", "motor_group"}:
            value = self.expression(symbol.args[0], symbol, trail)
            if isinstance(value, dict):
                value = value["ports"]
            if symbol.kind == "motor_group" and symbol.style == "{" and not isinstance(value, list):
                # Direct initializer-list construction: MotorGroup group{1, -2}.
                value = [self.expression(arg, symbol, trail) for arg in symbol.args]
            ports = list(value) if isinstance(value, list) else [value]
            if symbol.kind == "motor":
                # Legacy PROS has a reversed bool in argument 2 or 3. Gearset /
                # units identifiers must never be guessed to be booleans.
                for offset in (1, 2):
                    if len(symbol.args) <= offset:
                        continue
                    argument = symbol.args[offset]
                    if _MOTOR_ENUM.fullmatch("".join(argument)):
                        continue
                    try:
                        reverse = self.expression(argument, symbol, trail)
                    except ValueError as exc:
                        raise ValueError("unresolved constructor reversal/gearset/units; use an explicit supported enum or constant bool") from exc
                    if isinstance(reverse, bool):
                        # Signed ports and legacy reversed flag both reverse direction.
                        if reverse:
                            ports = [-port for port in ports]
                        break
                    raise ValueError("constructor reversal must be a resolved bool; enum conversions are not inferred")
            symbol.value = make_target(symbol.label, symbol.kind, ports, source=symbol.source, verified=True)
        else:
            count = {"tank": 2, "h_drive": 3, "x_drive": 4}[symbol.kind]
            if len(symbol.args) < count:
                raise ValueError(f"{symbol.kind} requires {count} motor-group references")
            groups = []
            for argument in symbol.args[:count]:
                group = self.expression(argument, symbol, trail)
                if not isinstance(group, dict) or group.get("kind") not in {"motor", "motor_group"}:
                    raise ValueError("drivetrain arguments must reference known motors or motor groups")
                groups.append(group["ports"])
            pattern = {"tank": {"forward": [1, 1], "turn": [1, -1]},
                       "h_drive": {"forward": [1, 1, 0], "turn": [1, -1, 0], "strafe": [0, 0, 1]},
                       "x_drive": {"forward": [1, 1, 1, 1], "turn": [1, -1, 1, -1], "strafe": [1, -1, -1, 1]}}[symbol.kind]
            axes = {axis: [float(coefficient) for coefficient, group in zip(coefficients, groups) for _ in group]
                    for axis, coefficients in pattern.items()}
            symbol.value = make_target(symbol.label, symbol.kind, [p for group in groups for p in group], axes,
                                       symbol.source, verified=symbol.kind != "h_drive")
            if symbol.kind == "h_drive":
                symbol.value["notes"].append("Conventional H-drive constructor order assumed: left, right, strafe. Verify axis directions before tuning.")
        if any(part.startswith("@") for part in symbol.scope):
            symbol.value["name"] += f" ({symbol.source})"
            symbol.value["notes"].append("Local declaration; availability depends on the program's execution path.")
        return symbol.value


def discover_targets(workspace: Path | str, config: dict[str, Any] | None = None) -> dict[str, Any]:
    """Return deterministic code targets plus clearly labelled simulation fallbacks."""
    warnings: list[str] = []
    symbols: list[_Symbol] = []
    root = Path(workspace).expanduser() if workspace else None
    if root and root.is_dir():
        roots = [root / name for name in ("src", "include") if (root / name).is_dir()]
        files = set()
        for base in roots or [root]:
            # Prune excluded trees before visiting them; build environments can
            # contain hundreds of thousands of files on an imported workspace.
            for directory, subdirectories, names in os.walk(base, followlinks=False):
                subdirectories[:] = [name for name in subdirectories if name.lower() not in _SKIP
                                     and not (Path(directory) / name).is_symlink()]
                files.update(Path(directory) / name for name in names if Path(name).suffix.lower() in _SUFFIXES)
                if len(files) > 5000:
                    warnings.append("Source discovery stopped at 5000 files; select a narrower PROS workspace")
                    break
            if len(files) > 5000:
                break
        for path in sorted(files)[:5000]:
            if path.is_symlink() or not path.resolve().is_relative_to(root.resolve()):
                warnings.append(f"Skipped source outside workspace: {path.name}")
                continue
            relative = path.relative_to(root).as_posix()
            try:
                if path.stat().st_size > 2_000_000:
                    warnings.append(f"{relative}: skipped source larger than 2 MB")
                    continue
                symbols.extend(_symbols(path.read_text(encoding="utf-8-sig", errors="replace"), relative, warnings))
            except OSError as exc:
                warnings.append(f"{relative}: could not read source ({exc})")
    elif workspace:
        warnings.append("The selected PROS workspace does not exist or is not a directory")
    else:
        warnings.append("Select a PROS workspace to discover code-defined motors and groups")
    resolver, targets = _Resolver(symbols), []
    for symbol in symbols:
        if symbol.kind == "constant":
            continue
        try:
            targets.append(resolver.resolve(symbol))
        except (ValueError, TypeError, OverflowError, RecursionError) as exc:
            warnings.append(f"{symbol.source}: {symbol.label}: {exc}")
    config = config or {}
    bindings = config.get("motors")
    if bindings is None and config.get("robot_config"):
        try:
            from .config import _resolve_path
            path = _resolve_path(config["robot_config"], root or Path.cwd())
            bindings = json.loads(path.read_text(encoding="utf-8"))["motors"]
        except (OSError, ValueError, KeyError, TypeError) as exc:
            warnings.append(f"Could not read robot motor bindings: {exc}")
    mapped = {abs(port) for target in targets for port in target["ports"]}
    for binding in bindings if isinstance(bindings, list) else []:
        if not isinstance(binding, dict):
            warnings.append("Skipped malformed simulation motor binding")
            continue
        port = binding.get("port")
        if isinstance(port, int) and abs(port) in mapped:
            continue
        try:
            target = make_target(f"{binding.get('joint', 'Motor')} (simulation port {port})", "motor", [port],
                                 source="simulation mapping", verified=False)
            target["notes"].append("Robot binding only; this motor was not discovered in the imported code.")
            targets.append(target)
            mapped.add(abs(port))
        except (ValueError, TypeError) as exc:
            warnings.append(f"Invalid simulation motor binding: {exc}")
    return {"targets": targets, "warnings": list(dict.fromkeys(warnings))}
