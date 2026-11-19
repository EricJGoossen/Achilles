#!/usr/bin/env python3
# Checks the rules in docs/conventions/ that a script can check exactly:
# include direction, folder structure, algorithm files, names, namespaces,
# forbidden code, required static_asserts, and that the docs' links and
# file tree match the files. Errors fail the run; warnings are heuristics
# that can misfire and never fail it. Run by hand, beside check-tidy.sh;
# nothing runs it automatically.
#
# The layer tables are read from docs/conventions/{domain,policy,engine}.md,
# so the docs stay the one source of truth for layer order.
#
# Usage: scripts/check-conventions.py [--since REV] [--only RULE,...]
#                                     [--list] [--no-warnings]
#   --since REV  report only findings in files changed since git revision
#                REV (committed, staged, unstaged or untracked)
#   --only       run only the named rules (see --list)

import argparse
import os
import re
import subprocess
import sys
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

# The include stack, lowest first. `achilles` holds the generated build
# config, which anything may include.
STACK = ["achilles", "util", "domain", "policy", "engine", "algorithms",
         "interface"]
LAYERED = ["domain", "policy", "engine"]
# Top-level folders the code checks skip: display/ has its own design and
# is moving to its own repo.
SKIPPED = {"display"}
# Shared top-level folders in algorithms/ that algorithm folders may
# include, and top-level files they may include.
SHARED_FOLDERS = {"row_sets"}
SHARED_FILES = {"conventions.hpp", "shared_slots.hpp"}

SOURCE_SUFFIXES = (".hpp", ".h", ".cpp", ".hpp.in")

RULES = {
    # errors
    "layer-stack": "includes only point down the util -> interface stack",
    "layer-table": "within domain/policy/engine, includes follow the layer "
                   "table, with no peers",
    "structure": "no loose files in domain/, policy/ or engine/; engine/ is "
                 "one level deep",
    "src-mirror": "every src/ .cpp has a header at the matching include/ path",
    "algorithm-files": "an algorithm folder holds 1-3 files: <folder>_data, "
                       "_ops, _step",
    "algorithm-includes": "algorithm folders never include each other or the "
                          "top-level lists",
    "unique-names": "no two source files share a name, apart from a header "
                    "and its .cpp",
    "namespace": "a file's namespace matches its folder, or folder + file "
                 "name; detail allowed",
    "concept-names": "concept names end in Like",
    "forbidden": "no NOLINT, virtual, override, std::function, std::variant "
                 "or std::any",
    "throw": "throw only in interface/",
    "step-control-flow": "step files have no for, while or plain if",
    "op-asserts": "every op struct has its OpLike static_assert",
    "policy-asserts": "every policy file has its concept static_assert",
    "concept-archetypes": "every concept has a <Name>Archetype in tests/",
    "file-tree": "docs/file-tree.md lists every header, and only real files",
    "doc-links": "every Doc: link exists, and every docs/code/ file is "
                 "linked",
    "md-links": "every link in CLAUDE.md and docs/ resolves",
    "file-comment": "every source file starts with a comment",
    # warnings
    "missing-test": "a header with no test of the matching name",
    "stale-test": "a test whose name matches no header or example",
    "loop-allocation": "allocation in op or step code",
    "comment-history": "comments that cite line numbers or history",
}
WARNINGS = {"missing-test", "stale-test", "loop-allocation",
            "comment-history"}


class Findings:
    def __init__(self):
        self.items = []

    def add(self, rule, path, line, message):
        self.items.append((rule, Path(path), line, message))


def rel(path):
    return Path(path).resolve().relative_to(ROOT)


def git_lines(*args):
    out = subprocess.run(["git", *args], cwd=ROOT, capture_output=True,
                         text=True, check=True).stdout
    return [line for line in out.splitlines() if line]


def changed_since(rev):
    files = set(git_lines("diff", "--name-only", rev))
    files |= set(git_lines("ls-files", "--others", "--exclude-standard"))
    return {Path(f) for f in files}


def source_files():
    files = []
    for top in ("include", "src", "tests", "examples"):
        for path in sorted((ROOT / top).rglob("*")):
            if not path.is_file() or not str(path).endswith(SOURCE_SUFFIXES):
                continue
            r = rel(path)
            if r.parts[:2] == ("tests", "consumer"):
                continue
            files.append(r)
    return files


def top_folder(r):
    # The top-level code folder of an include/ or src/ path, or None.
    if r.parts[0] in ("include", "src") and len(r.parts) > 2:
        return r.parts[1]
    return None


def is_code(r):
    return top_folder(r) is not None and top_folder(r) not in SKIPPED


def strip_code(text):
    # Blanks out comments and string and char literals, keeping newlines, so
    # line numbers still match. Handles raw strings.
    out = []
    i, n = 0, len(text)
    while i < n:
        c = text[i]
        if text.startswith("//", i):
            j = text.find("\n", i)
            j = n if j < 0 else j
            out.append(" " * (j - i))
            i = j
        elif text.startswith("/*", i):
            j = text.find("*/", i + 2)
            j = n if j < 0 else j + 2
            out.append(re.sub(r"[^\n]", " ", text[i:j]))
            i = j
        elif c == "R" and text.startswith('R"', i):
            m = re.match(r'R"([^(\s]*)\(', text[i:])
            if not m:
                out.append(c)
                i += 1
                continue
            end = text.find(")" + m.group(1) + '"', i)
            j = n if end < 0 else end + len(m.group(1)) + 2
            out.append(re.sub(r"[^\n]", " ", text[i:j]))
            i = j
        elif c in "\"'":
            j = i + 1
            while j < n and text[j] != c and text[j] != "\n":
                j += 2 if text[j] == "\\" else 1
            j = min(j + 1, n)
            out.append(c + " " * (j - i - 2) + c if j - i >= 2 else c)
            i = j
        else:
            out.append(c)
            i += 1
    return "".join(out)


def comments_of(text):
    # (line, comment text) for every // comment.
    result = []
    for number, line in enumerate(text.splitlines(), 1):
        m = re.search(r"//(.*)$", line)
        if m and '"' not in line[: m.start()]:
            result.append((number, m.group(1)))
    return result


def line_of(text, index):
    return text.count("\n", 0, index) + 1


def read_layer_tables():
    tables = {}
    for top in LAYERED:
        doc = ROOT / "docs" / "conventions" / f"{top}.md"
        layers = {}
        if doc.exists():
            for m in re.finditer(r"^\|\s*(\d+)\s*\|(.*)\|\s*$",
                                 doc.read_text(), re.M):
                for folder in re.findall(r"`([^`]+)`", m.group(2)):
                    layers[folder] = int(m.group(1))
        tables[top] = layers
    return tables


def resolve_include(r, target):
    # The repo path an #include "..." names, or None for a system or
    # third-party header.
    beside = (ROOT / r).parent / target
    if beside.exists():
        return rel(beside)
    for base in ("include", "src"):
        candidate = ROOT / base / target
        if candidate.exists():
            return rel(candidate)
    return None


def includes_of(r, text):
    for number, line in enumerate(text.splitlines(), 1):
        m = re.match(r'\s*#\s*include\s*"([^"]+)"', line)
        if m:
            yield number, resolve_include(r, m.group(1))


def check_layers(r, text, tables, f):
    top = top_folder(r)
    if top not in STACK:
        return
    for number, target in includes_of(r, text):
        if target is None:
            continue
        other = top_folder(target)
        if other is None or other in SKIPPED:
            continue
        if other not in STACK or STACK.index(other) > STACK.index(top):
            f.add("layer-stack", r, number,
                  f"{top}/ includes {other}/ ({target})")
            continue
        if other == top and top in LAYERED and len(r.parts) > 3 \
                and len(target.parts) > 3:
            mine, theirs = r.parts[2], target.parts[2]
            layers = tables[top]
            if mine == theirs or mine not in layers or theirs not in layers:
                continue
            if layers[theirs] >= layers[mine]:
                f.add("layer-table", r, number,
                      f"{top}/{mine} (layer {layers[mine]}) includes "
                      f"{top}/{theirs} (layer {layers[theirs]})")


def check_structure(r, f):
    top = top_folder(r)
    if r.parts[0] != "include" or top not in LAYERED:
        return
    if len(r.parts) == 3:
        f.add("structure", r, 1, f"file sits directly in {top}/")
    if top == "engine" and len(r.parts) > 4:
        f.add("structure", r, 1, "engine/ allows one level of subfolders")


def check_src_mirror(r, f):
    if r.parts[0] != "src" or r.suffix != ".cpp" or not is_code(r):
        return
    header = ROOT / "include" / Path(*r.parts[1:]).with_suffix(".hpp")
    if not header.exists():
        f.add("src-mirror", r, 1, f"no header at {rel(header)}")


def algorithm_folders():
    base = ROOT / "include" / "algorithms"
    folders = {}
    for d in sorted(p for p in base.iterdir() if p.is_dir()):
        names = [p.name for p in d.iterdir() if p.is_file()]
        if any(n.startswith(d.name + "_") for n in names):
            folders[d.name] = names
    return folders


def check_algorithm_files(f):
    allowed = ("_data.hpp", "_ops.hpp", "_step.hpp")
    for folder, names in algorithm_folders().items():
        path = Path("include") / "algorithms" / folder
        if not 1 <= len(names) <= 3:
            f.add("algorithm-files", path, 1,
                  f"{len(names)} files; an algorithm has 1-3")
        for name in names:
            if name not in {folder + s for s in allowed}:
                f.add("algorithm-files", path / name, 1,
                      f"not one of {folder}_data/_ops/_step.hpp")


def check_algorithm_includes(r, text, algorithms, f):
    if top_folder(r) != "algorithms" or len(r.parts) < 4:
        return
    mine = r.parts[2]
    if mine not in algorithms:
        return
    for number, target in includes_of(r, text):
        if target is None or top_folder(target) != "algorithms":
            continue
        if len(target.parts) == 3:
            if target.name not in SHARED_FILES:
                f.add("algorithm-includes", r, number,
                      f"includes the top-level {target.name}")
        elif target.parts[2] != mine and target.parts[2] not in \
                SHARED_FOLDERS:
            what = "another algorithm" if target.parts[2] in algorithms \
                else "a top-level folder other than row_sets/"
            f.add("algorithm-includes", r, number,
                  f"includes {what}: {target}")


def check_unique_names(files, f):
    by_stem = defaultdict(list)
    for r in files:
        stem = r.name.split(".")[0]
        by_stem[stem].append(r)
    for stem, paths in by_stem.items():
        if len(paths) < 2:
            continue
        mirrored = {Path(*p.parts[1:]).with_suffix("") for p in paths
                    if p.parts[0] in ("include", "src")}
        if len(paths) == 2 and len(mirrored) == 1 and all(
                p.parts[0] in ("include", "src") for p in paths):
            continue
        for p in paths:
            others = ", ".join(str(o) for o in paths if o != p)
            f.add("unique-names", p, 1, f"shares its name with {others}")


def check_namespaces(r, code, f):
    if not is_code(r):
        return
    folder = ["achilles", *r.parts[1:-1]]
    stem = r.name.split(".")[0]

    def valid(parts):
        if "detail" in parts:
            parts = parts[: parts.index("detail")]
        return (parts == folder[: len(parts)] or parts == folder
                or parts == folder + [stem])

    stack = []
    for m in re.finditer(r"\bnamespace\s+([\w:]*)\s*(=|\{)|[{}]", code):
        token = m.group(0)
        if token == "{":
            stack.append(None)
        elif token == "}":
            if stack:
                stack.pop()
        elif m.group(2) == "{":
            name = m.group(1)
            outer = [p for frame in stack if frame for p in frame]
            if not name:
                stack.append([])
                continue
            parts = outer + name.split("::")
            if not valid(parts):
                f.add("namespace", r, line_of(code, m.start()),
                      f"namespace {'::'.join(parts)} doesn't match "
                      f"{'::'.join(folder)}")
            stack.append(name.split("::"))


def check_concepts(r, code, concepts, f):
    for m in re.finditer(r"\bconcept\s+(\w+)\s*=", code):
        name = m.group(1)
        if not name.endswith("Like"):
            f.add("concept-names", r, line_of(code, m.start()),
                  f"concept {name} doesn't end in Like")
        if r.parts[0] == "include" and is_code(r):
            concepts.append((name, r, line_of(code, m.start())))


FORBIDDEN = [
    (r"\bvirtual\b", "virtual"),
    (r"\boverride\b", "override"),
    (r"\bstd::function\b", "std::function"),
    (r"\bstd::variant\b", "std::variant"),
    (r"\bstd::any\b", "std::any"),
]


def check_forbidden(r, text, code, f):
    if r.parts[0] in ("include", "src") and top_folder(r) in SKIPPED:
        return
    for number, line in enumerate(text.splitlines(), 1):
        if "NOLINT" in line:
            f.add("forbidden", r, number, "NOLINT")
    if not is_code(r):
        return
    for pattern, name in FORBIDDEN:
        for m in re.finditer(pattern, code):
            f.add("forbidden", r, line_of(code, m.start()), name)


def check_throw(r, code, f):
    if not is_code(r) or top_folder(r) == "interface":
        return
    for m in re.finditer(r"\bthrow\b", code):
        f.add("throw", r, line_of(code, m.start()), "throw outside interface/")


def check_step(r, code, f):
    if top_folder(r) != "algorithms" or not r.name.endswith("_step.hpp"):
        return
    for m in re.finditer(r"\b(for|while)\s*\(|\bif\s*(?!constexpr)\(", code):
        f.add("step-control-flow", r, line_of(code, m.start()),
              f"{m.group(0).strip('( ')} in a step")


def matching_brace(code, start):
    depth = 0
    for i in range(start, len(code)):
        if code[i] == "{":
            depth += 1
        elif code[i] == "}":
            depth -= 1
            if depth == 0:
                return i
    return len(code)


def check_op_asserts(r, code, f):
    if top_folder(r) != "algorithms" or not r.name.endswith("_ops.hpp"):
        return
    for m in re.finditer(r"\bstruct\s+(\w+)\b[^;{]*\{", code):
        body = code[m.end() - 1: matching_brace(code, m.end() - 1)]
        if "kOutputs" not in body:
            continue
        name = m.group(1)
        if not re.search(rf"\bOpLike\s*<\s*{name}\b", code):
            f.add("op-asserts", r, line_of(code, m.start()),
                  f"{name} has no OpLike static_assert")


def check_policy_asserts(r, code, f):
    if r.parts[:2] != ("include", "policy") or len(r.parts) < 4:
        return
    if r.stem == r.parts[2]:
        return
    if not re.search(r"static_assert\s*\(\s*[\w:]*Like\s*<", code):
        f.add("policy-asserts", r, 1, "no concept static_assert")


def check_concept_archetypes(concepts, test_text, f):
    for name, r, number in concepts:
        archetype = name[: -len("Like")] + "Archetype" \
            if name.endswith("Like") else name + "Archetype"
        if not re.search(rf"\b{archetype}\b", test_text):
            f.add("concept-archetypes", r, number,
                  f"no {archetype} in tests/")


def check_file_tree(files, f):
    tree_path = ROOT / "docs" / "file-tree.md"
    if not tree_path.exists():
        return
    tree = tree_path.read_text()
    tree_rel = rel(tree_path)
    algorithms = algorithm_folders()
    for r in files:
        if r.parts[0] == "include" and top_folder(r) not in SKIPPED:
            if top_folder(r) == "algorithms" and len(r.parts) > 3 \
                    and r.parts[2] in algorithms:
                if f"`{r.parts[2]}/`" not in tree:
                    f.add("file-tree", r, 1,
                          f"algorithm {r.parts[2]}/ missing from file-tree.md")
                continue
            if r.name not in tree:
                f.add("file-tree", r, 1, "missing from docs/file-tree.md")
        elif r.parts[:2] == ("tests", "support") or (
                r.parts[0] == "examples" and r.suffix == ".hpp"):
            if r.name not in tree:
                f.add("file-tree", r, 1, "missing from docs/file-tree.md")
    names = {p.name for p in ROOT.rglob("*") if p.is_file()}
    for number, line in enumerate(tree.splitlines(), 1):
        for name in re.findall(r"[\w/.<>-]+\.(?:hpp|cpp|sh|in)\b", line):
            base = Path(name).name
            if "<" not in name and base not in names:
                f.add("file-tree", tree_rel, number,
                      f"{name} doesn't exist")


def check_doc_links(files, f):
    linked = set()
    for r in files:
        text = (ROOT / r).read_text(errors="replace")
        for number, line in enumerate(text.splitlines(), 1):
            m = re.search(r"//\s*Doc:\s*(\S+)", line)
            if not m:
                continue
            linked.add(Path(m.group(1)))
            if not (ROOT / m.group(1)).exists():
                f.add("doc-links", r, number, f"{m.group(1)} doesn't exist")
    code_docs = ROOT / "docs" / "code"
    if code_docs.exists():
        for doc in sorted(code_docs.rglob("*.md")):
            if rel(doc) not in linked:
                f.add("doc-links", rel(doc), 1,
                      "no source file links this doc")


def check_md_links(f):
    docs = [ROOT / "CLAUDE.md", *sorted((ROOT / "docs").rglob("*.md"))]
    for doc in docs:
        if not doc.exists():
            continue
        text = doc.read_text()
        for m in re.finditer(r"\[[^\]]*\]\(([^)\s]+)\)", text):
            target = m.group(1).split("#")[0]
            if not target or re.match(r"[a-z]+:", target):
                continue
            if not (doc.parent / target).exists():
                f.add("md-links", rel(doc), line_of(text, m.start()),
                      f"{target} doesn't exist")


def check_file_comment(r, text, f):
    if not is_code(r):
        return
    for line in text.splitlines():
        s = line.strip()
        if not s or s == "#pragma once":
            continue
        if not s.startswith("//"):
            f.add("file-comment", r, 1, "doesn't start with a comment")
        return


def flattened(r):
    return "_".join(Path(*r.parts[1:]).with_suffix("").parts)


def check_tests(files, f):
    headers = [r for r in files if r.parts[0] == "include" and is_code(r)
               and r.suffix == ".hpp"]
    tests = {r.stem for r in files if r.parts[0] == "tests"
             and len(r.parts) == 2 and r.suffix == ".cpp"}
    flats = {flattened(h): h for h in headers}
    for flat, header in flats.items():
        if flat not in tests:
            f.add("missing-test", header, 1, f"no tests/{flat}.cpp")
    examples = {p.name for p in (ROOT / "examples").iterdir() if p.is_dir()}
    shared = {p.stem for p in (ROOT / "examples" / "shared").glob("*.hpp")}
    known_tops = {r.parts[1] for r in files if r.parts[0] == "include"}
    for test in sorted(tests):
        if test.startswith("examples_"):
            if test[len("examples_"):] in examples | shared:
                continue
        elif test in flats or test.split("_")[0] not in known_tops:
            continue
        f.add("stale-test", Path("tests") / f"{test}.cpp", 1,
              "matches no header or example")


ALLOCATION = re.compile(
    r"\bstd::vector\b|\bnew\b|\bmalloc\s*\(|\bmake_(unique|shared)\b")
HISTORY = re.compile(
    r"\bline \d+|\bused to (be|live|have|hold|take|run|return|call)\b|"
    r"\bno longer\b|\bpreviously\b|\banymore\b|\bthe old (version|code|"
    r"name|way)\b", re.I)


def check_loop_allocation(r, code, f):
    if top_folder(r) != "algorithms":
        return
    if not re.search(r"_(ops|step)\.(hpp|cpp)$", r.name):
        return
    for m in ALLOCATION.finditer(code):
        f.add("loop-allocation", r, line_of(code, m.start()),
              f"{m.group(0).strip()} in op or step code")


def check_comment_history(r, text, f):
    if not is_code(r):
        return
    for number, comment in comments_of(text):
        m = HISTORY.search(comment)
        if m:
            f.add("comment-history", r, number,
                  f"comment says \"{m.group(0)}\"")


def main():
    parser = argparse.ArgumentParser(
        description="Check the rules in docs/conventions/.")
    parser.add_argument("--since", metavar="REV")
    parser.add_argument("--only", metavar="RULE,...")
    parser.add_argument("--list", action="store_true")
    parser.add_argument("--no-warnings", action="store_true")
    args = parser.parse_args()

    if args.list:
        for name, text in RULES.items():
            kind = "warning" if name in WARNINGS else "error"
            print(f"{name:20} {kind:8} {text}")
        return 0
    only = set(args.only.split(",")) if args.only else set(RULES)
    unknown = only - set(RULES)
    if unknown:
        print(f"unknown rule(s): {', '.join(sorted(unknown))}",
              file=sys.stderr)
        return 2

    f = Findings()
    files = source_files()
    tables = read_layer_tables()
    algorithms = algorithm_folders()
    concepts = []
    for r in files:
        text = (ROOT / r).read_text(errors="replace")
        code = strip_code(text)
        check_layers(r, text, tables, f)
        check_structure(r, f)
        check_src_mirror(r, f)
        check_algorithm_includes(r, text, algorithms, f)
        check_namespaces(r, code, f)
        check_concepts(r, code, concepts, f)
        check_forbidden(r, text, code, f)
        check_throw(r, code, f)
        check_step(r, code, f)
        check_op_asserts(r, code, f)
        check_policy_asserts(r, code, f)
        check_file_comment(r, text, f)
        check_loop_allocation(r, code, f)
        check_comment_history(r, text, f)
    test_text = "\n".join((ROOT / r).read_text(errors="replace")
                          for r in files if r.parts[0] == "tests")
    check_algorithm_files(f)
    check_unique_names(files, f)
    check_concept_archetypes(concepts, test_text, f)
    check_file_tree(files, f)
    check_doc_links(files, f)
    check_md_links(f)
    check_tests(files, f)

    changed = changed_since(args.since) if args.since else None
    errors = warnings = 0
    for rule, path, line, message in sorted(
            f.items, key=lambda x: (str(x[1]), x[2], x[0])):
        if rule not in only or (changed is not None and path not in changed):
            continue
        is_warning = rule in WARNINGS
        if is_warning and args.no_warnings:
            continue
        kind = "warning" if is_warning else "error"
        print(f"{path}:{line}: {kind}: {rule}: {message}")
        warnings += is_warning
        errors += not is_warning
    print(f"{errors} error(s), {warnings} warning(s)", file=sys.stderr)
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
