"""
A check that the hand-written stub and the compiled module describe the same names.

Because the stub is maintained by hand, it can diverge from the bindings. This test compares the
public names in both directions to detect a binding without a stub entry or a stale stub entry. The
test checks names only; it does not check signatures.
"""

from __future__ import annotations

import ast
import inspect
from pathlib import Path

import pytest

import crx_kinematics
from crx_kinematics import crx_kinematics as native

_STUB = Path(crx_kinematics.__file__).parent / "crx_kinematics.pyi"


def _stub_tree() -> ast.Module:
    return ast.parse(_STUB.read_text())


def _stub_classes() -> dict[str, set[str]]:
    """The classes in the stub, each with the set of members declared on it."""
    classes = {}
    for node in _stub_tree().body:
        if isinstance(node, ast.ClassDef):
            members = set()
            for item in node.body:
                if isinstance(item, (ast.FunctionDef, ast.AsyncFunctionDef)):
                    members.add(item.name)
                elif isinstance(item, ast.AnnAssign) and isinstance(item.target, ast.Name):
                    members.add(item.target.id)
                elif isinstance(item, ast.Assign):
                    for target in item.targets:
                        if isinstance(target, ast.Name):
                            members.add(target.id)
            classes[node.name] = members
    return classes


def _public(names) -> set[str]:
    return {name for name in names if not name.startswith("_")}


def test_the_stub_file_is_shipped():
    assert _STUB.is_file()
    assert (_STUB.parent / "py.typed").is_file()


def test_the_stub_and_the_module_declare_the_same_classes():
    in_module = _public(name for name, obj in vars(native).items() if inspect.isclass(obj))
    in_stub = set(_stub_classes())

    assert in_module, "no classes discovered in the compiled module"
    assert in_module == in_stub


@pytest.mark.parametrize(
    "class_name", ["Crx", "CrxModel", "IkSolution", "LinkMesh", "LinkMeshes", "SolutionKind"]
)
def test_the_stub_and_the_module_declare_the_same_members(class_name: str):
    cls = getattr(native, class_name)
    in_module = _public(vars(cls))
    in_stub = _stub_classes()[class_name]

    # An enum carries its variants as class members alongside the `name` and `value` attributes
    # added by Python's enum machinery. The stub omits those attributes. Identify variants by
    # selecting members that are instances of the class itself. This approach handles spellings
    # such as `Crx10iAL`, which a capitalization rule would fail to recognize.
    variants = {name for name, obj in vars(cls).items() if isinstance(obj, cls)}
    if variants:
        in_module = variants
    assert in_module == in_stub
