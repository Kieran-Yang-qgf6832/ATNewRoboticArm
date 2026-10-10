from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from types import ModuleType


def add_vendored_figaroh() -> Path:
    """Put the vendored FIGAROH package at the front of sys.path."""
    figaroh_src = _find_vendored_figaroh_src()
    if not figaroh_src.exists():
        raise FileNotFoundError(
            f"Vendored FIGAROH not found at {figaroh_src}. "
            "Clone https://github.com/thanhndv212/figaroh-plus.git there first."
        )
    src_text = str(figaroh_src)
    if src_text not in sys.path:
        sys.path.insert(0, src_text)
    return figaroh_src


def _find_vendored_figaroh_src() -> Path:
    candidates = []
    for base in [Path(__file__).resolve(), Path.cwd().resolve()]:
        candidates.extend(parent / "third_party" / "figaroh-plus" / "src" for parent in [base, *base.parents])
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return Path(__file__).resolve().parents[3] / "third_party" / "figaroh-plus" / "src"


def _register_figaroh_namespace(figaroh_src: Path, canonical_name: str) -> None:
    """为 canonical_name 的每一级父包注册带 __path__ 的空壳模块。

    只建空壳、不执行它们的 __init__.py：figaroh/__init__.py 会 eager 导入
    tools/calibration/identification/...，其中 tools/robotipopt.py 需要
    numdifftools（以及用到时才导入的 cyipopt）——本辨识工作流完全用不到这些，
    缺一个就让整条 import 链崩掉。
    """
    parts = canonical_name.split(".")
    for depth in range(1, len(parts)):
        package_name = ".".join(parts[:depth])
        if package_name in sys.modules:
            continue
        package = ModuleType(package_name)
        package.__path__ = [str(figaroh_src.joinpath(*parts[:depth]))]
        package.__package__ = package_name
        sys.modules[package_name] = package


def load_vendored_figaroh_module(module_name: str, relative_path: str) -> ModuleType:
    """Load one FIGAROH source file without importing figaroh.__init__."""
    figaroh_src = add_vendored_figaroh()
    module_path = figaroh_src / relative_path
    if not module_path.exists():
        raise FileNotFoundError(f"Vendored FIGAROH module not found: {module_path}")
    spec = importlib.util.spec_from_file_location(module_name, module_path)
    if spec is None or spec.loader is None:
        raise ImportError(f"Could not load module spec for {module_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)

    # vendored 代码内部有**包内绝对导入**，例如 reconstruction.py:443 的
    #     from figaroh.identification.cad_constraints import ...
    # 直接让它走正常导入会执行 figaroh/__init__.py（见上面的说明）并抛 ImportError；
    # 更糟的是 reconstruction.py 把 ImportError 当成"求解器缺失"，于是报出
    # "Install picos/cvxopt" —— 在 picos/cvxopt 都装好的情况下完全指错方向
    # （docs/bugs.md 教训 7：错误日志里的数字/结论要有物理含义）。
    # 这里把**已经加载的这份**同时登记到它的规范名下，包内绝对导入即可命中它。
    if relative_path.endswith(".py"):
        canonical_name = relative_path[: -len(".py")].replace("/", ".")
        if canonical_name.startswith("figaroh."):
            _register_figaroh_namespace(figaroh_src, canonical_name)
            sys.modules.setdefault(canonical_name, module)
    return module
