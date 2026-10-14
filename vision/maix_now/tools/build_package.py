"""按app.yaml文件清单打包，并逐文件核对包内内容，防止部署旧代码。"""
from pathlib import Path
import ast
import re
import zipfile

ROOT = Path(__file__).resolve().parents[1]


def build():
    manifest = (ROOT / "app.yaml").read_text(encoding="utf-8")
    version = re.search(r"^version: ([0-9]+\.[0-9]+\.[0-9]+)\s*$", manifest, re.MULTILINE)
    if version is None:
        raise ValueError("missing package version")
    output = ROOT / "dist" / ("maix-gongxun_now-v" + version.group(1) + ".zip")
    names = re.findall(r"^  - (\S+)\s*$", manifest, flags=re.MULTILINE)
    if not names or "main.py" not in names or "vision_runtime.py" not in names:
        raise ValueError("incomplete package manifest")
    if len(set(names)) != len(names):
        raise ValueError("duplicate manifest entry")
    content = {}
    for name in names:
        target = (ROOT / name).resolve()
        if not target.is_relative_to(ROOT) or not target.is_file():
            raise ValueError("invalid package path: " + name)
        data = target.read_bytes()
        if name.endswith(".py"):
            compile(data.decode("utf-8"), name, "exec")
        content[name] = data
    main_source = content["main.py"].decode("utf-8")
    if "NOW_UI_TOP_LAB6_B2_TX28_V7" not in main_source:
        raise ValueError("unexpected build marker")
    # 核对所有运行模块的本地依赖；OCR/数字模板及旧入口不允许进入新包。
    allowed = {"app.png", "app.yaml", "main.py", "vision_blob.py", "vision_config.py",
               "vision_runtime.py", "vision_ring.py", "vision_tasks.py", "vision_ui.py"}
    if set(names) != allowed:
        raise ValueError("unexpected NOW runtime files")
    for name, data in content.items():
        if not name.endswith(".py"):
            continue
        tree = ast.parse(data.decode("utf-8"), filename=name)
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imports = [alias.name for alias in node.names]
            elif isinstance(node, ast.ImportFrom):
                imports = [node.module or ""]
            else:
                continue
            for module in imports:
                if module.startswith("vision_") and module + ".py" not in content:
                    raise ValueError("unpackaged runtime dependency: " + module)
    output.parent.mkdir(exist_ok=True)
    with zipfile.ZipFile(output, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        for name, data in content.items():
            archive.writestr(name, data)
    with zipfile.ZipFile(output) as archive:
        if set(archive.namelist()) != set(names):
            raise ValueError("archive differs from manifest")
        for name, data in content.items():
            if archive.read(name) != data:
                raise ValueError("stale archived file: " + name)
        if archive.testzip() is not None:
            raise ValueError("archive CRC failed")
    print("PACKAGE VERIFIED:", output, "files:", len(names))


if __name__ == "__main__":
    build()
