"""按app.yaml文件清单打包，并逐文件核对包内内容，防止部署旧代码。"""
from pathlib import Path
import re
import zipfile

ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "dist" / "maix-gongxun_now-v2.0.0.zip"


def build():
    manifest = (ROOT / "app.yaml").read_text(encoding="utf-8")
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
    if "NOW_RGB3_RING_NOOCR_TX40_V1" not in main_source:
        raise ValueError("unexpected build marker")
    if "vision_ocr" in main_source or "vision_ocr.py" in content:
        raise ValueError("OCR should not be in NOW runtime package")
    OUTPUT.parent.mkdir(exist_ok=True)
    with zipfile.ZipFile(OUTPUT, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        for name, data in content.items():
            archive.writestr(name, data)
    with zipfile.ZipFile(OUTPUT) as archive:
        if set(archive.namelist()) != set(names):
            raise ValueError("archive differs from manifest")
        for name, data in content.items():
            if archive.read(name) != data:
                raise ValueError("stale archived file: " + name)
        if archive.testzip() is not None:
            raise ValueError("archive CRC failed")
    print("PACKAGE VERIFIED:", OUTPUT, "files:", len(names))


if __name__ == "__main__":
    build()
