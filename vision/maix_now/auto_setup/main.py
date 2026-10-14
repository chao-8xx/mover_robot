import configparser
import os

# 当前仓库中新应用的 ID
new_id = "gongxun_now"

apps = configparser.ConfigParser()
apps.read("/maixapp/apps/app.info")

if not apps.has_section(new_id):
    print("已安装的应用 ID：", apps.sections())
    raise RuntimeError("未找到新应用，请先安装，或核对打包时的应用 ID")

path = "/maixapp/auto_start.txt"

old_id = None
if os.path.exists(path):
    with open(path, "r") as f:
        old_id = f.read().strip()

with open(path, "w") as f:
    f.write(new_id)

os.sync()

with open(path, "r") as f:
    print("原自启应用：", old_id)
    print("现在自启应用：", f.read().strip())