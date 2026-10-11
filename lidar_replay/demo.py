"""可重复的合成记录，只用于工具验收，不代表 D6 实测能力。"""
from recording import normalize_recording


def make_demo():
    frames = []
    for i in range(40):
        d, v, strength = [0] * 360, [0] * 360, [0] * 360
        def add(angles, distance):
            for a in angles:
                a %= 360
                d[a], v[a], strength[a] = distance, 1, 100
        add(range(35, 46), 1600 + (i % 3 - 1))
        add([0], 650)
        add([200], 30)
        add([220], 4500)
        add([230], 900)
        if i < 2:
            add([270] if i == 0 else [269, 270], 700)
            label = "细目标仅 1 / 2 点：被最小点数规则拒绝"
        elif i < 8:
            add(range(268, 273), 700 + i % 3 - 1)
            label = "细目标形成候选：第二次匹配后确认并封路"
        elif i < 12:
            label = "目标消失：已确认障碍和封路仍保留"
        elif i < 20:
            add(list(range(355, 360)) + list(range(0, 6)), 600)
            label = "跨 0°：观察完整聚类；可启用遮挡或对比旧遍历规则"
        elif i < 28:
            add([80, 81, 84, 85], 900)
            add([160, 161, 165, 166], 900)
            label = "连续缺 2° 可以连接；连续缺 3° 分割成小组"
        elif i < 34:
            for a in range(5, 360, 15):
                add([a, a + 1, a + 2], 800)
            label = "超过 20 组：检查被容量限制丢弃的候选"
        else:
            label = "空场复核：确认记录持续保留，不自动清路"
        frames.append({"scan_num": i + 1, "start_time": i * 100, "end_time": (i + 1) * 100,
                       "distance": d, "valid": v, "intensity": strength,
                       "pose": {"x": 1200, "y": 800, "yaw_deg": 0, "valid": True},
                       "label": label, "metadata": {"synthetic": True}})
    return normalize_recording({"format": "gongxun-lidar-v1", "synthetic": True,
                                "label": "合成验收 · 细目标 / 遮挡 / 漏点 / 地图保持",
                                "frames": frames,
                                "warnings": ["合成样本只验证软件规则，不能证明 D6 能检测真实细杆。",
                                             "样本的位姿、100 ms 圈间隔和目标距离均为人为设定。"]})
