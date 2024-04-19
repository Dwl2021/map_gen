import numpy as np
import random
import re

def densify_pcd(input_filename, output_filename, points_per_point=9, max_offset=0.01):
    with open(input_filename, 'r') as file:
        lines = file.readlines()

    header = True
    header_lines = []
    points = []
    for line in lines:
        if header:
            header_lines.append(line)
            if re.match(r"^DATA\s+ascii$", line.strip(), re.IGNORECASE):
                header = False
            continue
        parts = line.strip().split()
        if len(parts) >= 3:
            x, y, z = map(float, parts[:3])
            points.append([x, y, z])

    # 创建新点，每个原始点周围随机生成 points_per_point 个点
    new_points = []
    for point in points:
        new_points.append(point)  # 加入原始点
        for _ in range(points_per_point):
            offset = np.random.normal(0, max_offset, 3)
            new_point = point + offset
            new_points.append(new_point.tolist())

    # 更新点数信息
    for i, line in enumerate(header_lines):
        if line.startswith('POINTS'):
            header_lines[i] = f'POINTS {len(new_points)}\n'
        elif line.startswith('DATA'):
            header_lines[i] = 'DATA ascii\n'

    # 保存结果到新的PCD文件
    with open(output_filename, 'w') as file:
        for line in header_lines:
            file.write(line)
        for point in new_points:
            file.write(f"{point[0]} {point[1]} {point[2]}\n")

# 使用示例
densify_pcd('tmp.pcd', 'map.pcd', points_per_point=5, max_offset=0.01)
