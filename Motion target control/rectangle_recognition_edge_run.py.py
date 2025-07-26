import sensor, image, time, lcd, machine
from machine import UART
from fpioa_manager import fm
import math

# 初始化LCD显示
lcd.init()
lcd.clear(lcd.GREEN)
print("LCD initialized")

# 初始化摄像头
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=500)
sensor.set_vflip(0)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
print("Camera initialized")

# 配置串口通信
fm.register(6, fm.fpioa.UART2_RX)
fm.register(8, fm.fpioa.UART2_TX)
uart = UART(UART.UART2, 115200, timeout=100, read_buf_len=4096)
print("UART initialized at 115200 baud")

# 连续运动状态变量
current_position = 0.0  # 0-4之间的浮点数，表示在矩形边缘上的位置
MOVE_SPEED = 0.01
last_move_time = time.ticks_ms()

# 矩形检测历史缓存
RECT_HISTORY_SIZE = 7
rect_history = []
rect_history_index = 0
for i in range(RECT_HISTORY_SIZE):
    rect_history.append(None)

# 确保顶点顺序为顺时针
def ensure_clockwise(corners):
    if len(corners) != 4:
        return corners

    cx = sum(p[0] for p in corners) / 4
    cy = sum(p[1] for p in corners) / 4

    angles = []
    for x, y in corners:
        dx = x - cx
        dy = y - cy
        angles.append(math.atan2(dy, dx))

    sorted_corners = [p for _, p in sorted(zip(angles, corners))]

    min_index = 0
    min_distance = float('inf')
    for i, (x, y) in enumerate(sorted_corners):
        distance = x + y
        if distance < min_distance:
            min_distance = distance
            min_index = i

    return sorted_corners[min_index:] + sorted_corners[:min_index]

# 计算两点之间的距离
def distance(p1, p2):
    return math.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2)

# 计算矩形质量分数
def calculate_rectangle_quality(corners):
    if len(corners) != 4:
        return 0

    edges = [
        distance(corners[0], corners[1]),
        distance(corners[1], corners[2]),
        distance(corners[2], corners[3]),
        distance(corners[3], corners[0])
    ]

    angles = []
    for i in range(4):
        prev = corners[i-1]
        current = corners[i]
        next = corners[(i+1) % 4]

        v1 = (prev[0]-current[0], prev[1]-current[1])
        v2 = (next[0]-current[0], next[1]-current[1])

        dot = v1[0]*v2[0] + v1[1]*v2[1]

        v1_len = distance(prev, current)
        v2_len = distance(current, next)

        if v1_len == 0 or v2_len == 0:
            return 0

        cos_theta = dot / (v1_len * v2_len)
        cos_theta = max(-1.0, min(1.0, cos_theta))

        angle = math.degrees(math.acos(cos_theta))
        angles.append(angle)

    max_edge = max(edges)
    min_edge = min(edges)
    edge_score = 1 - (max_edge - min_edge) / max_edge if max_edge > 0 else 0

    angle_deviation = sum(abs(angle - 90) for angle in angles) / 4
    angle_score = 1 - (angle_deviation / 45)

    return (edge_score * 0.6) + (angle_score * 0.4)

# 计算在矩形边缘上的位置
def get_position_on_edge(avg_corners, position):
    position %= 4.0
    edge_index = int(position)
    t = position - edge_index

    p0 = avg_corners[edge_index % 4]
    p1 = avg_corners[(edge_index + 1) % 4]

    x = int(p0[0] * (1 - t) + p1[0] * t)
    y = int(p0[1] * (1 - t) + p1[1] * t)

    return (x, y)

# 发送坐标数据 - 按照文档1的格式
def send_target_position(x, y):
    """发送目标点坐标到串口 (文档1的格式)"""
    # 确保值在0-319和0-239范围内
    x_val = max(0, min(319, x))
    y_val = max(0, min(239, y))

    # 创建格式化的字符串 "XxxxYyyy"
    data_str = "X{:03d}Y{:03d}\n".format(x_val, y_val)

    # 发送数据
    uart.write(data_str.encode())
    print("Sent:", data_str.strip())

while True:
    img = sensor.snapshot()
    img.gaussian(1)

    # 缩小图像以提高处理速度
    img_scaled = img.copy().resize(img.width()//2, img.height()//2)

    rects = img_scaled.find_rects(threshold=28000)

    # 更新移动位置
    current_time = time.ticks_ms()
    time_diff = current_time - last_move_time
    last_move_time = current_time
    current_position += MOVE_SPEED * (time_diff / 100.0)
    if current_position >= 4.0:
        current_position -= 4.0

    center_x, center_y = None, None
    avg_corners = []

    valid_rect_found = False
    best_rect = None
    best_score = 0

    # 寻找最佳矩形
    for rect in rects:
        corners = rect.corners()
        if len(corners) != 4:
            continue

        corners = ensure_clockwise(corners)
        score = calculate_rectangle_quality(corners)

        if score > best_score and score > 0.6:
            best_score = score
            best_rect = rect
            valid_rect_found = True

    if valid_rect_found:
        rect = best_rect
        corners = rect.corners()
        corners = ensure_clockwise(corners)

        # 放大到原始图像尺寸
        corners_orig = [(x*2, y*2) for (x, y) in corners]

        # 计算矩形中心
        center_x = int(sum(p[0] for p in corners_orig) / 4)
        center_y = int(sum(p[1] for p in corners_orig) / 4)

        # 计算边框宽度
        w = distance(corners_orig[0], corners_orig[1])
        h = distance(corners_orig[1], corners_orig[2])
        border_width = min(w, h) * 0.12

        # 计算内矩形顶点
        inner_corners = []
        for i in range(4):
            prev = corners_orig[(i-1) % 4]
            curr = corners_orig[i]
            next = corners_orig[(i+1) % 4]

            v1 = (prev[0] - curr[0], prev[1] - curr[1])
            v1_len = math.sqrt(v1[0]**2 + v1[1]**2)
            if v1_len > 0:
                v1 = (v1[0]/v1_len, v1[1]/v1_len)

            v2 = (next[0] - curr[0], next[1] - curr[1])
            v2_len = math.sqrt(v2[0]**2 + v2[1]**2)
            if v2_len > 0:
                v2 = (v2[0]/v2_len, v2[1]/v2_len)

            bisector = (v1[0] + v2[0], v1[1] + v2[1])
            bisector_len = math.sqrt(bisector[0]**2 + bisector[1]**2)
            if bisector_len > 0:
                bisector = (bisector[0]/bisector_len, bisector[1]/bisector_len)

            inner_x = curr[0] + bisector[0] * border_width
            inner_y = curr[1] + bisector[1] * border_width
            inner_corners.append((int(inner_x), int(inner_y)))

        # 计算平均顶点
        current_avg_corners = []
        for i in range(4):
            avg_x = int((corners_orig[i][0] + inner_corners[i][0]) / 2)
            avg_y = int((corners_orig[i][1] + inner_corners[i][1]) / 2)
            current_avg_corners.append((avg_x, avg_y))

        # 保存到历史记录
        rect_history[rect_history_index] = current_avg_corners
        rect_history_index = (rect_history_index + 1) % RECT_HISTORY_SIZE

        # 计算历史平均
        avg_corners = [[0.0, 0.0] for _ in range(4)]
        total_weight = 0

        for i, corners in enumerate(rect_history):
            if corners is None:
                continue

            weight = RECT_HISTORY_SIZE - i
            total_weight += weight

            for j in range(4):
                avg_corners[j][0] += corners[j][0] * weight
                avg_corners[j][1] += corners[j][1] * weight

        if total_weight > 0:
            for i in range(4):
                avg_corners[i][0] = int(avg_corners[i][0] / total_weight)
                avg_corners[i][1] = int(avg_corners[i][1] / total_weight)
            avg_corners = [(x, y) for x, y in avg_corners]
        else:
            avg_corners = current_avg_corners

        # 绘制平均矩形（蓝色）
        for i in range(4):
            start = avg_corners[i]
            end = avg_corners[(i + 1) % 4]
            img.draw_line(start[0], start[1], end[0], end[1], color=(0, 0, 255), thickness=2)

        # 绘制顶点（绿色）
        for x, y in avg_corners:
            img.draw_circle(x, y, 5, color=(0, 255, 0), thickness=2)

        # 获取当前运动点位置（红色目标点）
        target_x, target_y = get_position_on_edge(avg_corners, current_position)

        # 绘制运动点（红色）
        img.draw_circle(target_x, target_y, 8, color=(255, 0, 0), thickness=2, fill=True)

        # 显示目标点坐标 - 按照文档1的格式
        coord_str = "X{:03d}Y{:03d}".format(target_x, target_y)
        img.draw_string(target_x + 5, target_y, coord_str, color=(255, 255, 255), scale=1.0)

        # 发送目标点坐标 - 按照文档1的格式
        send_target_position(target_x, target_y)

        # 显示矩形质量 - 使用文档1的格式
        quality_str = "Q{:.2f}".format(best_score)
        img.draw_string(10, 40, quality_str, color=(255, 255, 255), scale=1.0)

        # 绘制中心点（红色十字）
        img.draw_cross(center_x, center_y, color=(255, 0, 0), size=10)

        # 显示中心坐标 - 按照文档1的格式
        center_str = "X{:03d}Y{:03d}".format(center_x, center_y)
        img.draw_string(center_x, center_y, center_str, color=(255, 255, 255))

    else:
        # 更新历史记录
        rect_history[rect_history_index] = None
        rect_history_index = (rect_history_index + 1) % RECT_HISTORY_SIZE

        # 尝试使用历史数据
        valid_count = 0
        avg_corners = [[0.0, 0.0] for _ in range(4)]
        total_weight = 0

        for i, corners in enumerate(rect_history):
            if corners is None:
                continue

            weight = RECT_HISTORY_SIZE - i
            total_weight += weight

            for j in range(4):
                avg_corners[j][0] += corners[j][0] * weight
                avg_corners[j][1] += corners[j][1] * weight

        if total_weight > 0:
            for i in range(4):
                avg_corners[i][0] = int(avg_corners[i][0] / total_weight)
                avg_corners[i][1] = int(avg_corners[i][1] / total_weight)
            avg_corners = [(x, y) for x, y in avg_corners]

            # 在原始图像上绘制历史平均矩形（黄色）
            for i in range(4):
                start = avg_corners[i]
                end = avg_corners[(i + 1) % 4]
                img.draw_line(start[0], start[1], end[0], end[1], color=(255, 255, 0), thickness=1, dotted=True)

            # 获取当前运动点位置（黄色目标点）
            target_x, target_y = get_position_on_edge(avg_corners, current_position)

            # 绘制运动点（黄色）
            img.draw_circle(target_x, target_y, 8, color=(255, 255, 0), thickness=2, fill=True)

            # 显示目标点坐标 - 按照文档1的格式
            coord_str = "X{:03d}Y{:03d}".format(target_x, target_y)
            img.draw_string(target_x + 5, target_y, coord_str, color=(255, 255, 255), scale=1.0)

            # 发送目标点坐标 - 按照文档1的格式
            send_target_position(target_x, target_y)

            # 显示警告信息 - 使用文档1的格式
            img.draw_string(10, 80, "HIST DATA", color=(255, 255, 0))
        else:
            # 完全没有数据时显示错误
            img.draw_string(10, 10, "NO RECT", color=(255, 0, 0))

    # 更新LCD显示
    lcd.display(img)

    # 控制处理速度
    time.sleep_ms(20)
