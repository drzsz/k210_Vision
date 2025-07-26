import sensor, image, lcd, math, time
from machine import UART
from fpioa_manager import fm

# 初始化LCD显示屏
lcd.init()

# 初始化摄像头
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)  # 320x240
sensor.set_vflip(False)
sensor.set_auto_gain(False)  # 关闭自动增益
sensor.set_auto_whitebal(False)  # 关闭自动白平衡
sensor.set_auto_exposure(False, 1000)  # 固定曝光时间
sensor.set_brightness(0)  # 中等亮度
sensor.set_contrast(0)   # 中等对比度
sensor.run(1)

# 配置UART2引脚：IO6为RX，IO8为TX
fm.register(6, fm.fpioa.UART2_RX)
fm.register(8, fm.fpioa.UART2_TX)

# 初始化串口 (UART2，波特率115200)
uart = UART(UART.UART2, 115200, timeout=100, read_buf_len=4096)

# 系统状态
SEND_COORDINATES = True
PAUSED = False
last_center = None

# 连续运动状态变量
current_position = 0.0  # 0-4之间的浮点数，表示在矩形边缘上的位置
MOVE_SPEED = 0.02  # 增加红点移动速度 (原值0.01)
last_move_time = time.ticks_ms()
last_sent_time = 0

# 矩形检测历史缓存
RECT_HISTORY_SIZE = 7  # 增加历史缓存大小
rect_history = []
rect_history_index = 0

# 缩放图像
def scale_image(img, scale):
    width = img.width() // scale
    height = img.height() // scale
    img_scaled = img.resize(width, height)
    return img_scaled

# 确保顶点顺序为顺时针
def ensure_clockwise(corners):
    if len(corners) != 4:
        return corners

    # 计算凸包中心
    cx = sum(p[0] for p in corners) / 4
    cy = sum(p[1] for p in corners) / 4

    # 计算每个点相对于中心的角度
    angles = []
    for x, y in corners:
        dx = x - cx
        dy = y - cy
        angles.append(math.atan2(dy, dx))

    # 按角度排序
    sorted_corners = [p for _, p in sorted(zip(angles, corners))]

    # 确保第一个点是左上角
    min_index = 0
    min_distance = float('inf')
    for i, (x, y) in enumerate(sorted_corners):
        distance = x + y  # 左上角通常是x+y最小的点
        if distance < min_distance:
            min_distance = distance
            min_index = i

    # 旋转列表使左上角在前
    return sorted_corners[min_index:] + sorted_corners[:min_index]

# 计算两点之间的距离
def distance(p1, p2):
    return math.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2)

# 计算矩形质量分数 (更严格的评分)
def calculate_rectangle_quality(corners):
    if len(corners) != 4:
        return 0

    # 计算各边长度
    edges = [
        distance(corners[0], corners[1]),
        distance(corners[1], corners[2]),
        distance(corners[2], corners[3]),
        distance(corners[3], corners[0])
    ]

    # 计算角度
    angles = []
    for i in range(4):
        prev = corners[i-1]
        current = corners[i]
        next = corners[(i+1) % 4]

        # 计算两个向量
        v1 = (prev[0]-current[0], prev[1]-current[1])
        v2 = (next[0]-current[0], next[1]-current[1])

        # 计算点积
        dot = v1[0]*v2[0] + v1[1]*v2[1]

        # 计算向量模长
        v1_len = distance(prev, current)
        v2_len = distance(current, next)

        # 避免除零错误
        if v1_len == 0 or v2_len == 0:
            return 0

        # 计算角度余弦值
        cos_theta = dot / (v1_len * v2_len)

        # 确保余弦值在有效范围内
        cos_theta = max(-1.0, min(1.0, cos_theta))

        # 计算角度（度）
        angle = math.degrees(math.acos(cos_theta))
        angles.append(angle)

    # 计算质量分数（基于边长差异和角度差异）
    max_edge = max(edges)
    min_edge = min(edges)
    edge_score = 1 - (max_edge - min_edge) / max_edge if max_edge > 0 else 0

    # 计算角度与90度的平均偏差
    angle_deviation = sum(abs(angle - 90) for angle in angles) / 4
    angle_score = 1 - (angle_deviation / 45)  # 最大偏差45度

    # 综合评分（更严格的阈值）
    return (edge_score * 0.6) + (angle_score * 0.4)

# 处理串口命令
def handle_uart_commands():
    global SEND_COORDINATES, PAUSED

    if uart.any():
        try:
            cmd = uart.read(1).decode()
            print("Received command:", cmd)

            if cmd == 'P':  # 暂停/继续
                PAUSED = not PAUSED
                uart.write("Pause:" + ("ON" if PAUSED else "OFF") + "\n")

            elif cmd == 'R':  # 复位
                PAUSED = False
                current_position = 0.0
                uart.write("RESET\n")

            elif cmd == 'S':  # 开始/停止发送坐标
                SEND_COORDINATES = not SEND_COORDINATES
                uart.write("Send:" + ("ON" if SEND_COORDINATES else "OFF") + "\n")

        except Exception as e:
            print("UART error:", e)

# 计算在矩形边缘上的位置
def get_position_on_edge(avg_corners, position):
    # 确保位置在0-4范围内
    position %= 4.0

    # 确定当前所在的边
    edge_index = int(position)
    t = position - edge_index  # 在边上的位置比例 (0-1)

    # 获取边的起点和终点
    p0 = avg_corners[edge_index % 4]
    p1 = avg_corners[(edge_index + 1) % 4]

    # 线性插值计算位置
    x = int(p0[0] * (1 - t) + p1[0] * t)
    y = int(p0[1] * (1 - t) + p1[1] * t)

    return (x, y)

# 历史缓存初始化
for i in range(RECT_HISTORY_SIZE):
    rect_history.append(None)

while True:
    img = sensor.snapshot()

    # 增强图像预处理
    img.gaussian(1)  # 高斯模糊降噪
    #img.laplacian(1, sharpen=True)  # 锐化图像，增强边缘

    img_scaled = scale_image(img, 2)  # 将图像缩小为原来的一半

    # 使用更稳定的矩形检测方法
    rects = img_scaled.find_rects(threshold=28000)  # 调整阈值以提高检测精度

    # 处理串口命令
    handle_uart_commands()

    center_x, center_y = None, None
    avg_corners = []

    # 计算时间增量
    current_time = time.ticks_ms()
    time_diff = current_time - last_move_time
    last_move_time = current_time

    valid_rect_found = False
    best_rect = None
    best_score = 0

    # 寻找质量最高的矩形
    for rect in rects:
        corners = rect.corners()
        if len(corners) != 4:
            continue

        # 确保顺时针顺序
        corners = ensure_clockwise(corners)

        # 计算矩形质量分数
        score = calculate_rectangle_quality(corners)

        # 如果质量更好，则选择
        if score > best_score and score > 0.6:  # 提高最低质量阈值
            best_score = score
            best_rect = rect
            valid_rect_found = True

    if valid_rect_found:
        rect = best_rect
        corners = rect.corners()
        corners = ensure_clockwise(corners)  # 确保顺时针顺序

        # 将坐标放大到原始图像尺寸
        corners_orig = [(x*2, y*2) for (x, y) in corners]

        # 计算矩形中心
        center_x = int(sum(p[0] for p in corners_orig) / 4)
        center_y = int(sum(p[1] for p in corners_orig) / 4)

        # 计算矩形大小
        w = distance(corners_orig[0], corners_orig[1])
        h = distance(corners_orig[1], corners_orig[2])

        # 计算内缩距离（边框宽度的12%）
        border_width = min(w, h) * 0.12

        # 计算内矩形顶点
        inner_corners = []
        for i in range(4):
            # 获取当前点和相邻点
            prev = corners_orig[(i-1) % 4]
            curr = corners_orig[i]
            next = corners_orig[(i+1) % 4]

            # 计算向内法向量
            # 到前一点的向量
            v1 = (prev[0] - curr[0], prev[1] - curr[1])
            v1_len = math.sqrt(v1[0]**2 + v1[1]**2)
            if v1_len > 0:
                v1 = (v1[0]/v1_len, v1[1]/v1_len)

            # 到后一点的向量
            v2 = (next[0] - curr[0], next[1] - curr[1])
            v2_len = math.sqrt(v2[0]**2 + v2[1]**2)
            if v2_len > 0:
                v2 = (v2[0]/v2_len, v2[1]/v2_len)

            # 计算角平分线向量
            bisector = (
                (v1[0] + v2[0]),
                (v1[1] + v2[1])
            )
            bisector_len = math.sqrt(bisector[0]**2 + bisector[1]**2)
            if bisector_len > 0:
                bisector = (bisector[0]/bisector_len, bisector[1]/bisector_len)

            # 计算内缩点
            inner_x = curr[0] + bisector[0] * border_width
            inner_y = curr[1] + bisector[1] * border_width
            inner_corners.append((int(inner_x), int(inner_y)))

        # 计算平均矩形的顶点（原始图像尺寸）
        current_avg_corners = []
        for i in range(4):
            avg_x = int((corners_orig[i][0] + inner_corners[i][0]) / 2)
            avg_y = int((corners_orig[i][1] + inner_corners[i][1]) / 2)
            current_avg_corners.append((avg_x, avg_y))

        # 保存平均矩形到历史记录
        rect_history[rect_history_index] = current_avg_corners
        rect_history_index = (rect_history_index + 1) % RECT_HISTORY_SIZE

        # 计算历史平均顶点 (带权重)
        avg_corners = [[0.0, 0.0] for _ in range(4)]
        total_weight = 0

        # 遍历历史记录，计算加权平均
        for i, corners in enumerate(rect_history):
            if corners is None:
                continue

            # 最近的历史有更高的权重
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

        # 在原始图像上绘制平均矩形（蓝色）
        for i in range(4):
            start_point = avg_corners[i]
            end_point = avg_corners[(i + 1) % 4]
            img.draw_line(start_point[0], start_point[1],
                         end_point[0], end_point[1],
                         color=(0, 0, 255), thickness=2)

        # 绘制平均矩形的四个顶点（绿色）
        for i, (x, y) in enumerate(avg_corners):
            img.draw_circle(x, y, 5, color=(0, 255, 0), thickness=2)

        # 更新连续运动位置
        if not PAUSED:
            # 基于时间和速度更新位置
            current_position += MOVE_SPEED * (time_diff / 100.0)
            if current_position >= 4.0:
                current_position -= 4.0

        # 获取当前运动点位置
        target_x, target_y = get_position_on_edge(avg_corners, current_position)

        # 绘制运动点（红色）
        img.draw_circle(target_x, target_y, 8, color=(255, 0, 0), thickness=2, fill=True)


        # 显示目标点坐标
        coord_str = "({}, {})".format(target_x, target_y)
        img.draw_string(target_x + 5, target_y, coord_str, color=(255, 255, 255), scale=1.0)

        # 显示矩形质量
        quality_str = "Quality: {:.2f}".format(best_score)
        img.draw_string(10, 40, quality_str, color=(255, 255, 255), scale=1.0)

        # 显示移动速度
        speed_str = "Speed: {:.2f}".format(MOVE_SPEED)
        img.draw_string(10, 60, speed_str, color=(255, 255, 255), scale=1.0)

        # 绘制中心点（红色十字）
        img.draw_cross(center_x, center_y, color=(255, 0, 0), size=10)
        center_str = "Center ({}, {})".format(center_x, center_y)
        img.draw_string(center_x, center_y, center_str, color=(255, 255, 255))

        # 发送坐标数据
        if SEND_COORDINATES and not PAUSED and current_time - last_sent_time > 50:  # 每50ms发送一次
            # 发送中心坐标
            uart.write("C,{},{}\n".format(center_x, center_y))

            # 发送当前目标点（连续运动点）
            uart.write("T,{},{}\n".format(target_x, target_y))

            # 发送所有顶点
            uart.write("P,{},{},{},{},{},{},{},{}\n".format(
                avg_corners[0][0], avg_corners[0][1],
                avg_corners[1][0], avg_corners[1][1],
                avg_corners[2][0], avg_corners[2][1],
                avg_corners[3][0], avg_corners[3][1]
            ))

            last_sent_time = current_time

    else:
        # 更新历史记录
        rect_history[rect_history_index] = None
        rect_history_index = (rect_history_index + 1) % RECT_HISTORY_SIZE

        # 没有检测到矩形时发送错误信息
        if time.ticks_ms() % 1000 < 100:
            uart.write("E,No rectangle detected\n")

        # 显示未检测到矩形的消息
        img.draw_string(10, 10, "No rectangle detected", color=(255, 0, 0))

        # 如果没有矩形但历史中有数据，使用历史平均值
        valid_count = 0
        avg_corners = [[0.0, 0.0] for _ in range(4)]
        total_weight = 0

        # 遍历历史记录，计算加权平均
        for i, corners in enumerate(rect_history):
            if corners is None:
                continue

            # 最近的历史有更高的权重
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

            # 在原始图像上绘制平均矩形（黄色，表示历史数据）
            for i in range(4):
                start_point = avg_corners[i]
                end_point = avg_corners[(i + 1) % 4]
                img.draw_line(start_point[0], start_point[1],
                             end_point[0], end_point[1],
                             color=(255, 255, 0), thickness=1, dotted=True)

            # 显示警告信息
            img.draw_string(10, 80, "Using historical data", color=(255, 255, 0))

            # 更新连续运动位置
            if not PAUSED:
                current_position += MOVE_SPEED * (time_diff / 100.0)
                if current_position >= 4.0:
                    current_position -= 4.0

            # 获取当前运动点位置
            target_x, target_y = get_position_on_edge(avg_corners, current_position)

            # 绘制运动点（黄色）
            img.draw_circle(target_x, target_y, 8, color=(255, 255, 0), thickness=2, fill=True)

            # 显示目标点坐标
            coord_str = "({}, {})".format(target_x, target_y)
            img.draw_string(target_x + 5, target_y, coord_str, color=(255, 255, 255), scale=1.0)

    lcd.display(img)
