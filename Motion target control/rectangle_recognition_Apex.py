import sensor, image, lcd, math, time
from machine import UART
from fpioa_manager import fm

# 初始化LCD显示屏
lcd.init()

# 初始化摄像头
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_vflip(False)
sensor.set_auto_gain(False)  # 关闭自动增益
sensor.set_auto_whitebal(False)  # 关闭自动白平衡
sensor.set_auto_exposure(False, 1000)  # 固定曝光时间(根据实际调整)
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
current_target_index = 0
last_sent_time = 0

# 缩放图像
def scale_image(img, scale):
    width = img.width() // scale
    height = img.height() // scale
    img_scaled = img.resize(width, height)
    return img_scaled

# 自动估算边框宽度
def estimate_border_width(w, h):
    min_dim = min(w, h)
    return max(2, int(min_dim * 0.05))

# 确保顶点顺序为顺时针
def ensure_clockwise(corners):
    if len(corners) != 4:
        return corners

    area = 0
    for i in range(4):
        x1, y1 = corners[i]
        x2, y2 = corners[(i + 1) % 4]
        area += (x1 * y2 - x2 * y1)

    return corners if area > 0 else [corners[3], corners[2], corners[1], corners[0]]

# 计算两点之间的距离
def distance(p1, p2):
    return math.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2)

# 处理串口命令
def handle_uart_commands():
    global SEND_COORDINATES, PAUSED, current_target_index

    if uart.any():
        try:
            cmd = uart.read(1).decode()
            print("Received command:", cmd)

            if cmd == 'P':  # 暂停/继续
                PAUSED = not PAUSED
                uart.write("Pause:" + ("ON" if PAUSED else "OFF") + "\n")

            elif cmd == 'R':  # 复位
                PAUSED = False
                current_target_index = 0
                uart.write("RESET\n")

            elif cmd == 'S':  # 开始/停止发送坐标
                SEND_COORDINATES = not SEND_COORDINATES
                uart.write("Send:" + ("ON" if SEND_COORDINATES else "OFF") + "\n")

            elif cmd == 'N':  # 下一个点
                if not PAUSED:
                    current_target_index = (current_target_index + 1) % 4
                    uart.write("NEXT\n")

        except Exception as e:
            print("UART error:", e)

while True:
    img = sensor.snapshot()

    # 新增：图像预处理
    img.gaussian(1)  # 高斯模糊降噪


    img_scaled = scale_image(img, 2)  # 将图像缩小为原来的一半
    rects = img_scaled.find_rects(threshold=30000)



    # 处理串口命令
    handle_uart_commands()

    center_x, center_y = None, None
    avg_corners = []

    if rects:
        rect = rects[0]  # 只处理第一个矩形
        corners = rect.corners()
        corners = ensure_clockwise(corners)  # 确保顺时针顺序

        # 计算矩形的中心点（缩放图像上）
        center_x_scaled = sum(p[0] for p in corners) / 4
        center_y_scaled = sum(p[1] for p in corners) / 4

        # 估算边框宽度（在缩放图像上）
        w_scaled = distance(corners[0], corners[1])
        h_scaled = distance(corners[1], corners[2])
        border_width_scaled = estimate_border_width(w_scaled, h_scaled)

        # 计算四条边的向量
        edges = []
        for i in range(4):
            x0, y0 = corners[i]
            x1, y1 = corners[(i + 1) % 4]
            dx = x1 - x0
            dy = y1 - y0
            edges.append((dx, dy))

        # 计算单位法向量（指向内部）
        normals = []
        for dx, dy in edges:
            length = math.sqrt(dx*dx + dy*dy)
            if length < 1e-5:
                normals.append((0, 0))
            else:
                # 逆时针旋转90度得到指向内部的法向量
                normals.append((-dy/length, dx/length))

        # 计算内矩形的顶点（在缩放图像上）
        inner_corners_scaled = []
        for i in range(4):
            x, y = corners[i]
            # 获取相邻两条边的法向量
            normal_prev = normals[(i - 1) % 4]
            normal_curr = normals[i]

            # 计算新顶点位置
            new_x = x + (normal_prev[0] + normal_curr[0]) * border_width_scaled
            new_y = y + (normal_prev[1] + normal_curr[1]) * border_width_scaled
            inner_corners_scaled.append((new_x, new_y))

        # 将坐标放大到原始图像尺寸
        outer_corners_orig = [(x*2, y*2) for (x, y) in corners]
        inner_corners_orig = [(x*2, y*2) for (x, y) in inner_corners_scaled]

        # 计算平均矩形的顶点（原始图像尺寸）
        for i in range(4):
            avg_x = int((outer_corners_orig[i][0] + inner_corners_orig[i][0]) / 2)
            avg_y = int((outer_corners_orig[i][1] + inner_corners_orig[i][1]) / 2)
            avg_corners.append((avg_x, avg_y))

        # 在原始图像上绘制平均矩形（蓝色）
        for i in range(4):
            start_point = avg_corners[i]
            end_point = avg_corners[(i + 1) % 4]
            img.draw_line(start_point[0], start_point[1],
                         end_point[0], end_point[1],
                         color=(0, 0, 255), thickness=2)

        # 绘制平均矩形的四个顶点（绿色）
        for i, (x, y) in enumerate(avg_corners):
            color = (0, 255, 0)  # 绿色
            if i == current_target_index:
                color = (255, 0, 0)  # 红色表示当前目标点

            img.draw_circle(x, y, 5, color=color, thickness=2)

            # 显示坐标文本
            coord_str = "({}, {})".format(int(x), int(y))
            if x < img.width() - 50 and y < img.height() - 20:
                img.draw_string(int(x), int(y), coord_str,
                               color=(255, 255, 255), scale=1.0)

        # 计算中心点 (原始图像尺寸)
        center_x = int(sum(p[0] for p in outer_corners_orig) / 4)
        center_y = int(sum(p[1] for p in outer_corners_orig) / 4)
        last_center = (center_x, center_y)

        # 绘制中心点（红色十字）
        img.draw_cross(center_x, center_y, color=(255, 0, 0), size=10)
        img.draw_string(center_x, center_y, "Center", color=(255, 255, 255))

        # 发送坐标数据
        current_time = time.ticks_ms()
        if SEND_COORDINATES and not PAUSED and current_time - last_sent_time > 100:  # 每100ms发送一次
            # 发送中心坐标
            uart.write("C,{},{}\n".format(center_x, center_y))

            # 发送当前目标点
            target_x, target_y = avg_corners[current_target_index]
            uart.write("T,{},{}\n".format(target_x, target_y))

            # 发送所有顶点（调试用）
            uart.write("P,{},{},{},{},{},{},{},{}\n".format(
                avg_corners[0][0], avg_corners[0][1],
                avg_corners[1][0], avg_corners[1][1],
                avg_corners[2][0], avg_corners[2][1],
                avg_corners[3][0], avg_corners[3][1]
            ))

            last_sent_time = current_time

            # 自动移动到下一个点（每隔2秒）
            if current_time % 2000 < 100:  # 每2秒移动一次
                current_target_index = (current_target_index + 1) % 4
    else:
        # 没有检测到矩形时发送错误信息
        if time.ticks_ms() % 1000 < 100:
            uart.write("E,No rectangle detected\n")

        # 显示未检测到矩形的消息
        img.draw_string(10, 10, "No rectangle detected", color=(255, 0, 0))

    lcd.display(img)
