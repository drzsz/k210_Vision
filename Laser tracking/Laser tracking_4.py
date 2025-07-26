import sensor, image, time, lcd, machine
from machine import UART
from fpioa_manager import fm
import math

# ====== 初始化摄像头 ======
def init_camera():
    try:
        sensor.reset()
        sensor.set_pixformat(sensor.RGB565)
        sensor.set_framesize(sensor.QVGA)  # 320x240
        sensor.set_vflip(False)  # 根据硬件需要调整
        sensor.set_hmirror(True)
        sensor.set_auto_gain(False)
        sensor.set_auto_whitebal(False)
        sensor.set_contrast(3)
        sensor.set_brightness(-3)
        sensor.skip_frames(time=500)  # 等待设置生效
        print("Camera initialized successfully")
        return True
    except Exception as e:
        print("Camera init error:", e)
        return False

# ====== 初始化LCD ======
def init_lcd():
    try:
        lcd.init()
        lcd.clear(lcd.GREEN)
        print("LCD initialized")
        return True
    except Exception as e:
        print("LCD init error:", e)
        return False

# ====== 初始化串口 ======
def init_uart():
    try:
        fm.register(6, fm.fpioa.UART2_RX)
        fm.register(8, fm.fpioa.UART2_TX)
        uart = UART(UART.UART2, 115200, 8, 0, 0, timeout=1000, read_buf_len=4096)
        print("UART initialized")
        return uart
    except Exception as e:
        print("UART init error:", e)
        return None

# ====== 激光点检测函数 ======
def find_laser_point(img):
    # 创建灰度图像用于检测
    gray_img = img.copy().to_grayscale()

    # 查找最亮的像素
    max_brightness = 0
    max_x, max_y = 0, 0

    # 优化：只扫描图像中心区域 (160x120)
    for y in range(60, 180, 4):  # 间隔4行扫描
        for x in range(80, 240, 4):  # 间隔4列扫描
            pix = gray_img.get_pixel(x, y)
            if pix > max_brightness:
                max_brightness = pix
                max_x, max_y = x, y

    # 检查是否达到最小亮度
    if max_brightness < 180:  # 降低阈值
        return None, None, max_brightness

    # 检查周围区域确认是激光点
    count = 0
    for dy in range(-2, 3):
        for dx in range(-2, 3):
            nx, ny = max_x + dx, max_y + dy
            if 0 <= nx < gray_img.width() and 0 <= ny < gray_img.height():
                if gray_img.get_pixel(nx, ny) > 150:  # 降低阈值
                    count += 1

    # 如果是孤立的亮点，则认为是激光点
    if count < 5:  # 周围亮点太少，可能是噪声
        return None, None, max_brightness

    return max_x, max_y, max_brightness

# ====== 矩形识别函数 ======
def detect_rectangle(img):
    """
    识别图像中最大的矩形并返回四个顶点
    """
    # 转换为灰度并增强对比度
    gray = img.copy().to_grayscale()
    gray.histeq(adaptive=True, clip_limit=3.0)

    # 边缘检测
    edges = gray.find_edges(image.EDGE_CANNY, threshold=(30, 70))

    # 寻找所有轮廓
    contours = edges.find_contours(threshold=1500, roi=(40, 30, 240, 180))

    best_rect = None
    best_score = 0

    for contour in contours:
        # 计算轮廓面积
        area = contour.area()
        if area < 5000 or area > 30000:  # 排除过大或过小的轮廓
            continue

        # 获取最小外接矩形
        rect = contour.min_rect()
        w, h = rect.w(), rect.h()

        # 计算矩形度（轮廓面积/外接矩形面积）
        rect_area = w * h
        if rect_area == 0:
            continue

        rect_score = area / rect_area

        # 计算宽高比
        aspect = max(w, h) / min(w, h) if min(w, h) > 0 else 0

        # 筛选近似矩形的轮廓
        if rect_score > 0.7 and 0.7 < aspect < 1.3:
            # 计算得分 = 矩形度 + (1 - 宽高比偏差)
            score = rect_score + (1 - abs(1 - aspect))

            if score > best_score:
                best_score = score
                best_rect = rect

    if best_rect is None:
        return None

    # 获取并排序顶点（左上、右上、右下、左下）
    corners = list(best_rect.corners())

    # 按X坐标排序
    corners.sort(key=lambda p: p[0])

    # 按Y坐标将左右两边分开排序
    left_points = sorted(corners[:2], key=lambda p: p[1])
    right_points = sorted(corners[2:], key=lambda p: p[1])

    sorted_corners = [left_points[0], right_points[0], right_points[1], left_points[1]]

    return sorted_corners

# ====== 路径规划类 ======
class PathPlanner:
    def __init__(self):
        self.path_points = []
        self.current_target = 0
        self.path_step = 0
        self.TOTAL_STEPS = 60  # 每段路径步数
        self.ready = False

    def set_path_points(self, points):
        """设置路径点（必须4个点）"""
        if len(points) < 4:
            return False

        self.path_points = points
        self.ready = True
        return True

    def generate_target(self):
        """生成当前目标点"""
        if not self.ready or not self.path_points:
            return None, None

        start_idx = self.current_target
        end_idx = (self.current_target + 1) % len(self.path_points)

        start_x, start_y = self.path_points[start_idx]
        end_x, end_y = self.path_points[end_idx]

        # 线性插值生成目标点
        progress = self.path_step / self.TOTAL_STEPS
        target_x = start_x + (end_x - start_x) * progress
        target_y = start_y + (end_y - start_y) * progress

        self.path_step += 1
        if self.path_step > self.TOTAL_STEPS:
            self.path_step = 0
            self.current_target = (self.current_target + 1) % len(self.path_points)

        return int(target_x), int(target_y)

    def reset(self):
        self.path_points = []
        self.current_target = 0
        self.path_step = 0
        self.ready = False

# ====== 主程序 ======
def main():
    # 初始化硬件
    if not init_camera():
        print("Camera init failed, system halted")
        return

    if not init_lcd():
        print("LCD init failed, continuing without display")

    uart = init_uart()
    if uart is None:
        print("UART init failed, system halted")
        return

    # 初始化路径规划器
    planner = PathPlanner()

    # PID参数
    Kp = 0.025  # 比例系数
    servo_x, servo_y = 90, 90  # 舵机初始角度

    print("Starting laser tracking system...")

    last_rect_time = time.ticks_ms()
    rect_update_interval = 1000  # 每1秒更新一次矩形

    while True:
        try:
            # 尝试获取图像
            img = sensor.snapshot()
        except Exception as e:
            print("Snapshot error:", e)
            # 尝试重新初始化摄像头
            init_camera()
            time.sleep_ms(500)
            continue

        # 定期检测矩形（每秒一次）
        current_time = time.ticks_ms()
        if time.ticks_diff(current_time, last_rect_time) > rect_update_interval:
            rect_corners = detect_rectangle(img)

            if rect_corners:
                print("Detected rectangle corners:", rect_corners)
                planner.set_path_points(rect_corners)

                # 在图像上绘制矩形
                for i in range(4):
                    next_i = (i + 1) % 4
                    img.draw_line(rect_corners[i][0], rect_corners[i][1],
                                 rect_corners[next_i][0], rect_corners[next_i][1],
                                 color=(255, 0, 0), thickness=2)

            last_rect_time = current_time

        # 生成目标点
        if planner.ready:
            target_x, target_y = planner.generate_target()
        else:
            # 如果没有检测到矩形，使用图像中心
            target_x, target_y = 160, 120

        # 识别激光点
        laser_x, laser_y, brightness = find_laser_point(img)

        # 控制计算
        if laser_x is not None and laser_y is not None:
            # 计算误差
            error_x = target_x - laser_x
            error_y = target_y - laser_y

            # PID控制（只有比例项）
            control_x = error_x * Kp
            control_y = error_y * Kp

            # 更新舵机角度
            servo_x += control_x
            servo_y += control_y

            # 限幅保护
            servo_x = max(30, min(150, servo_x))
            servo_y = max(30, min(150, servo_y))

            # 发送舵机角度给STM32
            data_str = "X{:03d}Y{:03d}\n".format(int(servo_x), int(servo_y))
            try:
                uart.write(data_str.encode())
            except Exception as e:
                print("UART write error:", e)

        # 在图像上绘制目标点和激光点
        if planner.ready:
            img.draw_cross(target_x, target_y, color=(0, 255, 0), size=15, thickness=2)

            # 标记路径点
            for i, point in enumerate(planner.path_points):
                color = (0, 0, 255) if i == planner.current_target else (255, 128, 0)
                img.draw_cross(point[0], point[1], color=color, size=10, thickness=1)

        if laser_x is not None and laser_y is not None:
            img.draw_circle(laser_x, laser_y, 5, color=(0, 255, 255), thickness=2)

        # 显示状态信息
        img.draw_rectangle(0, 0, 320, 30, color=(255, 255, 255), fill=True)

        if planner.ready:
            status_text = "Tracking: {} ({},{})".format(planner.current_target, target_x, target_y)
        else:
            status_text = "No rectangle detected"

        img.draw_string(10, 5, status_text, color=(0, 0, 0), scale=1.5)

        # 显示到LCD
        try:
            lcd.display(img)
        except Exception as e:
            print("LCD display error:", e)

        # 控制循环速度
        time.sleep_ms(20)

# 启动主程序
if __name__ == "__main__":
    main()
