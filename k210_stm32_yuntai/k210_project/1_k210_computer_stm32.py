import sensor, image, time, lcd, machine
from machine import UART
from fpioa_manager import fm

# 初始化LCD显示
lcd.init()  # 根据您的LCD型号修改
lcd.clear(lcd.GREEN)       # 测试LCD是否能显示颜色
print("LCD initialized")

# 初始化摄像头
sensor.reset()
sensor.set_pixformat(sensor.RGB565)  # 设置为彩色
sensor.set_framesize(sensor.QVGA)    # 设置分辨率，320x240
sensor.skip_frames(time=500)         # 等待摄像头稳定
sensor.set_vflip(0)                  # 垂直方向翻转
sensor.set_auto_gain(False)          # 建议关闭自动增益
sensor.set_auto_whitebal(False)      # 建议关闭自动白平衡
print("Camera initialized")

# 配置串口通信
#fm.register(18, fm.fpioa.UART1_TX, force=True)
#uart = UART(UART.UART1, 9600, 8, 0, 1, timeout=1000, read_buf_len=4096)
#print("UART initialized at 9600 baud")

# binding UART2 IO:6->RX, 8->TX
fm.register(6, fm.fpioa.UART2_RX)
fm.register(8, fm.fpioa.UART2_TX)

uart = UART(UART.UART2, 115200, 8, 0, 0, timeout=1000, read_buf_len=4096)


# 颜色学习设置
BOX_SIZE = 20  # 学习区域的大小
LEARNING_FRAMES = 50  # 学习过程的帧数
learning = True  # 初始为学习模式
learned_threshold = [0, 100, -128, 127, -128, 127]  # 默认阈值(非常宽松)

# ===== 颜色学习功能 =====
def learn_color_threshold():
    global learned_threshold, learning

    # 计算屏幕中心区域
    center_x = sensor.width() // 2
    center_y = sensor.height() // 2
    half_box = BOX_SIZE // 2
    r = [center_x - half_box, center_y - half_box, BOX_SIZE, BOX_SIZE]

    print("Place the object to track in the center of the screen")
    print("Press RESET button to start/stop learning")

    # 初始化颜色阈值变量
    threshold = [0, 0, 0, 0, 0, 0]  # Lmin, Lmax, Amin, Amax, Bmin, Bmax

    # 进入学习模式
    for i in range(LEARNING_FRAMES):
        img = sensor.snapshot()

        # 绘制指导信息
        img.draw_rectangle(r, color=(0, 255, 0))
        img.draw_string(50, 100, "LEARNING COLOR", color=(255, 255, 0), scale=2)
        img.draw_string(50, 130, "Place object in box", color=(255, 255, 0), scale=2)
        img.draw_string(50, 160, "Frames: %d/%d" % (i+1, LEARNING_FRAMES), color=(255, 255, 0), scale=2)
        lcd.display(img)

        # 获取学习区域的直方图
        hist = img.get_histogram(roi=r)
        if not hist:
            continue

        lo = hist.get_percentile(0.05)  # 获取5%低端值
        hi = hist.get_percentile(0.95)  # 获取95%高端值

        # 累加计算结果
        threshold[0] = (threshold[0] + lo.l_value()) // 2
        threshold[1] = (threshold[1] + hi.l_value()) // 2
        threshold[2] = (threshold[2] + lo.a_value()) // 2
        threshold[3] = (threshold[3] + hi.a_value()) // 2
        threshold[4] = (threshold[4] + lo.b_value()) // 2
        threshold[5] = (threshold[5] + hi.b_value()) // 2

    # 设置学习结果
    learned_threshold = threshold
    learning = False
    print("Color threshold learned:", learned_threshold)

# 手动触发颜色学习
print("Starting color learning...")
learn_color_threshold()
print("Color learning complete!")

# ===== 实用功能函数 =====
def find_max(blobs):
    """寻找最大色块"""
    if not blobs:
        return None

    max_size = 0
    max_blob = None
    for blob in blobs:
        if blob.pixels() > max_size:
            max_blob = blob
            max_size = blob.pixels()
    return max_blob



#==========第一次尝试，实现K210的UART输出（用USB转串口连接到电脑）,=============
def sending_data(x, y):
    """发送坐标到串口"""
    # 确保值在0-255范围内
    x_val = max(0, min(319, x))
    y_val = max(0, min(239, y))

    # 发送格式化字符串 "XxxxYyyy"
    #data_str = "X%03dY%03d" % (x_val, y_val)
    #uart.write(data_str.encode())
    #print("Sent:", data_str)

#====可以传输，但是k210要连接电脑运行其程序，能在stm32的oled上显示坐标=====


    data_str = "X{:03d}Y{:03d}\n".format(x_val, y_val)
    uart.write(data_str.encode())
    print("Sent:", data_str.strip())  # 调试输出
#============================================================

# ===== 主循环 =====
clock = time.clock()  # 用于计算帧率
print("Starting color tracking...")



while True:


    # 拍摄一张照片
    img = sensor.snapshot()

    # 查找色块 - 使用学习到的阈值
    blobs = img.find_blobs([learned_threshold],
                          pixels_threshold=100,
                          area_threshold=100,
                          merge=True,
                          margin=10)

    # 初始化位置为屏幕中心
    cx, cy = 160, 120
    found = False

    # 如果找到色块
    if blobs:
        max_b = find_max(blobs)
        if max_b:
            # 获取色块中心和尺寸
            cx = max_b.cx()
            cy = max_b.cy()
            cw = max_b.w()
            ch = max_b.h()
            found = True

            # 绘制标记
            img.draw_rectangle(max_b.rect(), color=(0, 255, 0))  # 矩形框，绿色
            img.draw_cross(cx, cy, color=(255, 0, 0), size=10)  # 中心十字，红色

    # 发送坐标到串口
    sending_data(cx, cy)


    # 在顶部绘制状态栏
    img.draw_rectangle(0, 0, 320, 25, color=(0, 0, 0), fill=True)

    # 显示坐标值
    coord_text = "X:%d Y:%d" % (cx, cy)
    img.draw_string(10, 5, coord_text, color=(255, 255, 255), scale=2)



    # 显示目标位置状态
    if found:
        status_text = "Target Found"
        status_color = (0, 255, 0)
    else:
        status_text = "No Target"
        status_color = (255, 0, 0)

    img.draw_string(150, 5, status_text, color=status_color, scale=1.5)

    # 显示当前使用的颜色阈值
    thresh_text = "L:%d-%d A:%d-%d B:%d-%d" % (
        learned_threshold[0], learned_threshold[1],
        learned_threshold[2], learned_threshold[3],
        learned_threshold[4], learned_threshold[5]
    )
    img.draw_string(10, 220, thresh_text[:28], color=(200, 200, 0), scale=1)

    # 在图像上标记位置
    img.draw_circle(cx, cy, 5, color=(0, 0, 255), thickness=2)  # 蓝色小圆点

    # 更新LCD显示
    lcd.display(img)

    # 控制循环速度
    time.sleep_ms(50)  # 大约20FPS
