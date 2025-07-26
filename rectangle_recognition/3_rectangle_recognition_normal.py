#========识别内外矩形，并传输内矩形坐标========
#import sensor
#import image
#import lcd
#import math
#
## 初始化LCD显示屏
#lcd.init()
#
## 初始化摄像头
#sensor.reset()
#sensor.set_pixformat(sensor.RGB565)
#sensor.set_framesize(sensor.QVGA)
#sensor.set_vflip(True)
#sensor.run(1)
#
## 缩放图像
#def scale_image(img, scale):
#    width = img.width() // scale
#    height = img.height() // scale
#    img_scaled = img.resize(width, height)
#    return img_scaled
#
#while True:
#    # 获取摄像头图像
#    img = sensor.snapshot()
#
#    # 缩放图像
#    img_scaled = scale_image(img, 2)  # 将图像缩小为原来的一半
#
#    # 在缩放后的图像中查找所有矩形
#    rects = img_scaled.find_rects(threshold=50000)
#
#    # 存储有效矩形对（外矩形和内矩形）
#    valid_pairs = []
#
#    # 筛选嵌套矩形对
#    for i, r_outer in enumerate(rects):
#        for j, r_inner in enumerate(rects):
#            if i == j:
#                continue
#
#            # 检查内矩形是否完全在外矩形内
#            if (r_inner.x() > r_outer.x() and
#                r_inner.y() > r_outer.y() and
#                r_inner.x() + r_inner.w() < r_outer.x() + r_outer.w() and
#                r_inner.y() + r_inner.h() < r_outer.y() + r_outer.h()):
#
#                # 检查宽高比是否相似（避免误匹配）
#                ratio_outer = r_outer.w() / r_outer.h()
#                ratio_inner = r_inner.w() / r_inner.h()
#                if abs(ratio_outer - ratio_inner) < 0.2:  # 宽高比差异阈值
#                    valid_pairs.append((r_outer, r_inner))
#                    break  # 每个外矩形只匹配一个内矩形
#
#    # 处理每个有效矩形对
#    for outer, inner in valid_pairs:
#        # 将坐标放大到原始图像尺寸
#        outer_scaled = [outer.x() * 2, outer.y() * 2, outer.w() * 2, outer.h() * 2]
#        inner_scaled = [inner.x() * 2, inner.y() * 2, inner.w() * 2, inner.h() * 2]
#
#        # 计算平均矩形坐标
#        avg_x = (outer_scaled[0] + inner_scaled[0]) // 2
#        avg_y = (outer_scaled[1] + inner_scaled[1]) // 2
#        avg_w = (outer_scaled[2] + inner_scaled[2]) // 2
#        avg_h = (outer_scaled[3] + inner_scaled[3]) // 2
#
#        # 创建平均矩形对象
#        avg_rect = (avg_x, avg_y, avg_w, avg_h)
#
#        # 绘制平均矩形（蓝色边框）
#        img.draw_rectangle(avg_rect, color=(0, 0, 255), thickness=2)
#
#        # 计算并绘制四个顶点
#        x, y, w, h = avg_rect
#        points = [
#            (x, y),            # 左上角
#            (x + w, y),        # 右上角
#            (x + w, y + h),    # 右下角
#            (x, y + h)         # 左下角
#        ]
#
#        # 绘制顶点和坐标
#        for i, (px, py) in enumerate(points):
#            img.draw_circle(px, py, 5, color=(0, 255, 0), thickness=2)
#            img.draw_string(px + 5, py + 5, f"({px},{py})",
#                           color=(255, 255, 255), scale=1.2)
#            print(f"Point {i+1}: ({px}, {py})")
#
#        # 打印中心点坐标
#        center_x = x + w // 2
#        center_y = y + h // 2
#        print(f"Center: ({center_x}, {center_y})")
#        img.draw_cross(center_x, center_y, color=(255, 0, 0), size=15)
#        img.draw_string(center_x, center_y, f"Center: ({center_x},{center_y})",
#                       color=(255, 0, 0), scale=1.2)
#
#    # 在LCD显示屏上显示图像
#    lcd.display(img)

#=========================================================================

#==========能识别摆放正常的矩形，一旦矩形旋转则只能以各顶点所在边将其补全===========

import sensor
import image
import lcd
import math

# 初始化LCD显示屏
lcd.init()

# 初始化摄像头
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_vflip(False)
sensor.run(1)

# 缩放图像
def scale_image(img, scale):
    width = img.width() // scale
    height = img.height() // scale
    img_scaled = img.resize(width, height)
    return img_scaled

# 自动估算边框宽度（根据矩形大小比例）
def estimate_border_width(w, h):
    # 根据矩形大小自适应估算边框宽度
    min_dim = min(w, h)
    return max(2, int(min_dim * 0.05))  # 边框宽度约为矩形最小尺寸的5%

while True:
    # 获取摄像头图像
    img = sensor.snapshot()

    # 缩放图像
    img_scaled = scale_image(img, 2)  # 将图像缩小为原来的一半

    # 在缩放后的图像中查找矩形
    rects = img_scaled.find_rects(threshold=50000)

    # 将矩形坐标放大到原始图像尺寸
    for r in rects:
        # 原始外边框坐标
        x_outer = r.x() * 2
        y_outer = r.y() * 2
        w_outer = r.w() * 2
        h_outer = r.h() * 2

        # 自动估算边框宽度
        border_width = estimate_border_width(w_outer, h_outer)

        # 计算内部矩形坐标（向内收缩）
        x_inner = x_outer + border_width
        y_inner = y_outer + border_width
        w_inner = w_outer - 2 * border_width
        h_inner = h_outer - 2 * border_width

        # 确保内部矩形尺寸有效
        if w_inner <= 0 or h_inner <= 0:
            continue  # 跳过无效矩形

        # 计算平均矩形（中间矩形）
        x_avg = (x_outer + x_inner) // 2
        y_avg = (y_outer + y_inner) // 2
        w_avg = (w_outer + w_inner) // 2
        h_avg = (h_outer + h_inner) // 2

        # 计算平均矩形的四个顶点
        avg_points = [
            (x_avg, y_avg),                      # 左上角
            (x_avg + w_avg, y_avg),              # 右上角
            (x_avg + w_avg, y_avg + h_avg),      # 右下角
            (x_avg, y_avg + h_avg)               # 左下角
        ]

        # 只绘制平均矩形（蓝色）
        img.draw_rectangle(x_avg, y_avg, w_avg, h_avg, color=(0, 0, 255), thickness=2)

        # 绘制平均矩形的四个顶点（绿色）
        for (px, py) in avg_points:
            img.draw_circle(px, py, 5, color=(0, 255, 0), thickness=2)
            # 只在每个点附近显示坐标（避免重叠）
            if px < img.width() - 50 and py < img.height() - 20:  # 确保不超出屏幕
                img.draw_string(px, py, "({}, {})".format(px, py), color=(255, 255, 255), scale=1)  # 缩小字体

    # 在LCD显示屏上显示图像
    lcd.display(img)

    # 减少打印频率以避免卡顿
    if len(rects) > 0:
        # 只打印一个矩形的信息（如果有多个）
        r = rects[0]
        # 计算平均矩形
        x_avg = (r.x() * 2 + (r.x() * 2 + estimate_border_width(r.w()*2, r.h()*2))) // 2
        y_avg = (r.y() * 2 + (r.y() * 2 + estimate_border_width(r.w()*2, r.h()*2))) // 2
        w_avg = (r.w()*2 + (r.w()*2 - 2*estimate_border_width(r.w()*2, r.h()*2))) // 2
        h_avg = (r.h()*2 + (r.h()*2 - 2*estimate_border_width(r.w()*2, r.h()*2))) // 2

        print("Avg Rectangle: x={}, y={}, w={}, h={}".format(x_avg, y_avg, w_avg, h_avg))

