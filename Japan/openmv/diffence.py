import sensor
import time
import math
from machine import UART

uart = UART(1, 115200, timeout=10)
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.VGA)

# --- カメラの明るさ・色設定 ---
target_exposure = 40000
sensor.set_auto_exposure(False, exposure_us=target_exposure)
sensor.set_auto_gain(False, gain_db=10)
sensor.set_auto_whitebal(False, rgb_gain_db=(60, 58, 61))

# --- 設定エリア ---
yellow_threshold = (49, 75, 0, 19, 127, -3)
blue_threshold = (19, 46, -103, 3, -24, -13)
center_x = 335
center_y = 240

def get_angle(blob):
    dx = blob.cx() - center_x
    dy = blob.cy() - center_y
    angle_rad = math.atan2(dy, dx)
    angle_deg = math.degrees(angle_rad)
    robot_angle = 90 - angle_deg
    if robot_angle < 0:   robot_angle += 360
    if robot_angle >= 360: robot_angle -= 360
    return int(robot_angle)

sensor.skip_frames(time=2000)
clock = time.clock()

while True:
    clock.tick()
    img = sensor.snapshot()

    # 初期化（検知なしのデフォルト値）
    y_flag = 0; y_angle = 0; y_height = 0
    b_flag = 0; b_angle = 0; b_height = 0

    # 1. 黄色ゴールの検知
    yellow_blobs = img.find_blobs([yellow_threshold], pixels_threshold=100, area_threshold=100, merge=True)
    if yellow_blobs:
        y_target = max(yellow_blobs, key=lambda b: b.pixels())
        y_angle  = get_angle(y_target)
        y_height = y_target.h()   # ★ blobの縦幅（ピクセル）
        y_flag   = 1
        img.draw_rectangle(y_target.rect(), color=(255, 255, 0))
        img.draw_line(center_x, center_y, y_target.cx(), y_target.cy(), color=(255, 255, 0))
        img.draw_string(y_target.x(), y_target.y()-20,
                        f"Y:{y_angle} H:{y_height}", color=(255, 255, 0), scale=2)
        print(f"Yellow: angle={y_angle} deg, height={y_height} px")

    # 2. 青色ゴールの検知
    blue_blobs = img.find_blobs([blue_threshold], pixels_threshold=100, area_threshold=100, merge=True)
    if blue_blobs:
        b_target = max(blue_blobs, key=lambda b: b.pixels())
        b_angle  = get_angle(b_target)
        b_height = b_target.h()   # ★ blobの縦幅（ピクセル）
        b_flag   = 1
        img.draw_rectangle(b_target.rect(), color=(0, 0, 255))
        img.draw_line(center_x, center_y, b_target.cx(), b_target.cy(), color=(0, 0, 255))
        img.draw_string(b_target.x(), b_target.y()-20,
                        f"B:{b_angle} H:{b_height}", color=(0, 0, 255), scale=2)
        print(f"Blue: angle={b_angle} deg, height={b_height} px")

    img.draw_cross(center_x, center_y, color=(255, 255, 255))

    # --- マイコンからのリクエスト待機 ---
    if uart.any():
        request = uart.read(1)
        if request and request[0] == 253:
            # 送信パケット作成（各値を7bitx2バイトで送信）
            out_data = bytearray([
                253,
                y_flag,
                y_angle  & 0x7F, (y_angle  >> 7) & 0x7F,   # 黄角度
                y_height & 0x7F, (y_height >> 7) & 0x7F,   # ★ 黄高さ
                b_flag,
                b_angle  & 0x7F, (b_angle  >> 7) & 0x7F,   # 青角度
                b_height & 0x7F, (b_height >> 7) & 0x7F,   # ★ 青高さ
            ])
            uart.write(out_data)
            print("Sent Data to MCU!")
        else:
            print("Unknown Request: %s" % request)