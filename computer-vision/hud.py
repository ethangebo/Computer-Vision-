# hud display
import time
import cv2

_toast_txt, _toast_until = "", 0.0

def toast(msg, sec=2.0):
    global _toast_txt, _toast_until
    _toast_txt = msg
    _toast_until = time.time() + sec

def draw_toast(img):
    if time.time() > _toast_until or not _toast_txt:
        return
    cv2.putText(img, _toast_txt, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,0), 3)
    cv2.putText(img, _toast_txt, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
