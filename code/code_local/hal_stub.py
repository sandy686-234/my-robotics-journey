# hal_stub.py
import time, math

_HZ = 20.0
_DT = 1.0 / _HZ
_x, _y, _th = 0.0, 0.0, 0.0
_v, _w = 0.0, 0.0
_t_last = time.monotonic()

def _step(dt):
    global _x, _y, _th
    _th += _w * dt
    _x  += _v * math.cos(_th) * dt
    _y  += _v * math.sin(_th) * dt

def getPose2d():
    # 被 myCode.py 轮询
    return (_x, _y, _th)

def setV(v):
    global _v
    _v = float(v)

def setW(w):
    global _w
    _w = float(w)

# 供 Frequency.tick 调用的一步推进
def _advance(dt):
    _step(dt)
