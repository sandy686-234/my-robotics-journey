# frequency_stub.py
import time
from hal_stub import _advance

_HZ = 20.0
_DT = 1.0 / _HZ

def setHz(hz):
    global _HZ, _DT
    _HZ = float(hz)
    _DT = 1.0 / _HZ

def tick():
    # 睡一个控制周期并推进 HAL 内部状态
    time.sleep(_DT)
    _advance(_DT)
