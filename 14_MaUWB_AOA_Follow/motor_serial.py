"""UNO 馬達命令序列埠（F/L/R/Z/S/X）。"""

from __future__ import annotations

import time

import serial


class MotorCommander:
  def __init__(self, port: str, baud: int, interval_s: float = 0.15):
    self.port = port
    self.baud = baud
    self.interval_s = interval_s
    self._ser: serial.Serial | None = None
    self.ok = False
    self.last_error = ''
    self._last_sent = ''
    self._last_sent_at = 0.0

  def open(self) -> bool:
    try:
      self._ser = serial.Serial(self.port, self.baud, timeout=0.1)
      self.ok = True
      self.last_error = ''
      print(f'✅ Motor Serial: {self.port} @ {self.baud}')
      self.send('S', force=True)
      return True
    except Exception as e:
      self.ok = False
      self.last_error = str(e)
      print(f'⚠️ Motor Serial 開啟失敗 ({self.port}): {e}')
      return False

  def close(self) -> None:
    try:
      self.send('X', force=True)
    except Exception:
      pass
    if self._ser and self._ser.is_open:
      try:
        self._ser.close()
      except Exception:
        pass

  def send(self, cmd: str, force: bool = False) -> bool:
    cmd = (cmd or 'S').upper()[:1]
    now = time.time()
    if (
        not force
        and cmd == self._last_sent
        and (now - self._last_sent_at) < self.interval_s
    ):
      return True

    if self._ser is None or not self._ser.is_open:
      if not self.open():
        return False
    try:
      assert self._ser is not None
      self._ser.write(f'{cmd}\n'.encode('ascii'))
      self._ser.flush()
      if cmd != self._last_sent:
        print(f'📤 Motor -> UNO: {cmd}')
      self._last_sent = cmd
      self._last_sent_at = now
      self.ok = True
      self.last_error = ''
      return True
    except Exception as e:
      self.ok = False
      self.last_error = str(e)
      print(f'⚠️ 送馬達命令失敗: {e}')
      try:
        if self._ser:
          self._ser.close()
      except Exception:
        pass
      self._ser = None
      return False

  def estop(self) -> bool:
    return self.send('X', force=True)
