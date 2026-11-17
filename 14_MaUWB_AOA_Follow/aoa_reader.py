"""背景讀取 MaUWB STM32 AOA Anchor（USB-Native / VCP）。"""

from __future__ import annotations

import threading
import time

import serial

from aoa_protocol import AoaSample, parse_json_line, try_parse_binary_frame


class AoaReader:
  def __init__(self, port: str, baud: int, mode: str = 'binary'):
    self.port = port
    self.baud = baud
    self.mode = (mode or 'binary').strip().lower()
    self._ser: serial.Serial | None = None
    self._stop = threading.Event()
    self._thread: threading.Thread | None = None
    self._io_lock = threading.Lock()
    self._state_lock = threading.Lock()
    self.ok = False
    self.last_error = ''
    self.latest: AoaSample | None = None
    self.last_time = 0.0

  def start(self) -> None:
    self._thread = threading.Thread(target=self._run, daemon=True, name='aoa_reader')
    self._thread.start()

  def close(self) -> None:
    self._stop.set()
    with self._io_lock:
      if self._ser and self._ser.is_open:
        try:
          self._ser.close()
        except Exception:
          pass

  def snapshot(self) -> tuple[AoaSample | None, float]:
    with self._state_lock:
      age = time.time() - self.last_time if self.last_time else 999.0
      return self.latest, age

  def write_line(self, text: str) -> bool:
    """送設定命令（例如 USER_CMD 1）。命令需以 \\r\\n 結尾。"""
    payload = text if text.endswith('\r\n') else text.rstrip('\n') + '\r\n'
    with self._io_lock:
      if self._ser is None or not self._ser.is_open:
        if not self._open():
          return False
      try:
        assert self._ser is not None
        self._ser.write(payload.encode('ascii', errors='ignore'))
        self._ser.flush()
        return True
      except Exception as e:
        self.ok = False
        self.last_error = str(e)
        return False

  def _open(self) -> bool:
    try:
      self._ser = serial.Serial(self.port, self.baud, timeout=0.2)
      self.ok = True
      self.last_error = ''
      print(f'✅ AOA Serial: {self.port} @ {self.baud} mode={self.mode}')
      return True
    except Exception as e:
      self.ok = False
      self.last_error = str(e)
      print(f'⚠️ AOA Serial 開啟失敗 ({self.port}): {e}')
      return False

  def _publish(self, sample: AoaSample) -> None:
    with self._state_lock:
      self.latest = sample
      self.last_time = time.time()

  def _run(self) -> None:
    buf = b''
    text_carry = ''
    while not self._stop.is_set():
      with self._io_lock:
        need_open = self._ser is None or not self._ser.is_open
      if need_open:
        with self._io_lock:
          self._open()
        if not self.ok:
          time.sleep(2.0)
          continue
      try:
        with self._io_lock:
          chunk = self._ser.read(256) if self._ser else b''
        if not chunk:
          continue
        if self.mode == 'json':
          text_carry += chunk.decode('utf-8', errors='ignore')
          while '\n' in text_carry:
            line, text_carry = text_carry.split('\n', 1)
            line = line.strip()
            if not line:
              continue
            sample = parse_json_line(line)
            if sample:
              self._publish(sample)
        else:
          buf += chunk
          while True:
            sample, buf = try_parse_binary_frame(buf)
            if sample is None:
              break
            self._publish(sample)
          if len(buf) > 4096:
            buf = buf[-512:]
      except Exception as e:
        self.ok = False
        self.last_error = str(e)
        print(f'⚠️ AOA 讀取錯誤: {e}')
        with self._io_lock:
          try:
            if self._ser:
              self._ser.close()
          except Exception:
            pass
          self._ser = None
        time.sleep(1.0)
