"""
pico_rfid.py  –  MicroPython firmware for Raspberry Pi Pico (RP2040)
Bridges an RC522 RFID reader over USB-serial so a host computer can
read and write MIFARE Classic tags.

Protocol (newline-terminated JSON over USB CDC):
  Host → Pico  {"cmd": "scan"}
  Host → Pico  {"cmd": "read",  "block": 4}
  Host → Pico  {"cmd": "write", "block": 4, "data": [0..15 bytes]}
  Host → Pico  {"cmd": "ping"}

  Pico → Host  {"status": "ok",    ...extra fields...}
  Pico → Host  {"status": "error", "msg": "..."}

Wire the RC522 to the Pico using the pin variables below.
"""

import sys
import json
import time
from machine import Pin, SPI

# ── Pin configuration ──────────────────────────────────────────────────────────
# Change these to match your physical wiring.
SPI_ID   = 0          # SPI bus: 0 or 1
PIN_SCK  = 18          # SPI clock   (GP18 = SPI0 SCK)
PIN_MOSI = 19          # SPI MOSI    (GP19 = SPI0 TX)
PIN_MISO = 16          # SPI MISO    (GP16 = SPI0 RX)
PIN_CS   = 17          # Chip-select (GP17 = SPI0 CSn, driven as GPIO)
PIN_RST  = 21          # Reset
# ──────────────────────────────────────────────────────────────────────────────


# ── RC522 register addresses ───────────────────────────────────────────────────
REG_COMMAND        = 0x01
REG_COM_I_EN       = 0x02
REG_DIV_I_EN       = 0x03
REG_COM_IRQ        = 0x04
REG_DIV_IRQ        = 0x05
REG_ERROR          = 0x06
REG_STATUS2        = 0x08
REG_FIFO_DATA      = 0x09
REG_FIFO_LEVEL     = 0x0A
REG_CONTROL        = 0x0C
REG_BIT_FRAMING    = 0x0D
REG_COLL           = 0x0E
REG_MODE           = 0x11
REG_TX_CONTROL     = 0x14
REG_TX_ASK         = 0x15
REG_CRC_RESULT_MSB = 0x21
REG_CRC_RESULT_LSB = 0x22
REG_MOD_WIDTH      = 0x24
REG_RFC_FG         = 0x26
REG_T_MODE         = 0x2A
REG_T_PRESCALER    = 0x2B
REG_T_RELOAD_H     = 0x2C
REG_T_RELOAD_L     = 0x2D
REG_VERSION        = 0x37

CMD_IDLE           = 0x00
CMD_CALC_CRC       = 0x03
CMD_TRANSCEIVE     = 0x0C
CMD_MF_AUTH_KEY_A  = 0x0E
CMD_SOFT_RESET     = 0x0F

PICC_REQIDL        = 0x26
PICC_REQALL        = 0x52
PICC_ANTICOLL      = 0x93
PICC_SELECT_TAG    = 0x93
PICC_AUTH_KEY_A    = 0x60
PICC_READ          = 0x30
PICC_WRITE         = 0xA0
PICC_HALT          = 0x50
MIFARE_ACK         = 0x0A

MI_OK              = 0
MI_NO_TAG          = 1
MI_ERR             = 2
# ──────────────────────────────────────────────────────────────────────────────


class RC522:
    """Minimal RC522 driver for MicroPython."""

    def __init__(self, spi_id=SPI_ID, sck=PIN_SCK, mosi=PIN_MOSI,
                 miso=PIN_MISO, cs=PIN_CS, rst=PIN_RST):
        self._cs  = Pin(cs,  Pin.OUT)
        self._rst = Pin(rst, Pin.OUT)
        self._spi = SPI(spi_id, baudrate=1_000_000, polarity=0, phase=0,
                        sck=Pin(sck), mosi=Pin(mosi), miso=Pin(miso))
        self._cs.value(1)
        self._reset()
        self._init()

    # ── Low-level SPI helpers ─────────────────────────────────────────────────
    def _reset(self):
        self._rst.value(0)
        time.sleep_us(50)
        self._rst.value(1)
        time.sleep_ms(50)

    def _write(self, reg, val):
        self._cs.value(0)
        self._spi.write(bytes([(reg << 1) & 0x7E, val]))
        self._cs.value(1)

    def _read(self, reg):
        self._cs.value(0)
        self._spi.write(bytes([((reg << 1) & 0x7E) | 0x80]))
        buf = bytearray(1)
        self._spi.readinto(buf)
        self._cs.value(1)
        return buf[0]

    def _set_bits(self, reg, mask):
        self._write(reg, self._read(reg) | mask)

    def _clear_bits(self, reg, mask):
        self._write(reg, self._read(reg) & (~mask))

    def _init(self):
        self._write(REG_T_MODE,      0x8D)
        self._write(REG_T_PRESCALER, 0x3E)
        self._write(REG_T_RELOAD_H,  0x00)
        self._write(REG_T_RELOAD_L,  0x1E)
        self._write(REG_TX_ASK,      0x40)
        self._write(REG_MODE,        0x3D)
        self._set_bits(REG_TX_CONTROL, 0x03)   # enable antenna

    # ── CRC ──────────────────────────────────────────────────────────────────
    def _calc_crc(self, data):
        self._clear_bits(REG_DIV_IRQ, 0x04)
        self._set_bits(REG_FIFO_LEVEL, 0x80)
        for b in data:
            self._write(REG_FIFO_DATA, b)
        self._write(REG_COMMAND, CMD_CALC_CRC)
        for _ in range(5000):
            if self._read(REG_DIV_IRQ) & 0x04:
                break
        return [self._read(REG_CRC_RESULT_LSB),
                self._read(REG_CRC_RESULT_MSB)]

    # ── Transceive ────────────────────────────────────────────────────────────
    def _transceive(self, send_data):
        self._write(REG_COM_I_EN,   0x77)
        self._set_bits(REG_FIFO_LEVEL, 0x80)
        self._write(REG_COMMAND, CMD_IDLE)
        self._write(REG_COM_IRQ, 0x7F)    # clear all IRQ flags after CMD_IDLE
        for b in send_data:
            self._write(REG_FIFO_DATA, b)
        self._write(REG_COMMAND, CMD_TRANSCEIVE)
        self._set_bits(REG_BIT_FRAMING, 0x80)

        waiting = 2000
        while True:
            irq = self._read(REG_COM_IRQ)
            waiting -= 1
            if waiting == 0 or (irq & 0x01) or (irq & 0x30):
                break
        self._clear_bits(REG_BIT_FRAMING, 0x80)

        if waiting == 0:
            return MI_ERR, None
        if self._read(REG_ERROR) & 0x1B:
            return MI_ERR, None

        irq = self._read(REG_COM_IRQ)
        if not (irq & 0x30):
            return MI_NO_TAG, None

        n = self._read(REG_FIFO_LEVEL)
        last_bits = self._read(REG_CONTROL) & 0x07
        if last_bits:
            bits = (n - 1) * 8 + last_bits
        else:
            bits = n * 8

        if n == 0:
            n = 1
        recv = [self._read(REG_FIFO_DATA) for _ in range(n)]
        return MI_OK, recv

    # ── Public RFID API ───────────────────────────────────────────────────────
    def request(self, req_mode):
        self._write(REG_BIT_FRAMING, 0x07)
        status, back = self._transceive([req_mode])
        if status != MI_OK or len(back) != 2:
            return MI_ERR, None
        return MI_OK, back

    def anticoll(self):
        self._write(REG_BIT_FRAMING, 0x00)
        status, back = self._transceive([PICC_ANTICOLL, 0x20])
        if status == MI_OK and back and len(back) == 5:
            crc = 0
            for b in back[:4]:
                crc ^= b
            if crc != back[4]:
                return MI_ERR, None
        return status, back

    def select_tag(self, uid):
        buf = [PICC_SELECT_TAG, 0x70] + uid
        crc = self._calc_crc(buf)
        buf += crc
        status, back = self._transceive(buf)
        if status == MI_OK and back and len(back) == 3:
            return MI_OK, back[0]   # SAK byte
        return MI_ERR, None

    def auth(self, auth_mode, block, key, uid):
        buf = [auth_mode, block] + key + uid[:4]
        self._write(REG_COM_I_EN, 0x77)
        self._set_bits(REG_FIFO_LEVEL, 0x80)
        self._write(REG_COMMAND, CMD_IDLE)
        self._write(REG_COM_IRQ, 0x7F)
        for b in buf:
            self._write(REG_FIFO_DATA, b)
        self._write(REG_COMMAND, CMD_MF_AUTH_KEY_A)
        waiting = 2000
        while waiting:
            irq = self._read(REG_COM_IRQ)
            waiting -= 1
            if (irq & 0x01) or (irq & 0x10):
                break
        if waiting == 0 or not (self._read(REG_COM_IRQ) & 0x10):
            return MI_ERR
        if not (self._read(REG_STATUS2) & 0x08):
            return MI_ERR
        return MI_OK

    def stop_crypto(self):
        self._clear_bits(REG_STATUS2, 0x08)

    def read_block(self, block):
        buf = [PICC_READ, block]
        buf += self._calc_crc(buf)
        status, back = self._transceive(buf)
        if status == MI_OK and back and len(back) >= 16:
            return MI_OK, back[:16]
        return MI_ERR, None

    def write_block(self, block, data):
        if len(data) != 16:
            return MI_ERR
        buf = [PICC_WRITE, block]
        buf += self._calc_crc(buf)
        status, back = self._transceive(buf)
        if status != MI_OK or len(back) < 1 or (back[0] & 0x0F) != MIFARE_ACK:
            return MI_ERR
        buf2 = list(data) + self._calc_crc(data)
        status, back = self._transceive(buf2)
        if status != MI_OK or len(back) < 1 or (back[0] & 0x0F) != MIFARE_ACK:
            return MI_ERR
        return MI_OK

    def halt(self):
        buf = [PICC_HALT, 0x00]
        buf += self._calc_crc(buf)
        self._transceive(buf)

    @property
    def version(self):
        return self._read(REG_VERSION)


# ── USB-serial command loop ────────────────────────────────────────────────────

DEFAULT_KEY = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]

def _send(obj):
    sys.stdout.write(json.dumps(obj) + "\n")

def _sector_of(block):
    return block // 4

def _first_block_of_sector(sector):
    return sector * 4

def handle_scan(reader):
    status, _ = reader.request(PICC_REQIDL)
    if status != MI_OK:
        _send({"status": "no_tag"})
        return
    status, uid = reader.anticoll()
    if status != MI_OK:
        _send({"status": "error", "msg": "anticoll failed"})
        return
    status, sak = reader.select_tag(uid)
    if status != MI_OK:
        _send({"status": "error", "msg": "select failed"})
        return
    reader.halt()
    _send({"status": "ok", "uid": uid, "sak": sak})

def handle_read(reader, block, key=None):
    if key is None:
        key = DEFAULT_KEY
    status, _ = reader.request(PICC_REQALL)
    if status != MI_OK:
        _send({"status": "no_tag"})
        return
    status, uid = reader.anticoll()
    if status != MI_OK:
        _send({"status": "error", "msg": "anticoll failed"})
        return
    reader.select_tag(uid)
    sector = _sector_of(block)
    trailer = _first_block_of_sector(sector) + 3
    status = reader.auth(PICC_AUTH_KEY_A, trailer, key, uid)
    if status != MI_OK:
        reader.stop_crypto()
        reader.halt()
        _send({"status": "error", "msg": "auth failed"})
        return
    status, data = reader.read_block(block)
    reader.stop_crypto()
    reader.halt()
    if status == MI_OK:
        _send({"status": "ok", "block": block, "data": data, "uid": uid})
    else:
        _send({"status": "error", "msg": "read failed"})

def handle_write(reader, block, data, key=None):
    if key is None:
        key = DEFAULT_KEY
    if len(data) != 16:
        _send({"status": "error", "msg": "data must be exactly 16 bytes"})
        return
    status, _ = reader.request(PICC_REQALL)
    if status != MI_OK:
        _send({"status": "no_tag"})
        return
    status, uid = reader.anticoll()
    if status != MI_OK:
        _send({"status": "error", "msg": "anticoll failed"})
        return
    reader.select_tag(uid)
    sector = _sector_of(block)
    trailer = _first_block_of_sector(sector) + 3
    status = reader.auth(PICC_AUTH_KEY_A, trailer, key, uid)
    if status != MI_OK:
        reader.stop_crypto()
        reader.halt()
        _send({"status": "error", "msg": "auth failed"})
        return
    status = reader.write_block(block, data)
    reader.stop_crypto()
    reader.halt()
    if status == MI_OK:
        _send({"status": "ok", "block": block, "uid": uid})
    else:
        _send({"status": "error", "msg": "write failed"})

def main():
    reader = RC522(
        spi_id=SPI_ID,
        sck=PIN_SCK,
        mosi=PIN_MOSI,
        miso=PIN_MISO,
        cs=PIN_CS,
        rst=PIN_RST,
    )
    _send({"Cache Money RFID Reader: \nstatus": "ready", "version": hex(reader.version)})

    buf = ""
    while True:
        # Read one character at a time from USB CDC stdin
        ch = sys.stdin.read(1)
        if not ch:
            time.sleep_ms(5)
            continue
        if ch in ("\n", "\r"):
            line = buf.strip()
            buf = ""
            if not line:
                continue
            try:
                cmd_obj = json.loads(line)
            except Exception:
                _send({"status": "error", "msg": "invalid JSON"})
                continue

            cmd = cmd_obj.get("cmd", "")
            key = cmd_obj.get("key", None)  # optional override

            if cmd == "ping":
                _send({"status": "ok", "msg": "pong"})

            elif cmd == "scan":
                handle_scan(reader)

            elif cmd == "read":
                block = cmd_obj.get("block")
                if block is None:
                    _send({"status": "error", "msg": "missing 'block'"})
                else:
                    handle_read(reader, int(block), key)

            elif cmd == "write":
                block = cmd_obj.get("block")
                data  = cmd_obj.get("data")
                if block is None or data is None:
                    _send({"status": "error", "msg": "missing 'block' or 'data'"})
                elif len(data) != 16:
                    _send({"status": "error", "msg": "data must be 16 bytes"})
                else:
                    handle_write(reader, int(block), list(data), key)

            else:
                _send({"status": "error", "msg": f"unknown command: {cmd}"})
        else:
            buf += ch

main()