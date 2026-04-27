import time
from machine import Pin, SPI

SPI_ID=0; PIN_SCK=18; PIN_MOSI=19; PIN_MISO=16; PIN_CS=17; PIN_RST=21

cs  = Pin(PIN_CS,  Pin.OUT)
rst = Pin(PIN_RST, Pin.OUT)
spi = SPI(SPI_ID, baudrate=1_000_000, polarity=0, phase=0,
          sck=Pin(PIN_SCK), mosi=Pin(PIN_MOSI), miso=Pin(PIN_MISO))
cs.value(1)

def w(reg, val):
    cs.value(0); spi.write(bytes([(reg<<1)&0x7E, val])); cs.value(1)

def r(reg):
    cs.value(0); spi.write(bytes([((reg<<1)&0x7E)|0x80]))
    buf=bytearray(1); spi.readinto(buf); cs.value(1); return buf[0]

def sb(reg,mask): w(reg, r(reg)|mask)

# hard reset
rst.value(0); time.sleep_us(50); rst.value(1); time.sleep_ms(50)
print("version        :", hex(r(0x37)))

# --- Test 1: can we write/read a plain register? ---
w(0x0B, 0x05)   # WaterLevelReg = 5
print("WaterLevel w/r :", hex(r(0x0B)), "(expect 0x5)")

# --- Test 2: soft reset ---
w(0x01, 0x0F)   # CMD_SOFT_RESET
time.sleep_ms(50)
print("CMD after sreset:", hex(r(0x01)), "(expect 0x20 = RcvOff bit)")

# --- re-init after soft reset ---
w(0x2A, 0x8D); w(0x2B, 0x3E); w(0x2C, 0x00); w(0x2D, 0x1E)
w(0x15, 0x40); w(0x11, 0x3D); sb(0x14, 0x03)

# --- Test 3: write CMD_TRANSCEIVE, read back 5 times immediately ---
w(0x01, 0x07)
vals = [hex(r(0x01)) for _ in range(5)]
print("CMD reg x5 reads:", vals, "(expect all 0x7)")

# --- Test 4: attempt transceive with StartSend written atomically ---
print("\n--- full transceive attempt (hold card now) ---")
sb(0x0A, 0x80)          # flush FIFO
w(0x04, 0x7F)           # clear IRQ
w(0x09, 0x26)           # REQA into FIFO
w(0x01, 0x07)           # CMD_TRANSCEIVE
w(0x0D, 0x87)           # BIT_FRAMING: 7-bit + StartSend in ONE write (no sb read-modify-write)

time.sleep_ms(25)
print("IRQ after 25ms :", hex(r(0x04)))
print("FIFO_LVL       :", r(0x0A), "(0 = data was sent)")
print("ERROR          :", hex(r(0x06)))
print("CMD reg        :", hex(r(0x01)))
