
# ==========================================
# ATOM S3 + ATOM CAN Base
# Ver 7.7 - BLE遅延最終解消: VALS一括送信
# ==========================================
# 【V7.6からの変更点】
#
# 遅延の根本原因:
#   VAL〜VAL7 を7回 gatts_notify() → 複数のBLE接続イベントにまたがる
#   BLE接続インターバル(20〜100ms) × 複数回 = 数百ms遅延
#
# 解決:
#   "VALS=v1,v2,v3,v4,v5,v6,v7" の1パケットに全スロットを圧縮
#   → gatts_notify() 1回 = 1接続イベントで全スロット届く
#   → 遅延 = BLE接続インターバル1回分のみ
#
# Web app側の対応: VALS= ハンドラを handleNotifications() に追加済み
# ==========================================
import M5
from M5 import *
import time
from hardware import I2C, Pin
from machine import mem32
from unit import CANUnit
import bluetooth
import struct
from micropython import const
from esp32 import NVS

TX_PIN       = 6
RX_PIN       = 5
CAN_BAUDRATE = 1000000
CAN_GROUP_1_ID = 0x4E0
CAN_GROUP_2_ID = 0x4E1
can_tx_id    = 0x5A0
k_meter_id   = 0x661
slot_modes   = [1] * 7
can_state    = bytearray(8)

DRAIN_MAX     = 10
CAN_SEND_INT  = 100
TEMP_SEND_INT = 200
DISP_INT      = 250
VALS_SEND_INT = 20         # ★ VALS送信間隔 20ms (50Hz) CAN受信と非同期

can             = None
ble_tx_queue    = []
last_vals       = {}       # ★ 常に最新値を保持 (クリアしない)
last_vals_send  = 0
allowed_ids_set = set()
last_can_send       = 0
last_temp_send      = 0
last_display        = 0
last_recovery_check = 0
kmeter_found  = False
last_temp_val = 0
can_error     = False
filter_ok     = False
nvs           = None
_pending_config_send = False
ble_rx_queue  = []

UART_UUID = bluetooth.UUID("6e400001-b5a3-f393-e0a9-e50e24dcca9e")
UART_TX   = bluetooth.UUID("6e400003-b5a3-f393-e0a9-e50e24dcca9e")
UART_RX   = bluetooth.UUID("6e400002-b5a3-f393-e0a9-e50e24dcca9e")
_IRQ_CENTRAL_CONNECT    = const(1)
_IRQ_CENTRAL_DISCONNECT = const(2)
_IRQ_GATTS_WRITE        = const(3)

class BLEUART:
    def __init__(self, ble):
        self._ble = ble
        self._ble.active(True)
        self._ble.irq(self._irq)
        ((self._tx, self._rx),) = self._ble.gatts_register_services([
            (UART_UUID, ((UART_TX, bluetooth.FLAG_NOTIFY),
                         (UART_RX, bluetooth.FLAG_WRITE)))
        ])
        self._connections = set()
        self._advertise()

    def _advertise(self):
        try:
            self._ble.gap_advertise(100000, b'\x02\x01\x06\x12\x09M5AtomS3_CAN_Base')
        except: pass

    def _irq(self, event, data):
        global _pending_config_send
        if event == _IRQ_CENTRAL_CONNECT:
            self._connections.add(data[0])
            _pending_config_send = True
        elif event == _IRQ_CENTRAL_DISCONNECT:
            if data[0] in self._connections:
                self._connections.remove(data[0])
            self._advertise()
        elif event == _IRQ_GATTS_WRITE:
            try:
                cmd = self._ble.gatts_read(self._rx).decode().strip()
                if cmd: ble_rx_queue.append(cmd)
            except: pass

    def send(self, data):
        for conn in self._connections:
            try: self._ble.gatts_notify(conn, self._tx, data)
            except: pass

def update_hw_filter():
    global allowed_ids_set, filter_ok
    allowed_ids_set = {CAN_GROUP_1_ID, CAN_GROUP_2_ID}
    if not can: return
    id1 = CAN_GROUP_1_ID & 0x7FF
    id2 = CAN_GROUP_2_ID & 0x7FF
    diff    = (id1 ^ id2) & 0x7FF
    code_id = id1 & (~diff) & 0x7FF
    mask_id = diff
    code_b0 = (code_id >> 3) & 0xFF
    code_b1 = ((code_id & 0x07) << 5) & 0xFF
    mask_b0 = (mask_id >> 3) & 0xFF
    mask_b1 = ((mask_id & 0x07) << 5) | 0x1F
    TWAI_BASE = 0x6002B000
    try:
        mem32[TWAI_BASE + 0x000] = (mem32[TWAI_BASE + 0x000] & 0xFF) | 0x01
        mem32[TWAI_BASE + 0x000] = (mem32[TWAI_BASE + 0x000] & 0xFF) | 0x09
        mem32[TWAI_BASE + 0x040] = code_b0
        mem32[TWAI_BASE + 0x044] = code_b1
        mem32[TWAI_BASE + 0x048] = 0x00
        mem32[TWAI_BASE + 0x04C] = 0x00
        mem32[TWAI_BASE + 0x050] = mask_b0
        mem32[TWAI_BASE + 0x054] = mask_b1
        mem32[TWAI_BASE + 0x058] = 0xFF
        mem32[TWAI_BASE + 0x05C] = 0xFF
        mem32[TWAI_BASE + 0x000] = (mem32[TWAI_BASE + 0x000] & 0xFF) & 0xFE
        time.sleep_ms(10)   # ★ TWAIコントローラ安定待ち
        rb = mem32[TWAI_BASE + 0x040] & 0xFF
        filter_ok = (rb == code_b0)
        can_error = False   # ★ ResetMode中のsend失敗フラグをクリア
        print(f"[Filter] ACR0={hex(code_b0)} rb={hex(rb)} {'OK' if filter_ok else 'NG'}")
    except Exception as e:
        filter_ok = False
        print(f"[Filter] FAIL: {e}")

def check_can_recovery():
    global can, last_recovery_check, can_error
    now = time.ticks_ms()
    if time.ticks_diff(now, last_recovery_check) < 5000: return
    last_recovery_check = now
    try:
        if can and can.state() != CANUnit.RUNNING:
            can_error = True
            can.deinit()
            time.sleep_ms(50)
            can = CANUnit(0, port=(TX_PIN, RX_PIN), mode=CANUnit.NORMAL, baudrate=CAN_BAUDRATE)
            time.sleep_ms(100)
            update_hw_filter()
            can_error = False
    except: pass

def queue_config_sync():
    ble_tx_queue.append(("STATE=" + ",".join(str(b) for b in can_state)).encode())
    ble_tx_queue.append(f"GRP1={hex(CAN_GROUP_1_ID)}".encode())
    ble_tx_queue.append(f"GRP2={hex(CAN_GROUP_2_ID)}".encode())
    ble_tx_queue.append(f"ID={hex(can_tx_id)}".encode())
    ble_tx_queue.append(f"KID={hex(k_meter_id)}".encode())

def extract_val(data, pos, mode):
    if len(data) <= pos: return None
    if mode == 0: return data[pos]
    if len(data) <= pos + 1: return None
    if mode == 1: return data[pos] | (data[pos + 1] << 8)
    return (data[pos] << 8) | data[pos + 1]

def safe_can_send(id, data):
    global can_error
    if can:
        try:
            can.send(data, id, timeout=0)
            can_error = False
        except:
            can_error = True

def process_ble_cmd():
    global CAN_GROUP_1_ID, CAN_GROUP_2_ID, can_tx_id, k_meter_id
    if not ble_rx_queue: return
    cmd = ble_rx_queue.pop(0)
    try:
        if cmd == "REQUEST_STATE":
            # ★ web側から再同期要求 → 即座にSTATE=を返す
            queue_config_sync()
            return
        if cmd.startswith("SET_GROUPS="):
            p = cmd.split('=')[1].split(',')
            CAN_GROUP_1_ID = int(p[0], 16)
            CAN_GROUP_2_ID = int(p[1], 16)
            if nvs:
                nvs.set_i32("grp1_id", CAN_GROUP_1_ID)
                nvs.set_i32("grp2_id", CAN_GROUP_2_ID)
                nvs.commit()
            update_hw_filter()
            queue_config_sync()
        elif cmd.startswith("ID="):
            can_tx_id = int(cmd.split('=')[1], 16)
            if nvs: nvs.set_i32("my_id", can_tx_id); nvs.commit()
            ble_tx_queue.append(f"ID={hex(can_tx_id)}".encode())
        elif cmd.startswith("KID="):
            k_meter_id = int(cmd.split('=')[1], 16)
            if nvs: nvs.set_i32("k_meter_id", k_meter_id); nvs.commit()
            ble_tx_queue.append(f"KID={hex(k_meter_id)}".encode())
        elif '=' in cmd:
            p = cmd.split('=')
            if p[0].isdigit():
                idx = int(p[0]) - 1
                val = int(p[1])
                if 0 <= idx < 8:
                    can_state[idx] = val
                    safe_can_send(can_tx_id, can_state)
                    ble_tx_queue.append(("STATE=" + ",".join(str(b) for b in can_state)).encode())
    except: pass

def process_can_rx():
    global can_error
    if can is None: return
    try:
        for _ in range(DRAIN_MAX):
            if can.any(0) == 0: break
            msg = can.recv(0, timeout=0)
            if not msg: break
            can_id = msg[0] & 0x7FF
            if can_id not in allowed_ids_set: continue
            data = msg[4]
            if can_id == CAN_GROUP_1_ID:
                for i in range(4):
                    val = extract_val(data, i * 2, slot_modes[i])
                    if val is not None:
                        last_vals[i] = val   # ★ 上書き保持 (クリアしない)
            elif can_id == CAN_GROUP_2_ID:
                for i in range(3):
                    val = extract_val(data, i * 2, slot_modes[i + 4])
                    if val is not None:
                        last_vals[i + 4] = val
        can_error = False
    except MemoryError: pass
    except: can_error = True

# ★ 全スロットを1パケット送信 (last_vals保持・クリアしない)
def send_vals_packet(uart):
    if not last_vals: return
    parts = []
    for i in range(7):
        v = last_vals.get(i)
        parts.append("" if v is None else str(v))
    while parts and parts[-1] == "":
        parts.pop()
    if not parts: return
    msg = "VALS=" + ",".join(parts)
    try:
        uart.send(msg.encode())
    except: pass
    # ★ クリアしない → 次の20msでも同じ最新値を送り続ける

# ------------------------------
# 初期化
# ------------------------------
M5.begin()
try:
    nvs = NVS("can_app")
    CAN_GROUP_1_ID = nvs.get_i32("grp1_id")
    CAN_GROUP_2_ID = nvs.get_i32("grp2_id")
    can_tx_id      = nvs.get_i32("my_id")
    k_meter_id     = nvs.get_i32("k_meter_id")
except: pass
try:
    for i in range(7): slot_modes[i] = nvs.get_i32(f"slot{i}_mode")
except: pass

try:
    can = CANUnit(0, port=(TX_PIN, RX_PIN), mode=CANUnit.NORMAL, baudrate=CAN_BAUDRATE)
    time.sleep_ms(100)
    update_hw_filter()
    print("CAN Init OK")
except Exception as e:
    print("CAN Init Fail:", e)

try:
    i2c = I2C(0, scl=Pin(1), sda=Pin(2), freq=100000)
    kmeter_found = (0x66 in i2c.scan())
except: pass

ble  = bluetooth.BLE()
uart = BLEUART(ble)
M5.Display.setTextSize(2)
M5.Display.clear()

# ------------------------------
# メインループ
# ------------------------------
while True:
    M5.update()
    now = time.ticks_ms()

    process_ble_cmd()

    if _pending_config_send:
        _pending_config_send = False
        # ★ 接続直後300ms後にSTATE=/GRP1=等を送る (sleep_msなしでループ継続)
        ble_tx_queue.append(b"__SYNC_DELAY__")

    process_can_rx()

    if time.ticks_diff(now, last_can_send) > CAN_SEND_INT:
        safe_can_send(can_tx_id, can_state)
        last_can_send = now

    if time.ticks_diff(now, last_temp_send) > TEMP_SEND_INT:
        if kmeter_found:
            try:
                d = i2c.readfrom_mem(0x66, 0x00, 4)
                t = int(struct.unpack('<i', d)[0] / 100.0)
                last_temp_val = t
                s_data = bytearray([0x01, 1 if t >= 0 else 0,
                                    (abs(t) >> 8) & 0xFF, abs(t) & 0xFF,
                                    0, 0, 0, 0])
                safe_can_send(k_meter_id, s_data)
                ble_tx_queue.append(f"TEMP={t}".encode())
            except: pass
        last_temp_send = now

    # ★ ble_tx_queue優先 (再接続時のSTATE=/GRP1=/GRP2=等を確実に送る)
    # キューが空の時だけVALS定期送信 (20ms 50Hz)
    if ble_tx_queue:
        item = ble_tx_queue.pop(0)
        if item == b"__SYNC_DELAY__":
            # 再接続後: 少し待ってから設定同期を送る
            time.sleep_ms(300)
            queue_config_sync()
        else:
            try: uart.send(item)
            except: pass
    elif time.ticks_diff(now, last_vals_send) >= VALS_SEND_INT:
        send_vals_packet(uart)
        last_vals_send = now

    if time.ticks_diff(now, last_display) > DISP_INT:
        check_can_recovery()
        ble_tx_queue.append(b"STATUS=ERR" if can_error else b"STATUS=CAN_OK")
        M5.Display.setCursor(0, 0)
        M5.Display.setTextColor(0xFFFF, 0x0000)
        M5.Display.print("V7.7 %s" % ("ERR" if can_error else "OK "))
        M5.Display.setCursor(0, 20)
        M5.Display.setTextColor(0x07E0 if filter_ok else 0xF800, 0x0000)
        M5.Display.print("FLT:%s  " % ("HW" if filter_ok else "SW"))
        M5.Display.setCursor(0, 40)
        M5.Display.setTextColor(0xFFFF, 0x0000)
        M5.Display.print("T:%d Q:%d  " % (last_temp_val, len(ble_tx_queue)))
        last_display = now

    time.sleep_ms(1)
