# ==========================================
# ATOM S3 + ATOM CAN Base
# Ver 5.9.12 - Hardware TWAI Filter Activation
# Ver 5.9.11 - Native hardware.CAN tests
# 
# [修正内容]
# M5Stackの `CANUnit` が ESP32(TWAI) の仕様に合わせて
# 「一致が0、無視が1」という逆転マスクを要求していることに
# 対応。ハードウェアレベルで 0x4E0, 0x4E1 以外を完全に
# 遮断する真のTWAI専用フィルタを設定しました。
# これにより、CPUまでデータが来ないためシリアルスパムも
# 完全に消失し、フリーズが根絶されます。
# ==========================================
import M5
from M5 import *
import time
from hardware import I2C, Pin
from unit import CANUnit
import bluetooth
import struct
from micropython import const
from esp32 import NVS
# ------------------------------
# 基本設定
# ------------------------------
TX_PIN = 6
RX_PIN = 5
CAN_GROUP_1_ID = 0x4E0
CAN_GROUP_2_ID = 0x4E1
can_tx_id  = 0x5A0
k_meter_id = 0x661
slot_modes = [1] * 7
FIFO = 0
DISPLAY_INTERVAL     = 250
SLOT_UPDATE_INTERVAL = 50
# ------------------------------
# タイマー設定
# ------------------------------
CAN_SEND_INTERVAL    = 100
TEMP_SEND_INTERVAL   = 200
last_can_send_time   = 0
last_temp_send_time  = 0
# ------------------------------
# グローバル変数
# ------------------------------
can             = None
detected_rate   = 1000000
allowed_ids_set = set()
monitor_tx_vals = [None] * 7
ble_tx_queue    = []
ble_rx_queue    = []
can_error       = False
last_display    = 0
last_slot_update    = 0
last_recovery_check = 0
_pending_config_send = False
DEVICE_NAME = "M5AtomS3_CAN_Base"
nvs = None
can_state = bytearray(8)
i2c0 = None
kmeter_addr = 0x66
kmeter_found = False
last_temp_disp = "---"
# ------------------------------
# BLE UART
# ------------------------------
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
        self._rx_cb = None
        self._advertise()
    def _advertise(self):
        name = DEVICE_NAME.encode()
        payload = b'\x02\x01\x06' + bytes([len(name)+1, 0x09]) + name
        try: self._ble.gap_advertise(100000, payload)
        except: pass
    def _irq(self, event, data):
        global _pending_config_send
        if event == _IRQ_CENTRAL_CONNECT:
            conn, _, _ = data
            self._connections.add(conn)
            _pending_config_send = True
        elif event == _IRQ_CENTRAL_DISCONNECT:
            conn, _, _ = data
            if conn in self._connections:
                self._connections.remove(conn)
            self._advertise()
        elif event == _IRQ_GATTS_WRITE:
            if self._rx_cb:
                self._rx_cb(self._ble.gatts_read(self._rx))
    def send(self, data):
        for conn in self._connections:
            try: self._ble.gatts_notify(conn, self._tx, data)
            except: pass
    def on_rx(self, cb):
        self._rx_cb = cb
# ------------------------------
# BLE 受信ハンドラ
# ------------------------------
def on_ble_rx_irq(data):
    try:
        cmd = data.decode().strip()
        if cmd:
            ble_rx_queue.append(cmd)
    except:
        pass
# ------------------------------
# 設定同期キュー
# ------------------------------
def queue_config_sync():
    msg = "STATE=" + ",".join(str(b) for b in can_state)
    ble_tx_queue.append(msg.encode())
    ble_tx_queue.append(f"GRP1={hex(CAN_GROUP_1_ID)}".encode())
    ble_tx_queue.append(f"GRP2={hex(CAN_GROUP_2_ID)}".encode())
    ble_tx_queue.append(f"ID={hex(can_tx_id)}".encode())
    ble_tx_queue.append(f"KID={hex(k_meter_id)}".encode())
# ------------------------------
# コマンド処理
# ------------------------------
def process_ble_command():
    global CAN_GROUP_1_ID, CAN_GROUP_2_ID, can_tx_id, k_meter_id, slot_modes
    if not ble_rx_queue: return
    cmd = ble_rx_queue.pop(0)
    print(f"CMD: {cmd}")
    try:
        if cmd.startswith("SET_GROUPS="):
            parts = cmd.split('=')[1].split(',')
            if len(parts) == 2:
                new_g1 = int(parts[0], 16)
                new_g2 = int(parts[1], 16)
                CAN_GROUP_1_ID = new_g1
                CAN_GROUP_2_ID = new_g2
                if nvs:
                    nvs.set_i32("grp1_id", new_g1)
                    nvs.set_i32("grp2_id", new_g2)
                    nvs.commit()
                update_hw_filter()
                ble_tx_queue.append(f"GRP1={hex(CAN_GROUP_1_ID)}".encode())
                ble_tx_queue.append(f"GRP2={hex(CAN_GROUP_2_ID)}".encode())
        elif cmd.startswith("CFGBTN="):
            parts = cmd.split('=')[1].split(',')
            if len(parts) == 3:
                idx = int(parts[0])
                on_v = int(parts[1])
                off_v = int(parts[2])
                if 0 <= idx < 8 and nvs:
                    nvs.set_i32(f"btn{idx+1}_on", on_v)
                    nvs.set_i32(f"btn{idx+1}_off", off_v)
                    nvs.commit()
        elif cmd.startswith("SET_SLOT_MODE="):
            parts = cmd.split('=')[1].split(',')
            if len(parts) == 2:
                slot_i = int(parts[0])
                mode   = int(parts[1])
                if 0 <= slot_i < 7:
                    slot_modes[slot_i] = mode
                    if nvs:
                        nvs.set_i32(f"slot{slot_i}_mode", mode)
                        nvs.commit()
                    ble_tx_queue.append(f"SLOT_MODE_OK={slot_i},{mode}".encode())
        
        elif cmd.startswith("ID="):
             val = int(cmd.split('=')[1], 16)
             can_tx_id = val
             if nvs:
                 nvs.set_i32("my_id", val)
                 nvs.commit()
             ble_tx_queue.append(f"ID={hex(val)}".encode())
        elif cmd.startswith("KID="):
             val = int(cmd.split('=')[1], 16)
             k_meter_id = val
             if nvs:
                 nvs.set_i32("k_meter_id", val)
                 nvs.commit()
             ble_tx_queue.append(f"KID={hex(val)}".encode())
                 
        elif '=' in cmd:
            parts = cmd.split('=')
            if len(parts) == 2 and parts[0].isdigit():
                idx = int(parts[0]) - 1
                val = int(parts[1])
                if 0 <= idx < 8:
                    can_state[idx] = val & 0xFF
                    safe_can_send(can_tx_id, can_state)
                    msg = "STATE=" + ",".join(str(b) for b in can_state)
                    ble_tx_queue.append(msg.encode())
    except Exception as e:
        print("CMD Proc Err:", e)
# ------------------------------
# HW Filter (TWAI 仕様)
# ------------------------------
def update_hw_filter():
    global allowed_ids_set
    allowed_ids_set = {CAN_GROUP_1_ID, CAN_GROUP_2_ID}
    
    # 0x4E0 と 0x4E1 のみを通すフィルタ設定
    # TWAI Standard ID は 32ビットレジスタの 上位11ビット (21ビットシフト)
    # Mask仕様: 0=一致必須, 1=無視(Don't Care)
    
    # Base ID
    base_id = CAN_GROUP_1_ID
    code_32 = base_id << 21
    
    # 差分を計算して、違うビット(0x4E0と0x4E1なら一番下の1ビット)だけ無視(1)にする
    diff = CAN_GROUP_1_ID ^ CAN_GROUP_2_ID
    mask_11bit = diff & 0x7FF
    
    # 下位21ビット(データ本体等)はすべて無視(1)にする
    mask_32 = (mask_11bit << 21) | 0x1FFFFF
    
    if can:
        try:
            can.setfilter(0, CANUnit.MASK32, 0, [code_32, mask_32])
            print(f"TWAI Filter Applied: Code={hex(code_32)}, Mask={hex(mask_32)}")
        except Exception as e:
            print("Filter Err:", e)
# ------------------------------
# CAN 送信
# ------------------------------
def safe_can_send(id, data):
    if can:
        try:
            can.send(data, id, timeout=0)
        except Exception as e:
            pass
# ------------------------------
# 値抽出
# ------------------------------
def extract_val(data, pos, mode):
    if mode == 0:
        if pos < len(data): return data[pos]
        return None
    else:
        if pos + 1 < len(data):
            if mode == 1: return data[pos] | (data[pos + 1] << 8)
            else: return (data[pos] << 8) | data[pos + 1]
        return None
# ------------------------------
# 受信処理
# ------------------------------
def process_can_rx():
    global can_error
    
    try:
        # ハードウェアフィルタが完璧に弾くので、while Trueで全取得してOK
        while True:
            if can.any(FIFO) == 0:
                break
            
            # timeout=0 で即座にリターン
            msg = can.recv(FIFO, timeout=0)
            if not msg:
                break
                
            raw_id = msg[0]
            can_id = raw_id & 0x7FF
            if can_id != CAN_GROUP_1_ID and can_id != CAN_GROUP_2_ID:
                continue
            data = msg[4]
            if can_id == CAN_GROUP_1_ID:
                for i in range(4):
                    val = extract_val(data, i * 2, slot_modes[i])
                    if val is not None: monitor_tx_vals[i] = val
            elif can_id == CAN_GROUP_2_ID:
                for i in range(3):
                    val = extract_val(data, i * 2, slot_modes[i + 4])
                    if val is not None: monitor_tx_vals[i + 4] = val
                    
        can_error = False
    except:
        can_error = True
# ------------------------------
# BLE送信データ作成
# ------------------------------
def queue_ble_data():
    global last_slot_update
    now = time.ticks_ms()
    if time.ticks_diff(now, last_slot_update) < SLOT_UPDATE_INTERVAL: return
    for i in range(7):
        val = monitor_tx_vals[i]
        if val is not None:
            prefix = "VAL" if i == 0 else f"VAL{i+1}"
            if len(ble_tx_queue) < 14:
                ble_tx_queue.append(f"{prefix}={val}".encode())
            monitor_tx_vals[i] = None
    last_slot_update = now
# ------------------------------
# BUS OFF 復帰
# ------------------------------
def check_can_recovery():
    global can, last_recovery_check
    now = time.ticks_ms()
    if time.ticks_diff(now, last_recovery_check) < 5000: return
    last_recovery_check = now
    try:
        if can and can.state() != CANUnit.RUNNING:
            can.deinit()
            time.sleep_ms(100)
            can = CANUnit(id=0, port=(TX_PIN, RX_PIN), mode=CANUnit.NORMAL, baudrate=detected_rate)
            update_hw_filter()
    except: pass
# ------------------------------
# 初期化
# ------------------------------
M5.begin()
nvs = NVS("can_app")
try: CAN_GROUP_1_ID = nvs.get_i32("grp1_id")
except: pass
try: CAN_GROUP_2_ID = nvs.get_i32("grp2_id")
except: pass
try: can_tx_id = nvs.get_i32("my_id")
except: pass
try: k_meter_id = nvs.get_i32("k_meter_id")
except: pass
try:
    for i in range(7): slot_modes[i] = nvs.get_i32(f"slot{i}_mode")
except: pass
# CAN Init
try:
    can = CANUnit(id=0, port=(TX_PIN, RX_PIN), mode=CANUnit.NORMAL, baudrate=detected_rate)
    update_hw_filter()
except Exception as e:
    print("CAN Init ERR:", e)
# I2C (K-meter) Init
try:
    i2c0 = I2C(0, scl=Pin(1), sda=Pin(2), freq=100000)
    time.sleep(0.5)
    if kmeter_addr in i2c0.scan():
        kmeter_found = True
        print("K-meter Found")
except: pass
ble  = bluetooth.BLE()
uart = BLEUART(ble)
uart.on_rx(on_ble_rx_irq)
M5.Display.setTextSize(2)
M5.Display.clear()
# ------------------------------
# メインループ
# ------------------------------
while True:
    M5.update()
    now = time.ticks_ms()
    process_ble_command()
    if _pending_config_send:
        _pending_config_send = False
        time.sleep_ms(1000)
        queue_config_sync()
    process_can_rx()
    
    if time.ticks_diff(now, last_can_send_time) > CAN_SEND_INTERVAL:
        safe_can_send(can_tx_id, can_state)
        last_can_send_time = now
    if time.ticks_diff(now, last_temp_send_time) > TEMP_SEND_INTERVAL:
        if kmeter_found:
            try:
                data = i2c0.readfrom_mem(kmeter_addr, 0x00, 4)
                val_int = struct.unpack('<i', data)[0]
                temp_val = val_int / 100.0
                t_int = int(temp_val)
                s_data = bytearray([0x01, 1 if temp_val>=0 else 0, (abs(t_int)>>8)&0xFF, abs(t_int)&0xFF, 0,0,0,0])
                safe_can_send(k_meter_id, s_data)
                last_temp_disp = str(t_int)
                if len(ble_tx_queue) < 14:
                    ble_tx_queue.append(f"TEMP={t_int}".encode())
            except: pass
        last_temp_send_time = now
    queue_ble_data()
    check_can_recovery()
    if ble_tx_queue:
        try: uart.send(ble_tx_queue.pop(0))
        except: pass
    if time.ticks_diff(now, last_display) > DISPLAY_INTERVAL:
        M5.Display.setCursor(0, 0)
        M5.Display.setTextColor(0xFFFF, 0x0000) # White
        M5.Display.print("V5.9.12")
        M5.Display.setTextColor(0xF800 if can_error else 0x07FF, 0x0000)
        M5.Display.print("CAN:%s" % ("ERR" if can_error else "OK "))
        
        M5.Display.setCursor(0, 20)
        M5.Display.setTextColor(0x07FF, 0x0000) # Cyan
        M5.Display.print("Q:%d    " % len(ble_rx_queue))
        M5.Display.setCursor(0, 40)
        M5.Display.setTextColor(0xFFE0, 0x0000) # Yellow
        M5.Display.print("TMP:%s   " % last_temp_disp)
        
        last_display = now
    time.sleep_ms(1)
