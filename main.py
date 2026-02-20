
import M5
from M5 import *
import time
from unit import CANUnit, KMeterUnit
from hardware import I2C, Pin
import bluetooth
import struct
from micropython import const
from esp32 import NVS

# ==========================================
# ATOM S3 + ATOM CAN Base (Full BLE Ver)
# Ver 3.8: Multi-Monitor Support (7 Slots)
# ==========================================

# --- Configuration for ATOM CAN Base ---
TX_PIN = 6
RX_PIN = 5
# ---------------------------------------

can_tx_id = 0x100
k_meter_id = 0x200

# --- Generic Monitor Config (Array for 7 Slots) ---
# Each slot: [id, idx, mode]
# Slot 0 is the "Main" monitor (Backward compatible)
rx_monitors = [
    [0x90, 2, 1], # Slot 0
    [0x90, 2, 1], # Slot 1
    [0x90, 2, 1], # Slot 2
    [0x90, 2, 1], # Slot 3
    [0x90, 2, 1], # Slot 4
    [0x90, 2, 1], # Slot 5
    [0x90, 2, 1]  # Slot 6
]

# --- Deferred Config Request (Tuples) ---
rx_config_req = None  # (slot, id, idx, mode)

nvs = None

def init_nvs():
    global nvs, can_tx_id, k_meter_id, rx_monitors
    try:
        nvs = NVS("can_app")
        print("NVS Init OK (can_app)")
    except:
        try:
            nvs = NVS("m5_config") # Fallback namespace
            print("NVS Init OK (m5_config)")
        except:
            nvs = None
            print("NVS Init Failed")
            
    print(">>> FIRMWARE UPDATED: VER 3.8 (MULTI-MONITOR) <<<")
    
    if nvs:
        try:
            can_tx_id = nvs.get_i32("my_id")
            print("Loaded ID:", hex(can_tx_id))
        except OSError: pass

        try:
            k_meter_id = nvs.get_i32("k_meter_id")
            print("Loaded KID:", hex(k_meter_id))
        except OSError: pass
        
        # --- Load Generic Monitor Configs ---
        # Slot 0: Use legacy keys (rx_id, rx_idx, rx_m) for backward compatibility
        try:
            rx_monitors[0][0] = nvs.get_i32("rx_id")
            rx_monitors[0][1] = nvs.get_i32("rx_idx")
            rx_monitors[0][2] = nvs.get_i32("rx_m")
            print("Loaded Slot 0 (Legacy):", hex(rx_monitors[0][0]))
        except OSError: pass

        # Slot 1-6: Use new keys (rxN_id, rxN_idx, rxN_m)
        for i in range(1, 7):
            try:
                rx_monitors[i][0] = nvs.get_i32(f"rx{i}_id")
                rx_monitors[i][1] = nvs.get_i32(f"rx{i}_idx")
                rx_monitors[i][2] = nvs.get_i32(f"rx{i}_m")
                # print(f"Loaded Slot {i}:", hex(rx_monitors[i][0]))
            except OSError: pass

def load_btn_nvs():
    if not nvs: return
    try:
        updated = False
        for i in range(8):
            key_off = f"btn{i+1}_off"
            try:
                # Load OFF value and set as initial state
                off_val = nvs.get_i32(key_off)
                can_state[i] = off_val & 0xFF
                updated = True
            except OSError:
                pass # Key not found, keep default 0
        if updated:
            print("Loaded NVS Btn Config")
    except Exception as e:
        print("NVS Load Err:", e)

DEVICE_NAME = "M5AtomS3_CAN_Base"
FIFO = 0
can_state = bytearray(8)
can_error = False

# --- Global Display Variables ---
last_rx_id = 0
last_rx_monitor_val = "---" # Initial Monitor Value (Slot 0)
last_temp_disp = "0"
rx_count = 0  

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
            (UART_UUID, (
                (UART_TX, bluetooth.FLAG_NOTIFY),
                (UART_RX, bluetooth.FLAG_WRITE),
            )),
        ])
        self._connections = set()
        self._rx_cb = None
        self._adv_payload = self._get_adv_payload(DEVICE_NAME)
        self._advertise()

    def _get_adv_payload(self, name):
        name_bytes = name.encode()
        return b'\x02\x01\x06' + bytes([len(name_bytes)+1, 0x09]) + name_bytes

    def _advertise(self):
        try: self._ble.gap_advertise(100_000, self._adv_payload)
        except: pass

    def _irq(self, event, data):
        if event == _IRQ_CENTRAL_CONNECT:
            conn_handle, _, _ = data
            self._connections.add(conn_handle)
            time.sleep(0.5)
            self.send_state_sync()
            time.sleep(0.1)
            self.send_config()
        elif event == _IRQ_CENTRAL_DISCONNECT:
            conn_handle, _, _ = data
            if conn_handle in self._connections:
                self._connections.remove(conn_handle)
            self._advertise()
        elif event == _IRQ_GATTS_WRITE:
            if self._rx_cb: self._rx_cb(self._ble.gatts_read(self._rx))

    def send(self, data):
        for conn_handle in self._connections:
            try: self._ble.gatts_notify(conn_handle, self._tx, data)
            except: pass

    def send_state_sync(self):
        self.send(("STATE=" + ",".join(str(b) for b in can_state)).encode())
        
    def send_status(self, is_err):
        self.send((f"STATUS={'CAN_ERR' if is_err else 'CAN_OK'}").encode())
        
    def send_config(self):
        self.send((f"ID={hex(can_tx_id)}").encode())
        time.sleep(0.05)
        self.send((f"KID={hex(k_meter_id)}").encode())
        time.sleep(0.05)
        # Send initial RX Monitor Config (Slot 0 only for confirm)
        self.send((f"RXID={hex(rx_monitors[0][0])}").encode())

    def on_rx(self, cb):
        self._rx_cb = cb

def detect_baudrate():
    M5.Display.clear()
    M5.Display.setCursor(0, 0)
    M5.Display.setTextSize(2)
    M5.Display.setTextColor(0xFFFF, 0x0000)
    M5.Display.print("Detecting\nBaudrate...")
    print("--- Auto Baudrate Detection ---")
    
    try:
        # Loop attempts
        for attempt in range(1, 4): 
            print(f"Attempt {attempt}...")
            for rate in [1000000, 500000]:
                print(f"  Trying {rate}...")
                can = None
                try:
                    can = CANUnit(id=0, port=(TX_PIN, RX_PIN), mode=CANUnit.NORMAL, baudrate=rate)
                    
                    # Listen for traffic (1.5 sec)
                    start = time.ticks_ms()
                    detected = False
                    
                    while time.ticks_diff(time.ticks_ms(), start) < 1500:
                        try:
                            # Check RX FIFO
                            if can.any(0) > 0: 
                                print(f"  >> TRAFFIC DETECTED at {rate}!")
                                detected = True
                                break
                        except: pass
                        time.sleep_ms(50)
                        
                    if hasattr(can, 'deinit'): can.deinit()
                    
                    if detected:
                        print(f"LOCKED Baudrate: {rate}")
                        return rate
                        
                except Exception as e:
                    print(f"  Error at {rate}: {e}")
                    if can and hasattr(can, 'deinit'): can.deinit()
                
                time.sleep_ms(100)
                
    except KeyboardInterrupt:
        print("Baudrate Detect Interrupt")
        return 1000000
        
    print("Detection Failed, Defaulting to 1000000")
    return 1000000 

def on_ble_rx(data):
    # ★ FIX: rx_monitors used instead of single vars
    global can_state, can_tx_id, k_meter_id, rx_monitors, rx_config_req
    
    try:
        # Clean Logs (Removed verbose Hex/Parts)
        cmd = data.decode().strip()
        print("RX:", cmd) 
        
        updated = False
        
        # --- FIXED LOGIC ORDER (Ver 3.16/3.17) ---
        if cmd == "GETCONFIG":
            uart.send_config()
            
        elif cmd.startswith("CFGBTN="):
            # Format: CFGBTN=idx,on_val,off_val (Decimal)
            try:
                parts = cmd.split('=')[1].split(',')
                if len(parts) == 3:
                    idx = int(parts[0])
                    on_v = int(parts[1])
                    off_v = int(parts[2])
                    
                    if 0 <= idx < 8:
                        # 1. Priority: Update RAM immediately
                        # If current state matches ON value, maintain it (don't reset to OFF)
                        if can_state[idx] == (on_v & 0xFF):
                             pass 
                        else:
                             # Otherwise, apply the OFF value (default/updated)
                             new_val = off_v & 0xFF
                             if can_state[idx] != new_val:
                                 can_state[idx] = new_val
                                 updated = True
                          
                        # 2. Try Save to NVS
                        if nvs:
                            try:
                                nvs.set_i32(f"btn{idx+1}_on", on_v)
                                nvs.set_i32(f"btn{idx+1}_off", off_v)
                                nvs.commit()
                            except: pass
            except: pass
        
        elif '=' in cmd: 
            parts = cmd.split('=')
            if parts[0] == "ID":
                try:
                    new_id = int(parts[1], 16)
                    can_tx_id = new_id
                    if nvs: nvs.set_i32("my_id", new_id); nvs.commit()
                except: pass
            elif parts[0] == "KID":
                try:
                    new_kid = int(parts[1], 16)
                    k_meter_id = new_kid
                    if nvs: nvs.set_i32("k_meter_id", new_kid); nvs.commit()
                except: pass
                
            # --- PATCH APPLIED HERE: Multi-Slot Support ---
            elif parts[0] == "SET_RX": # Handles "SET_RX=slot,id,idx,mode"
                try:
                    rx_parts = parts[1].split(',')
                    # Check for 4 parameters (New format)
                    if len(rx_parts) >= 4:
                        new_rx_slot = int(rx_parts[0])
                        new_rx_id   = int(rx_parts[1])
                        new_rx_idx  = int(rx_parts[2])
                        new_rx_mode = int(rx_parts[3])
                        
                        if 0 <= new_rx_slot < 7:
                            # Update Global Array
                            rx_monitors[new_rx_slot][0] = new_rx_id
                            rx_monitors[new_rx_slot][1] = new_rx_idx
                            rx_monitors[new_rx_slot][2] = new_rx_mode
                            
                            # Trigger NVS Save in main loop
                            rx_config_req = (new_rx_slot, new_rx_id, new_rx_idx, new_rx_mode)
                            
                            # JSへ確認応答 (Slot 0の場合のみRXID=で返す、他は無視でもOK)
                            if new_rx_slot == 0:
                                try: uart.send(f"RXID={hex(new_rx_id)}".encode())
                                except: pass
                            
                            print(f"Slot {new_rx_slot} Updated: {hex(new_rx_id)}")
                    
                    # Backward Compatibility (3 parameters = Slot 0)
                    elif len(rx_parts) == 3:
                        new_rx_id   = int(rx_parts[0])
                        new_rx_idx  = int(rx_parts[1])
                        new_rx_mode = int(rx_parts[2])
                        rx_monitors[0][0] = new_rx_id
                        rx_monitors[0][1] = new_rx_idx
                        rx_monitors[0][2] = new_rx_mode
                        rx_config_req = (0, new_rx_id, new_rx_idx, new_rx_mode)
                        try: uart.send(f"RXID={hex(new_rx_id)}".encode())
                        except: pass
                        
                except Exception as e:
                    print("SET_RX Err:", e)
            # --------------------------

            elif len(parts) == 2 and parts[0].isdigit() and parts[1].isdigit():
                # Direct idx=val format
                idx = int(parts[0]) - 1
                val = int(parts[1])
                if 0 <= idx < 8:
                    if can_state[idx] != val:
                        can_state[idx] = val & 0xFF
                        updated = True
                        
        elif cmd.isdigit(): 
            idx = int(cmd) - 1
            if 0 <= idx < 8:
                can_state[idx] ^= 1
                updated = True
            
        if updated:
            can.send(id=can_tx_id, data=can_state)
            uart.send_state_sync()
            
    except Exception as e:
        print("BLE RX Err:", e)

# ---------------------------------------------
# Main Setup
# ---------------------------------------------
M5.begin()
init_nvs()
load_btn_nvs() # Load Initial Values

detected_rate = detect_baudrate()

try:
    can = CANUnit(id=0, port=(TX_PIN, RX_PIN), mode=CANUnit.NORMAL, baudrate=detected_rate)
    try:
        if hasattr(can, 'setfilter'):
            can.setfilter(0, CANUnit.FILTER_RAW_SINGLE, [0, 0])
        elif hasattr(can, 'set_filter'):
            can.set_filter(0, 0, 0)
    except:
        pass
    
    # --- Startup Broadcast ---
    print("Startup Broadcast")
    can.send(id=can_tx_id, data=can_state)
except Exception as e:
    print("CAN Init Err:", e)

ble = bluetooth.BLE()
uart = BLEUART(ble)
uart.on_rx(on_ble_rx)

# --- K-Meter Init ---
kmeter_addr = 0x66
kmeter_found = False

try:
    i2c0 = I2C(0, scl=Pin(1), sda=Pin(2), freq=100000)
    time.sleep(0.5) 
    devs = i2c0.scan()
    if kmeter_addr in devs:
        kmeter_found = True
except Exception as e:
    pass

M5.Display.clear()
time.sleep(0.5)

last_display_update = 0
DISPLAY_INTERVAL = 200

def update_lcd():
    M5.Display.setTextSize(2)
    
    # 1. CAN Status
    M5.Display.setCursor(0, 0)
    if can_error:
        M5.Display.setTextColor(0xF800, 0x0000)
        M5.Display.print("CAN: ERR  ")
    else:
        M5.Display.setTextColor(0x07FF, 0x0000) 
        M5.Display.print("CAN: OK   ")
        
    # 2. BLE Status
    M5.Display.setCursor(0, 20)
    if len(uart._connections) > 0:
        M5.Display.setTextColor(0xFFE0, 0x0000)
        M5.Display.print("BLE: ON   ")
    else:
        M5.Display.setTextColor(0x07FF, 0x0000)
        M5.Display.print("BLE: --   ")
        
    # 3. PAD (Was TX ID)
    M5.Display.setCursor(0, 40)
    M5.Display.setTextColor(0xFFFF, 0x0000)
    # Changed from "TX ID" to "PAD"
    M5.Display.print(f"PAD:{can_tx_id:X}     ")
    
    # 4. KID (Temp ID)
    M5.Display.setCursor(0, 60)
    M5.Display.setTextColor(0xFFFF, 0x0000) 
    M5.Display.print(f"KID:{k_meter_id:X}     ")

    # 5. Temp
    M5.Display.setCursor(0, 80)
    M5.Display.setTextColor(0x07E0, 0x0000)
    if not kmeter_found:
        if last_temp_disp != "0" and last_temp_disp != "ERR":
             M5.Display.print(f"Temp:{last_temp_disp}      ")
        else:
             M5.Display.print("Temp: NO-I2C    ")
    else:
        M5.Display.print(f"Temp:{last_temp_disp}C    ")

    # 6. Baudrate
    M5.Display.setCursor(0, 100)
    if detected_rate >= 1000000:
        M5.Display.setTextColor(0x07FF, 0x0000) 
        M5.Display.print("1M        ")
    else:
        M5.Display.setTextColor(0xFFE0, 0x0000) 
        M5.Display.print("500k      ")

    # 7. Generic Monitor (Show Slot 0 only for simplicity)
    M5.Display.setCursor(0, 120)
    M5.Display.setTextColor(0xFFE0, 0x0000)
    # Use Slot 0 config for display
    M5.Display.print(f"RX:{rx_monitors[0][0]:X} i:{rx_monitors[0][1]} m:{rx_monitors[0][2]}   ")
    M5.Display.setCursor(0, 140)
    M5.Display.print(f"Val:{last_rx_monitor_val}       ")

last_can_send_time = 0
CAN_SEND_INTERVAL = 200  
last_ble_sync_time = 0
BLE_SYNC_INTERVAL = 1000 
last_temp_send_time = 100 
TEMP_SEND_INTERVAL = 200 

while True:
    M5.update()
    now = time.ticks_ms()
    
    if BtnA.wasPressed():
        try:
            test_data = bytearray([0xA1, 0xB2, 0xC3, 0xD4, 0xE5, 0xF6, 0x07, 0x08])
            can.send(id=0x200, data=test_data)
        except:
            pass

    # --- Temp/TX Loop ---
    if time.ticks_diff(now, last_temp_send_time) > TEMP_SEND_INTERVAL:
        if kmeter_found:
            try:
                data = i2c0.readfrom_mem(kmeter_addr, 0x00, 4)
                val_int = struct.unpack('<i', data)[0] 
                temp_val = val_int / 100.0
                t_int = int(temp_val)
                sign = 1 if temp_val >= 0 else 0
                abs_val = abs(t_int)
                sensor_data = bytearray([0x01, sign, (abs_val >> 8) & 0xFF, abs_val & 0xFF, 0,0,0,0])
                
                last_temp_disp = str(t_int)
                
                try: can.send(id=k_meter_id, data=sensor_data)
                except: pass
                
                try: uart.send(f"TEMP={t_int}".encode())
                except: pass
            except: pass
        last_temp_send_time = now

    if time.ticks_diff(now, last_can_send_time) > CAN_SEND_INTERVAL:
        try:
            can.send(id=can_tx_id, data=can_state)
            can_error = False
        except:
            can_error = True
        last_can_send_time = now

    if time.ticks_diff(now, last_ble_sync_time) > BLE_SYNC_INTERVAL:
        uart.send_state_sync()
        uart.send_status(can_error)
        last_ble_sync_time = now

    # --- Process Deferred RX Config (NVS Save) ---
    if rx_config_req:
        try:
            (req_slot, req_id, req_idx, req_mode) = rx_config_req
            # Global vars are already updated in on_ble_rx
            rx_config_req = None # Clear request
            
            if nvs:
                try:
                    if req_slot == 0:
                        nvs.set_i32("rx_id", req_id)
                        nvs.set_i32("rx_idx", req_idx)
                        nvs.set_i32("rx_m", req_mode)
                    else:
                        nvs.set_i32(f"rx{req_slot}_id", req_id)
                        nvs.set_i32(f"rx{req_slot}_idx", req_idx)
                        nvs.set_i32(f"rx{req_slot}_m", req_mode)
                        
                    nvs.commit()
                    print(f"NVS Saved (Slot {req_slot})")
                except: pass
        except Exception as e:
            print("Config Apply Err:", e)
            rx_config_req = None

    # --- RX Loop (Multi-Monitor) ---
    if can and can.any(FIFO):
        try:
            msg = can.recv(FIFO)
            if msg:
                can_id, is_ext, rtr, dlc, rx_data = msg
                last_rx_id = can_id
                rx_count += 1
                
                # --- Check for K-Meter ID Match ---
                is_kmeter_msg = (can_id == k_meter_id) or (kmeter_found and can_id == 0x200)
                
                if is_kmeter_msg and dlc >= 4: 
                    t_high = rx_data[2]
                    t_low = rx_data[3]
                    val = (t_high << 8) | t_low
                    if rx_data[1] == 0: val = -val
                    last_temp_disp = str(val)
                
                # --- Generic Monitor Logic (Iterate all 7 slots) ---
                for slot_i in range(7):
                    cfg_id = rx_monitors[slot_i][0]
                    cfg_idx = rx_monitors[slot_i][1]
                    cfg_mode = rx_monitors[slot_i][2]
                    
                    if can_id == cfg_id:
                        try:
                            val = 0
                            # Mode 0: 1Byte
                            if cfg_mode == 0:
                                if cfg_idx < dlc:
                                    val = rx_data[cfg_idx]
                            # Mode 1: 2Byte LE
                            elif cfg_mode == 1:
                                if cfg_idx + 1 < dlc:
                                    val = rx_data[cfg_idx] | (rx_data[cfg_idx+1] << 8)
                            # Mode 2: 2Byte BE
                            elif cfg_mode == 2:
                                if cfg_idx + 1 < dlc:
                                    val = (rx_data[cfg_idx] << 8) | rx_data[cfg_idx+1]
                            
                            # Update display cache only for Slot 0
                            if slot_i == 0:
                                last_rx_monitor_val = str(val)
                            
                            # Send to BLE
                            # Slot 0 -> "VAL=..."
                            # Slot 1-6 -> "VAL2=...", "VAL3=..." etc.
                            try: 
                                prefix = "VAL" if slot_i == 0 else f"VAL{slot_i+1}"
                                uart.send(f"{prefix}={val}".encode())
                            except: pass
                            
                        except Exception as e:
                            pass
                        # Note: Do not 'break' here, as multiple slots might monitor the same ID (e.g. Byte 0 and Byte 2)
      
        except:
            pass

    # --- LCD Update ---
    if time.ticks_diff(now, last_display_update) > DISPLAY_INTERVAL:
        update_lcd()
        last_display_update = now
    
    time.sleep(0.01)
