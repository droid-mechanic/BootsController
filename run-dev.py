import sys, types, queue, threading

# ── Stub serial ───────────────────────────────────────────────────────────
serial_mod = types.ModuleType("serial")
serial_mod.serialutil = types.ModuleType("serial.serialutil")
class _SE(Exception): pass
serial_mod.serialutil.SerialException = _SE
sys.modules["serial"] = serial_mod
sys.modules["serial.serialutil"] = serial_mod.serialutil

# ── Patch SerialBridge: no-op ─────────────────────────────────────────────
import serial_bridge
class FakeSerialBridge:
    def open(self): print("[dev] SerialBridge: skipped (no hardware)")
    def close(self): pass
    def write_channels(self, ch): return True
    def reconnect(self): pass
    is_connected = True
serial_bridge.SerialBridge = FakeSerialBridge

# ── Run app normally ──────────────────────────────────────────────────────
import app
app.main()
