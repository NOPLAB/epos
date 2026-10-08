"""Bindings to the installed maxon Windows command library."""

import ctypes as ct
import os
from pathlib import Path

DEFAULT_DLL = Path(
    r"C:\Program Files (x86)\maxon motor ag\EPOS IDX\EPOS2\04 Programming"
    r"\Windows DLL\Microsoft Visual C Sharp\Example C#\EposCmd64.dll"
)
DWORD = ct.c_uint32
WORD = ct.c_uint16
BYTE = ct.c_uint8
LONG = ct.c_int32
BOOL = ct.c_int32
HANDLE = ct.c_void_p


class Device:
    def __init__(self, dll=DEFAULT_DLL, port="USB0", node=3):
        if not 1 <= node <= 127:
            raise ValueError("Node must be between 1 and 127")
        if os.name != "nt" or ct.sizeof(HANDLE) != 8:
            raise RuntimeError("The command library requires 64-bit Windows Python")
        self.library = ct.WinDLL(str(Path(dll).resolve()), use_last_error=True)
        self.node = node
        self.handle = None
        self.functions = {}
        error_pointer = ct.POINTER(DWORD)
        node_prefix = [HANDLE, WORD]
        signatures = {
            "OpenDevice": ([ct.c_char_p] * 4, HANDLE),
            "CloseDevice": ([HANDLE], BOOL),
            "GetObject": (node_prefix + [WORD, BYTE, HANDLE, DWORD, error_pointer], BOOL),
            "SetObject": (node_prefix + [WORD, BYTE, HANDLE, DWORD, error_pointer], BOOL),
            "SetVelocityRegulatorGain": (node_prefix + [WORD, WORD], BOOL),
            "SetVelocityRegulatorFeedForward": (node_prefix + [WORD, WORD], BOOL),
            "SetCurrentRegulatorGain": (node_prefix + [WORD, WORD], BOOL),
            "GetCurrentRegulatorGain": (node_prefix + [ct.POINTER(WORD)] * 2, BOOL),
            "GetVelocityRegulatorGain": (node_prefix + [ct.POINTER(WORD)] * 2, BOOL),
            "SetVelocityProfile": (node_prefix + [DWORD, DWORD], BOOL),
            "MoveWithVelocity": (node_prefix + [LONG], BOOL),
            "GetPositionIs": (node_prefix + [ct.POINTER(LONG)], BOOL),
            "GetVelocityIs": (node_prefix + [ct.POINTER(LONG)], BOOL),
            "GetCurrentIs": (node_prefix + [ct.POINTER(ct.c_int16)], BOOL),
            "GetFaultState": (node_prefix + [ct.POINTER(BOOL)], BOOL),
        }
        for name in ("SetEnableState", "SetDisableState", "ClearFault", "ActivateProfileVelocityMode", "HaltVelocityMovement"):
            signatures[name] = (node_prefix, BOOL)
        for name, (arguments, result) in signatures.items():
            function = getattr(self.library, "VCS_" + name)
            function.argtypes = arguments + [error_pointer]
            function.restype = result
            self.functions[name] = function
        self.port = port.encode("ascii")

    def _call(self, name, *arguments):
        error = DWORD()
        result = self.functions[name](*arguments, ct.byref(error))
        if not result:
            raise RuntimeError(f"{name} failed: 0x{error.value:08X}")
        return result

    def __enter__(self):
        self.handle = self._call("OpenDevice", b"EPOS2", b"MAXON SERIAL V2", b"USB", self.port)
        return self

    def __exit__(self, *exception):
        self._call("CloseDevice", self.handle)
        self.handle = None

    def command(self, name, *arguments):
        return self._call(name, self.handle, self.node, *arguments)

    def read(self, index, subindex, size, signed=False):
        if size not in (1, 2, 4):
            raise ValueError("Object size must be 1, 2, or 4")
        data = (BYTE * size)()
        transferred = DWORD()
        self.command("GetObject", index, subindex, data, size, ct.byref(transferred))
        if transferred.value != size:
            raise RuntimeError("Unexpected object read size")
        return int.from_bytes(bytes(data), "little", signed=signed)

    def write(self, index, subindex, value, size):
        if size not in (1, 2, 4):
            raise ValueError("Object size must be 1, 2, or 4")
        data = (BYTE * size).from_buffer_copy(value.to_bytes(size, "little"))
        transferred = DWORD()
        self.command("SetObject", index, subindex, data, size, ct.byref(transferred))
        if transferred.value != size or self.read(index, subindex, size) != value:
            raise RuntimeError(f"Write verification failed: 0x{index:04X}:{subindex}")

    def value(self, name, scalar=LONG):
        result = scalar()
        self.command(name, ct.byref(result))
        return result.value

    def gains(self, loop):
        proportional, integral = WORD(), WORD()
        self.command(f"Get{loop}RegulatorGain", ct.byref(proportional), ct.byref(integral))
        return proportional.value, integral.value

    def set_gains(self, loop, proportional, integral):
        if not all(0 <= value <= 65535 for value in (proportional, integral)):
            raise ValueError("Gains must fit unsigned 16-bit values")
        self.command(f"Set{loop}RegulatorGain", proportional, integral)
        if self.gains(loop) != (proportional, integral):
            raise RuntimeError(f"{loop} gain verification failed")

    def sample(self):
        return {
            "rpm": self.value("GetVelocityIs"),
            "current_ma": self.value("GetCurrentIs", ct.c_int16),
            "position_qc": self.value("GetPositionIs"),
            "fault": bool(self.value("GetFaultState", BOOL)),
            "statusword": self.read(0x6041, 0, 2),
            "demand_rpm": self.read(0x606B, 0, 4, signed=True),
        }
