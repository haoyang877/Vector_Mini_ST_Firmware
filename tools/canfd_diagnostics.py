"""Read ControlCANFD channel diagnostics without resetting the adapter.

Structures follow the bundled vendor header. Return codes are retained: an
unsupported API or failed read is unavailable, never evidence of zero errors.
This module performs no device I/O on import.
"""
import ctypes as C

CAN_ERR_FLAG = 0x20000000


class ChannelError(C.Structure):
    _fields_ = [('error_code', C.c_uint32), ('passive_ErrData', C.c_uint8 * 3),
                ('arLost_ErrData', C.c_uint8)]


class ChannelStatus(C.Structure):
    _fields_ = [(name, C.c_uint8) for name in (
        'errInterrupt', 'regMode', 'regStatus', 'regALCapture', 'regECCapture',
        'regEWLimit', 'regRECounter', 'regTECounter')] + [('Reserved', C.c_uint32)]


def error_record(identifier, flags, payload, device_us, host_time):
    """Retain the SDK record exactly; ERR and the FD ESI bit are distinct."""
    if not identifier & CAN_ERR_FLAG:
        return None
    return dict(raw_id=hex(identifier), fd_flags=flags, esi=bool(flags & 2),
                length=len(payload), data=payload.hex(), device_us=device_us,
                host_time=host_time)


def read_channel_diagnostics(bus, channel=0):
    """Call while the original session is open, preferably after sending STOP."""
    result = {}
    for name, kind in [('ZCAN_ReadChannelErrInfo', ChannelError),
                       ('ZCAN_ReadChannelStatus', ChannelStatus)]:
        try:
            function = getattr(bus.dll, name)
            function.argtypes = [C.c_void_p, C.POINTER(kind)]
            function.restype = C.c_uint32
            value = kind()
            code = function(bus.channels[channel], C.byref(value))
            row = {'return_code': code, 'available': code == 1}
            if code == 1:
                row['values'] = {
                    key: list(getattr(value, key)) if isinstance(getattr(value, key), C.Array)
                    else getattr(value, key) for key, _ in kind._fields_}
            result[name] = row
        except Exception as exc:
            result[name] = {'available': False, 'error': repr(exc)}
    return result
