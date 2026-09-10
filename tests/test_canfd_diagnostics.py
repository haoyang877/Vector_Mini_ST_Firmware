import ctypes as C
from pathlib import Path
import sys
import unittest
from types import SimpleNamespace

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from canfd_diagnostics import ChannelError, ChannelStatus, error_record, read_channel_diagnostics


class Function:
    def __init__(self, kind, code): self.kind,self.code=kind,code
    def __call__(self, handle, pointer):
        assert handle==123
        value=C.cast(pointer,C.POINTER(self.kind)).contents
        if self.kind is ChannelError:value.error_code=0x10
        else:value.regTECounter=122
        return self.code


class DiagnosticsTests(unittest.TestCase):
    def test_vendor_abi_and_error_flags_are_distinct(self):
        self.assertEqual(C.sizeof(ChannelError),8)
        self.assertEqual(C.sizeof(ChannelStatus),12)
        self.assertIsNone(error_record(0x7f3,3,b'abc',1,2))
        error=error_record(0x200007f3,1,b'abc',1,2)
        self.assertEqual(error['raw_id'],'0x200007f3')
        self.assertFalse(error['esi'])
        self.assertEqual(error['data'],'616263')

    def test_success_failure_and_missing_api(self):
        dll=SimpleNamespace(ZCAN_ReadChannelErrInfo=Function(ChannelError,1),
                            ZCAN_ReadChannelStatus=Function(ChannelStatus,0))
        bus=SimpleNamespace(dll=dll,channels={0:123})
        result=read_channel_diagnostics(bus)
        self.assertEqual(result['ZCAN_ReadChannelErrInfo']['values']['error_code'],16)
        self.assertFalse(result['ZCAN_ReadChannelStatus']['available'])
        self.assertNotIn('values',result['ZCAN_ReadChannelStatus'])
        del dll.ZCAN_ReadChannelStatus
        self.assertFalse(read_channel_diagnostics(bus)['ZCAN_ReadChannelStatus']['available'])


if __name__=='__main__':unittest.main()
