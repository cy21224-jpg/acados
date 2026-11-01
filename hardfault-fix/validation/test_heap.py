import struct,sys
from pathlib import Path
init=Path(__file__).with_name('emulate.py').read_text().split('returns=0;')[0]
exec(init)
def call(incr):
 u.reg_write(UC_ARM_REG_R0,incr&0xffffffff)
 u.reg_write(UC_ARM_REG_LR,0x081ff001)
 u.emu_start(syms['_sbrk'],0x081ff000,count=10000)
 return u.reg_read(UC_ARM_REG_R0)
base=call(0)
assert call(256)==base
assert call(-128)==base+256
assert call(-129)==0xffffffff
assert call(0)==base+128
assert call(0x80000)==0xffffffff
assert call(0)==base+128
get=lambda n:struct.unpack('<I',u.mem_read(syms[n],4))[0]
assert get('g_heap_peak_bytes')==256
assert get('g_heap_failed_requests')==2
print('PASS heap grow/shrink/bounds/high-water/failure counters')
