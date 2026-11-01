import sys,struct
from pathlib import Path
init=Path(__file__).with_name('emulate.py').read_text().split('returns=0;')[0]
for exc_return in [0xfffffff9,0xffffffe9]:
 exec(init)
 sp=0x20070000
 values=[1,2,3,4,12,0x08001235,0x08005678,0x01000000]+[400]*18
 u.mem_write(sp,struct.pack('<26I',*values))
 u.mem_write(0xe000ed28,struct.pack('<I',0x400))
 u.mem_write(0xe000ed2c,struct.pack('<I',0x40000000))
 u.reg_write(UC_ARM_REG_R0,sp);u.reg_write(UC_ARM_REG_R1,exc_return)
 u.emu_start(syms['hardfault_c'],0,count=1000)
 def get(n):return struct.unpack('<I',u.mem_read(syms[n],4))[0]
 assert get('g_hf_stacked_pc')==0x08005678
 assert get('g_hf_stacked_lr')==0x08001235
 assert get('g_hf_stacked_xpsr')==0x01000000
 assert get('g_hf_frame_valid')==1
 assert get('g_hf_cfsr')==0x400
 print('PASS exception frame',hex(exc_return))
# A stacking error must not cause dereferencing a damaged stack pointer.
exec(init)
u.mem_write(0xe000ed28,struct.pack('<I',0x1000))
u.reg_write(UC_ARM_REG_R0,0);u.reg_write(UC_ARM_REG_R1,0xfffffff9)
u.emu_start(syms['hardfault_c'],0,count=1000)
assert struct.unpack('<I',u.mem_read(syms['g_hf_frame_valid'],4))[0]==0
print('PASS damaged exception frame skipped')
