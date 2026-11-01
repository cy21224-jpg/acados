"""Run a compiled ARM test harness; requires unicorn and pyelftools.

Usage: python check_arm.py /path/to/ekf.elf [...]
ELF needs arm_ekf_step() calling EKF with the param_3link globals.
No peripheral startup, hardware interrupts or real-time timing are simulated.
"""
import math
import struct
import sys
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_HOOK_BLOCK
from unicorn.arm_const import (UC_CPU_ARM_MAX, UC_ARM_REG_CP_REG, UC_ARM_REG_FPEXC,
                               UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_PC)
from elftools.elf.elffile import ELFFile
from test_ekf import reference

def run(path):
    u=Uc(UC_ARCH_ARM,UC_MODE_THUMB)
    u.ctl_set_cpu_model(UC_CPU_ARM_MAX)
    for base,size in [(0x08000000,0x200000),(0x20000000,0x80000)]:u.mem_map(base,size)
    with open(path,'rb') as file:
        elf=ELFFile(file)
        symbols={s.name:s['st_value'] for s in elf.get_section_by_name('.symtab').iter_symbols()}
        for section in elf.iter_sections():
            if section['sh_flags']&2 and section['sh_type']!='SHT_NOBITS' and section['sh_size']:
                u.mem_write(section['sh_addr'],section.data())
    u.reg_write(UC_ARM_REG_CP_REG,(15,0,0,1,0,0,2,0xf00000))
    u.reg_write(UC_ARM_REG_FPEXC,0x40000000)
    lowest_sp=0x20080000
    def block(uc,address,size,data):
        nonlocal lowest_sp
        lowest_sp=min(lowest_sp,uc.reg_read(UC_ARM_REG_SP))
    u.hook_add(UC_HOOK_BLOCK,block)
    def write(name,values):u.mem_write(symbols[name],struct.pack('<'+str(len(values))+'d',*values))
    def read(name,size):return list(struct.unpack('<'+str(size)+'d',u.mem_read(symbols[name],8*size)))
    def call():
        u.reg_write(UC_ARM_REG_SP,0x20080000)
        u.reg_write(UC_ARM_REG_LR,0x081ff001)
        u.emu_start(symbols['arm_ekf_step'],0x081ff000,count=3000000)
        assert u.reg_read(UC_ARM_REG_PC)==0x081ff000
        return struct.unpack('<i',u.mem_read(symbols['arm_ekf_status'],4))[0]
    flatten=lambda m:[x for row in m for x in row]
    dt=struct.unpack('<f',u.mem_read(symbols['sampling_time'],4))[0]
    p=[[.5 if i==j else 0. for j in range(4)] for i in range(4)]
    q=[[.01 if i==j else 0. for j in range(4)] for i in range(4)]
    r=[[.001,0.],[0.,.0001]]
    x=[.1,.01,1.,100.]
    write('Qkf',flatten(q));write('Rkf',flatten(r));write('M_wy_k_1',[2.])
    for k in range(40):
        y=[.1*math.cos(k*.01),.01*math.sin(k*.01)]
        inputs=[math.sin(k*.01),.1,-.02,.03]
        write('y_k',y);write('u_k_1',inputs);write('x_hat_k_1',x);write('P_k_1',flatten(p))
        expected_x,expected_p=reference(y,inputs,2.,x,p,r,q,dt)
        assert call()==0
        x=read('x_hat_k',4)
        actual_p=read('P_k',16)
        assert all(math.isclose(a,b,rel_tol=3e-10,abs_tol=3e-12) for a,b in zip(x,expected_x))
        assert all(math.isclose(a,b,rel_tol=3e-10,abs_tol=3e-12) for a,b in zip(actual_p,flatten(expected_p)))
        p=[actual_p[i*4:(i+1)*4] for i in range(4)]
    before_x,before_p=read('x_hat_k',4),read('P_k',16)
    write('M_wy_k_1',[0.])
    assert call()==-2
    assert read('x_hat_k',4)==before_x and read('P_k',16)==before_p
    print('PASS ARM:',path,'40 four-input steps match independent Decimal reference; zero-mass rejection; observed stack',0x20080000-lowest_sp,'B')

if __name__=='__main__':
    for path in sys.argv[1:]:run(path)
