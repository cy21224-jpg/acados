"""Exercise the firmware check against real output plus injected bad readbacks."""
from pathlib import Path
import runpy, struct, math
from unicorn import UC_HOOK_CODE
from unicorn.arm_const import *
ctx=runpy.run_path(str(Path(__file__).with_name('emulate.py')))
u,syms=ctx['u'],ctx['syms']

def read_int(name):return struct.unpack('<i',u.mem_read(syms[name],4))[0]
def read_double(name):return struct.unpack('<d',u.mem_read(syms[name],8))[0]
def cstr(p):
 out=bytearray()
 while True:
  b=u.mem_read(p,1)
  if b==b'\0':return out.decode()
  out.extend(b);p+=1

def check(expected_flags, first_bad=-1):
 u.reg_write(UC_ARM_REG_SP,0x20080000)
 u.reg_write(UC_ARM_REG_LR,0x081ff001)
 u.emu_start(syms['MPC_CheckSolution'],0x081ff000,count=1000000)
 assert read_int('mpc_check_done')==1
 assert read_int('mpc_check_status')==expected_flags,(read_int('mpc_check_status'),expected_flags)
 assert read_int('mpc_first_bad_stage')==first_bad

check(0)
assert read_int('mpc_initial_state_fixed')==0
assert 0<=read_double('mpc_u0')<=10
print('PASS actual solver output: u0=',read_double('mpc_u0'), 'check=0, x0 not fixed')

# Skip just one API readback, simulating damaged or invalid solver results.
def inject(field,stage,values):
 def hook(uc,addr,size,data):
  sp=uc.reg_read(UC_ARM_REG_SP)
  fieldptr,dest=struct.unpack('<II',uc.mem_read(sp,8))
  if uc.reg_read(UC_ARM_REG_R3)==stage and cstr(fieldptr)==field:
   uc.mem_write(dest,struct.pack('<'+'d'*len(values),*values))
   uc.reg_write(UC_ARM_REG_PC,uc.reg_read(UC_ARM_REG_LR))
 return u.hook_add(UC_HOOK_CODE,hook,begin=syms['ocp_nlp_out_get']&~1,end=syms['ocp_nlp_out_get']&~1)

cases=[('u',0,[math.nan],2,0),('u',2,[math.inf],2,2),
       ('u',9,[10.01],4,9),('u',0,[-0.01],4,0),
       ('u',0,[0.0],0,-1),('u',0,[10.0],0,-1),
       ('u',0,[10.0+0.5e-6],0,-1),('u',0,[10.0+2e-6],4,0),
       ('x',10,[1.01,0,0,0],8,10),('x',3,[0,math.nan,0,0],2,3)]
for field,stage,values,flags,bad in cases:
 handle=inject(field,stage,values)
 check(flags,bad)
 if field=='u' and stage==0: assert (math.isnan(read_double('mpc_u0')) if math.isnan(values[0]) else read_double('mpc_u0')==values[0])
 u.hook_del(handle)
 print('PASS injected',field,stage,values,'flags=',flags)

u.mem_write(syms['mpc_solve_status'],struct.pack('<i',4))
check(1)
assert math.isnan(read_double('mpc_u0'))
u.mem_write(syms['mpc_solve_status'],struct.pack('<i',0))
print('PASS unsuccessful solve: output not accepted')
check(0)
print('PASS check can be rerun without retaining previous failure flags')

# Capture printf's retargeted UART text, without exercising UART hardware.
u.hook_del(ctx['uart_skip_handle']); u.hook_del(ctx['uart_stop_handle'])
chars=[]
def capture(uc,addr,size,data):
 chars.append(uc.reg_read(UC_ARM_REG_R0)&255)
 uc.reg_write(UC_ARM_REG_PC,uc.reg_read(UC_ARM_REG_LR))
handle=u.hook_add(UC_HOOK_CODE,capture,begin=syms['__io_putchar']&~1,end=syms['__io_putchar']&~1)
u.reg_write(UC_ARM_REG_SP,0x20080000);u.reg_write(UC_ARM_REG_LR,0x081ff001)
u.emu_start(syms['MPC_PrintDiagnostics'],0x081ff000,count=10000000)
text=bytes(chars).decode()
assert 'NUMERIC_BOUNDS_PASS' in text
assert 'u0=' in text and 'u[9]=' in text and 'x[10]=' in text
assert 'WARNING: x0 is a range' in text
assert 'bounds=[0.000000000, 10.000000000]' in text
print('PASS UART report includes values, full horizon and initial-state warning')
