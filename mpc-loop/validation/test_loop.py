"""ARM execution test for one-time initialization and repeated solver calls.
HAL initialization, time and UART are simulated, not board-level validation.
Usage: python test_loop.py path/to/acadosdayo.elf [normal|solve_failure|nan_output]
"""
import sys,struct,math
from unicorn import *
from unicorn.arm_const import *
from elftools.elf.elffile import ELFFile
mode=sys.argv[2] if len(sys.argv)>2 else 'normal'
assert mode in ['normal','solve_failure','nan_output']
u=Uc(UC_ARCH_ARM,UC_MODE_THUMB);u.ctl_set_cpu_model(UC_CPU_ARM_MAX)
u.mem_map(0x08000000,0x200000);u.mem_map(0x20000000,0x80000);u.mem_map(0xe0000000,0x100000)
with open(sys.argv[1],'rb') as f:
 e=ELFFile(f);syms={s.name:s.entry.st_value for s in e.get_section_by_name('.symtab').iter_symbols()}
 for sec in e.iter_sections():
  if sec['sh_flags']&2 and sec['sh_type']!='SHT_NOBITS' and sec['sh_size']:
   u.mem_write(sec['sh_addr'],sec.data())
u.reg_write(UC_ARM_REG_CP_REG,(15,0,0,1,0,0,2,0xf00000));u.reg_write(UC_ARM_REG_FPEXC,0x40000000)
u.reg_write(UC_ARM_REG_SP,0x20080000);u.reg_write(UC_ARM_REG_LR,0x08000101)
def ui(n):return struct.unpack('<I',u.mem_read(syms[n],4))[0]
def si(n):return struct.unpack('<i',u.mem_read(syms[n],4))[0]
def val(n):return struct.unpack('<d',u.mem_read(syms[n],8))[0]
def add(name,fn):return u.hook_add(UC_HOOK_CODE,fn,begin=syms[name]&~1,end=syms[name]&~1)
def ret(uc,value=0):
 uc.reg_write(UC_ARM_REG_R0,value);uc.reg_write(UC_ARM_REG_PC,uc.reg_read(UC_ARM_REG_LR))
for n in ['HAL_Init','SystemClock_Config','MX_GPIO_Init','MX_ETH_Init','MX_USART3_UART_Init','MX_USB_OTG_FS_PCD_Init','MX_TIM7_Init']:
 add(n,lambda uc,a,s,d:ret(uc))
tick=0; text=bytearray();creates=0;capsules=0;solves=0;cycles=0;heap=[];failed_cycles=[];pointers=set();faults=[]
def get_tick(uc,a,s,d):
 global tick
 tick+=1;ret(uc,tick)
add('HAL_GetTick',get_tick)
def uart(uc,a,s,d):
 text.append(uc.reg_read(UC_ARM_REG_R0)&255);ret(uc)
add('__io_putchar',uart)
def capsule(uc,a,s,d):
 global capsules
 capsules+=1
add('interaction_model_acados_create_capsule',capsule)
def create(uc,a,s,d):
 global creates
 creates+=1
add('interaction_model_acados_create',create)
def solve(uc,a,s,d):
 global solves
 solves+=1;pointers.add(uc.reg_read(UC_ARM_REG_R0))
 if mode=='solve_failure' and solves==5:ret(uc,4)
add('interaction_model_acados_solve',solve)
def cstr(p):
 result=bytearray()
 while True:
  b=u.mem_read(p,1)
  if b==b'\0':return result.decode()
  result.extend(b);p+=1
if mode=='nan_output':
 def out_get(uc,a,s,d):
  sp=uc.reg_read(UC_ARM_REG_SP)
  name,dest=struct.unpack('<II',uc.mem_read(sp,8))
  if solves==5 and uc.reg_read(UC_ARM_REG_R3)==1 and cstr(name)=='u':
   uc.mem_write(dest,struct.pack('<d',math.nan));ret(uc)
 add('ocp_nlp_out_get',out_get)
def delay(uc,a,s,d):
 global tick,cycles
 tick+=uc.reg_read(UC_ARM_REG_R0);cycles+=1
 assert ui('mpc_iteration_count')==cycles
 heap.append(ui('__sbrk_heap_end'))
 if mode!='normal' and cycles==5:
  assert si('mpc_result_valid')==0
  assert math.isnan(val('mpc_u0')) and math.isnan(val('mpc_u1'))
  assert all(math.isnan(v) for v in struct.unpack('<4d',uc.mem_read(syms['mpc_x0'],32)))
  assert si('mpc_solve_status')==(4 if mode=='solve_failure' else 0)
  failed_cycles.append(cycles)
 else:
  assert si('mpc_solve_status')==0,(mode,cycles,si('mpc_solve_status'))
  assert si('mpc_result_valid')==1
  assert math.isfinite(val('mpc_u0')) and math.isfinite(val('mpc_u1'))
 ret(uc)
 if cycles==300:uc.emu_stop()
add('HAL_Delay',delay)
for name in ['Error_Handler','hardfault_c','_exit']:
 def trap(uc,a,s,d):faults.append(a);uc.emu_stop()
 add(name,trap)
u.emu_start(syms['main'],0,timeout=55000000,count=300000000)
assert not faults,faults
assert creates==capsules==1,(creates,capsules)
assert solves==cycles==300,(solves,cycles)
assert len(pointers)==1
assert len(set(heap))==1, 'Heap grew across loops'
assert ui('g_heap_failed_requests')==0
assert ui('mpc_failure_count')==(0 if mode=='normal' else 1)
assert ui('mpc_success_count')==300-ui('mpc_failure_count')
assert text.count(b'========== MPC Result ==========')>=2
assert text.count(b'========== MPC Result ==========')<10
print('PASS',mode,': init=1, cycles=300, stable heap, UART throttled, failures=',ui('mpc_failure_count'))
print('latest u0=',val('mpc_u0'),'u1=',val('mpc_u1'))
