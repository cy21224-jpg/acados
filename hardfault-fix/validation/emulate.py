import sys, struct
from unicorn import *
from unicorn.arm_const import *
from elftools.elf.elffile import ELFFile
u=Uc(UC_ARCH_ARM,UC_MODE_THUMB);u.ctl_set_cpu_model(UC_CPU_ARM_MAX)
u.mem_map(0x8000000,0x200000);u.mem_map(0x20000000,0x80000);u.mem_map(0xe0000000,0x100000)
with open(sys.argv[1], 'rb') as f:
 e=ELFFile(f); syms={s.name:s.entry.st_value for s in e.get_section_by_name('.symtab').iter_symbols()}
 for sec in e.iter_sections():
  if sec['sh_flags']&2 and sec['sh_type']!='SHT_NOBITS' and sec['sh_size']:
   u.mem_write(sec['sh_addr'],sec.data())
u.mem_write(0xe000ed88,struct.pack('<I',0xf00000))
u.reg_write(UC_ARM_REG_CP_REG,(15,0,0,1,0,0,2,0xf00000)); u.reg_write(UC_ARM_REG_FPEXC,0x40000000)
u.reg_write(UC_ARM_REG_SP,0x20080000); u.reg_write(UC_ARM_REG_LR,0x8000101)
returns=0; lowsp=0x20080000
def block(uc,addr,size,data):
 global lowsp
 lowsp=min(lowsp,uc.reg_read(UC_ARM_REG_SP))
u.hook_add(UC_HOOK_BLOCK,block)

def skip(uc,addr,size,data):
 uc.reg_write(UC_ARM_REG_R0,0);uc.reg_write(UC_ARM_REG_PC,uc.reg_read(UC_ARM_REG_LR))
for name in ['HAL_Init','SystemClock_Config','MX_GPIO_Init','MX_ETH_Init','MX_USART3_UART_Init','MX_USB_OTG_FS_PCD_Init','MX_TIM7_Init','__io_putchar']:
 if name in syms: u.hook_add(UC_HOOK_CODE,skip,begin=syms[name]&~1,end=syms[name]&~1)
def hook(uc,addr,size,data):
 global returns
 returns+=1
 print('SOLVE ENTRY',returns,'heap_end',hex(struct.unpack('<I',uc.mem_read(syms['__sbrk_heap_end'],4))[0]),flush=True)
u.hook_add(UC_HOOK_CODE,hook,begin=syms['interaction_model_acados_solve']&~1,end=syms['interaction_model_acados_solve']&~1)
def fail(uc,access,addr,size,value,data):
 print('INVALID',access,hex(addr),'pc',hex(uc.reg_read(UC_ARM_REG_PC)),'sp',hex(uc.reg_read(UC_ARM_REG_SP)),flush=True);return False
u.hook_add(UC_HOOK_MEM_INVALID,fail)
def done(uc,addr,size,data):
 print('STOP',hex(addr),flush=True);uc.emu_stop()
for n in ['Error_Handler','hardfault_c','_exit','__io_putchar']:
 if n in syms and n!='__io_putchar':u.hook_add(UC_HOOK_CODE,done,begin=syms[n]&~1,end=syms[n]&~1)
def solved(uc,addr,size,data):
 v=struct.unpack('<i',uc.mem_read(syms['mpc_solve_status'],4))[0]
 if v!=-999:
  print('SOLVE RETURN',v,flush=True);uc.emu_stop()
u.hook_add(UC_HOOK_CODE,solved,begin=syms['__io_putchar']&~1,end=syms['__io_putchar']&~1)
try: u.emu_start(syms['main'],0,timeout=45000000,count=100000000)
except UcError as ex: print('ERROR',ex, 'pc',hex(u.reg_read(UC_ARM_REG_PC)),'sp',hex(u.reg_read(UC_ARM_REG_SP)))
for n in ['mpc_create_status','mpc_solve_status','__sbrk_heap_end','g_heap_peak_bytes','g_heap_failed_requests']:
 if n not in syms: continue
 print(n,hex(struct.unpack('<I',u.mem_read(syms[n],4))[0]))

print('Maximum observed stack bytes',0x20080000-lowsp)
assert returns==1, 'Expected exactly one solve call'
assert struct.unpack('<i',u.mem_read(syms['mpc_solve_status'],4))[0]==0, 'Solve did not succeed'
if 'g_heap_failed_requests' in syms: assert u.mem_read(syms['g_heap_failed_requests'],4)==bytes(4)
