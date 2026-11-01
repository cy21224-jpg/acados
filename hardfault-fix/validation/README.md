# ARM validation

Requires Python, `unicorn==2.1.4` and `pyelftools==0.33`.
From the hardfault-fix directory, run:

```sh
python validation/emulate.py firmware/acadosdayo.elf
python validation/test_fault_frame.py firmware/acadosdayo.elf
python validation/test_heap.py firmware/acadosdayo.elf
python validation/test_control_output.py firmware/acadosdayo.elf
```

Peripheral initialization is skipped. ARM MAX is used to execute double-precision instructions; this is not a complete Cortex-M7 or STM32 board emulator. Assertion failures return nonzero. Hardware verification remains necessary.
