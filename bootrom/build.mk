bootrom/rom.hex: bootrom/rom.bin
	xxd -c 1 -plain $< > $@

bootrom/rom.bin: bootrom/rom.elf
	riscv32-none-elf-objcopy --output-target=binary $< $@

bootrom/rom.elf: bootrom/start.o bootrom/main.o
	riscv32-none-elf-ld -nostdlib -o $@ $^ -Tbootrom/layout.ld

%.o: %.c
	riscv32-none-elf-gcc -c -O2 -o $@ $<

%.o: %.s
	riscv32-none-elf-as -o $@ $<

bootrom_clean:
	rm -f bootrom/rom.hex bootrom/rom.bin bootrom/*.elf bootrom/*.o

.PHONY: bootrom_clean
