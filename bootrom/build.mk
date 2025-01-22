bootrom/bootrom.v: bootrom/bootrom.bin
	python3 bootrom/conv.py

bootrom/bootrom.bin: bootrom/bootrom.elf
	riscv32-none-elf-objcopy --output-target=binary $< $@

bootrom/bootrom.elf: bootrom/main.o
	riscv32-none-elf-ld -nostdlib -o $@ $^ -Tbootrom/layout.ld

%.o: %.c
	riscv32-none-elf-gcc -c -O2 -nostdinc -Ishared/include -o $@ $<

bootrom_clean:
	rm -f bootrom/bootrom.v bootrom/bootrom.bin bootrom/*.elf bootrom/*.o

.PHONY: bootrom_clean
