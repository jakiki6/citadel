bootrom/bootrom.v: bootrom/bootrom.bin
	python3 bootrom/conv.py

bootrom/bootrom.bin: bootrom/bootrom.elf
	llvm-objcopy --output-target=binary $< $@

bootrom/bootrom.elf: bootrom/main.o
	ld.lld -nostdlib -o $@ $^ -Tbootrom/layout.ld

%.o: %.c
	clang -target riscv32-none-elf -c -Os -nostdinc -Ishared/include -o $@ $< -Werror -Wno-main-return-type -Wno-invalid-noreturn

bootrom_clean:
	rm -f bootrom/bootrom.v bootrom/bootrom.bin bootrom/*.elf bootrom/*.o

.PHONY: bootrom_clean
