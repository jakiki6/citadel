all: obj_dir/Vcitadel
	obj_dir/Vcitadel

include bootrom/build.mk

obj_dir/Vcitadel: obj_dir/Vcitadel.mk soc/main.cc
	cd obj_dir && make -f Vcitadel.mk CXX=clang++ LINK=clang++ -j

obj_dir/Vcitadel.mk: soc/citadel.v soc/picorv32.v soc/uart.v soc/main.cc bootrom/bootrom.v
	verilator --cc --exe --trace --top-module citadel soc/citadel.v soc/main.cc -Wno-fatal

clean: bootrom_clean
	rm -fr obj_dir

.PHONY: all clean
