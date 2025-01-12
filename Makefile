all: obj_dir/Vcitadel
	obj_dir/Vcitadel

include bootrom/build.mk

obj_dir/Vcitadel: obj_dir/Vcitadel.mk
	cd obj_dir && make -f Vcitadel.mk CXX=clang++ LINK=clang++ -j

obj_dir/Vcitadel.mk: picorv32.v citadel.v main.cc bootrom/bootrom.v
	verilator --cc --exe --top-module citadel picorv32.v citadel.v main.cc -Wno-fatal

clean: bootrom_clean
	rm -fr obj_dir

.PHONY: all clean
