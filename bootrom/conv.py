with open("bootrom/bootrom.bin", "rb") as inf:
    with open("bootrom/bootrom.v", "w") as outf:
        for i, c in enumerate(inf.read()):
            print(f"mem[{str(i).rjust(5)}] = 8'b{bin(c)[2:].zfill(8)};", file=outf)
