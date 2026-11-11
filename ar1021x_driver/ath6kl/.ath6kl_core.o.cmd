savedcmd_ath6kl_core.o := ld -m elf_x86_64 -z noexecstack --no-warn-rwx-segments   -r -o ath6kl_core.o @ath6kl_core.mod  ; /usr/src/kernels/7.2.15-1.surface.fc43.x86_64/tools/objtool/objtool --hacks=jump_label --hacks=noinstr --hacks=skylake --ibt --prefix=16 --orc --retpoline --rethunk --sls --static-call --uaccess  --link  --module ath6kl_core.o

ath6kl_core.o: $(wildcard /usr/src/kernels/7.2.15-1.surface.fc43.x86_64/tools/objtool/objtool)
