EMU_ARGS=""

while getopts ":d" opt; do
  case ${opt} in
    d )
      EMU_ARGS="$EMU_ARGS -S -s";;
  esac
done

~/git/qemu_stm32/build/arm-softmmu/qemu-system-arm $EMU_ARGS -nographic -monitor none -serial stdio -machine vex-cortex -kernel bin/output.bin 2> >(\
	while read line; do
		echo -e "\e[01;31m$(echo -e "$line" | tr -s "[:print:]\n")\e[0m" >&2;
		if [[ $line == *"R15="* ]]; then
  			echo -e "\n\e[34m$(addr2line -e bin/output.elf `echo $line | sed -n "s/^.*R15=\s*\(\S*\).*$/\1/p"`)\e[0m\n";
		fi
	done
)
