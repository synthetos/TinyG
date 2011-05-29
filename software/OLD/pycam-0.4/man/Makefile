.PHONY: clean

pycam.1: ../pycam pycam.1.inc
	help2man --no-info --name="Toolpath Generation for 3-Axis CNC machining" \
		--section=1 --manual="PyCAM manual" --include=pycam.1.inc --output=pycam.1 ../pycam

clean:
	@rm -f pycam.1

