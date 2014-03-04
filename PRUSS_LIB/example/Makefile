CC = clpru
DEVICE=am335x
CFLAGS = --silicon_version=3 -o1 -Dam335x 
HEXBIN = hexpru
PRU_COMP_PATH = ${PRU_BASE_PATH}/ARMLinuxA8

PRU_C_DIR = -I${PRU_COMP_PATH}/lib \
	    -I${PRU_COMP_PATH}/include \
	    -I${PRU_BASE_PATH}/AM335X_StarterWare_02_00_01_01/include \
	    -I${PRU_BASE_PATH}/AM335X_StarterWare_02_00_01_01/include/arm7a \
	    -I${PRU_BASE_PATH}/AM335X_StarterWare_02_00_01_01/include/hw \
	    -I${PRU_BASE_PATH}/AM335X_StarterWare_02_00_01_01/platform/beaglebone

DRIVER_DIR = ${PRU_BASE_PATH}/AM335X_StarterWare_02_00_01_01/drivers
SRC = $(wildcard ${DRIVER_DIR}/*.c) blinkled.c
OUT = blinkled.out
CMD = ${PRU_BASE_PATH}/ARMLinuxA8/example/AM3359_PRU.cmd
BUILD_DIR = build

$(BUILD_DIR)/$(OUT):$(SRC)
	mkdir -p $(BUILD_DIR)
	$(CC) $(CFLAGS) $(PRU_C_DIR) $^ -z $(CMD) -o $@ -m $(OUT).map
	rm -rf *.out *.map *.obj
	$(HEXBIN) $(PRU_COMP_PATH)/bin.cmd $@
	cp *.bin $(BUILD_DIR)
	rm -rf *.bin
clean:
	rm -r build/
