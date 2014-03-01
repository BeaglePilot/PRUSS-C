PRUSS-C
=======

PRUSS C compiler test
###Compiling App Loader
    git clone https://github.com/BeaglePilot/PRUSS-C.git
    cd PRUSS-C/am335x_pru_package/ pru_sw/app_loader/interface
    make
    cd ../lib
    scp libprussdrv.so root@beaglebone.local:/usr/local/lib
    
###Basic Command for Compilation
You can find an example blinkled PRU code @ PRUSS-C/am335x_pru_package/pru_sw/example_apps/blinkled/blinkled_pru.c
####for 64-bit systems only:    
    sudo apt-get install ia32-libs-multiarch
####else for 32-bit systems start from here 
######for x86_64/i686
    export PRU_C_DIR="/opt/ti/PRUCGT1.0.0B1/include;/opt/ti/PRUCGT1.0.0B1/lib"
    export PATH="/opt/ti/PRUCGT1.0.0B1/bin;/opt/ti/PRUCGT1.0.0B1/;/opt/ti/PRUCGT1.0.0B1/example":$PATH
    clpru --silicon_version=3 -o1 blinkled_pru.c -z AM3359_PRU.cmd -o PRU_tests.out -m PRU_tests.map
    hexpru bin.cmd PRU_tests.out
This generates 2 files: data.bin (containing the data sections) and text.bin (containing the .text sections).
######for arm systems
    export PRU_C_DIR="/home/root/ARMLinuxA8/include;/home/root/ARMLinuxA8/lib"
    export PATH="/home/root/ARMLinuxA8/bin;/home/root/ARMLinuxA8/;/home/root/ARMLinuxA8/example":$PATH
    clpru --silicon_version=3 -o1 blinkled_pru.c -z AM3359_PRU.cmd -o PRU_tests.out -m PRU_tests.map
    hexpru bin.cmd PRU_tests.out
This generates 2 files: data.bin (containing the data sections) and text.bin (containing the .text sections).

###Compiling your BBB main app
#####Example blinkled:
    cd PRUSS-C/am335x_pru_package/pru_sw/example_apps/
    make
    cd bin/
    scp blinkled root@beaglebone.local:~/
###Executing your Application on BBB
Copy data.bin and text.bin to the same folder as blinkled application.

Note:follow the steps in the shown order

    export LD_LIBRARY_PATH=/usr/local/lib
    modprobe uio_pruss

Create BB-PRU-00A0.dts with contents as shown @ http://hipstercircuits.com/category/pru/ and load the cape using capemgr.

and finally
    
    ./blinkled

you should see two LEDs blinking
    
