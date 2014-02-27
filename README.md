PRUSS-C
=======

PRUSS C compiler test

###Basic Command for Compilation in linux
####for 64-bit systems only:    
    sudo apt-get install ia32-libs-multiarch
####else for 32-bit systems start from here 
    export PRU_C_DIR="/opt/ti/PRUCGT1.0.0B1/include;/opt/ti/PRUCGT1.0.0B1/lib"
    export PATH="/opt/ti/PRUCGT1.0.0B1/bin":$PATH
    clpru test.c --run_linker --library=lnk.cmd --library=rtspruv3.lib --output_file=test.out
    
