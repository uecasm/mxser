=============================================================================
          MOXA Smartio/Industio Family Device Driver Installation Guide
            for Linux Kernel 4.x
           Copyright (C) 2022, Moxa Inc.
=============================================================================
Date: 07/14/2022 

Content

0. Note
1. Introduction
2. System Requirement
3. Installation
   3.1 Hardware installation
   3.2 Driver files   
   3.3 Device naming convention
   3.4 Module driver configuration   
   3.5 Static driver configuration 
   3.6 Custom configuration
   3.7 Verify driver installation
4. Utilities
5. Setserial
6. Non-standard baud rate
7. How to sign the driver with a cypher key to support a platform which enables the secure boot
8. Troubleshooting

-----------------------------------------------------------------------------
0. Note

   0.1 Installation step summary.
   
      Hardware installation:
        Refer to the "3.1 Hardware installation".
        
      Software/driver installation:           
        0.1.1 Extract the Moxa driver: Refer to the "3.2 Driver files".
        
        0.1.2 There are two methods to install Moxa driver: module & static. If
          you want to use the module method, refer to the "3.4 Module driver
          configuration". If you want to use the static method, refer to the
          "3.5 & 3.6 Static driver configuration".

-----------------------------------------------------------------------------
1. Introduction

   The Smartio/Industio/UPCI family Linux driver supports following  the 
   multiport boards.

    - 2 ports multiport board 
      CP-102U, CP-102UL, CP-102UF, CP-102E, CP-102EL, 
      CP-132U-I, CP-132UL,, CP-132EL, CP-132EL-I, 
      CP-132, CP-132I, CP132S, CP-132IS,
      CP-112UL, CP-112UL-I,
      CI-132, CI-132I, CI-132IS,  
      (C102H, C102HI, C102HIS, C102P, CP-102, CP-102S) 
      
    - 4 ports multiport board 
      CP-104EL, CP-104EL-A,
      CP-104UL, CP-104JU,
      CP-134U, CP-134U-I, CP-134EL-A-I,
      C104H/PCI, C104HS/PCI,
      CP-114, CP-114I, CP-114S, CP-114IS, CP-114UL, CP-114EL, CP-114EL-I,
      C104H, C104HS,
      CI-104J, CI-104JS,
      CI-134, CI-134I, CI-134IS,  
      (C114HI, CT-114I, C104P)
      POS-104UL
      
    - 8 ports multiport board 
      CP-118EL, CP-168EL,
      CP-118U, CP-118U-I,
      CP-118E-A-I, CP-138E-A-I, CP-116E-A,
      CP-118EL-A, CP-118E-I-A,
      CP-138U, CP-138U-I
      CP-168EL, CP-168U, CP-168EL-A, CP-168U,
      C168H/PCI,
      C168H, C168HS,
      (C168P)
      
    - 16 ports multiport board
      CP-116E-A

   This driver supports x86 and x64 hardware platform. In order to maintain
   compatibility, this version has also been properly tested with several
   Linux distribution (see version.txt). However, if compatibility problem 
   occurs, please contact Moxa Inc. technical support. (support@moxa.com)

   In addition, for the device driver, useful utilities are also provided in 
   this version. They are

    - msdiag    Diagnostic program for displaying installed Moxa 
                Smartio/Industio boards.

    - msmon     Monitor program to observe data count and line status signals.

    - msterm    A simple terminal program which is useful in testing serial
                ports.

    - muestty Device configuration tool for MUE series PCI Express
         multiport board(CP-102E, CP-102EL, CP-132EL, CP-132EL-I,
         CP-114EL, CP-114EL-I, CP-104EL-A, CP-168EL-A, CP-118EL-A,
         CP-118E-A-I, CP-138E-A, CP-134EL-A, CP-116E-A).
         The tool provides two  functions to set and get the interface
         and terminator resistor on the device.

    - io-irq.exe Configuration program to setup ISA boards. Please note that
                 this program can only be executed under DOS.

   All the drivers and utilities are published in the form of source code 
   under GNU General Public License in this version. Please refer to GNU 
   General Public License announcement in each source code file for more 
   detail.

   In Moxa's Web sites, you may always find the latest driver at
   http://www.moxa.com

   This version of driver can be installed as Loadable Module (Module driver)
   or built-in into the kernel (Static driver). You may refer to the following
   installation procedure a for suitable one. Before you install the driver,
   please refer to hardware installation procedure in the User's Manual.

   We assume the user should be familiar with the following documents.
   - Serial-HOWTO
   - Kernel-HOWTO

   Note: The MUE series includes CP-102E, CP-102EL, CP-132EL, CP-132EL-I,
         CP-114EL, CP-114EL-I, CP-104EL-A, CP-168EL-A, CP-118EL-A,
         CP-118E-A-I, CP-138E-A, CP-134EL-A, CP-116E-A, and supports
         linux kernel 4.x.

-----------------------------------------------------------------------------
2. System Requirement
   - Hardware platform: x86, x64
   - Kernel version: 4.x 
   - gcc version 2.72 or later
   - Maximum 4 boards can be installed in combination
   - Kernel source

   Note: If you want to use this driver in VM, you MUST to enable 
         VM-Compatible in physical computer with linux operating system.
         You can use muestty, the utility contained in this driver, to enable
         VM-Compatible feature. You can refer to the "4. Utilities" for more 
         detail.
        
-----------------------------------------------------------------------------
3. Installation

   3.1 Hardware installation
   3.2 Driver files   
   3.3 Device naming convention
   3.4 Module driver configuration   
   3.5 Static driver configuration
   3.6 Custom configuration
   3.7 Verify driver installation
       
             
   3.1 Hardware installation

       There are several types of buses, ISA and PCI/PCIE, for Smartio/Industio 
       family multiport board.

       ISA board
       ---------
       You'll have to configure CAP address, I/O address, Interrupt Vector
       as well as IRQ before installing this driver. Please refer to hardware
       installation procedure in User's Manual before proceeding any further.
       Please make sure the JP1 is open after the ISA board is set properly.

       PCI/UPCI board
       --------------
       You may need to adjust IRQ usage in BIOS to avoid IRQ conflict with 
       other ISA devices. Please refer to hardware installation procedure 
       in User's Manual in advance.

       PCI IRQ Sharing
       -----------
       Each port within the same multiport board shares the same IRQ. Up to
       4 Moxa Smartio/Industio PCI Family multiport boards can be installed 
       together on one system and they can share the same IRQ.


   3.2 Driver files

       The driver file could be obtained from the  website, under the product 
       page.
       The first step is making a copy of the driver file 
       "driv_linux_smart_[VERSION]_[BUILD].tgz" into the specified directory.
       e.g. /moxa.
       Please execute the following commands as below.

       # cd /moxa 
       # tar -xzvf driv_linux_smart_vx.x_build_yymmddhh.tgz
              
       In addition, you could find all driver files in /moxa/mxser. 

   3.3 Device naming convention
   
       You could find all drivers and utilities in /moxa/mxser.
       The installation procedures below are depended on the model you'd like 
       to run the driver. If you prefer the module driver, please refer to 
       3.4. If the static driver is required, please refer to 3.5.

       Dial-in and callout port
       -----------------------
       This driver remains traditional serial device properties and only  
       dial-in ports will be created. The device name for each serial port is 
       /dev/ttyMxx. The MUE series multiport board’s serial port name is  
       /dev/ttyMUExx.

       Device naming when more than 2 boards installed
       -----------------------------------------------
       Naming convention for each Smartio/Industio multiport board is 
       pre-defined as below.

       Board Num.     Dial-in Port 
       1st board    ttyM0  - ttyM7 
       2nd board    ttyM8  - ttyM15 
       3rd board    ttyM16 - ttyM23 
       4th board    ttyM24 - ttyM31

       For MUE series:
       
       Board Num.     Dial-in Port 
       1st board    ttyMUE0  - ttyMUE7 
       2nd board    ttyMUE8  - ttyMUE15 
       3rd board    ttyMUE16 - ttyMUE23 
       4th board    ttyMUE24 - ttyMUE31 

       Board sequence
       --------------
       This driver will activate ISA boards Module according to the 
       parameter set in the driver. After all specified ISA board 
       activated, PCI board will be installed in the system automatically 
       driven. Therefore the board number is sorted by the CAP address of 
       ISA boards. For PCI boards, their sequence will be after ISA boards 
       and C168H/PCI has higher priority than C104H/PCI boards.

   3.4 Module driver configuration
       Module driver is the easiest way to install. If you prefer the static
       driver installation, please skip this paragraph.
       

       ------------- Prepare to use the MOXA driver--------------------  
       3.4.1 Create tty device with correct major number
          Before using MOXA driver, your system must have the tty devices 
          which are created with the driver's major number. We offer one 
          shell script "msmknod" to simplify the procedure.
          This step is only needed to be executed once. But you still
          need to do this procedure when:
          a. You change the driver's major number. Please refer to the 
             "3.7" section.
          b. Your total installed MOXA boards number is changed. Maybe you 
             add/delete one MOXA board.
          c. You want to change the tty name. This needs to modify the 
             shell script "msmknod"

          The procedure is:
      # cd /moxa/mxser/driver
      # ./msmknod

          This shell script will require the major number for dial-in 
          device and callout device to create tty device. You also need 
          to specify the total installed MOXA board number. Default major 
          numbers for dial-in device and callout device are 30, 35. If
          you need to change to other numbers, please refer section "3.7"
          for more detailed procedure.
          Msmknod will delete any special files occupying the same device 
          naming.
          
       3.4.2 Build the MOXA driver and utilities
          Before using the MOXA driver and utilities, you need to compile 
          the all the source code. This step is only need to be executed 
          once. But you still re-compile the source code if you modify the
          source code. For example, if you change the driver's major number
          (see "3.7" section), then you need to do this step again.
                    
          Find "Makefile" in /moxa/mxser, then run

      # make;

      The driver files and utilities will be compiled respectively.

       3.4.3 Install the MOXA driver and utilities
      To install the MOXA driver and utilities, you have to execute the
      command as below in /moxa/mxser directory. The driver and the
      utilities will be installed to the system directory respectively.

      # make install

          !!!!!!!!!! NOTE !!!!!!!!!!!!!!!!! 
      For Red Hat Enterprise Linux AS4/ES4/WS4:
      # make installsp2

      For Red Hat Enterprise Linux 8
      # make installsp3
          !!!!!!!!!! NOTE !!!!!!!!!!!!!!!!! 
  
       ------------- Load MOXA driver--------------------  
       3.4.4 Load the MOXA driver  

      # modprobe mxser <argument>
        or
      # modprobe mxupcie <option>

      will activate the module driver. You may run "lsmod" to check
      if "mxser" (or "mxupcie") is activated. If the MOXA board is
      ISA board, the <argument> is needed. Please refer to section
      "3.4.7" for more information.

      The "mxupcie" is for MUE series multiport board only.

      To simplify the processes above, we provide a single step to 
      build, install and load the MOXA driver. You may execute the
      ./mxinstall in /moxa/mxser/ to use the MOXA product.

      # ./mxinstall
 
       ------------- Load MOXA driver on boot --------------------  
       3.4.5 For the above description, you may manually execute 
          "modprobe mxser" (or "modprobe mxupcie") to activate this
          driver and run "rmmod mxser" (or "rmmod mxupcie") to remove it. 
          However, it's better to have a boot time configuration to 
          eliminate manual operation. Boot time configuration can be 
          achieved by rc file. We offer one "rc.mxser" file to simplify 
          the procedure under "moxa/mxser/driver". 
          
          But if you use ISA board, please modify the "modprobe ..." command 
          to add the argument (see "3.4.7" section). After modifying the 
          rc.mxser, please try to execute "/moxa/mxser/driver/rc.mxser" 
          manually to make sure the modification is ok. If any error 
          encountered, please try to modify again. If the modification is 
          completed, follow the below step.
      
      a. For Red Hat
        Run the following command for setting rc files.
        
        # cd /moxa/mxser/driver
        # cp ./rc.mxser /etc/init.d/
        # cp ./mxser.service /etc/systemd/system/
        # systemctl enable mxser.service
        # systemctl start mxser.service

        Reboot and check if mxser or mxupcie activated by "lsmod" command.

      b. For Debian
	Run the following command for setting rc files.
        
        # cd /moxa/mxser/driver
        # cp ./rc.mxser /etc/init.d/
        
        Reboot and check if mxser or mxupcie activated by "lsmod" command.

        If the Debian system boots without Multiport Serial Boards driver loaded, please
        execute the following command with root administration to resolve this
        problem:

          # update-rc.d rc.mxser defaults

        Starting with Debian 6.0, the insserv command is used instead as 
        below instruction.

          # insserv /etc/init.d/rc.mxser

      c. For SuSE
        Run the following command for setting rc files.
        
        # cd /moxa/mxser/driver
        # cp ./rc.mxser /etc/rc.d/
        # cd /etc/rc.d

	Check "boot.local" is existed or not. If "boot.local" doesn't exist, 
        create it by vi, run "chmod 755 boot.local" to change the permission.
        Add "/etc/rc.d/rc.mxser" in last line, 

        Reboot and check if mxser or mxupcie activated by "lsmod" command.

        3.4.6. Automatically modify the port settings on boot
      In the rc files (see "3.4.5" section), it can modify the port settings
      automatically on the boot time by adding the configuration scripts into
      the rc files.

      For example:

        To configure the interface as RS-485 2 wire and the 120ohm terminator
        resistor for ttyMUE0 when system startup, the following scripts can be
        written into the rc files.

          muestty -i RS4852W /dev/ttyMUE0
          muestty -t 120TERM /dev/ttyMUE0

        To see the usage of muestty, please check Chapter 4.

       3.4.7. If you'd like to drive Smartio/Industio ISA boards in the system, 
          you'll have to add parameter to specify CAP address of given 
      board while activating "mxser.o". The format for parameters is
      as follows.

      Step 1: Unload the driver, if the driver is activating.
        Run "lsmod" to check if "mxser" is activated or not.

        #lsmod | grep mxser

        If the driver is activating, unload it.

        #rmmod mxser

      Step 2: Activate the driver with parameters.

        #modprobe mxser ioaddr=0x???,0x???,0x???,0x???
                                 |     |     |     |
                                 |     |     |     +- 4th ISA board
                                 |     |     +------ 3rd ISA board
                                 |     +------------ 2nd ISA board
                                 +------------------- 1st ISA board

      The MUE series multiport board provides two options to set
      the interface and terminator resistor while loading the driver.
      The two options are available while working in RS-422 and
      RS-485 mode. Option's value is applied to all ports on the
      devices.

      Step 1: Unload the driver, if the driver is activating.
        Run "lsmod" to check if "mxupcie" is activated or not.

        #lsmod | grep mxupcie

        If the driver is activating, unload it.

        #rmmod mxupcie

      Step 2: Activate the driver with parameters.

        #modprobe mxupcie interface=2 terminator=1
                          |           |
                          |           +- 120 ohm
                          +-------------- RS-422

      The interface and terminator have values to set as the following.
      
      Option    Value   Comment

      interface      1  RS-232
                     2  RS-422
                     4  RS-485 2 wire
                     8  RS-485 4 wire
    
      terminator     0      0 ohm
                     1    120 ohm 

      Note: You should unload the MOXA driver, before using modprobe
            command to activate it.

       3.4.8 Unload the MOXA driver
      # rmmod mxser
        or
      # rmmod mxupcie

      will deactivate the module driver. You may run "lsmod" to check
      if "mxser" (or "mxupcie") is activated or not.

       3.4.9 Clean the MOXA driver and utilities
      Clean the MOXA driver and utilities in /moxa/mxser, you have
      to execute the command below to clean the files.
    
      # make clean

      3.4.10 Uninstall the MOXA driver and utilities
      The MOXA driver and utilities will be removed from the system
      respectively after executing the command below in /moxa/mxser.

      # make uninstall
      

   3.5 Static driver configuration
       
       Note: To use static driver, you must install the Linux kernel
             source package.
   
       3.5.1 Backup the built-in driver in the kernel.
          # cd /usr/src/linux/drivers/char
          # mv mxser.c mxser.c.old
       
       3.5.2 Create link
      # cd /usr/src/linux/drivers/char
      # ln -s /moxa/mxser/driver/mxser.c mxser.c
      # ln -s /moxa/mxser/driver/mxser.h mxser.h
      # ln -s /moxa/mxser/driver/mxpcie.c mxupcie.c
      # ln -s /moxa/mxser/driver/mxpcie.h mxupcie.h

       3.5.3 Modify kernel configuration file.
          Add the following lines into the configuration file.

      Please modify the Kconfig file.
      /usr/src/<kernel-source directory>/drivers/char/Kconfig

      ...
      config MOXA_INTELLIO
      ...
      config MOXA_SMARTIO
      ...
      config MOXA_SMARTIO_MUE         <-- Add the
         tristate "Moxa SmartIO MUE support" <-- three
         depends on SERIAL_NONSTANDARD     <-- lines in here.
      ...


       3.5.4 Modify the kernel Makefile 
      Add the following line to the last line of Makefile.

      /usr/src/<kernel-source directory>/drviers/char/Makefile

      ...
      ...
      obj-$(CONFIG_MOXA_SMARTIO)     += mxser.o
      obj-$(CONFIG_MOXA_SMARTIO_MUE) += mxupcie.o <-- Add the line.
      ...

       3.5.5 Add CAP address, Interrupt Vector Address and IRQ list for 
       ISA boards. For PCI boards user, please skip this step.
          
      In module mode, the CAP address, Interrupt Vector Address and IRQ for 
      ISA board are given by parameters. 
      In static driver configuration, you'll have to assign it withini the 
      driver's source code. If you will not install any ISA boards, you may
      skip to next portion.
      The instructions to modify driver source code are as below.
      a. # cd /moxa/mxser/driver
         # vi mxser.c

      b. Find the array mxserBoardCAP[] as below.

         static int mxserBoardCAP[]
         ={0,0,0,0};
         static int mxserBoardIRQ[]
         ={0,0,0,0}
         static int mxserBoardVECT[]
         ={0,0,0,0}

      c. Change the address within this array using vi. For
         example, to driver 2 ISA boards with CAP address, vector, IRQ to
         (0x180, 0x1c0, 10) and (0x280,0x2c0, 11). Just to change
         the source code as follows.

         static int mxserBoardCAP[]
         = {0x180, 0x280, 0x00, 0x00};
         static int mxserBoardIRQ[]
         = {10, 11, 0x00, 0x00};
         static int mxserBoardVECT[]
         = {0x1c0, 0x2c0, 0x00, 0x00};
        

       3.5.6 Setup kernel configuration
          
          Configure the kernel:
          
            # cd /usr/src/linux
            # make menuconfig
            
          You will go into a menu-driven system. Please select [Character
          devices][Non-standard serial port support], enable the [Moxa 
          SmartIO support] (or [Moxa SmartIO MUE support]) driver with
      "[*]" for built-in (not "[M]"), then select [Exit] to exit this
      program. 
          
       3.5.7 Rebuild kernel
      The following are for Linux kernel rebuilding, for your 
          reference only.
      For appropriate details, please refer to the Linux document.

        a. cd /usr/src/linux
        b. make clean        /* take a few minutes
        d. make            /* take probably 10-20 minutes
        e. make modules_install    /* take a few minutes
        f. make install        /* copy boot image to correct position


      Please make sure the boot kernel (vmlinuz) is in the
      correct position. 

      If you use 'lilo' utility, you should check /etc/lilo.conf 
      'image' item specified the path which is the 'vmlinuz' path, 
      or you will load wrong (or old) boot kernel image (vmlinuz).
      After checking /etc/lilo.conf, please run "lilo".

      Note that if the result of "make bzImage" is ERROR, then you have to
      go back to Linux configuration Setup. Type "make menuconfig" in 
          directory /usr/src/linux.


       3.5.8 Make tty device and special file
          # cd /moxa/mxser/driver
          # ./msmknod
          
       3.5.9 Install the utilities
      There are four utilities to use with MOXA product.
      Please refer to section 4 for more detail.

      Install the utilities to system directory.
      # cd /moxa/mxser/utility
      # make install

       3.5.10 Reboot

   
   3.6 Custom configuration
       Although this driver already provides you default configuration, you
       still can change the device name and major number. The instruction to
       change these parameters are shown below.

       Change Device Name
       ------------------
       If you'd like to use other device names instead of the default naming
       the convention, all you have to do is to modify the internal code
       within the shell script "msmknod". First, you have to open "msmknod"
       by vi. Locate each line contains "ttyM" (or "ttyMUE") and "cum"
       (or"cumue") and change them to the device name you desired. "msmknod"
       creates the device names you need next time executed.

       Change Major Number
       -------------------
       If major number 30 and 31 had been occupied, you may have to select
       2 free major numbers for this driver. There are 3 steps to change
       major numbers.

       3.6.1 Find free major numbers
      In /proc/devices, you may find all the major numbers occupied
      in the system. Please select 2 major numbers that are available.
      e.g. 40, 45.

       3.6.2 Create special files
      Run /moxa/mxser/driver/msmknod to create special files with
      specified major numbers.

       3.6.3 Modify driver with new major number
      Run vi to open /moxa/mxser/driver/mxser.c. Locate the line
      contains "MXSERMAJOR". Change the content as below.
      #define      MXSERMAJOR          40
      #define      MXSERCUMAJOR          45

       3.6.4 Run "make clean; make install" in /moxa/mxser/driver.

   3.7 Verify driver installation
       You may refer to /var/log/messages to check the latest status
       log reported by this driver whenever it's activated.
       
-----------------------------------------------------------------------------
4. Utilities
   There are 4 utilities contained in this driver. They are msdiag, msmon, 
   msterm and muestty. These 4 utilities are released in the form of source
   code. They should be compiled into an executable file and copied into
   /usr/bin.

   +------------------------------------------------------------------------+
   | Note: “msmon” and “msterm” which will not be compiled in ARM system    |
   |       due to not being supported.                                      |
   +------------------------------------------------------------------------+

   Before using these utilities, please load driver (refer 3.4 & 3.5) and
   make sure you had run the "msmknod" utility.

   msdiag - Diagnostic
   --------------------
   This utility provides the function to display what Moxa Smartio/Industio 
   the board found by the driver in the system.

   msmon - Port Monitoring
   -----------------------
   This utility gives the user a quick view about all the MOXA ports'
   activities. One can easily learn each port's total received/transmitted
   (Rx/Tx) character count since the time when the monitoring is started.
   Rx/Tx throughputs per second are also reported in an interval basis (e.g.
   the last 5 seconds) and on average basis (since the time the monitoring
   is started). You can reset all ports' count by <HOME> key. <+> <->
   (plus/minus) keys to change the displaying time interval. Press <ENTER>
   on the port, that cursor stays, to view the port's communication
   parameters, signal status, and input/output queue.

   msterm - Terminal Emulation
   ---------------------------
   This utility provides data sending and receiving the ability of all tty 
   ports, especially for MOXA ports. It is quite useful for testing simple
   application, for example, sending AT command to a modem connected to the
   port or used as a terminal for login purpose. Note that this is only a
   dumb terminal emulation without handling full-screen operation.

   muestty - Port Configuration Tool
   ---------------------------------
   The utility provides options to set and get the interface and terminator
   resistor for the MUE series.

    Usage: muestty <operation> device_node
           muestty -h | -v
           muestty -v enable board_number
   
    device: The MUE series device node.

    operation:  -h                     Help
                -g                     Get the following information
                                       a) interface type
                                       b) terminator resistor
                                       c) pull high/low resistor
                -i intf                Set interface type with options below
                -t value               Set terminator resistor
                -p state               Set pull high/low resistor
                -a baud                Auto tune and display the proper resistor on
                                       RS-485 2W bus under specified baud rate
                -d baud                Diagnose and display the error status when
                                       negotiating on RS-485 2W bus under specified baud rate
                -v                     Show MOXA MUE series device board slot number and VM-Compatible information
                -v enable board_number Enable/Disable VM-Compatible with board number(only available for specific model)

    intf        RS232     RS-232 mode
                RS422     RS-422 mode
                RS4852W   RS-485 2 wire mode
                RS4854W   RS-485 4 wire mode

    value       NONTERM   None terminator resistor
                120TERM   120ohm terminator resistor

    state       150K      disable pull high/low resistor (150K ohm)
                1K        enable Pull high/low resistor (1K ohm)

    baud        921600    Baud rate = 921600
                460800    Baud rate = 460800
                230400    Baud rate = 230400
                115200    Baud rate = 115200
                 57600    Baud rate = 57600
                 38400    Baud rate = 38400
                 19200    Baud rate = 19200
                  9600    Baud rate = 9600

    enable      1         Enable VM-Compatible
                0         Disable VM-Compatible

    board_number          Get this information by the command "muestty -v"

   For example:
    
    To set the interface
    # muestty -i RS422 /dev/ttyMUE1

    To set the terminator resistor
    # muestty -t 120TERM /dev/ttyMUE1
   
    To diagnose whether the setting is correct or not 
    # muestty -d 115200 /dev/ttyMUE1
    
        Note: If Alarm Status shows Fail, then there are problems with the 
        setting. Do the auto-tuning process with the following command.
    
    To run the auto-tuning process and get the proper resistor values
    # muestty -a 115200 /dev/ttyMUE1 

    To enable specific device running in VM
    Step 1. Get the device board_number
    # muestty -v

    Step 2. Enable VM-Compatible with board_number
    # muestty -v 1 1

        Note: These steps must be set in physical computer.
   
-----------------------------------------------------------------------------
5. Setserial

   Supported Setserial parameters are listed as below.

   uart          set UART type(16450-->disable FIFO, 16550A-->enable FIFO)
   close_delay   set the amount of time(in 1/100 of a second) that DTR
          should be kept low while being closed.
   closing_wait  set the amount of time(in 1/100 of a second) that the
          serial port should wait for data to be drained while
          being closed, before the receiver is disable.
   spd_hi        Use  57.6kb  when  the application requests 38.4kb.
   spd_vhi       Use  115.2kb    when  the application requests 38.4kb.
   spd_shi       Use  230.4kb    when  the application requests 38.4kb.
   spd_warp      Use  460.8kb    when  the application requests 38.4kb.
   spd_normal    Use  38.4kb  when  the application requests 38.4kb.

-----------------------------------------------------------------------------
6. Non-standard baud rate

   To set non-standard baud rate, you can follow the belowe programming guide.

   Programming guide for special baud rate:

   - Define the I/O control code.

         #define  MOXA_SET_SPECIAL_BAUD_RATE  0x44D

   - Using the ioctl() API to set non-standard baud rate after
     setting the termios parameters.

         int  baud;
         baud = 500000;  /* set the non-standard baud rate */
         ...
         tcsetattr(fd,TCSANOW,&t);

         ioctl(fd, MOXA_SET_SPECIAL_BAUD_RATE, &baud);

-----------------------------------------------------------------------------
7. How to sign the driver with a cypher key to support a platform that enables a secure
   boot

   When your company or you have a platform with the secure boot function enabled, you
   may consider to install a driver with a signed cypher. There are two approaches for
   key generation and you will need to sign the driver to install it to the platform:

   1. To apply a public/private key generated by a third-party certificate authority, or
   2. To apply a public/private key generated by your company’s internal server.
   Approach 2 is a less costly and secure way if you use the pair of keys internally 
   (within the company). If you will use the key on the Internet, Moxa recommends the 
   Approach 1.

   The following steps are the guidance of signing the driver:

   1. Create a configuration file for generating the key pair
   2. Enrolling the public key
   3. Signing the module
   4. Loading the signed module

   7.1 Create a configuration file for generating the key pair

   x509.genkey
   [ req ]
   default_bits = 4096
   distinguished_name = req_distinguished_name
   prompt = no
   string_mask = utf8only
   x509_extensions = myexts

   [ req_distinguished_name ]
   #O = Unspecified company
   CN = Build time autogenerated kernel key
   #emailAddress = unspecified.user@unspecified.company

   [ myexts ]
   basicConstraints=critical,CA:FALSE
   keyUsage=digitalSignature
   subjectKeyIdentifier=hash
   authorityKeyIdentifier=keyid

   • Use the following command to generate an X.509 key pair.
   # openssl req -x509 -new -nodes -utf8 -sha256 -days 36500 -batch -config x509.genkey 
   -outform DER -out my_signing_key_pub.der -keyout my_signing_key.priv

   The keypair will be stored as my_signing_key_pub.der and my_signing_key.priv.
   my_signing_key_pub.der is the public key file.
   my_signing_key.priv is the private key file.
   The two files will be used when signing the module.

   7.2 Enrolling the public key
   • Use the following command to enroll the public key
   # mokutil --import my_signing_key_pub.der
   You will be asked to enter a password that will be used in MokManager later.

   • Reboot the machine
   When the system reboot, the MokManager will be loaded.

   • Choose “Eroll MOK”
   MokManager will ask you enter the password you typed in earlier when running mokutil.

   7.3 Signing the module
   Using the following command to sign the module.
   # /usr/src/linux-headers-$(uname -r)/scripts/sign-file sha256 my_signing_key.priv 
   my_signing_key_pub.der mxpcie.ko
   or
   # /usr/src/linux-headers-$(uname -r)/scripts/sign-file sha256 my_signing_key.priv 
   my_signing_key_pub.der mxser.ko

   You can validate that the module is signed by checking that it includes the string 
   “~Module signature appended~” by the following command.

   # hexdump -Cv my_module.ko | tail -n 5
   00010c30  f8 61 31 0e 53 39 2c 8b  91 b2 98 63 d1 dc 00 00  |.a1.S9,....c....|
   00010c40  02 00 00 00 00 00 00 00  02 9e 7e 4d 6f 64 75 6c  |..........~Modul|
   00010c50  65 20 73 69 67 6e 61 74  75 72 65 20 61 70 70 65  |e signature appe|
   00010c60  6e 64 65 64 7e 0a                                 |nded~.|
   00010c66

   7.4 Loading the signed module
   • Copy module to kernel
   After signing the module, you can copy the module to the kernel module directory that 
   you want.
   For example:
   # cp my_module.ko /lib/modules/$(uname -r)/kernel/drivers/char/
   • Update the modular dependency list
   # depmod -a
   • Load the kernel module
   # modprobe mxupcie
   or
   # modprobe mxser

-----------------------------------------------------------------------------
8. Troubleshooting

   8.1 Error messages
   The boot time error messages and solutions are stated as clearly as
   possible. If all the possible solutions fail, please contact our technical
   support team to get more help.

   Error msg: More than 4 Moxa Smartio/Industio family boards found. Fifth 
              board and after are ignored.
   Solution:
   To avoid this problem, please unplug fifth and after board, because of Moxa
   driver supports up to 4 boards.

   Error msg: Request_irq fail, IRQ(?) may be conflict with another device.
   Solution:
   Other PCI or ISA devices occupy the assigned IRQ. If you are not sure
   which device causes the situation, please check /proc/interrupts to find
   free IRQ and simply change another free IRQ for Moxa board.

   Error msg: Board #: C1xx Series(CAP=xxx) interrupt number invalid.
   Solution:
   Each port within the same multiport board shares the same IRQ. Please set
   one IRQ (IRQ doesn't equal to zero) for one Moxa board.

   Error msg: No interrupt vector be set for Moxa ISA board(CAP=xxx).
   Solution:
   Moxa ISA board needs an interrupt vector. Please refer to user's manual
   "Hardware Installation" chapter to set the interrupt vector.

   Error msg: Couldn't install MOXA Smartio/Industio family driver!
   Solution:
   Load Moxa driver fails, the major number may conflict with other devices.
   Please refer to previous section 3.7 to change a free major number for
   Moxa driver.

   Error msg: Couldn't install MOXA Smartio/Industio family callout driver!
   Solution:
   Load Moxa callout driver fail, the callout device major number may
   conflict with other devices. Please refer to previous section 3.7 to
   change a free callout device major number for Moxa driver.
   
   8.2 Baud rate mismatch when using MUE series PCI Express multiport board.

   This is a problem of the build-in kernel module or driver. 
   Please run the script in the following file path.

    mxser/driver/moxa_unbind

   The script will unbind build-in module or driver, and reactive MOXA device
   driver mxupcie. You can follow below steps to run the script automatically 
   on your system boot.

    Step 1. Using crontab for scheduling the job.

            # crontab -e

    Step 2. Type the following command on the last line

            @reboot sh <THE_FULL_PATH_OF_THIS_DRIVER>/mxser/driver/moxa_unbind

Please follow below steps
   to remove build-in module.

    Step 1. Create /etc/modprobe.d/blacklist.conf if it is not exit.
    Step 2. Add following line into the file
            blacklist 8250_moxa
    Step 3. Reboot the system
    Step 4. Reload the mxupcie module again.
   
   8.3 /dev/ppp device always try to reconnect while dail-in/dail-out

   This is a problem of the PCI device latency. Please follow below steps 
   to reduce latency by disabling FIFO.

    # setserial /dev/ttyMx uart 16450

   You can refer to the "5. Setserial" for more detail about how to 
   enable/disable FIFO setting.

   8.4 MOXA PCIE device can't read/write data in virtual machine.

   Please follow below steps to enable MOXA PCIE device VM-Compatible.
   
   Note: The following steps MUST be configure in physical computer with
         linux operating system. If you execute these commands in virtual
         machine, these configurations may not work correctly.

    Step 1. Use muestty, the utility contain in this driver, to get the 
            device board_number

    # muestty -v

    Step 2. Enable VM-Compatible with board_number

    # muestty -v 1 <board_number>

-----------------------------------------------------------------------------
