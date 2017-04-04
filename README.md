# mxser

This repository contains a Debianized version of the official [MOXA Smartio/Industio Linux serial driver](http://www.moxa.com/support/download.aspx?type=support&id=1122).

While some in-kernel drivers exist, this version is preferred because it is currently the only one that fully supports card configuration (such as selectible termination and cards that can switch between 232/422/485).

It is mostly intended for private use and is not expected to get picked up by the official Debian repository, partly because I'm a newbie maintainer and have probably violated policies but also because the upstream package contains some prebuilt binaries without source and is thus only buildable on limited architectures.  However it has been published for general use in the hope that others will find it useful.  (And perhaps to learn some tricks from any experienced maintainers who spot something I did incorrectly.)

Some of the packaging metadata assumes you're compiling it for kernel 4.9 (and for Debian Stretch), but this should be trivially editable for any other version -- if there's some magic way to make this pick up the correct version automatically, then please let me know.

## Packages

* **mxser-dkms**
    This is a DKMS-compatible package containing driver source code, such that the corresponding kernel modules `mxser` and `mxupcie` can be built for (hopefully) any arbitrary Linux kernel, provided that the corresponding **linux-headers** package is installed (or full source is) along with a suitable compiler.

    Simply installing this package is usually sufficient to compile and install the drivers for your current kernel.

* **mxser-tools**
    This contains the userland tools from the official driver package:

    * `muestty`
        Device configuration tool for MUE series PCI Express multiport boards.

    * `msdiag`
        Diagnostic program for displaying installed Moxa Smartio/Industio boards.

    * `msmon`
        Monitor program to observe data count and line status signals.

    * `msterm`
        A simple terminal program which is useful in testing serial ports.

More information can be found in the readme.txt file from the original vendor package, which will also be installed to `/usr/share/doc/mxser-tools/`.

## Installing from a Release

I've uploaded some pre-built packages in the Releases tab, to make life easier.  Just download them and `dpkg -i` them as discussed further below.  Of course, this assumes that you're using a compatible system (currently Debian Stretch).

## Installing from Source

The below assumes you have `git-buildpackage` installed.  Without that you can still build this but you'll need different commands.

    gbp clone [--pristine-tar] https://github.com/uecasm/mxser.git
    cd mxser
    gbp buildpackage -us -uc

This should create the appropriate `.deb` packages in `../build-area/`.  Currently it will also print some `lintian` errors (some of which I have no intention of fixing due to upstream limitations), but it should at least produce the packages.

(If you don't want to use `git-buildpackage`, then you should just be able to `git clone` the repository and then `debuild` or `dpkg-buildpackage` it instead, but you'll need to extract the pristine tarball first.)

You can use `dpkg -i` to install the packages, as usual.

## Installing modules on an embedded system

If you want to install the kernel modules onto a system that does not have compilers installed, then you'll first need to install the dkms module onto a compatible machine or chroot (ideally a clone, but mainly just the same architecture and kernel) that *does* have the compilers installed.

Once the modules have been built (and for the same kernel as on the target machine) you can use the following command to export a package containing only the compiled modules that can be installed on a machine without compilers:

    dkms mkbmdeb mxser/1.16+14030317 --all

(Or specify a specific kernel version to export; see the DKMS docs for more information.)
