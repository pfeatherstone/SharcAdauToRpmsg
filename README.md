# Synopsis #

This project includes Bare Metal code for SHARC1 and SHARC2 of an adsp-sc589-mini board.
The ARM is flashed with Yocto Linux using the ["Getting Started" guide](https://github.com/analogdevicesinc/lnxdsp-adi-meta/wiki/Getting-Started-with-ADSP%E2%80%90SC589%E2%80%90MINI-(Linux-for-ADSP%E2%80%90SC5xx-Processors-3.1.1)).

The aim of the project is simple:
* SHARC1 reads audio samples from Adau1761 and sends them over RPMsg
* SHARC2 is idle 
* The ARM reads audio from RPMsg and logs.

Notes:
* The ARM, which is running Yocto Linux, uses code from [rpmsg-utils](https://github.com/analogdevicesinc/rpmsg-utils/blob/develop/yocto-3.1.0/rpmsg-bind-chardev.c) to bind the RPMsg channels to file descriptors which are then managed using the C++ [Asio](https://www.boost.org/doc/libs/1_85_0/doc/html/boost_asio.html) library.
* SHARC1 uses [rpmsg-lite](https://github.com/analogdevicesinc/rpmsg-lite) as the bare-metal implementation.
* A very small protocol is wrapped around RPMsg so there are no unsolicited writes from the SHARC to the ARM. If you don't do this, the linux kernel gets flooded with virtio messages which fill journalctl and spins the mini's CPU at 100%.
