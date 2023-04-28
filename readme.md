#### README for neorv32-de0n-fatfs
This project is a port of an [older](https://www.emb4fun.de/fpga/fatfs/index.html) FPGA project of mine
where a different CPU was used. Since the DE0-Nano board does not have an SD card holder, an external
card holder was connected to the board with jumper wire. Here the following [card holder](https://www.waveshare.com/wiki/Micro_SD_Storage_Board)
from Waveshare was used. A microSD SanDisk Ultra 16GB was used as the SD card.

The output on the terminal should look like this, whereby the performance values can vary:

<<< FatFs filesystem benchmark >>>

Benchmark started ...

  15541728 KiB total drive space.<br>
  15537488 KiB available.

Simple write/read test... OK

Performance write/read:<br>
Write:  0.93 MB/s<br>
Read :  1.40 MB/s

Benchmark end.

SEGGER Embedded Studio for RISC-V was used as the development environment.


#### Project information:

| Board    | [Terasic DE0-Nano](https://www.terasic.com.tw/cgi-bin/page/archive.pl?Language=English&CategoryNo=139&No=593) |
| :------- | :------------- |
| FPGA     | Cyclone IV `EP4CE22F17C6N` |
| Quartus  | 15.0.2         |
| clk_i    | 100 MHz        |
| Terminal | 115200, 8, N, 1 |

