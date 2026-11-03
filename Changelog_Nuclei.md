# Nuclei OpenOCD Distribution

For **Nuclei OpenOCD documentation**, please check https://doc.nucleisys.com/nuclei_tools/openocd/intro.html

If the tool version and documentation version not match, please take care.

**Below is the changelog for Nuclei OpenOCD.**

## 2026.10

This release is based on **2025.10** version with some new features and bug fixes introduced.

* Add `nuclei trace` command group for trace-bundle instruction-trace and data-trace support
* Update `nuclei etrace` command group for ATB2AXI instruction-trace support
* Update `nuclei cpuinfo` command implementation using nuclei-sdk same source code to provide same behavior
* Sync nuclei custom CSR support for eclic_v2
* Modify N100 `vlenb`/`mtopi`/`mtopei` display
* Add flash support for HG25Q16B, GD25LE16E, GD25LQ128E, MX25R8035F, XTD25W02A, XTD25W04A, GT25Q16A, GT25Q32EB, GT25Q64EB
* Add dummy flash driver to support automatic hardware breakpoint placement in specified address ranges
* Update custom flash driver with `wrsize` support, flash-id based device matching, and fix memory issues in custom.c and nuspi.c
* Fix FTD2xx vendor IDs and product IDs to resolve matching issues when multiple FTDI devices are present
* Modify GD32VF103 IDCODE to correctly display chip information on the target

## 2025.10

This release is based on the same code base of **2025.02**, but do some new feature and bugfixes.

* Update ``nuclei cpuinfo`` command implementation using nuclei sdk 0.9.0 cpuinfo same source code to provide same behavior
* Correct custom flashloader binary file location search rules, where openocd executed, openocd ``scripts`` folder, and absolute path
* Sync nuclei custom CSRs support
* Optimize flash probe process, when no flash probed, it will exit with error instead of hang on it
* Support debug for hypervisor mode
* Fix debug ``Assertion failed!`` on HBird opensource cpu
* Fix nuclei riscv cross trigger command implementation ``nuclei cti xxx``
* Clear ``haltreq`` signal when openocd connect failed
* Fix segmentation fault issue when debug level set to 3 or 4
* When debug on FPGA CPU MHz is very slow such as **5MHz**, openocd may work not correctly, it will be fixed by make CPU run faster such as **10MHz**

## 2025.02

* improvement of ci and doc
* replace ``vslide1down_vx`` to read/write riscv vector register
* support live watch feature via riscv sba memory access channel
* organize nuclei commands into ``nuclei_riscv.c``
* add ``nuclei cti`` command group
* update ``nuclei etrace`` command group
* optimize ``nuclei cpuinfo`` command

* add new nuclei customized csrs

    | new csr addr | new csr name |
    | ------ | ------ |
    | 0x1a4~0x1af | smpuaddr4~15 |
    | 0x1c0~0x1ef | smpuaddr16~63 |

* rename nuclei customized csrs

    | old name | new name |
    | ------ | ------ |
    | spmpcfg0~3 | smpucfg0~3 |
    | spmpaddr0~15 | smpuaddr0~15 |
    | mfp16mode | mmisc_ctl1 |
    | mecc_ctrl | mecc_ctl |
    | mstack_ctrl | mstack_ctl |

* changes are based on [riscv/riscv-openocd@f82c5a7](https://github.com/riscv-collab/riscv-openocd/commit/f82c5a7)


## 2024.06

* Add and update nuclei custom CSR
* Fix `nuclei cpuinfo` command implementation
* Add debug map feature for Nuclei CPU with new `nuclei expose_cpu_core` and `nuclei examine_cpu_core` commands
* changes are based on [riscv/riscv-openocd](https://github.com/riscv/riscv-openocd/commit/52177592)
* Fix hbird/hbirdv2 flash programming, and re-route fespi to nuspi flash loader for hbird/hbirdv2 processor

## 2024.02

> **Still in development, not official release.**

* Add Nuclei N100 CSR support
* Nuclei etrace support multicore trace
* Fix riscv debug 0.11 call riscv_run_algorithm error such as hbird/hbirdv2 processor
* Update and fix openocd documentation
* changes are based on [riscv/riscv-openocd](https://github.com/riscv/riscv-openocd/commit/52177592)

## 2023.10

* Add nuclei command group to contains all nuclei customized commands
* Add `nuclei cpuinfo` dump support
* Add more spiflash devices according to customer request
* Fix gdb flash program error on address size > 32bit on windows
* Update nuclei custom csr
* Add `ftdi nscan1_mode` to support 2-wire cjtag for Nuclei CPU, which is replacement for `ftdi oscan1_mode`
* Add nuclei custom command in openocd documentation
* **Experimental and may change**: Add nuclei etrace command to support nuclei etrace hardware feature
* changes are based on [riscv/riscv-openocd](https://github.com/riscv/riscv-openocd/commit/52177592)

## 2022.12

This is release 2022.12 of openocd.

* nor/spi:add w25q512jv
* add 'init resethalt' command
* feature:auto search custom flashloader path
* fix riscv-debug v0.11 call riscv_run_algorithm error.


## 2022.08

This is release 2022.08 of openocd.

* spi_nor:add xt25f256b mac25l1633e gd25q80b gd25le32e en25s40a by25q32al fm25q128 gd25B512me.
* merge upstream https://github.com/riscv/riscv-openocd commit id 52177592f9d3afc6a008f8e1b321cf74e823018f.
* custom flashloader don't rely on 'src/flash/nor/spi.c', fix memory leaks bug.


## 2022.04

This is release 2022.04 of openocd.

* add DSP ucode csr register
* Adjustment simulation timeout param
* Fix after write_bank/write_image command read date error bug
* Add custom flash loader in openocd, please refer wiki for how to use it
* Custom flash loader: add 'simulation' parameter for simulation test.


## 2022.01

This is release 2022.01 of openocd.

* optimize cjtag support for nuclei cjtag
* spi_nor: add BoHong bh25d80a bh25d40a bh25d20a
* spi_nor: Add Micron MT25QU512
* flash:"flash bank" command add simulation param
* add nuclei all custom csr
* spi_nor: Add MXIC MX25U51245G
* transport/ftdi: Update to new standard cJTAG sequence
* flash: add XinSheng RISC-V MCU CM32M4xxR flash program driver
* Add nuspi loader support.
* Add nuspi SPI flash driver support.
* Enable multi-core debug.
* Previous fespi loader will be rerouted to nuspi loader.
* Changes are based on [openocd for riscv 0.11.0](https://github.com/riscv/riscv-openocd/commit/6edf98db7f98c5e24bc51cf98419bdf5bbc530e6)
