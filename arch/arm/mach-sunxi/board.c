// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2012 Henrik Nordstrom <henrik@henriknordstrom.net>
 *
 * (C) Copyright 2007-2011
 * Allwinner Technology Co., Ltd. <www.allwinnertech.com>
 * Tom Cubie <tangliang@allwinnertech.com>
 *
 * Some init for sunxi platform.
 */

#include <common.h>
#include <cpu_func.h>
#include <init.h>
#include <log.h>
#include <mmc.h>
#include <i2c.h>
#include <serial.h>
#include <spl.h>
#include <asm/cache.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch/spl.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/timer.h>
#include <asm/arch/tzpc.h>
#include <asm/arch/mmc.h>
#include <asm/arch/prcm.h>

#include <linux/compiler.h>

struct fel_stash {
	uint32_t sp;
	uint32_t lr;
	uint32_t cpsr;
	uint32_t sctlr;
	uint32_t vbar;
	uint32_t cr;
};

struct fel_stash fel_stash __section(".data");

#ifdef CONFIG_ARM64
#include <asm/armv8/mmu.h>

static struct mm_region sunxi_mem_map[] = {
	{
		/* SRAM, MMIO regions */
		.virt = 0x0UL,
		.phys = 0x0UL,
		.size = 0x40000000UL,
		.attrs = PTE_BLOCK_MEMTYPE(MT_DEVICE_NGNRNE) |
			 PTE_BLOCK_NON_SHARE
	}, {
		/* RAM */
		.virt = 0x40000000UL,
		.phys = 0x40000000UL,
		.size = CONFIG_SUNXI_DRAM_MAX_SIZE,
		.attrs = PTE_BLOCK_MEMTYPE(MT_NORMAL) |
			 PTE_BLOCK_INNER_SHARE
	}, {
		/* List terminator */
		0,
	}
};
struct mm_region *mem_map = sunxi_mem_map;

ulong board_get_usable_ram_top(ulong total_size)
{
	/* Some devices (like the EMAC) have a 32-bit DMA limit. */
	if (gd->ram_top > (1ULL << 32))
		return 1ULL << 32;

	return gd->ram_top;
}
#endif

static int gpio_init(void)
{
#if defined(CONFIG_MACH_SUNXI_H3_H5)
	/* enable R_PIO GPIO access */
	prcm_apb0_enable(PRCM_APB0_GATE_PIO);
#endif

	__maybe_unused uint val;
#if CONFIG_CONS_INDEX == 1 && defined(CONFIG_UART0_PORT_F)
#if defined(CONFIG_MACH_SUN4I) || \
    defined(CONFIG_MACH_SUN7I) || \
    defined(CONFIG_MACH_SUN8I_R40)
	/* disable GPB22,23 as uart0 tx,rx to avoid conflict */
	sunxi_gpio_set_cfgpin(SUNXI_GPB(22), SUNXI_GPIO_INPUT);
	sunxi_gpio_set_cfgpin(SUNXI_GPB(23), SUNXI_GPIO_INPUT);
#endif
#if defined(CONFIG_MACH_SUN8I) && !defined(CONFIG_MACH_SUN8I_R40)
	sunxi_gpio_set_cfgpin(SUNXI_GPF(2), SUN8I_GPF_UART0);
	sunxi_gpio_set_cfgpin(SUNXI_GPF(4), SUN8I_GPF_UART0);
#else
	sunxi_gpio_set_cfgpin(SUNXI_GPF(2), SUNXI_GPF_UART0);
	sunxi_gpio_set_cfgpin(SUNXI_GPF(4), SUNXI_GPF_UART0);
#endif
	sunxi_gpio_set_pull(SUNXI_GPF(4), 1);
#elif CONFIG_CONS_INDEX == 1 && (defined(CONFIG_MACH_SUN4I) || \
				 defined(CONFIG_MACH_SUN7I) || \
				 defined(CONFIG_MACH_SUN8I_R40))
	sunxi_gpio_set_cfgpin(SUNXI_GPB(22), SUN4I_GPB_UART0);
	sunxi_gpio_set_cfgpin(SUNXI_GPB(23), SUN4I_GPB_UART0);
	sunxi_gpio_set_pull(SUNXI_GPB(23), SUNXI_GPIO_PULL_UP);
#elif CONFIG_CONS_INDEX == 1 && defined(CONFIG_MACH_SUN5I)
	sunxi_gpio_set_cfgpin(SUNXI_GPB(19), SUN5I_GPB_UART0);
	sunxi_gpio_set_cfgpin(SUNXI_GPB(20), SUN5I_GPB_UART0);
	sunxi_gpio_set_pull(SUNXI_GPB(20), SUNXI_GPIO_PULL_UP);
#elif CONFIG_CONS_INDEX == 1 && defined(CONFIG_MACH_SUN6I)
	sunxi_gpio_set_cfgpin(SUNXI_GPH(20), SUN6I_GPH_UART0);
	sunxi_gpio_set_cfgpin(SUNXI_GPH(21), SUN6I_GPH_UART0);
	sunxi_gpio_set_pull(SUNXI_GPH(21), SUNXI_GPIO_PULL_UP);
#elif CONFIG_CONS_INDEX == 1 && defined(CONFIG_MACH_SUN8I_A33)
	sunxi_gpio_set_cfgpin(SUNXI_GPB(0), SUN8I_A33_GPB_UART0);
	sunxi_gpio_set_cfgpin(SUNXI_GPB(1), SUN8I_A33_GPB_UART0);
	sunxi_gpio_set_pull(SUNXI_GPB(1), SUNXI_GPIO_PULL_UP);
#elif CONFIG_CONS_INDEX == 1 && defined(CONFIG_MACH_SUNXI_H3_H5)
	sunxi_gpio_set_cfgpin(SUNXI_GPA(4), SUN8I_H3_GPA_UART0);
	sunxi_gpio_set_cfgpin(SUNXI_GPA(5), SUN8I_H3_GPA_UART0);
	sunxi_gpio_set_pull(SUNXI_GPA(5), SUNXI_GPIO_PULL_UP);
#elif CONFIG_CONS_INDEX == 1 && defined(CONFIG_MACH_SUN50I)
	sunxi_gpio_set_cfgpin(SUNXI_GPB(8), SUN50I_GPB_UART0);
	sunxi_gpio_set_cfgpin(SUNXI_GPB(9), SUN50I_GPB_UART0);
	sunxi_gpio_set_pull(SUNXI_GPB(9), SUNXI_GPIO_PULL_UP);
#elif CONFIG_CONS_INDEX == 1 && defined(CONFIG_MACH_SUN50I_H6)
	sunxi_gpio_set_cfgpin(SUNXI_GPH(0), SUN50I_H6_GPH_UART0);
	sunxi_gpio_set_cfgpin(SUNXI_GPH(1), SUN50I_H6_GPH_UART0);
	sunxi_gpio_set_pull(SUNXI_GPH(1), SUNXI_GPIO_PULL_UP);
#elif CONFIG_CONS_INDEX == 1 && defined(CONFIG_MACH_SUN50I_H616)
	sunxi_gpio_set_cfgpin(SUNXI_GPH(0), SUN50I_H616_GPH_UART0);
	sunxi_gpio_set_cfgpin(SUNXI_GPH(1), SUN50I_H616_GPH_UART0);
	sunxi_gpio_set_pull(SUNXI_GPH(1), SUNXI_GPIO_PULL_UP);
#elif CONFIG_CONS_INDEX == 1 && defined(CONFIG_MACH_SUN8I_A83T)
	sunxi_gpio_set_cfgpin(SUNXI_GPB(9), SUN8I_A83T_GPB_UART0);
	sunxi_gpio_set_cfgpin(SUNXI_GPB(10), SUN8I_A83T_GPB_UART0);
	sunxi_gpio_set_pull(SUNXI_GPB(10), SUNXI_GPIO_PULL_UP);
#elif CONFIG_CONS_INDEX == 1 && defined(CONFIG_MACH_SUN8I_V3S)
	sunxi_gpio_set_cfgpin(SUNXI_GPB(8), SUN8I_V3S_GPB_UART0);
	sunxi_gpio_set_cfgpin(SUNXI_GPB(9), SUN8I_V3S_GPB_UART0);
	sunxi_gpio_set_pull(SUNXI_GPB(9), SUNXI_GPIO_PULL_UP);
#elif CONFIG_CONS_INDEX == 1 && defined(CONFIG_MACH_SUN9I)
	sunxi_gpio_set_cfgpin(SUNXI_GPH(12), SUN9I_GPH_UART0);
	sunxi_gpio_set_cfgpin(SUNXI_GPH(13), SUN9I_GPH_UART0);
	sunxi_gpio_set_pull(SUNXI_GPH(13), SUNXI_GPIO_PULL_UP);
#elif CONFIG_CONS_INDEX == 2 && defined(CONFIG_MACH_SUN5I)
	sunxi_gpio_set_cfgpin(SUNXI_GPG(3), SUN5I_GPG_UART1);
	sunxi_gpio_set_cfgpin(SUNXI_GPG(4), SUN5I_GPG_UART1);
	sunxi_gpio_set_pull(SUNXI_GPG(4), SUNXI_GPIO_PULL_UP);
#elif CONFIG_CONS_INDEX == 3 && defined(CONFIG_MACH_SUN8I) && !defined(CONFIG_MACH_SUNXI_H3_H5)
	sunxi_gpio_set_cfgpin(SUNXI_GPB(0), SUN8I_GPB_UART2);
	sunxi_gpio_set_cfgpin(SUNXI_GPB(1), SUN8I_GPB_UART2);
	sunxi_gpio_set_pull(SUNXI_GPB(1), SUNXI_GPIO_PULL_UP);
#elif CONFIG_CONS_INDEX == 3 && defined(CONFIG_MACH_SUNXI_H3_H5)
	sunxi_gpio_set_cfgpin(SUNXI_GPA(0), SUN8I_H3_GPA_UART2);
	sunxi_gpio_set_cfgpin(SUNXI_GPA(1), SUN8I_H3_GPA_UART2);
	sunxi_gpio_set_pull(SUNXI_GPA(1), SUNXI_GPIO_PULL_UP);
#elif CONFIG_CONS_INDEX == 5 && defined(CONFIG_MACH_SUN8I)
	sunxi_gpio_set_cfgpin(SUNXI_GPL(2), SUN8I_GPL_R_UART);
	sunxi_gpio_set_cfgpin(SUNXI_GPL(3), SUN8I_GPL_R_UART);
	sunxi_gpio_set_pull(SUNXI_GPL(3), SUNXI_GPIO_PULL_UP);
#elif CONFIG_CONS_INDEX == 2 && defined(CONFIG_MACH_SUN8I) && \
				!defined(CONFIG_MACH_SUN8I_R40)
	sunxi_gpio_set_cfgpin(SUNXI_GPG(6), SUN8I_GPG_UART1);
	sunxi_gpio_set_cfgpin(SUNXI_GPG(7), SUN8I_GPG_UART1);
	sunxi_gpio_set_pull(SUNXI_GPG(7), SUNXI_GPIO_PULL_UP);
#else
#error Unsupported console port number. Please fix pin mux settings in board.c
#endif

#ifdef CONFIG_SUN50I_GEN_H6
	/* Update PIO power bias configuration by copy hardware detected value */
	val = readl(SUNXI_PIO_BASE + SUN50I_H6_GPIO_POW_MOD_VAL);
	writel(val, SUNXI_PIO_BASE + SUN50I_H6_GPIO_POW_MOD_SEL);
	val = readl(SUNXI_R_PIO_BASE + SUN50I_H6_GPIO_POW_MOD_VAL);
	writel(val, SUNXI_R_PIO_BASE + SUN50I_H6_GPIO_POW_MOD_SEL);
#endif

	return 0;
}

#if defined(CONFIG_SPL_BOARD_LOAD_IMAGE) && defined(CONFIG_SPL_BUILD)
static int spl_board_load_image(struct spl_image_info *spl_image,
				struct spl_boot_device *bootdev)
{
	debug("Returning to FEL sp=%x, lr=%x\n", fel_stash.sp, fel_stash.lr);
	return_to_fel(fel_stash.sp, fel_stash.lr);

	return 0;
}
SPL_LOAD_IMAGE_METHOD("FEL", 0, BOOT_DEVICE_BOARD, spl_board_load_image);
#endif

void s_init(void)
{
	/*
	 * Undocumented magic taken from boot0, without this DRAM
	 * access gets messed up (seems cache related).
	 * The boot0 sources describe this as: "config ema for cache sram"
	 */
#if defined CONFIG_MACH_SUN6I
	setbits_le32(SUNXI_SRAMC_BASE + 0x44, 0x1800);
#elif defined CONFIG_MACH_SUN8I
	__maybe_unused uint version;

	/* Unlock sram version info reg, read it, relock */
	setbits_le32(SUNXI_SRAMC_BASE + 0x24, (1 << 15));
	version = readl(SUNXI_SRAMC_BASE + 0x24) >> 16;
	clrbits_le32(SUNXI_SRAMC_BASE + 0x24, (1 << 15));

	/*
	 * Ideally this would be a switch case, but we do not know exactly
	 * which versions there are and which version needs which settings,
	 * so reproduce the per SoC code from the BSP.
	 */
#if defined CONFIG_MACH_SUN8I_A23
	if (version == 0x1650)
		setbits_le32(SUNXI_SRAMC_BASE + 0x44, 0x1800);
	else /* 0x1661 ? */
		setbits_le32(SUNXI_SRAMC_BASE + 0x44, 0xc0);
#elif defined CONFIG_MACH_SUN8I_A33
	if (version != 0x1667)
		setbits_le32(SUNXI_SRAMC_BASE + 0x44, 0xc0);
#endif
	/* A83T BSP never modifies SUNXI_SRAMC_BASE + 0x44 */
	/* No H3 BSP, boot0 seems to not modify SUNXI_SRAMC_BASE + 0x44 */
#endif

#if !defined(CONFIG_ARM_CORTEX_CPU_IS_UP) && !defined(CONFIG_ARM64)
	/* Enable SMP mode for CPU0, by setting bit 6 of Auxiliary Ctl reg */
	asm volatile(
		"mrc p15, 0, r0, c1, c0, 1\n"
		"orr r0, r0, #1 << 6\n"
		"mcr p15, 0, r0, c1, c0, 1\n"
		::: "r0");
#endif
#if defined CONFIG_MACH_SUN6I || defined CONFIG_MACH_SUN8I_H3
	/* Enable non-secure access to some peripherals */
	tzpc_init();
#endif

	clock_init();
	timer_init();
	gpio_init();
#if !CONFIG_IS_ENABLED(DM_I2C)
	i2c_init_board();
#endif
	eth_init_board();
}

#define SUNXI_INVALID_BOOT_SOURCE	-1

static int sunxi_get_boot_source(void)
{
	if (!is_boot0_magic(SPL_ADDR + 4)) /* eGON.BT0 */
		return SUNXI_INVALID_BOOT_SOURCE;

	return readb(SPL_ADDR + 0x28);
}

/* The sunxi internal brom will try to loader external bootloader
 * from mmc0, nand flash, mmc2.
 */
uint32_t sunxi_get_boot_device(void)
{
	int boot_source = sunxi_get_boot_source();

	/*
	 * When booting from the SD card or NAND memory, the "eGON.BT0"
	 * signature is expected to be found in memory at the address 0x0004
	 * (see the "mksunxiboot" tool, which generates this header).
	 *
	 * When booting in the FEL mode over USB, this signature is patched in
	 * memory and replaced with something else by the 'fel' tool. This other
	 * signature is selected in such a way, that it can't be present in a
	 * valid bootable SD card image (because the BROM would refuse to
	 * execute the SPL in this case).
	 *
	 * This checks for the signature and if it is not found returns to
	 * the FEL code in the BROM to wait and receive the main u-boot
	 * binary over USB. If it is found, it determines where SPL was
	 * read from.
	 */
	switch (boot_source) {
	case SUNXI_INVALID_BOOT_SOURCE:
		return BOOT_DEVICE_BOARD;
	case SUNXI_BOOTED_FROM_MMC0:
	case SUNXI_BOOTED_FROM_MMC0_HIGH:
		return BOOT_DEVICE_MMC1;
	case SUNXI_BOOTED_FROM_NAND:
		return BOOT_DEVICE_NAND;
	case SUNXI_BOOTED_FROM_MMC2:
	case SUNXI_BOOTED_FROM_MMC2_HIGH:
		return BOOT_DEVICE_MMC2;
	case SUNXI_BOOTED_FROM_SPI:
		return BOOT_DEVICE_SPI;
	}

	panic("Unknown boot source %d\n", boot_source);
	return -1;		/* Never reached */
}

#ifdef CONFIG_SPL_BUILD
static u32 sunxi_get_spl_size(void)
{
	if (!is_boot0_magic(SPL_ADDR + 4)) /* eGON.BT0 */
		return 0;

	return readl(SPL_ADDR + 0x10);
}

/*
 * The eGON SPL image can be located at 8KB or at 128KB into an SD card or
 * an eMMC device. The boot source has bit 4 set in the latter case.
 * By adding 120KB to the normal offset when booting from a "high" location
 * we can support both cases.
 * Also U-Boot proper is located at least 32KB after the SPL, but will
 * immediately follow the SPL if that is bigger than that.
 */
unsigned long spl_mmc_get_uboot_raw_sector(struct mmc *mmc,
					   unsigned long raw_sect)
{
	unsigned long spl_size = sunxi_get_spl_size();
	unsigned long sector;

	sector = max(raw_sect, spl_size / 512);

	switch (sunxi_get_boot_source()) {
	case SUNXI_BOOTED_FROM_MMC0_HIGH:
	case SUNXI_BOOTED_FROM_MMC2_HIGH:
		sector += (128 - 8) * 2;
		break;
	}

	return sector;
}

u32 spl_boot_device(void)
{
	return sunxi_get_boot_device();
}


#pragma pack(push, 1)
typedef struct
{
  union
  {
    uint8_t enabled;
    struct
    {
      uint8_t hold100ms : 4;
      uint8_t fade100ms : 4;
    };
  };
  uint8_t blue;
  uint8_t green;
  uint8_t red;
} color_t;
#pragma pack(pop)

#define SRAM_A2_SIZE            (48*1024)
#define SRAM_A2_ADDR            0x00040000
#define ARISC_COMM_SIZE         2048
#define ARISC_COMM_ADDR         (SRAM_A2_ADDR + SRAM_A2_SIZE - ARISC_COMM_SIZE)

#define ARISC_OVERRIDE_COLOR	(ARISC_COMM_ADDR + 0x00)	//4b800
#define ARISC_JUMP_TO_COLOR		(ARISC_COMM_ADDR + 0x04)
#define ARISC_FADE_TO_COLOR		(ARISC_COMM_ADDR + 0x08)
#define ARISC_COLOR_TABLE		(ARISC_COMM_ADDR + 0x10)	//4b810
#define ARISC_COLOR_COUNT		8
#define ARISC_COLOR_TABLE_SIZE	(sizeof(color_t)*ARISC_COLOR_COUNT)

#define ARISC_WATCHDOG_ENABLED	(ARISC_COMM_ADDR + 0x100) 	//4b900
#define ARISC_WATCHDOG_RESET	(ARISC_COMM_ADDR + 0x104)	//4b904
#define ARISC_WATCHDOG_TIMEOUT	(ARISC_COMM_ADDR + 0x108)	//4b908
#define ARISC_WATCHDOG_TIMEOUT_DEFAULT 60
#define ARISC_BUTTON_PRESSED	(ARISC_COMM_ADDR + 0x140) 	//4b940

void board_init_f(ulong dummy)
{
	spl_init();
	/* SPL loader before first print */

	gpio_direction_output(SUNXI_GPA(20), 0); //XR819 1.8V LDO = off
	gpio_direction_output(SUNXI_GPL(7), 0);  //XR819 RSTN=0 (power off)

	unsigned buttonGpio = SUNXI_GPA(11);
	unsigned redLedGpio = SUNXI_GPA(17);
	unsigned rs232Gpio = SUNXI_GPA(16);
	unsigned bleNReset = SUNXI_GPA(10);
	unsigned greenLedGpio = SUNXI_GPL(10);
	gpio_direction_output(redLedGpio, 1); //turn on RED led = bootloader active

	gpio_direction_input(buttonGpio);
	bool buttonPressed = gpio_get_value(buttonGpio) == 0;
	bool bootFromSpiFlash = sunxi_get_boot_device() == BOOT_DEVICE_SPI;
	gpio_direction_output(greenLedGpio, bootFromSpiFlash || buttonPressed ? 1 : 0); //button enables GREEN led
	gpio_direction_output(rs232Gpio, bootFromSpiFlash || buttonPressed ? 0 : 1); //button enables RS232
	gpio_direction_output(bleNReset, 0); //switch off BLE module

	mdelay(1); //wait for RS232 converter powerup
	preloader_console_init();

	void *sramA2 = (void*)SRAM_A2_ADDR;
	unsigned int sramA2Len = SRAM_A2_SIZE;
	volatile uint32_t *arisc_running = (uint32_t*)(0x01f01c00);
	volatile uint32_t *arisc_vcore = (uint32_t*)(0x01f00190);
	volatile uint32_t *jumpToColor = (uint32_t*)(ARISC_JUMP_TO_COLOR);
	volatile color_t *forceColor = (color_t*)(ARISC_OVERRIDE_COLOR);
	volatile color_t *colors = (color_t*)(ARISC_COLOR_TABLE);
	volatile uint32_t *wdEnable = (uint32_t*)(ARISC_WATCHDOG_ENABLED);
	volatile uint32_t *wdTimeout = (uint32_t*)(ARISC_WATCHDOG_TIMEOUT);

	*arisc_vcore = 5; // VCore = 0.7V+N*0.1V, default = 4 (1.1V)
	*arisc_running = 0;
	int load_res = 1; //tinf_uncompress(sramA2, &sramA2Len, arisc_fw_deflate, sizeof(arisc_fw_deflate));
	printf("SPL: AR100 FW loaded: %u bytes, err: %d, core at 1.2V", sramA2Len, load_res);
	if(load_res == 0)
	{
		*jumpToColor = 0;
		*arisc_running = 1;
		while(*jumpToColor == 0); //wait for ARISC initialization

		if(bootFromSpiFlash)
			*forceColor = (color_t) { .red = 180, .green = 0, .blue = 0, .enabled = true };
		else
		{
			if(buttonPressed)
			{
				colors[0] = (color_t) { .red = 110, .green = 0, .blue = 70, .hold100ms = 5, .fade100ms = 2 };
				colors[1] = (color_t) { .red = 30, .green = 0, .blue = 25, .hold100ms = 1, .fade100ms = 2 };
			}
			else
			{
				colors[0] = (color_t) { .red = 110, .green = 60, .blue = 0, .hold100ms = 5, .fade100ms = 2 };
				colors[1] = (color_t) { .red = 30, .green = 15, .blue = 0, .hold100ms = 1, .fade100ms = 2 };
			}
		}

		*wdTimeout = 10*60;
		*wdEnable = 1;
	}

#if CONFIG_IS_ENABLED(I2C) && CONFIG_IS_ENABLED(SYS_I2C_LEGACY)
	/* Needed early by sunxi_board_init if PMU is enabled */
	i2c_init(CONFIG_SYS_I2C_SPEED, CONFIG_SYS_I2C_SLAVE);
#endif
	sunxi_board_init();
}
#endif

#if !CONFIG_IS_ENABLED(SYSRESET)
void reset_cpu(void)
{
#if defined(CONFIG_SUNXI_GEN_SUN4I) || defined(CONFIG_MACH_SUN8I_R40)
	static const struct sunxi_wdog *wdog =
		 &((struct sunxi_timer_reg *)SUNXI_TIMER_BASE)->wdog;

	/* Set the watchdog for its shortest interval (.5s) and wait */
	writel(WDT_MODE_RESET_EN | WDT_MODE_EN, &wdog->mode);
	writel(WDT_CTRL_KEY | WDT_CTRL_RESTART, &wdog->ctl);

	while (1) {
		/* sun5i sometimes gets stuck without this */
		writel(WDT_MODE_RESET_EN | WDT_MODE_EN, &wdog->mode);
	}
#elif defined(CONFIG_SUNXI_GEN_SUN6I) || defined(CONFIG_SUN50I_GEN_H6)
#if defined(CONFIG_MACH_SUN50I_H6)
	/* WDOG is broken for some H6 rev. use the R_WDOG instead */
	static const struct sunxi_wdog *wdog =
		(struct sunxi_wdog *)SUNXI_R_WDOG_BASE;
#else
	static const struct sunxi_wdog *wdog =
		((struct sunxi_timer_reg *)SUNXI_TIMER_BASE)->wdog;
#endif
	/* Set the watchdog for its shortest interval (.5s) and wait */
	writel(WDT_CFG_RESET, &wdog->cfg);
	writel(WDT_MODE_EN, &wdog->mode);
	writel(WDT_CTRL_KEY | WDT_CTRL_RESTART, &wdog->ctl);
	while (1) { }
#endif
}
#endif

#if !CONFIG_IS_ENABLED(SYS_DCACHE_OFF) && !defined(CONFIG_ARM64)
void enable_caches(void)
{
	/* Enable D-cache. I-cache is already enabled in start.S */
	dcache_enable();
}
#endif
