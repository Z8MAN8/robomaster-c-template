/*
 * This file is part of the EasyFlash Library.
 *
 * Copyright (c) 2015-2019, Armink, <armink.ztl@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * 'Software'), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED 'AS IS', WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * Function: Portable interface for each platform.
 * Created on: 2015-01-16
 */

#include <easyflash.h>
#include <rthw.h>
#include <rtthread.h>

/* base address of the flash sectors */
#define ADDR_FLASH_SECTOR_0      ((uint32_t)0x08000000) /* Base address of Sector 0, 16 K bytes   */
#define ADDR_FLASH_SECTOR_1      ((uint32_t)0x08004000) /* Base address of Sector 1, 16 K bytes   */
#define ADDR_FLASH_SECTOR_2      ((uint32_t)0x08008000) /* Base address of Sector 2, 16 K bytes   */
#define ADDR_FLASH_SECTOR_3      ((uint32_t)0x0800C000) /* Base address of Sector 3, 16 K bytes   */
#define ADDR_FLASH_SECTOR_4      ((uint32_t)0x08010000) /* Base address of Sector 4, 64 K bytes   */
#define ADDR_FLASH_SECTOR_5      ((uint32_t)0x08020000) /* Base address of Sector 5, 128 K bytes  */
#define ADDR_FLASH_SECTOR_6      ((uint32_t)0x08040000) /* Base address of Sector 6, 128 K bytes  */
#define ADDR_FLASH_SECTOR_7      ((uint32_t)0x08060000) /* Base address of Sector 7, 128 K bytes  */
#define ADDR_FLASH_SECTOR_8      ((uint32_t)0x08080000) /* Base address of Sector 8, 128 K bytes  */
#define ADDR_FLASH_SECTOR_9      ((uint32_t)0x080A0000) /* Base address of Sector 9, 128 K bytes  */
#define ADDR_FLASH_SECTOR_10     ((uint32_t)0x080C0000) /* Base address of Sector 10, 128 K bytes */
#define ADDR_FLASH_SECTOR_11     ((uint32_t)0x080E0000) /* Base address of Sector 11, 128 K bytes */
#define ADDR_FLASH_SECTOR_12     ((uint32_t)0x08100000) /* Base address of Sector 12, 16 K bytes  */

/* default environment variables set for user */
static const ef_env default_env_set[] = {
        {"iap_need_copy_app","0"},
        {"iap_copy_app_size","0"},
        {"stop_in_bootloader","0"},
        {"device_id","1"},
        {"boot_times","0"},

};

static char log_buf[RT_CONSOLEBUF_SIZE];
static struct rt_semaphore env_cache_lock;

static uint32_t stm32_get_sector(uint32_t address);
static uint32_t stm32_get_sector_size(uint32_t sector);

/**
 * Flash port for hardware initialize.
 *
 * @param default_env default ENV set for user
 * @param default_env_size default ENV size
 *
 * @return result
 */
EfErrCode ef_port_init(ef_env const **default_env, size_t *default_env_size) {
    EfErrCode result = EF_NO_ERR;

    *default_env = default_env_set;
    *default_env_size = sizeof(default_env_set) / sizeof(default_env_set[0]);

    rt_sem_init(&env_cache_lock, "env lock", 1, RT_IPC_FLAG_PRIO);

    return result;
}

/**
 * Read data from flash.
 * @note This operation's units is word.
 *
 * @param addr flash address
 * @param buf buffer to store read data
 * @param size read bytes size
 *
 * @return result
 */
EfErrCode ef_port_read(uint32_t addr, uint32_t *buf, size_t size) {
    EfErrCode result = EF_NO_ERR;
    uint8_t *buf_8 = (uint8_t *)buf;
    size_t i;

    /*copy from flash to ram */
    for (i = 0; i < size; i++, addr ++, buf_8++) {
        *buf_8 = *(uint8_t *) addr;
    }

    return result;
}

/**
 * Erase data on flash.
 * @note This operation is irreversible.
 * @note This operation's units is different which on many chips.
 *
 * @param addr flash address
 * @param size erase bytes size
 *
 * @return result
 */
EfErrCode ef_port_erase(uint32_t addr, size_t size) {
    EfErrCode result = EF_NO_ERR;
    size_t erased_size = 0;
    uint32_t cur_erase_sector;

/* make sure the start address is a multiple of EF_ERASE_MIN_SIZE */
    EF_ASSERT(addr % EF_ERASE_MIN_SIZE == 0);
/* start erase*/
    HAL_FLASH_Unlock();

/* it will stop when erased size is greater than setting size */
    while(erased_size < size) {
        cur_erase_sector = stm32_get_sector(addr + erased_size);
        FLASH_Erase_Sector(cur_erase_sector, FLASH_VOLTAGE_RANGE_3);
        erased_size += stm32_get_sector_size(cur_erase_sector);
    }

    HAL_FLASH_Lock();

    return (EfErrCode)result;
}
/**
 * Write data to flash.
 * @note This operation's units is word.
 * @note This operation must after erase. @see flash_erase.
 *
 * @param addr flash address
 * @param buf the write data buffer
 * @param size write bytes size
 *
 * @return result
 */
EfErrCode ef_port_write(uint32_t addr, const uint32_t *buf, size_t size) {
    EfErrCode result = EF_NO_ERR;
    size_t i;
    uint32_t read_data;
    uint8_t *buf_8 = (uint8_t *)buf;

    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR
            | FLASH_FLAG_PGSERR);
    for (i = 0; i < size; i++, buf_8++, addr++)
    {
        /* write data */
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, addr, *buf_8);
        read_data = *(uint8_t *) addr;
        /* check data */
        if (read_data != *buf_8) {
            result = EF_WRITE_ERR;
            break;
        }
    }
    HAL_FLASH_Lock();

    return result;
}

/**
 * lock the ENV ram cache
 */
void ef_port_env_lock(void) {
    
    rt_sem_take(&env_cache_lock, RT_WAITING_FOREVER);
    
}

/**
 * unlock the ENV ram cache
 */
void ef_port_env_unlock(void) {
    
    rt_sem_release(&env_cache_lock);
    
}

/**
 * Get the sector of a given address
 *
 * @param address flash address
 *
 * @return The sector of a given address
 */
static uint32_t stm32_get_sector(uint32_t address) {
    uint32_t sector = 0;

    if ((address < ADDR_FLASH_SECTOR_1) && (address >= ADDR_FLASH_SECTOR_0)) {
        sector = FLASH_SECTOR_0;
    } else if ((address < ADDR_FLASH_SECTOR_2) && (address >= ADDR_FLASH_SECTOR_1)) {
        sector = FLASH_SECTOR_1;
    } else if ((address < ADDR_FLASH_SECTOR_3) && (address >= ADDR_FLASH_SECTOR_2)) {
        sector = FLASH_SECTOR_2;
    } else if ((address < ADDR_FLASH_SECTOR_4) && (address >= ADDR_FLASH_SECTOR_3)) {
        sector = FLASH_SECTOR_3;
    } else if ((address < ADDR_FLASH_SECTOR_5) && (address >= ADDR_FLASH_SECTOR_4)) {
        sector = FLASH_SECTOR_4;
    } else if ((address < ADDR_FLASH_SECTOR_6) && (address >= ADDR_FLASH_SECTOR_5)) {
        sector = FLASH_SECTOR_5;
    } else if ((address < ADDR_FLASH_SECTOR_7) && (address >= ADDR_FLASH_SECTOR_6)) {
        sector = FLASH_SECTOR_6;
    } else if ((address < ADDR_FLASH_SECTOR_8) && (address >= ADDR_FLASH_SECTOR_7)) {
        sector = FLASH_SECTOR_7;
    } else if ((address < ADDR_FLASH_SECTOR_9) && (address >= ADDR_FLASH_SECTOR_8)) {
        sector = FLASH_SECTOR_8;
    } else if ((address < ADDR_FLASH_SECTOR_10) && (address >= ADDR_FLASH_SECTOR_9)) {
        sector = FLASH_SECTOR_9;
    } else if ((address < ADDR_FLASH_SECTOR_11) && (address >= ADDR_FLASH_SECTOR_10)) {
        sector = FLASH_SECTOR_10;
    } else if ((address < ADDR_FLASH_SECTOR_12) && (address >= ADDR_FLASH_SECTOR_11)) {
        sector = FLASH_SECTOR_11;
    } else /*(address < FLASH_END_ADDR) && (address >= ADDR_FLASH_SECTOR_23))*/
    {
        sector = FLASH_SECTOR_11;
    }

    return sector;
}

/**
 * Get the sector size
 *
 * @param sector sector
 *
 * @return sector size
 */
static uint32_t stm32_get_sector_size(uint32_t sector) {
    EF_ASSERT(IS_FLASH_SECTOR(sector));

    switch (sector) {
    case FLASH_SECTOR_0: return 16 * 1024;
    case FLASH_SECTOR_1: return 16 * 1024;
    case FLASH_SECTOR_2: return 16 * 1024;
    case FLASH_SECTOR_3: return 16 * 1024;
    case FLASH_SECTOR_4: return 64 * 1024;
    case FLASH_SECTOR_5: return 128 * 1024;
    case FLASH_SECTOR_6: return 128 * 1024;
    case FLASH_SECTOR_7: return 128 * 1024;
    case FLASH_SECTOR_8: return 128 * 1024;
    case FLASH_SECTOR_9: return 128 * 1024;
    case FLASH_SECTOR_10: return 128 * 1024;
    case FLASH_SECTOR_11: return 128 * 1024;
    default : return 128 * 1024;
    }
}

/**
 * This function is print flash debug info.
 *
 * @param file the file which has call this function
 * @param line the line number which has call this function
 * @param format output format
 * @param ... args
 *
 */
void ef_log_debug(const char *file, const long line, const char *format, ...) {

#ifdef PRINT_DEBUG

    va_list args;

    /* args point to the first variable parameter */
    va_start(args, format);
    ef_print("[Flash](%s:%ld) ", file, line);
    /* must use vprintf to print */
    rt_vsprintf(log_buf, format, args);
    ef_print("%s", log_buf);
    va_end(args);

#endif

}

/**
 * This function is print flash routine info.
 *
 * @param format output format
 * @param ... args
 */
void ef_log_info(const char *format, ...) {
    va_list args;

    /* args point to the first variable parameter */
    va_start(args, format);
    ef_print("[Flash]");
    /* must use vprintf to print */
    rt_vsprintf(log_buf, format, args);
    ef_print("%s", log_buf);
    va_end(args);
}
/**
 * This function is print flash non-package info.
 *
 * @param format output format
 * @param ... args
 */
void ef_print(const char *format, ...) {
    va_list args;

    /* args point to the first variable parameter */
    va_start(args, format);

    /* must use vprintf to print */
    rt_vsprintf(log_buf, format, args);
    rt_kprintf("%s", log_buf);
    
    va_end(args);
}


#if defined(RT_USING_FINSH) && defined(FINSH_USING_MSH)
#include <finsh.h>
#if defined(EF_USING_ENV)
static void setenv(uint8_t argc, char **argv) {
    uint8_t i;

    if (argc > 3) {
        /* environment variable value string together */
        for (i = 0; i < argc - 2; i++) {
            argv[2 + i][rt_strlen(argv[2 + i])] = ' ';
        }
    }
    if (argc == 1) {
        rt_kprintf("Please input: setenv <key> [value]\n");
    } else if (argc == 2) {
        ef_set_env(argv[1], NULL);
    } else {
        ef_set_env(argv[1], argv[2]);
    }
}
MSH_CMD_EXPORT(setenv, Set an envrionment variable.);

static void printenv(uint8_t argc, char **argv) {
    ef_print_env();
}
MSH_CMD_EXPORT(printenv, Print all envrionment variables.);

static void saveenv(uint8_t argc, char **argv) {
    ef_save_env();
}
MSH_CMD_EXPORT(saveenv, Save all envrionment variables to flash.);

static void getvalue(uint8_t argc, char **argv) {
    char *value = NULL;
    value = ef_get_env(argv[1]);
    if (value) {
        rt_kprintf("The %s value is %s.\n", argv[1], value);
    } else {
        rt_kprintf("Can't find %s.\n", argv[1]);
    }
}
MSH_CMD_EXPORT(getvalue, Get an envrionment variable by name.);

static void resetenv(uint8_t argc, char **argv) {
    ef_env_set_default();
}
MSH_CMD_EXPORT(resetenv, Reset all envrionment variable to default.);
#endif /* defined(EF_USING_ENV) */
#endif /* defined(RT_USING_FINSH) && defined(FINSH_USING_MSH) */
