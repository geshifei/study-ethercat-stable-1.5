/******************************************************************************
 *
 *  $Id$
 *
 *  Copyright (C) 2006-2008  Florian Pose, Ingenieurgemeinschaft IgH
 *
 *  This file is part of the IgH EtherCAT Master.
 *
 *  The IgH EtherCAT Master is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License version 2, as
 *  published by the Free Software Foundation.
 *
 *  The IgH EtherCAT Master is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
 *  Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with the IgH EtherCAT Master; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 *  ---
 *
 *  The license mentioned above concerns the source code only. Using the
 *  EtherCAT technology and brand is only permitted in compliance with the
 *  industrial property and similar rights of Beckhoff Automation GmbH.
 *
 *****************************************************************************/

/** \file
 * EtherCAT master driver module.
 */

/*****************************************************************************/

#include <linux/module.h>
#include <linux/device.h>
#include <linux/err.h>

#include "globals.h"
#include "master.h"
#include "device.h"

/*****************************************************************************/

#define MAX_MASTERS 32 /**< Maximum number of masters. */

/*****************************************************************************/

int __init ec_init_module(void);
void __exit ec_cleanup_module(void);

static int ec_mac_parse(uint8_t *, const char *, int);

/*****************************************************************************/

static char *main_devices[MAX_MASTERS]; /**< Main devices parameter. */
static unsigned int master_count; /**< Number of masters. */
static char *backup_devices[MAX_MASTERS]; /**< Backup devices parameter. */
static unsigned int backup_count; /**< Number of backup devices. */
static unsigned int debug_level;  /**< Debug level parameter. */
static unsigned int run_on_cpu = 0xffffffff; /**< Bind created kernel threads to a cpu. Default do not bind*/

static ec_master_t *masters; /**< Array of masters. */
static struct semaphore master_sem; /**< Master semaphore. */

dev_t device_number; /**< Device number for master cdevs. */
struct class *class; /**< Device class. */

static uint8_t macs[MAX_MASTERS][2][ETH_ALEN]; /**< MAC addresses. */

char *ec_master_version_str = EC_MASTER_VERSION; /**< Version string. */

/*****************************************************************************/

/** \cond */

MODULE_AUTHOR("Florian Pose <fp@igh-essen.com>");
MODULE_DESCRIPTION("EtherCAT master driver module");
MODULE_LICENSE("GPL");
MODULE_VERSION(EC_MASTER_VERSION);

module_param_array(main_devices, charp, &master_count, S_IRUGO);
MODULE_PARM_DESC(main_devices, "MAC addresses of main devices");
module_param_array(backup_devices, charp, &backup_count, S_IRUGO);
MODULE_PARM_DESC(backup_devices, "MAC addresses of backup devices");
module_param_named(debug_level, debug_level, uint, S_IRUGO);
MODULE_PARM_DESC(debug_level, "Debug level");
module_param_named(run_on_cpu, run_on_cpu, uint, S_IRUGO);
MODULE_PARM_DESC(run_on_cpu, "Bind kthreads to a specific cpu");

/** \endcond */

/*****************************************************************************/

/** Module initialization.
 *
 * Initializes \a master_count masters.
 * \return 0 on success, else < 0
 */
/*
 * 如何看代码：
 * 1，加载主站ec_master模块，进入EC_ORPHANED阶段
 * 大部分操作是由ec_master_init完成的，参考函数说明.
 * 重要的是，master->fsm->state状态处理函数设置为ec_fsm_master_state_start.
 * 2，加载网卡驱动，主站进入EC_IDLE阶段
 * 加载网卡驱动时，根据网卡的mac，将网卡绑定到指定的主站，
 * 绑定后主站进入EC_IDLE阶段，并执行ec_master_idle_thread线程.
 * 以rt8139为例:
 * rtl8139_init_module --> pci_module_init (&rtl8139_pci_driver) -->
 * rtl8139_pci_driver，执行的probe函数是rtl8139_init_one（8139too-3.4-ethercat.c文件中），
 * 上面加载驱动的流程可从rtl8139_init_one函数看起.
 */
int __init ec_init_module(void)
{
    int i, ret = 0;

    EC_INFO("Master driver %s\n", EC_MASTER_VERSION);

    sema_init(&master_sem, 1);

    if (master_count) {
        if (alloc_chrdev_region(&device_number,
                    0, master_count, "EtherCAT")) {
            EC_ERR("Failed to obtain device number(s)!\n");
            ret = -EBUSY;
            goto out_return;
        }
    }

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 4, 0)
    class = class_create(THIS_MODULE, "EtherCAT");
#else
    class = class_create("EtherCAT");
#endif
    if (IS_ERR(class)) {
        EC_ERR("Failed to create device class.\n");
        ret = PTR_ERR(class);
        goto out_cdev;
    }

    // zero MAC addresses
    memset(macs, 0x00, sizeof(uint8_t) * MAX_MASTERS * 2 * ETH_ALEN);

    // process MAC parameters
    for (i = 0; i < master_count; i++) {
        /* insmod时指定的字符串类型的mac参数main_devices[]，转换成uint8_t形式存放在macs[] */
        ret = ec_mac_parse(macs[i][0], main_devices[i], 0);
        if (ret)
            goto out_class;

        if (i < backup_count) {
            ret = ec_mac_parse(macs[i][1], backup_devices[i], 1);
            if (ret)
                goto out_class;
        }
    }

    // initialize static master variables
    ec_master_init_static();

    if (master_count) {
        if (!(masters = kmalloc(sizeof(ec_master_t) * master_count,
                        GFP_KERNEL))) {
            EC_ERR("Failed to allocate memory"
                    " for EtherCAT masters.\n");
            ret = -ENOMEM;
            goto out_class;
        }
    }

    /*
     * ec_master_init功能:
     * 1) 主站状phase设为EC_ORPHANED,表示主站还没有关联网卡.    <==========很重要!!!
     * 2）初始化主站的数据报master->ext_datagram_ring[]，
     *    这个报文用于处理与slave相关请求，如配置、扫描、SDO、PDO等.
     * 3) 将macs[i]对应的device与master关联，并分配、初始化device->tx_skb[i]
     *    skb的长度位ETH_FRAME_LEN，即除去报文头、报文尾后，可用的报文长度（1514个字节）.
     *    device->tx_skb[i]缓冲区有EC_TX_RING_SIZE组（2组）.
     * 4) 初始化主状态机master->fsm（名字为"master-fsm"）用到的报master->fsm_datagram
     * 5) 为master->fsm_datagram分配payload缓冲区，缓冲区大小EC_MAX_DATA_SIZE是除去
     *    以太网头、ecat头等信息后EtherCAT可用的报文长度大小.
     * 6）ec_fsm_master_init
     *    6.1)通过ec_fsm_master_reset设为将主站状态机设为START(对应处理函数ec_fsm_master_state_start）
     *    6.2)初始化主状态机master->fsm的子状态机fsm->fsm_coe、fsm->fsm_pdo等等.
     * 7) 为master->ext_datagram_ring[]分配payload缓冲区(见1)
     * 8）初始化DC同步报文master->ref_sync_datagram，报文的名称为"refsync"
     * 9）初始化DC补偿报文master->sync_datagram，报文的名称为"sync"
     * 10）初始化DC同步监控报文master->sync_mon_datagram，报文的名称为"syncmon"
     * 11）通过ec_cdev_init创建master的字符设备，类似于/dev/EtherCAT0
     */
    for (i = 0; i < master_count; i++) {
        ret = ec_master_init(&masters[i], i, macs[i][0], macs[i][1],
                    device_number, class, debug_level, run_on_cpu);
        if (ret)
            goto out_free_masters;
    }

    EC_INFO("%u master%s waiting for devices.\n",
            master_count, (master_count == 1 ? "" : "s"));
    return ret;

out_free_masters:
    for (i--; i >= 0; i--)
        ec_master_clear(&masters[i]);
    kfree(masters);
out_class:
    class_destroy(class);
out_cdev:
    if (master_count)
        unregister_chrdev_region(device_number, master_count);
out_return:
    return ret;
}

/*****************************************************************************/

/** Module cleanup.
 *
 * Clears all master instances.
 */
void __exit ec_cleanup_module(void)
{
    unsigned int i;

    for (i = 0; i < master_count; i++) {
        ec_master_clear(&masters[i]);
    }

    if (master_count)
        kfree(masters);

    class_destroy(class);

    if (master_count)
        unregister_chrdev_region(device_number, master_count);

    EC_INFO("Master module cleaned up.\n");
}

/*****************************************************************************/

/** Get the number of masters.
 */
unsigned int ec_master_count(void)
{
    return master_count;
}

/*****************************************************************************
 * MAC address functions
 ****************************************************************************/

/**
 * \return true, if two MAC addresses are equal.
 */
int ec_mac_equal(
        const uint8_t *mac1, /**< First MAC address. */
        const uint8_t *mac2 /**< Second MAC address. */
        )
{
    unsigned int i;

    for (i = 0; i < ETH_ALEN; i++)
        if (mac1[i] != mac2[i])
            return 0;

    return 1;
}

/*****************************************************************************/

/** Maximum MAC string size.
 */
#define EC_MAX_MAC_STRING_SIZE (3 * ETH_ALEN)

/** Print a MAC address to a buffer.
 *
 * The buffer size must be at least EC_MAX_MAC_STRING_SIZE.
 *
 * \return number of bytes written.
 */
ssize_t ec_mac_print(
        const uint8_t *mac, /**< MAC address */
        char *buffer /**< Target buffer. */
        )
{
    off_t off = 0;
    unsigned int i;

    for (i = 0; i < ETH_ALEN; i++) {
        off += sprintf(buffer + off, "%02X", mac[i]);
        if (i < ETH_ALEN - 1) off += sprintf(buffer + off, ":");
    }

    return off;
}

/*****************************************************************************/

/**
 * \return true, if the MAC address is all-zero.
 */
int ec_mac_is_zero(
        const uint8_t *mac /**< MAC address. */
        )
{
    unsigned int i;

    for (i = 0; i < ETH_ALEN; i++)
        if (mac[i])
            return 0;

    return 1;
}

/*****************************************************************************/

/**
 * \return true, if the given MAC address is the broadcast address.
 */
int ec_mac_is_broadcast(
        const uint8_t *mac /**< MAC address. */
        )
{
    unsigned int i;

    for (i = 0; i < ETH_ALEN; i++)
        if (mac[i] != 0xff)
            return 0;

    return 1;
}

/*****************************************************************************/

/** Parse a MAC address from a string.
 *
 * The MAC address must match the regular expression
 * "([0-9a-fA-F]{2}:){5}[0-9a-fA-F]{2}".
 *
 * \return 0 on success, else < 0
 */
static int ec_mac_parse(uint8_t *mac, const char *src, int allow_empty)
{
    unsigned int i, value;
    const char *orig = src;
    char *rem;

    if (!strlen(src)) {
        if (allow_empty){
            return 0;
        } else {
            EC_ERR("MAC address may not be empty.\n");
            return -EINVAL;
        }
    }

    for (i = 0; i < ETH_ALEN; i++) {
        value = simple_strtoul(src, &rem, 16);
        if (rem != src + 2
                || value > 0xFF
                || (i < ETH_ALEN - 1 && *rem != ':')) {
            EC_ERR("Invalid MAC address \"%s\".\n", orig);
            return -EINVAL;
        }
        mac[i] = value;
        if (i < ETH_ALEN - 1) {
            src = rem + 1; // skip colon
        }
    }

    return 0;
}

/*****************************************************************************/

/** Outputs frame contents for debugging purposes.
 * If the data block is larger than 256 bytes, only the first 128
 * and the last 128 bytes will be shown
 */
void ec_print_data(const uint8_t *data, /**< pointer to data */
                   size_t size /**< number of bytes to output */
                   )
{
    unsigned int i;

    EC_DBG("");
    for (i = 0; i < size; i++) {
        printk(KERN_CONT "%02X ", data[i]);

        if ((i + 1) % 16 == 0 && i < size - 1) {
            printk(KERN_CONT "\n");
            EC_DBG("");
        }

        if (i + 1 == 128 && size > 256) {
            printk(KERN_CONT "dropped %zu bytes\n", size - 128 - i);
            i = size - 128;
            EC_DBG("");
        }
    }
    printk(KERN_CONT "\n");
}

/*****************************************************************************/

/** Outputs frame contents and differences for debugging purposes.
 */
void ec_print_data_diff(const uint8_t *d1, /**< first data */
                        const uint8_t *d2, /**< second data */
                        size_t size /** number of bytes to output */
                        )
{
    unsigned int i;

    EC_DBG("");
    for (i = 0; i < size; i++) {
        if (d1[i] == d2[i]) {
            printk(KERN_CONT ".. ");
        }
        else {
            printk(KERN_CONT "%02X ", d2[i]);
        }
        if ((i + 1) % 16 == 0) {
            printk(KERN_CONT "\n");
            EC_DBG("");
        }
    }
    printk(KERN_CONT "\n");
}

/*****************************************************************************/

/** Prints slave states in clear text.
 *
 * \return Size of the created string.
 */
size_t ec_state_string(uint8_t states, /**< slave states */
                       char *buffer, /**< target buffer
                                       (min. EC_STATE_STRING_SIZE bytes) */
                       uint8_t multi /**< Show multi-state mask. */
                       )
{
    off_t off = 0;
    unsigned int first = 1;

    if (!states) {
        off += sprintf(buffer + off, "(unknown)");
        return off;
    }

    if (multi) { // multiple slaves
        if (states & EC_SLAVE_STATE_INIT) {
            off += sprintf(buffer + off, "INIT");
            first = 0;
        }
        if (states & EC_SLAVE_STATE_PREOP) {
            if (!first) off += sprintf(buffer + off, ", ");
            off += sprintf(buffer + off, "PREOP");
            first = 0;
        }
        if (states & EC_SLAVE_STATE_SAFEOP) {
            if (!first) off += sprintf(buffer + off, ", ");
            off += sprintf(buffer + off, "SAFEOP");
            first = 0;
        }
        if (states & EC_SLAVE_STATE_OP) {
            if (!first) off += sprintf(buffer + off, ", ");
            off += sprintf(buffer + off, "OP");
        }
    } else { // single slave
        if ((states & EC_SLAVE_STATE_MASK) == EC_SLAVE_STATE_INIT) {
            off += sprintf(buffer + off, "INIT");
        } else if ((states & EC_SLAVE_STATE_MASK) == EC_SLAVE_STATE_PREOP) {
            off += sprintf(buffer + off, "PREOP");
        } else if ((states & EC_SLAVE_STATE_MASK) == EC_SLAVE_STATE_BOOT) {
            off += sprintf(buffer + off, "BOOT");
        } else if ((states & EC_SLAVE_STATE_MASK) == EC_SLAVE_STATE_SAFEOP) {
            off += sprintf(buffer + off, "SAFEOP");
        } else if ((states & EC_SLAVE_STATE_MASK) == EC_SLAVE_STATE_OP) {
            off += sprintf(buffer + off, "OP");
        } else {
            off += sprintf(buffer + off, "(invalid)");
        }
        first = 0;
    }

    if (states & EC_SLAVE_STATE_ACK_ERR) {
        if (!first) off += sprintf(buffer + off, " + ");
        off += sprintf(buffer + off, "ERROR");
    }

    return off;
}

/******************************************************************************
 *  Device interface
 *****************************************************************************/

/** Device names.
 */
const char *ec_device_names[2] = {
    "main",
    "backup"
};

/** Offers an EtherCAT device to a certain master.
 *
 * The master decides, if it wants to use the device for EtherCAT operation,
 * or not. It is important, that the offered net_device is not used by the
 * kernel IP stack. If the master, accepted the offer, the address of the
 * newly created EtherCAT device is returned, else \a NULL is returned.
 *
 * \return Pointer to device, if accepted, or NULL if declined.
 * \ingroup DeviceInterface
 */
/*
 * insmod XX网卡驱动XX.ko时会调用到probe函数,probe函数调用ecdev_offer将该网卡绑定到主站上.
 * 绑定到哪个主站是由mac地址决定的，创建主站模块时用户传入网卡的mac地址(见ec_init_module)，
 * 记录在masters[i]，网卡加载时就根据网卡的mac，绑定到对应的master.
 * 以rt8139网卡为例：
 * rtl8139_pci_driver的probe函数是rtl8139_init_one（8139too-3.4-ethercat.c文件中）,
 * rtl8139_init_one函数逻辑:
 * 1)stp->ecdev = ecdev_offer(dev, ec_poll, THIS_MODULE)将网卡绑定到主站,
 * 2)接着ecdev_open(tp->ecdev)打开主站，主站进入EC_IDLE阶段，并循环执行ec_master_idle_thread线程
 * ecdev_open -> ec_master_enter_idle_phase -> ec_master_thread_start循环执行
 * 线程ec_master_idle_thread
 */
ec_device_t *ecdev_offer(
        struct net_device *net_dev, /**< net_device to offer */
        ec_pollfunc_t poll, /**< device poll function */
        struct module *module /**< pointer to the module */
        )
{
    ec_master_t *master;
    char str[EC_MAX_MAC_STRING_SIZE];
    unsigned int i, dev_idx;

    for (i = 0; i < master_count; i++) {
        master = &masters[i];
        ec_mac_print(net_dev->dev_addr, str);

        if (down_interruptible(&master->device_sem)) {
            EC_MASTER_WARN(master, "%s() interrupted!\n", __func__);
            return NULL;
        }

        for (dev_idx = EC_DEVICE_MAIN;
                dev_idx < ec_master_num_devices(master); dev_idx++) {
            if (!master->devices[dev_idx].dev
                && (ec_mac_equal(master->macs[dev_idx], net_dev->dev_addr)
                    || ec_mac_is_broadcast(master->macs[dev_idx]))) {

                EC_INFO("Accepting %s as %s device for master %u.\n",
                        str, ec_device_names[dev_idx != 0], master->index);

                ec_device_attach(&master->devices[dev_idx],
                        net_dev, poll, module);
                up(&master->device_sem);

                snprintf(net_dev->name, IFNAMSIZ, "ec%c%u",
                        ec_device_names[dev_idx != 0][0], master->index);

                return &master->devices[dev_idx]; // offer accepted
            }
        }

        up(&master->device_sem);

        EC_MASTER_DBG(master, 1, "Master declined device %s.\n", str);
    }

    return NULL; // offer declined
}

/******************************************************************************
 * Application interface
 *****************************************************************************/

/** Request a master.
 *
 * Same as ecrt_request_master(), but with ERR_PTR() return value.
 *
 * \return Requested master.
 */
ec_master_t *ecrt_request_master_err(
        unsigned int master_index /**< Master index. */
        )
{
    ec_master_t *master, *errptr = NULL;
    unsigned int dev_idx = EC_DEVICE_MAIN;

    EC_INFO("Requesting master %u...\n", master_index);

    if (master_index >= master_count) {
        EC_ERR("Invalid master index %u.\n", master_index);
        errptr = ERR_PTR(-EINVAL);
        goto out_return;
    }
    master = &masters[master_index];

    if (down_interruptible(&master_sem)) {
        errptr = ERR_PTR(-EINTR);
        goto out_return;
    }

    if (master->reserved) {
        up(&master_sem);
        EC_MASTER_ERR(master, "Master already in use!\n");
        errptr = ERR_PTR(-EBUSY);
        goto out_return;
    }
    master->reserved = 1;
    up(&master_sem);

    if (down_interruptible(&master->device_sem)) {
        errptr = ERR_PTR(-EINTR);
        goto out_release;
    }

    if (master->phase != EC_IDLE) {
        up(&master->device_sem);
        EC_MASTER_ERR(master, "Master still waiting for devices!\n");
        errptr = ERR_PTR(-ENODEV);
        goto out_release;
    }

    for (; dev_idx < ec_master_num_devices(master); dev_idx++) {
        ec_device_t *device = &master->devices[dev_idx];
        if (!try_module_get(device->module)) {
            up(&master->device_sem);
            EC_MASTER_ERR(master, "Device module is unloading!\n");
            errptr = ERR_PTR(-ENODEV);
            goto out_module_put;
        }
    }

    up(&master->device_sem);

    if (ec_master_enter_operation_phase(master)) {
        EC_MASTER_ERR(master, "Failed to enter OPERATION phase!\n");
        errptr = ERR_PTR(-EIO);
        goto out_module_put;
    }

    EC_INFO("Successfully requested master %u.\n", master_index);
    return master;

 out_module_put:
    for (; dev_idx > 0; dev_idx--) {
        ec_device_t *device = &master->devices[dev_idx - 1];
        module_put(device->module);
    }
 out_release:
    master->reserved = 0;
 out_return:
    return errptr;
}

/*****************************************************************************/

ec_master_t *ecrt_request_master(unsigned int master_index)
{
    ec_master_t *master = ecrt_request_master_err(master_index);
    return IS_ERR(master) ? NULL : master;
}

/*****************************************************************************/

void ecrt_release_master(ec_master_t *master)
{
    unsigned int dev_idx;

    EC_MASTER_INFO(master, "Releasing master...\n");

    if (!master->reserved) {
        EC_MASTER_WARN(master, "%s(): Master was was not requested!\n",
                __func__);
        return;
    }

    ec_master_leave_operation_phase(master);

    for (dev_idx = EC_DEVICE_MAIN; dev_idx < ec_master_num_devices(master);
            dev_idx++) {
        module_put(master->devices[dev_idx].module);
    }

    master->reserved = 0;

    EC_MASTER_INFO(master, "Released.\n");
}

/*****************************************************************************/

unsigned int ecrt_version_magic(void)
{
    return ECRT_VERSION_MAGIC;
}

/*****************************************************************************/

/** Global request state type translation table.
 *
 * Translates an internal request state to an external one.
 */
const ec_request_state_t ec_request_state_translation_table[] = {
    EC_REQUEST_UNUSED,  // EC_INT_REQUEST_INIT,
    EC_REQUEST_BUSY,    // EC_INT_REQUEST_QUEUED,
    EC_REQUEST_BUSY,    // EC_INT_REQUEST_BUSY,
    EC_REQUEST_SUCCESS, // EC_INT_REQUEST_SUCCESS,
    EC_REQUEST_ERROR    // EC_INT_REQUEST_FAILURE
};

/*****************************************************************************/

/** \cond */

module_init(ec_init_module);
module_exit(ec_cleanup_module);

EXPORT_SYMBOL(ecdev_offer);

EXPORT_SYMBOL(ecrt_request_master);
EXPORT_SYMBOL(ecrt_release_master);
EXPORT_SYMBOL(ecrt_version_magic);

/** \endcond */

/*****************************************************************************/
