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

/**
   \file
   Methods of an EtherCAT datagram.
*/

/*****************************************************************************/

#include <linux/slab.h>

#include "datagram.h"
#include "master.h"

/*****************************************************************************/

/** \cond */

#define EC_FUNC_HEADER \
    ret = ec_datagram_prealloc(datagram, data_size); \
    if (unlikely(ret)) \
        return ret; \
    datagram->index = 0; \
    datagram->working_counter = 0; \
    datagram->state = EC_DATAGRAM_INIT;

#define EC_FUNC_FOOTER \
    datagram->data_size = data_size; \
    return 0;

/** \endcond */

/*****************************************************************************/

/** Array of datagram type strings used in ec_datagram_type_string().
 *
 * \attention This is indexed by ec_datagram_type_t.
 */
static const char *type_strings[] = {
    "?",
    "APRD",
    "APWR",
    "APRW",
    "FPRD",
    "FPWR",
    "FPRW",
    "BRD",
    "BWR",
    "BRW",
    "LRD",
    "LWR",
    "LRW",
    "ARMW",
    "FRMW"
};

/*****************************************************************************/

/** Constructor.
 */
void ec_datagram_init(ec_datagram_t *datagram /**< EtherCAT datagram. */)
{
    INIT_LIST_HEAD(&datagram->queue); // mark as unqueued
    datagram->device_index = EC_DEVICE_MAIN;
    datagram->type = EC_DATAGRAM_NONE;
    memset(datagram->address, 0x00, EC_ADDR_LEN);
    datagram->data = NULL;
    datagram->data_origin = EC_ORIG_INTERNAL;
    datagram->mem_size = 0;
    datagram->data_size = 0;
    datagram->index = 0x00;
    datagram->working_counter = 0x0000;
    datagram->state = EC_DATAGRAM_INIT;
#ifdef EC_HAVE_CYCLES
    datagram->cycles_sent = 0;
#endif
    datagram->jiffies_sent = 0;
#ifdef EC_HAVE_CYCLES
    datagram->cycles_received = 0;
#endif
    datagram->jiffies_received = 0;
    datagram->skip_count = 0;
    datagram->stats_output_jiffies = 0;
    memset(datagram->name, 0x00, EC_DATAGRAM_NAME_SIZE);
}

/*****************************************************************************/

/** Destructor.
 */
void ec_datagram_clear(ec_datagram_t *datagram /**< EtherCAT datagram. */)
{
    ec_datagram_unqueue(datagram);

    if (datagram->data_origin == EC_ORIG_INTERNAL && datagram->data) {
        kfree(datagram->data);
        datagram->data = NULL;
    }
}

/*****************************************************************************/

/** Unqueue datagram.
 */
void ec_datagram_unqueue(ec_datagram_t *datagram /**< EtherCAT datagram. */)
{
    if (!list_empty(&datagram->queue)) {
        list_del_init(&datagram->queue);
    }
}

/*****************************************************************************/

/** Allocates internal payload memory.
 *
 * If the allocated memory is already larger than requested, nothing is done.
 *
 * \attention If external payload memory has been provided, no range checking
 *            is done!
 *
 * \return 0 in case of success, otherwise \a -ENOMEM.
 */
int ec_datagram_prealloc(
        ec_datagram_t *datagram, /**< EtherCAT datagram. */
        size_t size /**< New payload size in bytes. */
        )
{
    if (datagram->data_origin == EC_ORIG_EXTERNAL
            || size <= datagram->mem_size)
        return 0;

    if (datagram->data) {
        kfree(datagram->data);
        datagram->data = NULL;
        datagram->mem_size = 0;
    }

    if (!(datagram->data = kmalloc(size, GFP_KERNEL))) {
        EC_ERR("Failed to allocate %zu bytes of datagram memory!\n", size);
        return -ENOMEM;
    }

    datagram->mem_size = size;
    return 0;
}

/*****************************************************************************/

/** Fills the datagram payload memory with zeros.
 */
void ec_datagram_zero(ec_datagram_t *datagram /**< EtherCAT datagram. */)
{
    memset(datagram->data, 0x00, datagram->data_size);
}

/*****************************************************************************/

/** Initializes an EtherCAT APRD datagram.
 *
 * \return Return value of ec_datagram_prealloc().
 */
int ec_datagram_aprd(
        ec_datagram_t *datagram, /**< EtherCAT datagram. */
        uint16_t ring_position, /**< Auto-increment address. */
        uint16_t mem_address, /**< Physical memory address. */
        size_t data_size /**< Number of bytes to read. */
        )
{
    int ret;
    EC_FUNC_HEADER;
    datagram->type = EC_DATAGRAM_APRD;
    EC_WRITE_S16(datagram->address, (int16_t) ring_position * (-1));
    EC_WRITE_U16(datagram->address + 2, mem_address);
    EC_FUNC_FOOTER;
}

/*****************************************************************************/

/** Initializes an EtherCAT APWR datagram.
 *
 * \return Return value of ec_datagram_prealloc().
 */
/*
 * 参考手册：
 * APWR:Auto Increment Write.A slave increments the address.
 * A slave writes data to a memory area if the address received is zero.
 */
int ec_datagram_apwr(
        ec_datagram_t *datagram, /**< EtherCAT datagram. */
        uint16_t ring_position, /**< Auto-increment address. */
        uint16_t mem_address, /**< Physical memory address. */
        size_t data_size /**< Number of bytes to write. */
        )
{
    int ret;
    EC_FUNC_HEADER;
    datagram->type = EC_DATAGRAM_APWR;
    EC_WRITE_S16(datagram->address, (int16_t) ring_position * (-1));
    EC_WRITE_U16(datagram->address + 2, mem_address);
    EC_FUNC_FOOTER;
}

/*****************************************************************************/

/** Initializes an EtherCAT APRW datagram.
 *
 * \return Return value of ec_datagram_prealloc().
 */
int ec_datagram_aprw(
        ec_datagram_t *datagram, /**< EtherCAT datagram. */
        uint16_t ring_position, /**< Auto-increment address. */
        uint16_t mem_address, /**< Physical memory address. */
        size_t data_size /**< Number of bytes to write. */
        )
{
    int ret;
    EC_FUNC_HEADER;
    datagram->type = EC_DATAGRAM_APRW;
    EC_WRITE_S16(datagram->address, (int16_t) ring_position * (-1));
    EC_WRITE_U16(datagram->address + 2, mem_address);
    EC_FUNC_FOOTER;
}

/*****************************************************************************/

/** Initializes an EtherCAT ARMW datagram.
 *
 * \return Return value of ec_datagram_prealloc().
 */
int ec_datagram_armw(
        ec_datagram_t *datagram, /**< EtherCAT datagram. */
        uint16_t ring_position, /**< Auto-increment address. */
        uint16_t mem_address, /**< Physical memory address. */
        size_t data_size /**< Number of bytes to read. */
        )
{
    int ret;
    EC_FUNC_HEADER;
    datagram->type = EC_DATAGRAM_ARMW;
    EC_WRITE_S16(datagram->address, (int16_t) ring_position * (-1));
    EC_WRITE_U16(datagram->address + 2, mem_address);
    EC_FUNC_FOOTER;
}

/*****************************************************************************/

/** Initializes an EtherCAT FPRD datagram.
 *
 * \return Return value of ec_datagram_prealloc().
 */
int ec_datagram_fprd(
        ec_datagram_t *datagram, /**< EtherCAT datagram. */
        uint16_t configured_address, /**< Configured station address. */
        uint16_t mem_address, /**< Physical memory address. */
        size_t data_size /**< Number of bytes to read. */
        )
{
    int ret;

    if (unlikely(configured_address == 0x0000))
        EC_WARN("Using configured station address 0x0000!\n");

    EC_FUNC_HEADER;
    datagram->type = EC_DATAGRAM_FPRD;
    EC_WRITE_U16(datagram->address, configured_address);
    EC_WRITE_U16(datagram->address + 2, mem_address);
    EC_FUNC_FOOTER;
}

/*****************************************************************************/

/** Initializes an EtherCAT FPWR datagram.
 *
 * \return Return value of ec_datagram_prealloc().
 */
int ec_datagram_fpwr(
        ec_datagram_t *datagram, /**< EtherCAT datagram. */
        uint16_t configured_address, /**< Configured station address. */
        uint16_t mem_address, /**< Physical memory address. */
        size_t data_size /**< Number of bytes to write. */
        )
{
    int ret;

    if (unlikely(configured_address == 0x0000))
        EC_WARN("Using configured station address 0x0000!\n");

    EC_FUNC_HEADER;
    datagram->type = EC_DATAGRAM_FPWR;
    EC_WRITE_U16(datagram->address, configured_address);
    EC_WRITE_U16(datagram->address + 2, mem_address);
    EC_FUNC_FOOTER;
}

/*****************************************************************************/

/** Initializes an EtherCAT FPRW datagram.
 *
 * \return Return value of ec_datagram_prealloc().
 */
int ec_datagram_fprw(
        ec_datagram_t *datagram, /**< EtherCAT datagram. */
        uint16_t configured_address, /**< Configured station address. */
        uint16_t mem_address, /**< Physical memory address. */
        size_t data_size /**< Number of bytes to write. */
        )
{
    int ret;

    if (unlikely(configured_address == 0x0000))
        EC_WARN("Using configured station address 0x0000!\n");

    EC_FUNC_HEADER;
    datagram->type = EC_DATAGRAM_FPRW;
    EC_WRITE_U16(datagram->address, configured_address);
    EC_WRITE_U16(datagram->address + 2, mem_address);
    EC_FUNC_FOOTER;
}

/*****************************************************************************/

/** Initializes an EtherCAT FRMW datagram.
 *
 * \return Return value of ec_datagram_prealloc().
 */
int ec_datagram_frmw(
        ec_datagram_t *datagram, /**< EtherCAT datagram. */
        uint16_t configured_address, /**< Configured station address. */
        uint16_t mem_address, /**< Physical memory address. */
        size_t data_size /**< Number of bytes to write. */
        )
{
    int ret;

    if (unlikely(configured_address == 0x0000))
        EC_WARN("Using configured station address 0x0000!\n");

    EC_FUNC_HEADER;
    datagram->type = EC_DATAGRAM_FRMW;
    EC_WRITE_U16(datagram->address, configured_address);
    EC_WRITE_U16(datagram->address + 2, mem_address);
    EC_FUNC_FOOTER;
}

/*****************************************************************************/

/** Initializes an EtherCAT BRD datagram.
 *
 * \return Return value of ec_datagram_prealloc().
 */
/*
 * 报文格式(下面的图表在sourceinsight中会显示乱掉，可在linux下用vim看，windows下用notepad看)
 *
 * !!!注意！！！  报文左端为低字节，右端为高字节，以太网发送数据时，从最左端到最右段按bit
 * 发送，也就是说，低字节先发送。以ecat帧类型为例88是低字节先发送，A4是高字节后发送。
 *
 * ecat帧类型0x88A4通过eth->h_proto = htons(0x88A4)将本地字节序转换成了大端的网络字节序（高字
 * 节在地地址），即88存在低地址，A4存在高地址。
 *
 * 再以读从站状态为例，广播报文的ADO是0x0130，在下图中16bit ADO先发送低字节30，再发送高字节01。
 *
 * 再以2字节的Ethercat头为例，如果dmesg看的ecat头是0E 10，由于是低字节在前，所以2字节应该是
 * 10 0E（0001 0000 0000 1110），下图中，"长度"在左端，是低字节部分，所以长度=000 0000 1110= 14个字节。
 * 由于ethernet帧最小46个字节，所以需要做一些填充，最可dmesg能看到如下log:
 * [18773.590656] EtherCAT DEBUG 0: frame size: 46
 * [18773.590656] EtherCAT DEBUG 0: Sending frame:
 * [18773.590658] EtherCAT DEBUG: FF FF FF FF FF FF 6C 24 08 29 52 19 88 A4 0E 10
 * [18773.590664] EtherCAT DEBUG: 07 00 00 00 30 01 02 00 00 00 00 00 00 00 00 00
 * [18773.590669] EtherCAT DEBUG: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
 * [18773.590674] EtherCAT DEBUG: 00 00 00 00 00 00 00 00 00 00 00 00
 *
 * |            以太网帧头         |
 * |<----------------------------->|
 * |     6B    |    6B    |   2B   |      2B     |                44-1498B        |  4B  |
 * |___________|__________|________|_____________|________________________________|______|
 * |  目的地址 |   源地址 | 帧类型 | EtherCAT头  |               EtherCAT数据     |  FCS |
 * |___________|__________|________|_____________|________________________________|______|
 *                           88A4 /              \                                       \
 *                               /                \                                       \
 *                              /                  \                                       \
 *                             / 11bit  1bit   4bit \                                       \
 *                             ______________________________________________________________
 *                            | 长度 | 保留位 | 类型 |   子报文   |    子报文    |    ...   |
 *                            |______|________|______|____________|______________|__________|
 *                                                   /             \
 *                                                  /               \
 *                                     ---------- -/                 ----------------
  *                                   /    10B             最多1486B            2B   \
 *                                   _________________________________________________
 *                                   |   子报文头 |           数据         |    WKC  |
 *                                   |____________|________________________|_________|
 *                                  /              \
 *                                 /                \
 *           ----------------------                  ----------------------------
 *          / 8bit    8bit              32bit       11bit   4bit   1bit   16bit  \
 *          ______________________________________________________________________
 *         |  命令 |  索引 |         地址区       |  长度  |  R  |  M  |  状态位 |
 *         |_______|_______|______________________|________|_____|_____|_________|
 *                         /                       \
 *                        /                         \
 *                      ______________________________
 *                      |   16bit ADP  |   16bit ADO  |
 *                      |______________|______________|
 *   广播报文:
 *     ADP（Address Position），16位从站设备地址为0.
 *     ADO（Address Offset），16位从站设备内部寄存器地址.             
 */
int ec_datagram_brd(
        ec_datagram_t *datagram, /**< EtherCAT datagram. */
        uint16_t mem_address, /**< Physical memory address. */
        size_t data_size /**< Number of bytes to read. */
        )
{
    int ret;
    EC_FUNC_HEADER;
    datagram->type = EC_DATAGRAM_BRD;
    EC_WRITE_U16(datagram->address, 0x0000);
    EC_WRITE_U16(datagram->address + 2, mem_address);
    EC_FUNC_FOOTER;
}

/*****************************************************************************/

/** Initializes an EtherCAT BWR datagram.
 *
 * \return Return value of ec_datagram_prealloc().
 */
int ec_datagram_bwr(
        ec_datagram_t *datagram, /**< EtherCAT datagram. */
        uint16_t mem_address, /**< Physical memory address. */
        size_t data_size /**< Number of bytes to write. */
        )
{
    int ret;
    EC_FUNC_HEADER;
    datagram->type = EC_DATAGRAM_BWR;
    EC_WRITE_U16(datagram->address, 0x0000);
    EC_WRITE_U16(datagram->address + 2, mem_address);
    EC_FUNC_FOOTER;
}

/*****************************************************************************/

/** Initializes an EtherCAT BRW datagram.
 *
 * \return Return value of ec_datagram_prealloc().
 */
int ec_datagram_brw(
        ec_datagram_t *datagram, /**< EtherCAT datagram. */
        uint16_t mem_address, /**< Physical memory address. */
        size_t data_size /**< Number of bytes to write. */
        )
{
    int ret;
    EC_FUNC_HEADER;
    datagram->type = EC_DATAGRAM_BRW;
    EC_WRITE_U16(datagram->address, 0x0000);
    EC_WRITE_U16(datagram->address + 2, mem_address);
    EC_FUNC_FOOTER;
}

/*****************************************************************************/

/** Initializes an EtherCAT LRD datagram.
 *
 * \return Return value of ec_datagram_prealloc().
 */
int ec_datagram_lrd(
        ec_datagram_t *datagram, /**< EtherCAT datagram. */
        uint32_t offset, /**< Logical address. */
        size_t data_size /**< Number of bytes to read/write. */
        )
{
    int ret;
    EC_FUNC_HEADER;
    datagram->type = EC_DATAGRAM_LRD;
    EC_WRITE_U32(datagram->address, offset);
    EC_FUNC_FOOTER;
}

/*****************************************************************************/

/** Initializes an EtherCAT LWR datagram.
 *
 * \return Return value of ec_datagram_prealloc().
 */
int ec_datagram_lwr(
        ec_datagram_t *datagram, /**< EtherCAT datagram. */
        uint32_t offset, /**< Logical address. */
        size_t data_size /**< Number of bytes to read/write. */
        )
{
    int ret;
    EC_FUNC_HEADER;
    datagram->type = EC_DATAGRAM_LWR;
    EC_WRITE_U32(datagram->address, offset);
    EC_FUNC_FOOTER;
}

/*****************************************************************************/

/** Initializes an EtherCAT LRW datagram.
 *
 * \return Return value of ec_datagram_prealloc().
 */
int ec_datagram_lrw(
        ec_datagram_t *datagram, /**< EtherCAT datagram. */
        uint32_t offset, /**< Logical address. */
        size_t data_size /**< Number of bytes to read/write. */
        )
{
    int ret;
    EC_FUNC_HEADER;
    datagram->type = EC_DATAGRAM_LRW;
    EC_WRITE_U32(datagram->address, offset);
    EC_FUNC_FOOTER;
}

/*****************************************************************************/

/** Initializes an EtherCAT LRD datagram with external memory.
 *
 * \attention It is assumed, that the external memory is at least \a data_size
 *            bytes large.
 *
 * \return Return value of ec_datagram_prealloc().
 */
int ec_datagram_lrd_ext(
        ec_datagram_t *datagram, /**< EtherCAT datagram. */
        uint32_t offset, /**< Logical address. */
        size_t data_size, /**< Number of bytes to read/write. */
        uint8_t *external_memory /**< Pointer to the memory to use. */
        )
{
    int ret;
    datagram->data = external_memory;
    datagram->data_origin = EC_ORIG_EXTERNAL;
    EC_FUNC_HEADER;
    datagram->type = EC_DATAGRAM_LRD;
    EC_WRITE_U32(datagram->address, offset);
    EC_FUNC_FOOTER;
}

/*****************************************************************************/

/** Initializes an EtherCAT LWR datagram with external memory.
 *
 * \attention It is assumed, that the external memory is at least \a data_size
 *            bytes large.
 *
 * \return Return value of ec_datagram_prealloc().
 */
int ec_datagram_lwr_ext(
        ec_datagram_t *datagram, /**< EtherCAT datagram. */
        uint32_t offset, /**< Logical address. */
        size_t data_size, /**< Number of bytes to read/write. */
        uint8_t *external_memory /**< Pointer to the memory to use. */
        )
{
    int ret;
    datagram->data = external_memory;
    datagram->data_origin = EC_ORIG_EXTERNAL;
    EC_FUNC_HEADER;
    datagram->type = EC_DATAGRAM_LWR;
    EC_WRITE_U32(datagram->address, offset);
    EC_FUNC_FOOTER;
}

/*****************************************************************************/

/** Initializes an EtherCAT LRW datagram with external memory.
 *
 * \attention It is assumed, that the external memory is at least \a data_size
 *            bytes large.
 *
 * \return Return value of ec_datagram_prealloc().
 */
int ec_datagram_lrw_ext(
        ec_datagram_t *datagram, /**< EtherCAT datagram. */
        uint32_t offset, /**< Logical address. */
        size_t data_size, /**< Number of bytes to read/write. */
        uint8_t *external_memory /**< Pointer to the memory to use. */
        )
{
    int ret;
    datagram->data = external_memory;
    datagram->data_origin = EC_ORIG_EXTERNAL;
    EC_FUNC_HEADER;
    datagram->type = EC_DATAGRAM_LRW;
    EC_WRITE_U32(datagram->address, offset);
    EC_FUNC_FOOTER;
}

/*****************************************************************************/

/** Prints the state of a datagram.
 *
 * Outputs a text message.
 */
void ec_datagram_print_state(
        const ec_datagram_t *datagram /**< EtherCAT datagram */
        )
{
    printk(KERN_CONT "Datagram ");
    switch (datagram->state) {
        case EC_DATAGRAM_INIT:
            printk(KERN_CONT "initialized");
            break;
        case EC_DATAGRAM_QUEUED:
            printk(KERN_CONT "queued");
            break;
        case EC_DATAGRAM_SENT:
            printk(KERN_CONT "sent");
            break;
        case EC_DATAGRAM_RECEIVED:
            printk(KERN_CONT "received");
            break;
        case EC_DATAGRAM_TIMED_OUT:
            printk(KERN_CONT "timed out");
            break;
        case EC_DATAGRAM_ERROR:
            printk(KERN_CONT "error");
            break;
        default:
            printk(KERN_CONT "???");
    }

    printk(KERN_CONT ".\n");
}

/*****************************************************************************/

/** Evaluates the working counter of a single-cast datagram.
 *
 * Outputs an error message.
 */
void ec_datagram_print_wc_error(
        const ec_datagram_t *datagram /**< EtherCAT datagram */
        )
{
    if (datagram->working_counter == 0) {
        printk(KERN_CONT "No response.");
    }
    else if (datagram->working_counter > 1) {
        printk(KERN_CONT "%u slaves responded!", datagram->working_counter);
    }
    else {
        printk(KERN_CONT "Success.");
    }
    printk(KERN_CONT "\n");
}

/*****************************************************************************/

/** Outputs datagram statistics at most every second.
 */
void ec_datagram_output_stats(
        ec_datagram_t *datagram
        )
{
    if (jiffies - datagram->stats_output_jiffies > HZ) {
        datagram->stats_output_jiffies = jiffies;

        if (unlikely(datagram->skip_count)) {
            EC_WARN("Datagram %p (%s) was SKIPPED %u time%s.\n",
                    datagram, datagram->name,
                    datagram->skip_count,
                    datagram->skip_count == 1 ? "" : "s");
            datagram->skip_count = 0;
        }
    }
}

/*****************************************************************************/

/** Returns a string describing the datagram type.
 *
 * \return Pointer on a static memory containing the requested string.
 */
const char *ec_datagram_type_string(
        const ec_datagram_t *datagram /**< EtherCAT datagram. */
        )
{
    return type_strings[datagram->type];
}

/*****************************************************************************/
