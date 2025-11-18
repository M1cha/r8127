// SPDX-License-Identifier: GPL-2.0-only
/*
################################################################################
#
# r8127 is the Linux device driver released for Realtek 10 Gigabit Ethernet
# controllers with PCI-Express interface.
#
# Copyright(c) 2025 Realtek Semiconductor Corp. All rights reserved.
#
# This program is free software; you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation; either version 2 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along with
# this program; if not, see <http://www.gnu.org/licenses/>.
#
# Author:
# Realtek NIC software team <nicfae@realtek.com>
# No. 2, Innovation Road II, Hsinchu Science Park, Hsinchu 300, Taiwan
#
################################################################################
*/

/************************************************************************************
 *  This product is covered by one or more of the following patents:
 *  US6,570,884, US6,115,776, and US6,327,625.
 ***********************************************************************************/

/*
 * This driver is modified from r8169.c in Linux kernel 2.6.18
 */

#include <linux/module.h>
#include <linux/version.h>
#include <linux/pci.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/delay.h>
#include <linux/mii.h>
#include <linux/if_vlan.h>
#include <linux/crc32.h>
#include <linux/interrupt.h>
#include <linux/in.h>
#include <linux/ip.h>
#include <linux/ipv6.h>
#include <net/ip6_checksum.h>
#include <linux/tcp.h>
#include <linux/init.h>
#include <linux/rtnetlink.h>
#include <linux/completion.h>
#include <linux/prefetch.h>
#include <linux/dma-mapping.h>
#include <linux/moduleparam.h>
#include <linux/mdio.h>
#include <net/gso.h>

#include <asm/io.h>
#include <asm/irq.h>

#include "r8127.h"
#include "rtl_eeprom.h"
#include "rtltool.h"
#include "r8127_firmware.h"

#define _R(NAME,MAC,RCR,MASK,JumFrameSz) \
    { .name = NAME, .mcfg = MAC, .RCR_Cfg = RCR, .RxConfigMask = MASK, .jumbo_frame_sz = JumFrameSz }

static const struct {
        const char *name;
        u8 mcfg;
        u32 RCR_Cfg;
        u32 RxConfigMask;   /* Clears the bits supported by this chip */
        u32 jumbo_frame_sz;
} rtl_chip_info[] = {
        _R("RTL8127",
        CFG_METHOD_1,
        Rx_Fetch_Number_8 | Rx_Close_Multiple | RxCfg_pause_slot_en | EnableInnerVlan | EnableOuterVlan | (RX_DMA_BURST_512 << RxCfgDMAShift),
        0xff7e5880,
        Jumbo_Frame_9k),

        _R("RTL8127",
        CFG_METHOD_2,
        Rx_Fetch_Number_8 | Rx_Close_Multiple | RxCfg_pause_slot_en | EnableInnerVlan | EnableOuterVlan | (RX_DMA_BURST_512 << RxCfgDMAShift),
        0xff7e5880,
        Jumbo_Frame_9k),

        _R("Unknown",
        CFG_METHOD_DEFAULT,
        (RX_DMA_BURST_512 << RxCfgDMAShift),
        0xff7e5880,
        Jumbo_Frame_1k)
};
#undef _R


static struct pci_device_id rtl8127_pci_tbl[] = {
        { PCI_DEVICE(PCI_VENDOR_ID_REALTEK, 0x8127), },
        { PCI_DEVICE(PCI_VENDOR_ID_REALTEK, 0x0E10), },
        {0,},
};

MODULE_DEVICE_TABLE(pci, rtl8127_pci_tbl);

static int use_dac = 1;
static int timer_count = 0x2600;
static int timer_count_v2 = (0x2600 / 0x200);

static struct {
        u32 msg_enable;
} debug = { -1 };

static unsigned int speed_mode = SPEED_10000;
static unsigned int duplex_mode = DUPLEX_FULL;
static unsigned int autoneg_mode = AUTONEG_ENABLE;
#ifdef CONFIG_ASPM
static int aspm = 1;
#else
static int aspm = 0;
#endif
#ifdef ENABLE_S5WOL
static int s5wol = 1;
#else
static int s5wol = 0;
#endif
#ifdef ENABLE_S5_KEEP_CURR_MAC
static int s5_keep_curr_mac = 1;
#else
static int s5_keep_curr_mac = 0;
#endif
#ifdef ENABLE_EEE
static int eee_enable = 1;
#else
static int eee_enable = 0;
#endif
#ifdef CONFIG_SOC_LAN
static ulong hwoptimize = HW_PATCH_SOC_LAN;
#else
static ulong hwoptimize = 0;
#endif
#ifdef ENABLE_S0_MAGIC_PACKET
static int s0_magic_packet = 1;
#else
static int s0_magic_packet = 0;
#endif
#ifdef ENABLE_TX_NO_CLOSE
static int tx_no_close_enable = 1;
#else
static int tx_no_close_enable = 0;
#endif
#ifdef DISABLE_WOL_SUPPORT
static int disable_wol_support = 1;
#else
static int disable_wol_support = 0;
#endif
#ifdef ENABLE_DOUBLE_VLAN
static int enable_double_vlan = 1;
#else
static int enable_double_vlan = 0;
#endif
#ifdef ENABLE_GIGA_LITE
static int eee_giga_lite = 1;
#else
static int eee_giga_lite = 0;
#endif

MODULE_AUTHOR("Realtek and the Linux r8127 crew <netdev@vger.kernel.org>");
MODULE_DESCRIPTION("Realtek r8127 Ethernet controller driver");

module_param(speed_mode, uint, 0);
MODULE_PARM_DESC(speed_mode, "force phy operation. Deprecated by ethtool (8).");

module_param(duplex_mode, uint, 0);
MODULE_PARM_DESC(duplex_mode, "force phy operation. Deprecated by ethtool (8).");

module_param(autoneg_mode, uint, 0);
MODULE_PARM_DESC(autoneg_mode, "force phy operation. Deprecated by ethtool (8).");

module_param(aspm, int, 0);
MODULE_PARM_DESC(aspm, "Enable ASPM.");

module_param(s5wol, int, 0);
MODULE_PARM_DESC(s5wol, "Enable Shutdown Wake On Lan.");

module_param(s5_keep_curr_mac, int, 0);
MODULE_PARM_DESC(s5_keep_curr_mac, "Enable Shutdown Keep Current MAC Address.");

module_param(use_dac, int, 0);
MODULE_PARM_DESC(use_dac, "Enable PCI DAC. Unsafe on 32 bit PCI slot.");

module_param(timer_count, int, 0);
MODULE_PARM_DESC(timer_count, "Timer Interrupt Interval.");

module_param(eee_enable, int, 0);
MODULE_PARM_DESC(eee_enable, "Enable Energy Efficient Ethernet.");

module_param(hwoptimize, ulong, 0);
MODULE_PARM_DESC(hwoptimize, "Enable HW optimization function.");

module_param(s0_magic_packet, int, 0);
MODULE_PARM_DESC(s0_magic_packet, "Enable S0 Magic Packet.");

module_param(tx_no_close_enable, int, 0);
MODULE_PARM_DESC(tx_no_close_enable, "Enable TX No Close.");

module_param(disable_wol_support, int, 0);
MODULE_PARM_DESC(disable_wol_support, "Disable PM support.");

module_param(enable_double_vlan, int, 0);
MODULE_PARM_DESC(enable_double_vlan, "Enable Double VLAN.");

module_param(eee_giga_lite, int, 0);
MODULE_PARM_DESC(eee_giga_lite, "Enable Giga Lite.");

module_param_named(debug, debug.msg_enable, int, 0);
MODULE_PARM_DESC(debug, "Debug verbosity level (0=none, ..., 16=all)");

MODULE_LICENSE("GPL");

MODULE_VERSION(RTL8127_VERSION);

static netdev_tx_t rtl8127_start_xmit(struct sk_buff *skb, struct net_device *dev);
static irqreturn_t rtl8127_interrupt(int irq, void *dev_instance);
static irqreturn_t rtl8127_interrupt_msix(int irq, void *dev_instance);
static void rtl8127_set_rx_mode(struct net_device *dev);
static void rtl8127_tx_timeout(struct net_device *dev, unsigned int txqueue);
static int rtl8127_rx_interrupt(struct net_device *, struct rtl8127_private *, struct rtl8127_rx_ring *, napi_budget);
static int rtl8127_tx_interrupt(struct rtl8127_tx_ring *ring, int budget);
static int rtl8127_tx_interrupt_with_vector(struct rtl8127_private *tp, const int message_id, int budget);
static void rtl8127_wait_for_quiescence(struct net_device *dev);
static int rtl8127_change_mtu(struct net_device *dev, int new_mtu);
static void rtl8127_down(struct net_device *dev);

static int rtl8127_set_mac_address(struct net_device *dev, void *p);
static void rtl8127_rar_set(struct rtl8127_private *tp, const u8 *addr);
static void rtl8127_desc_addr_fill(struct rtl8127_private *);
static void rtl8127_tx_desc_init(struct rtl8127_private *tp);
static void rtl8127_rx_desc_init(struct rtl8127_private *tp);

static u16 rtl8127_get_hw_phy_mcu_code_ver(struct rtl8127_private *tp);
static void rtl8127_phy_power_up(struct net_device *dev);
static void rtl8127_phy_power_down(struct net_device *dev);
static int rtl8127_set_speed(struct net_device *dev, u8 autoneg, u32 speed, u8 duplex, u64 adv);
static bool rtl8127_set_phy_mcu_patch_request(struct rtl8127_private *tp);
static bool rtl8127_clear_phy_mcu_patch_request(struct rtl8127_private *tp);

static int rtl8127_poll(napi_ptr napi, napi_budget budget);

static void rtl8127_reset_task(struct work_struct *work);
static void rtl8127_esd_task(struct work_struct *work);
static void rtl8127_linkchg_task(struct work_struct *work);
static void rtl8127_dash_task(struct work_struct *work);
static void rtl8127_schedule_reset_work(struct rtl8127_private *tp);
static void rtl8127_schedule_esd_work(struct rtl8127_private *tp);
static void rtl8127_schedule_linkchg_work(struct rtl8127_private *tp);
void rtl8127_schedule_dash_work(struct rtl8127_private *tp);
static void rtl8127_init_all_schedule_work(struct rtl8127_private *tp);
static void rtl8127_cancel_all_schedule_work(struct rtl8127_private *tp);

static inline struct device *tp_to_dev(struct rtl8127_private *tp)
{
        return &tp->pci_dev->dev;
}

#ifndef SET_ETHTOOL_OPS
#define SET_ETHTOOL_OPS(netdev,ops) \
         ((netdev)->ethtool_ops = (ops))
#endif //SET_ETHTOOL_OPS

#ifndef netif_msg_init
#define netif_msg_init _kc_netif_msg_init
/* copied from linux kernel 2.6.20 include/linux/netdevice.h */
static inline u32 netif_msg_init(int debug_value, int default_msg_enable_bits)
{
        /* use default */
        if (debug_value < 0 || debug_value >= (sizeof(u32) * 8))
                return default_msg_enable_bits;
        if (debug_value == 0)   /* no output */
                return 0;
        /* set low N bits */
        return (1 << debug_value) - 1;
}

#endif //netif_msg_init

static inline void eth_copy_and_sum (struct sk_buff *dest,
                                     const unsigned char *src,
                                     int len, int base)
{
        skb_copy_to_linear_data(dest, src, len);
}

int rtl8127_dump_tally_counter(struct rtl8127_private *tp, dma_addr_t paddr)
{
        u32 cmd;
        u32 WaitCnt;
        int retval = -1;

        RTL_W32(tp, CounterAddrHigh, (u64)paddr >> 32);
        cmd = (u64)paddr & DMA_BIT_MASK(32);
        RTL_W32(tp, CounterAddrLow, cmd);
        RTL_W32(tp, CounterAddrLow, cmd | CounterDump);

        WaitCnt = 0;
        while (RTL_R32(tp, CounterAddrLow) & CounterDump) {
                udelay(10);

                WaitCnt++;
                if (WaitCnt > 20)
                        break;
        }

        if (WaitCnt <= 20)
                retval = 0;

        return retval;
}

static u32
rtl8127_get_hw_clo_ptr(struct rtl8127_tx_ring *ring)
{
        struct rtl8127_private *tp = ring->priv;

        switch (tp->HwSuppTxNoCloseVer) {
        case 3:
                return RTL_R16(tp, ring->hw_clo_ptr_reg);
        case 4:
        case 5:
        case 6:
                return RTL_R32(tp, ring->hw_clo_ptr_reg);
        default:
                WARN_ON(1);
                return 0;
        }
}

static u32
rtl8127_get_phy_status(struct rtl8127_private *tp)
{
        return RTL_R32(tp, PHYstatus);
}

static u32 rtl8127_convert_link_speed(u32 status)
{
        u32 speed = SPEED_UNKNOWN;

        if (status & LinkStatus) {
                if (status & _10000bpsF)
                        speed = SPEED_10000;
                else if (status & (_5000bpsF | _10000bpsL))
                        speed = SPEED_5000;
                else if (status & (_2500bpsF | _5000bpsL))
                        speed = SPEED_2500;
                else if (status & (_1000bpsF | _2500bpsL | _1000bpsL))
                        speed = SPEED_1000;
                else if (status & _100bps)
                        speed = SPEED_100;
                else if (status & _10bps)
                        speed = SPEED_10;
        }

        return speed;
}

static inline u16 map_phy_ocp_addr(u16 PageNum, u8 RegNum)
{
        u16 OcpPageNum = 0;
        u8 OcpRegNum = 0;
        u16 OcpPhyAddress = 0;

        if (PageNum == 0) {
                OcpPageNum = OCP_STD_PHY_BASE_PAGE + (RegNum / 8);
                OcpRegNum = 0x10 + (RegNum % 8);
        } else {
                OcpPageNum = PageNum;
                OcpRegNum = RegNum;
        }

        OcpPageNum <<= 4;

        if (OcpRegNum < 16) {
                OcpPhyAddress = 0;
        } else {
                OcpRegNum -= 16;
                OcpRegNum <<= 1;

                OcpPhyAddress = OcpPageNum + OcpRegNum;
        }


        return OcpPhyAddress;
}

static void mdio_real_direct_write_phy_ocp(struct rtl8127_private *tp,
                u16 RegAddr,
                u16 value)
{
        u32 data32;
        int i;

        WARN_ON_ONCE(RegAddr % 2);
        data32 = RegAddr/2;
        data32 <<= OCPR_Addr_Reg_shift;
        data32 |= OCPR_Write | value;

        RTL_W32(tp, PHYOCP, data32);
        for (i = 0; i < R8127_CHANNEL_WAIT_COUNT; i++) {
                udelay(R8127_CHANNEL_WAIT_TIME);

                if (!(RTL_R32(tp, PHYOCP) & OCPR_Flag))
                        break;
        }
}

void rtl8127_mdio_direct_write_phy_ocp(struct rtl8127_private *tp,
                                       u16 RegAddr,
                                       u16 value)
{
        if (tp->rtk_enable_diag)
                return;

        mdio_real_direct_write_phy_ocp(tp, RegAddr, value);
}

static void rtl8127_mdio_real_write_phy_ocp(struct rtl8127_private *tp,
                u16 PageNum,
                u32 RegAddr,
                u32 value)
{
        u16 ocp_addr;

        ocp_addr = map_phy_ocp_addr(PageNum, RegAddr);

        mdio_real_direct_write_phy_ocp(tp, ocp_addr, value);
}

static void mdio_real_write(struct rtl8127_private *tp,
                            u16 RegAddr,
                            u16 value)
{
        if (RegAddr == 0x1F) {
                tp->cur_page = value;
                return;
        }
        rtl8127_mdio_real_write_phy_ocp(tp, tp->cur_page, RegAddr, value);
}

void rtl8127_mdio_write(struct rtl8127_private *tp,
                        u16 RegAddr,
                        u16 value)
{
        if (tp->rtk_enable_diag)
                return;

        mdio_real_write(tp, RegAddr, value);
}

void rtl8127_mdio_prot_write(struct rtl8127_private *tp,
                             u32 RegAddr,
                             u32 value)
{
        mdio_real_write(tp, RegAddr, value);
}

void rtl8127_mdio_prot_direct_write_phy_ocp(struct rtl8127_private *tp,
                u32 RegAddr,
                u32 value)
{
        mdio_real_direct_write_phy_ocp(tp, RegAddr, value);
}

static u32 mdio_real_direct_read_phy_ocp(struct rtl8127_private *tp,
                u16 RegAddr)
{
        u32 data32;
        int i, value = 0;

        WARN_ON_ONCE(RegAddr % 2);
        data32 = RegAddr/2;
        data32 <<= OCPR_Addr_Reg_shift;

        RTL_W32(tp, PHYOCP, data32);
        for (i = 0; i < R8127_CHANNEL_WAIT_COUNT; i++) {
                udelay(R8127_CHANNEL_WAIT_TIME);

                if (RTL_R32(tp, PHYOCP) & OCPR_Flag)
                        break;
        }
        value = RTL_R32(tp, PHYOCP) & OCPDR_Data_Mask;

        return value;
}

u32 rtl8127_mdio_direct_read_phy_ocp(struct rtl8127_private *tp,
                                     u16 RegAddr)
{
        if (tp->rtk_enable_diag)
                return 0xffffffff;

        return mdio_real_direct_read_phy_ocp(tp, RegAddr);
}

static u32 rtl8127_mdio_real_read_phy_ocp(struct rtl8127_private *tp,
                u16 PageNum,
                u32 RegAddr)
{
        u16 ocp_addr;

        ocp_addr = map_phy_ocp_addr(PageNum, RegAddr);

        return mdio_real_direct_read_phy_ocp(tp, ocp_addr);
}

static u32 mdio_real_read(struct rtl8127_private *tp,
                          u16 RegAddr)
{
        return rtl8127_mdio_real_read_phy_ocp(tp, tp->cur_page, RegAddr);
}

u32 rtl8127_mdio_read(struct rtl8127_private *tp,
                      u16 RegAddr)
{
        if (tp->rtk_enable_diag)
                return 0xffffffff;

        return mdio_real_read(tp, RegAddr);
}

u32 rtl8127_mdio_prot_read(struct rtl8127_private *tp,
                           u32 RegAddr)
{
        return mdio_real_read(tp, RegAddr);
}

u32 rtl8127_mdio_prot_direct_read_phy_ocp(struct rtl8127_private *tp,
                u32 RegAddr)
{
        return mdio_real_direct_read_phy_ocp(tp, RegAddr);
}

static void rtl8127_clear_and_set_eth_phy_bit(struct rtl8127_private *tp, u8  addr, u16 clearmask, u16 setmask)
{
        u16 PhyRegValue;

        PhyRegValue = rtl8127_mdio_read(tp, addr);
        PhyRegValue &= ~clearmask;
        PhyRegValue |= setmask;
        rtl8127_mdio_write(tp, addr, PhyRegValue);
}

void rtl8127_clear_eth_phy_bit(struct rtl8127_private *tp, u8 addr, u16 mask)
{
        rtl8127_clear_and_set_eth_phy_bit(tp,
                                          addr,
                                          mask,
                                          0);
}

void rtl8127_set_eth_phy_bit(struct rtl8127_private *tp,  u8  addr, u16  mask)
{
        rtl8127_clear_and_set_eth_phy_bit(tp,
                                          addr,
                                          0,
                                          mask);
}

void rtl8127_clear_and_set_eth_phy_ocp_bit(struct rtl8127_private *tp, u16 addr, u16 clearmask, u16 setmask)
{
        u16 PhyRegValue;

        PhyRegValue = rtl8127_mdio_direct_read_phy_ocp(tp, addr);
        PhyRegValue &= ~clearmask;
        PhyRegValue |= setmask;
        rtl8127_mdio_direct_write_phy_ocp(tp, addr, PhyRegValue);
}

void rtl8127_clear_eth_phy_ocp_bit(struct rtl8127_private *tp, u16 addr, u16 mask)
{
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              addr,
                                              mask,
                                              0);
}

void rtl8127_set_eth_phy_ocp_bit(struct rtl8127_private *tp,  u16 addr, u16 mask)
{
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              addr,
                                              0,
                                              mask);
}

void rtl8127_mac_ocp_write(struct rtl8127_private *tp, u16 reg_addr, u16 value)
{
        u32 data32;

        WARN_ON_ONCE(reg_addr % 2);

        data32 = reg_addr/2;
        data32 <<= OCPR_Addr_Reg_shift;
        data32 += value;
        data32 |= OCPR_Write;

        RTL_W32(tp, MACOCP, data32);
}

u16 rtl8127_mac_ocp_read(struct rtl8127_private *tp, u16 reg_addr)
{
        u32 data32;
        u16 data16 = 0;

        WARN_ON_ONCE(reg_addr % 2);

        data32 = reg_addr/2;
        data32 <<= OCPR_Addr_Reg_shift;

        RTL_W32(tp, MACOCP, data32);
        data16 = (u16)RTL_R32(tp, MACOCP);

        return data16;
}

static void
rtl8127_clear_set_mac_ocp_bit(
        struct rtl8127_private *tp,
        u16   addr,
        u16   clearmask,
        u16   setmask
)
{
        u16 PhyRegValue;

        PhyRegValue = rtl8127_mac_ocp_read(tp, addr);
        PhyRegValue &= ~clearmask;
        PhyRegValue |= setmask;
        rtl8127_mac_ocp_write(tp, addr, PhyRegValue);
}

void
rtl8127_clear_mac_ocp_bit(
        struct rtl8127_private *tp,
        u16   addr,
        u16   mask
)
{
        rtl8127_clear_set_mac_ocp_bit(tp,
                                      addr,
                                      mask,
                                      0);
}

static void
rtl8127_set_mac_ocp_bit(
        struct rtl8127_private *tp,
        u16   addr,
        u16   mask
)
{
        rtl8127_clear_set_mac_ocp_bit(tp,
                                      addr,
                                      0,
                                      mask);
}

u32 rtl8127_ocp_read_with_oob_base_address(struct rtl8127_private *tp, u16 addr, u8 len, const u32 base_address)
{
        return rtl8127_eri_read_with_oob_base_address(tp, addr, len, ERIAR_OOB, base_address);
}

u32 rtl8127_ocp_read(struct rtl8127_private *tp, u16 addr, u8 len)
{
        if (!tp->AllowAccessDashOcp || tp->HwSuppOcpChannelVer != 2)
                return 0xffffffff;

        return rtl8127_ocp_read_with_oob_base_address(tp, addr, len,
                        NO_BASE_ADDRESS);
}

u32 rtl8127_ocp_write_with_oob_base_address(struct rtl8127_private *tp, u16 addr, u8 len, u32 value, const u32 base_address)
{
        return rtl8127_eri_write_with_oob_base_address(tp, addr, len, value,
                        ERIAR_OOB, base_address);
}

void rtl8127_ocp_write(struct rtl8127_private *tp, u16 addr, u8 len, u32 value)
{
        if (!tp->AllowAccessDashOcp || tp->HwSuppOcpChannelVer != 2)
                return;

        rtl8127_ocp_write_with_oob_base_address(tp, addr, len, value, NO_BASE_ADDRESS);
}

void rtl8127_oob_mutex_lock(struct rtl8127_private *tp)
{
        u8 reg_16, reg_a0;
        u32 wait_cnt_0, wait_Cnt_1;
        u16 ocp_reg_mutex_ib;
        u16 ocp_reg_mutex_oob;
        u16 ocp_reg_mutex_prio;

        if (!HW_DASH_SUPPORT_DASH(tp))
                return;

        if (!tp->DASH)
                return;

        ocp_reg_mutex_oob = 0x110;
        ocp_reg_mutex_ib = 0x114;
        ocp_reg_mutex_prio = 0x11C;

        rtl8127_ocp_write(tp, ocp_reg_mutex_ib, 1, BIT_0);
        reg_16 = rtl8127_ocp_read(tp, ocp_reg_mutex_oob, 1);
        wait_cnt_0 = 0;
        while(reg_16) {
                reg_a0 = rtl8127_ocp_read(tp, ocp_reg_mutex_prio, 1);
                if (reg_a0) {
                        rtl8127_ocp_write(tp, ocp_reg_mutex_ib, 1, 0x00);
                        reg_a0 = rtl8127_ocp_read(tp, ocp_reg_mutex_prio, 1);
                        wait_Cnt_1 = 0;
                        while(reg_a0) {
                                reg_a0 = rtl8127_ocp_read(tp, ocp_reg_mutex_prio, 1);

                                wait_Cnt_1++;

                                if (wait_Cnt_1 > 2000)
                                        break;
                        };
                        rtl8127_ocp_write(tp, ocp_reg_mutex_ib, 1, BIT_0);

                }
                reg_16 = rtl8127_ocp_read(tp, ocp_reg_mutex_oob, 1);

                wait_cnt_0++;

                if (wait_cnt_0 > 2000)
                        break;
        };
}

void rtl8127_oob_mutex_unlock(struct rtl8127_private *tp)
{
        u16 ocp_reg_mutex_ib;
        u16 ocp_reg_mutex_prio;

        if (!HW_DASH_SUPPORT_DASH(tp))
                return;

        if (!tp->DASH)
                return;

        ocp_reg_mutex_ib = 0x114;
        ocp_reg_mutex_prio = 0x11C;

        rtl8127_ocp_write(tp, ocp_reg_mutex_prio, 1, BIT_0);
        rtl8127_ocp_write(tp, ocp_reg_mutex_ib, 1, 0x00);
}

static bool
rtl8127_is_allow_access_dash_ocp(struct rtl8127_private *tp)
{
        bool allow_access = false;
        u16 mac_ocp_data;

        if (!HW_DASH_SUPPORT_DASH(tp))
                goto exit;

        allow_access = true;
        switch (tp->mcfg) {
        case CFG_METHOD_2:
                mac_ocp_data = rtl8127_mac_ocp_read(tp, 0xd4c0);
                if (mac_ocp_data == 0xffff || (mac_ocp_data & BIT_3))
                        allow_access = false;
                break;
        default:
                goto exit;
        }
exit:
        return allow_access;
}

static u32 rtl8127_get_dash_fw_ver(struct rtl8127_private *tp)
{
        u32 ver = 0xffffffff;

        if (FALSE == HW_DASH_SUPPORT_GET_FIRMWARE_VERSION(tp))
                goto exit;

        ver = rtl8127_ocp_read(tp, OCP_REG_FIRMWARE_MAJOR_VERSION, 4);

exit:
        return ver;
}

static int _rtl8127_check_dash(struct rtl8127_private *tp)
{
        if (!tp->AllowAccessDashOcp)
                return 0;

        if (!HW_DASH_SUPPORT_IPC2(tp))
                return 0;

        if (rtl8127_ocp_read(tp, 0x128, 1) & BIT_0)
                return 1;

        return 0;
}

static int rtl8127_check_dash(struct rtl8127_private *tp)
{
        if (HW_DASH_SUPPORT_DASH(tp) && _rtl8127_check_dash(tp)) {
                u32 ver = rtl8127_get_dash_fw_ver(tp);
                if (!(ver == 0 || ver == 0xffffffff))
                        return 1;
        }

        return 0;
}

static int rtl8127_wait_dash_fw_ready(struct rtl8127_private *tp)
{
        int rc = -1;
        int timeout;

        if (!tp->DASH)
                goto out;

        for (timeout = 0; timeout < 10; timeout++) {
                fsleep(10000);
                if (rtl8127_ocp_read(tp, 0x124, 1) & BIT_0) {
                        rc = 1;
                        goto out;
                }
        }

        rc = 0;

out:
        return rc;
}

static void
rtl8127_notify_dash_oob_ipc2(struct rtl8127_private *tp, u32 cmd)
{
        if (FALSE == HW_DASH_SUPPORT_IPC2(tp))
                return;

        rtl8127_ocp_write(tp, IPC2_IB2SOC_DATA, 4, cmd);
        rtl8127_ocp_write(tp, IPC2_IB2SOC_CMD, 4, 0x00);
        rtl8127_ocp_write(tp, IPC2_IB2SOC_SET, 4, 0x01);
}

static void
rtl8127_notify_dash_oob(struct rtl8127_private *tp, u32 cmd)
{
        if (HW_DASH_SUPPORT_IPC2(tp))
                return rtl8127_notify_dash_oob_ipc2(tp, cmd);
        else
                return;
}

static void rtl8127_driver_start(struct rtl8127_private *tp)
{
        if (!tp->AllowAccessDashOcp)
                return;

        rtl8127_notify_dash_oob(tp, OOB_CMD_DRIVER_START);

        rtl8127_wait_dash_fw_ready(tp);
}

static void rtl8127_driver_stop(struct rtl8127_private *tp)
{
        if (!tp->AllowAccessDashOcp)
                return;

        rtl8127_notify_dash_oob(tp, OOB_CMD_DRIVER_STOP);

        rtl8127_wait_dash_fw_ready(tp);
}

static void _rtl8127_ephy_write(struct rtl8127_private *tp, int addr, int data)
{
        int i;

        RTL_W32(tp, EPHYAR,
                EPHYAR_Write |
                (addr & EPHYAR_Reg_Mask_v2) << EPHYAR_Reg_shift |
                (data & EPHYAR_Data_Mask));

        for (i = 0; i < R8127_CHANNEL_WAIT_COUNT; i++) {
                udelay(R8127_CHANNEL_WAIT_TIME);

                /* Check if the RTL8125 has completed EPHY write */
                if (!(RTL_R32(tp, EPHYAR) & EPHYAR_Flag))
                        break;
        }

        udelay(R8127_CHANNEL_EXIT_DELAY_TIME);
}

static void rtl8127_set_ephy_ext_addr(struct rtl8127_private *tp, int addr)
{
        _rtl8127_ephy_write(tp, EPHYAR_EXT_ADDR, addr);
}

static int rtl8127_check_ephy_ext_addr(struct rtl8127_private *tp, int addr)
{
        int data;

        data = ((u16)addr >> 12);

        rtl8127_set_ephy_ext_addr(tp, data);

        return (addr & 0xfff);
}

void rtl8127_ephy_write(struct rtl8127_private *tp, int addr, int data)
{
        _rtl8127_ephy_write(tp, rtl8127_check_ephy_ext_addr(tp, addr), data);
}

static u16 _rtl8127_ephy_read(struct rtl8127_private *tp, int addr)
{
        int i;
        u16 data = 0xffff;

        RTL_W32(tp, EPHYAR,
                EPHYAR_Read | (addr & EPHYAR_Reg_Mask_v2) << EPHYAR_Reg_shift);

        for (i = 0; i < R8127_CHANNEL_WAIT_COUNT; i++) {
                udelay(R8127_CHANNEL_WAIT_TIME);

                /* Check if the RTL8125 has completed EPHY read */
                if (RTL_R32(tp, EPHYAR) & EPHYAR_Flag) {
                        data = (u16) (RTL_R32(tp, EPHYAR) & EPHYAR_Data_Mask);
                        break;
                }
        }

        udelay(R8127_CHANNEL_EXIT_DELAY_TIME);

        return data;
}

u16 rtl8127_ephy_read(struct rtl8127_private *tp, int addr)
{
        return  _rtl8127_ephy_read(tp, rtl8127_check_ephy_ext_addr(tp, addr));
}

static u32
rtl8127_csi_other_fun_read(struct rtl8127_private *tp,
                           u8 multi_fun_sel_bit,
                           u32 addr)
{
        u32 cmd;
        int i;
        u32 value = 0xffffffff;

        cmd = CSIAR_Read | CSIAR_ByteEn << CSIAR_ByteEn_shift | (addr & CSIAR_Addr_Mask);

        if (tp->mcfg == CFG_METHOD_DEFAULT)
                multi_fun_sel_bit = 0;

        if (multi_fun_sel_bit > 7)
                goto exit;

        cmd |= multi_fun_sel_bit << 16;

        RTL_W32(tp, CSIAR, cmd);

        for (i = 0; i < R8127_CHANNEL_WAIT_COUNT; i++) {
                udelay(R8127_CHANNEL_WAIT_TIME);

                /* Check if the RTL8125 has completed CSI read */
                if (RTL_R32(tp, CSIAR) & CSIAR_Flag) {
                        value = (u32)RTL_R32(tp, CSIDR);
                        break;
                }
        }

        udelay(R8127_CHANNEL_EXIT_DELAY_TIME);

exit:
        return value;
}

static void
rtl8127_csi_other_fun_write(struct rtl8127_private *tp,
                            u8 multi_fun_sel_bit,
                            u32 addr,
                            u32 value)
{
        u32 cmd;
        int i;

        RTL_W32(tp, CSIDR, value);
        cmd = CSIAR_Write | CSIAR_ByteEn << CSIAR_ByteEn_shift | (addr & CSIAR_Addr_Mask);
        if (tp->mcfg == CFG_METHOD_DEFAULT)
                multi_fun_sel_bit = 0;

        if (multi_fun_sel_bit > 7)
                return;

        cmd |= multi_fun_sel_bit << 16;

        RTL_W32(tp, CSIAR, cmd);

        for (i = 0; i < R8127_CHANNEL_WAIT_COUNT; i++) {
                udelay(R8127_CHANNEL_WAIT_TIME);

                /* Check if the RTL8125 has completed CSI write */
                if (!(RTL_R32(tp, CSIAR) & CSIAR_Flag))
                        break;
        }

        udelay(R8127_CHANNEL_EXIT_DELAY_TIME);
}

static u32
rtl8127_csi_read(struct rtl8127_private *tp,
                 u32 addr)
{
        u8 multi_fun_sel_bit;

        multi_fun_sel_bit = 0;

        return rtl8127_csi_other_fun_read(tp, multi_fun_sel_bit, addr);
}

static void
rtl8127_csi_write(struct rtl8127_private *tp,
                  u32 addr,
                  u32 value)
{
        u8 multi_fun_sel_bit;

        multi_fun_sel_bit = 0;

        rtl8127_csi_other_fun_write(tp, multi_fun_sel_bit, addr, value);
}

static u8
rtl8127_csi_fun0_read_byte(struct rtl8127_private *tp,
                           u32 addr)
{
        u8 RetVal = 0;

        if (tp->mcfg == CFG_METHOD_DEFAULT) {
                struct pci_dev *pdev = tp->pci_dev;

                pci_read_config_byte(pdev, addr, &RetVal);
        } else {
                u32 TmpUlong;
                u16 RegAlignAddr;
                u8 ShiftByte;

                RegAlignAddr = addr & ~(0x3);
                ShiftByte = addr & (0x3);
                TmpUlong = rtl8127_csi_other_fun_read(tp, 0, RegAlignAddr);
                TmpUlong >>= (8*ShiftByte);
                RetVal = (u8)TmpUlong;
        }

        udelay(R8127_CHANNEL_EXIT_DELAY_TIME);

        return RetVal;
}

static void
rtl8127_csi_fun0_write_byte(struct rtl8127_private *tp,
                            u32 addr,
                            u8 value)
{
        if (tp->mcfg == CFG_METHOD_DEFAULT) {
                struct pci_dev *pdev = tp->pci_dev;

                pci_write_config_byte(pdev, addr, value);
        } else {
                u32 TmpUlong;
                u16 RegAlignAddr;
                u8 ShiftByte;

                RegAlignAddr = addr & ~(0x3);
                ShiftByte = addr & (0x3);
                TmpUlong = rtl8127_csi_other_fun_read(tp, 0, RegAlignAddr);
                TmpUlong &= ~(0xFF << (8*ShiftByte));
                TmpUlong |= (value << (8*ShiftByte));
                rtl8127_csi_other_fun_write(tp, 0, RegAlignAddr, TmpUlong);
        }

        udelay(R8127_CHANNEL_EXIT_DELAY_TIME);
}

u32 rtl8127_eri_read_with_oob_base_address(struct rtl8127_private *tp, int addr, int len, int type, const u32 base_address)
{
        int i, val_shift, shift = 0;
        u32 value1 = 0, value2 = 0, mask;
        u32 eri_cmd;
        const u32 transformed_base_address = ((base_address & 0x00FFF000) << 6) | (base_address & 0x000FFF);

        if (len > 4 || len <= 0)
                return -1;

        while (len > 0) {
                val_shift = addr % ERIAR_Addr_Align;
                addr = addr & ~0x3;

                eri_cmd = ERIAR_Read |
                          transformed_base_address |
                          type << ERIAR_Type_shift |
                          ERIAR_ByteEn << ERIAR_ByteEn_shift |
                          (addr & 0x0FFF);
                if (addr & 0xF000) {
                        u32 tmp;

                        tmp = addr & 0xF000;
                        tmp >>= 12;
                        eri_cmd |= (tmp << 20) & 0x00F00000;
                }

                RTL_W32(tp, ERIAR, eri_cmd);

                for (i = 0; i < R8127_CHANNEL_WAIT_COUNT; i++) {
                        udelay(R8127_CHANNEL_WAIT_TIME);

                        /* Check if the RTL8125 has completed ERI read */
                        if (RTL_R32(tp, ERIAR) & ERIAR_Flag)
                                break;
                }

                if (len == 1)       mask = (0xFF << (val_shift * 8)) & 0xFFFFFFFF;
                else if (len == 2)  mask = (0xFFFF << (val_shift * 8)) & 0xFFFFFFFF;
                else if (len == 3)  mask = (0xFFFFFF << (val_shift * 8)) & 0xFFFFFFFF;
                else            mask = (0xFFFFFFFF << (val_shift * 8)) & 0xFFFFFFFF;

                value1 = RTL_R32(tp, ERIDR) & mask;
                value2 |= (value1 >> val_shift * 8) << shift * 8;

                if (len <= 4 - val_shift) {
                        len = 0;
                } else {
                        len -= (4 - val_shift);
                        shift = 4 - val_shift;
                        addr += 4;
                }
        }

        udelay(R8127_CHANNEL_EXIT_DELAY_TIME);

        return value2;
}

u32 rtl8127_eri_read(struct rtl8127_private *tp, int addr, int len, int type)
{
        return rtl8127_eri_read_with_oob_base_address(tp, addr, len, type, 0);
}

int rtl8127_eri_write_with_oob_base_address(struct rtl8127_private *tp, int addr, int len, u32 value, int type, const u32 base_address)
{
        int i, val_shift, shift = 0;
        u32 value1 = 0, mask;
        u32 eri_cmd;
        const u32 transformed_base_address = ((base_address & 0x00FFF000) << 6) | (base_address & 0x000FFF);

        if (len > 4 || len <= 0)
                return -1;

        while (len > 0) {
                val_shift = addr % ERIAR_Addr_Align;
                addr = addr & ~0x3;

                if (len == 1)       mask = (0xFF << (val_shift * 8)) & 0xFFFFFFFF;
                else if (len == 2)  mask = (0xFFFF << (val_shift * 8)) & 0xFFFFFFFF;
                else if (len == 3)  mask = (0xFFFFFF << (val_shift * 8)) & 0xFFFFFFFF;
                else            mask = (0xFFFFFFFF << (val_shift * 8)) & 0xFFFFFFFF;

                value1 = rtl8127_eri_read_with_oob_base_address(tp, addr, 4, type, base_address) & ~mask;
                value1 |= ((value << val_shift * 8) >> shift * 8);

                RTL_W32(tp, ERIDR, value1);

                eri_cmd = ERIAR_Write |
                          transformed_base_address |
                          type << ERIAR_Type_shift |
                          ERIAR_ByteEn << ERIAR_ByteEn_shift |
                          (addr & 0x0FFF);
                if (addr & 0xF000) {
                        u32 tmp;

                        tmp = addr & 0xF000;
                        tmp >>= 12;
                        eri_cmd |= (tmp << 20) & 0x00F00000;
                }

                RTL_W32(tp, ERIAR, eri_cmd);

                for (i = 0; i < R8127_CHANNEL_WAIT_COUNT; i++) {
                        udelay(R8127_CHANNEL_WAIT_TIME);

                        /* Check if the RTL8125 has completed ERI write */
                        if (!(RTL_R32(tp, ERIAR) & ERIAR_Flag))
                                break;
                }

                if (len <= 4 - val_shift) {
                        len = 0;
                } else {
                        len -= (4 - val_shift);
                        shift = 4 - val_shift;
                        addr += 4;
                }
        }

        udelay(R8127_CHANNEL_EXIT_DELAY_TIME);

        return 0;
}

int rtl8127_eri_write(struct rtl8127_private *tp, int addr, int len, u32 value, int type)
{
        return rtl8127_eri_write_with_oob_base_address(tp, addr, len, value, type, NO_BASE_ADDRESS);
}

static void
rtl8127_enable_rxdvgate(struct net_device *dev)
{
        struct rtl8127_private *tp = netdev_priv(dev);

        RTL_W8(tp, 0xF2, RTL_R8(tp, 0xF2) | BIT_3);
}

static void
rtl8127_disable_rxdvgate(struct net_device *dev)
{
        struct rtl8127_private *tp = netdev_priv(dev);

        RTL_W8(tp, 0xF2, RTL_R8(tp, 0xF2) & ~BIT_3);
}

static u8
rtl8127_is_gpio_low(struct net_device *dev)
{
        struct rtl8127_private *tp = netdev_priv(dev);
        u8 gpio_low = FALSE;

        switch (tp->HwSuppCheckPhyDisableModeVer) {
        case 3:
                if (!(rtl8127_mac_ocp_read(tp, 0xDC04) & BIT_13))
                        gpio_low = TRUE;
                break;
        }

        if (gpio_low)
                dprintk("gpio is low.\n");

        return gpio_low;
}

static u8
rtl8127_is_phy_disable_mode_enabled(struct net_device *dev)
{
        struct rtl8127_private *tp = netdev_priv(dev);
        u8 phy_disable_mode_enabled = FALSE;

        switch (tp->HwSuppCheckPhyDisableModeVer) {
        case 3:
                if (RTL_R8(tp, 0xF2) & BIT_5)
                        phy_disable_mode_enabled = TRUE;
                break;
        }

        if (phy_disable_mode_enabled)
                dprintk("phy disable mode enabled.\n");

        return phy_disable_mode_enabled;
}

static u8
rtl8127_is_in_phy_disable_mode(struct net_device *dev)
{
        u8 in_phy_disable_mode = FALSE;

        if (rtl8127_is_phy_disable_mode_enabled(dev) && rtl8127_is_gpio_low(dev))
                in_phy_disable_mode = TRUE;

        if (in_phy_disable_mode)
                dprintk("Hardware is in phy disable mode.\n");

        return in_phy_disable_mode;
}

static void
rtl8127_stop_all_request(struct net_device *dev)
{
        struct rtl8127_private *tp = netdev_priv(dev);

        RTL_W8(tp, ChipCmd, RTL_R8(tp, ChipCmd) | StopReq);
        udelay(200);
}

static void
rtl8127_clear_stop_all_request(struct net_device *dev)
{
        struct rtl8127_private *tp = netdev_priv(dev);

        RTL_W8(tp, ChipCmd, RTL_R8(tp, ChipCmd) & (CmdTxEnb | CmdRxEnb));
}

void
rtl8127_wait_txrx_fifo_empty(struct net_device *dev)
{
        struct rtl8127_private *tp = netdev_priv(dev);
        int i;

        /* Txfifo_empty require StopReq been set */
        for (i = 0; i < 3000; i++) {
                udelay(50);
                if ((RTL_R8(tp, MCUCmd_reg) & (Txfifo_empty | Rxfifo_empty)) == (Txfifo_empty | Rxfifo_empty))
                        break;
        }

        for (i = 0; i < 3000; i++) {
                udelay(50);
                if ((RTL_R16(tp, IntrMitigate) & (BIT_0 | BIT_1 | BIT_8)) == (BIT_0 | BIT_1 | BIT_8))
                        break;
        }
}

#ifdef ENABLE_DASH_SUPPORT

static inline void
rtl8127_enable_dash2_interrupt(struct rtl8127_private *tp)
{
        if (!HW_DASH_SUPPORT_IPC2(tp))
                return;

        if (!tp->DASH)
                return;

        rtl8127_set_ipc2_soc_imr_bit(tp, RISC_IPC2_INTR);
}

static inline void
rtl8127_disable_dash2_interrupt(struct rtl8127_private *tp)
{
        if (!HW_DASH_SUPPORT_IPC2(tp))
                return;

        rtl8127_clear_ipc2_soc_imr_bit(tp, RISC_IPC2_INTR);
}
#endif

void
rtl8127_enable_hw_linkchg_interrupt(struct rtl8127_private *tp)
{
        switch (tp->HwCurrIsrVer) {
        case 6:
                RTL_W32(tp, IMR_V2_SET_REG_8125, ISRIMR_V6_LINKCHG);
                break;
        case 5:
                RTL_W32(tp, IMR_V2_SET_REG_8125, ISRIMR_V5_LINKCHG);
                break;
        case 4:
                RTL_W32(tp, IMR_V2_SET_REG_8125, ISRIMR_V4_LINKCHG);
                break;
        case 2:
        case 3:
                RTL_W32(tp, IMR_V2_SET_REG_8125, ISRIMR_V2_LINKCHG);
                break;
        case 1:
                RTL_W32(tp, tp->imr_reg[0], LinkChg | RTL_R32(tp, tp->imr_reg[0]));
                break;
        }

#ifdef ENABLE_DASH_SUPPORT
        if (tp->DASH)
                rtl8127_enable_dash2_interrupt(tp);
#endif
}

static inline void
rtl8127_enable_hw_interrupt(struct rtl8127_private *tp)
{
        switch (tp->HwCurrIsrVer) {
        case 2:
        case 3:
        case 4:
        case 5:
        case 6:
                RTL_W32(tp, IMR_V2_SET_REG_8125, tp->intr_mask);
                break;
        case 1:
                RTL_W32(tp, tp->imr_reg[0], tp->intr_mask);

                if (R8127_MULTI_RX_Q(tp)) {
                        int i;
                        for (i=1; i<tp->num_rx_rings; i++)
                                RTL_W16(tp, tp->imr_reg[i], other_q_intr_mask);
                }
                break;
        }

#ifdef ENABLE_DASH_SUPPORT
        if (tp->DASH)
                rtl8127_enable_dash2_interrupt(tp);
#endif
}

static inline void rtl8127_clear_hw_isr_v2(struct rtl8127_private *tp,
                u32 message_id)
{
        RTL_W32(tp, ISR_V2_8125, BIT(message_id));
}

static inline void
rtl8127_disable_hw_interrupt(struct rtl8127_private *tp)
{
        if (tp->HwCurrIsrVer > 1) {
                RTL_W32(tp, IMR_V2_CLEAR_REG_8125, 0xFFFFFFFF);
                if (tp->HwCurrIsrVer > 3)
                        RTL_W32(tp, IMR_V4_L2_CLEAR_REG_8125, 0xFFFFFFFF);
        } else {
                RTL_W32(tp, tp->imr_reg[0], 0x0000);

                if (R8127_MULTI_RX_Q(tp)) {
                        int i;
                        for (i=1; i<tp->num_rx_rings; i++)
                                RTL_W16(tp, tp->imr_reg[i], 0);
                }

#ifdef ENABLE_DASH_SUPPORT
                rtl8127_disable_dash2_interrupt(tp);
#endif
        }
}

static inline void
rtl8127_switch_to_hw_interrupt(struct rtl8127_private *tp)
{
        RTL_W32(tp, TIMER_INT0_8125, 0x0000);

        rtl8127_enable_hw_interrupt(tp);
}

static inline void
rtl8127_switch_to_timer_interrupt(struct rtl8127_private *tp)
{
        if (tp->use_timer_interrupt) {
                RTL_W32(tp, TIMER_INT0_8125, timer_count);
                RTL_W32(tp, TCTR0_8125, timer_count);
                RTL_W32(tp, tp->imr_reg[0], tp->timer_intr_mask);
        } else {
                rtl8127_switch_to_hw_interrupt(tp);
        }
}

static void
rtl8127_irq_mask_and_ack(struct rtl8127_private *tp)
{
        rtl8127_disable_hw_interrupt(tp);

        if (tp->HwCurrIsrVer > 1) {
                RTL_W32(tp, ISR_V2_8125, 0xFFFFFFFF);
                if (tp->HwCurrIsrVer > 3)
                        RTL_W32(tp, ISR_V4_L2_8125, 0xFFFFFFFF);
        } else {
                if (R8127_MULTI_RX_Q(tp)) {
                        int i;
                        for (i=1; i<tp->num_rx_rings; i++)
                                RTL_W16(tp, tp->isr_reg[i], RTL_R16(tp, tp->isr_reg[i]));
                }
        }

#ifdef ENABLE_DASH_SUPPORT
        rtl8127_clear_ipc2_isr(tp);
#endif
}

static void
rtl8127_disable_rx_packet_filter(struct rtl8127_private *tp)
{
        RTL_W32(tp, RxConfig, RTL_R32(tp, RxConfig) &
                ~(AcceptErr | AcceptRunt |AcceptBroadcast | AcceptMulticast |
                  AcceptMyPhys |  AcceptAllPhys));
}

static void
rtl8127_nic_reset(struct net_device *dev)
{
        struct rtl8127_private *tp = netdev_priv(dev);
        int i;

        rtl8127_disable_rx_packet_filter(tp);

        rtl8127_enable_rxdvgate(dev);

        rtl8127_stop_all_request(dev);

        rtl8127_wait_txrx_fifo_empty(dev);

        rtl8127_clear_stop_all_request(dev);

        /* Soft reset the chip. */
        RTL_W8(tp, ChipCmd, CmdReset);

        /* Check that the chip has finished the reset. */
        for (i = 100; i > 0; i--) {
                udelay(100);
                if ((RTL_R8(tp, ChipCmd) & CmdReset) == 0)
                        break;
        }

        /* reset rcr */
        RTL_W32(tp, RxConfig, (RX_DMA_BURST_512 << RxCfgDMAShift));
}

static void
rtl8127_hw_set_interrupt_type(struct rtl8127_private *tp, u8 isr_ver)
{
        u8 tmp;

        if (tp->HwSuppIsrVer < 2)
                return;

        tmp = RTL_R8(tp, INT_CFG0_8125);

        switch (tp->HwSuppIsrVer) {
        case 6:
                tmp &= ~INT_CFG0_AVOID_MISS_INTR;
                fallthrough;
        case 4:
        case 5:
                if (tp->HwSuppIsrVer == 6)
                        tmp &= ~INT_CFG0_AUTO_CLEAR_IMR;
                else
                        tmp &= ~INT_CFG0_MSIX_ENTRY_NUM_MODE;
                fallthrough;
        case 2:
        case 3:
                tmp &= ~(INT_CFG0_ENABLE_8125);
                if (isr_ver > 1)
                        tmp |= INT_CFG0_ENABLE_8125;
                break;
        default:
                return;
        }

        RTL_W8(tp, INT_CFG0_8125, tmp);
}

static void
rtl8127_hw_clear_timer_int(struct net_device *dev)
{
        struct rtl8127_private *tp = netdev_priv(dev);

        RTL_W32(tp, TIMER_INT0_8125, 0x0000);
        RTL_W32(tp, TIMER_INT1_8125, 0x0000);
        RTL_W32(tp, TIMER_INT2_8125, 0x0000);
        RTL_W32(tp, TIMER_INT3_8125, 0x0000);
}

static void
rtl8127_hw_clear_int_miti(struct net_device *dev)
{
        struct rtl8127_private *tp = netdev_priv(dev);
        int i;

        switch (tp->HwSuppIntMitiVer) {
        case 3:
        case 6:
                //IntMITI_0-IntMITI_31
                for (i=0xA00; i<0xB00; i+=4)
                        RTL_W32(tp, i, 0x0000);
                break;
        case 4:
        case 5:
                //IntMITI_0-IntMITI_15
                for (i = 0xA00; i < 0xA80; i += 4)
                        RTL_W32(tp, i, 0x0000);

                if (tp->HwSuppIntMitiVer == 5)
                        RTL_W8(tp, INT_CFG0_8125, RTL_R8(tp, INT_CFG0_8125) &
                               ~(INT_CFG0_TIMEOUT0_BYPASS_8125 |
                                 INT_CFG0_MITIGATION_BYPASS_8125 |
                                 INT_CFG0_RDU_BYPASS_8126));
                else
                        RTL_W8(tp, INT_CFG0_8125, RTL_R8(tp, INT_CFG0_8125) &
                               ~(INT_CFG0_TIMEOUT0_BYPASS_8125 | INT_CFG0_MITIGATION_BYPASS_8125));

                RTL_W16(tp, INT_CFG1_8125, 0x0000);
                break;
        }
}

static bool
rtl8127_vec_2_tx_q_num(
        struct rtl8127_private *tp,
        u32 messageId,
        u32 *qnum
)
{
        u32 whichQ = 0xffffffff;
        bool rc = false;

        switch (tp->HwSuppIsrVer) {
        case 2:
                if (messageId == 0x10)
                        whichQ = 0;
                else if (messageId == 0x12 && tp->num_tx_rings > 1)
                        whichQ = 1;
                break;
        case 3:
        case 4:
                if (messageId == 0x00)
                        whichQ = 0;
                else if (messageId == 0x01 && tp->num_tx_rings > 1)
                        whichQ = 1;
                break;
        case 5:
                if (messageId == 0x10)
                        whichQ = 0;
                else if (messageId == 0x11 && tp->num_tx_rings > 1)
                        whichQ = 1;
                break;
        case 6:
                if (messageId == 0x08)
                        whichQ = 0;
                else if (messageId == 0x09 && tp->num_tx_rings > 1)
                        whichQ = 1;
                break;
        case 7:
                if (messageId == 0x1B)
                        whichQ = 0;
                else if (messageId == 0x1C && tp->num_tx_rings > 1)
                        whichQ = 1;
                break;
        }

        if (whichQ != 0xffffffff) {
                *qnum = whichQ;
                rc = true;
        }

        return rc;
}

static bool
rtl8127_vec_2_rx_q_num(
        struct rtl8127_private *tp,
        u32 messageId,
        u32 *qnum
)
{
        u32 whichQ = 0xffffffff;
        bool rc = false;

        switch (tp->HwSuppIsrVer) {
        case 2:
        case 3:
        case 4:
        case 5:
        case 6:
        case 7:
                if (messageId < tp->HwSuppNumRxQueues)
                        whichQ = messageId;
                break;
        }

        if (whichQ != 0xffffffff) {
                *qnum = whichQ;
                rc = true;
        }

        return rc;
}

void
rtl8127_hw_set_timer_int(struct rtl8127_private *tp,
                         u32 message_id,
                         u8 timer_intmiti_val)
{
        u32 qnum;

        switch (tp->HwSuppIntMitiVer) {
        case 4:
        case 5:
        case 6:
                //ROK
                if (rtl8127_vec_2_rx_q_num(tp, message_id, &qnum))
                        RTL_W8(tp,INT_MITI_V2_0_RX + 8 * qnum, timer_intmiti_val);
                //TOK
                if (rtl8127_vec_2_tx_q_num(tp, message_id, &qnum))
                        RTL_W8(tp,INT_MITI_V2_0_TX + 8 * qnum, timer_intmiti_val);
                break;
        }
}

void
rtl8127_hw_reset(struct net_device *dev)
{
        struct rtl8127_private *tp = netdev_priv(dev);

        rtl8127_lib_reset_prepare(tp);

        /* Disable interrupts */
        rtl8127_irq_mask_and_ack(tp);

        rtl8127_hw_clear_timer_int(dev);

        rtl8127_nic_reset(dev);
}

static unsigned int
rtl8127_xmii_reset_pending(struct net_device *dev)
{
        struct rtl8127_private *tp = netdev_priv(dev);
        unsigned int retval;
        unsigned long flags;

        r8127_spin_lock(&tp->phy_lock, flags);
        rtl8127_mdio_write(tp, 0x1f, 0x0000);
        retval = rtl8127_mdio_read(tp, MII_BMCR) & BMCR_RESET;
        r8127_spin_unlock(&tp->phy_lock, flags);

        return retval;
}

static unsigned int
rtl8127_xmii_link_ok(struct net_device *dev)
{
        struct rtl8127_private *tp = netdev_priv(dev);
        u32 status;

        status = rtl8127_get_phy_status(tp);
        if (status == UINT_MAX)
                return 0;

        return (status & LinkStatus) ? 1 : 0;
}

static int
rtl8127_wait_phy_reset_complete(struct rtl8127_private *tp)
{
        int i, val;

        for (i = 0; i < 2500; i++) {
                val = rtl8127_mdio_read(tp, MII_BMCR) & BMCR_RESET;
                if (!val)
                        return 0;

                mdelay(1);
        }

        return -1;
}

static void
rtl8127_xmii_reset_enable(struct net_device *dev)
{
        struct rtl8127_private *tp = netdev_priv(dev);
        unsigned long flags;
        int ret;

        if (rtl8127_is_in_phy_disable_mode(dev))
                return;

        r8127_spin_lock(&tp->phy_lock, flags);

        rtl8127_mdio_write(tp, 0x1f, 0x0000);
        rtl8127_mdio_write(tp, MII_ADVERTISE, rtl8127_mdio_read(tp, MII_ADVERTISE) &
                           ~(ADVERTISE_10HALF | ADVERTISE_10FULL |
                             ADVERTISE_100HALF | ADVERTISE_100FULL));
        rtl8127_mdio_write(tp, MII_CTRL1000, rtl8127_mdio_read(tp, MII_CTRL1000) &
                           ~(ADVERTISE_1000HALF | ADVERTISE_1000FULL));
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA5D4, rtl8127_mdio_direct_read_phy_ocp(tp, 0xA5D4) &
                                          ~(RTK_ADVERTISE_2500FULL | RTK_ADVERTISE_5000FULL |
                                            RTK_ADVERTISE_10000FULL));
        rtl8127_mdio_write(tp, MII_BMCR, BMCR_RESET | BMCR_ANENABLE);

        ret = rtl8127_wait_phy_reset_complete(tp);

        r8127_spin_unlock(&tp->phy_lock, flags);

        if (ret != 0 && netif_msg_link(tp))
                printk(KERN_ERR "%s: PHY reset failed.\n", dev->name);
}

void
rtl8127_init_ring_indexes(struct rtl8127_private *tp)
{
        int i;

        for (i = 0; i < tp->HwSuppNumTxQueues; i++) {
                struct rtl8127_tx_ring *ring = &tp->tx_ring[i];
                ring->dirty_tx = ring->cur_tx = 0;
                ring->NextHwDesCloPtr = 0;
                ring->BeginHwDesCloPtr = 0;
                ring->index = i;
                ring->priv = tp;
                ring->netdev = tp->dev;

                /* reset BQL for queue */
                netdev_tx_reset_queue(txring_txq(ring));
        }

        for (i = 0; i < tp->HwSuppNumRxQueues; i++) {
                struct rtl8127_rx_ring *ring = &tp->rx_ring[i];
                ring->dirty_rx = ring->cur_rx = 0;
                ring->index = i;
                ring->priv = tp;
                ring->netdev = tp->dev;
        }

#ifdef ENABLE_LIB_SUPPORT
        for (i = 0; i < tp->HwSuppNumTxQueues; i++) {
                struct rtl8127_ring *ring = &tp->lib_tx_ring[i];
                ring->direction = RTL8127_CH_DIR_TX;
                ring->queue_num = i;
                ring->private = tp;
        }

        for (i = 0; i < tp->HwSuppNumRxQueues; i++) {
                struct rtl8127_ring *ring = &tp->lib_rx_ring[i];
                ring->direction = RTL8127_CH_DIR_RX;
                ring->queue_num = i;
                ring->private = tp;
        }
#endif
}

static void
rtl8127_issue_offset_99_event(struct rtl8127_private *tp)
{
        rtl8127_mac_ocp_write(tp, 0xE09A,  rtl8127_mac_ocp_read(tp, 0xE09A) | BIT_0);
}

#ifdef ENABLE_DASH_SUPPORT
static void
rtl8127_check_and_enable_dash_interrupt(struct rtl8127_private *tp)
{
        if (!HW_DASH_SUPPORT_IPC2(tp))
                return;

        if (!tp->DASH)
                return;

        //
        // even disconnected, enable dash interrupt mask bits for in-band/out-band communication
        //
        rtl8127_enable_dash2_interrupt(tp);
        if (tp->HwCurrIsrVer > 1)
                RTL_W32(tp, IMR_V4_L2_SET_REG_8125, ISRIMR_V6_L2_MISC_INTR);
        else
                RTL_W16(tp, tp->imr_reg[0], ISRIMR_DASH_INTR_EN);
}
#endif

static int rtl8127_enable_eee_plus(struct rtl8127_private *tp)
{
        rtl8127_mac_ocp_write(tp, 0xE080, rtl8127_mac_ocp_read(tp, 0xE080)|BIT_1);

        return 0;
}

static int rtl8127_disable_eee_plus(struct rtl8127_private *tp)
{
        rtl8127_mac_ocp_write(tp, 0xE080, rtl8127_mac_ocp_read(tp, 0xE080)&~BIT_1);

        return 0;
}

static void rtl8127_enable_double_vlan(struct rtl8127_private *tp)
{
        RTL_W16(tp, DOUBLE_VLAN_CONFIG, 0xf002);
}

static void rtl8127_disable_double_vlan(struct rtl8127_private *tp)
{
        RTL_W16(tp, DOUBLE_VLAN_CONFIG, 0);
}

static void rtl8127_set_radm_fifo_prot(struct rtl8127_private *tp, bool enable)
{
        switch (tp->mcfg) {
        case CFG_METHOD_2:
                RTL_W16(tp, RADMFIFO_PROTECT, enable ? 0x2001 : 0);
                break;
        default:
                break;
        }
}

static void
rtl8127_link_on_patch(struct net_device *dev)
{
        struct rtl8127_private *tp = netdev_priv(dev);
        unsigned long flags;
        u32 status;

        rtl8127_hw_config(dev);

        status = rtl8127_get_phy_status(tp);
        if (status & _10bps)
                rtl8127_enable_eee_plus(tp);

        if (status & (_1000bpsL | _2500bpsL | _10bps | _100bps | _1000bpsF))
                rtl8127_set_radm_fifo_prot(tp, 1);
        else
                rtl8127_set_radm_fifo_prot(tp, 0);

        rtl8127_hw_start(dev);

        netif_carrier_on(dev);

        netif_tx_wake_all_queues(dev);

        r8127_spin_lock(&tp->phy_lock, flags);
        tp->phy_reg_aner = rtl8127_mdio_read(tp, MII_EXPANSION);
        tp->phy_reg_anlpar = rtl8127_mdio_read(tp, MII_LPA);
        tp->phy_reg_gbsr = rtl8127_mdio_read(tp, MII_STAT1000);
        tp->phy_reg_status_2500 = rtl8127_mdio_direct_read_phy_ocp(tp, 0xA5D6);
        r8127_spin_unlock(&tp->phy_lock, flags);

#ifdef ENABLE_PTP_SUPPORT
        if (tp->EnablePtp)
                rtl8127_set_local_time(tp);
#endif // ENABLE_PTP_SUPPORT
}

static void
rtl8127_link_down_patch(struct net_device *dev)
{
        struct rtl8127_private *tp = netdev_priv(dev);
        unsigned long flags;

        r8127_spin_lock(&tp->phy_lock, flags);

        tp->phy_reg_aner = 0;
        tp->phy_reg_anlpar = 0;
        tp->phy_reg_gbsr = 0;
        tp->phy_reg_status_2500 = 0;

        r8127_spin_unlock(&tp->phy_lock, flags);

        rtl8127_disable_eee_plus(tp);

        netif_carrier_off(dev);

        netif_tx_disable(dev);

        rtl8127_hw_reset(dev);

        rtl8127_tx_clear(tp);

        rtl8127_rx_clear(tp);

        rtl8127_init_ring(dev);

        rtl8127_enable_hw_linkchg_interrupt(tp);

        //rtl8127_set_speed(dev, tp->autoneg, tp->speed, tp->duplex, tp->advertising);

#ifdef ENABLE_DASH_SUPPORT
        rtl8127_check_and_enable_dash_interrupt(tp);
#endif
}

static void
_rtl8127_check_link_status(struct net_device *dev, unsigned int link_state)
{
        struct rtl8127_private *tp = netdev_priv(dev);

        if (link_state != R8127_LINK_STATE_OFF &&
            link_state != R8127_LINK_STATE_ON)
                link_state = tp->link_ok(dev);

        if (link_state == R8127_LINK_STATE_ON) {
                rtl8127_link_on_patch(dev);

                if (netif_msg_ifup(tp))
                        printk(KERN_INFO PFX "%s: link up\n", dev->name);
        } else {
                if (netif_msg_ifdown(tp))
                        printk(KERN_INFO PFX "%s: link down\n", dev->name);

                rtl8127_link_down_patch(dev);
        }
}

static void
rtl8127_check_link_status(struct net_device *dev)
{
        struct rtl8127_private *tp = netdev_priv(dev);
        unsigned int link_status_on;

        tp->resume_not_chg_speed = 0;

        link_status_on = tp->link_ok(dev);
        if (netif_carrier_ok(dev) == link_status_on)
                return;

        _rtl8127_check_link_status(dev, link_status_on);
}

static bool
rtl8127_is_autoneg_mode_valid(u32 autoneg)
{
        switch(autoneg) {
        case AUTONEG_ENABLE:
        case AUTONEG_DISABLE:
                return true;
        default:
                return false;
        }
}

static bool
rtl8127_is_speed_mode_valid(struct rtl8127_private *tp, u32 speed)
{
        if (HW_FIBER_MODE_ENABLED(tp)) {
                switch(speed) {
                case SPEED_10000:
                case SPEED_1000:
                        return true;
                default:
                        return false;
                }
        } else {
                switch(speed) {
                case SPEED_10000:
                case SPEED_5000:
                case SPEED_2500:
                case SPEED_1000:
                case SPEED_100:
                case SPEED_10:
                        return true;
                default:
                        return false;
                }
        }
}

static bool
rtl8127_is_duplex_mode_valid(u8 duplex)
{
        switch(duplex) {
        case DUPLEX_FULL:
        case DUPLEX_HALF:
                return true;
        default:
                return false;
        }
}

static void
rtl8127_set_link_option(struct rtl8127_private *tp,
                        u8 autoneg,
                        u32 speed,
                        u8 duplex,
                        enum rtl8127_fc_mode fc)
{
        u64 adv;

        if (!rtl8127_is_speed_mode_valid(tp, speed))
                speed = SPEED_10000;

        if (!rtl8127_is_duplex_mode_valid(duplex))
                duplex = DUPLEX_FULL;

        if (!rtl8127_is_autoneg_mode_valid(autoneg))
                autoneg = AUTONEG_ENABLE;

        speed = min(speed, tp->HwSuppMaxPhyLinkSpeed);

        adv = 0;
        if (HW_FIBER_MODE_ENABLED(tp)) {
                switch(speed) {
                case SPEED_10000:
                        adv |= ADVERTISED_10000baseT_Full;
                        fallthrough;
                case SPEED_1000:
                        adv |= ADVERTISED_1000baseT_Full;
                        break;
                default:
                        break;
                }
        } else {
                switch(speed) {
                case SPEED_10000:
                        adv |= ADVERTISED_10000baseT_Full;
                        fallthrough;
                case SPEED_5000:
                        adv |= RTK_ADVERTISED_5000baseX_Full;
                        fallthrough;
                case SPEED_2500:
                        adv |= ADVERTISED_2500baseX_Full;
                        fallthrough;
                default:
                        adv |= (ADVERTISED_10baseT_Half | ADVERTISED_10baseT_Full |
                                ADVERTISED_100baseT_Half | ADVERTISED_100baseT_Full |
                                ADVERTISED_1000baseT_Half | ADVERTISED_1000baseT_Full);
                        break;
                }
        }

        tp->autoneg = autoneg;
        tp->speed = speed;
        tp->duplex = duplex;
        tp->advertising = adv;
        tp->fcpause = fc;
}

static void
rtl8127_wait_ll_share_fifo_ready(struct net_device *dev)
{
        struct rtl8127_private *tp = netdev_priv(dev);
        int i;

        for (i = 0; i < 10; i++) {
                udelay(100);
                if (RTL_R16(tp, 0xD2) & BIT_9)
                        break;
        }
}

static void
rtl8127_disable_pci_offset_99(struct rtl8127_private *tp)
{
        rtl8127_mac_ocp_write(tp, 0xE032, rtl8127_mac_ocp_read(tp, 0xE032) & ~(BIT_0 | BIT_1));

        rtl8127_csi_fun0_write_byte(tp, 0x99, 0x00);
}

static void
rtl8127_enable_pci_offset_99(struct rtl8127_private *tp)
{
        u32 csi_tmp;

        rtl8127_csi_fun0_write_byte(tp, 0x99, tp->org_pci_offset_99);

        csi_tmp = rtl8127_mac_ocp_read(tp, 0xE032);
        csi_tmp &= ~(BIT_0 | BIT_1);
        if (tp->org_pci_offset_99 & (BIT_5 | BIT_6))
                csi_tmp |= BIT_1;
        if (tp->org_pci_offset_99 & BIT_2)
                csi_tmp |= BIT_0;
        rtl8127_mac_ocp_write(tp, 0xE032, csi_tmp);
}

static void
rtl8127_init_pci_offset_99(struct rtl8127_private *tp)
{
        rtl8127_mac_ocp_write(tp, 0xCDD0, 0x9003);
        rtl8127_set_mac_ocp_bit(tp, 0xE034, (BIT_15 | BIT_14));
        rtl8127_mac_ocp_write(tp, 0xCDD2, 0x8C17);
        rtl8127_mac_ocp_write(tp, 0xCDD8, 0x9003);
        rtl8127_mac_ocp_write(tp, 0xCDD4, 0x9003);
        rtl8127_mac_ocp_write(tp, 0xCDDA, 0x9003);
        rtl8127_mac_ocp_write(tp, 0xCDD6, 0x9003);
        rtl8127_mac_ocp_write(tp, 0xCDDC, 0x9003);
        rtl8127_mac_ocp_write(tp, 0xCDE8, 0x8C08);
        rtl8127_mac_ocp_write(tp, 0xCDEA, 0x9003);
        rtl8127_mac_ocp_write(tp, 0xCDEC, 0x8C12);
        rtl8127_mac_ocp_write(tp, 0xCDEE, 0x9003);
        rtl8127_mac_ocp_write(tp, 0xCDF0, 0x8C2E);
        rtl8127_mac_ocp_write(tp, 0xCDF2, 0x9003);
        rtl8127_mac_ocp_write(tp, 0xCDF4, 0x8892);
        rtl8127_mac_ocp_write(tp, 0xCDF6, 0x9003);
        rtl8127_mac_ocp_write(tp, 0xCDF4, 0x8849);
        rtl8127_mac_ocp_write(tp, 0xCDF6, 0x9003);
        rtl8127_set_mac_ocp_bit(tp, 0xE032, BIT_14);
        rtl8127_set_mac_ocp_bit(tp, 0xE0A2, BIT_0);

        rtl8127_enable_pci_offset_99(tp);
}

static void
rtl8127_disable_pci_offset_180(struct rtl8127_private *tp)
{
        rtl8127_clear_mac_ocp_bit(tp, 0xE092, 0x00FF);
}

static void
rtl8127_enable_pci_offset_180(struct rtl8127_private *tp)
{
        rtl8127_clear_mac_ocp_bit(tp, 0xE094, 0xFF00);

        rtl8127_clear_set_mac_ocp_bit(tp, 0xE092, 0x00FF, BIT_2);
}

static void
rtl8127_init_pci_offset_180(struct rtl8127_private *tp)
{
        rtl8127_enable_pci_offset_180(tp);
}

static void
rtl8127_set_pci_99_exit_driver_para(struct net_device *dev)
{
        struct rtl8127_private *tp = netdev_priv(dev);

        if (tp->org_pci_offset_99 & BIT_2)
                rtl8127_issue_offset_99_event(tp);
        rtl8127_disable_pci_offset_99(tp);
}

static void
rtl8127_enable_cfg9346_write(struct rtl8127_private *tp)
{
        RTL_W8(tp, Cfg9346, RTL_R8(tp, Cfg9346) | Cfg9346_Unlock);
}

static void
rtl8127_disable_cfg9346_write(struct rtl8127_private *tp)
{
        RTL_W8(tp, Cfg9346, RTL_R8(tp, Cfg9346) & ~Cfg9346_Unlock);
}

static void
rtl8127_enable_exit_l1_mask(struct rtl8127_private *tp)
{
        //(1)ERI(0xD4)(OCP 0xC0AC).bit[7:12]=6'b111111, L1 Mask
        rtl8127_set_mac_ocp_bit(tp, 0xC0AC, (BIT_7 | BIT_8 | BIT_9 | BIT_10 | BIT_11 | BIT_12));
}

static void
rtl8127_disable_exit_l1_mask(struct rtl8127_private *tp)
{
        //(1)ERI(0xD4)(OCP 0xC0AC).bit[7:12]=6'b000000, L1 Mask
        rtl8127_clear_mac_ocp_bit(tp, 0xC0AC, (BIT_7 | BIT_8 | BIT_9 | BIT_10 | BIT_11 | BIT_12));
}

static void
rtl8127_enable_extend_tally_couter(struct rtl8127_private *tp)
{
        switch (tp->HwSuppExtendTallyCounterVer) {
        case 1:
                rtl8127_set_mac_ocp_bit(tp, 0xEA84, (BIT_1 | BIT_0));
                break;
        }
}

static void
rtl8127_disable_extend_tally_couter(struct rtl8127_private *tp)
{
        switch (tp->HwSuppExtendTallyCounterVer) {
        case 1:
                rtl8127_clear_mac_ocp_bit(tp, 0xEA84, (BIT_1 | BIT_0));
                break;
        }
}

static void
rtl8127_enable_force_clkreq(struct rtl8127_private *tp, bool enable)
{
        if (enable)
                RTL_W8(tp, 0xF1, RTL_R8(tp, 0xF1) | BIT_7);
        else
                RTL_W8(tp, 0xF1, RTL_R8(tp, 0xF1) & ~BIT_7);
}

static void
rtl8127_enable_aspm_clkreq_lock(struct rtl8127_private *tp, bool enable)
{
        bool unlock_cfg_wr;

        if ((RTL_R8(tp, Cfg9346) & Cfg9346_EEM_MASK) == Cfg9346_Unlock)
                unlock_cfg_wr = false;
        else
                unlock_cfg_wr = true;

        if (unlock_cfg_wr)
                rtl8127_enable_cfg9346_write(tp);

        if (enable) {
                RTL_W8(tp, INT_CFG0_8125, RTL_R8(tp, INT_CFG0_8125) | BIT_3);
                RTL_W8(tp, Config5, RTL_R8(tp, Config5) | BIT_0);
        } else {
                RTL_W8(tp, INT_CFG0_8125, RTL_R8(tp, INT_CFG0_8125) & ~BIT_3);
                RTL_W8(tp, Config5, RTL_R8(tp, Config5) & ~BIT_0);
        }

        if (unlock_cfg_wr)
                rtl8127_disable_cfg9346_write(tp);
}

static void
rtl8127_hw_d3_para(struct net_device *dev)
{
        struct rtl8127_private *tp = netdev_priv(dev);

        RTL_W16(tp, RxMaxSize, RX_BUF_SIZE);

        rtl8127_enable_force_clkreq(tp, 0);
        rtl8127_enable_aspm_clkreq_lock(tp, 0);

        rtl8127_disable_exit_l1_mask(tp);

#ifdef ENABLE_REALWOW_SUPPORT
        rtl8127_set_realwow_d3_para(dev);
#endif

        rtl8127_set_pci_99_exit_driver_para(dev);

        rtl8127_disable_rxdvgate(dev);

        rtl8127_disable_extend_tally_couter(tp);

        rtl8127_set_radm_fifo_prot(tp, 1);
}

static void
rtl8127_enable_magic_packet(struct net_device *dev)
{
        struct rtl8127_private *tp = netdev_priv(dev);

        switch (tp->HwSuppMagicPktVer) {
        case WAKEUP_MAGIC_PACKET_V3:
                rtl8127_mac_ocp_write(tp, 0xC0B6, rtl8127_mac_ocp_read(tp, 0xC0B6) | BIT_0);
                break;
        }
}
static void
rtl8127_disable_magic_packet(struct net_device *dev)
{
        struct rtl8127_private *tp = netdev_priv(dev);

        switch (tp->HwSuppMagicPktVer) {
        case WAKEUP_MAGIC_PACKET_V3:
                rtl8127_mac_ocp_write(tp, 0xC0B6, rtl8127_mac_ocp_read(tp, 0xC0B6) & ~BIT_0);
                break;
        }
}

static void
rtl8127_enable_linkchg_wakeup(struct net_device *dev)
{
        struct rtl8127_private *tp = netdev_priv(dev);

        switch (tp->HwSuppLinkChgWakeUpVer) {
        case 3:
                RTL_W8(tp, Config3, RTL_R8(tp, Config3) | LinkUp);
                rtl8127_clear_set_mac_ocp_bit(tp, 0xE0C6, (BIT_5 | BIT_3 | BIT_2), (BIT_4 | BIT_1 | BIT_0));
                break;
        }
}

static void
rtl8127_disable_linkchg_wakeup(struct net_device *dev)
{
        struct rtl8127_private *tp = netdev_priv(dev);

        switch (tp->HwSuppLinkChgWakeUpVer) {
        case 3:
                RTL_W8(tp, Config3, RTL_R8(tp, Config3) & ~LinkUp);
                if (!(rtl8127_mac_ocp_read(tp, 0xE0C6) & BIT_0))
                        rtl8127_clear_set_mac_ocp_bit(tp, 0xE0C6, (BIT_5 | BIT_3 | BIT_2 | BIT_1), BIT_4);
                break;
        }
}

#define WAKE_ANY (WAKE_PHY | WAKE_MAGIC | WAKE_UCAST | WAKE_BCAST | WAKE_MCAST)

static u32
rtl8127_get_hw_wol(struct rtl8127_private *tp)
{
        u8 options;
        u32 csi_tmp;
        u32 wol_opts = 0;

        if (disable_wol_support)
                goto out;

        options = RTL_R8(tp, Config1);
        if (!(options & PMEnable))
                goto out;

        options = RTL_R8(tp, Config3);
        if (options & LinkUp)
                wol_opts |= WAKE_PHY;

        switch (tp->HwSuppMagicPktVer) {
        case WAKEUP_MAGIC_PACKET_V3:
                csi_tmp = rtl8127_mac_ocp_read(tp, 0xC0B6);
                if (csi_tmp & BIT_0)
                        wol_opts |= WAKE_MAGIC;
                break;
        }

        options = RTL_R8(tp, Config5);
        if (options & UWF)
                wol_opts |= WAKE_UCAST;
        if (options & BWF)
                wol_opts |= WAKE_BCAST;
        if (options & MWF)
                wol_opts |= WAKE_MCAST;

out:
        return wol_opts;
}

static void
rtl8127_enable_d0_speedup(struct rtl8127_private *tp)
{
        u16 clearmask;
        u16 setmask;

        if (FALSE == HW_SUPPORT_D0_SPEED_UP(tp))
                return;

        if (tp->D0SpeedUpSpeed == D0_SPEED_UP_SPEED_DISABLE)
                return;

        if (tp->HwSuppD0SpeedUpVer == 1 || tp->HwSuppD0SpeedUpVer == 2) {
                //speed up speed
                clearmask = (BIT_10 | BIT_9 | BIT_8 | BIT_7);
                if (tp->D0SpeedUpSpeed == D0_SPEED_UP_SPEED_2500)
                        setmask = BIT_7;
                else if (tp->D0SpeedUpSpeed == D0_SPEED_UP_SPEED_5000)
                        setmask = BIT_8;
                else if (tp->D0SpeedUpSpeed == D0_SPEED_UP_SPEED_10000)
                        setmask = BIT_7 | BIT_8;
                else
                        setmask = 0;
                rtl8127_clear_set_mac_ocp_bit(tp, 0xE10A, clearmask, setmask);

                //speed up flowcontrol
                clearmask = (BIT_15 | BIT_14);
                if (tp->HwSuppD0SpeedUpVer == 2)
                        clearmask |= BIT_13;

                if (tp->fcpause == rtl8127_fc_full) {
                        setmask = (BIT_15 | BIT_14);
                        if (tp->HwSuppD0SpeedUpVer == 2)
                                setmask |= BIT_13;
                } else
                        setmask = 0;
                rtl8127_clear_set_mac_ocp_bit(tp, 0xE860, clearmask, setmask);
        }

        RTL_W8(tp, 0xD0, RTL_R8(tp, 0xD0) | BIT_3);
}

static void
rtl8127_disable_d0_speedup(struct rtl8127_private *tp)
{
        if (FALSE == HW_SUPPORT_D0_SPEED_UP(tp))
                return;

        RTL_W8(tp, 0xD0, RTL_R8(tp, 0xD0) & ~BIT_3);
}

static void
rtl8127_set_hw_wol(struct net_device *dev, u32 wolopts)
{
        struct rtl8127_private *tp = netdev_priv(dev);
        int i,tmp = 0;
        static struct {
                u32 opt;
                u16 reg;
                u8  mask;
        } cfg[] = {
                { WAKE_PHY,   Config3, LinkUp },
                { WAKE_UCAST, Config5, UWF },
                { WAKE_BCAST, Config5, BWF },
                { WAKE_MCAST, Config5, MWF },
                { WAKE_ANY,   Config5, LanWake },
                { WAKE_MAGIC, Config3, MagicPacket },
        };

        switch (tp->HwSuppMagicPktVer) {
        case WAKEUP_MAGIC_PACKET_V3:
                tmp = ARRAY_SIZE(cfg) - 1;

                if (wolopts & WAKE_MAGIC)
                        rtl8127_enable_magic_packet(dev);
                else
                        rtl8127_disable_magic_packet(dev);
                break;
        default:
                break;
        }

        rtl8127_enable_cfg9346_write(tp);

        for (i = 0; i < tmp; i++) {
                u8 options = RTL_R8(tp, cfg[i].reg) & ~cfg[i].mask;
                if (wolopts & cfg[i].opt)
                        options |= cfg[i].mask;
                RTL_W8(tp, cfg[i].reg, options);
        }

        switch (tp->HwSuppLinkChgWakeUpVer) {
        case 3:
                if (wolopts & WAKE_PHY)
                        rtl8127_enable_linkchg_wakeup(dev);
                else
                        rtl8127_disable_linkchg_wakeup(dev);
                break;
        }

        rtl8127_disable_cfg9346_write(tp);
}

static void
rtl8127_phy_restart_nway(struct net_device *dev)
{
        struct rtl8127_private *tp = netdev_priv(dev);

        if (rtl8127_is_in_phy_disable_mode(dev))
                return;

        rtl8127_mdio_write(tp, 0x1F, 0x0000);
        rtl8127_mdio_write(tp, MII_BMCR, BMCR_ANENABLE | BMCR_ANRESTART);
}

static void
rtl8127_phy_setup_force_mode(struct net_device *dev, u32 speed, u8 duplex)
{
        struct rtl8127_private *tp = netdev_priv(dev);
        u16 bmcr_true_force = 0;

        if (rtl8127_is_in_phy_disable_mode(dev))
                return;

        if ((speed == SPEED_10) && (duplex == DUPLEX_HALF)) {
                bmcr_true_force = BMCR_SPEED10;
        } else if ((speed == SPEED_10) && (duplex == DUPLEX_FULL)) {
                bmcr_true_force = BMCR_SPEED10 | BMCR_FULLDPLX;
        } else if ((speed == SPEED_100) && (duplex == DUPLEX_HALF)) {
                bmcr_true_force = BMCR_SPEED100;
        } else if ((speed == SPEED_100) && (duplex == DUPLEX_FULL)) {
                bmcr_true_force = BMCR_SPEED100 | BMCR_FULLDPLX;
        } else {
                netif_err(tp, drv, dev, "Failed to set phy force mode!\n");
                return;
        }

        rtl8127_mdio_write(tp, 0x1F, 0x0000);
        rtl8127_mdio_write(tp, MII_BMCR, bmcr_true_force);
}

static void
rtl8127_set_pci_pme(struct rtl8127_private *tp, int set)
{
        struct pci_dev *pdev = tp->pci_dev;
        u16 pmc;

        if (!pdev->pm_cap)
                return;

        pci_read_config_word(pdev, pdev->pm_cap + PCI_PM_CTRL, &pmc);
        pmc |= PCI_PM_CTRL_PME_STATUS;
        if (set)
                pmc |= PCI_PM_CTRL_PME_ENABLE;
        else
                pmc &= ~PCI_PM_CTRL_PME_ENABLE;
        pci_write_config_word(pdev, pdev->pm_cap + PCI_PM_CTRL, pmc);
}

static void
rtl8127_enable_giga_lite(struct rtl8127_private *tp, u64 adv)
{
        if (adv & ADVERTISED_1000baseT_Full)
                rtl8127_set_eth_phy_ocp_bit(tp, 0xA428, BIT_9);
        else
                rtl8127_clear_eth_phy_ocp_bit(tp, 0xA428, BIT_9);

        if (adv & ADVERTISED_2500baseX_Full)
                rtl8127_set_eth_phy_ocp_bit(tp, 0xA5EA, BIT_0);
        else
                rtl8127_clear_eth_phy_ocp_bit(tp, 0xA5EA, BIT_0);

        if (adv & RTK_ADVERTISED_5000baseX_Full)
                rtl8127_set_eth_phy_ocp_bit(tp, 0xA5EA, BIT_1);
        else
                rtl8127_clear_eth_phy_ocp_bit(tp, 0xA5EA, BIT_1);

        if (adv & ADVERTISED_10000baseT_Full)
                rtl8127_set_eth_phy_ocp_bit(tp, 0xA5EA, BIT_2);
        else
                rtl8127_clear_eth_phy_ocp_bit(tp, 0xA5EA, BIT_2);
}

static void
rtl8127_disable_giga_lite(struct rtl8127_private *tp)
{
        rtl8127_clear_eth_phy_ocp_bit(tp, 0xA428, BIT_9);
        rtl8127_clear_eth_phy_ocp_bit(tp, 0xA5EA, BIT_0 | BIT_1 | BIT_2);
}

static void
rtl8127_set_wol_link_speed(struct net_device *dev)
{
        struct rtl8127_private *tp = netdev_priv(dev);
        unsigned long flags;
        int auto_nego;
        int giga_ctrl;
        int ctrl_2500;
        u64 adv;
        u16 anlpar;
        u16 gbsr;
        u16 status_2500;
        u16 aner;

        r8127_spin_lock(&tp->phy_lock, flags);

        if (tp->autoneg != AUTONEG_ENABLE)
                goto exit;

        rtl8127_mdio_write(tp, 0x1F, 0x0000);

        auto_nego = rtl8127_mdio_read(tp, MII_ADVERTISE);
        auto_nego &= ~(ADVERTISE_10HALF | ADVERTISE_10FULL
                       | ADVERTISE_100HALF | ADVERTISE_100FULL);

        giga_ctrl = rtl8127_mdio_read(tp, MII_CTRL1000);
        giga_ctrl &= ~(ADVERTISE_1000HALF | ADVERTISE_1000FULL);

        ctrl_2500 = rtl8127_mdio_direct_read_phy_ocp(tp, 0xA5D4);
        ctrl_2500 &= ~(RTK_ADVERTISE_2500FULL | RTK_ADVERTISE_5000FULL |
                       RTK_ADVERTISE_10000FULL);

        aner = tp->phy_reg_aner;
        anlpar = tp->phy_reg_anlpar;
        gbsr = tp->phy_reg_gbsr;
        status_2500 = tp->phy_reg_status_2500;
        if (tp->link_ok(dev)) {
                aner = rtl8127_mdio_read(tp, MII_EXPANSION);
                anlpar = rtl8127_mdio_read(tp, MII_LPA);
                gbsr = rtl8127_mdio_read(tp, MII_STAT1000);
                status_2500 = rtl8127_mdio_direct_read_phy_ocp(tp, 0xA5D6);
        }

        adv = tp->advertising;
        if ((aner | anlpar | gbsr | status_2500) == 0) {
                int auto_nego_tmp = 0;
                if (adv & ADVERTISED_10baseT_Half)
                        auto_nego_tmp |= ADVERTISE_10HALF;
                if (adv & ADVERTISED_10baseT_Full)
                        auto_nego_tmp |= ADVERTISE_10FULL;
                if (adv & ADVERTISED_100baseT_Half)
                        auto_nego_tmp |= ADVERTISE_100HALF;
                if (adv & ADVERTISED_100baseT_Full)
                        auto_nego_tmp |= ADVERTISE_100FULL;

                if (auto_nego_tmp == 0)
                        goto exit;

                auto_nego |= auto_nego_tmp;
                goto skip_check_lpa;
        }
        if (!(aner & EXPANSION_NWAY))
                goto exit;

        if ((adv & ADVERTISED_10baseT_Half) && (anlpar & LPA_10HALF))
                auto_nego |= ADVERTISE_10HALF;
        else if ((adv & ADVERTISED_10baseT_Full) && (anlpar & LPA_10FULL))
                auto_nego |= ADVERTISE_10FULL;
        else if ((adv & ADVERTISED_100baseT_Half) && (anlpar & LPA_100HALF))
                auto_nego |= ADVERTISE_100HALF;
        else if ((adv & ADVERTISED_100baseT_Full) && (anlpar & LPA_100FULL))
                auto_nego |= ADVERTISE_100FULL;
        else if (adv & ADVERTISED_1000baseT_Half && (gbsr & LPA_1000HALF))
                giga_ctrl |= ADVERTISE_1000HALF;
        else if (adv & ADVERTISED_1000baseT_Full && (gbsr & LPA_1000FULL))
                giga_ctrl |= ADVERTISE_1000FULL;
        else if (adv & ADVERTISED_2500baseX_Full && (status_2500 & RTK_LPA_ADVERTISE_2500FULL))
                ctrl_2500 |= RTK_ADVERTISE_2500FULL;
        else if (adv & RTK_ADVERTISED_5000baseX_Full && (status_2500 & RTK_LPA_ADVERTISE_5000FULL))
                ctrl_2500 |= RTK_ADVERTISE_5000FULL;
        else if (adv & ADVERTISED_10000baseT_Full && (status_2500 & RTK_LPA_ADVERTISE_10000FULL))
                ctrl_2500 |= RTK_ADVERTISE_10000FULL;
        else
                goto exit;

skip_check_lpa:
        if (tp->DASH)
                auto_nego |= (ADVERTISE_100FULL | ADVERTISE_100HALF | ADVERTISE_10HALF | ADVERTISE_10FULL);

#ifdef CONFIG_DOWN_SPEED_100
        auto_nego |= (ADVERTISE_100FULL | ADVERTISE_100HALF | ADVERTISE_10HALF | ADVERTISE_10FULL);
#endif

        rtl8127_mdio_write(tp, MII_ADVERTISE, auto_nego);
        rtl8127_mdio_write(tp, MII_CTRL1000, giga_ctrl);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA5D4, ctrl_2500);

        rtl8127_disable_giga_lite(tp);

        rtl8127_phy_restart_nway(dev);

exit:
        r8127_spin_unlock(&tp->phy_lock, flags);

        return;
}

static bool
rtl8127_keep_wol_link_speed(struct net_device *dev, u8 from_suspend)
{
        struct rtl8127_private *tp = netdev_priv(dev);

        if (from_suspend && tp->link_ok(dev) && (tp->wol_opts & WAKE_PHY))
                return 1;

        if (!from_suspend && tp->resume_not_chg_speed)
                return 1;

        return 0;
}

static void
rtl8127_set_wol_rx_packet_filter(struct rtl8127_private *tp)
{
        RTL_W32(tp, RxConfig, RTL_R32(tp, RxConfig) | AcceptBroadcast |
                AcceptMulticast | AcceptMyPhys);
}

static void
rtl8127_powerdown_pll(struct net_device *dev, u8 from_suspend)
{
        struct rtl8127_private *tp = netdev_priv(dev);

        /* Reboot not set wol link speed */
        if (system_state == SYSTEM_RESTART)
                return;

        tp->check_keep_link_speed = 0;
        if (tp->wol_enabled == WOL_ENABLED || tp->DASH || tp->EnableKCPOffload) {
                rtl8127_set_hw_wol(dev, tp->wol_opts);

                rtl8127_enable_cfg9346_write(tp);
                RTL_W8(tp, Config2, RTL_R8(tp, Config2) | PMSTS_En);
                rtl8127_disable_cfg9346_write(tp);

                /* Enable the PME and clear the status */
                rtl8127_set_pci_pme(tp, 1);

                rtl8127_set_wol_rx_packet_filter(tp);

                if (rtl8127_keep_wol_link_speed(dev, from_suspend)) {
                        tp->check_keep_link_speed = 1;
                } else {
                        if (tp->D0SpeedUpSpeed != D0_SPEED_UP_SPEED_DISABLE) {
                                rtl8127_enable_d0_speedup(tp);
                                tp->check_keep_link_speed = 1;
                        }

                        rtl8127_set_wol_link_speed(dev);
                }

                return;
        }

        if (tp->DASH)
                return;

        rtl8127_phy_power_down(dev);

        RTL_W8(tp, 0xF2, RTL_R8(tp, 0xF2) & ~BIT_6);
}

static void rtl8127_powerup_pll(struct net_device *dev)
{
        struct rtl8127_private *tp = netdev_priv(dev);

        RTL_W8(tp, PMCH, RTL_R8(tp, PMCH) | BIT_7 | BIT_6);

        if (tp->resume_not_chg_speed)
                return;

        rtl8127_phy_power_up(dev);
}

static void
rtl8127_get_wol(struct net_device *dev,
                struct ethtool_wolinfo *wol)
{
        struct rtl8127_private *tp = netdev_priv(dev);
        u8 options;

        wol->wolopts = 0;

        if (tp->mcfg == CFG_METHOD_DEFAULT || disable_wol_support) {
                wol->supported = 0;
                return;
        } else {
                wol->supported = WAKE_ANY;
        }

        options = RTL_R8(tp, Config1);
        if (!(options & PMEnable))
                return;

        wol->wolopts = tp->wol_opts;
}

static int
rtl8127_set_wol(struct net_device *dev,
                struct ethtool_wolinfo *wol)
{
        struct rtl8127_private *tp = netdev_priv(dev);

        if (tp->mcfg == CFG_METHOD_DEFAULT || disable_wol_support)
                return -EOPNOTSUPP;

        tp->wol_opts = wol->wolopts;

        tp->wol_enabled = (tp->wol_opts) ? WOL_ENABLED : WOL_DISABLED;

        device_set_wakeup_enable(tp_to_dev(tp), wol->wolopts);

        return 0;
}

static void
rtl8127_get_drvinfo(struct net_device *dev,
                    struct ethtool_drvinfo *info)
{
        struct rtl8127_private *tp = netdev_priv(dev);

        strscpy(info->driver, MODULENAME, sizeof(info->driver));
        strscpy(info->version, RTL8127_VERSION, sizeof(info->version));
        strscpy(info->bus_info, pci_name(tp->pci_dev), sizeof(info->bus_info));
        info->regdump_len = R8127_REGS_DUMP_SIZE;
        info->eedump_len = tp->eeprom_len;
}

static int
rtl8127_get_regs_len(struct net_device *dev)
{
        return R8127_REGS_DUMP_SIZE;
}

static void
rtl8127_set_d0_speedup_speed(struct rtl8127_private *tp)
{
        if (FALSE == HW_SUPPORT_D0_SPEED_UP(tp))
                return;

        tp->D0SpeedUpSpeed = D0_SPEED_UP_SPEED_DISABLE;
        if (tp->autoneg == AUTONEG_ENABLE) {
                if (tp->speed == SPEED_10000)
                        tp->D0SpeedUpSpeed = D0_SPEED_UP_SPEED_10000;
                else if (tp->speed == SPEED_5000)
                        tp->D0SpeedUpSpeed = D0_SPEED_UP_SPEED_5000;
                else if (tp->speed == SPEED_2500)
                        tp->D0SpeedUpSpeed = D0_SPEED_UP_SPEED_2500;
                else if (tp->speed == SPEED_1000)
                        tp->D0SpeedUpSpeed = D0_SPEED_UP_SPEED_1000;
        }
}

static int
rtl8127_set_speed_xmii(struct net_device *dev,
                       u8 autoneg,
                       u32 speed,
                       u8 duplex,
                       u64 adv)
{
        struct rtl8127_private *tp = netdev_priv(dev);
        unsigned long flags;
        int auto_nego = 0;
        int giga_ctrl = 0;
        int ctrl_2500 = 0;
        int rc = -EINVAL;

        r8127_spin_lock(&tp->phy_lock, flags);

        if (!rtl8127_is_speed_mode_valid(tp, speed)) {
                speed = SPEED_10000;
                duplex = DUPLEX_FULL;
                adv |= tp->advertising;
        }

        if (eee_giga_lite && (autoneg == AUTONEG_ENABLE))
                rtl8127_enable_giga_lite(tp, adv);
        else
                rtl8127_disable_giga_lite(tp);

        if (HW_FIBER_MODE_ENABLED(tp))
                goto set_speed;

        giga_ctrl = rtl8127_mdio_read(tp, MII_CTRL1000);
        giga_ctrl &= ~(ADVERTISE_1000HALF | ADVERTISE_1000FULL);
        ctrl_2500 = rtl8127_mdio_direct_read_phy_ocp(tp, 0xA5D4);
        ctrl_2500 &= ~(RTK_ADVERTISE_2500FULL | RTK_ADVERTISE_5000FULL |
                       RTK_ADVERTISE_10000FULL);

        if (autoneg == AUTONEG_ENABLE) {
                /*n-way force*/
                auto_nego = rtl8127_mdio_read(tp, MII_ADVERTISE);
                auto_nego &= ~(ADVERTISE_10HALF | ADVERTISE_10FULL |
                               ADVERTISE_100HALF | ADVERTISE_100FULL |
                               ADVERTISE_PAUSE_CAP | ADVERTISE_PAUSE_ASYM);

                if (adv & ADVERTISED_10baseT_Half)
                        auto_nego |= ADVERTISE_10HALF;
                if (adv & ADVERTISED_10baseT_Full)
                        auto_nego |= ADVERTISE_10FULL;
                if (adv & ADVERTISED_100baseT_Half)
                        auto_nego |= ADVERTISE_100HALF;
                if (adv & ADVERTISED_100baseT_Full)
                        auto_nego |= ADVERTISE_100FULL;
                if (adv & ADVERTISED_1000baseT_Half)
                        giga_ctrl |= ADVERTISE_1000HALF;
                if (adv & ADVERTISED_1000baseT_Full)
                        giga_ctrl |= ADVERTISE_1000FULL;
                if (adv & ADVERTISED_2500baseX_Full)
                        ctrl_2500 |= RTK_ADVERTISE_2500FULL;
                if (HW_SUPP_PHY_LINK_SPEED_5000M(tp)) {
                        if (adv & RTK_ADVERTISED_5000baseX_Full)
                                ctrl_2500 |= RTK_ADVERTISE_5000FULL;
                }
                if (HW_SUPP_PHY_LINK_SPEED_10000M(tp)) {
                        if (adv & ADVERTISED_10000baseT_Full)
                                ctrl_2500 |= RTK_ADVERTISE_10000FULL;
                }

                //flow control
                if (tp->fcpause == rtl8127_fc_full)
                        auto_nego |= ADVERTISE_PAUSE_CAP | ADVERTISE_PAUSE_ASYM;

                tp->phy_auto_nego_reg = auto_nego;
                tp->phy_1000_ctrl_reg = giga_ctrl;

                tp->phy_2500_ctrl_reg = ctrl_2500;

                rtl8127_mdio_write(tp, 0x1f, 0x0000);
                rtl8127_mdio_write(tp, MII_ADVERTISE, auto_nego);
                rtl8127_mdio_write(tp, MII_CTRL1000, giga_ctrl);
                rtl8127_mdio_direct_write_phy_ocp(tp, 0xA5D4, ctrl_2500);
                rtl8127_phy_restart_nway(dev);
        } else {
                /*true force*/
                if (speed == SPEED_10 || speed == SPEED_100)
                        rtl8127_phy_setup_force_mode(dev, speed, duplex);
                else
                        goto out;
        }

set_speed:
        tp->autoneg = autoneg;
        tp->speed = speed;
        tp->duplex = duplex;
        tp->advertising = adv;

        rtl8127_set_d0_speedup_speed(tp);

        rtl8127_hw_fiber_phy_config(tp);

        rc = 0;
out:
        r8127_spin_unlock(&tp->phy_lock, flags);

        return rc;
}

static int
rtl8127_set_speed(struct net_device *dev,
                  u8 autoneg,
                  u32 speed,
                  u8 duplex,
                  u64 adv)
{
        struct rtl8127_private *tp = netdev_priv(dev);
        int ret;

        if (tp->resume_not_chg_speed)
                return 0;

        ret = tp->set_speed(dev, autoneg, speed, duplex, adv);

        return ret;
}

static int
rtl8127_set_settings(struct net_device *dev,
                     const struct ethtool_link_ksettings *cmd
                    )
{
        int ret;
        u8 autoneg;
        u32 speed;
        u8 duplex;
        u64 supported = 0, advertising = 0;

        struct rtl8127_private *tp = netdev_priv(dev);
        const struct ethtool_link_settings *base = &cmd->base;
        autoneg = base->autoneg;
        speed = base->speed;
        duplex = base->duplex;
        ethtool_convert_link_mode_to_legacy_u32((u32*)&supported,
                                                cmd->link_modes.supported);
        ethtool_convert_link_mode_to_legacy_u32((u32*)&advertising,
                                                cmd->link_modes.advertising);
        if (test_bit(ETHTOOL_LINK_MODE_2500baseT_Full_BIT,
                     cmd->link_modes.supported))
                supported |= ADVERTISED_2500baseX_Full;
        if (test_bit(ETHTOOL_LINK_MODE_2500baseT_Full_BIT,
                     cmd->link_modes.advertising))
                advertising |= ADVERTISED_2500baseX_Full;
        if (HW_SUPP_PHY_LINK_SPEED_5000M(tp)) {
                if (test_bit(ETHTOOL_LINK_MODE_5000baseT_Full_BIT,
                             cmd->link_modes.supported))
                        supported |= RTK_ADVERTISED_5000baseX_Full;
                if (test_bit(ETHTOOL_LINK_MODE_5000baseT_Full_BIT,
                             cmd->link_modes.advertising))
                        advertising |= RTK_ADVERTISED_5000baseX_Full;
        }
        if (HW_SUPP_PHY_LINK_SPEED_10000M(tp)) {
                if (test_bit(ETHTOOL_LINK_MODE_10000baseT_Full_BIT,
                             cmd->link_modes.supported))
                        supported |= ADVERTISED_10000baseT_Full;
                if (test_bit(ETHTOOL_LINK_MODE_10000baseT_Full_BIT,
                             cmd->link_modes.advertising))
                        advertising |= ADVERTISED_10000baseT_Full;
        }
        if (advertising & ~supported)
                return -EINVAL;

        ret = rtl8127_set_speed(dev, autoneg, speed, duplex, advertising);

        return ret;
}

static u32
rtl8127_rx_desc_opts1(struct rtl8127_private *tp,
                      struct RxDesc *desc)
{
        switch (tp->InitRxDescType) {
        case RX_DESC_RING_TYPE_3:
                return READ_ONCE(((struct RxDescV3 *)desc)->RxDescNormalDDWord4.opts1);
        case RX_DESC_RING_TYPE_4:
                return READ_ONCE(((struct RxDescV4 *)desc)->RxDescNormalDDWord2.opts1);
        default:
                return READ_ONCE(desc->opts1);
        }
}

static u32
rtl8127_rx_desc_opts2(struct rtl8127_private *tp,
                      struct RxDesc *desc)
{
        switch (tp->InitRxDescType) {
        case RX_DESC_RING_TYPE_3:
                return ((struct RxDescV3 *)desc)->RxDescNormalDDWord4.opts2;
        case RX_DESC_RING_TYPE_4:
                return ((struct RxDescV4 *)desc)->RxDescNormalDDWord2.opts2;
        default:
                return desc->opts2;
        }
}

#ifdef CONFIG_R8127_VLAN

static void
rtl8127_clear_rx_desc_opts2(struct rtl8127_private *tp,
                            struct RxDesc *desc)
{
        switch (tp->InitRxDescType) {
        case RX_DESC_RING_TYPE_3:
                ((struct RxDescV3 *)desc)->RxDescNormalDDWord4.opts2 = 0;
                break;
        case RX_DESC_RING_TYPE_4:
                ((struct RxDescV4 *)desc)->RxDescNormalDDWord2.opts2 = 0;
                break;
        default:
                desc->opts2 = 0;
                break;
        }
}


static inline u32
rtl8127_tx_vlan_tag(struct rtl8127_private *tp,
                    struct sk_buff *skb)
{
        return (skb_vlan_tag_present(skb)) ?
               TxVlanTag | swab16(skb_vlan_tag_get(skb)) : 0x00;

        return 0;
}

static int
rtl8127_rx_vlan_skb(struct rtl8127_private *tp,
                    struct RxDesc *desc,
                    struct sk_buff *skb)
{
        u32 opts2 = le32_to_cpu(rtl8127_rx_desc_opts2(tp, desc));
        int ret = -1;

        if (opts2 & RxVlanTag)
                __vlan_hwaccel_put_tag(skb, htons(ETH_P_8021Q), swab16(opts2 & 0xffff));

        rtl8127_clear_rx_desc_opts2(tp, desc);
        return ret;
}

#else /* !CONFIG_R8127_VLAN */

static inline u32
rtl8127_tx_vlan_tag(struct rtl8127_private *tp,
                    struct sk_buff *skb)
{
        return 0;
}

static int
rtl8127_rx_vlan_skb(struct rtl8127_private *tp,
                    struct RxDesc *desc,
                    struct sk_buff *skb)
{
        return -1;
}

#endif

static netdev_features_t rtl8127_fix_features(struct net_device *dev,
                netdev_features_t features)
{
        if (dev->mtu > MSS_MAX || dev->mtu > ETH_DATA_LEN)
                features &= ~NETIF_F_ALL_TSO;
#ifndef CONFIG_R8127_VLAN
        features &= ~NETIF_F_ALL_CSUM;
#endif

        return features;
}

static int rtl8127_hw_set_features(struct net_device *dev,
                                   netdev_features_t features)
{
        struct rtl8127_private *tp = netdev_priv(dev);
        u32 rx_config;

        rx_config = RTL_R32(tp, RxConfig);
        if (features & NETIF_F_RXALL) {
                tp->rtl8127_rx_config |= (AcceptErr | AcceptRunt);
                rx_config |= (AcceptErr | AcceptRunt);
        } else {
                tp->rtl8127_rx_config &= ~(AcceptErr | AcceptRunt);
                rx_config &= ~(AcceptErr | AcceptRunt);
        }

        if (features & NETIF_F_HW_VLAN_RX) {
                tp->rtl8127_rx_config |= (EnableInnerVlan | EnableOuterVlan);
                rx_config |= (EnableInnerVlan | EnableOuterVlan);
        } else {
                tp->rtl8127_rx_config &= ~(EnableInnerVlan | EnableOuterVlan);
                rx_config &= ~(EnableInnerVlan | EnableOuterVlan);
        }

        RTL_W32(tp, RxConfig, rx_config);

        if (features & NETIF_F_RXCSUM)
                tp->cp_cmd |= RxChkSum;
        else
                tp->cp_cmd &= ~RxChkSum;

        RTL_W16(tp, CPlusCmd, tp->cp_cmd);
        RTL_R16(tp, CPlusCmd);

        return 0;
}

static int rtl8127_set_features(struct net_device *dev,
                                netdev_features_t features)
{
        features &= NETIF_F_RXALL | NETIF_F_RXCSUM | NETIF_F_HW_VLAN_RX;

        rtl8127_hw_set_features(dev, features);

        return 0;
}

static u8 rtl8127_get_mdi_status(struct rtl8127_private *tp)
{
        if (!tp->link_ok(tp->dev))
                return ETH_TP_MDI_INVALID;

        if (rtl8127_mdio_direct_read_phy_ocp(tp, 0xA444) & BIT_1)
                return ETH_TP_MDI;
        else
                return ETH_TP_MDI_X;
}

static void rtl8127_gset_xmii(struct net_device *dev,
                              struct ethtool_link_ksettings *cmd
                             )
{
        struct rtl8127_private *tp = netdev_priv(dev);
        u16 aner = tp->phy_reg_aner;
        u16 anlpar = tp->phy_reg_anlpar;
        u16 gbsr = tp->phy_reg_gbsr;
        u16 status_2500 = tp->phy_reg_status_2500;
        unsigned long flags;
        u64 lpa_adv = 0;
        u32 status;
        u8 autoneg, duplex;
        u32 speed = 0;
        u16 bmcr;
        u64 supported, advertising;
        u8 report_lpa = 0;

        if (HW_FIBER_MODE_ENABLED(tp)) {
                supported = SUPPORTED_1000baseT_Full |
                            SUPPORTED_10000baseT_Full |
                            SUPPORTED_TP;
        } else {
                supported = SUPPORTED_10baseT_Half |
                            SUPPORTED_10baseT_Full |
                            SUPPORTED_100baseT_Half |
                            SUPPORTED_100baseT_Full |
                            SUPPORTED_1000baseT_Full |
                            SUPPORTED_2500baseX_Full |
                            RTK_SUPPORTED_5000baseX_Full |
                            SUPPORTED_10000baseT_Full |
                            SUPPORTED_Autoneg |
                            SUPPORTED_TP |
                            SUPPORTED_Pause |
                            SUPPORTED_Asym_Pause;
        }

        if (!HW_SUPP_PHY_LINK_SPEED_2500M(tp))
                supported &= ~SUPPORTED_2500baseX_Full;

        if (!HW_SUPP_PHY_LINK_SPEED_5000M(tp))
                supported &= ~RTK_SUPPORTED_5000baseX_Full;

        if (!HW_SUPP_PHY_LINK_SPEED_10000M(tp))
                supported &= ~SUPPORTED_10000baseT_Full;

        advertising = tp->advertising;
        if (tp->phy_auto_nego_reg || tp->phy_1000_ctrl_reg ||
            tp->phy_2500_ctrl_reg) {
                advertising = 0;
                if (tp->phy_auto_nego_reg & ADVERTISE_10HALF)
                        advertising |= ADVERTISED_10baseT_Half;
                if (tp->phy_auto_nego_reg & ADVERTISE_10FULL)
                        advertising |= ADVERTISED_10baseT_Full;
                if (tp->phy_auto_nego_reg & ADVERTISE_100HALF)
                        advertising |= ADVERTISED_100baseT_Half;
                if (tp->phy_auto_nego_reg & ADVERTISE_100FULL)
                        advertising |= ADVERTISED_100baseT_Full;
                if (tp->phy_1000_ctrl_reg & ADVERTISE_1000FULL)
                        advertising |= ADVERTISED_1000baseT_Full;
                if (tp->phy_2500_ctrl_reg & RTK_ADVERTISE_2500FULL)
                        advertising |= ADVERTISED_2500baseX_Full;
                if (tp->phy_2500_ctrl_reg & RTK_ADVERTISE_5000FULL)
                        advertising |= RTK_ADVERTISED_5000baseX_Full;
                if (tp->phy_2500_ctrl_reg & RTK_ADVERTISE_10000FULL)
                        advertising |= ADVERTISED_10000baseT_Full;
        }

        r8127_spin_lock(&tp->phy_lock, flags);

        rtl8127_mdio_write(tp, 0x1F, 0x0000);
        bmcr = rtl8127_mdio_read(tp, MII_BMCR);
        if (bmcr & BMCR_ANENABLE) {
                autoneg = AUTONEG_ENABLE;
                advertising |= ADVERTISED_Autoneg;
        } else {
                autoneg = AUTONEG_DISABLE;
        }

        advertising |= ADVERTISED_TP;

        status = rtl8127_get_phy_status(tp);
        if (netif_running(dev) && (status & LinkStatus))
                report_lpa = 1;

        if (report_lpa) {
                /*link on*/
                speed = rtl8127_convert_link_speed(status);

                if (status & TxFlowCtrl)
                        advertising |= ADVERTISED_Asym_Pause;

                if (status & RxFlowCtrl)
                        advertising |= ADVERTISED_Pause;

                duplex = ((status & (_1000bpsF | _2500bpsF | _5000bpsF | _10000bpsF)) ||
                          (status & FullDup)) ?
                         DUPLEX_FULL : DUPLEX_HALF;

                /*link partner*/
                if (aner & EXPANSION_NWAY)
                        lpa_adv |= ADVERTISED_Autoneg;
                if (anlpar & LPA_10HALF)
                        lpa_adv |= ADVERTISED_10baseT_Half;
                if (anlpar & LPA_10FULL)
                        lpa_adv |= ADVERTISED_10baseT_Full;
                if (anlpar & LPA_100HALF)
                        lpa_adv |= ADVERTISED_100baseT_Half;
                if (anlpar & LPA_100FULL)
                        lpa_adv |= ADVERTISED_100baseT_Full;
                if (anlpar & LPA_PAUSE_CAP)
                        lpa_adv |= ADVERTISED_Pause;
                if (anlpar & LPA_PAUSE_ASYM)
                        lpa_adv |= ADVERTISED_Asym_Pause;
                if (gbsr & LPA_1000HALF)
                        lpa_adv |= ADVERTISED_1000baseT_Half;
                if (gbsr & LPA_1000FULL)
                        lpa_adv |= ADVERTISED_1000baseT_Full;
                if (status_2500 & RTK_LPA_ADVERTISE_2500FULL)
                        lpa_adv |= ADVERTISED_2500baseX_Full;
                if (status_2500 & RTK_LPA_ADVERTISE_5000FULL)
                        lpa_adv |= RTK_ADVERTISED_5000baseX_Full;
                if (status_2500 & RTK_LPA_ADVERTISE_10000FULL)
                        lpa_adv |= ADVERTISED_10000baseT_Full;
        } else {
                /*link down*/
                speed = SPEED_UNKNOWN;
                duplex = DUPLEX_UNKNOWN;
                lpa_adv = 0;
        }

        ethtool_convert_legacy_u32_to_link_mode(cmd->link_modes.supported,
                                                supported);
        ethtool_convert_legacy_u32_to_link_mode(cmd->link_modes.advertising,
                                                advertising);
        ethtool_convert_legacy_u32_to_link_mode(cmd->link_modes.lp_advertising,
                                                lpa_adv);

        if (supported & SUPPORTED_2500baseX_Full) {
                linkmode_mod_bit(ETHTOOL_LINK_MODE_2500baseT_Full_BIT,
                                 cmd->link_modes.supported, 1);
        }
        if (advertising & ADVERTISED_2500baseX_Full) {
                linkmode_mod_bit(ETHTOOL_LINK_MODE_2500baseT_Full_BIT,
                                 cmd->link_modes.advertising, 1);
        }
        if (supported & RTK_SUPPORTED_5000baseX_Full) {
                linkmode_mod_bit(ETHTOOL_LINK_MODE_5000baseT_Full_BIT,
                                 cmd->link_modes.supported, 1);
        }
        if (advertising & RTK_ADVERTISED_5000baseX_Full) {
                linkmode_mod_bit(ETHTOOL_LINK_MODE_5000baseT_Full_BIT,
                                 cmd->link_modes.advertising, 1);
        }
        if (supported & SUPPORTED_10000baseT_Full) {
                linkmode_mod_bit(ETHTOOL_LINK_MODE_10000baseT_Full_BIT,
                                 cmd->link_modes.supported, 1);
        }
        if (advertising & ADVERTISED_10000baseT_Full) {
                linkmode_mod_bit(ETHTOOL_LINK_MODE_10000baseT_Full_BIT,
                                 cmd->link_modes.advertising, 1);
        }
        if (report_lpa) {
                if (lpa_adv & ADVERTISED_2500baseX_Full) {
                        linkmode_mod_bit(ETHTOOL_LINK_MODE_2500baseT_Full_BIT,
                                         cmd->link_modes.lp_advertising, 1);
                }
                if (lpa_adv & RTK_ADVERTISED_5000baseX_Full)
                        linkmode_mod_bit(ETHTOOL_LINK_MODE_5000baseT_Full_BIT,
                                         cmd->link_modes.lp_advertising, 1);
                if (lpa_adv & ADVERTISED_10000baseT_Full)
                        linkmode_mod_bit(ETHTOOL_LINK_MODE_10000baseT_Full_BIT,
                                         cmd->link_modes.lp_advertising, 1);
        }

        /* Use ETHTOOL_LINK_MODE_2500baseT_Full_BIT instead of
           ETHTOOL_LINK_MODE_2500baseX_Full_BIT. */
        linkmode_mod_bit(ETHTOOL_LINK_MODE_2500baseX_Full_BIT,
                         cmd->link_modes.supported, 0);

        linkmode_mod_bit(ETHTOOL_LINK_MODE_2500baseX_Full_BIT,
                         cmd->link_modes.advertising, 0);

        linkmode_mod_bit(ETHTOOL_LINK_MODE_2500baseX_Full_BIT,
                         cmd->link_modes.lp_advertising, 0);

        cmd->base.autoneg = autoneg;
        cmd->base.speed = speed;
        cmd->base.duplex = duplex;
        cmd->base.port = PORT_TP;
        cmd->base.eth_tp_mdix = rtl8127_get_mdi_status(tp);
        r8127_spin_unlock(&tp->phy_lock, flags);
}

static int
rtl8127_get_settings(struct net_device *dev,
                     struct ethtool_link_ksettings *cmd
                    )
{
        struct rtl8127_private *tp = netdev_priv(dev);

        tp->get_settings(dev, cmd);

        return 0;
}

static void rtl8127_get_regs(struct net_device *dev, struct ethtool_regs *regs,
                             void *p)
{
        struct rtl8127_private *tp = netdev_priv(dev);
        void __iomem *ioaddr = tp->mmio_addr;
        unsigned int i;
        u8 *data = p;

        if (regs->len < R8127_REGS_DUMP_SIZE)
                return /* -EINVAL */;

        memset(p, 0, regs->len);

        for (i = 0; i < R8127_MAC_REGS_SIZE; i++)
                *data++ = readb(ioaddr + i);
        data = (u8*)p + 256;

        rtl8127_mdio_write(tp, 0x1F, 0x0000);
        for (i = 0; i < R8127_PHY_REGS_SIZE/2; i++) {
                *(u16*)data = rtl8127_mdio_read(tp, i);
                data += 2;
        }
        data = (u8*)p + 256 * 2;

        for (i = 0; i < R8127_EPHY_REGS_SIZE/2; i++) {
                *(u16*)data = rtl8127_ephy_read(tp, i);
                data += 2;
        }
        data = (u8*)p + 256 * 3;

        for (i = 0; i < R8127_ERI_REGS_SIZE; i+=4) {
                *(u32*)data = rtl8127_eri_read(tp, i , 4, ERIAR_ExGMAC);
                data += 4;
        }
}

static void rtl8127_get_pauseparam(struct net_device *dev,
                                   struct ethtool_pauseparam *pause)
{
        struct rtl8127_private *tp = netdev_priv(dev);

        pause->autoneg = (tp->autoneg ? AUTONEG_ENABLE : AUTONEG_DISABLE);
        if (tp->fcpause == rtl8127_fc_rx_pause)
                pause->rx_pause = 1;
        else if (tp->fcpause == rtl8127_fc_tx_pause)
                pause->tx_pause = 1;
        else if (tp->fcpause == rtl8127_fc_full) {
                pause->rx_pause = 1;
                pause->tx_pause = 1;
        }
}

static int rtl8127_set_pauseparam(struct net_device *dev,
                                  struct ethtool_pauseparam *pause)
{
        struct rtl8127_private *tp = netdev_priv(dev);
        enum rtl8127_fc_mode newfc;

        if (pause->tx_pause || pause->rx_pause)
                newfc = rtl8127_fc_full;
        else
                newfc = rtl8127_fc_none;

        if (tp->fcpause != newfc) {
                tp->fcpause = newfc;

                rtl8127_set_speed(dev, tp->autoneg, tp->speed, tp->duplex, tp->advertising);
        }

        return 0;

}

static u32
rtl8127_get_msglevel(struct net_device *dev)
{
        struct rtl8127_private *tp = netdev_priv(dev);

        return tp->msg_enable;
}

static void
rtl8127_set_msglevel(struct net_device *dev,
                     u32 value)
{
        struct rtl8127_private *tp = netdev_priv(dev);

        tp->msg_enable = value;
}

static const char rtl8127_gstrings[][ETH_GSTRING_LEN] = {
        /* legacy */
        "tx_packets",
        "rx_packets",
        "tx_errors",
        "rx_errors",
        "rx_missed",
        "align_errors",
        "tx_single_collisions",
        "tx_multi_collisions",
        "unicast",
        "broadcast",
        "multicast",
        "tx_aborted",
        "tx_underrun",

        /* extended */
        "tx_octets",
        "rx_octets",
        "rx_multicast64",
        "tx_unicast64",
        "tx_broadcast64",
        "tx_multicast64",
        "tx_pause_on",
        "tx_pause_off",
        "tx_pause_all",
        "tx_deferred",
        "tx_late_collision",
        "tx_all_collision",
        "tx_aborted32",
        "align_errors32",
        "rx_frame_too_long",
        "rx_runt",
        "rx_pause_on",
        "rx_pause_off",
        "rx_pause_all",
        "rx_unknown_opcode",
        "rx_mac_error",
        "tx_underrun32",
        "rx_mac_missed",
        "rx_tcam_dropped",
        "tdu",
        "rdu",
};

static int rtl8127_get_sset_count(struct net_device *dev, int sset)
{
        switch (sset) {
        case ETH_SS_STATS:
                return ARRAY_SIZE(rtl8127_gstrings);
        default:
                return -EOPNOTSUPP;
        }
}

static void
rtl8127_set_ring_size(struct rtl8127_private *tp, u32 rx, u32 tx)
{
        int i;

        for (i = 0; i < R8127_MAX_RX_QUEUES; i++)
                tp->rx_ring[i].num_rx_desc = rx;

        for (i = 0; i < R8127_MAX_TX_QUEUES; i++)
                tp->tx_ring[i].num_tx_desc = tx;
}

static void rtl8127_get_ringparam(struct net_device *dev,
                                  struct ethtool_ringparam *ring,
                                  struct kernel_ethtool_ringparam *kernel_ring,
                                  struct netlink_ext_ack *extack)
{
        struct rtl8127_private *tp = netdev_priv(dev);

        ring->rx_max_pending = MAX_NUM_TX_DESC;
        ring->tx_max_pending = MAX_NUM_RX_DESC;
        ring->rx_pending = tp->rx_ring[0].num_rx_desc;
        ring->tx_pending = tp->tx_ring[0].num_tx_desc;
}

static int rtl8127_set_ringparam(struct net_device *dev,
                                 struct ethtool_ringparam *ring,
                                 struct kernel_ethtool_ringparam *kernel_ring,
                                 struct netlink_ext_ack *extack)
{
        struct rtl8127_private *tp = netdev_priv(dev);
        u32 new_rx_count, new_tx_count;
        int rc = 0;

        if ((ring->rx_mini_pending) || (ring->rx_jumbo_pending))
                return -EINVAL;

        new_tx_count = clamp_t(u32, ring->tx_pending,
                               MIN_NUM_TX_DESC, MAX_NUM_TX_DESC);

        new_rx_count = clamp_t(u32, ring->rx_pending,
                               MIN_NUM_RX_DESC, MAX_NUM_RX_DESC);

        if ((new_rx_count == tp->rx_ring[0].num_rx_desc) &&
            (new_tx_count == tp->tx_ring[0].num_tx_desc)) {
                /* nothing to do */
                return 0;
        }

        if (netif_running(dev)) {
                rtl8127_wait_for_quiescence(dev);
                rtl8127_close(dev);
        }

        rtl8127_set_ring_size(tp, new_rx_count, new_tx_count);

        if (netif_running(dev))
                rc = rtl8127_open(dev);

        return rc;
}

static void
rtl8127_get_ethtool_stats(struct net_device *dev,
                          struct ethtool_stats *stats,
                          u64 *data)
{
        struct rtl8127_private *tp = netdev_priv(dev);
        struct rtl8127_counters *counters;
        dma_addr_t paddr;

        ASSERT_RTNL();

        counters = tp->tally_vaddr;
        paddr = tp->tally_paddr;
        if (!counters)
                return;

        rtl8127_dump_tally_counter(tp, paddr);

        data[0] = le64_to_cpu(counters->tx_packets);
        data[1] = le64_to_cpu(counters->rx_packets);
        data[2] = le64_to_cpu(counters->tx_errors);
        data[3] = le32_to_cpu(counters->rx_errors);
        data[4] = le16_to_cpu(counters->rx_missed);
        data[5] = le16_to_cpu(counters->align_errors);
        data[6] = le32_to_cpu(counters->tx_one_collision);
        data[7] = le32_to_cpu(counters->tx_multi_collision);
        data[8] = le64_to_cpu(counters->rx_unicast);
        data[9] = le64_to_cpu(counters->rx_broadcast);
        data[10] = le32_to_cpu(counters->rx_multicast);
        data[11] = le16_to_cpu(counters->tx_aborted);
        data[12] = le16_to_cpu(counters->tx_underrun);

        data[13] = le64_to_cpu(counters->tx_octets);
        data[14] = le64_to_cpu(counters->rx_octets);
        data[15] = le64_to_cpu(counters->rx_multicast64);
        data[16] = le64_to_cpu(counters->tx_unicast64);
        data[17] = le64_to_cpu(counters->tx_broadcast64);
        data[18] = le64_to_cpu(counters->tx_multicast64);
        data[19] = le32_to_cpu(counters->tx_pause_on);
        data[20] = le32_to_cpu(counters->tx_pause_off);
        data[21] = le32_to_cpu(counters->tx_pause_all);
        data[22] = le32_to_cpu(counters->tx_deferred);
        data[23] = le32_to_cpu(counters->tx_late_collision);
        data[24] = le32_to_cpu(counters->tx_all_collision);
        data[25] = le32_to_cpu(counters->tx_aborted32);
        data[26] = le32_to_cpu(counters->align_errors32);
        data[27] = le32_to_cpu(counters->rx_frame_too_long);
        data[28] = le32_to_cpu(counters->rx_runt);
        data[29] = le32_to_cpu(counters->rx_pause_on);
        data[30] = le32_to_cpu(counters->rx_pause_off);
        data[31] = le32_to_cpu(counters->rx_pause_all);
        data[32] = le32_to_cpu(counters->rx_unknown_opcode);
        data[33] = le32_to_cpu(counters->rx_mac_error);
        data[34] = le32_to_cpu(counters->tx_underrun32);
        data[35] = le32_to_cpu(counters->rx_mac_missed);
        data[36] = le32_to_cpu(counters->rx_tcam_dropped);
        data[37] = le32_to_cpu(counters->tdu);
        data[38] = le32_to_cpu(counters->rdu);
}

static void
rtl8127_get_strings(struct net_device *dev,
                    u32 stringset,
                    u8 *data)
{
        switch (stringset) {
        case ETH_SS_STATS:
                memcpy(data, rtl8127_gstrings, sizeof(rtl8127_gstrings));
                break;
        }
}

static int rtl_get_eeprom_len(struct net_device *dev)
{
        struct rtl8127_private *tp = netdev_priv(dev);

        return tp->eeprom_len;
}

static int rtl_get_eeprom(struct net_device *dev, struct ethtool_eeprom *eeprom, u8 *buf)
{
        struct rtl8127_private *tp = netdev_priv(dev);
        int i,j,ret;
        int start_w, end_w;
        int VPD_addr, VPD_data;
        u32 *eeprom_buff;
        u16 tmp;

        if (tp->eeprom_type == EEPROM_TYPE_NONE) {
                dev_printk(KERN_DEBUG, tp_to_dev(tp), "Detect none EEPROM\n");
                return -EOPNOTSUPP;
        } else if (eeprom->len == 0 || (eeprom->offset+eeprom->len) > tp->eeprom_len) {
                dev_printk(KERN_DEBUG, tp_to_dev(tp), "Invalid parameter\n");
                return -EINVAL;
        }

        VPD_addr = 0xD2;
        VPD_data = 0xD4;

        start_w = eeprom->offset >> 2;
        end_w = (eeprom->offset + eeprom->len - 1) >> 2;

        eeprom_buff = kmalloc(sizeof(u32)*(end_w - start_w + 1), GFP_KERNEL);
        if (!eeprom_buff)
                return -ENOMEM;

        rtl8127_enable_cfg9346_write(tp);
        ret = -EFAULT;
        for (i=start_w; i<=end_w; i++) {
                pci_write_config_word(tp->pci_dev, VPD_addr, (u16)i*4);
                ret = -EFAULT;
                for (j = 0; j < 10; j++) {
                        udelay(400);
                        pci_read_config_word(tp->pci_dev, VPD_addr, &tmp);
                        if (tmp&0x8000) {
                                ret = 0;
                                break;
                        }
                }

                if (ret)
                        break;

                pci_read_config_dword(tp->pci_dev, VPD_data, &eeprom_buff[i-start_w]);
        }
        rtl8127_disable_cfg9346_write(tp);

        if (!ret)
                memcpy(buf, (u8 *)eeprom_buff + (eeprom->offset & 3), eeprom->len);

        kfree(eeprom_buff);

        return ret;
}

#undef ethtool_op_get_link
#define ethtool_op_get_link _kc_ethtool_op_get_link
static u32 _kc_ethtool_op_get_link(struct net_device *dev)
{
        return netif_carrier_ok(dev) ? 1 : 0;
}

static void
rtl8127_set_eee_lpi_timer(struct rtl8127_private *tp)
{
        u16 dev_lpi_timer;

        dev_lpi_timer = tp->eee.tx_lpi_timer;

        RTL_W16(tp, EEE_TXIDLE_TIMER_8125, dev_lpi_timer);
}

static bool rtl8127_is_adv_eee_enabled(struct rtl8127_private *tp)
{
        if (rtl8127_mdio_direct_read_phy_ocp(tp, 0xA430) & BIT_15)
                return true;
        else
                return false;
}

static void _rtl8127_disable_adv_eee(struct rtl8127_private *tp)
{
        bool lock;

        if (rtl8127_is_adv_eee_enabled(tp))
                lock = true;
        else
                lock = false;

        if (lock)
                rtl8127_set_phy_mcu_patch_request(tp);

        rtl8127_clear_mac_ocp_bit(tp, 0xE052, BIT_0);
        rtl8127_clear_eth_phy_ocp_bit(tp, 0xA442, BIT_12 | BIT_13);
        rtl8127_clear_eth_phy_ocp_bit(tp, 0xA430, BIT_15);

        if (lock)
                rtl8127_clear_phy_mcu_patch_request(tp);
}

static void rtl8127_disable_adv_eee(struct rtl8127_private *tp)
{
        rtl8127_oob_mutex_lock(tp);

        _rtl8127_disable_adv_eee(tp);

        rtl8127_oob_mutex_unlock(tp);
}

static int rtl8127_enable_eee(struct rtl8127_private *tp)
{
        struct ethtool_keee *eee = &tp->eee;
        u16 eee_adv_cap1_t = rtl8127_ethtool_adv_to_mmd_eee_adv_cap1_t(eee->advertised);
        u16 eee_adv_cap2_t = rtl8127_ethtool_adv_to_mmd_eee_adv_cap2_t(eee->advertised);

        if (!HW_HAS_WRITE_PHY_MCU_RAM_CODE(tp))
                return 0;

        rtl8127_set_mac_ocp_bit(tp, 0xE040, (BIT_1|BIT_0));

        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xA5D0,
                                              MDIO_EEE_100TX | MDIO_EEE_1000T | MDIO_EEE_10GT,
                                              eee_adv_cap1_t);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xA6D4,
                                              MDIO_EEE_2_5GT | MDIO_EEE_5GT,
                                              eee_adv_cap2_t);
        rtl8127_clear_eth_phy_ocp_bit(tp, 0xA6D8, BIT_4);
        rtl8127_clear_eth_phy_ocp_bit(tp, 0xA428, BIT_7);
        rtl8127_clear_eth_phy_ocp_bit(tp, 0xA4A2, BIT_9);

        /*Advanced EEE*/
        rtl8127_disable_adv_eee(tp);

        return 0;
}

static int rtl8127_disable_eee(struct rtl8127_private *tp)
{
        rtl8127_clear_mac_ocp_bit(tp, 0xE040, (BIT_1|BIT_0));

        rtl8127_clear_eth_phy_ocp_bit(tp, 0xA5D0,
                                      (MDIO_EEE_100TX | MDIO_EEE_1000T | MDIO_EEE_10GT));
        rtl8127_clear_eth_phy_ocp_bit(tp, 0xA6D4,
                                      (MDIO_EEE_2_5GT | MDIO_EEE_5GT));

        rtl8127_clear_eth_phy_ocp_bit(tp, 0xA6D8, BIT_4);
        rtl8127_clear_eth_phy_ocp_bit(tp, 0xA428, BIT_7);
        rtl8127_clear_eth_phy_ocp_bit(tp, 0xA4A2, BIT_9);

        /*Advanced EEE*/
        rtl8127_disable_adv_eee(tp);

        return 0;
}

static int rtl_nway_reset(struct net_device *dev)
{
        struct rtl8127_private *tp = netdev_priv(dev);
        int ret, bmcr;

        if (unlikely(tp->rtk_enable_diag))
                return -EBUSY;

        /* if autoneg is off, it's an error */
        rtl8127_mdio_write(tp, 0x1F, 0x0000);
        bmcr = rtl8127_mdio_read(tp, MII_BMCR);

        if (bmcr & BMCR_ANENABLE) {
                bmcr |= BMCR_ANRESTART;
                rtl8127_mdio_write(tp, MII_BMCR, bmcr);
                ret = 0;
        } else {
                ret = -EINVAL;
        }

        return ret;
}

static u32
rtl8127_device_lpi_t_to_ethtool_lpi_t(struct rtl8127_private *tp , u32 lpi_timer)
{
        u32 to_us;
        u32 status;

        to_us = lpi_timer * 80;
        status = rtl8127_get_phy_status(tp);
        if (status & LinkStatus) {
                /*link on*/
                if (HW_SUPP_PHY_LINK_SPEED_10000M(tp)) {
                        //5G : lpi_timer * 12.8ns
                        //2.5G : lpi_timer * 25.6ns
                        //Giga: lpi_timer * 8ns
                        //100M : lpi_timer * 80ns
                        if (status & (_10000bpsF))
                                to_us = (lpi_timer * 128) / 10;
                        else if (status & (_5000bpsF))
                                to_us = (lpi_timer * 128) / 10;
                        else if (status & _2500bpsF)
                                to_us = (lpi_timer * 256) / 10;
                        else if (status & _1000bpsF)
                                to_us = lpi_timer * 8;
                } else if (HW_SUPP_PHY_LINK_SPEED_5000M(tp)) {
                        //5G : lpi_timer * 12.8ns
                        //2.5G : lpi_timer * 25.6ns
                        //Giga: lpi_timer * 8ns
                        //100M : lpi_timer * 80ns
                        if (status & (_5000bpsF))
                                to_us = (lpi_timer * 128) / 10;
                        else if (status & _2500bpsF)
                                to_us = (lpi_timer * 256) / 10;
                        else if (status & _1000bpsF)
                                to_us = lpi_timer * 8;
                } else {
                        //2.5G : lpi_timer * 3.2ns
                        //Giga: lpi_timer * 8ns
                        //100M : lpi_timer * 80ns
                        if (status & _2500bpsF)
                                to_us = (lpi_timer * 32) / 10;
                        else if (status & _1000bpsF)
                                to_us = lpi_timer * 8;
                }
        }

        //ns to us
        to_us /= 1000;

        return to_us;
}

static void
rtl8127_adv_to_linkmode(unsigned long *mode, u64 adv)
{
        linkmode_zero(mode);

        if (adv & ADVERTISED_10baseT_Half)
                linkmode_set_bit(ETHTOOL_LINK_MODE_10baseT_Half_BIT, mode);
        if (adv & ADVERTISED_10baseT_Full)
                linkmode_set_bit(ETHTOOL_LINK_MODE_10baseT_Full_BIT, mode);
        if (adv & ADVERTISED_100baseT_Half)
                linkmode_set_bit(ETHTOOL_LINK_MODE_100baseT_Half_BIT, mode);
        if (adv & ADVERTISED_100baseT_Full)
                linkmode_set_bit(ETHTOOL_LINK_MODE_100baseT_Full_BIT, mode);
        if (adv & ADVERTISED_1000baseT_Half)
                linkmode_set_bit(ETHTOOL_LINK_MODE_1000baseT_Half_BIT, mode);
        if (adv & ADVERTISED_1000baseT_Full)
                linkmode_set_bit(ETHTOOL_LINK_MODE_1000baseT_Full_BIT, mode);
        if (adv & ADVERTISED_2500baseX_Full)
                linkmode_set_bit(ETHTOOL_LINK_MODE_2500baseT_Full_BIT, mode);
        if (adv & RTK_ADVERTISED_5000baseX_Full)
                linkmode_set_bit(ETHTOOL_LINK_MODE_5000baseT_Full_BIT, mode);
        if (adv & ADVERTISED_10000baseT_Full)
                linkmode_set_bit(ETHTOOL_LINK_MODE_10000baseT_Full_BIT, mode);
}

static int
rtl_ethtool_get_eee(struct net_device *net, struct ethtool_keee *edata)
{
        __ETHTOOL_DECLARE_LINK_MODE_MASK(common);
        struct rtl8127_private *tp = netdev_priv(net);
        struct ethtool_keee *eee = &tp->eee;
        unsigned long flags;
        u32 tx_lpi_timer;
        u16 val;

        if (unlikely(tp->rtk_enable_diag))
                return -EBUSY;

        r8127_spin_lock(&tp->phy_lock, flags);

        /* Get LP advertisement EEE */
        val = rtl8127_mdio_direct_read_phy_ocp(tp, 0xA5D2);
        mii_eee_cap1_mod_linkmode_t(edata->lp_advertised, val);
        val = rtl8127_mdio_direct_read_phy_ocp(tp, 0xA6D0);
        mii_eee_cap2_mod_linkmode_sup_t(edata->lp_advertised, val);

        r8127_spin_unlock(&tp->phy_lock, flags);

        /* Get EEE Tx LPI timer*/
        tx_lpi_timer = rtl8127_device_lpi_t_to_ethtool_lpi_t(tp, eee->tx_lpi_timer);

        val = rtl8127_mac_ocp_read(tp, 0xE040);
        val &= BIT_1 | BIT_0;

        edata->eee_enabled = !!val;
        linkmode_copy(edata->supported, eee->supported);
        linkmode_copy(edata->advertised, eee->advertised);
        edata->tx_lpi_enabled = edata->eee_enabled;
        edata->tx_lpi_timer = tx_lpi_timer;
        linkmode_and(common, edata->advertised, edata->lp_advertised);
        edata->eee_active = !linkmode_empty(common);

        return 0;
}

static int
rtl_ethtool_set_eee(struct net_device *net, struct ethtool_keee *edata)
{
        __ETHTOOL_DECLARE_LINK_MODE_MASK(advertising);
        __ETHTOOL_DECLARE_LINK_MODE_MASK(tmp);
        struct rtl8127_private *tp = netdev_priv(net);
        struct ethtool_keee *eee = &tp->eee;
        unsigned long flags;
        int rc = 0;

        r8127_spin_lock(&tp->phy_lock, flags);

        if (!HW_HAS_WRITE_PHY_MCU_RAM_CODE(tp) ||
            tp->DASH) {
                rc = -EOPNOTSUPP;
                goto out;
        }

        if (unlikely(tp->rtk_enable_diag)) {
                dev_printk(KERN_WARNING, tp_to_dev(tp), "Diag Enabled\n");
                rc = -EBUSY;
                goto out;
        }

        if (tp->autoneg != AUTONEG_ENABLE) {
                dev_printk(KERN_WARNING, tp_to_dev(tp), "EEE requires autoneg\n");
                rc = -EINVAL;
                goto out;
        }

        rtl8127_adv_to_linkmode(advertising, tp->advertising);
        if (linkmode_empty(edata->advertised)) {
                linkmode_and(edata->advertised, advertising, eee->supported);
        } else if (linkmode_andnot(tmp, edata->advertised, advertising)) {
                dev_printk(KERN_WARNING, tp_to_dev(tp), "EEE advertised must be a subset of autoneg advertised speeds\n");
                rc = -EINVAL;
                goto out;
        }

        if (linkmode_andnot(tmp, edata->advertised, eee->supported)) {
                dev_printk(KERN_WARNING, tp_to_dev(tp), "EEE advertised must be a subset of support \n");
                rc = -EINVAL;
                goto out;
        }

        linkmode_copy(eee->advertised, edata->advertised);
        eee->eee_enabled = edata->eee_enabled;

        if (eee->eee_enabled)
                rtl8127_enable_eee(tp);
        else
                rtl8127_disable_eee(tp);

        rtl_nway_reset(net);

out:
        r8127_spin_unlock(&tp->phy_lock, flags);

        return rc;
}

static void rtl8127_get_channels(struct net_device *dev,
                                 struct ethtool_channels *channel)
{
        struct rtl8127_private *tp = netdev_priv(dev);

        channel->max_rx = tp->HwSuppNumRxQueues;
        channel->max_tx = tp->HwSuppNumTxQueues;
        channel->rx_count = tp->num_rx_rings;
        channel->tx_count = tp->num_tx_rings;
}

static const struct ethtool_ops rtl8127_ethtool_ops = {
        .get_drvinfo        = rtl8127_get_drvinfo,
        .get_regs_len       = rtl8127_get_regs_len,
        .get_link       = ethtool_op_get_link,
        .get_ringparam      = rtl8127_get_ringparam,
        .set_ringparam      = rtl8127_set_ringparam,
        .get_link_ksettings       = rtl8127_get_settings,
        .set_link_ksettings       = rtl8127_set_settings,
        .get_pauseparam     = rtl8127_get_pauseparam,
        .set_pauseparam     = rtl8127_set_pauseparam,
        .get_msglevel       = rtl8127_get_msglevel,
        .set_msglevel       = rtl8127_set_msglevel,
        .get_regs       = rtl8127_get_regs,
        .get_wol        = rtl8127_get_wol,
        .set_wol        = rtl8127_set_wol,
        .get_strings        = rtl8127_get_strings,
        .get_sset_count     = rtl8127_get_sset_count,
        .get_ethtool_stats  = rtl8127_get_ethtool_stats,
        .get_eeprom     = rtl_get_eeprom,
        .get_eeprom_len     = rtl_get_eeprom_len,
#ifdef ENABLE_RSS_SUPPORT
        .get_rxnfc		= rtl8127_get_rxnfc,
        .set_rxnfc		= rtl8127_set_rxnfc,
        .get_rxfh_indir_size	= rtl8127_rss_indir_size,
        .get_rxfh_key_size	= rtl8127_get_rxfh_key_size,
        .get_rxfh		= rtl8127_get_rxfh,
        .set_rxfh		= rtl8127_set_rxfh,
#endif //ENABLE_RSS_SUPPORT
#ifdef ENABLE_PTP_SUPPORT
        .get_ts_info        = rtl8127_get_ts_info,
#else
        .get_ts_info        = ethtool_op_get_ts_info,
#endif //ENABLE_PTP_SUPPORT
        .get_eee = rtl_ethtool_get_eee,
        .set_eee = rtl_ethtool_set_eee,
        .get_channels		= rtl8127_get_channels,
        .nway_reset = rtl_nway_reset,

};

static void rtl8127_get_mac_version(struct rtl8127_private *tp)
{
        u32 reg,val32;
        u32 ICVerID;

        val32 = RTL_R32(tp, TxConfig);
        reg = val32 & 0x7c800000;
        ICVerID = val32 & 0x00700000;

        switch (reg) {
        case 0x6C800000:
                if (ICVerID == 0x00000000) {
                        tp->mcfg = CFG_METHOD_1;
                } else if (ICVerID == 0x100000) {
                        tp->mcfg = CFG_METHOD_2;
                } else {
                        tp->mcfg = CFG_METHOD_2;
                        tp->HwIcVerUnknown = TRUE;
                }

                tp->efuse_ver = EFUSE_SUPPORT_V4;
                break;
        default:
                printk("unknown chip version (%x)\n",reg);
                tp->mcfg = CFG_METHOD_DEFAULT;
                tp->HwIcVerUnknown = TRUE;
                tp->efuse_ver = EFUSE_NOT_SUPPORT;
                break;
        }
}

static void
rtl8127_print_mac_version(struct rtl8127_private *tp)
{
        int i;
        for (i = ARRAY_SIZE(rtl_chip_info) - 1; i >= 0; i--) {
                if (tp->mcfg == rtl_chip_info[i].mcfg) {
                        dprintk("Realtek %s Ethernet controller mcfg = %04d\n",
                                MODULENAME, rtl_chip_info[i].mcfg);
                        return;
                }
        }

        dprintk("mac_version == Unknown\n");
}

static void
rtl8127_tally_counter_addr_fill(struct rtl8127_private *tp)
{
        if (!tp->tally_paddr)
                return;

        RTL_W32(tp, CounterAddrHigh, (u64)tp->tally_paddr >> 32);
        RTL_W32(tp, CounterAddrLow, (u64)tp->tally_paddr & (DMA_BIT_MASK(32)));
}

static void
rtl8127_tally_counter_clear(struct rtl8127_private *tp)
{
        if (!tp->tally_paddr)
                return;

        RTL_W32(tp, CounterAddrHigh, (u64)tp->tally_paddr >> 32);
        RTL_W32(tp, CounterAddrLow, ((u64)tp->tally_paddr & (DMA_BIT_MASK(32))) | CounterReset);
}

static void
rtl8127_clear_phy_ups_reg(struct net_device *dev)
{
        struct rtl8127_private *tp = netdev_priv(dev);

        rtl8127_clear_eth_phy_ocp_bit(tp, 0xA466, BIT_0);
        rtl8127_clear_eth_phy_ocp_bit(tp, 0xA468, BIT_3 | BIT_1);
}

static int
rtl8127_is_ups_resume(struct net_device *dev)
{
        struct rtl8127_private *tp = netdev_priv(dev);

        return (rtl8127_mac_ocp_read(tp, 0xD42C) & BIT_8);
}

static void
rtl8127_clear_ups_resume_bit(struct net_device *dev)
{
        struct rtl8127_private *tp = netdev_priv(dev);

        rtl8127_clear_mac_ocp_bit(tp, 0xD42C, BIT_8);
}

static u8
rtl8127_get_phy_state(struct rtl8127_private *tp)
{
        return (rtl8127_mdio_direct_read_phy_ocp(tp, 0xA420) & 0x7);
}

static void
rtl8127_wait_phy_ups_resume(struct net_device *dev, u16 PhyState)
{
        struct rtl8127_private *tp = netdev_priv(dev);
        int i;

        for (i=0; i< 100; i++) {
                if (rtl8127_get_phy_state(tp) == PhyState)
                        break;
                else
                        mdelay(1);
        }

        WARN_ON_ONCE(i == 100);
}

static void
rtl8127_set_mcu_d3_stack(struct rtl8127_private *tp)
{
        switch (tp->mcfg) {
        case CFG_METHOD_2:
                rtl8127_mac_ocp_write(tp, 0xD018, 0xC896);
                rtl8127_mac_ocp_write(tp, 0xC896, 0x661A);
                break;
        default:
                return;
        }
}

static void
_rtl8127_enable_now_is_oob(struct rtl8127_private *tp)
{
        if (tp->HwSuppNowIsOobVer == 1)
                RTL_W8(tp, MCUCmd_reg, RTL_R8(tp, MCUCmd_reg) | Now_is_oob);
}

void
rtl8127_enable_now_is_oob(struct rtl8127_private *tp)
{
        rtl8127_set_mcu_d3_stack(tp);
        _rtl8127_enable_now_is_oob(tp);
}

void
rtl8127_disable_now_is_oob(struct rtl8127_private *tp)
{
        if (tp->HwSuppNowIsOobVer == 1)
                RTL_W8(tp, MCUCmd_reg, RTL_R8(tp, MCUCmd_reg) & ~Now_is_oob);
}

static void
rtl8127_exit_oob(struct net_device *dev)
{
        struct rtl8127_private *tp = netdev_priv(dev);
        u16 data16;

        rtl8127_disable_rx_packet_filter(tp);

        if (HW_DASH_SUPPORT_DASH(tp))
                rtl8127_driver_start(tp);

#ifdef ENABLE_REALWOW_SUPPORT
        rtl8127_realwow_hw_init(dev);
#else
        rtl8127_mac_ocp_write(tp, 0xC0BC, 0x00FF);
#endif //ENABLE_REALWOW_SUPPORT

        rtl8127_nic_reset(dev);

        rtl8127_disable_now_is_oob(tp);

        data16 = rtl8127_mac_ocp_read(tp, 0xE8DE) & ~BIT_14;
        rtl8127_mac_ocp_write(tp, 0xE8DE, data16);
        rtl8127_wait_ll_share_fifo_ready(dev);

        rtl8127_mac_ocp_write(tp, 0xC0AA, 0x07D0);
#ifdef ENABLE_LIB_SUPPORT
        rtl8127_mac_ocp_write(tp, 0xC0A6, 0x04E2);
#else
        rtl8127_mac_ocp_write(tp, 0xC0A6, 0x01B5);
#endif
        rtl8127_mac_ocp_write(tp, 0xC01E, 0x5555);

        rtl8127_wait_ll_share_fifo_ready(dev);

        //wait ups resume (phy state 2)
        if (rtl8127_is_ups_resume(dev)) {
                rtl8127_wait_phy_ups_resume(dev, 2);
                rtl8127_clear_ups_resume_bit(dev);
                rtl8127_clear_phy_ups_reg(dev);
        }
}

void
rtl8127_hw_disable_mac_mcu_bps(struct net_device *dev)
{
        u16 regAddr;

        struct rtl8127_private *tp = netdev_priv(dev);

        rtl8127_enable_aspm_clkreq_lock(tp, 0);

        rtl8127_mac_ocp_write(tp, 0xFC48, 0x0000);

        for (regAddr = 0xFC28; regAddr < 0xFC48; regAddr += 2) {
                rtl8127_mac_ocp_write(tp, regAddr, 0x0000);
        }

        fsleep(3000);

        rtl8127_mac_ocp_write(tp, 0xFC26, 0x0000);
}

static void
rtl8127_hw_init(struct net_device *dev)
{
        struct rtl8127_private *tp = netdev_priv(dev);
        u32 csi_tmp;

        rtl8127_enable_aspm_clkreq_lock(tp, 0);
        rtl8127_enable_force_clkreq(tp, 0);

        //Disable UPS
        rtl8127_mac_ocp_write(tp, 0xD40A, rtl8127_mac_ocp_read(tp, 0xD40A) & ~(BIT_4));

        //Set PCIE uncorrectable error status mask pcie 0x108
        csi_tmp = rtl8127_csi_read(tp, 0x108);
        csi_tmp |= BIT_20;
        rtl8127_csi_write(tp, 0x108, csi_tmp);

        rtl8127_enable_cfg9346_write(tp);
        rtl8127_disable_linkchg_wakeup(dev);
        rtl8127_disable_cfg9346_write(tp);
        rtl8127_disable_magic_packet(dev);
        rtl8127_disable_d0_speedup(tp);
        rtl8127_set_pci_pme(tp, 0);
        if (s0_magic_packet == 1)
                rtl8127_enable_magic_packet(dev);
}

static void
rtl8127_clear_ephy_ext_addr(struct rtl8127_private *tp)
{
        rtl8127_set_ephy_ext_addr(tp, 0x0000);
}

static void
rtl8127_hw_ephy_config_8127_1(struct rtl8127_private *tp)
{
        rtl8127_ephy_write(tp, 0x8088, 0x0064);
        rtl8127_ephy_write(tp, 0x8488, 0x0064);
        rtl8127_ephy_write(tp, 0x8888, 0x0064);
        rtl8127_ephy_write(tp, 0x8C88, 0x0064);
        rtl8127_ephy_write(tp, 0x8188, 0x0064);
        rtl8127_ephy_write(tp, 0x8588, 0x0064);
        rtl8127_ephy_write(tp, 0x8988, 0x0064);
        rtl8127_ephy_write(tp, 0x8D88, 0x0064);
        rtl8127_ephy_write(tp, 0x808C, 0x09B0);
        rtl8127_ephy_write(tp, 0x848C, 0x09B0);
        rtl8127_ephy_write(tp, 0x888C, 0x0F90);
        rtl8127_ephy_write(tp, 0x8C8C, 0x0F90);
        rtl8127_ephy_write(tp, 0x818C, 0x09B0);
        rtl8127_ephy_write(tp, 0x858C, 0x09B0);
        rtl8127_ephy_write(tp, 0x898C, 0x0F90);
        rtl8127_ephy_write(tp, 0x8D8C, 0x0F90);
        rtl8127_ephy_write(tp, 0x808A, 0x09B8);
        rtl8127_ephy_write(tp, 0x848A, 0x09B8);
        rtl8127_ephy_write(tp, 0x888A, 0x0F98);
        rtl8127_ephy_write(tp, 0x8C8A, 0x0F98);
        rtl8127_ephy_write(tp, 0x818A, 0x09B8);
        rtl8127_ephy_write(tp, 0x858A, 0x09B8);
        rtl8127_ephy_write(tp, 0x898A, 0x0F98);
        rtl8127_ephy_write(tp, 0x8D8A, 0x0F98);
        rtl8127_ephy_write(tp, 0x9020, 0x0080);
        rtl8127_ephy_write(tp, 0x9420, 0x0080);
        rtl8127_ephy_write(tp, 0x9820, 0x0080);
        rtl8127_ephy_write(tp, 0x9C20, 0x0080);
        rtl8127_ephy_write(tp, 0x901E, 0x0190);
        rtl8127_ephy_write(tp, 0x941E, 0x0190);
        rtl8127_ephy_write(tp, 0x981E, 0x0140);
        rtl8127_ephy_write(tp, 0x9C1E, 0x0140);
        rtl8127_ephy_write(tp, 0x901C, 0x0190);
        rtl8127_ephy_write(tp, 0x941C, 0x0190);
        rtl8127_ephy_write(tp, 0x981C, 0x0140);
        rtl8127_ephy_write(tp, 0x9C1C, 0x0140);

        /* Clear extended address */
        rtl8127_clear_ephy_ext_addr(tp);
}

static void
rtl8127_hw_ephy_config(struct net_device *dev)
{
        struct rtl8127_private *tp = netdev_priv(dev);

        switch (tp->mcfg) {
        case CFG_METHOD_2:
                rtl8127_hw_ephy_config_8127_1(tp);
                break;
        default:
                /* nothing to do */
                break;
        }
}

static u16
rtl8127_get_hw_phy_mcu_code_ver(struct rtl8127_private *tp)
{
        u16 hw_ram_code_ver;

        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA436, 0x801E);
        hw_ram_code_ver = rtl8127_mdio_direct_read_phy_ocp(tp, 0xA438);

        return hw_ram_code_ver;
}

static int
rtl8127_check_hw_phy_mcu_code_ver(struct net_device *dev)
{
        struct rtl8127_private *tp = netdev_priv(dev);
        int ram_code_ver_match = 0;

        tp->hw_ram_code_ver = rtl8127_get_hw_phy_mcu_code_ver(tp);

        if (tp->hw_ram_code_ver == tp->sw_ram_code_ver) {
                ram_code_ver_match = 1;
                tp->HwHasWrRamCodeToMicroP = TRUE;
        }

        return ram_code_ver_match;
}

bool
rtl8127_set_phy_mcu_patch_request(struct rtl8127_private *tp)
{
        u16 gphy_val;
        u16 WaitCount;
        bool bSuccess = TRUE;

        rtl8127_set_eth_phy_ocp_bit(tp, 0xB820, BIT_4);

        WaitCount = 0;
        do {
                gphy_val = rtl8127_mdio_direct_read_phy_ocp(tp, 0xB800);
                udelay(100);
                WaitCount++;
        } while (!(gphy_val & BIT_6) && (WaitCount < 1000));

        if (!(gphy_val & BIT_6) && (WaitCount == 1000))
                bSuccess = FALSE;

        if (!bSuccess)
                dprintk("rtl8127_set_phy_mcu_patch_request fail.\n");

        return bSuccess;
}

bool
rtl8127_clear_phy_mcu_patch_request(struct rtl8127_private *tp)
{
        u16 gphy_val;
        u16 WaitCount;
        bool bSuccess = TRUE;

        rtl8127_clear_eth_phy_ocp_bit(tp, 0xB820, BIT_4);

        WaitCount = 0;
        do {
                gphy_val = rtl8127_mdio_direct_read_phy_ocp(tp, 0xB800);
                udelay(100);
                WaitCount++;
        } while ((gphy_val & BIT_6) && (WaitCount < 1000));

        if ((gphy_val & BIT_6) && (WaitCount == 1000))
                bSuccess = FALSE;

        if (!bSuccess)
                dprintk("rtl8127_clear_phy_mcu_patch_request fail.\n");

        return bSuccess;
}

static void
rtl8127_enable_phy_aldps(struct rtl8127_private *tp)
{
        //enable aldps
        //GPHY OCP 0xA430 bit[2] = 0x1 (en_aldps)
        rtl8127_set_eth_phy_ocp_bit(tp, 0xA430, BIT_2);
}

static void
rtl8127_tgphy_irq_mask_and_ack(struct rtl8127_private *tp)
{
        switch (tp->mcfg) {
        case CFG_METHOD_2:
                rtl8127_mdio_direct_write_phy_ocp(tp, 0xA4D2, 0x0000);
                (void)rtl8127_mdio_direct_read_phy_ocp(tp, 0xA4D4);
                break;
        default:
                break;
        }
}

static void
rtl8127_hw_phy_config_8127a_tc_1(struct net_device *dev)
{
        struct rtl8127_private *tp = netdev_priv(dev);

        rtl8127_set_eth_phy_ocp_bit(tp, 0xA442, BIT_11);


        rtl8127_mdio_direct_write_phy_ocp(tp, 0xa436, 0x815E);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xa438,
                                              0xFF00,
                                              0x8600);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xa436, 0x8169);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xa438,
                                              0xFF00,
                                              0x8600);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xa436, 0x8174);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xa438,
                                              0xFF00,
                                              0xA100);

        rtl8127_mdio_direct_write_phy_ocp(tp, 0xa436, 0x83BF);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xa438,
                                              0xFF00,
                                              0x5A00);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xa436, 0x83C5);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xa438,
                                              0xFF00,
                                              0x5A00);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xa436, 0x83CB);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xa438,
                                              0xFF00,
                                              0x8B00);

        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8238);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xB87E,
                                              0xC000,
                                              0x4000);

        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x823A);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xB87E,
                                              0xFF00,
                                              0xA000);

        rtl8127_mdio_direct_write_phy_ocp(tp, 0xa436, 0x8148);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xa438,
                                              0xFF00,
                                              0x0100);


        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA436, 0x84AD);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xA438,
                                              0xFF00,
                                              0x0C00);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA436, 0x84B2);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xA438,
                                              0xFF00,
                                              0x0800);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA436, 0x84B7);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xA438,
                                              0xFF00,
                                              0x1400);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA436, 0x84BC);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0040);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA436, 0x84C0);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA438, 0x00D6);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA436, 0x84BE);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA438, 0x00A0);

        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA436, 0x84AE);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0C0C);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA436, 0x84B0);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0C0C);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xBD7A, 0xAAAA);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xBCE0, 0x6666);


        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x85FC);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0AAA);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x85FF);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0AAA);
        rtl8127_clear_eth_phy_ocp_bit(tp, 0xAC32, BIT_3);

        rtl8127_clear_eth_phy_ocp_bit(tp, 0xAC32, BIT_11);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xADDC,
                                              0x3FFF,
                                              0x2000);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8111);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xB87E,
                                              0x0F00,
                                              0x0F00);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x80E9);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xB87E,
                                              0x0F00,
                                              0x0F00);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xAEC4,
                                              0xFF00,
                                              0x4600);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xAC56,
                                              0x0007,
                                              0x0005);

        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x825B);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xB87E,
                                              0x0F00,
                                              0x0D00);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8283);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0334);

        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8289);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xB87E,
                                              0xFF00,
                                              0x5600);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xADB8, 0x0190);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xAE3A,
                                              0x00FF,
                                              0x0026);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xAE4A,
                                              0x0FF0,
                                              0x0150);

        rtl8127_set_eth_phy_ocp_bit(tp, 0xAEC2, BIT_12);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xAE22, 0x0352);

        rtl8127_mdio_direct_write_phy_ocp(tp, 0xAEC0, 0x00FA);

        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8188);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xB87E,
                                              0xFF00,
                                              0xF500);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8203);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xB87E,
                                              0xFF00,
                                              0xF500);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x827E);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xB87E,
                                              0xFF00,
                                              0xF500);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x81CB);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xB87E,
                                              0xFF00,
                                              0x1000);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8246);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xB87E,
                                              0xFF00,
                                              0x1000);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x818A);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xB87E,
                                              0xFF00,
                                              0xF500);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8205);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xB87E,
                                              0xFF00,
                                              0xF500);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8280);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xB87E,
                                              0xFF00,
                                              0xF500);


        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x83DD);
        rtl8127_set_eth_phy_ocp_bit(tp, 0xB87E, BIT_9);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x840B);
        rtl8127_set_eth_phy_ocp_bit(tp, 0xB87E, BIT_9);

        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA436, 0x83BC);
        rtl8127_set_eth_phy_ocp_bit(tp, 0xA438, BIT_10);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA436, 0x83BE);
        rtl8127_set_eth_phy_ocp_bit(tp, 0xA438, BIT_10);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA436, 0x83C0);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xA438,
                                              0x0700,
                                              0x0400);

        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA436, 0x83C2);
        rtl8127_set_eth_phy_ocp_bit(tp, 0xA438, BIT_10);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA436, 0x83C4);
        rtl8127_set_eth_phy_ocp_bit(tp, 0xA438, BIT_10);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA436, 0x83C6);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xA438,
                                              0x0700,
                                              0x0400);

        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA436, 0x84C2);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA438, 0xFEFF);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA436, 0x84C4);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0003);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA436, 0x84C6);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0116);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA436, 0x84C8);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xA438,
                                              0xFF00,
                                              0x6300);

        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x822A);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x4FFF);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x81AF);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x4067);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8134);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x5069);

        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x822C);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xB87E,
                                              0xFF00,
                                              0x1A00);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x81B1);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xB87E,
                                              0xFF00,
                                              0x3A00);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8136);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xB87E,
                                              0xFF00,
                                              0x5000);

        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x810E);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xB87E,
                                              0xF000,
                                              0x9000);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8114);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xB87E,
                                              0xF000,
                                              0x9000);
        rtl8127_clear_eth_phy_ocp_bit(tp, 0xB63C, BIT_9);


        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x80B4);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87E, 0xB63B);

        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x80E4);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xB87E,
                                              0x0F00,
                                              0x0700);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x80E5);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xB87E,
                                              0xF000,
                                              0x7000);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x80EA);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xB87E,
                                              0x0F00,
                                              0x0500);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x80EB);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xB87E,
                                              0xF000,
                                              0x5000);

        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8291);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0416);

        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8015);
        rtl8127_set_eth_phy_ocp_bit(tp, 0xB87E, BIT_11);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x895E);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x01A0);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8960);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x01A0);

        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x826B);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87E, 0xF0AF);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x81F0);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87E, 0xF0AF);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8175);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87E, 0xF0AF);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x826D);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xB87E,
                                              0x0F00,
                                              0x0100);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x81F2);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xB87E,
                                              0x0F00,
                                              0x0100);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8177);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xB87E,
                                              0x0F00,
                                              0x0100);

        rtl8127_clear_eth_phy_ocp_bit(tp, 0xAC1C, BIT_8 | BIT_7);
        rtl8127_clear_eth_phy_ocp_bit(tp, 0xAC1E, BIT_13 | BIT_12);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xAD96, 0xAAFF);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xAD98,
                                              0x00FF,
                                              0x00AA);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xAE38,
                                              0x3FFF,
                                              0x2554);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xAE3A,
                                              0xF000,
                                              0xA000);

        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8932);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xB87E,
                                              0x0F00,
                                              0x0900);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x892F);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xB87E,
                                              0x0F00,
                                              0x0900);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x892C);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xB87E,
                                              0x0F00,
                                              0x0900);

        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x85B6);
        rtl8127_set_eth_phy_ocp_bit(tp, 0xB87E, 0xFF00);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x85B4);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87E, 0xFFFF);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8905);
        rtl8127_clear_eth_phy_ocp_bit(tp, 0xB87E, 0xFF00);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8853);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xB87E,
                                              0xFF00,
                                              0x2800);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x884B);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xB87E,
                                              0xFF00,
                                              0x3F00);


        rtl8127_clear_eth_phy_ocp_bit(tp, 0xBDE6, 0x3FFF);
        rtl8127_clear_eth_phy_ocp_bit(tp, 0xBDE8, 0x3FFF);
        rtl8127_clear_eth_phy_ocp_bit(tp, 0xBF0E, 0x0003);


        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8156);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xA438,
                                              0xFF00,
                                              0x1600);

        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA436, 0x80AB);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xA438,
                                              0xFF00,
                                              0x7500);

        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA436, 0x80C3);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA438, 0x090D);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA436, 0x80C6);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xA438,
                                              0xFF00,
                                              0xC600);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA436, 0x80BF);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xA438,
                                              0xFF00,
                                              0x5500);

        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8096);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xA438,
                                              0xFF00,
                                              0x4500);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA436, 0x809D);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xA438,
                                              0xFF00,
                                              0x0200);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA436, 0x809B);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA438, 0xE50A);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8099);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA438, 0x9906);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA436, 0x831F);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xA438,
                                              0xFF00,
                                              0x5000);


        rtl8127_set_eth_phy_ocp_bit(tp, 0xB648, BIT_14);
        rtl8127_clear_eth_phy_ocp_bit(tp, 0xA4E0, BIT_15);


        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA436, 0x849A);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0004);


        rtl8127_clear_eth_phy_ocp_bit(tp, 0xAC1C, 0x0C00);
        rtl8127_set_eth_phy_ocp_bit(tp, 0xA42C, BIT_6);
        rtl8127_clear_eth_phy_ocp_bit(tp, 0xACBA, 0xFC00);

        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8122);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xB87E,
                                              0xF000,
                                              0xC000);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8123);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xB87E,
                                              0xF000,
                                              0xC000);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x80FA);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xB87E,
                                              0xF000,
                                              0x2000);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x825B);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xB87E,
                                              0xF000,
                                              0xB000);

        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x80D2);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xB87E,
                                              0xF000,
                                              0xD000);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x80D3);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xB87E,
                                              0xF000,
                                              0xD000);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x80C8);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xB87E,
                                              0x0F00,
                                              0x0200);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x80CA);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xB87E,
                                              0x0F00,
                                              0x0300);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x80E2);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xB87E,
                                              0xFF00,
                                              0x2300);

        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x80A9);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xB87E,
                                              0x0F00,
                                              0x0A00);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x80AA);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xB87E,
                                              0xF000,
                                              0x5000);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x80AB);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xB87E,
                                              0xF000,
                                              0xA000);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x805A);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xB87E,
                                              0xF000,
                                              0x2000);


        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8106);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x40CC);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x812C);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x40CC);

        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8096);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xB87E,
                                              0xFF00,
                                              0x7500);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x809C);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xB87E,
                                              0xFF00,
                                              0x6300);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x859E);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xB87E,
                                              0xFF00,
                                              0x1F00);


        if (aspm && HW_HAS_WRITE_PHY_MCU_RAM_CODE(tp))
                rtl8127_enable_phy_aldps(tp);
}

static void
rtl8127_hw_phy_config_8127a_1(struct net_device *dev)
{
        struct rtl8127_private *tp = netdev_priv(dev);

        rtl8127_tgphy_irq_mask_and_ack(tp);


        rtl8127_clear_eth_phy_ocp_bit(tp, 0xA442, BIT_11);


        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8415);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xA438,
                                              0xFF00,
                                              0x9300);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA436, 0x81A3);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xA438,
                                              0xFF00,
                                              0x0F00);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA436, 0x81AE);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xA438,
                                              0xFF00,
                                              0x0F00);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA436, 0x81B9);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xA438,
                                              0xFF00,
                                              0xB900);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x83B0);
        rtl8127_clear_eth_phy_ocp_bit(tp, 0xB87E, 0x0E00);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x83C5);
        rtl8127_clear_eth_phy_ocp_bit(tp, 0xB87E, 0x0E00);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x83DA);
        rtl8127_clear_eth_phy_ocp_bit(tp, 0xB87E, 0x0E00);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x83EF);
        rtl8127_clear_eth_phy_ocp_bit(tp, 0xB87E, 0x0E00);

        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xBF38,
                                              0x01F0,
                                              0x0160);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xBF3A,
                                              0x001F,
                                              0x0014);
        rtl8127_clear_eth_phy_ocp_bit(tp, 0xBF28, 0x6000);
        rtl8127_clear_eth_phy_ocp_bit(tp, 0xBF2C, 0xC000);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xBF28,
                                              0x1FFF,
                                              0x0187);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xBF2A,
                                              0x003F,
                                              0x0003);


        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8173);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8620);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8175);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8671);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA436, 0x817C);
        rtl8127_set_eth_phy_ocp_bit(tp, 0xA438, BIT_13);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8187);
        rtl8127_set_eth_phy_ocp_bit(tp, 0xA438, BIT_13);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8192);
        rtl8127_set_eth_phy_ocp_bit(tp, 0xA438, BIT_13);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA436, 0x819D);
        rtl8127_set_eth_phy_ocp_bit(tp, 0xA438, BIT_13);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA436, 0x81A8);
        rtl8127_clear_eth_phy_ocp_bit(tp, 0xA438, BIT_13);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA436, 0x81B3);
        rtl8127_clear_eth_phy_ocp_bit(tp, 0xA438, BIT_13);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA436, 0x81BE);
        rtl8127_set_eth_phy_ocp_bit(tp, 0xA438, BIT_13);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA436, 0x817D);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xA438,
                                              0xFF00,
                                              0xA600);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8188);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xA438,
                                              0xFF00,
                                              0xA600);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8193);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xA438,
                                              0xFF00,
                                              0xA600);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA436, 0x819E);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xA438,
                                              0xFF00,
                                              0xA600);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA436, 0x81A9);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xA438,
                                              0xFF00,
                                              0x1400);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA436, 0x81B4);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xA438,
                                              0xFF00,
                                              0x1400);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA436, 0x81BF);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xA438,
                                              0xFF00,
                                              0xA600);


        rtl8127_clear_eth_phy_ocp_bit(tp, 0xAEAA, (BIT_5 | BIT_3));

        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x84F0);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x201C);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x84F2);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x3117);

        rtl8127_mdio_direct_write_phy_ocp(tp, 0xAEC6, 0x0000);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xAE20, 0xFFFF);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xAECE, 0xFFFF);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xAED2, 0xFFFF);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xAEC8, 0x0000);
        rtl8127_clear_eth_phy_ocp_bit(tp, 0xAED0, BIT_0);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xADB8, 0x0150);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8197);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xB87E,
                                              0xFF00,
                                              0x5000);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8231);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xB87E,
                                              0xFF00,
                                              0x5000);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x82CB);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xB87E,
                                              0xFF00,
                                              0x5000);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x82CD);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xB87E,
                                              0xFF00,
                                              0x5700);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8233);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xB87E,
                                              0xFF00,
                                              0x5700);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8199);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xB87E,
                                              0xFF00,
                                              0x5700);

        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x815A);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0150);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x81F4);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0150);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x828E);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0150);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x81B1);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0000);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x824B);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0000);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x82E5);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0000);

        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x84F7);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xB87E,
                                              0xFF00,
                                              0x2800);
        rtl8127_set_eth_phy_ocp_bit(tp, 0xAEC2, BIT_12);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x81B3);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xB87E,
                                              0xFF00,
                                              0xAD00);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x824D);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xB87E,
                                              0xFF00,
                                              0xAD00);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x82E7);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xB87E,
                                              0xFF00,
                                              0xAD00);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xAE4E,
                                              0x000F,
                                              0x0001);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x82CE);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xB87E,
                                              0xF000,
                                              0x4000);


        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x84AC);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0000);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x84AE);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0000);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x84B0);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87E, 0xF818);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x84B2);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xB87E,
                                              0xFF00,
                                              0x6000);

        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8FFC);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x6008);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8FFE);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87E, 0xF450);

        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8015);
        rtl8127_set_eth_phy_ocp_bit(tp, 0xB87E, BIT_9);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8016);
        rtl8127_clear_eth_phy_ocp_bit(tp, 0xB87E, BIT_11);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8FE6);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xB87E,
                                              0xFF00,
                                              0x0800);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8FE4);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x2114);

        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8647);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87E, 0xA7B1);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8649);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87E, 0xBBCA);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x864B);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xB87E,
                                              0xFF00,
                                              0xDC00);

        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8154);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xB87E,
                                              0xC000,
                                              0x4000);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8158);
        rtl8127_clear_eth_phy_ocp_bit(tp, 0xB87E, 0xC000);

        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x826C);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87E, 0xFFFF);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x826E);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87E, 0xFFFF);

        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8872);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xB87E,
                                              0xFF00,
                                              0x0E00);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8012);
        rtl8127_set_eth_phy_ocp_bit(tp, 0xA438, BIT_11);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8012);
        rtl8127_set_eth_phy_ocp_bit(tp, 0xA438, BIT_14);
        rtl8127_set_eth_phy_ocp_bit(tp, 0xB576, BIT_0);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA436, 0x834A);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xA438,
                                              0xFF00,
                                              0x0700);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8217);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xB87E,
                                              0x3F00,
                                              0x2A00);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA436, 0x81B1);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xA438,
                                              0xFF00,
                                              0x0B00);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8FED);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xB87E,
                                              0xFF00,
                                              0x4E00);

        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x88AC);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xB87E,
                                              0xFF00,
                                              0x2300);
        rtl8127_set_eth_phy_ocp_bit(tp, 0xBF0C, 0x3800);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x88DE);
        rtl8127_clear_eth_phy_ocp_bit(tp, 0xB87E, 0xFF00);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x80B4);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x5195);


        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8370);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA438, 0x8671);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8372);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA438, 0x86C8);

        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8401);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA438, 0x86C8);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8403);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA438, 0x86DA);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8406);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xA438,
                                              0x1800,
                                              0x1000);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8408);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xA438,
                                              0x1800,
                                              0x1000);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA436, 0x840A);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xA438,
                                              0x1800,
                                              0x1000);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA436, 0x840C);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xA438,
                                              0x1800,
                                              0x1000);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA436, 0x840E);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xA438,
                                              0x1800,
                                              0x1000);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8410);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xA438,
                                              0x1800,
                                              0x1000);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8412);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xA438,
                                              0x1800,
                                              0x1000);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8414);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xA438,
                                              0x1800,
                                              0x1000);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8416);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xA438,
                                              0x1800,
                                              0x1000);


        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA436, 0x82BD);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1F40);


        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xBFB4,
                                              0x07FF,
                                              0x0328);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xBFB6, 0x3E14);


        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA436, 0x81C4);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA438, 0x003B);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0086);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA438, 0x00B7);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA438, 0x00DB);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA438, 0x00FE);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA438, 0x00FE);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA438, 0x00FE);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA438, 0x00FE);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA438, 0x00C3);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0078);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0047);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0023);


        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x88D7);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x01A0);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x88D9);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x01A0);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8FFA);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x002A);

        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8FEE);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87E, 0xFFDF);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8FF0);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87E, 0xFFFF);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8FF2);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0A4A);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8FF4);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87E, 0xAA5A);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8FF6);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0A4A);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8FF8);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87E, 0xAA5A);

        rtl8127_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x88D5);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xB87E,
                                              0xFF00,
                                              0x0200);


        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA436, 0x84BB);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xA438,
                                              0xFF00,
                                              0x0A00);
        rtl8127_mdio_direct_write_phy_ocp(tp, 0xA436, 0x84C0);
        rtl8127_clear_and_set_eth_phy_ocp_bit(tp,
                                              0xA438,
                                              0xFF00,
                                              0x1600);


        rtl8127_set_eth_phy_ocp_bit(tp, 0xA430, BIT_1 | BIT_0);


        if (aspm && HW_HAS_WRITE_PHY_MCU_RAM_CODE(tp))
                rtl8127_enable_phy_aldps(tp);
}

static void
rtl8127_hw_phy_config(struct net_device *dev)
{
        struct rtl8127_private *tp = netdev_priv(dev);
        unsigned long flags;

        if (tp->resume_not_chg_speed)
                return;

        tp->phy_reset_enable(dev);

        r8127_spin_lock(&tp->phy_lock, flags);

        switch (tp->mcfg) {
        case CFG_METHOD_1:
                rtl8127_hw_phy_config_8127a_tc_1(dev);
                break;
        case CFG_METHOD_2:
                rtl8127_hw_phy_config_8127a_1(dev);
                break;
        default:
                break;
        }

        //legacy force mode(Chap 22)
        rtl8127_clear_eth_phy_ocp_bit(tp, 0xA5B4, BIT_15);

        rtl8127_hw_fiber_phy_config(tp);

        rtl8127_mdio_write(tp, 0x1F, 0x0000);

        if (tp->eee.eee_enabled)
                rtl8127_enable_eee(tp);
        else
                rtl8127_disable_eee(tp);

        r8127_spin_unlock(&tp->phy_lock, flags);
}

static void
rtl8127_up(struct net_device *dev)
{
        rtl8127_hw_init(dev);
        rtl8127_hw_reset(dev);
        rtl8127_powerup_pll(dev);
        rtl8127_hw_ephy_config(dev);
        rtl8127_hw_phy_config(dev);
        rtl8127_hw_config(dev);
}

#ifdef CONFIG_NET_POLL_CONTROLLER
/*
 * Polling 'interrupt' - used by things like netconsole to send skbs
 * without having to re-enable interrupts. It's not called while
 * the interrupt routine is executing.
 */
static void
rtl8127_netpoll(struct net_device *dev)
{
        struct rtl8127_private *tp = netdev_priv(dev);
        int i;
        for (i = 0; i < tp->irq_nvecs; i++) {
                struct r8127_irq *irq = &tp->irq_tbl[i];
                struct r8127_napi *r8127napi = &tp->r8127napi[i];

                disable_irq(irq->vector);

                irq->handler(irq->vector, r8127napi);

                enable_irq(irq->vector);
        }
}
#endif //CONFIG_NET_POLL_CONTROLLER

static void
rtl8127_setup_interrupt_mask(struct rtl8127_private *tp)
{
        int i;

        if (tp->HwCurrIsrVer == 6) {
                tp->intr_mask = ISRIMR_V6_LINKCHG | ISRIMR_V6_TOK_Q0;
                if (tp->num_tx_rings > 1)
                        tp->intr_mask |= ISRIMR_V6_TOK_Q1;
                for (i = 0; i < tp->num_rx_rings; i++)
                        tp->intr_mask |= ISRIMR_V6_ROK_Q0 << i;

                if (tp->DASH)
                        tp->intr_l2_mask |= ISRIMR_V6_L2_MISC_INTR;
        } else if (tp->HwCurrIsrVer == 5) {
                tp->intr_mask = ISRIMR_V5_LINKCHG | ISRIMR_V5_TOK_Q0;
                if (tp->num_tx_rings > 1)
                        tp->intr_mask |= ISRIMR_V5_TOK_Q1;
                for (i = 0; i < tp->num_rx_rings; i++)
                        tp->intr_mask |= ISRIMR_V5_ROK_Q0 << i;
        } else if (tp->HwCurrIsrVer == 4) {
                tp->intr_mask = ISRIMR_V4_LINKCHG;
                for (i = 0; i < tp->num_rx_rings; i++)
                        tp->intr_mask |= ISRIMR_V4_ROK_Q0 << i;
        } else if (tp->HwCurrIsrVer == 3) {
                tp->intr_mask = ISRIMR_V2_LINKCHG;
                for (i = 0; i < max(tp->num_tx_rings, tp->num_rx_rings); i++)
                        tp->intr_mask |= ISRIMR_V2_ROK_Q0 << i;
        } else if (tp->HwCurrIsrVer == 2) {
                tp->intr_mask = ISRIMR_V2_LINKCHG | ISRIMR_TOK_Q0;
                if (tp->num_tx_rings > 1)
                        tp->intr_mask |= ISRIMR_TOK_Q1;

                for (i = 0; i < tp->num_rx_rings; i++)
                        tp->intr_mask |= ISRIMR_V2_ROK_Q0 << i;
        } else {
                tp->intr_mask = LinkChg | RxDescUnavail | TxOK | RxOK | SWInt;
                tp->timer_intr_mask = LinkChg | PCSTimeout;

#ifdef ENABLE_DASH_SUPPORT
                if (tp->DASH) {
                        if (HW_DASH_SUPPORT_IPC2(tp)) {
                                tp->timer_intr_mask |= ISRIMR_DASH_INTR_EN;
                                tp->intr_mask |= ISRIMR_DASH_INTR_EN;
                        }
                }
#endif
        }
}

static void
rtl8127_setup_mqs_reg(struct rtl8127_private *tp)
{
        u16 hw_clo_ptr0_reg, sw_tail_ptr0_reg;
        u16 reg_len;
        int i;

        //tx
        tp->tx_ring[0].tdsar_reg = TxDescStartAddrLow;
        for (i = 1; i < tp->HwSuppNumTxQueues; i++)
                tp->tx_ring[i].tdsar_reg = (u16)(TNPDS_Q1_LOW_8125 + (i - 1) * 8);

        switch (tp->HwSuppTxNoCloseVer) {
        case 4:
        case 5:
                hw_clo_ptr0_reg = HW_CLO_PTR0_8126;
                sw_tail_ptr0_reg = SW_TAIL_PTR0_8126;
                reg_len = 4;
                break;
        case 6:
                hw_clo_ptr0_reg = HW_CLO_PTR0_8125BP;
                sw_tail_ptr0_reg = SW_TAIL_PTR0_8125BP;
                reg_len = 8;
                break;
        default:
                hw_clo_ptr0_reg = HW_CLO_PTR0_8125;
                sw_tail_ptr0_reg = SW_TAIL_PTR0_8125;
                reg_len = 4;
                break;
        }

        for (i = 0; i < tp->HwSuppNumTxQueues; i++) {
                tp->tx_ring[i].hw_clo_ptr_reg = (u16)(hw_clo_ptr0_reg + i * reg_len);
                tp->tx_ring[i].sw_tail_ptr_reg = (u16)(sw_tail_ptr0_reg + i * reg_len);
        }

        //rx
        tp->rx_ring[0].rdsar_reg = RxDescAddrLow;
        for (i = 1; i < tp->HwSuppNumRxQueues; i++)
                tp->rx_ring[i].rdsar_reg = (u16)(RDSAR_Q1_LOW_8125 + (i - 1) * 8);

        tp->isr_reg[0] = ISR0_8125;
        for (i = 1; i < tp->hw_supp_irq_nvecs; i++)
                tp->isr_reg[i] = (u16)(ISR1_8125 + (i - 1) * 4);

        tp->imr_reg[0] = IMR0_8125;
        for (i = 1; i < tp->hw_supp_irq_nvecs; i++)
                tp->imr_reg[i] = (u16)(IMR1_8125 + (i - 1) * 4);
}

static void
rtl8127_init_software_variable(struct net_device *dev)
{
        struct rtl8127_private *tp = netdev_priv(dev);
        struct pci_dev *pdev = tp->pci_dev;

#ifdef ENABLE_LIB_SUPPORT
        tp->ring_lib_enabled = 1;
#endif

        switch (tp->mcfg) {
        case CFG_METHOD_2: {
                u8 tmp = (u8)rtl8127_mac_ocp_read(tp, 0xD006);
                if (tmp == 0x03)
                        tp->HwSuppDashVer = 4;
        }
        break;
        }

        switch (tp->mcfg) {
        case CFG_METHOD_2:
                if (HW_DASH_SUPPORT_DASH(tp))
                        tp->HwSuppOcpChannelVer = 2;
                break;
        }
        tp->AllowAccessDashOcp = rtl8127_is_allow_access_dash_ocp(tp);

        tp->HwPkgDet = rtl8127_mac_ocp_read(tp, 0xDC00);
        tp->HwPkgDet = (tp->HwPkgDet >> 3) & 0x07;

        tp->HwSuppNowIsOobVer = 1;

        tp->HwPcieSNOffset = 0x168;

#ifdef ENABLE_REALWOW_SUPPORT
        rtl8127_get_realwow_hw_version(dev);
#endif //ENABLE_REALWOW_SUPPORT

        tp->DASH = rtl8127_check_dash(tp);

        if (tp->DASH) {
                eee_enable = 0;

                tp->SizeOfSendToFwBuffer = SEND_TO_FW_BUF_SIZE;
                tp->SizeOfRecvFromFwBuffer = RECV_FROM_FW_BUF_SIZE;

                tp->DashFirmwareVersion = rtl8127_get_dash_fw_ver(tp);
        }

        if (aspm) {
                tp->org_pci_offset_99 = rtl8127_csi_fun0_read_byte(tp, 0x99);
                tp->org_pci_offset_99 &= ~(BIT_5|BIT_6);

                tp->org_pci_offset_180 = rtl8127_csi_fun0_read_byte(tp, 0x254);
        }

        pci_read_config_byte(pdev, 0x80, &tp->org_pci_offset_80);
        pci_read_config_byte(pdev, 0x81, &tp->org_pci_offset_81);

        tp->use_timer_interrupt = TRUE;

        tp->HwSuppMaxPhyLinkSpeed = 10000;

        if (timer_count == 0 || tp->mcfg == CFG_METHOD_DEFAULT)
                tp->use_timer_interrupt = FALSE;

        tp->ShortPacketSwChecksum = TRUE;
        tp->UseSwPaddingShortPkt = TRUE;

        rtl8127_check_fiber_mode_support(tp);
        if (HW_FIBER_MODE_ENABLED(tp)) {
                eee_enable = 0;
                eee_giga_lite = 0;
        }

        tp->HwSuppMagicPktVer = WAKEUP_MAGIC_PACKET_V3;

        tp->HwSuppLinkChgWakeUpVer = 3;

        tp->HwSuppD0SpeedUpVer = 2;

        tp->HwSuppCheckPhyDisableModeVer = 3;

        tp->HwSuppTxNoCloseVer = 6;

        switch (tp->HwSuppTxNoCloseVer) {
        case 5:
        case 6:
                tp->MaxTxDescPtrMask = MAX_TX_NO_CLOSE_DESC_PTR_MASK_V4;
                break;
        case 4:
                tp->MaxTxDescPtrMask = MAX_TX_NO_CLOSE_DESC_PTR_MASK_V3;
                break;
        case 3:
                tp->MaxTxDescPtrMask = MAX_TX_NO_CLOSE_DESC_PTR_MASK_V2;
                break;
        default:
                tx_no_close_enable = 0;
                break;
        }

        if (tp->HwSuppTxNoCloseVer > 0 && tx_no_close_enable == 1)
                tp->EnableTxNoClose = TRUE;

        switch (tp->mcfg) {
        case CFG_METHOD_1:
                tp->sw_ram_code_ver = NIC_RAMCODE_VERSION_CFG_METHOD_1;
                break;
        case CFG_METHOD_2:
                tp->sw_ram_code_ver = NIC_RAMCODE_VERSION_CFG_METHOD_2;
                break;
        default:
                break;
        }

        if (tp->HwIcVerUnknown) {
                tp->NotWrRamCodeToMicroP = TRUE;
                tp->NotWrMcuPatchCode = TRUE;
        }

        tp->HwSuppMacMcuVer = 2;

        tp->MacMcuPageSize = RTL8127_MAC_MCU_PAGE_SIZE;

        tp->HwSuppNumTxQueues = 2;
        tp->HwSuppNumRxQueues = 4;

        //init interrupt
        tp->HwSuppIsrVer = 6;

        tp->HwCurrIsrVer = tp->HwSuppIsrVer;
        if (tp->HwCurrIsrVer > 1) {
                if (!(tp->features & RTL_FEATURE_MSIX) ||
                    tp->irq_nvecs < tp->min_irq_nvecs)
                        tp->HwCurrIsrVer = 1;
        }

        tp->num_tx_rings = 1;
#ifdef ENABLE_MULTIPLE_TX_QUEUE
#ifndef ENABLE_LIB_SUPPORT
        tp->num_tx_rings = tp->HwSuppNumTxQueues;
#endif
#endif
        if (tp->HwCurrIsrVer < 2 ||
            (tp->HwCurrIsrVer == 2 && tp->irq_nvecs < 19))
                tp->num_tx_rings = 1;

        //RSS
        tp->HwSuppRssVer = 5;
        tp->HwSuppIndirTblEntries = 128;

        tp->num_rx_rings = 1;
#ifdef ENABLE_RSS_SUPPORT
#ifdef ENABLE_LIB_SUPPORT
        if (tp->HwSuppRssVer > 0)
                tp->EnableRss = 1;
#else
        if (tp->HwSuppRssVer > 0 && tp->HwCurrIsrVer > 1) {
                u8 rss_queue_num = netif_get_num_default_rss_queues();
                tp->num_rx_rings = (tp->HwSuppNumRxQueues > rss_queue_num)?
                                   rss_queue_num : tp->HwSuppNumRxQueues;

                if (!(tp->num_rx_rings >= 2 && tp->irq_nvecs >= tp->num_rx_rings))
                        tp->num_rx_rings = 1;

                if (tp->num_rx_rings >= 2)
                        tp->EnableRss = 1;
        }
#endif
#endif

        //interrupt mask
        rtl8127_setup_interrupt_mask(tp);

        rtl8127_setup_mqs_reg(tp);

        rtl8127_set_ring_size(tp, NUM_RX_DESC, NUM_TX_DESC);

        tp->HwSuppPtpVer = 2;
#ifdef ENABLE_PTP_SUPPORT
        if (tp->HwSuppPtpVer > 0)
                tp->EnablePtp = 1;
#endif

        tp->HwSuppIntMitiVer = 6;

        tp->HwSuppTcamVer = 2;

        tp->TcamNotValidReg = TCAM_NOTVALID_ADDR_V2;
        tp->TcamValidReg = TCAM_VALID_ADDR_V2;
        tp->TcamMaAddrcOffset = TCAM_MAC_ADDR_V2;
        tp->TcamVlanTagOffset = TCAM_VLAN_TAG_V2;

        tp->HwSuppExtendTallyCounterVer = 1;

        timer_count_v2 = (timer_count / 0x200);

        tp->HwSuppRxDescType = RX_DESC_RING_TYPE_4;

        tp->InitRxDescType = RX_DESC_RING_TYPE_1;
        tp->RxDescLength = RX_DESC_LEN_TYPE_1;
        switch (tp->HwSuppRxDescType) {
        case RX_DESC_RING_TYPE_3:
                if (tp->EnableRss) {
                        tp->InitRxDescType = RX_DESC_RING_TYPE_3;
                        tp->RxDescLength = RX_DESC_LEN_TYPE_3;
                }
                break;
        case RX_DESC_RING_TYPE_4:
                if (tp->EnableRss) {
                        tp->InitRxDescType = RX_DESC_RING_TYPE_4;
                        tp->RxDescLength = RX_DESC_LEN_TYPE_4;
                }
                break;
        }

        tp->rtl8127_rx_config = rtl_chip_info[tp->chipset].RCR_Cfg;
        if (tp->InitRxDescType == RX_DESC_RING_TYPE_3)
                tp->rtl8127_rx_config |= EnableRxDescV3;
        else if (tp->InitRxDescType == RX_DESC_RING_TYPE_4)
                tp->rtl8127_rx_config &= ~EnableRxDescV4_1;

        tp->NicCustLedValue = RTL_R16(tp, CustomLED);

        tp->wol_opts = rtl8127_get_hw_wol(tp);
        tp->wol_enabled = (tp->wol_opts) ? WOL_ENABLED : WOL_DISABLED;

        rtl8127_set_link_option(tp, autoneg_mode, speed_mode, duplex_mode,
                                rtl8127_fc_full);

        tp->max_jumbo_frame_size = rtl_chip_info[tp->chipset].jumbo_frame_sz;
        /* MTU range: 60 - hw-specific max */
        dev->min_mtu = ETH_MIN_MTU;
        dev->max_mtu = tp->max_jumbo_frame_size;

        if (tp->mcfg != CFG_METHOD_DEFAULT) {
                struct ethtool_keee *eee = &tp->eee;

                eee->eee_enabled = eee_enable;
                linkmode_set_bit(ETHTOOL_LINK_MODE_100baseT_Full_BIT, eee->supported);
                linkmode_set_bit(ETHTOOL_LINK_MODE_1000baseT_Full_BIT, eee->supported);
                linkmode_set_bit(ETHTOOL_LINK_MODE_2500baseT_Full_BIT, eee->supported);
                linkmode_set_bit(ETHTOOL_LINK_MODE_5000baseT_Full_BIT, eee->supported);
                linkmode_set_bit(ETHTOOL_LINK_MODE_10000baseT_Full_BIT, eee->supported);
                linkmode_set_bit(ETHTOOL_LINK_MODE_100baseT_Full_BIT, eee->advertised);
                linkmode_set_bit(ETHTOOL_LINK_MODE_1000baseT_Full_BIT, eee->advertised);
                linkmode_set_bit(ETHTOOL_LINK_MODE_2500baseT_Full_BIT, eee->advertised);
                linkmode_set_bit(ETHTOOL_LINK_MODE_5000baseT_Full_BIT, eee->advertised);
                linkmode_set_bit(ETHTOOL_LINK_MODE_10000baseT_Full_BIT, eee->advertised);
                eee->tx_lpi_enabled = eee_enable;
                eee->tx_lpi_timer = dev->mtu + ETH_HLEN + 0x20;
        }

#ifdef ENABLE_RSS_SUPPORT
        if (tp->EnableRss)
                rtl8127_init_rss(tp);
#endif
}

static void
rtl8127_release_board(struct pci_dev *pdev,
                      struct net_device *dev)
{
        struct rtl8127_private *tp = netdev_priv(dev);
        void __iomem *ioaddr = tp->mmio_addr;

        rtl8127_rar_set(tp, tp->org_mac_addr);
        tp->wol_enabled = WOL_DISABLED;

        if (!tp->DASH)
                rtl8127_phy_power_down(dev);

        iounmap(ioaddr);
        pci_release_regions(pdev);
        pci_clear_mwi(pdev);
        pci_disable_device(pdev);
        free_netdev(dev);
}

static void
rtl8127_hw_address_set(struct net_device *dev, u8 mac_addr[MAC_ADDR_LEN])
{
        eth_hw_addr_set(dev, mac_addr);
}

static int
rtl8127_get_mac_address(struct net_device *dev)
{
        struct rtl8127_private *tp = netdev_priv(dev);
        int i;
        u8 mac_addr[MAC_ADDR_LEN];

        for (i = 0; i < MAC_ADDR_LEN; i++)
                mac_addr[i] = RTL_R8(tp, MAC0 + i);

        *(u32*)&mac_addr[0] = RTL_R32(tp, BACKUP_ADDR0_8125);
        *(u16*)&mac_addr[4] = RTL_R16(tp, BACKUP_ADDR1_8125);

        if (!is_valid_ether_addr(mac_addr)) {
                netif_err(tp, probe, dev, "Invalid ether addr %pM\n",
                          mac_addr);
                eth_random_addr(mac_addr);
                dev->addr_assign_type = NET_ADDR_RANDOM;
                netif_info(tp, probe, dev, "Random ether addr %pM\n",
                           mac_addr);
                tp->random_mac = 1;
        }

        rtl8127_hw_address_set(dev, mac_addr);
        rtl8127_rar_set(tp, mac_addr);

        /* keep the original MAC address */
        memcpy(tp->org_mac_addr, dev->dev_addr, MAC_ADDR_LEN);
        memcpy(dev->perm_addr, dev->dev_addr, MAC_ADDR_LEN);
        return 0;
}

/**
 * rtl8127_set_mac_address - Change the Ethernet Address of the NIC
 * @dev: network interface device structure
 * @p:   pointer to an address structure
 *
 * Return 0 on success, negative on failure
 **/
static int
rtl8127_set_mac_address(struct net_device *dev,
                        void *p)
{
        struct rtl8127_private *tp = netdev_priv(dev);
        struct sockaddr *addr = p;

        if (!is_valid_ether_addr(addr->sa_data))
                return -EADDRNOTAVAIL;

        rtl8127_hw_address_set(dev, addr->sa_data);

        rtl8127_rar_set(tp, dev->dev_addr);

        return 0;
}

/******************************************************************************
 * rtl8127_rar_set - Puts an ethernet address into a receive address register.
 *
 * tp - The private data structure for driver
 * addr - Address to put into receive address register
 *****************************************************************************/
void
rtl8127_rar_set(struct rtl8127_private *tp,
                const u8 *addr)
{
        uint32_t rar_low = 0;
        uint32_t rar_high = 0;

        rar_low = ((uint32_t) addr[0] |
                   ((uint32_t) addr[1] << 8) |
                   ((uint32_t) addr[2] << 16) |
                   ((uint32_t) addr[3] << 24));

        rar_high = ((uint32_t) addr[4] |
                    ((uint32_t) addr[5] << 8));

        rtl8127_enable_cfg9346_write(tp);
        RTL_W32(tp, MAC0, rar_low);
        RTL_W32(tp, MAC4, rar_high);

        rtl8127_disable_cfg9346_write(tp);
}

#ifdef ETHTOOL_OPS_COMPAT
static int ethtool_get_settings(struct net_device *dev, void *useraddr)
{
        struct ethtool_cmd cmd = { ETHTOOL_GSET };
        int err;

        if (!ethtool_ops->get_settings)
                return -EOPNOTSUPP;

        err = ethtool_ops->get_settings(dev, &cmd);
        if (err < 0)
                return err;

        if (copy_to_user(useraddr, &cmd, sizeof(cmd)))
                return -EFAULT;
        return 0;
}

static int ethtool_set_settings(struct net_device *dev, void *useraddr)
{
        struct ethtool_cmd cmd;

        if (!ethtool_ops->set_settings)
                return -EOPNOTSUPP;

        if (copy_from_user(&cmd, useraddr, sizeof(cmd)))
                return -EFAULT;

        return ethtool_ops->set_settings(dev, &cmd);
}

static int ethtool_get_drvinfo(struct net_device *dev, void *useraddr)
{
        struct ethtool_drvinfo info;
        struct ethtool_ops *ops = ethtool_ops;

        if (!ops->get_drvinfo)
                return -EOPNOTSUPP;

        memset(&info, 0, sizeof(info));
        info.cmd = ETHTOOL_GDRVINFO;
        ops->get_drvinfo(dev, &info);

        if (ops->self_test_count)
                info.testinfo_len = ops->self_test_count(dev);
        if (ops->get_stats_count)
                info.n_stats = ops->get_stats_count(dev);
        if (ops->get_regs_len)
                info.regdump_len = ops->get_regs_len(dev);
        if (ops->get_eeprom_len)
                info.eedump_len = ops->get_eeprom_len(dev);

        if (copy_to_user(useraddr, &info, sizeof(info)))
                return -EFAULT;
        return 0;
}

static int ethtool_get_regs(struct net_device *dev, char *useraddr)
{
        struct ethtool_regs regs;
        struct ethtool_ops *ops = ethtool_ops;
        void *regbuf;
        int reglen, ret;

        if (!ops->get_regs || !ops->get_regs_len)
                return -EOPNOTSUPP;

        if (copy_from_user(&regs, useraddr, sizeof(regs)))
                return -EFAULT;

        reglen = ops->get_regs_len(dev);
        if (regs.len > reglen)
                regs.len = reglen;

        regbuf = kmalloc(reglen, GFP_USER);
        if (!regbuf)
                return -ENOMEM;

        ops->get_regs(dev, &regs, regbuf);

        ret = -EFAULT;
        if (copy_to_user(useraddr, &regs, sizeof(regs)))
                goto out;
        useraddr += offsetof(struct ethtool_regs, data);
        if (copy_to_user(useraddr, regbuf, reglen))
                goto out;
        ret = 0;

out:
        kfree(regbuf);
        return ret;
}

static int ethtool_get_wol(struct net_device *dev, char *useraddr)
{
        struct ethtool_wolinfo wol = { ETHTOOL_GWOL };

        if (!ethtool_ops->get_wol)
                return -EOPNOTSUPP;

        ethtool_ops->get_wol(dev, &wol);

        if (copy_to_user(useraddr, &wol, sizeof(wol)))
                return -EFAULT;
        return 0;
}

static int ethtool_set_wol(struct net_device *dev, char *useraddr)
{
        struct ethtool_wolinfo wol;

        if (!ethtool_ops->set_wol)
                return -EOPNOTSUPP;

        if (copy_from_user(&wol, useraddr, sizeof(wol)))
                return -EFAULT;

        return ethtool_ops->set_wol(dev, &wol);
}

static int ethtool_get_msglevel(struct net_device *dev, char *useraddr)
{
        struct ethtool_value edata = { ETHTOOL_GMSGLVL };

        if (!ethtool_ops->get_msglevel)
                return -EOPNOTSUPP;

        edata.data = ethtool_ops->get_msglevel(dev);

        if (copy_to_user(useraddr, &edata, sizeof(edata)))
                return -EFAULT;
        return 0;
}

static int ethtool_set_msglevel(struct net_device *dev, char *useraddr)
{
        struct ethtool_value edata;

        if (!ethtool_ops->set_msglevel)
                return -EOPNOTSUPP;

        if (copy_from_user(&edata, useraddr, sizeof(edata)))
                return -EFAULT;

        ethtool_ops->set_msglevel(dev, edata.data);
        return 0;
}

static int ethtool_nway_reset(struct net_device *dev)
{
        if (!ethtool_ops->nway_reset)
                return -EOPNOTSUPP;

        return ethtool_ops->nway_reset(dev);
}

static int ethtool_get_link(struct net_device *dev, void *useraddr)
{
        struct ethtool_value edata = { ETHTOOL_GLINK };

        if (!ethtool_ops->get_link)
                return -EOPNOTSUPP;

        edata.data = ethtool_ops->get_link(dev);

        if (copy_to_user(useraddr, &edata, sizeof(edata)))
                return -EFAULT;
        return 0;
}

static int ethtool_get_eeprom(struct net_device *dev, void *useraddr)
{
        struct ethtool_eeprom eeprom;
        struct ethtool_ops *ops = ethtool_ops;
        u8 *data;
        int ret;

        if (!ops->get_eeprom || !ops->get_eeprom_len)
                return -EOPNOTSUPP;

        if (copy_from_user(&eeprom, useraddr, sizeof(eeprom)))
                return -EFAULT;

        /* Check for wrap and zero */
        if (eeprom.offset + eeprom.len <= eeprom.offset)
                return -EINVAL;

        /* Check for exceeding total eeprom len */
        if (eeprom.offset + eeprom.len > ops->get_eeprom_len(dev))
                return -EINVAL;

        data = kmalloc(eeprom.len, GFP_USER);
        if (!data)
                return -ENOMEM;

        ret = -EFAULT;
        if (copy_from_user(data, useraddr + sizeof(eeprom), eeprom.len))
                goto out;

        ret = ops->get_eeprom(dev, &eeprom, data);
        if (ret)
                goto out;

        ret = -EFAULT;
        if (copy_to_user(useraddr, &eeprom, sizeof(eeprom)))
                goto out;
        if (copy_to_user(useraddr + sizeof(eeprom), data, eeprom.len))
                goto out;
        ret = 0;

out:
        kfree(data);
        return ret;
}

static int ethtool_set_eeprom(struct net_device *dev, void *useraddr)
{
        struct ethtool_eeprom eeprom;
        struct ethtool_ops *ops = ethtool_ops;
        u8 *data;
        int ret;

        if (!ops->set_eeprom || !ops->get_eeprom_len)
                return -EOPNOTSUPP;

        if (copy_from_user(&eeprom, useraddr, sizeof(eeprom)))
                return -EFAULT;

        /* Check for wrap and zero */
        if (eeprom.offset + eeprom.len <= eeprom.offset)
                return -EINVAL;

        /* Check for exceeding total eeprom len */
        if (eeprom.offset + eeprom.len > ops->get_eeprom_len(dev))
                return -EINVAL;

        data = kmalloc(eeprom.len, GFP_USER);
        if (!data)
                return -ENOMEM;

        ret = -EFAULT;
        if (copy_from_user(data, useraddr + sizeof(eeprom), eeprom.len))
                goto out;

        ret = ops->set_eeprom(dev, &eeprom, data);
        if (ret)
                goto out;

        if (copy_to_user(useraddr + sizeof(eeprom), data, eeprom.len))
                ret = -EFAULT;

out:
        kfree(data);
        return ret;
}

static int ethtool_get_coalesce(struct net_device *dev, void *useraddr)
{
        struct ethtool_coalesce coalesce = { ETHTOOL_GCOALESCE };

        if (!ethtool_ops->get_coalesce)
                return -EOPNOTSUPP;

        ethtool_ops->get_coalesce(dev, &coalesce);

        if (copy_to_user(useraddr, &coalesce, sizeof(coalesce)))
                return -EFAULT;
        return 0;
}

static int ethtool_set_coalesce(struct net_device *dev, void *useraddr)
{
        struct ethtool_coalesce coalesce;

        if (!ethtool_ops->get_coalesce)
                return -EOPNOTSUPP;

        if (copy_from_user(&coalesce, useraddr, sizeof(coalesce)))
                return -EFAULT;

        return ethtool_ops->set_coalesce(dev, &coalesce);
}

static int ethtool_get_ringparam(struct net_device *dev, void *useraddr)
{
        struct ethtool_ringparam ringparam = { ETHTOOL_GRINGPARAM };

        if (!ethtool_ops->get_ringparam)
                return -EOPNOTSUPP;

        ethtool_ops->get_ringparam(dev, &ringparam);

        if (copy_to_user(useraddr, &ringparam, sizeof(ringparam)))
                return -EFAULT;
        return 0;
}

static int ethtool_set_ringparam(struct net_device *dev, void *useraddr)
{
        struct ethtool_ringparam ringparam;

        if (!ethtool_ops->get_ringparam)
                return -EOPNOTSUPP;

        if (copy_from_user(&ringparam, useraddr, sizeof(ringparam)))
                return -EFAULT;

        return ethtool_ops->set_ringparam(dev, &ringparam);
}

static int ethtool_get_pauseparam(struct net_device *dev, void *useraddr)
{
        struct ethtool_pauseparam pauseparam = { ETHTOOL_GPAUSEPARAM };

        if (!ethtool_ops->get_pauseparam)
                return -EOPNOTSUPP;

        ethtool_ops->get_pauseparam(dev, &pauseparam);

        if (copy_to_user(useraddr, &pauseparam, sizeof(pauseparam)))
                return -EFAULT;
        return 0;
}

static int ethtool_set_pauseparam(struct net_device *dev, void *useraddr)
{
        struct ethtool_pauseparam pauseparam;

        if (!ethtool_ops->get_pauseparam)
                return -EOPNOTSUPP;

        if (copy_from_user(&pauseparam, useraddr, sizeof(pauseparam)))
                return -EFAULT;

        return ethtool_ops->set_pauseparam(dev, &pauseparam);
}

static int ethtool_get_rx_csum(struct net_device *dev, char *useraddr)
{
        struct ethtool_value edata = { ETHTOOL_GRXCSUM };

        if (!ethtool_ops->get_rx_csum)
                return -EOPNOTSUPP;

        edata.data = ethtool_ops->get_rx_csum(dev);

        if (copy_to_user(useraddr, &edata, sizeof(edata)))
                return -EFAULT;
        return 0;
}

static int ethtool_set_rx_csum(struct net_device *dev, char *useraddr)
{
        struct ethtool_value edata;

        if (!ethtool_ops->set_rx_csum)
                return -EOPNOTSUPP;

        if (copy_from_user(&edata, useraddr, sizeof(edata)))
                return -EFAULT;

        ethtool_ops->set_rx_csum(dev, edata.data);
        return 0;
}

static int ethtool_get_tx_csum(struct net_device *dev, char *useraddr)
{
        struct ethtool_value edata = { ETHTOOL_GTXCSUM };

        if (!ethtool_ops->get_tx_csum)
                return -EOPNOTSUPP;

        edata.data = ethtool_ops->get_tx_csum(dev);

        if (copy_to_user(useraddr, &edata, sizeof(edata)))
                return -EFAULT;
        return 0;
}

static int ethtool_set_tx_csum(struct net_device *dev, char *useraddr)
{
        struct ethtool_value edata;

        if (!ethtool_ops->set_tx_csum)
                return -EOPNOTSUPP;

        if (copy_from_user(&edata, useraddr, sizeof(edata)))
                return -EFAULT;

        return ethtool_ops->set_tx_csum(dev, edata.data);
}

static int ethtool_get_sg(struct net_device *dev, char *useraddr)
{
        struct ethtool_value edata = { ETHTOOL_GSG };

        if (!ethtool_ops->get_sg)
                return -EOPNOTSUPP;

        edata.data = ethtool_ops->get_sg(dev);

        if (copy_to_user(useraddr, &edata, sizeof(edata)))
                return -EFAULT;
        return 0;
}

static int ethtool_set_sg(struct net_device *dev, char *useraddr)
{
        struct ethtool_value edata;

        if (!ethtool_ops->set_sg)
                return -EOPNOTSUPP;

        if (copy_from_user(&edata, useraddr, sizeof(edata)))
                return -EFAULT;

        return ethtool_ops->set_sg(dev, edata.data);
}

static int ethtool_get_tso(struct net_device *dev, char *useraddr)
{
        struct ethtool_value edata = { ETHTOOL_GTSO };

        if (!ethtool_ops->get_tso)
                return -EOPNOTSUPP;

        edata.data = ethtool_ops->get_tso(dev);

        if (copy_to_user(useraddr, &edata, sizeof(edata)))
                return -EFAULT;
        return 0;
}

static int ethtool_set_tso(struct net_device *dev, char *useraddr)
{
        struct ethtool_value edata;

        if (!ethtool_ops->set_tso)
                return -EOPNOTSUPP;

        if (copy_from_user(&edata, useraddr, sizeof(edata)))
                return -EFAULT;

        return ethtool_ops->set_tso(dev, edata.data);
}

static int ethtool_self_test(struct net_device *dev, char *useraddr)
{
        struct ethtool_test test;
        struct ethtool_ops *ops = ethtool_ops;
        u64 *data;
        int ret;

        if (!ops->self_test || !ops->self_test_count)
                return -EOPNOTSUPP;

        if (copy_from_user(&test, useraddr, sizeof(test)))
                return -EFAULT;

        test.len = ops->self_test_count(dev);
        data = kmalloc(test.len * sizeof(u64), GFP_USER);
        if (!data)
                return -ENOMEM;

        ops->self_test(dev, &test, data);

        ret = -EFAULT;
        if (copy_to_user(useraddr, &test, sizeof(test)))
                goto out;
        useraddr += sizeof(test);
        if (copy_to_user(useraddr, data, test.len * sizeof(u64)))
                goto out;
        ret = 0;

out:
        kfree(data);
        return ret;
}

static int ethtool_get_strings(struct net_device *dev, void *useraddr)
{
        struct ethtool_gstrings gstrings;
        struct ethtool_ops *ops = ethtool_ops;
        u8 *data;
        int ret;

        if (!ops->get_strings)
                return -EOPNOTSUPP;

        if (copy_from_user(&gstrings, useraddr, sizeof(gstrings)))
                return -EFAULT;

        switch (gstrings.string_set) {
        case ETH_SS_TEST:
                if (!ops->self_test_count)
                        return -EOPNOTSUPP;
                gstrings.len = ops->self_test_count(dev);
                break;
        case ETH_SS_STATS:
                if (!ops->get_stats_count)
                        return -EOPNOTSUPP;
                gstrings.len = ops->get_stats_count(dev);
                break;
        default:
                return -EINVAL;
        }

        data = kmalloc(gstrings.len * ETH_GSTRING_LEN, GFP_USER);
        if (!data)
                return -ENOMEM;

        ops->get_strings(dev, gstrings.string_set, data);

        ret = -EFAULT;
        if (copy_to_user(useraddr, &gstrings, sizeof(gstrings)))
                goto out;
        useraddr += sizeof(gstrings);
        if (copy_to_user(useraddr, data, gstrings.len * ETH_GSTRING_LEN))
                goto out;
        ret = 0;

out:
        kfree(data);
        return ret;
}

static int ethtool_phys_id(struct net_device *dev, void *useraddr)
{
        struct ethtool_value id;

        if (!ethtool_ops->phys_id)
                return -EOPNOTSUPP;

        if (copy_from_user(&id, useraddr, sizeof(id)))
                return -EFAULT;

        return ethtool_ops->phys_id(dev, id.data);
}

static int ethtool_get_stats(struct net_device *dev, void *useraddr)
{
        struct ethtool_stats stats;
        struct ethtool_ops *ops = ethtool_ops;
        u64 *data;
        int ret;

        if (!ops->get_ethtool_stats || !ops->get_stats_count)
                return -EOPNOTSUPP;

        if (copy_from_user(&stats, useraddr, sizeof(stats)))
                return -EFAULT;

        stats.n_stats = ops->get_stats_count(dev);
        data = kmalloc(stats.n_stats * sizeof(u64), GFP_USER);
        if (!data)
                return -ENOMEM;

        ops->get_ethtool_stats(dev, &stats, data);

        ret = -EFAULT;
        if (copy_to_user(useraddr, &stats, sizeof(stats)))
                goto out;
        useraddr += sizeof(stats);
        if (copy_to_user(useraddr, data, stats.n_stats * sizeof(u64)))
                goto out;
        ret = 0;

out:
        kfree(data);
        return ret;
}

static int ethtool_ioctl(struct ifreq *ifr)
{
        struct net_device *dev = __dev_get_by_name(ifr->ifr_name);
        void *useraddr = (void *) ifr->ifr_data;
        u32 ethcmd;

        /*
         * XXX: This can be pushed down into the ethtool_* handlers that
         * need it.  Keep existing behaviour for the moment.
         */
        if (!capable(CAP_NET_ADMIN))
                return -EPERM;

        if (!dev || !netif_device_present(dev))
                return -ENODEV;

        if (copy_from_user(&ethcmd, useraddr, sizeof (ethcmd)))
                return -EFAULT;

        switch (ethcmd) {
        case ETHTOOL_GSET:
                return ethtool_get_settings(dev, useraddr);
        case ETHTOOL_SSET:
                return ethtool_set_settings(dev, useraddr);
        case ETHTOOL_GDRVINFO:
                return ethtool_get_drvinfo(dev, useraddr);
        case ETHTOOL_GREGS:
                return ethtool_get_regs(dev, useraddr);
        case ETHTOOL_GWOL:
                return ethtool_get_wol(dev, useraddr);
        case ETHTOOL_SWOL:
                return ethtool_set_wol(dev, useraddr);
        case ETHTOOL_GMSGLVL:
                return ethtool_get_msglevel(dev, useraddr);
        case ETHTOOL_SMSGLVL:
                return ethtool_set_msglevel(dev, useraddr);
        case ETHTOOL_NWAY_RST:
                return ethtool_nway_reset(dev);
        case ETHTOOL_GLINK:
                return ethtool_get_link(dev, useraddr);
        case ETHTOOL_GEEPROM:
                return ethtool_get_eeprom(dev, useraddr);
        case ETHTOOL_SEEPROM:
                return ethtool_set_eeprom(dev, useraddr);
        case ETHTOOL_GCOALESCE:
                return ethtool_get_coalesce(dev, useraddr);
        case ETHTOOL_SCOALESCE:
                return ethtool_set_coalesce(dev, useraddr);
        case ETHTOOL_GRINGPARAM:
                return ethtool_get_ringparam(dev, useraddr);
        case ETHTOOL_SRINGPARAM:
                return ethtool_set_ringparam(dev, useraddr);
        case ETHTOOL_GPAUSEPARAM:
                return ethtool_get_pauseparam(dev, useraddr);
        case ETHTOOL_SPAUSEPARAM:
                return ethtool_set_pauseparam(dev, useraddr);
        case ETHTOOL_GRXCSUM:
                return ethtool_get_rx_csum(dev, useraddr);
        case ETHTOOL_SRXCSUM:
                return ethtool_set_rx_csum(dev, useraddr);
        case ETHTOOL_GTXCSUM:
                return ethtool_get_tx_csum(dev, useraddr);
        case ETHTOOL_STXCSUM:
                return ethtool_set_tx_csum(dev, useraddr);
        case ETHTOOL_GSG:
                return ethtool_get_sg(dev, useraddr);
        case ETHTOOL_SSG:
                return ethtool_set_sg(dev, useraddr);
        case ETHTOOL_GTSO:
                return ethtool_get_tso(dev, useraddr);
        case ETHTOOL_STSO:
                return ethtool_set_tso(dev, useraddr);
        case ETHTOOL_TEST:
                return ethtool_self_test(dev, useraddr);
        case ETHTOOL_GSTRINGS:
                return ethtool_get_strings(dev, useraddr);
        case ETHTOOL_PHYS_ID:
                return ethtool_phys_id(dev, useraddr);
        case ETHTOOL_GSTATS:
                return ethtool_get_stats(dev, useraddr);
        default:
                return -EOPNOTSUPP;
        }

        return -EOPNOTSUPP;
}
#endif //ETHTOOL_OPS_COMPAT

static int rtl8127_siocdevprivate(struct net_device *dev, struct ifreq *ifr,
                                  void __user *data, int cmd)
{
        struct rtl8127_private *tp = netdev_priv(dev);
        int ret = 0;

        switch (cmd) {
#ifdef ENABLE_DASH_SUPPORT
        case SIOCDEVPRIVATE_RTLDASH:
                if (!netif_running(dev)) {
                        ret = -ENODEV;
                        break;
                }
                if (!capable(CAP_NET_ADMIN)) {
                        ret = -EPERM;
                        break;
                }

                ret = rtl8127_dash_ioctl(dev, ifr);
                break;
#endif

#ifdef ENABLE_REALWOW_SUPPORT
        case SIOCDEVPRIVATE_RTLREALWOW:
                if (!netif_running(dev)) {
                        ret = -ENODEV;
                        break;
                }

                ret = rtl8127_realwow_ioctl(dev, ifr);
                break;
#endif

        case SIOCRTLTOOL:
                if (!capable(CAP_NET_ADMIN)) {
                        ret = -EPERM;
                        break;
                }

                ret = rtl8127_tool_ioctl(tp, ifr);
                break;

        default:
                ret = -EOPNOTSUPP;
        }

        return ret;
}

static int
rtl8127_do_ioctl(struct net_device *dev,
                 struct ifreq *ifr,
                 int cmd)
{
        struct rtl8127_private *tp = netdev_priv(dev);
        struct mii_ioctl_data *data = if_mii(ifr);
        int ret = 0;

        switch (cmd) {
        case SIOCGMIIPHY:
                data->phy_id = 32; /* Internal PHY */
                break;

        case SIOCGMIIREG:
                rtl8127_mdio_write(tp, 0x1F, 0x0000);
                data->val_out = rtl8127_mdio_read(tp, data->reg_num);
                break;

        case SIOCSMIIREG:
                if (!capable(CAP_NET_ADMIN))
                        return -EPERM;
                rtl8127_mdio_write(tp, 0x1F, 0x0000);
                rtl8127_mdio_write(tp, data->reg_num, data->val_in);
                break;

#ifdef ETHTOOL_OPS_COMPAT
        case SIOCETHTOOL:
                ret = ethtool_ioctl(ifr);
                break;
#endif

#ifdef ENABLE_PTP_SUPPORT
        case SIOCSHWTSTAMP:
        case SIOCGHWTSTAMP:
                if (tp->EnablePtp)
                        ret = rtl8127_ptp_ioctl(dev, ifr, cmd);
                else
                        ret = -EOPNOTSUPP;
                break;
#endif

        default:
                ret = -EOPNOTSUPP;
                break;
        }

        return ret;
}

static void
rtl8127_phy_power_up(struct net_device *dev)
{
        struct rtl8127_private *tp = netdev_priv(dev);
        unsigned long flags;

        if (rtl8127_is_in_phy_disable_mode(dev))
                return;

        r8127_spin_lock(&tp->phy_lock, flags);

        rtl8127_mdio_write(tp, 0x1F, 0x0000);
        rtl8127_mdio_write(tp, MII_BMCR, BMCR_ANENABLE);

        //wait ups resume (phy state 3)
        rtl8127_wait_phy_ups_resume(dev, 3);

        r8127_spin_unlock(&tp->phy_lock, flags);
}

static void
rtl8127_phy_power_down(struct net_device *dev)
{
        struct rtl8127_private *tp = netdev_priv(dev);
        unsigned long flags;

        r8127_spin_lock(&tp->phy_lock, flags);
        rtl8127_mdio_write(tp, 0x1F, 0x0000);
        rtl8127_mdio_write(tp, MII_BMCR, BMCR_ANENABLE | BMCR_PDOWN);
        r8127_spin_unlock(&tp->phy_lock, flags);
}

static int __devinit
rtl8127_init_board(struct pci_dev *pdev,
                   struct net_device **dev_out,
                   void __iomem **ioaddr_out)
{
        void __iomem *ioaddr;
        struct net_device *dev;
        struct rtl8127_private *tp;
        int rc = -ENOMEM, i, pm_cap;

        assert(ioaddr_out != NULL);

        /* dev zeroed in alloc_etherdev */
        dev = alloc_etherdev_mq(sizeof (*tp), R8127_MAX_QUEUES);
        if (dev == NULL) {
                if (netif_msg_drv(&debug))
                        dev_err(&pdev->dev, "unable to alloc new ethernet\n");
                goto err_out;
        }

        SET_MODULE_OWNER(dev);
        SET_NETDEV_DEV(dev, &pdev->dev);
        tp = netdev_priv(dev);
        tp->dev = dev;
        tp->pci_dev = pdev;
        tp->msg_enable = netif_msg_init(debug.msg_enable, R8127_MSG_DEFAULT);

        if (!aspm)
                pci_disable_link_state(pdev, PCIE_LINK_STATE_L0S | PCIE_LINK_STATE_L1 |
                                       PCIE_LINK_STATE_CLKPM);

        /* enable device (incl. PCI PM wakeup and hotplug setup) */
        rc = pci_enable_device(pdev);
        if (rc < 0) {
                if (netif_msg_probe(tp))
                        dev_err(&pdev->dev, "enable failure\n");
                goto err_out_free_dev;
        }

        if (pci_set_mwi(pdev) < 0) {
                if (netif_msg_drv(&debug))
                        dev_info(&pdev->dev, "Mem-Wr-Inval unavailable.\n");
        }

        /* save power state before pci_enable_device overwrites it */
        pm_cap = pci_find_capability(pdev, PCI_CAP_ID_PM);
        if (pm_cap) {
                u16 pwr_command;

                pci_read_config_word(pdev, pm_cap + PCI_PM_CTRL, &pwr_command);
        } else {
                if (netif_msg_probe(tp))
                        dev_err(&pdev->dev, "PowerManagement capability not found.\n");
        }

        /* make sure PCI base addr 1 is MMIO */
        if (!(pci_resource_flags(pdev, 2) & IORESOURCE_MEM)) {
                if (netif_msg_probe(tp))
                        dev_err(&pdev->dev, "region #1 not an MMIO resource, aborting\n");
                rc = -ENODEV;
                goto err_out_mwi;
        }
        /* check for weird/broken PCI region reporting */
        if (pci_resource_len(pdev, 2) < R8127_REGS_SIZE) {
                if (netif_msg_probe(tp))
                        dev_err(&pdev->dev, "Invalid PCI region size(s), aborting\n");
                rc = -ENODEV;
                goto err_out_mwi;
        }

        rc = pci_request_regions(pdev, MODULENAME);
        if (rc < 0) {
                if (netif_msg_probe(tp))
                        dev_err(&pdev->dev, "could not request regions.\n");
                goto err_out_mwi;
        }

        if ((sizeof(dma_addr_t) > 4) &&
            use_dac &&
            !dma_set_mask(&pdev->dev, DMA_BIT_MASK(64)) &&
            !dma_set_coherent_mask(&pdev->dev, DMA_BIT_MASK(64))) {
                dev->features |= NETIF_F_HIGHDMA;
        } else {
                rc = dma_set_mask(&pdev->dev, DMA_BIT_MASK(32));
                if (rc < 0) {
                        if (netif_msg_probe(tp))
                                dev_err(&pdev->dev, "DMA configuration failed.\n");
                        goto err_out_free_res;
                }
        }

        /* ioremap MMIO region */
        ioaddr = ioremap(pci_resource_start(pdev, 2), pci_resource_len(pdev, 2));
        if (ioaddr == NULL) {
                if (netif_msg_probe(tp))
                        dev_err(&pdev->dev, "cannot remap MMIO, aborting\n");
                rc = -EIO;
                goto err_out_free_res;
        }

        tp->mmio_addr = ioaddr;

        /* Identify chip attached to board */
        rtl8127_get_mac_version(tp);

        rtl8127_print_mac_version(tp);

        for (i = ARRAY_SIZE(rtl_chip_info) - 1; i >= 0; i--) {
                if (tp->mcfg == rtl_chip_info[i].mcfg)
                        break;
        }

        if (i < 0) {
                /* Unknown chip: assume array element #0, original RTL-8125 */
                if (netif_msg_probe(tp))
                        dev_printk(KERN_DEBUG, &pdev->dev, "unknown chip version, assuming %s\n", rtl_chip_info[0].name);
                i++;
        }

        tp->chipset = i;

        *ioaddr_out = ioaddr;
        *dev_out = dev;
out:
        return rc;

err_out_free_res:
        pci_release_regions(pdev);
err_out_mwi:
        pci_clear_mwi(pdev);
        pci_disable_device(pdev);
err_out_free_dev:
        free_netdev(dev);
err_out:
        *ioaddr_out = NULL;
        *dev_out = NULL;
        goto out;
}

static void
rtl8127_esd_checker(struct rtl8127_private *tp)
{
        struct net_device *dev = tp->dev;
        struct pci_dev *pdev = tp->pci_dev;
        u8 cmd;
        u16 io_base_l;
        u16 mem_base_l;
        u16 mem_base_h;
        u8 ilr;
        u16 resv_0x1c_h;
        u16 resv_0x1c_l;
        u16 resv_0x20_l;
        u16 resv_0x20_h;
        u16 resv_0x24_l;
        u16 resv_0x24_h;
        u16 resv_0x2c_h;
        u16 resv_0x2c_l;
        u32 pci_sn_l;
        u32 pci_sn_h;

        if (unlikely(tp->rtk_enable_diag))
                goto exit;

        tp->esd_flag = 0;

        pci_read_config_byte(pdev, PCI_COMMAND, &cmd);
        if (cmd != tp->pci_cfg_space.cmd) {
                printk(KERN_ERR "%s: cmd = 0x%02x, should be 0x%02x \n.", dev->name, cmd, tp->pci_cfg_space.cmd);
                pci_write_config_byte(pdev, PCI_COMMAND, tp->pci_cfg_space.cmd);
                tp->esd_flag |= BIT_0;

                pci_read_config_byte(pdev, PCI_COMMAND, &cmd);
                if (cmd == 0xff) {
                        printk(KERN_ERR "%s: pci link is down \n.", dev->name);
                        goto exit;
                }
        }

        pci_read_config_word(pdev, PCI_BASE_ADDRESS_0, &io_base_l);
        if (io_base_l != tp->pci_cfg_space.io_base_l) {
                printk(KERN_ERR "%s: io_base_l = 0x%04x, should be 0x%04x \n.", dev->name, io_base_l, tp->pci_cfg_space.io_base_l);
                pci_write_config_word(pdev, PCI_BASE_ADDRESS_0, tp->pci_cfg_space.io_base_l);
                tp->esd_flag |= BIT_1;
        }

        pci_read_config_word(pdev, PCI_BASE_ADDRESS_2, &mem_base_l);
        if (mem_base_l != tp->pci_cfg_space.mem_base_l) {
                printk(KERN_ERR "%s: mem_base_l = 0x%04x, should be 0x%04x \n.", dev->name, mem_base_l, tp->pci_cfg_space.mem_base_l);
                pci_write_config_word(pdev, PCI_BASE_ADDRESS_2, tp->pci_cfg_space.mem_base_l);
                tp->esd_flag |= BIT_2;
        }

        pci_read_config_word(pdev, PCI_BASE_ADDRESS_2 + 2, &mem_base_h);
        if (mem_base_h!= tp->pci_cfg_space.mem_base_h) {
                printk(KERN_ERR "%s: mem_base_h = 0x%04x, should be 0x%04x \n.", dev->name, mem_base_h, tp->pci_cfg_space.mem_base_h);
                pci_write_config_word(pdev, PCI_BASE_ADDRESS_2 + 2, tp->pci_cfg_space.mem_base_h);
                tp->esd_flag |= BIT_3;
        }

        pci_read_config_word(pdev, PCI_BASE_ADDRESS_3, &resv_0x1c_l);
        if (resv_0x1c_l != tp->pci_cfg_space.resv_0x1c_l) {
                printk(KERN_ERR "%s: resv_0x1c_l = 0x%04x, should be 0x%04x \n.", dev->name, resv_0x1c_l, tp->pci_cfg_space.resv_0x1c_l);
                pci_write_config_word(pdev, PCI_BASE_ADDRESS_3, tp->pci_cfg_space.resv_0x1c_l);
                tp->esd_flag |= BIT_4;
        }

        pci_read_config_word(pdev, PCI_BASE_ADDRESS_3 + 2, &resv_0x1c_h);
        if (resv_0x1c_h != tp->pci_cfg_space.resv_0x1c_h) {
                printk(KERN_ERR "%s: resv_0x1c_h = 0x%04x, should be 0x%04x \n.", dev->name, resv_0x1c_h, tp->pci_cfg_space.resv_0x1c_h);
                pci_write_config_word(pdev, PCI_BASE_ADDRESS_3 + 2, tp->pci_cfg_space.resv_0x1c_h);
                tp->esd_flag |= BIT_5;
        }

        pci_read_config_word(pdev, PCI_BASE_ADDRESS_4, &resv_0x20_l);
        if (resv_0x20_l != tp->pci_cfg_space.resv_0x20_l) {
                printk(KERN_ERR "%s: resv_0x20_l = 0x%04x, should be 0x%04x \n.", dev->name, resv_0x20_l, tp->pci_cfg_space.resv_0x20_l);
                pci_write_config_word(pdev, PCI_BASE_ADDRESS_4, tp->pci_cfg_space.resv_0x20_l);
                tp->esd_flag |= BIT_6;
        }

        pci_read_config_word(pdev, PCI_BASE_ADDRESS_4 + 2, &resv_0x20_h);
        if (resv_0x20_h != tp->pci_cfg_space.resv_0x20_h) {
                printk(KERN_ERR "%s: resv_0x20_h = 0x%04x, should be 0x%04x \n.", dev->name, resv_0x20_h, tp->pci_cfg_space.resv_0x20_h);
                pci_write_config_word(pdev, PCI_BASE_ADDRESS_4 + 2, tp->pci_cfg_space.resv_0x20_h);
                tp->esd_flag |= BIT_7;
        }

        pci_read_config_word(pdev, PCI_BASE_ADDRESS_5, &resv_0x24_l);
        if (resv_0x24_l != tp->pci_cfg_space.resv_0x24_l) {
                printk(KERN_ERR "%s: resv_0x24_l = 0x%04x, should be 0x%04x \n.", dev->name, resv_0x24_l, tp->pci_cfg_space.resv_0x24_l);
                pci_write_config_word(pdev, PCI_BASE_ADDRESS_5, tp->pci_cfg_space.resv_0x24_l);
                tp->esd_flag |= BIT_8;
        }

        pci_read_config_word(pdev, PCI_BASE_ADDRESS_5 + 2, &resv_0x24_h);
        if (resv_0x24_h != tp->pci_cfg_space.resv_0x24_h) {
                printk(KERN_ERR "%s: resv_0x24_h = 0x%04x, should be 0x%04x \n.", dev->name, resv_0x24_h, tp->pci_cfg_space.resv_0x24_h);
                pci_write_config_word(pdev, PCI_BASE_ADDRESS_5 + 2, tp->pci_cfg_space.resv_0x24_h);
                tp->esd_flag |= BIT_9;
        }

        pci_read_config_byte(pdev, PCI_INTERRUPT_LINE, &ilr);
        if (ilr != tp->pci_cfg_space.ilr) {
                printk(KERN_ERR "%s: ilr = 0x%02x, should be 0x%02x \n.", dev->name, ilr, tp->pci_cfg_space.ilr);
                pci_write_config_byte(pdev, PCI_INTERRUPT_LINE, tp->pci_cfg_space.ilr);
                tp->esd_flag |= BIT_10;
        }

        pci_read_config_word(pdev, PCI_SUBSYSTEM_VENDOR_ID, &resv_0x2c_l);
        if (resv_0x2c_l != tp->pci_cfg_space.resv_0x2c_l) {
                printk(KERN_ERR "%s: resv_0x2c_l = 0x%04x, should be 0x%04x \n.", dev->name, resv_0x2c_l, tp->pci_cfg_space.resv_0x2c_l);
                pci_write_config_word(pdev, PCI_SUBSYSTEM_VENDOR_ID, tp->pci_cfg_space.resv_0x2c_l);
                tp->esd_flag |= BIT_11;
        }

        pci_read_config_word(pdev, PCI_SUBSYSTEM_VENDOR_ID + 2, &resv_0x2c_h);
        if (resv_0x2c_h != tp->pci_cfg_space.resv_0x2c_h) {
                printk(KERN_ERR "%s: resv_0x2c_h = 0x%04x, should be 0x%04x \n.", dev->name, resv_0x2c_h, tp->pci_cfg_space.resv_0x2c_h);
                pci_write_config_word(pdev, PCI_SUBSYSTEM_VENDOR_ID + 2, tp->pci_cfg_space.resv_0x2c_h);
                tp->esd_flag |= BIT_12;
        }

        if (tp->HwPcieSNOffset > 0) {
                pci_sn_l = rtl8127_csi_read(tp, tp->HwPcieSNOffset);
                if (pci_sn_l != tp->pci_cfg_space.pci_sn_l) {
                        printk(KERN_ERR "%s: pci_sn_l = 0x%08x, should be 0x%08x \n.", dev->name, pci_sn_l, tp->pci_cfg_space.pci_sn_l);
                        rtl8127_csi_write(tp, tp->HwPcieSNOffset, tp->pci_cfg_space.pci_sn_l);
                        tp->esd_flag |= BIT_13;
                }

                pci_sn_h = rtl8127_csi_read(tp, tp->HwPcieSNOffset + 4);
                if (pci_sn_h != tp->pci_cfg_space.pci_sn_h) {
                        printk(KERN_ERR "%s: pci_sn_h = 0x%08x, should be 0x%08x \n.", dev->name, pci_sn_h, tp->pci_cfg_space.pci_sn_h);
                        rtl8127_csi_write(tp, tp->HwPcieSNOffset + 4, tp->pci_cfg_space.pci_sn_h);
                        tp->esd_flag |= BIT_14;
                }
        }

        if (tp->esd_flag != 0) {
                printk(KERN_ERR "%s: esd_flag = 0x%04x\n.\n", dev->name, tp->esd_flag);
                netif_carrier_off(dev);
                netif_tx_disable(dev);
                rtl8127_hw_reset(dev);
                rtl8127_tx_clear(tp);
                rtl8127_rx_clear(tp);
                rtl8127_init_ring(dev);
                rtl8127_up(dev);
                rtl8127_enable_hw_linkchg_interrupt(tp);
                rtl8127_set_speed(dev, tp->autoneg, tp->speed, tp->duplex, tp->advertising);
                tp->esd_flag = 0;
        }
exit:
        return;
}

/* Cfg9346_Unlock assumed. */
static int rtl8127_try_msi(struct rtl8127_private *tp)
{
        struct pci_dev *pdev = tp->pci_dev;
        unsigned int hw_supp_irq_nvecs;
        unsigned msi = 0;
        int nvecs = 1;

        if (HW_DASH_SUPPORT_DASH(tp))
                hw_supp_irq_nvecs = R8127_MAX_MSIX_VEC_8127AP;
        else
                hw_supp_irq_nvecs = R8127_MAX_MSIX_VEC_8127;
        tp->hw_supp_irq_nvecs = clamp_val(hw_supp_irq_nvecs, 1,
                                          R8127_MAX_MSIX_VEC);

        tp->max_irq_nvecs = tp->hw_supp_irq_nvecs;
        if (HW_DASH_SUPPORT_DASH(tp))
                tp->min_irq_nvecs = R8127_MIN_MSIX_VEC_8127AP;
        else
                tp->min_irq_nvecs = R8127_MIN_MSIX_VEC_8127;
#ifdef DISABLE_MULTI_MSIX_VECTOR
        tp->max_irq_nvecs = 1;
#endif

        if ((nvecs = pci_alloc_irq_vectors(pdev, tp->min_irq_nvecs, tp->max_irq_nvecs, PCI_IRQ_MSIX)) > 0)
                msi |= RTL_FEATURE_MSIX;
        else if ((nvecs = pci_alloc_irq_vectors(pdev, 1, 1, PCI_IRQ_ALL_TYPES)) > 0 &&
                 pci_dev_msi_enabled(pdev))
                msi |= RTL_FEATURE_MSI;
        if (!(msi & (RTL_FEATURE_MSI | RTL_FEATURE_MSIX)))
                dev_info(&pdev->dev, "no MSI/MSI-X. Back to INTx.\n");

        if (!(msi & RTL_FEATURE_MSIX) || nvecs < 1)
                nvecs = 1;

        tp->irq_nvecs = nvecs;

        tp->features |= msi;

        return nvecs;
}

static void rtl8127_disable_msi(struct pci_dev *pdev, struct rtl8127_private *tp)
{
        if (tp->features & (RTL_FEATURE_MSI | RTL_FEATURE_MSIX))
                pci_free_irq_vectors(pdev);
        tp->features &= ~(RTL_FEATURE_MSI | RTL_FEATURE_MSIX);
}

static int rtl8127_get_irq(struct pci_dev *pdev)
{
        return pci_irq_vector(pdev, 0);
}

static void
rtl8127_get_stats64(struct net_device *dev, struct rtnl_link_stats64 *stats)
{
        struct rtl8127_private *tp = netdev_priv(dev);
        struct rtl8127_counters *counters = tp->tally_vaddr;
        dma_addr_t paddr = tp->tally_paddr;

        if (!counters)
                return;

        netdev_stats_to_stats64(stats, &dev->stats);
        dev_fetch_sw_netstats(stats, dev->tstats);

        /*
         * Fetch additional counter values missing in stats collected by driver
         * from tally counters.
         */
        rtl8127_dump_tally_counter(tp, paddr);

        stats->tx_errors = le64_to_cpu(counters->tx_errors);
        stats->collisions = le32_to_cpu(counters->tx_multi_collision);
        stats->tx_aborted_errors = le16_to_cpu(counters->tx_aborted);
        stats->rx_missed_errors = le16_to_cpu(counters->rx_missed);
}

static const struct net_device_ops rtl8127_netdev_ops = {
        .ndo_open       = rtl8127_open,
        .ndo_stop       = rtl8127_close,
        .ndo_get_stats64    = rtl8127_get_stats64,
        .ndo_start_xmit     = rtl8127_start_xmit,
        .ndo_tx_timeout     = rtl8127_tx_timeout,
        .ndo_change_mtu     = rtl8127_change_mtu,
        .ndo_set_mac_address    = rtl8127_set_mac_address,
        .ndo_siocdevprivate = rtl8127_siocdevprivate,
        .ndo_eth_ioctl      = rtl8127_do_ioctl,
        .ndo_set_rx_mode    = rtl8127_set_rx_mode,
        .ndo_fix_features   = rtl8127_fix_features,
        .ndo_set_features   = rtl8127_set_features,
#ifdef CONFIG_NET_POLL_CONTROLLER
        .ndo_poll_controller    = rtl8127_netpoll,
#endif
};



static int rtl8127_poll(napi_ptr napi, napi_budget budget)
{
        struct r8127_napi *r8127napi = RTL_GET_PRIV(napi, struct r8127_napi);
        struct rtl8127_private *tp = r8127napi->priv;
        RTL_GET_NETDEV(tp)
        unsigned int work_to_do = RTL_NAPI_QUOTA(budget, dev);
        unsigned int work_done = 0;
        int i;

        for (i = 0; i < tp->num_tx_rings; i++)
                rtl8127_tx_interrupt(&tp->tx_ring[i], budget);

        for (i = 0; i < tp->num_rx_rings; i++)
                work_done += rtl8127_rx_interrupt(dev, tp, &tp->rx_ring[i], budget);

        work_done = min(work_done, work_to_do);

        RTL_NAPI_QUOTA_UPDATE(dev, work_done, budget);

        if (work_done < work_to_do) {
#ifdef ENABLE_DASH_SUPPORT
                if (rtl8127_check_dash_interrupt(tp))
                        rtl8127_schedule_dash_work(tp);
#endif

                if (RTL_NETIF_RX_COMPLETE(dev, napi, work_done) == FALSE)
                        return RTL_NAPI_RETURN_VALUE;
                /*
                 * 20040426: the barrier is not strictly required but the
                 * behavior of the irq handler could be less predictable
                 * without it. Btw, the lack of flush for the posted pci
                 * write is safe - FR
                 */
                smp_wmb();

                rtl8127_switch_to_timer_interrupt(tp);
        }

        return RTL_NAPI_RETURN_VALUE;
}

static int rtl8127_poll_msix_ring(napi_ptr napi, napi_budget budget)
{
        struct r8127_napi *r8127napi = RTL_GET_PRIV(napi, struct r8127_napi);
        struct rtl8127_private *tp = r8127napi->priv;
        RTL_GET_NETDEV(tp)
        unsigned int work_to_do = RTL_NAPI_QUOTA(budget, dev);
        unsigned int work_done = 0;
        const int message_id = r8127napi->index;

        if (message_id < tp->num_tx_rings)
                rtl8127_tx_interrupt_with_vector(tp, message_id, budget);

        if (message_id < tp->num_rx_rings)
                work_done += rtl8127_rx_interrupt(dev, tp, &tp->rx_ring[message_id], budget);

        RTL_NAPI_QUOTA_UPDATE(dev, work_done, budget);

        if (work_done < work_to_do) {
#ifdef ENABLE_DASH_SUPPORT
                if (message_id == MSIX_ID_V6_L2_MISC_INTR)
                        if (rtl8127_check_dash_interrupt(tp))
                                rtl8127_schedule_dash_work(tp);
#endif

                if (RTL_NETIF_RX_COMPLETE(dev, napi, work_done) == FALSE)
                        return RTL_NAPI_RETURN_VALUE;
                /*
                 * 20040426: the barrier is not strictly required but the
                 * behavior of the irq handler could be less predictable
                 * without it. Btw, the lack of flush for the posted pci
                 * write is safe - FR
                 */
                smp_wmb();

                rtl8127_enable_hw_layered_interrupt(tp, message_id);
        }

        return RTL_NAPI_RETURN_VALUE;
}

static int rtl8127_poll_msix_tx(napi_ptr napi, napi_budget budget)
{
        struct r8127_napi *r8127napi = RTL_GET_PRIV(napi, struct r8127_napi);
        struct rtl8127_private *tp = r8127napi->priv;
        RTL_GET_NETDEV(tp)
        unsigned int work_to_do = RTL_NAPI_QUOTA(budget, dev);
        unsigned int work_done = 0;
        const int message_id = r8127napi->index;

        //suppress unused variable
        (void)(dev);

        rtl8127_tx_interrupt_with_vector(tp, message_id, budget);

        RTL_NAPI_QUOTA_UPDATE(dev, work_done, budget);

        if (work_done < work_to_do) {
                if (RTL_NETIF_RX_COMPLETE(dev, napi, work_done) == FALSE)
                        return RTL_NAPI_RETURN_VALUE;
                /*
                 * 20040426: the barrier is not strictly required but the
                 * behavior of the irq handler could be less predictable
                 * without it. Btw, the lack of flush for the posted pci
                 * write is safe - FR
                 */
                smp_wmb();

                rtl8127_enable_hw_layered_interrupt(tp, message_id);
        }

        return RTL_NAPI_RETURN_VALUE;
}

static int rtl8127_poll_msix_other(napi_ptr napi, napi_budget budget)
{
        struct r8127_napi *r8127napi = RTL_GET_PRIV(napi, struct r8127_napi);
        struct rtl8127_private *tp = r8127napi->priv;
        RTL_GET_NETDEV(tp)
        unsigned int work_to_do = RTL_NAPI_QUOTA(budget, dev);
        const int message_id = r8127napi->index;

        //suppress unused variable
        (void)(dev);
        (void)(work_to_do);

        RTL_NETIF_RX_COMPLETE(dev, napi, work_to_do);

        rtl8127_enable_hw_layered_interrupt(tp, message_id);

        return 1;
}

static int rtl8127_poll_msix_rx(napi_ptr napi, napi_budget budget)
{
        struct r8127_napi *r8127napi = RTL_GET_PRIV(napi, struct r8127_napi);
        struct rtl8127_private *tp = r8127napi->priv;
        RTL_GET_NETDEV(tp)
        unsigned int work_to_do = RTL_NAPI_QUOTA(budget, dev);
        unsigned int work_done = 0;
        const int message_id = r8127napi->index;

        if (message_id < tp->num_rx_rings)
                work_done += rtl8127_rx_interrupt(dev, tp, &tp->rx_ring[message_id], budget);

        RTL_NAPI_QUOTA_UPDATE(dev, work_done, budget);

        if (work_done < work_to_do) {
                if (RTL_NETIF_RX_COMPLETE(dev, napi, work_done) == FALSE)
                        return RTL_NAPI_RETURN_VALUE;
                /*
                 * 20040426: the barrier is not strictly required but the
                 * behavior of the irq handler could be less predictable
                 * without it. Btw, the lack of flush for the posted pci
                 * write is safe - FR
                 */
                smp_wmb();

                rtl8127_enable_hw_layered_interrupt(tp, message_id);
        }

        return RTL_NAPI_RETURN_VALUE;
}

void rtl8127_enable_napi(struct rtl8127_private *tp)
{
        int i;

        for (i = 0; i < tp->irq_nvecs; i++)
                RTL_NAPI_ENABLE(tp->dev, &tp->r8127napi[i].napi);
}

static void rtl8127_disable_napi(struct rtl8127_private *tp)
{
        int i;

        for (i = 0; i < tp->irq_nvecs; i++)
                RTL_NAPI_DISABLE(tp->dev, &tp->r8127napi[i].napi);
}

static void rtl8127_del_napi(struct rtl8127_private *tp)
{
        int i;

        for (i = 0; i < tp->irq_nvecs; i++)
                RTL_NAPI_DEL((&tp->r8127napi[i]));
}

static void rtl8127_init_napi(struct rtl8127_private *tp)
{
        int i;

        for (i=0; i<tp->irq_nvecs; i++) {
                struct r8127_napi *r8127napi = &tp->r8127napi[i];
                int (*poll)(struct napi_struct *, int);

                poll = rtl8127_poll;
                if (tp->features & RTL_FEATURE_MSIX) {
                        switch (tp->HwCurrIsrVer) {
                        case 6:
                                if (i < R8127_MAX_RX_QUEUES_VEC_V4)
                                        poll = rtl8127_poll_msix_rx;
                                else if (i == 8 || i == 9)
                                        poll = rtl8127_poll_msix_tx;
                                else
                                        poll = rtl8127_poll_msix_other;
                                break;
                        case 5:
                                if (i < R8127_MAX_RX_QUEUES_VEC_V3)
                                        poll = rtl8127_poll_msix_rx;
                                else if (i == 16 || i == 17)
                                        poll = rtl8127_poll_msix_tx;
                                else
                                        poll = rtl8127_poll_msix_other;
                                break;
                        case 2:
                                if (i < R8127_MAX_RX_QUEUES_VEC_V3)
                                        poll = rtl8127_poll_msix_rx;
                                else if (i == 16 || i == 18)
                                        poll = rtl8127_poll_msix_tx;
                                else
                                        poll = rtl8127_poll_msix_other;
                                break;
                        case 3:
                        case 4:
                                if (i < R8127_MAX_RX_QUEUES_VEC_V3)
                                        poll = rtl8127_poll_msix_ring;
                                else
                                        poll = rtl8127_poll_msix_other;
                                break;
                        }
                }

                RTL_NAPI_CONFIG(tp->dev, r8127napi, poll, R8127_NAPI_WEIGHT);

                r8127napi->priv = tp;
                r8127napi->index = i;
        }
}

static int
rtl8127_set_real_num_queue(struct rtl8127_private *tp)
{
        int retval = 0;

        retval = netif_set_real_num_tx_queues(tp->dev, tp->num_tx_rings);
        if (retval < 0)
                goto exit;

        retval = netif_set_real_num_rx_queues(tp->dev, tp->num_rx_rings);
        if (retval < 0)
                goto exit;

exit:
        return retval;
}

static int __devinit
rtl8127_init_one(struct pci_dev *pdev,
                 const struct pci_device_id *ent)
{
        struct net_device *dev = NULL;
        struct rtl8127_private *tp;
        void __iomem *ioaddr = NULL;
        static int board_idx = -1;

        int rc;

        assert(pdev != NULL);
        assert(ent != NULL);

        board_idx++;

        if (netif_msg_drv(&debug))
                printk(KERN_INFO "%s Ethernet controller driver %s loaded\n",
                       MODULENAME, RTL8127_VERSION);

        rc = rtl8127_init_board(pdev, &dev, &ioaddr);
        if (rc)
                goto out;

        tp = netdev_priv(dev);
        assert(ioaddr != NULL);

        spin_lock_init(&tp->phy_lock);

        tp->set_speed = rtl8127_set_speed_xmii;
        tp->get_settings = rtl8127_gset_xmii;
        tp->phy_reset_enable = rtl8127_xmii_reset_enable;
        tp->phy_reset_pending = rtl8127_xmii_reset_pending;
        tp->link_ok = rtl8127_xmii_link_ok;

        dev->tstats = devm_netdev_alloc_pcpu_stats(&pdev->dev,
                        struct pcpu_sw_netstats);
        if (!dev->tstats)
                goto err_out_1;

        rc = rtl8127_try_msi(tp);
        if (rc < 0) {
                dev_err(&pdev->dev, "Can't allocate interrupt\n");
                goto err_out_1;
        }

        rtl8127_init_software_variable(dev);

        RTL_NET_DEVICE_OPS(rtl8127_netdev_ops);

        SET_ETHTOOL_OPS(dev, &rtl8127_ethtool_ops);

        dev->watchdog_timeo = RTL8127_TX_TIMEOUT;
        dev->irq = rtl8127_get_irq(pdev);
        dev->base_addr = (unsigned long) ioaddr;

        rtl8127_init_napi(tp);

#ifdef CONFIG_R8127_VLAN
        if (tp->mcfg != CFG_METHOD_DEFAULT) {
                dev->features |= NETIF_F_HW_VLAN_TX | NETIF_F_HW_VLAN_RX;
        }
#endif

        /* There has been a number of reports that using SG/TSO results in
         * tx timeouts. However for a lot of people SG/TSO works fine.
         * Therefore disable both features by default, but allow users to
         * enable them. Use at own risk!
         */
        tp->cp_cmd |= RTL_R16(tp, CPlusCmd);
        if (tp->mcfg != CFG_METHOD_DEFAULT) {
                dev->features |= NETIF_F_IP_CSUM;
                dev->features |= NETIF_F_RXCSUM;
                dev->features |= NETIF_F_SG | NETIF_F_TSO;
                dev->hw_features = NETIF_F_SG | NETIF_F_IP_CSUM | NETIF_F_TSO |
                                   NETIF_F_RXCSUM | NETIF_F_HW_VLAN_TX | NETIF_F_HW_VLAN_RX;
                dev->vlan_features = NETIF_F_SG | NETIF_F_IP_CSUM | NETIF_F_TSO |
                                     NETIF_F_HIGHDMA;
                dev->priv_flags |= IFF_LIVE_ADDR_CHANGE;
                dev->hw_features |= NETIF_F_RXALL;
                dev->hw_features |= NETIF_F_RXFCS;
                dev->hw_features |= NETIF_F_IPV6_CSUM | NETIF_F_TSO6;
                dev->features |= NETIF_F_IPV6_CSUM;
                dev->features |= NETIF_F_TSO6;
                netif_set_tso_max_size(dev, LSO_64K);
                netif_set_tso_max_segs(dev, NIC_MAX_PHYS_BUF_COUNT_LSO2);

#ifdef ENABLE_RSS_SUPPORT
                if (tp->EnableRss) {
                        dev->hw_features |= NETIF_F_RXHASH;
                        dev->features |= NETIF_F_RXHASH;
                }
#endif
        }

        netdev_sw_irq_coalesce_default_on(dev);

#ifdef ENABLE_LIB_SUPPORT
        BLOCKING_INIT_NOTIFIER_HEAD(&tp->lib_nh);
#endif
        rtl8127_init_all_schedule_work(tp);

        rc = rtl8127_set_real_num_queue(tp);
        if (rc < 0)
                goto err_out;

        rtl8127_exit_oob(dev);

        rtl8127_powerup_pll(dev);

        rtl8127_hw_init(dev);

        rtl8127_hw_reset(dev);

        /* Get production from EEPROM */
        rtl8127_eeprom_type(tp);

        if (tp->eeprom_type == EEPROM_TYPE_93C46 || tp->eeprom_type == EEPROM_TYPE_93C56)
                rtl8127_set_eeprom_sel_low(tp);

        rtl8127_get_mac_address(dev);

        tp->tally_vaddr = dma_alloc_coherent(&pdev->dev, sizeof(*tp->tally_vaddr),
                                             &tp->tally_paddr, GFP_KERNEL);
        if (!tp->tally_vaddr) {
                rc = -ENOMEM;
                goto err_out;
        }

        rtl8127_tally_counter_clear(tp);

        pci_set_drvdata(pdev, dev);

        rc = register_netdev(dev);
        if (rc)
                goto err_out;

        printk(KERN_INFO "%s: This product is covered by one or more of the following patents: US6,570,884, US6,115,776, and US6,327,625.\n", MODULENAME);

        rtl8127_disable_rxdvgate(dev);

        device_set_wakeup_enable(&pdev->dev, tp->wol_enabled);

        netif_carrier_off(dev);

        printk("%s", GPL_CLAIM);

out:
        return rc;

err_out:
        if (tp->tally_vaddr != NULL) {
                dma_free_coherent(&pdev->dev, sizeof(*tp->tally_vaddr), tp->tally_vaddr,
                                  tp->tally_paddr);

                tp->tally_vaddr = NULL;
        }
        rtl8127_del_napi(tp);
        rtl8127_disable_msi(pdev, tp);

err_out_1:
        rtl8127_release_board(pdev, dev);

        goto out;
}

static void __devexit
rtl8127_remove_one(struct pci_dev *pdev)
{
        struct net_device *dev = pci_get_drvdata(pdev);
        struct rtl8127_private *tp = netdev_priv(dev);

        assert(dev != NULL);
        assert(tp != NULL);

        set_bit(R8127_FLAG_DOWN, tp->task_flags);

        rtl8127_cancel_all_schedule_work(tp);

        if (HW_DASH_SUPPORT_DASH(tp))
                rtl8127_driver_stop(tp);

        rtl8127_disable_pci_offset_180(tp);

        unregister_netdev(dev);
        rtl8127_del_napi(tp);
        rtl8127_disable_msi(pdev, tp);
        if (tp->tally_vaddr != NULL) {
                dma_free_coherent(&pdev->dev, sizeof(*tp->tally_vaddr), tp->tally_vaddr, tp->tally_paddr);
                tp->tally_vaddr = NULL;
        }

        rtl8127_release_board(pdev, dev);

        pci_set_drvdata(pdev, NULL);
}

#ifdef ENABLE_PAGE_REUSE
static inline unsigned int rtl8127_rx_page_order(unsigned rx_buf_sz, unsigned page_size)
{
        unsigned truesize = SKB_DATA_ALIGN(sizeof(struct skb_shared_info)) +
                            SKB_DATA_ALIGN(rx_buf_sz + R8127_RX_ALIGN);

        return get_order(truesize * 2);
}
#endif //ENABLE_PAGE_REUSE

static void
rtl8127_set_rxbufsize(struct rtl8127_private *tp,
                      struct net_device *dev)
{
        unsigned int mtu = dev->mtu;

        tp->rms = (mtu > ETH_DATA_LEN) ?
                  mtu + ETH_HLEN + RT_VALN_HLEN + ETH_FCS_LEN:
                  RX_BUF_SIZE;
        tp->rx_buf_sz = tp->rms;
#ifdef ENABLE_RX_PACKET_FRAGMENT
        tp->rx_buf_sz =  SKB_DATA_ALIGN(RX_BUF_SIZE);
#endif //ENABLE_RX_PACKET_FRAGMENT
#ifdef ENABLE_PAGE_REUSE
        tp->rx_buf_page_order = rtl8127_rx_page_order(tp->rx_buf_sz, PAGE_SIZE);
        tp->rx_buf_page_size = rtl8127_rx_page_size(tp->rx_buf_page_order);
#endif //ENABLE_PAGE_REUSE
}

static void
rtl8127_set_rms(struct rtl8127_private *tp, u16 rms)
{
        RTL_W16(tp, RxMaxSize, rms | AcceppVlanPhys);
}

static void rtl8127_free_irq(struct rtl8127_private *tp)
{
        int i;

        for (i=0; i<tp->irq_nvecs; i++) {
                struct r8127_irq *irq = &tp->irq_tbl[i];
                struct r8127_napi *r8127napi = &tp->r8127napi[i];

                if (irq->requested) {
                        irq->requested = 0;
                        pci_free_irq(tp->pci_dev, i, r8127napi);
                }
        }
}

static int rtl8127_alloc_irq(struct rtl8127_private *tp)
{
        struct net_device *dev = tp->dev;
        int rc = 0;
        struct r8127_irq *irq;
        struct r8127_napi *r8127napi;
        int i = 0;
        const int len = sizeof(tp->irq_tbl[0].name);

        for (i=0; i<tp->irq_nvecs; i++) {
                irq = &tp->irq_tbl[i];
                if (tp->features & RTL_FEATURE_MSIX &&
                    tp->HwCurrIsrVer > 1)
                        irq->handler = rtl8127_interrupt_msix;
                else
                        irq->handler = rtl8127_interrupt;

                r8127napi = &tp->r8127napi[i];
                snprintf(irq->name, len, "%s-%d", dev->name, i);
                rc = pci_request_irq(tp->pci_dev, i, irq->handler, NULL, r8127napi,
                                     irq->name);
                if (rc)
                        break;

                irq->vector = pci_irq_vector(tp->pci_dev, i);
                irq->requested = 1;
        }
        if (rc)
                rtl8127_free_irq(tp);

        return rc;
}

static int rtl8127_alloc_tx_desc(struct rtl8127_private *tp)
{
        struct rtl8127_tx_ring *ring;
        struct pci_dev *pdev = tp->pci_dev;
        int i;

        for (i = 0; i < tp->num_tx_rings; i++) {
                ring = &tp->tx_ring[i];
                ring->TxDescAllocSize = (ring->num_tx_desc + 1) * sizeof(struct TxDesc);
                ring->TxDescArray = dma_alloc_coherent(&pdev->dev,
                                                       ring->TxDescAllocSize,
                                                       &ring->TxPhyAddr,
                                                       GFP_KERNEL);

                if (!ring->TxDescArray)
                        return -1;
        }

        return 0;
}

static int rtl8127_alloc_rx_desc(struct rtl8127_private *tp)
{
        struct rtl8127_rx_ring *ring;
        struct pci_dev *pdev = tp->pci_dev;
        int i;

        for (i = 0; i < tp->num_rx_rings; i++) {
                ring = &tp->rx_ring[i];
                ring->RxDescAllocSize = (ring->num_rx_desc + 1) * tp->RxDescLength;
                ring->RxDescArray = dma_alloc_coherent(&pdev->dev,
                                                       ring->RxDescAllocSize,
                                                       &ring->RxPhyAddr,
                                                       GFP_KERNEL);

                if (!ring->RxDescArray)
                        return -1;
        }

        return 0;
}

static void rtl8127_free_tx_desc(struct rtl8127_private *tp)
{
        struct rtl8127_tx_ring *ring;
        struct pci_dev *pdev = tp->pci_dev;
        int i;

        for (i = 0; i < tp->num_tx_rings; i++) {
                ring = &tp->tx_ring[i];
                if (ring->TxDescArray) {
                        dma_free_coherent(&pdev->dev,
                                          ring->TxDescAllocSize,
                                          ring->TxDescArray,
                                          ring->TxPhyAddr);
                        ring->TxDescArray = NULL;
                }
        }
}

static void rtl8127_free_rx_desc(struct rtl8127_private *tp)
{
        struct rtl8127_rx_ring *ring;
        struct pci_dev *pdev = tp->pci_dev;
        int i;

        for (i = 0; i < tp->num_rx_rings; i++) {
                ring = &tp->rx_ring[i];
                if (ring->RxDescArray) {
                        dma_free_coherent(&pdev->dev,
                                          ring->RxDescAllocSize,
                                          ring->RxDescArray,
                                          ring->RxPhyAddr);
                        ring->RxDescArray = NULL;
                }
        }
}

static void rtl8127_free_alloc_resources(struct rtl8127_private *tp)
{
        rtl8127_free_rx_desc(tp);

        rtl8127_free_tx_desc(tp);
}

int rtl8127_open(struct net_device *dev)
{
        struct rtl8127_private *tp = netdev_priv(dev);
        int retval;

        retval = -ENOMEM;

        rtl8127_set_rxbufsize(tp, dev);
        /*
         * Rx and Tx descriptors needs 256 bytes alignment.
         * pci_alloc_consistent provides more.
         */
        if (rtl8127_alloc_tx_desc(tp) < 0 || rtl8127_alloc_rx_desc(tp) < 0)
                goto err_free_all_allocated_mem;

        retval = rtl8127_init_ring(dev);
        if (retval < 0)
                goto err_free_all_allocated_mem;

        retval = rtl8127_alloc_irq(tp);
        if (retval < 0)
                goto err_free_all_allocated_mem;

        if (netif_msg_probe(tp)) {
                printk(KERN_INFO "%s: 0x%lx, "
                       "%2.2x:%2.2x:%2.2x:%2.2x:%2.2x:%2.2x, "
                       "IRQ %d\n",
                       dev->name,
                       dev->base_addr,
                       dev->dev_addr[0], dev->dev_addr[1],
                       dev->dev_addr[2], dev->dev_addr[3],
                       dev->dev_addr[4], dev->dev_addr[5], dev->irq);
        }

        pci_set_master(tp->pci_dev);

        rtl8127_enable_napi(tp);

        rtl8127_exit_oob(dev);

        rtl8127_up(dev);

#ifdef ENABLE_PTP_SUPPORT
        if (tp->EnablePtp)
                rtl8127_ptp_init(tp);
#endif
        clear_bit(R8127_FLAG_DOWN, tp->task_flags);

        if (tp->resume_not_chg_speed)
                _rtl8127_check_link_status(dev, R8127_LINK_STATE_UNKNOWN);
        else
                rtl8127_set_speed(dev, tp->autoneg, tp->speed, tp->duplex, tp->advertising);

        if (tp->esd_flag == 0) {
                rtl8127_schedule_esd_work(tp);
        }

        rtl8127_enable_hw_linkchg_interrupt(tp);

out:

        return retval;

err_free_all_allocated_mem:
        rtl8127_free_alloc_resources(tp);

        goto out;
}

static void
_rtl8127_set_l1_l0s_entry_latency(struct rtl8127_private *tp, u8 setting)
{
        u32 csi_tmp;
        u32 temp;

        temp = setting & 0x3f;
        temp <<= 24;
        /*set PCI configuration space offset 0x70F to setting*/
        /*When the register offset of PCI configuration space larger than 0xff, use CSI to access it.*/

        csi_tmp = rtl8127_csi_read(tp, 0x70c) & 0xc0ffffff;
        rtl8127_csi_write(tp, 0x70c, csi_tmp | temp);
}

static void
rtl8127_set_l1_l0s_entry_latency(struct rtl8127_private *tp)
{
        _rtl8127_set_l1_l0s_entry_latency(tp, 0x27);
}

static void
_rtl8127_set_mrrs(struct rtl8127_private *tp, u8 setting)
{
        //Set PCI configuration space offset 0x79 to setting

        struct pci_dev *pdev = tp->pci_dev;
        u8 device_control;

        pci_read_config_byte(pdev, 0x79, &device_control);
        device_control &= ~0x70;
        device_control |= setting;
        pci_write_config_byte(pdev, 0x79, device_control);
}

static void
rtl8127_set_mrrs(struct rtl8127_private *tp)
{
        if (hwoptimize & HW_PATCH_SOC_LAN)
                return;

        _rtl8127_set_mrrs(tp, 0x40);
}

static void
rtl8127_disable_l1_timeout(struct rtl8127_private *tp)
{
        rtl8127_csi_write(tp, 0x890, rtl8127_csi_read(tp, 0x890) & ~BIT(0));
}

void
rtl8127_hw_set_rx_packet_filter(struct net_device *dev)
{
        struct rtl8127_private *tp = netdev_priv(dev);
        u32 mc_filter[2];   /* Multicast hash filter */
        int rx_mode;
        u32 tmp = 0;

        if (dev->flags & IFF_PROMISC) {
                /* Unconditionally log net taps. */
                if (netif_msg_link(tp))
                        printk(KERN_NOTICE "%s: Promiscuous mode enabled.\n",
                               dev->name);

                rx_mode =
                        AcceptBroadcast | AcceptMulticast | AcceptMyPhys |
                        AcceptAllPhys;
                mc_filter[1] = mc_filter[0] = 0xffffffff;
        } else if (dev->flags & IFF_ALLMULTI) {
                /* accept all multicasts. */
                rx_mode = AcceptBroadcast | AcceptMulticast | AcceptMyPhys;
                mc_filter[1] = mc_filter[0] = 0xffffffff;
        } else {
                struct netdev_hw_addr *ha;

                rx_mode = AcceptBroadcast | AcceptMyPhys;
                mc_filter[1] = mc_filter[0] = 0;
                netdev_for_each_mc_addr(ha, dev) {
                        int bit_nr = ether_crc(ETH_ALEN, ha->addr) >> 26;
                        mc_filter[bit_nr >> 5] |= 1 << (bit_nr & 31);
                        rx_mode |= AcceptMulticast;
                }
        }

        if (dev->features & NETIF_F_RXALL)
                rx_mode |= (AcceptErr | AcceptRunt);

        tmp = mc_filter[0];
        mc_filter[0] = swab32(mc_filter[1]);
        mc_filter[1] = swab32(tmp);

        tmp = tp->rtl8127_rx_config | rx_mode | (RTL_R32(tp, RxConfig) & rtl_chip_info[tp->chipset].RxConfigMask);

        RTL_W32(tp, RxConfig, tmp);
        RTL_W32(tp, MAR0 + 0, mc_filter[0]);
        RTL_W32(tp, MAR0 + 4, mc_filter[1]);
}

static void
rtl8127_set_rx_mode(struct net_device *dev)
{
        rtl8127_hw_set_rx_packet_filter(dev);
}

void
rtl8127_set_rx_q_num(struct rtl8127_private *tp,
                     unsigned int num_rx_queues)
{
        u16 q_ctrl;
        u16 rx_q_num;

        rx_q_num = (u16)ilog2(num_rx_queues);
        rx_q_num &= (BIT_0 | BIT_1 | BIT_2);
        rx_q_num <<= 2;
        q_ctrl = RTL_R16(tp, Q_NUM_CTRL_8125);
        q_ctrl &= ~(BIT_2 | BIT_3 | BIT_4);
        q_ctrl |= rx_q_num;
        RTL_W16(tp, Q_NUM_CTRL_8125, q_ctrl);
}

void
rtl8127_set_tx_q_num(struct rtl8127_private *tp,
                     unsigned int num_tx_queues)
{
        u16 mac_ocp_data;

        mac_ocp_data = rtl8127_mac_ocp_read(tp, 0xE63E);
        mac_ocp_data &= ~(BIT_11 | BIT_10);
        mac_ocp_data |= ((ilog2(num_tx_queues) & 0x03) << 10);
        rtl8127_mac_ocp_write(tp, 0xE63E, mac_ocp_data);
}

void
rtl8127_enable_mcu(struct rtl8127_private *tp, bool enable)
{
        if (FALSE == HW_SUPPORT_MAC_MCU(tp))
                return;

        if (enable)
                rtl8127_set_mac_ocp_bit(tp, 0xC0B4, BIT_0);
        else
                rtl8127_clear_mac_ocp_bit(tp, 0xC0B4, BIT_0);
}

static void
rtl8127_clear_tcam_entries(struct rtl8127_private *tp)
{
        if (FALSE == HW_SUPPORT_TCAM(tp))
                return;

        rtl8127_set_mac_ocp_bit(tp, 0xEB54, BIT_0);
        udelay(1);
        rtl8127_clear_mac_ocp_bit(tp, 0xEB54, BIT_0);
}

static u8
rtl8127_get_l1off_cap_bits(struct rtl8127_private *tp)
{
        u8 l1offCapBits = 0;

        l1offCapBits = (BIT_0 | BIT_1);
        l1offCapBits |= (BIT_2 | BIT_3);

        return l1offCapBits;
}

void
rtl8127_hw_config(struct net_device *dev)
{
        struct rtl8127_private *tp = netdev_priv(dev);
        struct pci_dev *pdev = tp->pci_dev;
        u16 mac_ocp_data;

        rtl8127_disable_rx_packet_filter(tp);

        rtl8127_hw_reset(dev);

        rtl8127_enable_cfg9346_write(tp);
        rtl8127_enable_force_clkreq(tp, 0);
        rtl8127_enable_aspm_clkreq_lock(tp, 0);

        rtl8127_set_eee_lpi_timer(tp);

        //keep magic packet only
        mac_ocp_data = rtl8127_mac_ocp_read(tp, 0xC0B6);
        mac_ocp_data &= BIT_0;
        rtl8127_mac_ocp_write(tp, 0xC0B6, mac_ocp_data);

        rtl8127_tally_counter_addr_fill(tp);

        rtl8127_enable_extend_tally_couter(tp);

        rtl8127_desc_addr_fill(tp);

        /* Set DMA burst size and Interframe Gap Time */
        RTL_W32(tp, TxConfig, (TX_DMA_BURST_unlimited << TxDMAShift) |
                (InterFrameGap << TxInterFrameGapShift));

        if (tp->EnableTxNoClose)
                RTL_W32(tp, TxConfig, (RTL_R32(tp, TxConfig) | BIT_6));

        if (enable_double_vlan)
                rtl8127_enable_double_vlan(tp);
        else
                rtl8127_disable_double_vlan(tp);

        rtl8127_set_l1_l0s_entry_latency(tp);

        rtl8127_set_mrrs(tp);

        rtl8127_disable_l1_timeout(tp);

#ifdef ENABLE_RSS_SUPPORT
        rtl8127_config_rss(tp);
#else
        RTL_W32(tp, RSS_CTRL_8125, 0x00);
#endif
        rtl8127_set_rx_q_num(tp, rtl8127_tot_rx_rings(tp));

        RTL_W8(tp, Config1, RTL_R8(tp, Config1) & ~0x10);

        rtl8127_mac_ocp_write(tp, 0xC140, 0xFFFF);
        rtl8127_mac_ocp_write(tp, 0xC142, 0xFFFF);

        //new tx desc format
        mac_ocp_data = rtl8127_mac_ocp_read(tp, 0xEB58);
        mac_ocp_data &= ~(BIT_0 | BIT_1);
        mac_ocp_data |= (BIT_0);
        rtl8127_mac_ocp_write(tp, 0xEB58, mac_ocp_data);

        if (tp->EnableTxNoClose)
                RTL_W8(tp, 0x20E4, RTL_R8(tp, 0x20E4) | BIT_2);
        else
                RTL_W8(tp, 0x20E4, RTL_R8(tp, 0x20E4) & ~BIT_2);

        if (tp->HwSuppRxDescType == RX_DESC_RING_TYPE_4) {
                if (tp->InitRxDescType == RX_DESC_RING_TYPE_4)
                        RTL_W8(tp, 0xd8, RTL_R8(tp, 0xd8) |
                               EnableRxDescV4_0);
                else
                        RTL_W8(tp, 0xd8, RTL_R8(tp, 0xd8) &
                               ~EnableRxDescV4_0);
        }

        if (tp->mcfg == CFG_METHOD_2) {
                rtl8127_clear_mac_ocp_bit(tp, 0xE00C, BIT_12);

                rtl8127_clear_mac_ocp_bit(tp, 0xC0C2, BIT_6);
        }

        mac_ocp_data = rtl8127_mac_ocp_read(tp, 0xE614);
        mac_ocp_data &= ~(BIT_11 | BIT_10 | BIT_9 | BIT_8);
        mac_ocp_data |= (15 << 8);
        rtl8127_mac_ocp_write(tp, 0xE614, mac_ocp_data);

        rtl8127_set_tx_q_num(tp, rtl8127_tot_tx_rings(tp));

        mac_ocp_data = rtl8127_mac_ocp_read(tp, 0xE63E);
        mac_ocp_data &= ~(BIT_5 | BIT_4);
        mac_ocp_data |= ((0x02 & 0x03) << 4);
        rtl8127_mac_ocp_write(tp, 0xE63E, mac_ocp_data);

        rtl8127_enable_mcu(tp, 0);
        rtl8127_enable_mcu(tp, 1);

        mac_ocp_data = rtl8127_mac_ocp_read(tp, 0xC0B4);
        mac_ocp_data |= (BIT_3 | BIT_2);
        rtl8127_mac_ocp_write(tp, 0xC0B4, mac_ocp_data);

        mac_ocp_data = rtl8127_mac_ocp_read(tp, 0xEB6A);
        mac_ocp_data &= ~(BIT_7 | BIT_6 | BIT_5 | BIT_4 | BIT_3 | BIT_2 | BIT_1 | BIT_0);
        mac_ocp_data |= (BIT_5 | BIT_4 | BIT_1 | BIT_0);
        rtl8127_mac_ocp_write(tp, 0xEB6A, mac_ocp_data);

        mac_ocp_data = rtl8127_mac_ocp_read(tp, 0xEB50);
        mac_ocp_data &= ~(BIT_9 | BIT_8 | BIT_7 | BIT_6 | BIT_5);
        mac_ocp_data |= (BIT_6);
        rtl8127_mac_ocp_write(tp, 0xEB50, mac_ocp_data);

        mac_ocp_data = rtl8127_mac_ocp_read(tp, 0xE056);
        mac_ocp_data &= ~(BIT_7 | BIT_6 | BIT_5 | BIT_4);
        //mac_ocp_data |= (BIT_4 | BIT_5);
        rtl8127_mac_ocp_write(tp, 0xE056, mac_ocp_data);

        RTL_W8(tp, TDFNR, 0x10);

        mac_ocp_data = rtl8127_mac_ocp_read(tp, 0xE040);
        mac_ocp_data &= ~(BIT_12);
        rtl8127_mac_ocp_write(tp, 0xE040, mac_ocp_data);

        mac_ocp_data = rtl8127_mac_ocp_read(tp, 0xEA1C);
        mac_ocp_data &= ~(BIT_1 | BIT_0);
        mac_ocp_data |= (BIT_0);
        rtl8127_mac_ocp_write(tp, 0xEA1C, mac_ocp_data);

        rtl8127_mac_ocp_write(tp, 0xE0C0, 0x4000);

        rtl8127_set_mac_ocp_bit(tp, 0xE052, (BIT_6 | BIT_5));
        rtl8127_clear_mac_ocp_bit(tp, 0xE052, BIT_3 | BIT_7);

        mac_ocp_data = rtl8127_mac_ocp_read(tp, 0xD430);
        mac_ocp_data &= ~(BIT_11 | BIT_10 | BIT_9 | BIT_8 | BIT_7 | BIT_6 | BIT_5 | BIT_4 | BIT_3 | BIT_2 | BIT_1 | BIT_0);
        mac_ocp_data |= 0x45F;
        rtl8127_mac_ocp_write(tp, 0xD430, mac_ocp_data);

        if (!tp->DASH)
                RTL_W8(tp, 0xD0, RTL_R8(tp, 0xD0) | BIT_6 | BIT_7);
        else
                RTL_W8(tp, 0xD0, RTL_R8(tp, 0xD0) & ~(BIT_6 | BIT_7));

        rtl8127_disable_eee_plus(tp);

        mac_ocp_data = rtl8127_mac_ocp_read(tp, 0xEA1C);
        mac_ocp_data &= ~(BIT_2);
        mac_ocp_data &= ~(BIT_9 | BIT_8);
        rtl8127_mac_ocp_write(tp, 0xEA1C, mac_ocp_data);

        rtl8127_clear_tcam_entries(tp);

        RTL_W16(tp, 0x1880, RTL_R16(tp, 0x1880) & ~(BIT_4 | BIT_5));

        rtl8127_clear_set_mac_ocp_bit(tp, 0xD40C, 0xE038, 0x8020);

        /* csum offload command for RTL8125 */
        tp->tx_tcp_csum_cmd = TxTCPCS_C;
        tp->tx_udp_csum_cmd = TxUDPCS_C;
        tp->tx_ip_csum_cmd = TxIPCS_C;
        tp->tx_ipv6_csum_cmd = TxIPV6F_C;

        /* config interrupt type for RTL8125B */
        if (tp->HwSuppIsrVer > 1)
                rtl8127_hw_set_interrupt_type(tp, tp->HwCurrIsrVer);

        //other hw parameters
        rtl8127_hw_clear_timer_int(dev);

        rtl8127_hw_clear_int_miti(dev);

        if (tp->use_timer_interrupt &&
            (tp->HwCurrIsrVer > 1) &&
            (tp->HwSuppIntMitiVer > 3) &&
            (tp->features & RTL_FEATURE_MSIX)) {
                int i;
                for (i = 0; i < tp->irq_nvecs; i++)
                        rtl8127_hw_set_timer_int(tp, i, timer_count_v2);
        }

        rtl8127_enable_exit_l1_mask(tp);

        rtl8127_mac_ocp_write(tp, 0xE098, 0xC302);

        if (aspm && (tp->org_pci_offset_99 & (BIT_2 | BIT_5 | BIT_6)))
                rtl8127_init_pci_offset_99(tp);
        else
                rtl8127_disable_pci_offset_99(tp);

        if (aspm && (tp->org_pci_offset_180 & rtl8127_get_l1off_cap_bits(tp)))
                rtl8127_init_pci_offset_180(tp);
        else
                rtl8127_disable_pci_offset_180(tp);

        tp->cp_cmd &= ~(EnableBist | Macdbgo_oe | Force_halfdup |
                        Force_rxflow_en | Force_txflow_en | Cxpl_dbg_sel |
                        ASF | Macdbgo_sel);

        rtl8127_hw_set_features(dev, dev->features);
        rtl8127_set_rms(tp, tp->rms);

        rtl8127_disable_rxdvgate(dev);

        if (!tp->pci_cfg_is_read) {
                pci_read_config_byte(pdev, PCI_COMMAND, &tp->pci_cfg_space.cmd);
                pci_read_config_word(pdev, PCI_BASE_ADDRESS_0, &tp->pci_cfg_space.io_base_l);
                pci_read_config_word(pdev, PCI_BASE_ADDRESS_0 + 2, &tp->pci_cfg_space.io_base_h);
                pci_read_config_word(pdev, PCI_BASE_ADDRESS_2, &tp->pci_cfg_space.mem_base_l);
                pci_read_config_word(pdev, PCI_BASE_ADDRESS_2 + 2, &tp->pci_cfg_space.mem_base_h);
                pci_read_config_word(pdev, PCI_BASE_ADDRESS_3, &tp->pci_cfg_space.resv_0x1c_l);
                pci_read_config_word(pdev, PCI_BASE_ADDRESS_3 + 2, &tp->pci_cfg_space.resv_0x1c_h);
                pci_read_config_byte(pdev, PCI_INTERRUPT_LINE, &tp->pci_cfg_space.ilr);
                pci_read_config_word(pdev, PCI_BASE_ADDRESS_4, &tp->pci_cfg_space.resv_0x20_l);
                pci_read_config_word(pdev, PCI_BASE_ADDRESS_4 + 2, &tp->pci_cfg_space.resv_0x20_h);
                pci_read_config_word(pdev, PCI_BASE_ADDRESS_5, &tp->pci_cfg_space.resv_0x24_l);
                pci_read_config_word(pdev, PCI_BASE_ADDRESS_5 + 2, &tp->pci_cfg_space.resv_0x24_h);
                pci_read_config_word(pdev, PCI_SUBSYSTEM_VENDOR_ID, &tp->pci_cfg_space.resv_0x2c_l);
                pci_read_config_word(pdev, PCI_SUBSYSTEM_VENDOR_ID + 2, &tp->pci_cfg_space.resv_0x2c_h);
                if (tp->HwPcieSNOffset > 0) {
                        tp->pci_cfg_space.pci_sn_l = rtl8127_csi_read(tp, tp->HwPcieSNOffset);
                        tp->pci_cfg_space.pci_sn_h = rtl8127_csi_read(tp, tp->HwPcieSNOffset + 4);
                }

                tp->pci_cfg_is_read = 1;
        }

        /* Set Rx packet filter */
        rtl8127_hw_set_rx_packet_filter(dev);

#ifdef ENABLE_DASH_SUPPORT
        rtl8127_check_and_enable_dash_interrupt(tp);
#endif

        rtl8127_set_radm_fifo_prot(tp, 1);

        rtl8127_enable_aspm_clkreq_lock(tp, aspm ? 1 : 0);

        rtl8127_disable_cfg9346_write(tp);

        udelay(10);
}

void
rtl8127_hw_start(struct net_device *dev)
{
        struct rtl8127_private *tp = netdev_priv(dev);

#ifdef ENABLE_LIB_SUPPORT
        rtl8127_init_lib_ring(tp);
#endif

        RTL_W8(tp, ChipCmd, CmdTxEnb | CmdRxEnb);

        rtl8127_enable_hw_interrupt(tp);

        rtl8127_lib_reset_complete(tp);
}

static int
rtl8127_change_mtu(struct net_device *dev,
                   int new_mtu)
{
        struct rtl8127_private *tp = netdev_priv(dev);
        int ret = 0;

        dev->mtu = new_mtu;

        tp->eee.tx_lpi_timer = dev->mtu + ETH_HLEN + 0x20;

        if (!netif_running(dev))
                goto out;

        rtl8127_down(dev);

        rtl8127_set_rxbufsize(tp, dev);

        ret = rtl8127_init_ring(dev);

        if (ret < 0)
                goto err_out;

        rtl8127_enable_napi(tp);

        if (tp->link_ok(dev))
                rtl8127_link_on_patch(dev);
        else
                rtl8127_link_down_patch(dev);
out:
        netdev_update_features(dev);

err_out:
        return ret;
}

static inline void
rtl8127_set_desc_dma_addr(struct rtl8127_private *tp,
                          struct RxDesc *desc,
                          dma_addr_t mapping)
{
        switch (tp->InitRxDescType) {
        case RX_DESC_RING_TYPE_3:
                ((struct RxDescV3 *)desc)->addr = cpu_to_le64(mapping);
                break;
        case RX_DESC_RING_TYPE_4:
                ((struct RxDescV4 *)desc)->addr = cpu_to_le64(mapping);
                break;
        default:
                desc->addr = cpu_to_le64(mapping);
                break;
        }
}

static inline void
rtl8127_mark_to_asic_v1(struct RxDesc *desc,
                        u32 rx_buf_sz)
{
        u32 eor = le32_to_cpu(desc->opts1) & RingEnd;

        WRITE_ONCE(desc->opts1, cpu_to_le32(DescOwn | eor | rx_buf_sz));
}

static inline void
rtl8127_mark_to_asic_v3(struct RxDescV3 *descv3,
                        u32 rx_buf_sz)
{
        u32 eor = le32_to_cpu(descv3->RxDescNormalDDWord4.opts1) & RingEnd;

        WRITE_ONCE(descv3->RxDescNormalDDWord4.opts1, cpu_to_le32(DescOwn | eor | rx_buf_sz));
}

static inline void
rtl8127_mark_to_asic_v4(struct RxDescV4 *descv4,
                        u32 rx_buf_sz)
{
        u32 eor = le32_to_cpu(descv4->RxDescNormalDDWord2.opts1) & RingEnd;

        WRITE_ONCE(descv4->RxDescNormalDDWord2.opts1, cpu_to_le32(DescOwn | eor | rx_buf_sz));
}

void
rtl8127_mark_to_asic(struct rtl8127_private *tp,
                     struct RxDesc *desc,
                     u32 rx_buf_sz)
{
        switch (tp->InitRxDescType) {
        case RX_DESC_RING_TYPE_3:
                rtl8127_mark_to_asic_v3((struct RxDescV3 *)desc, rx_buf_sz);
                break;
        case RX_DESC_RING_TYPE_4:
                rtl8127_mark_to_asic_v4((struct RxDescV4 *)desc, rx_buf_sz);
                break;
        default:
                rtl8127_mark_to_asic_v1(desc, rx_buf_sz);
                break;
        }
}

static inline void
rtl8127_map_to_asic(struct rtl8127_private *tp,
                    struct rtl8127_rx_ring *ring,
                    struct RxDesc *desc,
                    dma_addr_t mapping,
                    u32 rx_buf_sz,
                    const u32 cur_rx)
{
        ring->RxDescPhyAddr[cur_rx] = mapping;
        rtl8127_set_desc_dma_addr(tp, desc, mapping);
        wmb();
        rtl8127_mark_to_asic(tp, desc, rx_buf_sz);
}

#ifdef ENABLE_PAGE_REUSE

static int
rtl8127_alloc_rx_page(struct rtl8127_private *tp, struct rtl8127_rx_ring *ring,
                      struct rtl8127_rx_buffer *rxb)
{
        struct page *page;
        dma_addr_t dma;
        unsigned int order = tp->rx_buf_page_order;

        //get free page
        page = dev_alloc_pages(order);

        if (unlikely(!page))
                return -ENOMEM;

        dma = dma_map_page_attrs(&tp->pci_dev->dev, page, 0,
                                 tp->rx_buf_page_size,
                                 DMA_FROM_DEVICE,
                                 (DMA_ATTR_SKIP_CPU_SYNC | DMA_ATTR_WEAK_ORDERING));

        if (unlikely(dma_mapping_error(&tp->pci_dev->dev, dma))) {
                __free_pages(page, order);
                return -ENOMEM;
        }

        rxb->page = page;
        rxb->data = page_address(page);
        rxb->page_offset = ring->rx_offset;
        rxb->dma = dma;

        //after page alloc, page refcount already = 1

        return 0;
}

static void
rtl8127_free_rx_page(struct rtl8127_private *tp, struct rtl8127_rx_buffer *rxb)
{
        if (!rxb->page)
                return;

        dma_unmap_page_attrs(&tp->pci_dev->dev, rxb->dma,
                             tp->rx_buf_page_size,
                             DMA_FROM_DEVICE,
                             (DMA_ATTR_SKIP_CPU_SYNC | DMA_ATTR_WEAK_ORDERING));
        __free_pages(rxb->page, tp->rx_buf_page_order);
        rxb->page = NULL;
}

static void
_rtl8127_rx_clear(struct rtl8127_private *tp, struct rtl8127_rx_ring *ring)
{
        int i;
        struct rtl8127_rx_buffer *rxb;

        for (i = 0; i < ring->num_rx_desc; i++) {
                rxb = &ring->rx_buffer[i];
                if (rxb->skb) {
                        dev_kfree_skb(rxb->skb);
                        rxb->skb = NULL;
                }
                rtl8127_free_rx_page(tp, rxb);
        }
}

static u32
rtl8127_rx_fill(struct rtl8127_private *tp,
                struct rtl8127_rx_ring *ring,
                struct net_device *dev,
                u32 start,
                u32 end,
                u8 in_intr)
{
        u32 cur;
        struct rtl8127_rx_buffer *rxb;

        for (cur = start; end - cur > 0; cur++) {
                int ret, i = cur % ring->num_rx_desc;

                rxb = &ring->rx_buffer[i];
                if (rxb->page)
                        continue;

                ret = rtl8127_alloc_rx_page(tp, ring, rxb);
                if (ret)
                        break;

                dma_sync_single_range_for_device(tp_to_dev(tp),
                                                 rxb->dma,
                                                 rxb->page_offset,
                                                 tp->rx_buf_sz,
                                                 DMA_FROM_DEVICE);

                rtl8127_map_to_asic(tp, ring,
                                    rtl8127_get_rxdesc(tp, ring->RxDescArray, i),
                                    rxb->dma + rxb->page_offset,
                                    tp->rx_buf_sz, i);
        }
        return cur - start;
}

#else //ENABLE_PAGE_REUSE

static void
rtl8127_free_rx_skb(struct rtl8127_private *tp,
                    struct rtl8127_rx_ring *ring,
                    struct sk_buff **sk_buff,
                    struct RxDesc *desc,
                    const u32 cur_rx)
{
        struct pci_dev *pdev = tp->pci_dev;

        dma_unmap_single(&pdev->dev, ring->RxDescPhyAddr[cur_rx], tp->rx_buf_sz,
                         DMA_FROM_DEVICE);
        dev_kfree_skb(*sk_buff);
        *sk_buff = NULL;
        rtl8127_make_unusable_by_asic(tp, desc);
}

static int
rtl8127_alloc_rx_skb(struct rtl8127_private *tp,
                     struct rtl8127_rx_ring *ring,
                     struct sk_buff **sk_buff,
                     struct RxDesc *desc,
                     int rx_buf_sz,
                     const u32 cur_rx,
                     u8 in_intr)
{
        struct sk_buff *skb;
        dma_addr_t mapping;
        int ret = 0;

        if (in_intr)
                skb = RTL_ALLOC_SKB_INTR(&tp->r8127napi[ring->index].napi, rx_buf_sz + R8127_RX_ALIGN);
        else
                skb = dev_alloc_skb(rx_buf_sz + R8127_RX_ALIGN);

        if (unlikely(!skb))
                goto err_out;

        if (!in_intr || !R8127_USE_NAPI_ALLOC_SKB)
                skb_reserve(skb, R8127_RX_ALIGN);

        mapping = dma_map_single(tp_to_dev(tp), skb->data, rx_buf_sz,
                                 DMA_FROM_DEVICE);
        if (unlikely(dma_mapping_error(tp_to_dev(tp), mapping))) {
                if (unlikely(net_ratelimit()))
                        netif_err(tp, drv, tp->dev, "Failed to map RX DMA!\n");
                goto err_out;
        }

        *sk_buff = skb;
        rtl8127_map_to_asic(tp, ring, desc, mapping, rx_buf_sz, cur_rx);
out:
        return ret;

err_out:
        if (skb)
                dev_kfree_skb(skb);
        ret = -ENOMEM;
        rtl8127_make_unusable_by_asic(tp, desc);
        goto out;
}

static void
_rtl8127_rx_clear(struct rtl8127_private *tp, struct rtl8127_rx_ring *ring)
{
        int i;

        for (i = 0; i < ring->num_rx_desc; i++) {
                if (ring->Rx_skbuff[i]) {
                        rtl8127_free_rx_skb(tp,
                                            ring,
                                            ring->Rx_skbuff + i,
                                            rtl8127_get_rxdesc(tp, ring->RxDescArray, i),
                                            i);
                        ring->Rx_skbuff[i] = NULL;
                }
        }
}

static u32
rtl8127_rx_fill(struct rtl8127_private *tp,
                struct rtl8127_rx_ring *ring,
                struct net_device *dev,
                u32 start,
                u32 end,
                u8 in_intr)
{
        u32 cur;

        for (cur = start; end - cur > 0; cur++) {
                int ret, i = cur % ring->num_rx_desc;

                if (ring->Rx_skbuff[i])
                        continue;

                ret = rtl8127_alloc_rx_skb(tp,
                                           ring,
                                           ring->Rx_skbuff + i,
                                           rtl8127_get_rxdesc(tp, ring->RxDescArray, i),
                                           tp->rx_buf_sz,
                                           i,
                                           in_intr);
                if (ret < 0)
                        break;
        }
        return cur - start;
}

#endif //ENABLE_PAGE_REUSE

void
rtl8127_rx_clear(struct rtl8127_private *tp)
{
        int i;

        for (i = 0; i < tp->num_rx_rings; i++) {
                struct rtl8127_rx_ring *ring = &tp->rx_ring[i];

                _rtl8127_rx_clear(tp, ring);
        }
}

static void
rtl8127_mark_as_last_descriptor_v1(struct RxDesc *desc)
{
        desc->opts1 |= cpu_to_le32(RingEnd);
}

static void
rtl8127_mark_as_last_descriptor_v3(struct RxDescV3 *descv3)
{
        descv3->RxDescNormalDDWord4.opts1 |= cpu_to_le32(RingEnd);
}

static void
rtl8127_mark_as_last_descriptor_v4(struct RxDescV4 *descv4)
{
        descv4->RxDescNormalDDWord2.opts1 |= cpu_to_le32(RingEnd);
}

void
rtl8127_mark_as_last_descriptor(struct rtl8127_private *tp,
                                struct RxDesc *desc)
{
        switch (tp->InitRxDescType) {
        case RX_DESC_RING_TYPE_3:
                rtl8127_mark_as_last_descriptor_v3((struct RxDescV3 *)desc);
                break;
        case RX_DESC_RING_TYPE_4:
                rtl8127_mark_as_last_descriptor_v4((struct RxDescV4 *)desc);
                break;
        default:
                rtl8127_mark_as_last_descriptor_v1(desc);
                break;
        }
}

static void
rtl8127_desc_addr_fill(struct rtl8127_private *tp)
{
        int i;

        for (i = 0; i < tp->num_tx_rings; i++) {
                struct rtl8127_tx_ring *ring = &tp->tx_ring[i];
                RTL_W32(tp, ring->tdsar_reg, ((u64)ring->TxPhyAddr & DMA_BIT_MASK(32)));
                RTL_W32(tp, ring->tdsar_reg + 4, ((u64)ring->TxPhyAddr >> 32));
        }

        for (i = 0; i < tp->num_rx_rings; i++) {
                struct rtl8127_rx_ring *ring = &tp->rx_ring[i];
                RTL_W32(tp, ring->rdsar_reg, ((u64)ring->RxPhyAddr & DMA_BIT_MASK(32)));
                RTL_W32(tp, ring->rdsar_reg + 4, ((u64)ring->RxPhyAddr >> 32));
        }
}

static void
rtl8127_tx_desc_init(struct rtl8127_private *tp)
{
        int i = 0;

        for (i = 0; i < tp->num_tx_rings; i++) {
                struct rtl8127_tx_ring *ring = &tp->tx_ring[i];
                memset(ring->TxDescArray, 0x0, ring->TxDescAllocSize);

                ring->TxDescArray[ring->num_tx_desc - 1].opts1 = cpu_to_le32(RingEnd);
        }
}

static void
rtl8127_rx_desc_init(struct rtl8127_private *tp)
{
        int i;

        for (i = 0; i < tp->num_rx_rings; i++) {
                struct rtl8127_rx_ring *ring = &tp->rx_ring[i];
                memset(ring->RxDescArray, 0x0, ring->RxDescAllocSize);
        }
}

int
rtl8127_init_ring(struct net_device *dev)
{
        struct rtl8127_private *tp = netdev_priv(dev);
        int i;

        rtl8127_init_ring_indexes(tp);

        rtl8127_tx_desc_init(tp);
        rtl8127_rx_desc_init(tp);

        for (i = 0; i < tp->num_tx_rings; i++) {
                struct rtl8127_tx_ring *ring = &tp->tx_ring[i];
                memset(ring->tx_skb, 0x0, sizeof(ring->tx_skb));
        }

        for (i = 0; i < tp->num_rx_rings; i++) {
                struct rtl8127_rx_ring *ring = &tp->rx_ring[i];
#ifdef ENABLE_PAGE_REUSE
                ring->rx_offset = R8127_RX_ALIGN;
#else
                memset(ring->Rx_skbuff, 0x0, sizeof(ring->Rx_skbuff));
#endif //ENABLE_PAGE_REUSE
                if (rtl8127_rx_fill(tp, ring, dev, 0, ring->num_rx_desc, 0) != ring->num_rx_desc)
                        goto err_out;

                rtl8127_mark_as_last_descriptor(tp, rtl8127_get_rxdesc(tp, ring->RxDescArray, ring->num_rx_desc - 1));
        }

        return 0;

err_out:
        rtl8127_rx_clear(tp);
        return -ENOMEM;
}

static void
rtl8127_unmap_tx_skb(struct pci_dev *pdev,
                     struct ring_info *tx_skb,
                     struct TxDesc *desc)
{
        unsigned int len = tx_skb->len;

        dma_unmap_single(&pdev->dev, le64_to_cpu(desc->addr), len, DMA_TO_DEVICE);

        desc->opts1 = cpu_to_le32(RTK_MAGIC_DEBUG_VALUE);
        desc->opts2 = 0x00;
        desc->addr = RTL8127_MAGIC_NUMBER;
        tx_skb->len = 0;
}

static void
rtl8127_tx_clear_range(struct rtl8127_private *tp,
                       struct rtl8127_tx_ring *ring,
                       u32 start,
                       unsigned int n)
{
        unsigned int i;
        struct net_device *dev = tp->dev;

        for (i = 0; i < n; i++) {
                unsigned int entry = (start + i) % ring->num_tx_desc;
                struct ring_info *tx_skb = ring->tx_skb + entry;
                unsigned int len = tx_skb->len;

                if (len) {
                        struct sk_buff *skb = tx_skb->skb;

                        rtl8127_unmap_tx_skb(tp->pci_dev, tx_skb,
                                             ring->TxDescArray + entry);
                        if (skb) {
                                RTLDEV->stats.tx_dropped++;
                                dev_kfree_skb_any(skb);
                                tx_skb->skb = NULL;
                        }
                }
        }
}

void
rtl8127_tx_clear(struct rtl8127_private *tp)
{
        int i;

        for (i = 0; i < tp->num_tx_rings; i++) {
                struct rtl8127_tx_ring *ring = &tp->tx_ring[i];
                rtl8127_tx_clear_range(tp, ring, ring->dirty_tx, ring->num_tx_desc);
                ring->cur_tx = ring->dirty_tx = 0;
        }
}

static void rtl8127_schedule_reset_work(struct rtl8127_private *tp)
{
        set_bit(R8127_FLAG_TASK_RESET_PENDING, tp->task_flags);
        schedule_delayed_work(&tp->reset_task, 4);
}

static void rtl8127_cancel_schedule_reset_work(struct rtl8127_private *tp)
{
        struct work_struct *work = &tp->reset_task.work;

        if (!work->func)
                return;

        cancel_delayed_work_sync(&tp->reset_task);
}

static void rtl8127_schedule_esd_work(struct rtl8127_private *tp)
{
        set_bit(R8127_FLAG_TASK_ESD_CHECK_PENDING, tp->task_flags);
        schedule_delayed_work(&tp->esd_task, RTL8127_ESD_TIMEOUT);
}

static void rtl8127_cancel_schedule_esd_work(struct rtl8127_private *tp)
{
        struct work_struct *work = &tp->esd_task.work;

        if (!work->func)
                return;

        cancel_delayed_work_sync(&tp->esd_task);
}

static void rtl8127_schedule_linkchg_work(struct rtl8127_private *tp)
{
        set_bit(R8127_FLAG_TASK_LINKCHG_CHECK_PENDING, tp->task_flags);
        schedule_delayed_work(&tp->linkchg_task, 4);
}

static void rtl8127_cancel_schedule_linkchg_work(struct rtl8127_private *tp)
{
        struct work_struct *work = &tp->linkchg_task.work;

        if (!work->func)
                return;

        cancel_delayed_work_sync(&tp->linkchg_task);
}

void rtl8127_schedule_dash_work(struct rtl8127_private *tp)
{
        set_bit(R8127_FLAG_TASK_DASH_CHECK_PENDING, tp->task_flags);
        schedule_delayed_work(&tp->dash_task, RTL8127_DASH_TIMEOUT);
}

static void rtl8127_cancel_schedule_dash_work(struct rtl8127_private *tp)
{
        struct work_struct *work = &tp->dash_task.work;

        if (!work->func)
                return;

        cancel_delayed_work_sync(&tp->dash_task);
}

static void rtl8127_init_all_schedule_work(struct rtl8127_private *tp)
{
        INIT_DELAYED_WORK(&tp->reset_task, rtl8127_reset_task);
        INIT_DELAYED_WORK(&tp->esd_task, rtl8127_esd_task);
        INIT_DELAYED_WORK(&tp->linkchg_task, rtl8127_linkchg_task);
        INIT_DELAYED_WORK(&tp->dash_task, rtl8127_dash_task);
}

static void rtl8127_cancel_all_schedule_work(struct rtl8127_private *tp)
{
        rtl8127_cancel_schedule_reset_work(tp);
        rtl8127_cancel_schedule_esd_work(tp);
        rtl8127_cancel_schedule_linkchg_work(tp);
        rtl8127_cancel_schedule_dash_work(tp);
}

static void
rtl8127_wait_for_irq_complete(struct rtl8127_private *tp)
{
        if (tp->features & RTL_FEATURE_MSIX) {
                int i;
                for (i = 0; i < tp->irq_nvecs; i++)
                        synchronize_irq(tp->irq_tbl[i].vector);
        } else {
                synchronize_irq(tp->dev->irq);
        }
}

void
_rtl8127_wait_for_quiescence(struct net_device *dev)
{
        struct rtl8127_private *tp = netdev_priv(dev);

        /* Wait for any pending NAPI task to complete */
        rtl8127_disable_napi(tp);

        /* Give a racing hard_start_xmit a few cycles to complete. */
        synchronize_net();

        rtl8127_irq_mask_and_ack(tp);

        rtl8127_wait_for_irq_complete(tp);
}

static void
rtl8127_wait_for_quiescence(struct net_device *dev)
{
        struct rtl8127_private *tp = netdev_priv(dev);

        //suppress unused variable
        (void)(tp);

        _rtl8127_wait_for_quiescence(dev);

        rtl8127_enable_napi(tp);
}

static void rtl8127_reset_task(struct work_struct *work)
{
        struct rtl8127_private *tp =
                container_of(work, struct rtl8127_private, reset_task.work);
        struct net_device *dev = tp->dev;
        int i;

        rtnl_lock();

        if (!netif_running(dev) ||
            test_bit(R8127_FLAG_DOWN, tp->task_flags) ||
            !test_and_clear_bit(R8127_FLAG_TASK_RESET_PENDING, tp->task_flags))
                goto out_unlock;

        netdev_err(dev, "Device reseting!\n");

        netif_carrier_off(dev);
        netif_tx_disable(dev);
        _rtl8127_wait_for_quiescence(dev);
        rtl8127_hw_reset(dev);

        rtl8127_tx_clear(tp);

        rtl8127_init_ring_indexes(tp);

        rtl8127_tx_desc_init(tp);
        for (i = 0; i < tp->num_rx_rings; i++) {
                struct rtl8127_rx_ring *ring;
                u32 entry;

                ring = &tp->rx_ring[i];
                for (entry = 0; entry < ring->num_rx_desc; entry++) {
                        struct RxDesc *desc;

                        desc = rtl8127_get_rxdesc(tp, ring->RxDescArray, entry);
                        rtl8127_mark_to_asic(tp, desc, tp->rx_buf_sz);
                }
        }

#ifdef ENABLE_PTP_SUPPORT
        rtl8127_ptp_reset(tp);
#endif

        rtl8127_enable_napi(tp);

        if (tp->resume_not_chg_speed) {
                _rtl8127_check_link_status(dev, R8127_LINK_STATE_UNKNOWN);

                tp->resume_not_chg_speed = 0;
        } else {
                rtl8127_enable_hw_linkchg_interrupt(tp);

                rtl8127_set_speed(dev, tp->autoneg, tp->speed, tp->duplex, tp->advertising);
        }

out_unlock:
        rtnl_unlock();
}

static void rtl8127_esd_task(struct work_struct *work)
{
        struct rtl8127_private *tp =
                container_of(work, struct rtl8127_private, esd_task.work);
        struct net_device *dev = tp->dev;
        rtnl_lock();

        if (!netif_running(dev) ||
            test_bit(R8127_FLAG_DOWN, tp->task_flags) ||
            !test_and_clear_bit(R8127_FLAG_TASK_ESD_CHECK_PENDING, tp->task_flags))
                goto out_unlock;

        rtl8127_esd_checker(tp);

        rtl8127_schedule_esd_work(tp);

out_unlock:
        rtnl_unlock();
}

static void rtl8127_linkchg_task(struct work_struct *work)
{
        struct rtl8127_private *tp =
                container_of(work, struct rtl8127_private, linkchg_task.work);
        struct net_device *dev = tp->dev;
        rtnl_lock();

        if (!netif_running(dev) ||
            test_bit(R8127_FLAG_DOWN, tp->task_flags) ||
            !test_and_clear_bit(R8127_FLAG_TASK_LINKCHG_CHECK_PENDING, tp->task_flags))
                goto out_unlock;

        rtl8127_check_link_status(dev);

out_unlock:
        rtnl_unlock();
}

static void rtl8127_dash_task(struct work_struct *work)
{
        struct rtl8127_private *tp =
                container_of(work, struct rtl8127_private, dash_task.work);
        struct net_device *dev = tp->dev;
        rtnl_lock();

        if (!netif_running(dev) ||
            test_bit(R8127_FLAG_DOWN, tp->task_flags) ||
            !test_and_clear_bit(R8127_FLAG_TASK_DASH_CHECK_PENDING, tp->task_flags))
                goto out_unlock;

#ifdef ENABLE_DASH_SUPPORT
        rtl8127_handle_dash_interrupt(dev);
#endif

out_unlock:
        rtnl_unlock();
}

static void
rtl8127_tx_timeout(struct net_device *dev, unsigned int txqueue)
{
        struct rtl8127_private *tp = netdev_priv(dev);

        netdev_err(dev, "Transmit timeout reset Device!\n");

        /* Let's wait a bit while any (async) irq lands on */
        rtl8127_schedule_reset_work(tp);
}

static u32
rtl8127_get_txd_opts1(struct rtl8127_tx_ring *ring,
                      u32 opts1,
                      u32 len,
                      unsigned int entry)
{
        u32 status = opts1 | len;

        if (entry == ring->num_tx_desc - 1)
                status |= RingEnd;

        return status;
}

static int
rtl8127_xmit_frags(struct rtl8127_private *tp,
                   struct rtl8127_tx_ring *ring,
                   struct sk_buff *skb,
                   const u32 *opts)
{
        struct skb_shared_info *info = skb_shinfo(skb);
        unsigned int cur_frag, entry;
        struct TxDesc *txd = NULL;
        const unsigned char nr_frags = info->nr_frags;
        unsigned long PktLenCnt = 0;
        bool LsoPatchEnabled = FALSE;

        entry = ring->cur_tx;
        for (cur_frag = 0; cur_frag < nr_frags; cur_frag++) {
                skb_frag_t *frag = info->frags + cur_frag;
                dma_addr_t mapping;
                u32 status, len;
                void *addr;

                entry = (entry + 1) % ring->num_tx_desc;

                txd = ring->TxDescArray + entry;
                len = skb_frag_size(frag);
                addr = skb_frag_address(frag);
                mapping = dma_map_single(tp_to_dev(tp), addr, len, DMA_TO_DEVICE);

                if (unlikely(dma_mapping_error(tp_to_dev(tp), mapping))) {
                        if (unlikely(net_ratelimit()))
                                netif_err(tp, drv, tp->dev,
                                          "Failed to map TX fragments DMA!\n");
                        goto err_out;
                }

                /* anti gcc 2.95.3 bugware (sic) */
                status = rtl8127_get_txd_opts1(ring, opts[0], len, entry);
                if (cur_frag == (nr_frags - 1) || LsoPatchEnabled == TRUE)
                        status |= LastFrag;

                txd->addr = cpu_to_le64(mapping);

                ring->tx_skb[entry].len = len;

                txd->opts2 = cpu_to_le32(opts[1]);
                wmb();
                txd->opts1 = cpu_to_le32(status);

                PktLenCnt += len;
        }

        return cur_frag;

err_out:
        rtl8127_tx_clear_range(tp, ring, ring->cur_tx + 1, cur_frag);
        return -EIO;
}

static inline
__be16 get_protocol(struct sk_buff *skb)
{
        return vlan_get_protocol(skb);
}

static inline
u8 rtl8127_get_l4_protocol(struct sk_buff *skb)
{
        int no = skb_network_offset(skb);
        struct ipv6hdr *i6h, _i6h;
        struct iphdr *ih, _ih;
        u8 ip_protocol = IPPROTO_RAW;

        switch (get_protocol(skb)) {
        case  __constant_htons(ETH_P_IP):
                ih = skb_header_pointer(skb, no, sizeof(_ih), &_ih);
                if (ih)
                        ip_protocol = ih->protocol;
                break;
        case  __constant_htons(ETH_P_IPV6):
                i6h = skb_header_pointer(skb, no, sizeof(_i6h), &_i6h);
                if (i6h)
                        ip_protocol = i6h->nexthdr;
                break;
        }

        return ip_protocol;
}

static bool rtl8127_skb_pad_with_len(struct sk_buff *skb, unsigned int len)
{
        if (skb_padto(skb, len))
                return false;
        skb_put(skb, len - skb->len);
        return true;
}

static bool rtl8127_skb_pad(struct sk_buff *skb)
{
        return !eth_skb_pad(skb);
}

/* msdn_giant_send_check()
 * According to the document of microsoft, the TCP Pseudo Header excludes the
 * packet length for IPv6 TCP large packets.
 */
static int msdn_giant_send_check(struct sk_buff *skb)
{
        const struct ipv6hdr *ipv6h;
        struct tcphdr *th;
        int ret;

        ret = skb_cow_head(skb, 0);
        if (ret)
                return ret;

        ipv6h = ipv6_hdr(skb);
        th = tcp_hdr(skb);

        th->check = 0;
        th->check = ~tcp_v6_check(0, &ipv6h->saddr, &ipv6h->daddr, 0);

        return ret;
}

static bool rtl8127_require_pad_ptp_pkt(struct rtl8127_private *tp)
{
        return false;
}

#define MIN_PATCH_LEN (47)
static u32
rtl8127_get_patch_pad_len(struct rtl8127_private *tp,
                          struct sk_buff *skb)
{
        u32 pad_len = 0;
        int trans_data_len;
        u32 hdr_len;
        u32 pkt_len = skb->len;
        u8 ip_protocol;
        bool has_trans = skb_transport_header_was_set(skb);

        if (!rtl8127_require_pad_ptp_pkt(tp))
                goto no_padding;

        if (!(has_trans && (pkt_len < 175))) //128 + MIN_PATCH_LEN
                goto no_padding;

        ip_protocol = rtl8127_get_l4_protocol(skb);
        if (!(ip_protocol == IPPROTO_TCP || ip_protocol == IPPROTO_UDP))
                goto no_padding;

        trans_data_len = pkt_len -
                         (skb->transport_header -
                          skb_headroom(skb));
        if (ip_protocol == IPPROTO_UDP) {
                if (trans_data_len > 3 && trans_data_len < MIN_PATCH_LEN) {
                        u16 dest_port = 0;

                        skb_copy_bits(skb, skb->transport_header - skb_headroom(skb) + 2, &dest_port, 2);
                        dest_port = ntohs(dest_port);

                        if (dest_port == 0x13f ||
                            dest_port == 0x140) {
                                pad_len = MIN_PATCH_LEN - trans_data_len;
                                goto out;
                        }
                }
        }

        hdr_len = 0;
        if (ip_protocol == IPPROTO_TCP)
                hdr_len = 20;
        else if (ip_protocol == IPPROTO_UDP)
                hdr_len = 8;
        if (trans_data_len < hdr_len)
                pad_len = hdr_len - trans_data_len;

out:
        if ((pkt_len + pad_len) < ETH_ZLEN)
                pad_len = ETH_ZLEN - pkt_len;

        return pad_len;

no_padding:

        return 0;
}

static bool
rtl8127_tso_csum(struct sk_buff *skb,
                 struct net_device *dev,
                 u32 *opts,
                 unsigned int *bytecount,
                 unsigned short *gso_segs)
{
        struct rtl8127_private *tp = netdev_priv(dev);
        unsigned long large_send = 0;
        u32 csum_cmd = 0;
        u8 sw_calc_csum = false;
        u8 check_patch_required = true;

        if (dev->features & (NETIF_F_TSO | NETIF_F_TSO6)) {
                u32 mss = skb_shinfo(skb)->gso_size;

                /* TCP Segmentation Offload (or TCP Large Send) */
                if (mss) {
                        union {
                                struct iphdr *v4;
                                struct ipv6hdr *v6;
                                unsigned char *hdr;
                        } ip;
                        union {
                                struct tcphdr *tcp;
                                struct udphdr *udp;
                                unsigned char *hdr;
                        } l4;
                        u32 l4_offset, hdr_len;

                        ip.hdr = skb_network_header(skb);
                        l4.hdr = skb_checksum_start(skb);

                        l4_offset = skb_transport_offset(skb);
                        assert((l4_offset%2) == 0);
                        switch (get_protocol(skb)) {
                        case __constant_htons(ETH_P_IP):
                                if (l4_offset <= GTTCPHO_MAX) {
                                        opts[0] |= GiantSendv4;
                                        opts[0] |= l4_offset << GTTCPHO_SHIFT;
                                        opts[1] |= min(mss, MSS_MAX) << 18;
                                        large_send = 1;
                                }
                                break;
                        case __constant_htons(ETH_P_IPV6):
                                if (msdn_giant_send_check(skb))
                                        return false;
                                if (l4_offset <= GTTCPHO_MAX) {
                                        opts[0] |= GiantSendv6;
                                        opts[0] |= l4_offset << GTTCPHO_SHIFT;
                                        opts[1] |= min(mss, MSS_MAX) << 18;
                                        large_send = 1;
                                }
                                break;
                        default:
                                if (unlikely(net_ratelimit()))
                                        dprintk("tso proto=%x!\n", skb->protocol);
                                break;
                        }

                        if (large_send == 0)
                                return false;


                        /* compute length of segmentation header */
                        hdr_len = (l4.tcp->doff * 4) + l4_offset;
                        /* update gso size and bytecount with header size */
                        *gso_segs = skb_shinfo(skb)->gso_segs;
                        *bytecount += (*gso_segs - 1) * hdr_len;

                        return true;
                }
        }

        if (skb->ip_summed == CHECKSUM_PARTIAL) {
                u8 ip_protocol = IPPROTO_RAW;

                switch (get_protocol(skb)) {
                case  __constant_htons(ETH_P_IP):
                        if (dev->features & NETIF_F_IP_CSUM) {
                                ip_protocol = ip_hdr(skb)->protocol;
                                csum_cmd = tp->tx_ip_csum_cmd;
                        }
                        break;
                case  __constant_htons(ETH_P_IPV6):
                        if (dev->features & NETIF_F_IPV6_CSUM) {
                                if (skb_transport_offset(skb) > 0 && skb_transport_offset(skb) <= TCPHO_MAX) {
                                        ip_protocol = ipv6_hdr(skb)->nexthdr;
                                        csum_cmd = tp->tx_ipv6_csum_cmd;
                                        csum_cmd |= skb_transport_offset(skb) << TCPHO_SHIFT;
                                }
                        }
                        break;
                default:
                        if (unlikely(net_ratelimit()))
                                dprintk("checksum_partial proto=%x!\n", skb->protocol);
                        break;
                }

                if (ip_protocol == IPPROTO_TCP)
                        csum_cmd |= tp->tx_tcp_csum_cmd;
                else if (ip_protocol == IPPROTO_UDP)
                        csum_cmd |= tp->tx_udp_csum_cmd;
                if (csum_cmd == 0) {
                        sw_calc_csum = true;
                        WARN_ON(1); /* we need a WARN() */
                }

                if (ip_protocol == IPPROTO_TCP)
                        check_patch_required = false;
        }

        if (check_patch_required) {
                u32 pad_len = rtl8127_get_patch_pad_len(tp, skb);

                if (pad_len > 0) {
                        if (!rtl8127_skb_pad_with_len(skb, skb->len + pad_len))
                                return false;

                        if (csum_cmd != 0)
                                sw_calc_csum = true;
                }
        }

        if (skb->len < ETH_ZLEN) {
                if (tp->UseSwPaddingShortPkt ||
                    (tp->ShortPacketSwChecksum && csum_cmd != 0)) {
                        if (!rtl8127_skb_pad(skb))
                                return false;

                        if (csum_cmd != 0)
                                sw_calc_csum = true;
                }
        }

        if (sw_calc_csum) {
                skb_checksum_help(skb);
        } else
                opts[1] |= csum_cmd;

        return true;
}

static bool rtl8127_tx_slots_avail(struct rtl8127_private *tp,
                                   struct rtl8127_tx_ring *ring)
{
        unsigned int slots_avail = READ_ONCE(ring->dirty_tx) + ring->num_tx_desc
                                   - READ_ONCE(ring->cur_tx);

        /* A skbuff with nr_frags needs nr_frags+1 entries in the tx queue */
        return slots_avail > MAX_SKB_FRAGS;
}

static inline u32
rtl8127_fast_mod_mask(const u32 input, const u32 mask)
{
        return input > mask ? input & mask : input;
}

static void rtl8127_doorbell(struct rtl8127_private *tp,
                             struct rtl8127_tx_ring *ring)
{
        if (tp->EnableTxNoClose) {
                if (tp->HwSuppTxNoCloseVer > 3)
                        RTL_W32(tp, ring->sw_tail_ptr_reg, ring->cur_tx);
                else
                        RTL_W16(tp, ring->sw_tail_ptr_reg, ring->cur_tx);
        } else {
                /* set polling bit */
                RTL_W32(tp, TPPOLL_8125, BIT(ring->index));
        }
}

static netdev_tx_t
rtl8127_start_xmit(struct sk_buff *skb,
                   struct net_device *dev)
{
        struct rtl8127_private *tp = netdev_priv(dev);
        unsigned int   bytecount;
        unsigned short gso_segs;
        struct ring_info *last;
        unsigned int last_entry;
        unsigned int entry;
        struct TxDesc *txd;
        dma_addr_t mapping;
        u32 len;
        u32 opts[2];
        netdev_tx_t ret = NETDEV_TX_OK;
        int frags;
        u8 EnableTxNoClose = tp->EnableTxNoClose;
        const u16 queue_mapping = skb_get_queue_mapping(skb);
        struct rtl8127_tx_ring *ring;
        bool stop_queue;

        assert(queue_mapping < tp->num_tx_rings);

        ring = &tp->tx_ring[queue_mapping];

        if (unlikely(!rtl8127_tx_slots_avail(tp, ring))) {
                if (netif_msg_drv(tp)) {
                        printk(KERN_ERR
                               "%s: BUG! Tx Ring[%d] full when queue awake!\n",
                               dev->name,
                               queue_mapping);
                }
                goto err_stop;
        }

        entry = ring->cur_tx % ring->num_tx_desc;
        txd = ring->TxDescArray + entry;

        if (!EnableTxNoClose) {
                if (unlikely(le32_to_cpu(txd->opts1) & DescOwn)) {
                        if (netif_msg_drv(tp)) {
                                printk(KERN_ERR
                                       "%s: BUG! Tx Desc is own by hardware!\n",
                                       dev->name);
                        }
                        goto err_stop;
                }
        }

        bytecount = skb->len;
        gso_segs = 1;

        opts[0] = DescOwn;
        opts[1] = rtl8127_tx_vlan_tag(tp, skb);

        if (unlikely(!rtl8127_tso_csum(skb, dev, opts, &bytecount, &gso_segs)))
                goto err_dma_0;

        frags = rtl8127_xmit_frags(tp, ring, skb, opts);
        if (unlikely(frags < 0))
                goto err_dma_0;
        if (frags) {
                len = skb_headlen(skb);
                opts[0] |= FirstFrag;
        } else {
                len = skb->len;
                opts[0] |= FirstFrag | LastFrag;
        }

        opts[0] = rtl8127_get_txd_opts1(ring, opts[0], len, entry);
        mapping = dma_map_single(tp_to_dev(tp), skb->data, len, DMA_TO_DEVICE);
        if (unlikely(dma_mapping_error(tp_to_dev(tp), mapping))) {
                if (unlikely(net_ratelimit()))
                        netif_err(tp, drv, dev, "Failed to map TX DMA!\n");
                goto err_dma_1;
        }

#ifdef ENABLE_PTP_SUPPORT
        if (unlikely(skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP)) {
                if (!test_and_set_bit_lock(__RTL8127_PTP_TX_IN_PROGRESS, &tp->state)) {
                        if (tp->hwtstamp_config.tx_type == HWTSTAMP_TX_ON &&
                            !tp->ptp_tx_skb) {
                                skb_shinfo(skb)->tx_flags |= SKBTX_IN_PROGRESS;

                                tp->ptp_tx_skb = skb_get(skb);
                                tp->ptp_tx_start = jiffies;
                                schedule_work(&tp->ptp_tx_work);
                        } else
                                tp->tx_hwtstamp_skipped++;
                }
        }
#endif
        /* set first fragment's length */
        ring->tx_skb[entry].len = len;

        /* set skb to last fragment */
        last_entry = (entry + frags) % ring->num_tx_desc;
        last = &ring->tx_skb[last_entry];
        last->skb = skb;
        last->gso_segs = gso_segs;
        last->bytecount = bytecount;

        txd->addr = cpu_to_le64(mapping);
        txd->opts2 = cpu_to_le32(opts[1]);
        wmb();
        txd->opts1 = cpu_to_le32(opts[0]);

        netdev_tx_sent_queue(txring_txq(ring), bytecount);

        skb_tx_timestamp(skb);

        /* rtl_tx needs to see descriptor changes before updated tp->cur_tx */
        smp_wmb();

        WRITE_ONCE(ring->cur_tx, ring->cur_tx + frags + 1);

        stop_queue = !rtl8127_tx_slots_avail(tp, ring);
        if (unlikely(stop_queue)) {
                /* Avoid wrongly optimistic queue wake-up: rtl_tx thread must
                 * not miss a ring update when it notices a stopped queue.
                 */
                smp_wmb();
                netif_stop_subqueue(dev, queue_mapping);
        }

        if (netif_xmit_stopped(txring_txq(ring)) || !netdev_xmit_more())
                rtl8127_doorbell(tp, ring);

        if (unlikely(stop_queue)) {
                /* Sync with rtl_tx:
                 * - publish queue status and cur_tx ring index (write barrier)
                 * - refresh dirty_tx ring index (read barrier).
                 * May the current thread have a pessimistic view of the ring
                 * status and forget to wake up queue, a racing rtl_tx thread
                 * can't.
                 */
                smp_mb();
                if (rtl8127_tx_slots_avail(tp, ring))
                        netif_start_subqueue(dev, queue_mapping);
        }
out:
        return ret;
err_dma_1:
        rtl8127_tx_clear_range(tp, ring, ring->cur_tx + 1, frags);
err_dma_0:
        RTLDEV->stats.tx_dropped++;
        dev_kfree_skb_any(skb);
        ret = NETDEV_TX_OK;
        goto out;
err_stop:
        netif_stop_subqueue(dev, queue_mapping);
        ret = NETDEV_TX_BUSY;
        RTLDEV->stats.tx_dropped++;
        goto out;
}

/* recycle tx no close desc*/
static int
rtl8127_tx_interrupt_noclose(struct rtl8127_tx_ring *ring, int budget)
{
        unsigned int total_bytes = 0, total_packets = 0;
        struct rtl8127_private *tp = ring->priv;
        struct net_device *dev = tp->dev;
        unsigned int dirty_tx, tx_left;
        unsigned int tx_desc_closed;
        unsigned int count = 0;

        dirty_tx = ring->dirty_tx;
        ring->NextHwDesCloPtr = rtl8127_get_hw_clo_ptr(ring);
        tx_desc_closed = rtl8127_fast_mod_mask(ring->NextHwDesCloPtr -
                                               ring->BeginHwDesCloPtr,
                                               tp->MaxTxDescPtrMask);
        tx_left = min((READ_ONCE(ring->cur_tx) - dirty_tx), tx_desc_closed);
        ring->BeginHwDesCloPtr += tx_left;

        while (tx_left > 0) {
                unsigned int entry = dirty_tx % ring->num_tx_desc;
                struct ring_info *tx_skb = ring->tx_skb + entry;

                rtl8127_unmap_tx_skb(tp->pci_dev,
                                     tx_skb,
                                     ring->TxDescArray + entry);

                if (tx_skb->skb != NULL) {
                        /* update the statistics for this packet */
                        total_bytes += tx_skb->bytecount;
                        total_packets += tx_skb->gso_segs;

                        RTL_NAPI_CONSUME_SKB_ANY(tx_skb->skb, budget);
                        tx_skb->skb = NULL;
                }
                dirty_tx++;
                tx_left--;
        }

        if (total_packets) {
                netdev_tx_completed_queue(txring_txq(ring),
                                          total_packets, total_bytes);

                RTLDEV->stats.tx_bytes += total_bytes;
                RTLDEV->stats.tx_packets+= total_packets;
        }

        if (ring->dirty_tx != dirty_tx) {
                count = dirty_tx - ring->dirty_tx;
                WRITE_ONCE(ring->dirty_tx, dirty_tx);
                smp_wmb();
                if (__netif_subqueue_stopped(dev, ring->index) &&
                    rtl8127_tx_slots_avail(tp, ring) && netif_carrier_ok(dev)) {
                        netif_start_subqueue(dev, ring->index);
                }
        }

        return count;
}

/* recycle tx close desc*/
static int
rtl8127_tx_interrupt_close(struct rtl8127_tx_ring *ring, int budget)
{
        unsigned int total_bytes = 0, total_packets = 0;
        struct rtl8127_private *tp = ring->priv;
        struct net_device *dev = tp->dev;
        unsigned int dirty_tx, tx_left;
        unsigned int count = 0;

        dirty_tx = ring->dirty_tx;
        tx_left = READ_ONCE(ring->cur_tx) - dirty_tx;

        while (tx_left > 0) {
                unsigned int entry = dirty_tx % ring->num_tx_desc;
                struct ring_info *tx_skb = ring->tx_skb + entry;

                if (le32_to_cpu(READ_ONCE(ring->TxDescArray[entry].opts1)) & DescOwn)
                        break;

                rtl8127_unmap_tx_skb(tp->pci_dev,
                                     tx_skb,
                                     ring->TxDescArray + entry);

                if (tx_skb->skb != NULL) {
                        /* update the statistics for this packet */
                        total_bytes += tx_skb->bytecount;
                        total_packets += tx_skb->gso_segs;

                        RTL_NAPI_CONSUME_SKB_ANY(tx_skb->skb, budget);
                        tx_skb->skb = NULL;
                }
                dirty_tx++;
                tx_left--;
        }

        if (total_packets) {
                netdev_tx_completed_queue(txring_txq(ring),
                                          total_packets, total_bytes);

                RTLDEV->stats.tx_bytes += total_bytes;
                RTLDEV->stats.tx_packets+= total_packets;
        }

        if (ring->dirty_tx != dirty_tx) {
                count = dirty_tx - ring->dirty_tx;
                WRITE_ONCE(ring->dirty_tx, dirty_tx);
                smp_wmb();
                if (__netif_subqueue_stopped(dev, ring->index) &&
                    rtl8127_tx_slots_avail(tp, ring) && netif_carrier_ok(dev)) {
                        netif_start_subqueue(dev, ring->index);
                }

                if (READ_ONCE(ring->cur_tx) != dirty_tx)
                        rtl8127_doorbell(tp, ring);
        }

        return count;
}

static int
rtl8127_tx_interrupt(struct rtl8127_tx_ring *ring, int budget)
{
        struct rtl8127_private *tp = ring->priv;

        if (tp->EnableTxNoClose)
                return rtl8127_tx_interrupt_noclose(ring, budget);
        else
                return rtl8127_tx_interrupt_close(ring, budget);
}

static int
rtl8127_tx_interrupt_with_vector(struct rtl8127_private *tp,
                                 const int message_id,
                                 int budget)
{
        int count = 0;

        switch (tp->HwCurrIsrVer) {
        case 3:
        case 4:
                if (message_id < tp->num_tx_rings)
                        count += rtl8127_tx_interrupt(&tp->tx_ring[message_id], budget);
                break;
        case 5:
                if (message_id == 16)
                        count += rtl8127_tx_interrupt(&tp->tx_ring[0], budget);
#ifdef ENABLE_MULTIPLE_TX_QUEUE
                else if (message_id == 17 && tp->num_tx_rings > 1)
                        count += rtl8127_tx_interrupt(&tp->tx_ring[1], budget);
#endif
                break;
        case 6:
                if (message_id == 8)
                        count += rtl8127_tx_interrupt(&tp->tx_ring[0], budget);
#ifdef ENABLE_MULTIPLE_TX_QUEUE
                else if (message_id == 9 && tp->num_tx_rings > 1)
                        count += rtl8127_tx_interrupt(&tp->tx_ring[1], budget);
#endif
                break;
        default:
                if (message_id == 16)
                        count += rtl8127_tx_interrupt(&tp->tx_ring[0], budget);
#ifdef ENABLE_MULTIPLE_TX_QUEUE
                else if (message_id == 18 && tp->num_tx_rings > 1)
                        count += rtl8127_tx_interrupt(&tp->tx_ring[1], budget);
#endif
                break;
        }

        return count;
}

static inline int
rtl8127_fragmented_frame(struct rtl8127_private *tp, u32 status)
{
        switch (tp->InitRxDescType) {
        case RX_DESC_RING_TYPE_3:
                return (status & (FirstFrag_V3 | LastFrag_V3)) != (FirstFrag_V3 | LastFrag_V3);
        case RX_DESC_RING_TYPE_4:
                return (status & (FirstFrag_V4 | LastFrag_V4)) != (FirstFrag_V4 | LastFrag_V4);
        default:
                return (status & (FirstFrag | LastFrag)) != (FirstFrag | LastFrag);
        }
}

static inline int
rtl8127_is_non_eop(struct rtl8127_private *tp, u32 status)
{
        switch (tp->InitRxDescType) {
        case RX_DESC_RING_TYPE_3:
                return !(status & LastFrag_V3);
        case RX_DESC_RING_TYPE_4:
                return !(status & LastFrag_V4);
        default:
                return !(status & LastFrag);
        }
}

static inline int
rtl8127_rx_desc_type(u32 status)
{
        return ((status >> 26) & 0x0F);
}

static inline void
rtl8127_rx_v1_csum(struct rtl8127_private *tp,
                   struct sk_buff *skb,
                   struct RxDesc *desc)
{
        u32 opts1 = le32_to_cpu(desc->opts1);

        if (((opts1 & RxTCPT) && !(opts1 & RxTCPF)) ||
            ((opts1 & RxUDPT) && !(opts1 & RxUDPF)))
                skb->ip_summed = CHECKSUM_UNNECESSARY;
        else
                skb_checksum_none_assert(skb);
}

static inline void
rtl8127_rx_v3_csum(struct rtl8127_private *tp,
                   struct sk_buff *skb,
                   struct RxDescV3 *descv3)
{
        u32 opts2 = le32_to_cpu(descv3->RxDescNormalDDWord4.opts2);

        /* rx csum offload for RTL8125 */
        if (((opts2 & RxTCPT_v3) && !(opts2 & RxTCPF_v3)) ||
            ((opts2 & RxUDPT_v3) && !(opts2 & RxUDPF_v3)))
                skb->ip_summed = CHECKSUM_UNNECESSARY;
        else
                skb_checksum_none_assert(skb);
}

static inline void
rtl8127_rx_v4_csum(struct rtl8127_private *tp,
                   struct sk_buff *skb,
                   struct RxDescV4 *descv4)
{
        u32 opts1 = le32_to_cpu(descv4->RxDescNormalDDWord2.opts1);

        /* rx csum offload for RTL8125 */
        if (((opts1 & RxTCPT_v4) && !(opts1 & RxTCPF_v4)) ||
            ((opts1 & RxUDPT_v4) && !(opts1 & RxUDPF_v4)))
                skb->ip_summed = CHECKSUM_UNNECESSARY;
        else
                skb_checksum_none_assert(skb);
}

static inline void
rtl8127_rx_csum(struct rtl8127_private *tp,
                struct sk_buff *skb,
                struct RxDesc *desc)
{
        switch (tp->InitRxDescType) {
        case RX_DESC_RING_TYPE_3:
                rtl8127_rx_v3_csum(tp, skb, (struct RxDescV3 *)desc);
                break;
        case RX_DESC_RING_TYPE_4:
                rtl8127_rx_v4_csum(tp, skb, (struct RxDescV4 *)desc);
                break;
        default:
                rtl8127_rx_v1_csum(tp, skb, desc);
                break;
        }
}

static inline void
rtl8127_rx_skb(struct rtl8127_private *tp,
               struct sk_buff *skb,
               u32 ring_index)
{
        napi_gro_receive(&tp->r8127napi[ring_index].napi, skb);
}

static int
rtl8127_check_rx_desc_error(struct net_device *dev,
                            struct rtl8127_private *tp,
                            u32 status)
{
        int ret = 0;

        switch (tp->InitRxDescType) {
        case RX_DESC_RING_TYPE_3:
                if (unlikely(status & RxRES_V3)) {
                        if (status & (RxRWT_V3 | RxRUNT_V3))
                                RTLDEV->stats.rx_length_errors++;
                        if (status & RxCRC_V3)
                                RTLDEV->stats.rx_crc_errors++;

                        ret = -1;
                }
                break;
        case RX_DESC_RING_TYPE_4:
                if (unlikely(status & RxRES_V4)) {
                        if (status & RxRUNT_V4)
                                RTLDEV->stats.rx_length_errors++;
                        if (status & RxCRC_V4)
                                RTLDEV->stats.rx_crc_errors++;

                        ret = -1;
                }
                break;
        default:
                if (unlikely(status & RxRES)) {
                        if (status & (RxRWT | RxRUNT))
                                RTLDEV->stats.rx_length_errors++;
                        if (status & RxCRC)
                                RTLDEV->stats.rx_crc_errors++;

                        ret = -1;
                }
                break;
        }

        return ret;
}

#ifdef ENABLE_PAGE_REUSE

static inline bool
rtl8127_reuse_rx_ok(struct page *page)
{
        /* avoid re-using remote pages */
        if (!dev_page_is_reusable(page)) {
                //printk(KERN_INFO "r8127 page pfmemalloc, can't reuse!\n");
                return false;
        }
        /* if we are only owner of page we can reuse it */
        if (unlikely(page_ref_count(page) != 1)) {
                //printk(KERN_INFO "r8127 page refcnt %d, can't reuse!\n", page_ref_count(page));
                return false;
        }

        return true;
}

static void
rtl8127_reuse_rx_buffer(struct rtl8127_private *tp, struct rtl8127_rx_ring *ring, u32 cur_rx, struct rtl8127_rx_buffer *rxb)
{
        struct page *page = rxb->page;

        u32 dirty_rx = ring->dirty_rx;
        u32 entry = dirty_rx % ring->num_rx_desc;
        struct rtl8127_rx_buffer *nrxb = &ring->rx_buffer[entry];

        u32 noffset;

        //the page gonna be shared by us and kernel, keep page ref = 2
        page_ref_inc(page);

        //flip the buffer in page to use next
        noffset = rxb->page_offset ^ (tp->rx_buf_page_size / 2); //one page, two buffer, ping-pong

        nrxb->dma = rxb->dma;
        nrxb->page_offset = noffset;
        nrxb->data = rxb->data;

        if (cur_rx != dirty_rx) {
                //move the buffer to other slot
                nrxb->page = page;
                rxb->page = NULL;
        }
}

static void rtl8127_put_rx_buffer(struct rtl8127_private *tp,
                                  struct rtl8127_rx_ring *ring,
                                  u32 cur_rx,
                                  struct rtl8127_rx_buffer *rxb)
{
        struct rtl8127_rx_buffer *nrxb;
        struct page *page = rxb->page;
        u32 entry;

        entry = ring->dirty_rx % ring->num_rx_desc;
        nrxb = &ring->rx_buffer[entry];
        if (likely(rtl8127_reuse_rx_ok(page))) {
                /* hand second half of page back to the ring */
                rtl8127_reuse_rx_buffer(tp, ring, cur_rx, rxb);
        } else {
                tp->page_reuse_fail_cnt++;

                dma_unmap_page_attrs(&tp->pci_dev->dev, rxb->dma,
                                     tp->rx_buf_page_size,
                                     DMA_FROM_DEVICE,
                                     (DMA_ATTR_SKIP_CPU_SYNC | DMA_ATTR_WEAK_ORDERING));
                //the page ref is kept 1, uniquely owned by kernel now
                rxb->page = NULL;

                return;
        }

        dma_sync_single_range_for_device(tp_to_dev(tp),
                                         nrxb->dma,
                                         nrxb->page_offset,
                                         tp->rx_buf_sz,
                                         DMA_FROM_DEVICE);

        rtl8127_map_to_asic(tp, ring,
                            rtl8127_get_rxdesc(tp, ring->RxDescArray, entry),
                            nrxb->dma + nrxb->page_offset,
                            tp->rx_buf_sz, entry);

        ring->dirty_rx++;
}

#endif //ENABLE_PAGE_REUSE

static int
rtl8127_rx_interrupt(struct net_device *dev,
                     struct rtl8127_private *tp,
                     struct rtl8127_rx_ring *ring,
                     napi_budget budget)
{
        unsigned int cur_rx, rx_left;
        unsigned int delta, count = 0;
        unsigned int entry;
        struct RxDesc *desc;
        struct sk_buff *skb;
        u32 status;
        u32 rx_quota;
        u32 ring_index = ring->index;
#ifdef ENABLE_PAGE_REUSE
        struct rtl8127_rx_buffer *rxb;
#else //ENABLE_PAGE_REUSE
        u64 rx_buf_phy_addr;
#endif //ENABLE_PAGE_REUSE
        unsigned int total_rx_multicast_packets = 0;
        unsigned int total_rx_bytes = 0, total_rx_packets = 0;

        assert(dev != NULL);
        assert(tp != NULL);

        if (ring->RxDescArray == NULL)
                goto rx_out;

        rx_quota = RTL_RX_QUOTA(budget);
        cur_rx = ring->cur_rx;
        rx_left = ring->num_rx_desc + ring->dirty_rx - cur_rx;
        rx_left = rtl8127_rx_quota(rx_left, (u32)rx_quota);

        for (; rx_left > 0; rx_left--, cur_rx++) {
#ifndef ENABLE_PAGE_REUSE
                const void *rx_buf;
#endif //!ENABLE_PAGE_REUSE
                u32 pkt_size;

                entry = cur_rx % ring->num_rx_desc;
                desc = rtl8127_get_rxdesc(tp, ring->RxDescArray, entry);
                status = le32_to_cpu(rtl8127_rx_desc_opts1(tp, desc));
                if (status & DescOwn) {
                        RTL_R8(tp, tp->imr_reg[0]);
                        status = le32_to_cpu(rtl8127_rx_desc_opts1(tp, desc));
                        if (status & DescOwn)
                                break;
                }

                rmb();

                if (unlikely(rtl8127_check_rx_desc_error(dev, tp, status) < 0)) {
                        if (netif_msg_rx_err(tp)) {
                                printk(KERN_INFO
                                       "%s: Rx ERROR. status = %08x\n",
                                       dev->name, status);
                        }

                        RTLDEV->stats.rx_errors++;

                        if (!(dev->features & NETIF_F_RXALL))
                                goto release_descriptor;
                }
                pkt_size = status & 0x00003fff;
                if (likely(!(dev->features & NETIF_F_RXFCS))) {
#ifdef ENABLE_RX_PACKET_FRAGMENT
                        if (rtl8127_is_non_eop(tp, status) &&
                            pkt_size == tp->rx_buf_sz) {
                                struct RxDesc *desc_next;
                                unsigned int entry_next;
                                int pkt_size_next;
                                u32 status_next;

                                entry_next = (cur_rx + 1) % ring->num_rx_desc;
                                desc_next = rtl8127_get_rxdesc(tp, ring->RxDescArray, entry_next);
                                status_next = le32_to_cpu(rtl8127_rx_desc_opts1(tp, desc_next));
                                if (!(status_next & DescOwn)) {
                                        pkt_size_next = status_next & 0x00003fff;
                                        if (pkt_size_next < ETH_FCS_LEN)
                                                pkt_size -= (ETH_FCS_LEN - pkt_size_next);
                                }
                        }
#endif //ENABLE_RX_PACKET_FRAGMENT
                        if (!rtl8127_is_non_eop(tp, status)) {
                                if (pkt_size < ETH_FCS_LEN) {
#ifdef ENABLE_RX_PACKET_FRAGMENT
                                        pkt_size = 0;
#else
                                        goto drop_packet;
#endif //ENABLE_RX_PACKET_FRAGMENT
                                } else
                                        pkt_size -= ETH_FCS_LEN;
                        }
                }

                if (unlikely(pkt_size > tp->rx_buf_sz))
                        goto drop_packet;

#if !defined(ENABLE_RX_PACKET_FRAGMENT) || !defined(ENABLE_PAGE_REUSE)
                /*
                 * The driver does not support incoming fragmented
                 * frames. They are seen as a symptom of over-mtu
                 * sized frames.
                 */
                if (unlikely(rtl8127_fragmented_frame(tp, status)))
                        goto drop_packet;
#endif //!ENABLE_RX_PACKET_FRAGMENT || !ENABLE_PAGE_REUSE

#ifdef ENABLE_PAGE_REUSE
                rxb = &ring->rx_buffer[entry];
                skb = rxb->skb;
                rxb->skb = NULL;
                if (!skb) {
                        skb = RTL_BUILD_SKB_INTR(rxb->data + rxb->page_offset - ring->rx_offset, tp->rx_buf_page_size / 2);
                        if (!skb) {
                                goto drop_packet;
                        }

                        skb->dev = dev;
                        if (!R8127_USE_NAPI_ALLOC_SKB)
                                skb_reserve(skb, R8127_RX_ALIGN);
                        skb_put(skb, pkt_size);
#ifdef ENABLE_RSS_SUPPORT
                        rtl8127_rx_hash(tp, desc, skb);
#endif
                        rtl8127_rx_csum(tp, skb, desc);
                } else
                        skb_add_rx_frag(skb, skb_shinfo(skb)->nr_frags, rxb->page,
                                        rxb->page_offset, pkt_size, tp->rx_buf_page_size / 2);
                //recycle desc
                rtl8127_put_rx_buffer(tp, ring, cur_rx, rxb);

                dma_sync_single_range_for_cpu(tp_to_dev(tp),
                                              rxb->dma,
                                              rxb->page_offset,
                                              tp->rx_buf_sz,
                                              DMA_FROM_DEVICE);
#else //ENABLE_PAGE_REUSE
                skb = RTL_ALLOC_SKB_INTR(&tp->r8127napi[ring->index].napi, pkt_size + R8127_RX_ALIGN);
                if (!skb) {
                        goto drop_packet;
                }

                skb->dev = dev;
                if (!R8127_USE_NAPI_ALLOC_SKB)
                        skb_reserve(skb, R8127_RX_ALIGN);
                skb_put(skb, pkt_size);

                rx_buf_phy_addr = ring->RxDescPhyAddr[entry];
                dma_sync_single_for_cpu(tp_to_dev(tp),
                                        rx_buf_phy_addr, tp->rx_buf_sz,
                                        DMA_FROM_DEVICE);
                rx_buf = ring->Rx_skbuff[entry]->data;
                prefetch(rx_buf - R8127_RX_ALIGN);
                eth_copy_and_sum(skb, rx_buf, pkt_size, 0);

                dma_sync_single_for_device(tp_to_dev(tp), rx_buf_phy_addr,
                                           tp->rx_buf_sz, DMA_FROM_DEVICE);
#endif //ENABLE_PAGE_REUSE

#ifdef ENABLE_PTP_SUPPORT
                if (tp->flags & RTL_FLAG_RX_HWTSTAMP_ENABLED)
                        rtl8127_rx_ptp_timestamp(tp, skb);
#endif // ENABLE_PTP_SUPPORT

#ifdef ENABLE_RX_PACKET_FRAGMENT
                if (rtl8127_is_non_eop(tp, status)) {
                        unsigned int entry_next;
                        entry_next = (entry + 1) % ring->num_rx_desc;
                        rxb = &ring->rx_buffer[entry_next];
                        rxb->skb = skb;
                        continue;
                }
#endif //ENABLE_RX_PACKET_FRAGMENT

#ifndef ENABLE_PAGE_REUSE
#ifdef ENABLE_RSS_SUPPORT
                rtl8127_rx_hash(tp, desc, skb);
#endif
                rtl8127_rx_csum(tp, skb, desc);
#endif /* !ENABLE_PAGE_REUSE */

                skb->protocol = eth_type_trans(skb, dev);

                total_rx_bytes += skb->len;

                if (skb->pkt_type == PACKET_MULTICAST)
                        total_rx_multicast_packets++;

                if (rtl8127_rx_vlan_skb(tp, desc, skb) < 0)
                        rtl8127_rx_skb(tp, skb, ring_index);

                total_rx_packets++;

#ifdef ENABLE_PAGE_REUSE
                rxb->skb = NULL;
                continue;
#endif

release_descriptor:
                switch (tp->InitRxDescType) {
                case RX_DESC_RING_TYPE_3:
                case RX_DESC_RING_TYPE_4:
                        rtl8127_set_desc_dma_addr(tp, desc,
                                                  ring->RxDescPhyAddr[entry]);
                        wmb();
                        break;
                }
                rtl8127_mark_to_asic(tp, desc, tp->rx_buf_sz);
                continue;
drop_packet:
                RTLDEV->stats.rx_dropped++;
                RTLDEV->stats.rx_length_errors++;
                goto release_descriptor;
        }

        count = cur_rx - ring->cur_rx;
        ring->cur_rx = cur_rx;

        delta = rtl8127_rx_fill(tp, ring, dev, ring->dirty_rx, ring->cur_rx, 1);
        if (!delta && count && netif_msg_intr(tp))
                printk(KERN_INFO "%s: no Rx buffer allocated\n", dev->name);
        ring->dirty_rx += delta;

        RTLDEV->stats.rx_bytes += total_rx_bytes;
        RTLDEV->stats.rx_packets += total_rx_packets;
        RTLDEV->stats.multicast += total_rx_multicast_packets;

        /*
         * FIXME: until there is periodic timer to try and refill the ring,
         * a temporary shortage may definitely kill the Rx process.
         * - disable the asic to try and avoid an overflow and kick it again
         *   after refill ?
         * - how do others driver handle this condition (Uh oh...).
         */
        if ((ring->dirty_rx + ring->num_rx_desc == ring->cur_rx) && netif_msg_intr(tp))
                printk(KERN_EMERG "%s: Rx buffers exhausted\n", dev->name);

rx_out:
        return total_rx_packets;
}

static bool
rtl8127_linkchg_interrupt(struct rtl8127_private *tp, u32 status)
{
        switch (tp->HwCurrIsrVer) {
        case 2:
        case 3:
                return status & ISRIMR_V2_LINKCHG;
        case 4:
                return status & ISRIMR_V4_LINKCHG;
        case 5:
                return status & ISRIMR_V5_LINKCHG;
        case 6:
                return status & ISRIMR_V6_LINKCHG;
        default:
                return status & LinkChg;
        }
}

static u32
rtl8127_get_linkchg_message_id(struct rtl8127_private *tp)
{
        switch (tp->HwCurrIsrVer) {
        case 4:
        case 6:
                return MSIX_ID_V4_LINKCHG;
        case 5:
                return MSIX_ID_V5_LINKCHG;
        default:
                return MSIX_ID_V2_LINKCHG;
        }
}

/*
 *The interrupt handler does all of the Rx thread work and cleans up after
 *the Tx thread.
 */
static irqreturn_t rtl8127_interrupt(int irq, void *dev_instance)
{
        struct r8127_napi *r8127napi = dev_instance;
        struct rtl8127_private *tp = r8127napi->priv;
        struct net_device *dev = tp->dev;
        u32 status;
        int handled = 0;

        do {
                status = RTL_R32(tp, tp->isr_reg[0]);

                if (!(tp->features & (RTL_FEATURE_MSI | RTL_FEATURE_MSIX))) {
                        /* hotplug/major error/no more work/shared irq */
                        if (!status)
                                break;

                        if (status == 0xFFFFFFFF)
                                break;

                        if (!(status & (tp->intr_mask | tp->timer_intr_mask)))
                                break;
                }

                handled = 1;

                if (!tp->irq_tbl[0].requested)
                        break;
                rtl8127_disable_hw_interrupt(tp);

                RTL_W32(tp, tp->isr_reg[0], status&~RxFIFOOver);

                if (rtl8127_linkchg_interrupt(tp, status))
                        rtl8127_schedule_linkchg_work(tp);

#ifdef ENABLE_DASH_SUPPORT
                if ((status & ISRIMR_V6_LAYER2_INTR_STS) &&
                    rtl8127_check_dash_interrupt(tp))
                        rtl8127_schedule_dash_work(tp);
#endif

                if (status & tp->intr_mask || tp->keep_intr_cnt-- > 0) {
                        if (status & tp->intr_mask)
                                tp->keep_intr_cnt = RTK_KEEP_INTERRUPT_COUNT;

                        if (likely(RTL_NETIF_RX_SCHEDULE_PREP(dev, &tp->r8127napi[0].napi)))
                                __RTL_NETIF_RX_SCHEDULE(dev, &tp->r8127napi[0].napi);
                        else if (netif_msg_intr(tp))
                                printk(KERN_INFO "%s: interrupt %04x in poll\n",
                                       dev->name, status);
                } else {
                        tp->keep_intr_cnt = RTK_KEEP_INTERRUPT_COUNT;
                        rtl8127_switch_to_hw_interrupt(tp);
                }
        } while (false);

        return IRQ_RETVAL(handled);
}

static irqreturn_t rtl8127_interrupt_msix(int irq, void *dev_instance)
{
        struct r8127_napi *r8127napi = dev_instance;
        struct rtl8127_private *tp = r8127napi->priv;
        struct net_device *dev = tp->dev;
        int message_id = r8127napi->index;

        do {
                if (!tp->irq_tbl[message_id].requested)
                        break;
                //link change
                if (message_id == rtl8127_get_linkchg_message_id(tp)) {
                        rtl8127_disable_hw_layered_interrupt(tp, message_id);
                        rtl8127_clear_hw_isr_v2(tp, message_id);
                        rtl8127_schedule_linkchg_work(tp);
                        break;
                }

#ifdef ENABLE_DASH_SUPPORT
                if (message_id == MSIX_ID_V6_L2_MISC_INTR) {
                        if (rtl8127_check_dash_interrupt(tp))
                                rtl8127_disable_hw_layered_interrupt(tp, message_id);
                        rtl8127_clear_hw_isr_v2(tp, message_id);
                        rtl8127_schedule_dash_work(tp);
                        rtl8127_enable_hw_layered_interrupt(tp, message_id);
                        break;
                }
#endif

                if (likely(RTL_NETIF_RX_SCHEDULE_PREP(dev, &r8127napi->napi))) {
                        rtl8127_disable_hw_layered_interrupt(tp, message_id);
                        __RTL_NETIF_RX_SCHEDULE(dev, &r8127napi->napi);
                } else if (netif_msg_intr(tp))
                        printk(KERN_INFO "%s: interrupt message id %d in poll_msix\n",
                               dev->name, message_id);
                rtl8127_clear_hw_isr_v2(tp, message_id);
        } while (false);

        return IRQ_HANDLED;
}

static void rtl8127_down(struct net_device *dev)
{
        struct rtl8127_private *tp = netdev_priv(dev);

        netif_carrier_off(dev);

        netif_tx_disable(dev);

        _rtl8127_wait_for_quiescence(dev);

        rtl8127_hw_reset(dev);

        rtl8127_tx_clear(tp);

        rtl8127_rx_clear(tp);
}

static int rtl8127_resource_freed(struct rtl8127_private *tp)
{
        int i;

        for (i = 0; i < tp->num_tx_rings; i++)
                if (tp->tx_ring[i].TxDescArray)
                        return 0;

        for (i = 0; i < tp->num_rx_rings; i++)
                if (tp->rx_ring[i].RxDescArray)
                        return 0;

        return 1;
}

int rtl8127_close(struct net_device *dev)
{
        struct rtl8127_private *tp = netdev_priv(dev);

        if (!rtl8127_resource_freed(tp)) {
                set_bit(R8127_FLAG_DOWN, tp->task_flags);

                rtl8127_down(dev);

                pci_clear_master(tp->pci_dev);

#ifdef ENABLE_PTP_SUPPORT
                rtl8127_ptp_stop(tp);
#endif
                rtl8127_hw_d3_para(dev);

                rtl8127_powerdown_pll(dev, 0);

                rtl8127_free_irq(tp);

                rtl8127_free_alloc_resources(tp);
        } else {
                rtl8127_hw_d3_para(dev);

                rtl8127_powerdown_pll(dev, 0);
        }

        return 0;
}

static void rtl8127_shutdown(struct pci_dev *pdev)
{
        struct net_device *dev = pci_get_drvdata(pdev);
        struct rtl8127_private *tp = netdev_priv(dev);

        rtnl_lock();

        if (HW_DASH_SUPPORT_DASH(tp))
                rtl8127_driver_stop(tp);

        rtl8127_disable_pci_offset_180(tp);

        if (s5_keep_curr_mac == 0 && tp->random_mac == 0)
                rtl8127_rar_set(tp, tp->org_mac_addr);

        if (s5wol == 0)
                tp->wol_enabled = WOL_DISABLED;

        rtl8127_close(dev);
        rtl8127_disable_msi(pdev, tp);

        rtnl_unlock();

        if (system_state == SYSTEM_POWER_OFF) {
                pci_clear_master(tp->pci_dev);
                pci_wake_from_d3(pdev, tp->wol_enabled);
                pci_set_power_state(pdev, PCI_D3hot);
        }
}

#ifdef CONFIG_PM

static int
rtl8127_suspend(struct device *device)
{
        struct pci_dev *pdev = to_pci_dev(device);
        struct net_device *dev = pci_get_drvdata(pdev);
        struct rtl8127_private *tp = netdev_priv(dev);
        rtnl_lock();

        if (!netif_running(dev))
                goto out;

        set_bit(R8127_FLAG_DOWN, tp->task_flags);

        netif_carrier_off(dev);

        netif_tx_disable(dev);

        netif_device_detach(dev);

#ifdef ENABLE_PTP_SUPPORT
        rtl8127_ptp_suspend(tp);
#endif
        rtl8127_hw_reset(dev);

        pci_clear_master(pdev);

        rtl8127_hw_d3_para(dev);

        rtl8127_powerdown_pll(dev, 1);
out:
        if (HW_DASH_SUPPORT_DASH(tp))
                rtl8127_driver_stop(tp);

        rtnl_unlock();

        pci_disable_device(pdev);

        pci_save_state(pdev);

        pci_prepare_to_sleep(pdev);

        return 0;
}

static int
rtl8127_hw_d3_not_power_off(struct net_device *dev)
{
        return rtl8127_check_hw_phy_mcu_code_ver(dev);
}

static int rtl8127_wait_phy_nway_complete_sleep(struct rtl8127_private *tp)
{
        int i, val;

        for (i = 0; i < 30; i++) {
                val = rtl8127_mdio_read(tp, MII_BMSR) & BMSR_ANEGCOMPLETE;
                if (val)
                        return 0;

                mdelay(100);
        }

        return -1;
}

static int
rtl8127_resume(struct device *device)
{
        struct pci_dev *pdev = to_pci_dev(device);
        struct net_device *dev = pci_get_drvdata(pdev);
        struct rtl8127_private *tp = netdev_priv(dev);
        unsigned long flags;
        u32 err;

        rtnl_lock();

        err = pci_enable_device(pdev);
        if (err) {
                dev_err(&pdev->dev, "Cannot enable PCI device from suspend\n");
                goto out_unlock;
        }
        pci_restore_state(pdev);
        pci_enable_wake(pdev, PCI_D0, 0);

        /* restore last modified mac address */
        rtl8127_rar_set(tp, dev->dev_addr);

        r8127_spin_lock(&tp->phy_lock, flags);

        tp->resume_not_chg_speed = 0;
        if (tp->check_keep_link_speed &&
            //tp->link_ok(dev) &&
            rtl8127_hw_d3_not_power_off(dev) &&
            rtl8127_wait_phy_nway_complete_sleep(tp) == 0)
                tp->resume_not_chg_speed = 1;

        r8127_spin_unlock(&tp->phy_lock, flags);

        if (!netif_running(dev))
                goto out_unlock;

        pci_set_master(pdev);

        rtl8127_exit_oob(dev);

        rtl8127_up(dev);

        clear_bit(R8127_FLAG_DOWN, tp->task_flags);

        rtl8127_schedule_reset_work(tp);

        rtl8127_schedule_esd_work(tp);

out_unlock:
        netif_device_attach(dev);

        rtnl_unlock();

        return err;
}

static struct dev_pm_ops rtl8127_pm_ops = {
        .suspend = rtl8127_suspend,
        .resume = rtl8127_resume,
        .freeze = rtl8127_suspend,
        .thaw = rtl8127_resume,
        .poweroff = rtl8127_suspend,
        .restore = rtl8127_resume,
};

#define RTL8127_PM_OPS	(&rtl8127_pm_ops)

#else /* !CONFIG_PM */

#define RTL8127_PM_OPS	NULL

#endif /* CONFIG_PM */

static struct pci_driver rtl8127_pci_driver = {
        .name       = MODULENAME,
        .id_table   = rtl8127_pci_tbl,
        .probe      = rtl8127_init_one,
        .remove     = __devexit_p(rtl8127_remove_one),
        .shutdown   = rtl8127_shutdown,
#ifdef CONFIG_PM
        .driver.pm	= RTL8127_PM_OPS,
#endif
};

static int __init
rtl8127_init_module(void)
{
        int ret = 0;

        ret = pci_register_driver(&rtl8127_pci_driver);

        return ret;
}

static void __exit
rtl8127_cleanup_module(void)
{
        pci_unregister_driver(&rtl8127_pci_driver);
}

module_init(rtl8127_init_module);
module_exit(rtl8127_cleanup_module);
