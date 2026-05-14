/*
 * Loadable AHCI/SATA HBA driver for Cact kmod.
 * Lives under source/AHCI-for-Cact (outside kernel tree).
 * Build: make  (KERN_ROOT defaults to ../CactKernel-x86_32)
 *        or from kernel: make ahci-module
 *
 * Load (root): modload /lib/ahci.cctk
 *   Manifest binds PCI class 01:06 (SATA AHCI). Probe filters prog_if==0x01.
 *
 * I/O model: interrupt-driven. PxIE enables D2H/PIO/DMA-setup/TFES sources;
 * GHC.IE enables HBA-level interrupt routing via PCI INTx. Each rw_sector()
 * issues exactly one command (serialised by cmd_mutex), then sleeps on
 * cmd_done; the ISR clears port/HBA IS bits and wakes the waiter.
 */

#include <stddef.h>
#include <stdint.h>
#include "ahci.h"
#include "pci_enum.h"
#include "devfs.h"
#include "sync.h"

/* ── kernel entry points (resolved via ksym at load time) ───────── */
extern void     kprint(char* s);
extern void     kprint_hex(uint32_t v);
extern void     klog(int level, const char* msg);
extern void*    kmalloc(uint32_t size);
extern void*    kmalloc_aligned(uint32_t size, uint32_t align);
extern void     kfree_aligned(void* p);
extern void     kfree_heap(void* p);
extern void*    memset(void* s, int c, uint32_t n);
extern void*    memcpy(void* dst, const void* src, uint32_t n);
extern void     itoa(int n, char* str);
extern void     hex_to_ascii(uint32_t n, char str[]);
extern uint32_t pci_read32(uint8_t bus, uint8_t dev, uint8_t fn, uint8_t reg);
extern void     pci_write32(uint8_t bus, uint8_t dev, uint8_t fn, uint8_t reg,
                            uint32_t val);
extern uint8_t  port_byte_in (uint16_t port);
extern void     port_byte_out(uint16_t port, uint8_t val);
extern void     vmm_map(uint32_t* pd, uint32_t va, uint32_t pa, int flags);
extern void     irq_spinlock_init   (irq_spinlock_t* lock);
extern void     irq_spinlock_acquire(irq_spinlock_t* lock);
extern void     irq_spinlock_release(irq_spinlock_t* lock);
extern void     irq_register_handler(unsigned int irq, void (*handler)(void));
extern void     mutex_init  (mutex_t* m);
extern void     mutex_lock  (mutex_t* m);
extern void     mutex_unlock(mutex_t* m);
extern void     sema_init   (semaphore_t* s, int val);
extern void     sema_down   (semaphore_t* s);
extern void     sema_up     (semaphore_t* s);
extern devfs_entry_t* devfs_register  (const char* name, uint32_t flags,
                                       devfs_driver_t* drv, void* drv_priv);
extern int            devfs_unregister(const char* name);
extern int blkdev_register(const char *name, uint32_t max_lba,
                           void (*read_sector)(uint32_t lba, uint8_t *buf),
                           void (*write_sector)(uint32_t lba, uint8_t *buf));
extern void blkdev_unregister(const char *name);

/* PTE flags (memory.h) — duplicated here so the module is independent */
#define PAGE_PRESENT 0x1
#define PAGE_RW      0x2
#define PAGE_PWT     0x8
#define PAGE_PCD     0x10

/* klog levels (kernel.h enum log_level_t) */
#define KLOG_OK    0
#define KLOG_WARN  1
#define KLOG_ERROR 2
#define KLOG_FAIL  3

/* GHC bits */
#define HBA_GHC_HR  (1u << 0)
#define HBA_GHC_IE  (1u << 1)
#define HBA_GHC_AE  (1u << 31)

/* Inline CR3 read — replaces static-inline get_current_pd() from kernel.h */
static inline uint32_t* get_current_pd(void) {
    uint32_t val;
    __asm__ __volatile__("mov %%cr3, %0" : "=r"(val));
    return (uint32_t*)val;
}

/* ── manifest (read by pci_peek_module_manifest at load time) ──── */
const uint8_t cact_pci_class    = 0x01;
const uint8_t cact_pci_subclass = 0x06;

/* ── module-private state ──────────────────────────────────────── */
static hba_mem_t        *abar = 0;
static ahci_port_info_t  ports_info[AHCI_MAX_PORTS];
static int               ahci_ready;
static int               ahci_attached;
static int               devfs_was_registered;

static uint8_t           ahci_irq_line;
static int               ahci_irq_armed;
static uint32_t          saved_pci_cmd_dw;
static uint8_t           ahci_bus, ahci_dev, ahci_fn;

static irq_spinlock_t    ahci_lock;     /* short, for register & flag updates */
static mutex_t           cmd_mutex;     /* sleepable, serialises commands */
static semaphore_t       cmd_done;      /* ISR ups → waiter downs */
static volatile int      cmd_inflight;  /* protected by ahci_lock */
static volatile int      cmd_err;       /* set by ISR, read by waiter */

static hba_cmd_header_t *cmd_headers[AHCI_MAX_PORTS];
static hba_cmd_tbl_t    *cmd_tables [AHCI_MAX_PORTS];
static uint8_t          *fis_bufs   [AHCI_MAX_PORTS];

/* ── PIC mask helpers (8259A) ──────────────────────────────────── */
static void pic_mask_line(unsigned irq) {
    if (irq >= 16) return;
    uint16_t port = irq < 8 ? 0x21u : 0xA1u;
    uint8_t  line = irq < 8 ? (uint8_t)irq : (uint8_t)(irq - 8);
    uint8_t  m    = port_byte_in(port);
    m |= (uint8_t)(1u << line);
    port_byte_out(port, m);
}

static void pic_unmask_line(unsigned irq) {
    if (irq >= 16) return;
    uint16_t port = irq < 8 ? 0x21u : 0xA1u;
    uint8_t  line = irq < 8 ? (uint8_t)irq : (uint8_t)(irq - 8);
    uint8_t  m    = port_byte_in(port);
    m &= (uint8_t)~(1u << line);
    port_byte_out(port, m);
}

/* ~10 us busy-wait with PAUSE hint */
static void ahci_udelay(uint32_t us) {
    for (volatile uint32_t i = 0; i < us * 10; i++)
        __asm__ __volatile__("pause");
}

static int ahci_check_type(hba_port_t *port) {
    uint32_t ssts = port->ssts;
    uint8_t  ipm  = (ssts >> 8) & 0x0F;
    uint8_t  det  = ssts & 0x0F;

    if (det != HBA_PORT_DET_PRESENT || ipm != HBA_PORT_IPM_ACTIVE)
        return AHCI_DEV_NULL;

    switch (port->sig) {
        case AHCI_SIG_ATAPI: return AHCI_DEV_SATAPI;
        case AHCI_SIG_SEMB:  return AHCI_DEV_SEMB;
        case AHCI_SIG_PM:    return AHCI_DEV_PM;
        default:             return AHCI_DEV_SATA;
    }
}

static void ahci_stop_cmd(hba_port_t *port) {
    port->cmd &= ~HBA_PxCMD_ST;
    port->cmd &= ~HBA_PxCMD_FRE;

    for (int i = 0; i < 1000; i++) {
        if (!(port->cmd & HBA_PxCMD_FR) && !(port->cmd & HBA_PxCMD_CR))
            return;
        ahci_udelay(1000);
    }
    kprint("[AHCI] stop_cmd timeout port\n");
}

static void ahci_start_cmd(hba_port_t *port) {
    while (port->cmd & HBA_PxCMD_CR)
        ahci_udelay(100);
    port->cmd |= HBA_PxCMD_FRE;
    port->cmd |= HBA_PxCMD_ST;
}

static int ahci_find_cmdslot(hba_port_t *port) {
    uint32_t slots = (port->sact | port->ci);
    for (int i = 0; i < AHCI_CMD_SLOTS; i++)
        if (!(slots & (1u << i)))
            return i;
    kprint("[AHCI] no free cmd slot\n");
    return -1;
}

static void ahci_port_rebase(int portno) {
    hba_port_t *port = &abar->ports[portno];

    ahci_stop_cmd(port);

    cmd_headers[portno] = (hba_cmd_header_t *)kmalloc_aligned(
        sizeof(hba_cmd_header_t) * AHCI_CMD_SLOTS, 1024);
    if (!cmd_headers[portno]) {
        kprint("[AHCI] alloc cmd_headers failed port=");
        char b[16]; itoa(portno, b); kprint(b); kprint("\n");
        return;
    }
    memset((void *)cmd_headers[portno], 0, sizeof(hba_cmd_header_t) * AHCI_CMD_SLOTS);

    port->clb  = (uint32_t)(uintptr_t)cmd_headers[portno];
    port->clbu = 0;

    fis_bufs[portno] = (uint8_t *)kmalloc_aligned(256, 256);
    if (!fis_bufs[portno]) {
        kprint("[AHCI] alloc fis_buf failed\n");
        return;
    }
    memset(fis_bufs[portno], 0, 256);

    port->fb  = (uint32_t)(uintptr_t)fis_bufs[portno];
    port->fbu = 0;

    cmd_tables[portno] = (hba_cmd_tbl_t *)kmalloc_aligned(
        sizeof(hba_cmd_tbl_t) * AHCI_CMD_SLOTS, 128);
    if (!cmd_tables[portno]) {
        kprint("[AHCI] alloc cmd_tables failed\n");
        return;
    }
    memset((void *)cmd_tables[portno], 0, sizeof(hba_cmd_tbl_t) * AHCI_CMD_SLOTS);

    for (int i = 0; i < AHCI_CMD_SLOTS; i++) {
        cmd_headers[portno][i].prdtl = 8;
        cmd_headers[portno][i].ctba  = (uint32_t)(uintptr_t)&cmd_tables[portno][i];
        cmd_headers[portno][i].ctbau = 0;
    }

    port->serr = port->serr;
    port->is   = (uint32_t)-1;
    /* Per-port IRQ sources: D2H Reg FIS, PIO Setup, DMA Setup, TFES */
    port->ie   = (1u << 0) | (1u << 1) | (1u << 2) | (1u << 30);

    ahci_start_cmd(port);
}

/* ── ISR (called from kernel IRQ stub with interrupts disabled) ──── */
static void ahci_isr(void) {
    if (!abar) return;

    uint32_t is = abar->is;
    if (!is) return;            /* shared INTx — not for us */

    int wake = 0;
    for (int p = 0; p < AHCI_MAX_PORTS; p++) {
        if (!(is & (1u << p))) continue;

        hba_port_t *port = &abar->ports[p];
        uint32_t pis = port->is;

        if (pis & HBA_PxIS_TFES)
            cmd_err = 1;

        port->is = pis;         /* W1C — clear handled bits */
        wake = 1;
    }

    abar->is = is;              /* clear HBA-level IS bits */

    if (wake && cmd_inflight) {
        cmd_inflight = 0;
        sema_up(&cmd_done);
    }
}

static int ahci_wait_cmd_done(hba_port_t *port, int slot, const char *tag) {
    uint32_t mask = (1u << slot);
    for (int i = 0; i < 500000; i++) {
        if (cmd_err) return -1;
        if (!(port->ci & mask)) return 0;
        if (!cmd_inflight) return cmd_err ? -1 : 0;
        ahci_udelay(10);
    }
    kprint("[AHCI] timeout: ");
    kprint((char *)tag);
    kprint("\n");
    cmd_inflight = 0;
    return -1;
}

/* ── synchronous I/O over IRQ wakeup ────────────────────────────── */
static int ahci_rw_sector(int portno, uint32_t lba, uint8_t *buf, int is_write) {
    if (!ahci_ready || !ports_info[portno].active) return -1;

    mutex_lock(&cmd_mutex);

    hba_port_t *port = &abar->ports[portno];

    irq_spinlock_acquire(&ahci_lock);
    port->is = (uint32_t)-1;

    int slot = ahci_find_cmdslot(port);
    if (slot < 0) {
        irq_spinlock_release(&ahci_lock);
        mutex_unlock(&cmd_mutex);
        return -1;
    }

    hba_cmd_header_t *hdr = &cmd_headers[portno][slot];
    hdr->cfis_len = sizeof(fis_reg_h2d_t) / 4;
    hdr->w        = is_write ? 1 : 0;
    hdr->prdtl    = 1;
    hdr->prdbc    = 0;

    hba_cmd_tbl_t *tbl = &cmd_tables[portno][slot];
    memset((void *)tbl, 0, sizeof(hba_cmd_tbl_t));

    tbl->prdt[0].dba  = (uint32_t)(uintptr_t)buf;
    tbl->prdt[0].dbau = 0;
    tbl->prdt[0].dbc  = AHCI_SECTOR_SIZE - 1;
    tbl->prdt[0].dbc |= (1u << 31);   /* Interrupt on Completion */

    fis_reg_h2d_t *fis = (fis_reg_h2d_t *)tbl->cfis;
    memset(fis, 0, sizeof(fis_reg_h2d_t));

    fis->fis_type = FIS_TYPE_REG_H2D;
    fis->c        = 1;
    fis->command  = is_write ? ATA_CMD_WRITE_DMA_EX : ATA_CMD_READ_DMA_EX;
    fis->device   = 1 << 6;          /* LBA mode */

    fis->lba0 = (uint8_t)(lba);
    fis->lba1 = (uint8_t)(lba >> 8);
    fis->lba2 = (uint8_t)(lba >> 16);
    fis->lba3 = (uint8_t)(lba >> 24);
    fis->lba4 = 0;
    fis->lba5 = 0;
    fis->count = 1;

    /* Wait for port to leave BSY/DRQ before issuing (very short busy-wait) */
    for (int i = 0; i < 1000; i++) {
        if (!(port->tfd & (ATA_DEV_BUSY | ATA_DEV_DRQ))) break;
        ahci_udelay(1000);
    }
    if (port->tfd & (ATA_DEV_BUSY | ATA_DEV_DRQ)) {
        irq_spinlock_release(&ahci_lock);
        mutex_unlock(&cmd_mutex);
        return -1;
    }

    cmd_err      = 0;
    cmd_inflight = 1;
    port->ci     = (1u << slot);

    irq_spinlock_release(&ahci_lock);

    /* Block until ISR wakes us */
    int err = ahci_wait_cmd_done(port, slot, "rw");
    mutex_unlock(&cmd_mutex);
    return (err || cmd_err) ? -1 : 0;
}

static int ahci_identify(int portno) {
    hba_port_t *port = &abar->ports[portno];
    uint8_t *id_buf = (uint8_t *)kmalloc_aligned(512, 512);
    if (!id_buf) return -1;
    memset(id_buf, 0, 512);

    mutex_lock(&cmd_mutex);

    irq_spinlock_acquire(&ahci_lock);
    port->is = (uint32_t)-1;

    int slot = ahci_find_cmdslot(port);
    if (slot < 0) {
        irq_spinlock_release(&ahci_lock);
        mutex_unlock(&cmd_mutex);
        kfree_aligned(id_buf);
        return -1;
    }

    hba_cmd_header_t *hdr = &cmd_headers[portno][slot];
    hdr->cfis_len = sizeof(fis_reg_h2d_t) / 4;
    hdr->w        = 0;
    hdr->prdtl    = 1;
    hdr->prdbc    = 0;

    hba_cmd_tbl_t *tbl = &cmd_tables[portno][slot];
    memset((void *)tbl, 0, sizeof(hba_cmd_tbl_t));

    tbl->prdt[0].dba  = (uint32_t)(uintptr_t)id_buf;
    tbl->prdt[0].dbau = 0;
    tbl->prdt[0].dbc  = 511;
    tbl->prdt[0].dbc |= (1u << 31);

    fis_reg_h2d_t *fis = (fis_reg_h2d_t *)tbl->cfis;
    memset(fis, 0, sizeof(fis_reg_h2d_t));
    fis->fis_type = FIS_TYPE_REG_H2D;
    fis->c        = 1;
    fis->command  = ATA_CMD_IDENTIFY;
    fis->device   = 0;

    for (int i = 0; i < 1000; i++) {
        if (!(port->tfd & (ATA_DEV_BUSY | ATA_DEV_DRQ))) break;
        ahci_udelay(1000);
    }

    cmd_err      = 0;
    cmd_inflight = 1;
    port->ci     = (1u << slot);

    irq_spinlock_release(&ahci_lock);
    int err = ahci_wait_cmd_done(port, slot, "identify");
    mutex_unlock(&cmd_mutex);

    if (err) { kfree_aligned(id_buf); return -1; }

    uint16_t *id16 = (uint16_t *)id_buf;
    uint32_t  lba28 = (uint32_t)id16[60] | ((uint32_t)id16[61] << 16);
    ports_info[portno].max_lba = lba28;

    for (int i = 0; i < 20; i++) {
        uint16_t w = id16[27 + i];
        ports_info[portno].model[i * 2]     = (char)(w >> 8);
        ports_info[portno].model[i * 2 + 1] = (char)(w & 0xFF);
    }
    ports_info[portno].model[40] = '\0';
    for (int i = 39; i >= 0 && ports_info[portno].model[i] == ' '; i--)
        ports_info[portno].model[i] = '\0';

    kfree_aligned(id_buf);
    return 0;
}

static int ahci_first_port(void) {
    for (int i = 0; i < AHCI_MAX_PORTS; i++)
        if (ports_info[i].active) return i;
    return -1;
}

static void ahci_probe_ports(void) {
    uint32_t pi = abar->pi;

    for (int i = 0; i < AHCI_MAX_PORTS; i++) {
        if (!(pi & (1u << i))) continue;

        int dt = ahci_check_type(&abar->ports[i]);
        if (dt == AHCI_DEV_SATA) {
            ports_info[i].active   = 1;
            ports_info[i].type     = AHCI_DEV_SATA;
            ports_info[i].port_num = (uint8_t)i;

            ahci_port_rebase(i);
            /* Identify uses cmd_done — must run AFTER the IRQ handler is wired
             * and PIC line is unmasked. Caller must ensure ordering. */
        }
    }
}

static int ahci_init_controller(uint32_t mmio) {
    uint32_t bar_size = 0x2000;

    for (uint32_t off = 0; off < bar_size; off += 0x1000)
        vmm_map(get_current_pd(), mmio + off, mmio + off,
                PAGE_PRESENT | PAGE_RW | PAGE_PCD | PAGE_PWT);

    abar = (hba_mem_t *)(uintptr_t)mmio;

    abar->ghc |= HBA_GHC_AE;
    abar->ghc |= HBA_GHC_HR;
    ahci_udelay(100000);
    abar->ghc &= ~HBA_GHC_HR;

    for (int i = 0; i < 1000; i++) {
        if (!(abar->ghc & HBA_GHC_HR)) break;
        ahci_udelay(1000);
    }
    if (abar->ghc & HBA_GHC_HR) {
        kprint("[AHCI] HBA reset timeout\n");
        return -1;
    }

    abar->ghc |= HBA_GHC_AE;
    abar->is   = (uint32_t)-1;

    memset(ports_info, 0, sizeof(ports_info));
    ahci_probe_ports();           /* port_rebase enables PxIE */

    /* HBA-level IE — must be after PxIE so the first edge isn't lost */
    abar->ghc |= HBA_GHC_IE;

    ahci_ready = 1;
    return 0;
}

/* ── devfs ops ──────────────────────────────────────────────────── */

static int _ahci_devfs_read(void *p, uint32_t off, uint32_t size, char *buf) {
    (void)p;
    uint8_t sector_buf[512] __attribute__((aligned(512)));
    int port = ahci_first_port();
    if (port < 0) return -1;

    uint32_t lba = off / 512, written = 0;
    while (written < size) {
        if (ahci_rw_sector(port, lba, sector_buf, 0) < 0) return -1;
        uint32_t c = 512;
        if (c > size - written) c = size - written;
        memcpy(buf + written, sector_buf, c);
        written += c;
        lba++;
    }
    return (int)written;
}

static int _ahci_devfs_write(void *p, uint32_t off, uint32_t size, char *buf) {
    (void)p;
    uint8_t sector_buf[512] __attribute__((aligned(512)));
    int port = ahci_first_port();
    if (port < 0) return -1;

    uint32_t lba = off / 512, written = 0;
    while (written < size) {
        if (ahci_rw_sector(port, lba, sector_buf, 0) < 0) return -1;
        uint32_t c = 512;
        if (c > size - written) c = size - written;
        memcpy(sector_buf, buf + written, c);
        if (ahci_rw_sector(port, lba, sector_buf, 1) < 0) return -1;
        written += c;
        lba++;
    }
    return (int)written;
}

static int _ahci_devfs_status(void *p, char *buf, uint32_t size) {
    (void)p;
    int port = ahci_first_port();
    if (port < 0) {
        const char *s = "device: sda\ntype: AHCI (no ports)\n";
        uint32_t n = 0;
        while (s[n] && n < size - 1) { buf[n] = s[n]; n++; }
        buf[n] = '\0';
        return (int)n;
    }

    int pos = 0;
    const char *h1 = "device: sda\ntype: AHCI/SATA (IRQ)\nmodel: ";
    while (h1[pos] && (uint32_t)pos < size - 1) { buf[pos] = h1[pos]; pos++; }
    for (int i = 0; ports_info[port].model[i] && (uint32_t)pos < size - 1; i++)
        buf[pos++] = ports_info[port].model[i];
    buf[pos++] = '\n';
    buf[pos]   = '\0';
    return pos;
}

static devfs_driver_t drv_ahci = {
    .read   = _ahci_devfs_read,
    .write  = _ahci_devfs_write,
    .status = _ahci_devfs_status,
};

static int ahci_blk_port = -1;

static void ahci_blk_read_sector(uint32_t lba, uint8_t *buf) {
    if (ahci_blk_port < 0) {
        memset(buf, 0, 512);
        return;
    }
    if (ahci_rw_sector(ahci_blk_port, lba, buf, 0) < 0)
        memset(buf, 0, 512);
}

static void ahci_blk_write_sector(uint32_t lba, uint8_t *buf) {
    if (ahci_blk_port >= 0)
        (void)ahci_rw_sector(ahci_blk_port, lba, buf, 1);
}

/* ── teardown ───────────────────────────────────────────────────── */

static void ahci_detach(void) {
    blkdev_unregister("sda");
    ahci_blk_port = -1;

    if (devfs_was_registered) {
        devfs_unregister("sda");
        devfs_was_registered = 0;
    }

    if (abar) {
        /* Quiesce: disable HBA and per-port IRQs, stop command engines */
        abar->ghc &= ~HBA_GHC_IE;
        for (int i = 0; i < AHCI_MAX_PORTS; i++) {
            if (ports_info[i].active) {
                abar->ports[i].ie = 0;
                abar->ports[i].is = (uint32_t)-1;
                ahci_stop_cmd(&abar->ports[i]);
            }
        }
        abar->is = (uint32_t)-1;
    }

    if (ahci_irq_armed && ahci_irq_line < 16) {
        pic_mask_line(ahci_irq_line);
        irq_register_handler(ahci_irq_line, NULL);
        ahci_irq_armed = 0;
    }

    /* Restore PCI command register if we changed it */
    if (ahci_attached)
        pci_write32(ahci_bus, ahci_dev, ahci_fn, 0x04, saved_pci_cmd_dw);

    for (int i = 0; i < AHCI_MAX_PORTS; i++) {
        if (cmd_headers[i]) { kfree_aligned(cmd_headers[i]); cmd_headers[i] = 0; }
        if (cmd_tables [i]) { kfree_aligned(cmd_tables [i]); cmd_tables [i] = 0; }
        if (fis_bufs   [i]) { kfree_aligned(fis_bufs   [i]); fis_bufs   [i] = 0; }
    }

    abar             = 0;
    ahci_ready       = 0;
    ahci_attached    = 0;
    cmd_inflight     = 0;
    cmd_err          = 0;
    saved_pci_cmd_dw = 0;
    ahci_bus = ahci_dev = ahci_fn = 0;
    memset(ports_info, 0, sizeof(ports_info));
}

/* ── ksym entry points ─────────────────────────────────────────── */

int pci_driver_probe(pci_device_t *pdev) {
    if (!pdev) return -1;

    if (ahci_attached) {
        kprint("[AHCI] additional HBA at bus=");
        kprint_hex(pdev->bus); kprint(" dev="); kprint_hex(pdev->dev);
        kprint(" — already attached, skipping\n");
        return -1;
    }

    if (pdev->prog_if != 0x01) {
        kprint("[AHCI] prog_if!=0x01 (IDE-emulation) — skipping\n");
        return -1;
    }

    irq_spinlock_init(&ahci_lock);
    mutex_init  (&cmd_mutex);
    sema_init   (&cmd_done, 0);
    cmd_inflight = 0;
    cmd_err      = 0;

    kprint("[AHCI] probe bus=");
    kprint_hex(pdev->bus); kprint(" dev="); kprint_hex(pdev->dev);
    kprint(" fn="); kprint_hex(pdev->fn);
    kprint(" irq="); kprint_hex(pdev->irq_line); kprint("\n");

    ahci_bus = pdev->bus;
    ahci_dev = pdev->dev;
    ahci_fn  = pdev->fn;

    uint32_t mmio = 0;
    for (int i = 0; i < 6; i++) {
        if (!pdev->bars[i].is_io && pdev->bars[i].base) {
            mmio = pdev->bars[i].base; break;
        }
    }
    if (!mmio)
        mmio = pci_read32(pdev->bus, pdev->dev, pdev->fn, 0x24) & ~0xFu;
    if (!mmio) {
        klog(KLOG_FAIL, "AHCI (kmod): no MMIO BAR");
        return -1;
    }

    /* Save PCI command register, enable bus master + memory space, leave INTx
     * routing enabled (clear bit 10 == INTx Disable; we WANT INTx). */
    saved_pci_cmd_dw = pci_read32(pdev->bus, pdev->dev, pdev->fn, 0x04);
    uint32_t cmd     = saved_pci_cmd_dw;
    cmd |=  0x06u;            /* MEM | BUS_MASTER */
    cmd &= ~(1u << 10);       /* clear INTx Disable */
    pci_write32(pdev->bus, pdev->dev, pdev->fn, 0x04, cmd);

    /* Wire ISR BEFORE enabling controller IE so a fast device can't fire
     * an IRQ into a NULL handler. */
    ahci_irq_line = pdev->irq_line;
    if (ahci_irq_line < 16) {
        irq_register_handler(ahci_irq_line, ahci_isr);
        pic_unmask_line(ahci_irq_line);
        ahci_irq_armed = 1;
    } else {
        klog(KLOG_FAIL, "AHCI (kmod): no valid PCI IRQ line — refusing");
        ahci_detach();
        return -1;
    }

    if (ahci_init_controller(mmio) < 0) {
        klog(KLOG_FAIL, "AHCI (kmod): controller init failed");
        ahci_detach();
        return -1;
    }

    int port = -1;
    for (int i = 0; i < AHCI_MAX_PORTS; i++) {
        if (!ports_info[i].active) continue;
        if (ahci_identify(i) == 0) { port = i; break; }
    }
    if (port < 0) port = ahci_first_port();
    if (!ahci_ready || port < 0) {
        klog(KLOG_WARN, "AHCI (kmod): no active SATA ports");
        ahci_detach();
        return -1;
    }

    if (!ports_info[port].model[0]) {
        klog(KLOG_WARN, "AHCI (kmod): identify timeout, using fallback geometry");
        ports_info[port].max_lba = 0x0FFFFFFFu;
        ports_info[port].model[0] = 'S';
        ports_info[port].model[1] = 'A';
        ports_info[port].model[2] = 'T';
        ports_info[port].model[3] = 'A';
        ports_info[port].model[4] = ' ';
        ports_info[port].model[5] = 'D';
        ports_info[port].model[6] = 'i';
        ports_info[port].model[7] = 's';
        ports_info[port].model[8] = 'k';
        ports_info[port].model[9] = '\0';
    }

    {
        char tmp[16]; itoa(port, tmp);
        kprint("[AHCI] active SATA port="); kprint(tmp);
        kprint(" model="); kprint(ports_info[port].model);
        kprint("  IRQ="); kprint_hex(ahci_irq_line); kprint("\n");
    }

    ahci_attached = 1;            /* set BEFORE devfs so detach restores PCI cmd */

    if (devfs_register("sda", DEVFS_F_BLOCK, &drv_ahci, 0)) {
        devfs_was_registered = 1;
    } else {
        klog(KLOG_WARN, "AHCI (kmod): devfs_register('sda') failed");
    }

    ahci_blk_port = port;
    if (blkdev_register("sda", ports_info[port].max_lba, ahci_blk_read_sector,
                        ahci_blk_write_sector) != 0)
        klog(KLOG_WARN, "AHCI (kmod): blkdev_register('sda') failed");

    klog(KLOG_OK, "AHCI (kmod): IRQ-driven driver attached — /dev/sda registered");
    return 0;
}

void pci_driver_remove(pci_device_t *dev) {
    (void)dev;
    int had_any = ahci_attached || devfs_was_registered || abar != 0;
    ahci_detach();
    if (had_any)
        klog(KLOG_OK, "AHCI (kmod): unloaded");
}
