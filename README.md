# 🛠 AHCI-for-Cact

<p align="center">
  <img src="https://img.shields.io/badge/version-2.0.0-green.svg?style=for-the-badge" alt="Version: 2.0.0">
  <img src="https://img.shields.io/badge/license-GPLv3-blue.svg?style=for-the-badge" alt="License: GPLv3">
  <img src="https://img.shields.io/badge/arch-i686-red.svg?style=for-the-badge" alt="Arch: i686">
  <img src="https://img.shields.io/badge/format-cctk-green.svg?style=for-the-badge" alt="Output: ahci.cctk">
  <img src="https://img.shields.io/badge/bus-PCI-blue.svg?style=for-the-badge" alt="PCI">
  <img src="https://img.shields.io/badge/irq-MSI--X-brightgreen.svg?style=for-the-badge" alt="MSI-X">
</p>

<p align="center">
  <strong>English.</strong> Out-of-tree <strong>AHCI / SATA HBA</strong> driver packaged as a relocatable <strong><code>ahci.cctk</code></strong> for the Cact kernel module loader.<br>
  <strong>2.0.0:</strong> migrated from PIC to <strong>MSI-X</strong> (removed <code>pic_mask_line</code>/<code>pic_unmask_line</code>, added <code>msix_alloc_vector</code>/<code>msix_register_handler</code>/<code>pci_msix_enable</code>). Falls back to poll mode if MSI-X is unavailable.<br>
  <strong>Русский.</strong> Драйвер <strong>AHCI</strong> вне дерева ядра — модуль <strong><code>ahci.cctk</code></strong> для <strong>kmod</strong>.<br>
  <strong>2.0.0:</strong> переведён с PIC на <strong>MSI-X</strong>. Режим опроса при недоступности MSI-X.
</p>

---

## 🔨 Building

**Standalone**

```sh
make install   # auto-detects ../CactKernel-x86_32 and ../LocalRepoCactOS
make clean
```

Override paths if needed: `make KERN_ROOT=/custom/path LOCAL_REPO=/custom/path install`.
