# 🛠 AHCI-for-Cact

<p align="center">
  <img src="https://img.shields.io/badge/license-GPLv3-blue.svg?style=for-the-badge" alt="License: GPLv3">
  <img src="https://img.shields.io/badge/arch-i686-red.svg?style=for-the-badge" alt="Arch: i686">
  <img src="https://img.shields.io/badge/format-cctk-green.svg?style=for-the-badge" alt="Output: ahci.cctk">
  <img src="https://img.shields.io/badge/bus-PCI-blue.svg?style=for-the-badge" alt="PCI">
</p>

<p align="center">
  <strong>English.</strong> Out-of-tree <strong>AHCI / SATA HBA</strong> driver packaged as a relocatable <strong><code>ahci.cctk</code></strong> for the Cact kernel module loader.<br>
  <strong>Русский.</strong> Драйвер <strong>AHCI</strong> вне дерева ядра — модуль <strong><code>ahci.cctk</code></strong> для <strong>kmod</strong>.
</p>

---

## 🔨 Building

**Standalone**

```sh
make install   # auto-detects ../CactKernel-x86_32 and ../LocalRepoCactOS
make clean
```

Override paths if needed: `make KERN_ROOT=/custom/path LOCAL_REPO=/custom/path install`.
