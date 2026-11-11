#include <linux/module.h>
#include <linux/export-internal.h>
#include <linux/compiler.h>

MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__section(".gnu.linkonce.this_module") = {
	.name = KBUILD_MODNAME,
	.arch = MODULE_ARCH_INIT,
};

KSYMTAB_FUNC(ath6kl_printk, "");
SYMBOL_FLAGS(ath6kl_printk, 0x00);
KSYMTAB_FUNC(ath6kl_info, "");
SYMBOL_FLAGS(ath6kl_info, 0x00);
KSYMTAB_FUNC(ath6kl_err, "");
SYMBOL_FLAGS(ath6kl_err, 0x00);
KSYMTAB_FUNC(ath6kl_warn, "");
SYMBOL_FLAGS(ath6kl_warn, 0x00);
KSYMTAB_FUNC(ath6kl_read_tgt_stats, "");
SYMBOL_FLAGS(ath6kl_read_tgt_stats, 0x00);
KSYMTAB_FUNC(ath6kl_dbg, "");
SYMBOL_FLAGS(ath6kl_dbg, 0x00);
KSYMTAB_FUNC(ath6kl_dbg_dump, "");
SYMBOL_FLAGS(ath6kl_dbg_dump, 0x00);
KSYMTAB_FUNC(ath6kl_hif_rw_comp_handler, "");
SYMBOL_FLAGS(ath6kl_hif_rw_comp_handler, 0x00);
KSYMTAB_FUNC(ath6kl_hif_intr_bh_handler, "");
SYMBOL_FLAGS(ath6kl_hif_intr_bh_handler, 0x00);
KSYMTAB_FUNC(ath6kl_cfg80211_suspend, "");
SYMBOL_FLAGS(ath6kl_cfg80211_suspend, 0x00);
KSYMTAB_FUNC(ath6kl_cfg80211_resume, "");
SYMBOL_FLAGS(ath6kl_cfg80211_resume, 0x00);
KSYMTAB_FUNC(ath6kl_stop_txrx, "");
SYMBOL_FLAGS(ath6kl_stop_txrx, 0x00);
KSYMTAB_FUNC(ath6kl_core_tx_complete, "");
SYMBOL_FLAGS(ath6kl_core_tx_complete, 0x00);
KSYMTAB_FUNC(ath6kl_core_rx_complete, "");
SYMBOL_FLAGS(ath6kl_core_rx_complete, 0x00);
KSYMTAB_FUNC(ath6kl_core_init, "");
SYMBOL_FLAGS(ath6kl_core_init, 0x00);
KSYMTAB_FUNC(ath6kl_core_create, "");
SYMBOL_FLAGS(ath6kl_core_create, 0x00);
KSYMTAB_FUNC(ath6kl_core_cleanup, "");
SYMBOL_FLAGS(ath6kl_core_cleanup, 0x00);
KSYMTAB_FUNC(ath6kl_core_destroy, "");
SYMBOL_FLAGS(ath6kl_core_destroy, 0x00);

MODULE_INFO(depends, "cfg80211");

