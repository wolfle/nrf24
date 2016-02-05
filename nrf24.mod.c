#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

MODULE_INFO(vermagic, VERMAGIC_STRING);

struct module __this_module
__attribute__((section(".gnu.linkonce.this_module"))) = {
 .name = KBUILD_MODNAME,
 .init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
 .exit = cleanup_module,
#endif
 .arch = MODULE_ARCH_INIT,
};

static const struct modversion_info ____versions[]
__used
__attribute__((section("__versions"))) = {
	{ 0xda527c2f, "module_layout" },
	{ 0x6d044c26, "param_ops_uint" },
	{ 0x910c91f4, "driver_unregister" },
	{ 0x2cfd8cf7, "spi_register_driver" },
	{ 0xbc94c439, "register_netdev" },
	{ 0x2072ee9b, "request_threaded_irq" },
	{ 0x10679eab, "put_device" },
	{ 0x631d77a6, "spi_setup" },
	{ 0x5ac68aeb, "get_device" },
	{ 0xdec5ae0f, "alloc_netdev_mqs" },
	{ 0x8574ca6c, "gpio_request_array" },
	{ 0x9dfdf722, "gpio_free_array" },
	{ 0xa5a1e7f9, "free_netdev" },
	{ 0x31a8d705, "unregister_netdev" },
	{ 0x6ac6eee7, "dev_set_drvdata" },
	{ 0x1d67db85, "dev_get_drvdata" },
	{ 0xe5f89da4, "dev_kfree_skb_any" },
	{ 0x27d3da7f, "netif_rx" },
	{ 0xf54a03db, "skb_put" },
	{ 0xc092eb7e, "__netdev_alloc_skb" },
	{ 0xfcec0987, "enable_irq" },
	{ 0x27e1a049, "printk" },
	{ 0xc76bc1c9, "skb_dequeue" },
	{ 0xac2215a0, "consume_skb" },
	{ 0x3ce4ca6f, "disable_irq" },
	{ 0x432fd7f6, "__gpio_set_value" },
	{ 0x7d11c268, "jiffies" },
	{ 0x308c837a, "skb_queue_tail" },
	{ 0x9d669763, "memcpy" },
	{ 0xcc116cea, "spi_sync" },
	{ 0xfa2a45e, "__memzero" },
	{ 0x5f754e5a, "memset" },
	{ 0xefd6cf06, "__aeabi_unwind_cpp_pr0" },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=";


MODULE_INFO(srcversion, "C56997C77DCC5C01A884DC6");
