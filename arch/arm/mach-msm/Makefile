obj-y += io.o irq.o timer.o dma.o memory.o
obj-$(CONFIG_ARCH_MSM_SCORPION) += sirc.o
obj-y += devices.o pwrtest.o
obj-y += proc_comm.o
obj-y += vreg.o
obj-y += pmic.o
obj-y += remote_spinlock.o
obj-$(CONFIG_ARCH_MSM_ARM11) += acpuclock-arm11.o idle.o
# obj-$(CONFIG_ARCH_MSM_SCORPION) += acpuclock-scorpion.o idle-v7.o
obj-$(CONFIG_ARCH_MSM_SCORPION) += arch-init-scorpion.o
obj-$(CONFIG_ARCH_MSM_SCORPION) += acpuclock-scorpion.o idle-v7.o appsbl_sc_sa_fuse_ro.o

obj-y += clock.o
obj-y += gpio.o generic_gpio.o
obj-y += nand_partitions.o
obj-y += drv_callback.o
obj-$(CONFIG_ARCH_QSD8X50) += pmic.o htc_wifi_nvs.o htc_bluetooth.o

obj-$(CONFIG_MSM_FIQ_SUPPORT) += fiq_glue.o
obj-$(CONFIG_MACH_TROUT) += board-trout-rfkill.o
obj-$(CONFIG_MSM_SMD) += smd.o smd_debug.o
obj-$(CONFIG_MSM_SMD) += smd_tty.o smd_qmi.o
obj-$(CONFIG_MSM_SMD) += smem_log.o
obj-$(CONFIG_MSM_SMD) += last_radio_log.o
obj-$(CONFIG_MSM_SMD) += htc_port_list.o
obj-$(CONFIG_MSM_ONCRPCROUTER) += smd_rpcrouter.o
obj-$(CONFIG_MSM_ONCRPCROUTER) += smd_rpcrouter_device.o
obj-$(CONFIG_MSM_ONCRPCROUTER) += smd_rpcrouter_servers.o
obj-$(CONFIG_MSM_RPCSERVERS) += rpc_server_dog_keepalive.o
obj-$(CONFIG_MSM_RPCSERVERS) += rpc_server_time_remote.o
obj-$(CONFIG_MSM_DALRPC) += dal.o
obj-$(CONFIG_MSM_DALRPC_TEST) += dal_remotetest.o
obj-$(CONFIG_MSM_ADSP) += qdsp5/
obj-$(CONFIG_MSM_ADSP_COMP) += qdsp5_comp/
obj-$(CONFIG_QSD_AUDIO) += qdsp6/
obj-$(CONFIG_MSM_HW3D) += hw3d.o
obj-$(CONFIG_PM) += pm.o
obj-$(CONFIG_CPU_FREQ) += cpufreq.o

obj-$(CONFIG_HTC_ACOUSTIC) += htc_acoustic.o
obj-$(CONFIG_HTC_ACOUSTIC_QSD) += htc_acoustic_qsd.o
obj-$(CONFIG_SENSORS_AKM8976) += htc_akm_cal.o

obj-$(CONFIG_MACH_HALIBUT) += board-halibut.o board-halibut-panel.o
obj-$(CONFIG_MACH_HALIBUT) += board-halibut-keypad.o fish_battery.o
obj-$(CONFIG_MACH_SWORDFISH) += board-swordfish.o
obj-$(CONFIG_MACH_SWORDFISH) += board-swordfish-keypad.o fish_battery.o
obj-$(CONFIG_MACH_SWORDFISH) += board-swordfish-panel.o
obj-$(CONFIG_MACH_SWORDFISH) += board-swordfish-mmc.o
obj-$(CONFIG_MACH_FIRESTONE) += board-firestone.o board-firestone-panel.o
obj-$(CONFIG_MACH_FIRESTONE) += board-firestone-keypad.o board-firestone-mmc.o
obj-$(CONFIG_MACH_FIRESTONE) += board-firestone-rfkill.o
obj-$(CONFIG_MACH_TROUT) += board-trout.o board-trout-gpio.o
obj-$(CONFIG_MACH_TROUT) += board-trout-keypad.o board-trout-panel.o
obj-$(CONFIG_MACH_TROUT) += htc_akm_cal.o htc_wifi_nvs.o htc_acoustic.o
obj-$(CONFIG_MACH_TROUT) += board-trout-mmc.o board-trout-wifi.o
obj-$(CONFIG_MACH_TROUT) += devices_htc.o
obj-$(CONFIG_MACH_MAHIMAHI) += board-mahimahi.o board-mahimahi-panel.o
obj-$(CONFIG_MACH_MAHIMAHI) += board-mahimahi-keypad.o board-mahimahi-mmc.o

obj-$(CONFIG_MACH_MAHIMAHI) += board-mahimahi-rfkill.o
obj-$(CONFIG_MACH_MAHIMAHI) += board-mahimahi-wifi.o board-mahimahi-audio.o
obj-$(CONFIG_MACH_MAHIMAHI) += msm_vibrator.o
obj-$(CONFIG_MACH_MAHIMAHI) += board-mahimahi-microp.o
obj-$(CONFIG_MACH_MAHIMAHI) += htc_acoustic_qsd.o
# obj-$(CONFIG_MACH_MAHIMAHI) += board-mahimahi-flashlight.o

obj-$(CONFIG_MACH_PASSIONC) += board-passionc.o board-amoled-panel.o
obj-$(CONFIG_MACH_PASSIONC) += board-passionc-keypad.o board-passionc-mmc.o
obj-$(CONFIG_MACH_PASSIONC) += board-passionc-rfkill.o board-passionc-audio.o
obj-$(CONFIG_MACH_PASSIONC) += board-passionc-wifi.o htc_awb_cal.o
obj-$(CONFIG_MACH_PASSIONC) += board-passionc-microp.o
obj-$(CONFIG_MACH_INCREDIBLE) += board-incredible.o board-incredible-panel.o
obj-$(CONFIG_MACH_INCREDIBLE) += board-incredible-keypad.o board-incredible-mmc.o
obj-$(CONFIG_MACH_INCREDIBLE) += board-incredible-rfkill.o board-incredible-audio.o
obj-$(CONFIG_MACH_INCREDIBLEC) += board-incrediblec.o board-amoled-panel.o board-incrediblec-tv.o
obj-$(CONFIG_MACH_INCREDIBLEC) += board-incrediblec-keypad.o board-incrediblec-mmc.o
obj-$(CONFIG_MACH_INCREDIBLEC) += board-incrediblec-rfkill.o board-incrediblec-audio.o
obj-$(CONFIG_MACH_INCREDIBLEC) += board-incrediblec-wifi.o htc_awb_cal.o
obj-$(CONFIG_MACH_INCREDIBLEC) += board-incrediblec-microp.o
obj-$(CONFIG_MACH_INCREDIBLEC) += proc_engineerid.o
obj-$(CONFIG_MACH_SUPERSONIC) += board-supersonic.o board-supersonic-panel.o
obj-$(CONFIG_MACH_SUPERSONIC) += board-supersonic-keypad.o board-supersonic-mmc.o
obj-$(CONFIG_MACH_SUPERSONIC) += board-supersonic-rfkill.o board-supersonic-audio.o
obj-$(CONFIG_MACH_SUPERSONIC) += board-supersonic-wifi.o htc_awb_cal.o
obj-$(CONFIG_MACH_SUPERSONIC) += board-supersonic-microp.o
obj-$(CONFIG_MACH_ORCA) += board-orca.o board-orca-panel.o
obj-$(CONFIG_MACH_ORCA) += board-orca-keypad.o board-orca-mmc.o
obj-$(CONFIG_MACH_ORCA) += board-orca-rfkill.o
obj-$(CONFIG_MACH_ORCA) += board-orca-audio.o board-orca-wifi.o
obj-$(CONFIG_MACH_ORCA) += board-orca-microp.o
obj-$(CONFIG_MACH_BRAVO) += board-bravo.o board-amoled-panel.o
obj-$(CONFIG_MACH_BRAVO) += board-bravo-keypad.o board-bravo-mmc.o
obj-$(CONFIG_MACH_BRAVO) += board-bravo-rfkill.o
obj-$(CONFIG_MACH_BRAVO) += board-bravo-audio.o board-bravo-wifi.o htc_awb_cal.o
obj-$(CONFIG_MACH_BRAVO) += board-bravo-microp.o
obj-$(CONFIG_MACH_BRAVOC) += board-bravoc.o board-amoled-panel.o
obj-$(CONFIG_MACH_BRAVOC) += board-bravoc-keypad.o board-bravoc-mmc.o
obj-$(CONFIG_MACH_BRAVOC) += board-bravoc-rfkill.o board-bravoc-audio.o
obj-$(CONFIG_MACH_BRAVOC) += board-bravoc-wifi.o htc_awb_cal.o
obj-$(CONFIG_MACH_BRAVOC) += board-bravoc-microp.o

# MSM7200A boards
obj-$(CONFIG_MACH_HERO) += board-hero.o board-hero-panel.o board-hero-keypad.o
obj-$(CONFIG_MACH_HERO) += board-hero-rfkill.o
obj-$(CONFIG_MACH_HERO) += board-hero-mmc.o board-hero-wifi.o htc_wifi_nvs.o htc_awb_cal.o

obj-$(CONFIG_MACH_SAPPHIRE) += board-sapphire.o board-sapphire-panel.o board-sapphire-keypad.o board-sapphire-gpio.o
obj-$(CONFIG_MACH_SAPPHIRE) += board-sapphire-rfkill.o
obj-$(CONFIG_MACH_SAPPHIRE) += board-sapphire-mmc.o board-sapphire-wifi.o
#obj-$(CONFIG_MACH_SAPPHIRE) +=  msm_vibrator.o
#obj-$(CONFIG_MACH_SAPPHIRE) += devices_htc.o


# MSM7225 boards
obj-$(CONFIG_MACH_BUZZ) += board-buzz.o board-buzz-panel.o board-buzz-keypad.o board-buzz-microp.o
obj-$(CONFIG_MACH_BUZZ) += board-buzz-rfkill.o board-buzz-mmc.o board-buzz-wifi.o
obj-$(CONFIG_MACH_BEE) += board-bee.o board-bee-panel.o board-bee-keypad.o board-bee-microp.o
obj-$(CONFIG_MACH_BEE) += board-bee-rfkill.o board-bee-mmc.o board-bee-wifi.o
obj-$(CONFIG_MACH_BUZZC) += board-buzzc.o board-buzzc-panel.o board-buzzc-keypad.o board-buzzc-microp.o
obj-$(CONFIG_MACH_BUZZC) += board-buzzc-rfkill.o board-buzzc-mmc.o board-buzzc-wifi.o
obj-$(CONFIG_MACH_PARADISE) += board-paradise.o board-paradise-panel.o board-paradise-keypad.o
obj-$(CONFIG_MACH_PARADISE) += board-paradise-rfkill.o board-paradise-wifi.o
obj-$(CONFIG_MACH_PARADISE) += board-paradise-mmc.o board-paradise-microp.o
obj-$(CONFIG_MACH_BAHAMAS) += board-bahamas.o board-click-panel.o board-bahamas-keypad.o
obj-$(CONFIG_MACH_BAHAMAS) += board-bahamas-mmc.o board-bahamas-rfkill.o

# MSM7227 boards
obj-$(CONFIG_MACH_LEGEND) += board-legend-rfkill.o
obj-$(CONFIG_MACH_LEGEND) += board-legend.o board-legend-keypad.o board-legend-panel.o
obj-$(CONFIG_MACH_LEGEND) += board-legend-mmc.o board-legend-wifi.o board-legend-microp.o
obj-$(CONFIG_MACH_LATTE) += board-latte-rfkill.o
obj-$(CONFIG_MACH_LATTE) += board-latte.o board-latte-keypad.o board-latte-panel.o
obj-$(CONFIG_MACH_LATTE) += board-latte-mmc.o board-latte-wifi.o board-latte-microp.o
obj-$(CONFIG_MACH_LIBERTY) += board-liberty-rfkill.o htc_bluetooth.o
obj-$(CONFIG_MACH_LIBERTY) += board-liberty.o board-liberty-keypad.o board-liberty-panel.o
obj-$(CONFIG_MACH_LIBERTY) += board-liberty-mmc.o board-liberty-wifi.o board-liberty-microp.o

# MSM7600 boards
obj-$(CONFIG_MACH_DESIREC) += board-desirec.o board-desirec-panel.o board-desirec-keypad.o
obj-$(CONFIG_MACH_DESIREC) += board-desirec-rfkill.o
obj-$(CONFIG_MACH_DESIREC) += board-desirec-mmc.o board-desirec-wifi.o htc_wifi_nvs.o
obj-$(CONFIG_MACH_HEROC) += board-heroc.o board-heroc-panel.o board-heroc-keypad.o
obj-$(CONFIG_MACH_HEROC) += board-heroc-rfkill.o
obj-$(CONFIG_MACH_HEROC) += board-heroc-mmc.o board-heroc-wifi.o htc_wifi_nvs.o

obj-$(CONFIG_ARCH_MSM7X00A) += msm_vibrator.o devices_htc.o
obj-$(CONFIG_ARCH_MSM7225) += msm_vibrator.o devices_htc.o htc_wifi_nvs.o htc_awb_cal.o htc_bluetooth.o
obj-$(CONFIG_ARCH_MSM7227) += msm_vibrator.o devices_htc.o htc_wifi_nvs.o htc_awb_cal.o
obj-$(CONFIG_ARCH_QSD8X50) += msm_vibrator.o
obj-$(CONFIG_ARCH_MSM_FLASHLIGHT) += msm_flashlight.o
# obj-$(CONFIG_TROUT_BATTCHG) += htc_battery.o
# obj-$(CONFIG_TROUT_BATTCHG) += htc_power_supply.o
obj-$(CONFIG_TROUT_BATTCHG_DOCK) += htc_one_wire.o

obj-$(CONFIG_TROUT_BATTCHG) += htc_battery_trout.o
obj-$(CONFIG_HTC_BATTCHG) += htc_battery.o
obj-$(CONFIG_HTC_PWRSINK) += htc_pwrsink.o
# obj-$(CONFIG_HTC_HEADSET) += htc_headset.o
# obj-$(CONFIG_HTC_35MM_JACK) += htc_35mm_jack.o

obj-$(CONFIG_HTC_HEADSET_V1) += htc_headset_v1.o
obj-$(CONFIG_HTC_35MM_REMOTE_V1) += htc_35mm_remote_v1.o
obj-$(CONFIG_HTC_AUDIOJACK_V1) += htc_audio_jack_v1.o
obj-$(CONFIG_HTC_HEADSET) += htc_35mm_remote.o htc_headset_common.o
obj-$(CONFIG_HTC_AUDIOJACK) += htc_audio_jack.o
obj-$(CONFIG_PERFLOCK) += perflock.o
obj-$(CONFIG_PERFLOCK) += htc_set_perflock.o
obj-$(CONFIG_MICROP_COMMON) += atmega_microp_common.o
obj-$(CONFIG_TOUCHSCREEN_TSSC_MANAGER) += htc_tp_cal.o

