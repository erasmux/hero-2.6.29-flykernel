
# core framebuffer
#
obj-y := msm_fb.o logo.o
ifeq ($(CONFIG_FB_MSM_LOGO),y)
obj-y += logo.o
endif

# MDP DMA/PPP engine
#
obj-y += mdp.o mdp_ppp.o


obj-$(CONFIG_MSM_MDP22) += mdp_ppp22.o
obj-$(CONFIG_MSM_MDP30) += mdp_ppp22.o
obj-$(CONFIG_MSM_MDP302)+= mdp_ppp22.o
obj-$(CONFIG_MSM_MDP31) += mdp_ppp31.o

# MDDI interface
#
obj-y += mddi.o

# MDDI client/panel drivers
#
obj-y += mddi_client_dummy.o
obj-y += mddi_client_toshiba.o
obj-y += mddi_client_eid.o mddi_eid_cabc.o
obj-y += mddi_client_nt35399.o
obj-y += mddi_client_eid.o
obj-y += mddi_client_s6d0154.o
obj-y += mddi_client_epson.o
obj-y += mddi_client_novb9f6_5582.o
obj-y += mddi_client_s6d04d1.o
obj-y += mddi_client_hx8356.o mddi_himax_cabc.o

# MDP LCD controller driver
obj-$(CONFIG_FB_MSM_LCDC) += mdp_lcdc.o
obj-$(CONFIG_FB_MSM_TVOUT) += tvenc.o tvfb.o

# Yamato GL driver
obj-$(CONFIG_GPU_MSM_KGSL) += gpu/kgsl/

obj-$(CONFIG_MSM_HDMI) += hdmi/
