#
# Makefile for the EMXX framebuffer device driver
#


obj-$(CONFIG_FB_EMXX)  += emxx_fb.o
obj-$(CONFIG_FB_EMXX)  += emxx_lcd.o
obj-$(CONFIG_FB_EMXX)  += emxx_lcdhw.o

obj-$(CONFIG_FB_EMXX_COPYBIT) += emxx_fb_blit.o alphacomposite.o
obj-$(CONFIG_FB_EMXX_COPYBIT) += neon_alphablend_type2_0_0_0.o neon_alphablend_type2_0_1_1.o neon_alphablend_type2_0_2_2.o

obj-$(CONFIG_EMXX_HDMI_ADV7523) += emxx_adv7523.o

