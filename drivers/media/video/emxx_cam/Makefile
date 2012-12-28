# Makefile for emxx_camera driver for emxx
###
#obj-$(CONFIG_VIDEO_EMXX_CAMERA) += emxx_cam_front.o
obj-$(CONFIG_EMXX_CAMERA_OV767X) += emxx_cam_front.o emxx_ov7675.o
obj-$(CONFIG_EMXX_CAMERA_GC0307) += emxx_cam_front.o emxx_gc0307.o

#obj-$(CONFIG_VIDEO_EMXX_REAR_CAMERA) += emxx_cam_back.o
obj-$(CONFIG_CAMERA_EMXX_OV3640) += emxx_cam_back.o emxx_ov3640.o


EXTRA_CFLAGS = -I$(src)/.. -Wall
