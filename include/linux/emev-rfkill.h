/*
 *  File Name          : include/linux/emxx-rfkill.h
 *  Function           : emev_board
 *  Release Version    : Ver 1.00
 *  Release Date       : 2011/05/20
 *
 * Copyright (C) 2010 Renesas Electronics Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1335, USA.
 */

#ifndef _LINUX_EMEV_RFKILL_H
#define _LINUX_EMEV_RFKILL_H

#include <linux/rfkill.h>

struct emev_rfkill_platform_data {
       int nshutdown_gpio;

       struct rfkill *rfkill;  /* for driver only */
};

#endif

