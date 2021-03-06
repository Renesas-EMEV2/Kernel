/*
*  File Name       : arch/arm/mach-emxx/include/mach/emxx_v4l2.h
*  Function        : V4L2 Driver I/F definitions
*  Release Version : Ver 1.00
*  Release Date    : 2010.03.01
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
 * Inc., 51 Franklin Street, Suite 500, Boston, MA 02110-1335, USA.
*
*/

#ifndef _EMXX_V4L2_H_
#define _EMXX_V4L2_H_

/*===========================================================================*/
/* extern declarations of public functions                                   */
/*===========================================================================*/
extern int emxx_v4l2_mmap(struct vm_area_struct *vma);

#endif /* _EMXX_V4L2_H_ */
