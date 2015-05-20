/**********************************************************************
 *
 * Copyright (C) Imagination Technologies Ltd. All rights reserved.
 * 
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 * 
 * This program is distributed in the hope it will be useful but, except 
 * as otherwise stated in writing, without any warranty; without even the 
 * implied warranty of merchantability or fitness for a particular purpose. 
 * See the GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 * 
 * The full GNU General Public License is included in this distribution in
 * the file called "COPYING".
 *
 * Contact Information:
 * Imagination Technologies Ltd. <gpl-support@imgtec.com>
 * Home Park Estate, Kings Langley, Herts, WD4 8LZ, UK 
 *
 ******************************************************************************/

#if defined(__linux__)
#include "sysutils_linux.c"
#if defined(SYS_OMAP4_HAS_DVFS_FRAMEWORK)
#include "sgxfreq.c"
#include "sgxfreq_onoff.c"
#include "sgxfreq_activeidle.c"
#include "sgxfreq_on3demand.c"
#include "sgxfreq_userspace.c"
#if defined(CONFIG_THERMAL_FRAMEWORK)
#include "sgxfreq_cool.c"
#endif
#endif
#endif


