
/**
 * Hamilton-Wright, Andrew (2015)
 *
 * This is a driver modified from the original psm.c driver, for use in
 * teaching CIS*3110.
 *
 * This driver, with some modifications, will handle both the PS/2 mouse
 * and keyboard devices.
 *
 * You will notice that both the tsleep() and wakeup() functions are
 * missing; it is your task to decide where they must go.  The tsleep()
 * function takes some arguments, so as a hint, the tsleep line should
 * look more or less like this:
 *     error = tsleep(sc, PZERO | PCATCH, "psmrea", 0);
 *
 * Lines identified with "???" are to be described as indicated in
 * the assignment description.
 *
 * You can modify this file and recompile/reinstall the kernel to
 * help you determine what is going on, however keep in mind that this
 * may cause the keyboard to stop functioning!  For this reason, it is
 * important that you learn how to boot from an older kernel as described
 * in the assignment document.
 *
 * This is a simplified version of the original FreeBSD driver, and therefore
 * only handles one type of mouse
 */

/*-
 * Copyright (c) 1992, 1993 Erik Forsberg.
 * Copyright (c) 1996, 1997 Kazutaka YOKOTA.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * THIS SOFTWARE IS PROVIDED BY ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 * NO EVENT SHALL I BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*
 *  Ported to 386bsd Oct 17, 1992
 *  Sandi Donno, Computer Science, University of Cape Town, South Africa
 *  Please send bug reports to sandi@cs.uct.ac.za
 *
 *  Thanks are also due to Rick Macklem, rick@snowhite.cis.uoguelph.ca -
 *  although I was only partially successful in getting the alpha release
 *  of his "driver for the Logitech and ATI Inport Bus mice for use with
 *  386bsd and the X386 port" to work with my Microsoft mouse, I nevertheless
 *  found his code to be an invaluable reference when porting this driver
 *  to 386bsd.
 *
 *  Further modifications for latest 386BSD+patchkit and port to NetBSD,
 *  Andrew Herbert <andrew@werple.apana.org.au> - 8 June 1993
 *
 *  Cloned from the Microsoft Bus Mouse driver, also by Erik Forsberg, by
 *  Andrew Herbert - 12 June 1993
 *
 *  Modified for PS/2 mouse by Charles Hannum <mycroft@ai.mit.edu>
 *  - 13 June 1993
 *
 *  Modified for PS/2 AUX mouse by Shoji Yuen <yuen@nuie.nagoya-u.ac.jp>
 *  - 24 October 1993
 *
 *  Hardware access routines and probe logic rewritten by
 *  Kazutaka Yokota <yokota@zodiac.mech.utsunomiya-u.ac.jp>
 *  - 3, 14, 22 October 1996.
 *  - 12 November 1996. IOCTLs and rearranging `psmread', `psmioctl'...
 *  - 14, 30 November 1996. Uses `kbdio.c'.
 *  - 13 December 1996. Uses queuing version of `kbdio.c'.
 *  - January/February 1997. Tweaked probe logic for
 *    HiNote UltraII/Latitude/Armada laptops.
 *  - 30 July 1997. Added APM support.
 *  - 5 March 1997. Defined driver configuration flags (PSM_CONFIG_XXX).
 *    Improved sync check logic.
 *    Vendor specific support routines.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD: src/sys/dev/atkbdc/psm.c,v 1.93.2.4.2.1 2009/04/15 03:14:26 kensmith Exp $");

#include "opt_isa.h"
#include "opt_psm.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/filio.h>
#include <sys/poll.h>
#include <sys/sigio.h>
#include <sys/signalvar.h>
#include <sys/syslog.h>
#include <machine/bus.h>
#include <sys/rman.h>
#include <sys/selinfo.h>
#include <sys/sysctl.h>
#include <sys/time.h>
#include <sys/uio.h>

#include <sys/limits.h>
#include <sys/mouse.h>
#include <machine/resource.h>

#ifdef DEV_ISA
#include <isa/isavar.h>
#endif

#include <dev/atkbdc/atkbdcreg.h>
#include <dev/atkbdc/psm.h>

/*
 * Driver specific options: the following options may be set by
 * `options' statements in the kernel configuration file.
 */

/*
 * ***
 * debugging : see "verbose" variable below
 */
#ifndef PSM_DEBUG
#define	PSM_DEBUG	1	/*
				 * logging: 0: none, 1: brief, 2: verbose
				 *          3: sync errors, 4: all packets
				 */
#endif
#define	VLOG(level, args)	do {	\
		if (verbose >= level)		\
			log args;		\
	} while (0)

/* end of driver specific options */



/*
 * driver name as advertised in Plug'n'Play
 */
#define	PSMCPNP_DRIVER_NAME	"psmcpnp"



/*
 * macros for major/minor device node interpretation
 */
#define	PSM_UNIT(dev)		(minor(dev) >> 1)
#define	PSM_NBLOCKIO(dev)	(minor(dev) & 1)
#define	PSM_MKMINOR(unit,block)	(((unit) << 1) | ((block) ? 0:1))


/*
 * Our name as reported to the kernel
 */
#define PSM_MOUSE_MODEL_GENERIC	"Generic PS/2 mouse"


/*
 * *** ???
 * Q1: What effect does this size have?
 - It defines the size of message queue

 */
#define	PSMQSIZE		10

/*
 * driver control block -- global data definitions shared between functions,
 * including the interrupt handler.
 *
 * Mostly queue(s) to pass data between the upper half (I/O functions
 * called from the user process) and lower half (interrupt service
 * routine) called psmintr().
 */
struct psm_softc {		/* Driver status information */
	int		unit;
	struct selinfo	rsel;		/* Process selecting for Input */
	u_char		state;		/* Mouse driver state */
	int		config;		/* driver configuration flags */
	KBDC		kbdc;		/* handle to access kbd controller */
	struct resource	*intr;		/* IRQ resource */
	void		*ih;		/* interrupt handle */
	mousehw_t	hw;		/* hardware information */
	mousemode_t	mode;		/* operation mode */
	mousemode_t	dflt_mode;	/* default operation mode */

	int		localmode;	/* used to demonstrate ioctl */

		/* data queue and sizes */
	u_char		rqueue[PSMQSIZE];
	int		rinpos;
	int		routpos;
	int		rqsize;

	struct cdev	*dev;
	struct cdev	*bdev;
	int		lasterr;
	int		cmdcount;
	struct sigio	*async;		/* Processes waiting for SIGIO */
};

static devclass_t psm_devclass;

#define	PSM_SOFTC(unit)	\
    ((struct psm_softc*)devclass_get_softc(psm_devclass, unit))


/* driver state flags (state) */
#define	PSM_VALID		0x80
#define	PSM_OPEN		1	/* Device is open */
#define	PSM_ASLP		2	/* Waiting for mouse data */
#define	PSM_SOFTARMED		4	/* Software interrupt armed */
#define	PSM_NEED_SYNCBITS	8	/* Set syncbits using next data pkt */

/* driver configuration flags (config) */
#define	PSM_CONFIG_RESOLUTION	0x000f	/* resolution */
#define	PSM_CONFIG_ACCEL	0x00f0  /* acceleration factor */
#define	PSM_CONFIG_NOCHECKSYNC	0x0100  /* disable sync. test */
#define	PSM_CONFIG_IGNPORTERROR	0x0200  /* ignore error in aux port test */
#define	PSM_CONFIG_HOOKRESUME	0x0400	/* hook the system resume event */
#define	PSM_CONFIG_INITAFTERSUSPEND 0x0800 /* init the device at the resume event */

#define	PSM_CONFIG_FLAGS	\
    (PSM_CONFIG_RESOLUTION |	\
    PSM_CONFIG_ACCEL |		\
    PSM_CONFIG_NOCHECKSYNC |	\
    PSM_CONFIG_IGNPORTERROR |	\
    PSM_CONFIG_HOOKRESUME |	\
    PSM_CONFIG_INITAFTERSUSPEND)

/*
 * ***
 * Tunable log level: This can be adjusted using
 *	sysctl debug.psm.loglevel=2
 */
static int verbose = PSM_DEBUG;
TUNABLE_INT("debug.psm.loglevel", &verbose);



/*
 * ***
 * function prototypes
 *
 * These are registered in the loadable driver module through the
 * variable psm_driver, below
 */
static void	psmidentify(driver_t *, device_t);
static int	psmprobe(device_t);
static int	psmattach(device_t);
static int	psmdetach(device_t);
static int	psmresume(device_t);


static d_open_t		psmopen;
static d_close_t	psmclose;
static d_read_t		psmread;
static d_write_t	psmwrite;
static d_ioctl_t	psmioctl;
static d_poll_t		psmpoll;


static int	enable_aux_dev(KBDC);
static int	disable_aux_dev(KBDC);
static int	get_mouse_status(KBDC, int *, int, int);
static int	get_aux_id(KBDC);
static int	set_mouse_sampling_rate(KBDC, int);
static int	set_mouse_resolution(KBDC, int);
static int	doopen(struct psm_softc *, int);
static void	psmintr(void *);



/*
 * device driver declaration -- the loadable driver module finds
 * the important functions this way
 */
static device_method_t psm_methods[] = {
	/* Device interface */
	DEVMETHOD(device_identify,	psmidentify),
	DEVMETHOD(device_probe,		psmprobe),
	DEVMETHOD(device_attach,	psmattach),
	DEVMETHOD(device_detach,	psmdetach),
	DEVMETHOD(device_resume,	psmresume),

	{ 0, 0 }
};

/*
 * ***
 * Here the methods are bundled with the driver name
 * (defined in psm.h) to allow the driver to be recognized
 * by the module loader.
 */
static driver_t psm_driver = {
	PSM_DRIVER_NAME,
	psm_methods,
	sizeof(struct psm_softc),
};

/*
 * ***
 * Bundle the function accessible through the device driver
 * into a generic device structure; these are then attached
 * to the driver in psmattach(), below (note reference of
 * psm_cdevsw) in that function
 */
static struct cdevsw psm_cdevsw = {
	.d_version =	D_VERSION,
	.d_flags =	D_NEEDGIANT,
	.d_open =	psmopen,
	.d_close =	psmclose,
	.d_read =	psmread,
	.d_write =	psmwrite,
	.d_ioctl =	psmioctl,
	.d_poll =	psmpoll,
	.d_name =	PSM_DRIVER_NAME,
};



/*
 * ***
 * device I/O support routines -- the "main parts" of the
 * driver are below, and have function names of the form psm*()
 */

/* enable the PS/2 keyboard */
static int
enable_aux_dev(KBDC kbdc)
{
	int res;

	res = send_aux_command(kbdc, PSMC_ENABLE_DEV);
	VLOG(2, (LOG_DEBUG, "psm: ENABLE_DEV return code:%04x\n", res));

	return (res == PSM_ACK);
}

/* disable the PS/2 keyboard */
static int
disable_aux_dev(KBDC kbdc)
{
	int res;

	res = send_aux_command(kbdc, PSMC_DISABLE_DEV);
	VLOG(2, (LOG_DEBUG, "psm: DISABLE_DEV return code:%04x\n", res));

	return (res == PSM_ACK);
}

static int
get_mouse_status(KBDC kbdc, int *status, int flag, int len)
{
	int cmd;
	int res;
	int i;

	switch (flag) {
	case 0:
	default:
		cmd = PSMC_SEND_DEV_STATUS;
		break;
	case 1:
		cmd = PSMC_SEND_DEV_DATA;
		break;
	}
	empty_aux_buffer(kbdc, 5);
	res = send_aux_command(kbdc, cmd);
	VLOG(2, (LOG_DEBUG, "psm: SEND_AUX_DEV_%s return code:%04x\n",
	    (flag == 1) ? "DATA" : "STATUS", res));
	if (res != PSM_ACK)
		return (0);

	for (i = 0; i < len; ++i) {
		status[i] = read_aux_data(kbdc);
		if (status[i] < 0)
			break;
	}

	VLOG(1, (LOG_DEBUG, "psm: %s %02x %02x %02x\n",
	    (flag == 1) ? "data" : "status", status[0], status[1], status[2]));

	return (i);
}

static int
get_aux_id(KBDC kbdc)
{
	int res;
	int id;

	empty_aux_buffer(kbdc, 5);
	res = send_aux_command(kbdc, PSMC_SEND_DEV_ID);
	VLOG(2, (LOG_DEBUG, "psm: SEND_DEV_ID return code:%04x\n", res));
	if (res != PSM_ACK)
		return (-1);

	/* 10ms delay */
	DELAY(10000);

	id = read_aux_data(kbdc);
	VLOG(2, (LOG_DEBUG, "psm: device ID: %04x\n", id));

	return (id);
}

static int
set_mouse_sampling_rate(KBDC kbdc, int rate)
{
	int res;

	res = send_aux_command_and_data(kbdc, PSMC_SET_SAMPLING_RATE, rate);
	VLOG(2, (LOG_DEBUG, "psm: SET_SAMPLING_RATE (%d) %04x\n", rate, res));

	return ((res == PSM_ACK) ? rate : -1);
}

/* `val' must be 0 through PSMD_MAX_RESOLUTION */
static int
set_mouse_resolution(KBDC kbdc, int val)
{
	int res;

	res = send_aux_command_and_data(kbdc, PSMC_SET_RESOLUTION, val);
	VLOG(2, (LOG_DEBUG, "psm: SET_RESOLUTION (%d) %04x\n", val, res));

	return ((res == PSM_ACK) ? val : -1);
}

/*
 * ***
 * Set up our queue
 */
static void
initqueue(struct psm_softc *sc)
{
	sc->rinpos = sc->routpos = sc->rqsize = 0;
}

static int
doopen(struct psm_softc *sc, int command_byte)
{
	int stat[3];

	/* enable the mouse device */
	if (!enable_aux_dev(sc->kbdc)) {
		/* MOUSE ERROR: failed to enable the mouse because:
		 * 1) the mouse is faulty, or
		 * 2) the mouse has been removed(!?)
		 */
		/* mark this device is no longer available */
		sc->state &= ~PSM_VALID;
		log(LOG_ERR,
		    "psm%d: failed to enable the device (doopen).\n",
			sc->unit);
		return (EIO);
	}

	if (get_mouse_status(sc->kbdc, stat, 0, 3) < 3)
		log(LOG_DEBUG, "psm%d: failed to get status (doopen).\n",
		    sc->unit);

	/* enable the aux port and interrupt */
	if (!set_controller_command_byte(sc->kbdc,
	    kbdc_get_device_mask(sc->kbdc),
	    (command_byte & KBD_KBD_CONTROL_BITS) |
	    KBD_ENABLE_AUX_PORT | KBD_ENABLE_AUX_INT)) {
		/* CONTROLLER ERROR */
		disable_aux_dev(sc->kbdc);
		log(LOG_ERR,
		    "psm%d: failed to enable the aux interrupt (doopen).\n",
		    sc->unit);
		return (EIO);
	}

	return (0);
}


/***
 *** psm driver entry points
 ***/

static void
psmidentify(driver_t *driver, device_t parent)
{
	device_t psmc;
	device_t psm;
	u_long irq;
	int unit;

	unit = device_get_unit(parent);

	/* always add at least one child */
	psm = BUS_ADD_CHILD(parent, KBDC_RID_AUX, driver->name, unit);
	if (psm == NULL)
		return;

	irq = bus_get_resource_start(psm, SYS_RES_IRQ, KBDC_RID_AUX);
	if (irq > 0)
		return;

	/*
	 * If the PS/2 mouse device has already been reported by ACPI or
	 * PnP BIOS, obtain the IRQ resource from it.
	 * (See psmcpnp_attach() below.)
	 */
	psmc = device_find_child(device_get_parent(parent),
	   	 PSMCPNP_DRIVER_NAME, unit);
	if (psmc == NULL)
		return;
	irq = bus_get_resource_start(psmc, SYS_RES_IRQ, 0);
	if (irq <= 0)
		return;
	bus_set_resource(psm, SYS_RES_IRQ, KBDC_RID_AUX, irq, 1);
}

#define	endprobe(v)	\
		do {						\
			kbdc_set_device_mask(sc->kbdc, mask);	\
			kbdc_lock(sc->kbdc, FALSE);		\
			return (v);				\
		} while (0)

static int
psmprobe(device_t dev)
{
	int unit = device_get_unit(dev);
	struct psm_softc *sc = device_get_softc(dev);
	int stat[3];
	int command_byte;
	int mask;
	int rid;
	int i;

	/* see if IRQ is available */
	rid = KBDC_RID_AUX;
	sc->intr = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid,
	    RF_SHAREABLE | RF_ACTIVE);
	if (sc->intr == NULL) {
		return (ENXIO);
	}
	bus_release_resource(dev, SYS_RES_IRQ, rid, sc->intr);

	sc->unit = unit;
	sc->kbdc = atkbdc_open(device_get_unit(device_get_parent(dev)));
	sc->config = device_get_flags(dev) & PSM_CONFIG_FLAGS;

	device_set_desc(dev, "PS/2 Mouse");

	if (!kbdc_lock(sc->kbdc, TRUE)) {
		printf("psm%d: unable to lock the controller.\n", unit);
		return (ENXIO);
	}

	/*
	 * NOTE: two bits in the command byte controls the operation of the
	 * aux port (mouse port): the aux port disable bit (bit 5) and the aux
	 * port interrupt (IRQ 12) enable bit (bit 2).
	 */

	/* discard anything left after the keyboard initialization */
	empty_both_buffers(sc->kbdc, 10);

	/* save the current command byte; it will be used later */
	mask = kbdc_get_device_mask(sc->kbdc) & ~KBD_AUX_CONTROL_BITS;
	command_byte = get_controller_command_byte(sc->kbdc);
	if (verbose)
		printf("psm%d: current command byte:%04x\n", unit,
		    command_byte);
	if (command_byte == -1) {
		/* CONTROLLER ERROR */
		printf("psm%d: unable to get the current command byte value.\n",
			unit);
		endprobe(ENXIO);
	}

	/*
	 * disable the keyboard port while probing the aux port, which must be
	 * enabled during this routine
	 */
	if (!set_controller_command_byte(sc->kbdc,
	    KBD_KBD_CONTROL_BITS | KBD_AUX_CONTROL_BITS,
	    KBD_DISABLE_KBD_PORT | KBD_DISABLE_KBD_INT |
	    KBD_ENABLE_AUX_PORT | KBD_DISABLE_AUX_INT)) {
		/*
		 * this is CONTROLLER ERROR; I don't know how to recover
		 * from this error...
		 */
		printf("psm%d: unable to set the command byte.\n", unit);
		endprobe(ENXIO);
	}
	write_controller_command(sc->kbdc, KBDC_ENABLE_AUX_PORT);

	/*
	 * NOTE: `test_aux_port()' is designed to return with zero if the aux
	 * port exists and is functioning. However, some controllers appears
	 * to respond with zero even when the aux port doesn't exist. (It may
	 * be that this is only the case when the controller DOES have the aux
	 * port but the port is not wired on the motherboard.) The keyboard
	 * controllers without the port, such as the original AT, are
	 * supporsed to return with an error code or simply time out. In any
	 * case, we have to continue probing the port even when the controller
	 * passes this test.
	 *
	 * XXX: some controllers erroneously return the error code 1, 2 or 3
	 * when it has the perfectly functional aux port. We have to ignore
	 * this error code. Even if the controller HAS error with the aux
	 * port, it will be detected later...
	 * XXX: another incompatible controller returns PSM_ACK (0xfa)...
	 */
	switch ((i = test_aux_port(sc->kbdc))) {
	case 1:		/* ignore these errors */
	case 2:
	case 3:
	case PSM_ACK:
		if (verbose)
			printf("psm%d: strange result for test aux port "
			    "(%d).\n", unit, i);
		/* FALLTHROUGH */
	case 0:		/* no error */
		break;
	case -1:	/* time out */
	default:	/* error */
		printf("psm%d: the aux port is not functioning (%d).\n",
			    unit, i);
		endprobe(ENXIO);
	}

	/*
	 * both the aux port and the aux device is functioning, see if the
	 * device can be enabled. NOTE: when enabled, the device will start
	 * sending data; we shall immediately disable the device once we know
	 * the device can be enabled.
	 */
	if (!enable_aux_dev(sc->kbdc) || !disable_aux_dev(sc->kbdc)) {
		/* MOUSE ERROR */
		printf("psm%d: failed to enable the aux device.\n", unit);
		endprobe(ENXIO);
	}

	/* save the default values after reset */
	if (get_mouse_status(sc->kbdc, stat, 0, 3) >= 3) {
		sc->dflt_mode.rate = sc->mode.rate = stat[2];
		sc->dflt_mode.resolution = sc->mode.resolution = stat[1];
	} else {
		sc->dflt_mode.rate = sc->mode.rate = -1;
		sc->dflt_mode.resolution = sc->mode.resolution = -1;
	}

	/* hardware information */
	sc->hw.iftype = MOUSE_IF_PS2;

	/* verify the device is a mouse */
	sc->hw.hwid = get_aux_id(sc->kbdc);

	sc->hw.buttons = 2;
	sc->hw.type = MOUSE_UNKNOWN;
	sc->hw.model = MOUSE_MODEL_GENERIC;

	sc->dflt_mode.level = 0;
	sc->dflt_mode.packetsize = MOUSE_PS2_PACKETSIZE;
	sc->dflt_mode.accelfactor = 1;
	if (sc->config & PSM_CONFIG_NOCHECKSYNC)
		sc->dflt_mode.syncmask[0] = 0;
	else
		sc->dflt_mode.syncmask[0] = 0xc0;

	sc->dflt_mode.syncmask[1] = 0;	/* syncbits */
	sc->mode = sc->dflt_mode;
	sc->mode.packetsize = MOUSE_PS2_PACKETSIZE;

	/* set mouse parameters */
	if (sc->config & PSM_CONFIG_RESOLUTION)
		sc->mode.resolution =
		    set_mouse_resolution(sc->kbdc,
		    (sc->config & PSM_CONFIG_RESOLUTION) - 1);
	else if (sc->mode.resolution >= 0)
		sc->mode.resolution =
		    set_mouse_resolution(sc->kbdc, sc->dflt_mode.resolution);

	if (sc->mode.rate > 0)
		sc->mode.rate =
		    set_mouse_sampling_rate(sc->kbdc, sc->dflt_mode.rate);


	/* just check the status of the mouse */
	/*
	 * NOTE: XXX there are some arcane controller/mouse combinations out
	 * there, which hung the controller unless there is data transmission
	 * after ACK from the mouse.
	 */
	if (get_mouse_status(sc->kbdc, stat, 0, 3) < 3)
		printf("psm%d: failed to get status.\n", unit);
	else {
		/*
		 * When in its native mode, some mice operate with different
		 * default parameters than in the PS/2 compatible mode.
		 */
		sc->dflt_mode.rate = sc->mode.rate = stat[2];
		sc->dflt_mode.resolution = sc->mode.resolution = stat[1];
	}

	/* disable the aux port for now... */
	if (!set_controller_command_byte(sc->kbdc,
	    KBD_KBD_CONTROL_BITS | KBD_AUX_CONTROL_BITS,
	    (command_byte & KBD_KBD_CONTROL_BITS) |
	    KBD_DISABLE_AUX_PORT | KBD_DISABLE_AUX_INT)) {
		/*
		 * this is CONTROLLER ERROR; I don't know the proper way to
		 * recover from this error...
		 */
		printf("psm%d: unable to set the command byte.\n", unit);
		endprobe(ENXIO);
	}

	/* done */
	kbdc_set_device_mask(sc->kbdc, mask | KBD_AUX_CONTROL_BITS);
	kbdc_lock(sc->kbdc, FALSE);
	return (0);
}

static int
psmattach(device_t dev)
{
	int unit = device_get_unit(dev);
	struct psm_softc *sc = device_get_softc(dev);
	int error;
	int rid;

	/* Setup initial state */
	sc->state = PSM_VALID;

	/* Setup our interrupt handler */
	rid = KBDC_RID_AUX;
	sc->intr = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid,
	    RF_SHAREABLE | RF_ACTIVE);
	if (sc->intr == NULL)
		return (ENXIO);
	error = bus_setup_intr(dev, sc->intr, INTR_TYPE_TTY, NULL, psmintr, sc,
	    &sc->ih);
	if (error) {
		bus_release_resource(dev, SYS_RES_IRQ, rid, sc->intr);
		return (error);
	}

	/* Done */
	sc->dev = make_dev(&psm_cdevsw, PSM_MKMINOR(unit, FALSE), 0, 0, 0666,
	    "psm%d", unit);
	sc->bdev = make_dev(&psm_cdevsw, PSM_MKMINOR(unit, TRUE), 0, 0, 0666,
	    "bpsm%d", unit);

	if (!verbose)
		printf("psm%d: model %s, device ID %d\n",
		    unit, PSM_MOUSE_MODEL_GENERIC, sc->hw.hwid & 0x00ff);
	else {
		printf("psm%d: model %s, device ID %d-%02x, %d buttons\n",
		    unit, PSM_MOUSE_MODEL_GENERIC, sc->hw.hwid & 0x00ff,
		    sc->hw.hwid >> 8, sc->hw.buttons);
		printf("psm%d: config:%08x, packet size:%d\n",
		    unit, sc->config, sc->mode.packetsize);
		printf("psm%d: syncmask:%02x, syncbits:%02x\n",
		    unit, sc->mode.syncmask[0], sc->mode.syncmask[1]);
	}

	return (0);
}

static int
psmdetach(device_t dev)
{
	struct psm_softc *sc;
	int rid;

	sc = device_get_softc(dev);
	if (sc->state & PSM_OPEN)
		return (EBUSY);

	rid = KBDC_RID_AUX;
	bus_teardown_intr(dev, sc->intr, sc->ih);
	bus_release_resource(dev, SYS_RES_IRQ, rid, sc->intr);

	destroy_dev(sc->dev);
	destroy_dev(sc->bdev);

	return (0);
}

static int
psmopen(struct cdev *dev, int flag, int fmt, struct thread *td)
{
	int unit = PSM_UNIT(dev);
	struct psm_softc *sc;
	int command_byte;
	int err;
	int s;

	printf("In psmopen()\n");

	/* Get device data */
	sc = PSM_SOFTC(unit);
	if ((sc == NULL) || (sc->state & PSM_VALID) == 0) {
		/* the device is no longer valid/functioning */
		return (ENXIO);
	}

	/* Disallow multiple opens */
	if (sc->state & PSM_OPEN)
		return (EBUSY);

	device_busy(devclass_get_device(psm_devclass, unit));

	/* Initialize state */
	sc->mode.level = sc->dflt_mode.level;
	sc->mode.protocol = sc->dflt_mode.protocol;
	sc->async = NULL;

	/* initialize event queue */
	initqueue(sc);

	/* don't let timeout routines in the keyboard driver to poll the kbdc */
	if (!kbdc_lock(sc->kbdc, TRUE))
		return (EIO);

	/* save the current controller command byte */
	s = spltty();
	command_byte = get_controller_command_byte(sc->kbdc);

	/* enable the aux port and temporalily disable the keyboard */
	if (command_byte == -1 || !set_controller_command_byte(sc->kbdc,
	    kbdc_get_device_mask(sc->kbdc),
	    KBD_DISABLE_KBD_PORT | KBD_DISABLE_KBD_INT |
	    KBD_ENABLE_AUX_PORT | KBD_DISABLE_AUX_INT)) {
		kbdc_lock(sc->kbdc, FALSE);
		splx(s);
		log(LOG_ERR,
		    "psm%d: CONTROLLER ERROR -- unable to set the command byte (psmopen).\n", unit);
		return (EIO);
	}
	/*
	 * Now that the keyboard controller is told not to generate
	 * the keyboard and mouse interrupts, call `splx()' to allow
	 * the other tty interrupts. The clock interrupt may also occur,
	 * but timeout routines will be blocked by the poll flag set
	 * via `kbdc_lock()'
	 */
	splx(s);

	/* enable the mouse device */
	err = doopen(sc, command_byte);

	/* done */
	if (err == 0)
		sc->state |= PSM_OPEN;
	kbdc_lock(sc->kbdc, FALSE);

	return (err);
}

static int
psmclose(struct cdev *dev, int flag, int fmt, struct thread *td)
{
	int unit = PSM_UNIT(dev);
	struct psm_softc *sc = PSM_SOFTC(unit);
	int stat[3];
	int command_byte;
	int s;

	VLOG(1, (LOG_DEBUG, "psm: psmclose()\n"));

	/* don't let timeout routines in the keyboard driver to poll the kbdc */
	if (!kbdc_lock(sc->kbdc, TRUE))
		return (EIO);

	/* save the current controller command byte */
	s = spltty();
	command_byte = get_controller_command_byte(sc->kbdc);
	if (command_byte == -1) {
		kbdc_lock(sc->kbdc, FALSE);
		splx(s);
		return (EIO);
	}

	/* disable the aux interrupt and temporalily disable the keyboard */
	if (!set_controller_command_byte(sc->kbdc,
	    kbdc_get_device_mask(sc->kbdc),
	    KBD_DISABLE_KBD_PORT | KBD_DISABLE_KBD_INT |
	    KBD_ENABLE_AUX_PORT | KBD_DISABLE_AUX_INT)) {
		log(LOG_ERR,
		    "psm%d: failed to disable the aux int (psmclose).\n", unit);
		/* CONTROLLER ERROR;
		 * NOTE: we shall force our way through. Because the only
		 * ill effect we shall see is that we may not be able
		 * to read ACK from the mouse, and it doesn't matter much
		 * so long as the mouse will accept the DISABLE command.
		 */
	}
	splx(s);

	/* remove anything left in the output buffer */
	empty_aux_buffer(sc->kbdc, 10);

	/* disable the aux device, port and interrupt */
	if (sc->state & PSM_VALID) {
		if (!disable_aux_dev(sc->kbdc)) {
			/* MOUSE ERROR;
			 * NOTE: we don't return (error) and continue,
			 * pretending we have successfully disabled the device.
			 * It's OK because the interrupt routine will discard
			 * any data from the mouse hereafter.
			 */
			log(LOG_ERR,
			    "psm%d: failed to disable the device (psmclose).\n",
			    unit);
		}

		if (get_mouse_status(sc->kbdc, stat, 0, 3) < 3)
			log(LOG_DEBUG,
			    "psm%d: failed to get status (psmclose).\n", unit);
	}

	if (!set_controller_command_byte(sc->kbdc,
	    kbdc_get_device_mask(sc->kbdc),
	    (command_byte & KBD_KBD_CONTROL_BITS) |
	    KBD_DISABLE_AUX_PORT | KBD_DISABLE_AUX_INT)) {
		/*
		 * CONTROLLER ERROR;
		 * we shall ignore this error; see the above comment.
		 */
		log(LOG_ERR,
		    "psm%d: failed to disable the aux port (psmclose).\n",
		    unit);
	}

	/* remove anything left in the output buffer */
	empty_aux_buffer(sc->kbdc, 10);

	/* clean up and sigio requests */
	if (sc->async != NULL) {
		funsetown(&sc->async);
		sc->async = NULL;
	}

	/* close is almost always successful */
	sc->state &= ~PSM_OPEN;
	kbdc_lock(sc->kbdc, FALSE);
	device_unbusy(devclass_get_device(psm_devclass, unit));
	return (0);
}

static int
psmread(struct cdev *dev, struct uio *uio, int flag)
{
	register struct psm_softc *sc = PSM_SOFTC(PSM_UNIT(dev));
	register int len, xfer;
	int s, error;

	/* *** ??? 
	 * Q2: What does this do?
	 - It gets into a critical section
	 */
	s = spltty();

	/*
	 * Wait for the queue to be non-empty.
	 */
	while (sc->rqsize == 0) {
		/*
		 *** ???
		 * Something (obviously) needs to go here
		 * Q3: describe what goes here, and why.
		 */
	}

	/*
	 * Copy data from the queue to the user program buffer.
	 * uiomove(bufferp, N, uiop) does the actual copy from kernel to user
	 * memory of N contiguous bytes pointed at by bufferp.
	 *
	 * *** ???
	 * Q4: Why is this a loop?
	 * 	Describe how this loop may run more than once.
	 */
	len = imin(sc->rqsize, uio->uio_resid);
	while (len > 0) {
		xfer = min(len, PSMQSIZE - sc->routpos);
		error = uiomove(&sc->rqueue[sc->routpos], xfer, uio);
		if (error) {
			splx(s);
			return (error);
		}
		sc->routpos = (sc->routpos + xfer) % PSMQSIZE;
		sc->rqsize -= xfer;
		len -= xfer;
	}
	splx(s);
	return (0);
}

static int
psmwrite(struct cdev *dev, struct uio *uio, int flag)
{
	printf("In psmwrite() -- returning fail\n");
	/*
	 * Just return an error.
	 */
	return (ENXIO);
}

static int
psmioctl(struct cdev *dev, u_long cmd, caddr_t addr, int flag,
    struct thread *td)
{
	struct psm_softc *sc = PSM_SOFTC(PSM_UNIT(dev));
	int error = 0;
	int s;

	printf("In psmioctl(cmd=%lu, addr=%p, flag=%d)\n",
			cmd, addr, flag);

	/* Perform IOCTL command */
	switch (cmd) {

	/** *** pass a value in/out of the driver using ioctl */
	case MOUSE_SETMODE:
		s = spltty();
		sc->localmode = *(int *)addr;
		splx(s);
		break;

	case MOUSE_GETMODE:
		s = spltty();
		*(int *)addr = sc->localmode;
		splx(s);
		break;

	default:
		return (ENOTTY);
	}

	return (error);
}

static int
psmpoll(struct cdev *dev, int events, struct thread *td)
{
	/* our driver is too simple to support polling */
	return (-1);
}

static int
psmresume(device_t dev)
{
	/* our driver doesn't try to make the mouse work after reinit */
	return (-1);
}



/* Add all sysctls under the debug.psm and hw.psm nodes */
SYSCTL_NODE(_debug, OID_AUTO, psm, CTLFLAG_RD, 0, "ps/2 mouse");
SYSCTL_NODE(_hw, OID_AUTO, psm, CTLFLAG_RD, 0, "ps/2 mouse");

SYSCTL_INT(_debug_psm, OID_AUTO, loglevel, CTLFLAG_RW, &verbose, 0, "");

static int psmhz = 20;
SYSCTL_INT(_debug_psm, OID_AUTO, hz, CTLFLAG_RW, &psmhz, 0, "");
static int psmerrsecs = 2;
SYSCTL_INT(_debug_psm, OID_AUTO, errsecs, CTLFLAG_RW, &psmerrsecs, 0, "");
static int psmerrusecs = 0;
SYSCTL_INT(_debug_psm, OID_AUTO, errusecs, CTLFLAG_RW, &psmerrusecs, 0, "");
static int psmsecs = 0;
SYSCTL_INT(_debug_psm, OID_AUTO, secs, CTLFLAG_RW, &psmsecs, 0, "");
static int psmusecs = 500000;
SYSCTL_INT(_debug_psm, OID_AUTO, usecs, CTLFLAG_RW, &psmusecs, 0, "");
static int pkterrthresh = 2;
SYSCTL_INT(_debug_psm, OID_AUTO, pkterrthresh, CTLFLAG_RW, &pkterrthresh,
    0, "");

static void
psmintr(void *arg)
{
	struct psm_softc *sc = arg;
	u_char byte;

	if ((sc->state & PSM_OPEN) == 0) {
		/*
		 * Stray interrupt, just discard the data.
		 */
		return;
	}

	/*
	 * Get the byte.
	 */
	byte = read_aux_data_no_wait(sc->kbdc);

	/*
	 * Queue the byte for psmread().
	 */
	if (sc->rqsize < PSMQSIZE) {

		/* *** ????
		 * Q5: What should be happening here?
		 */

	} else
		printf("psm: Rcv queue full, discarding data\n");

	return;
}


/**
 * register the psm_driver variable, which contains the
 * pointers to all of the relevant functions
 */
DRIVER_MODULE(psm, atkbdc, psm_driver, psm_devclass, 0, 0);

