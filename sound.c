/*---------------------------------------------------------------------------
 *	This program is free software; you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation; either version 2 of the License, or
 *	(at your option) any later version. See also the license.txt file for
 *	additional informations.
 *---------------------------------------------------------------------------
 */

/* sound.cpp: implementation of the sound class. */

#include "types.h"

#ifdef DRZ80
#include "DrZ80_support.h"
#else
#if defined(CZ80)
#include "cz80_support.h"
#else
#include "z80.h"
#endif
#endif

#include <math.h>

#define Machine (&m_emuInfo)

#include "neopopsound.h"
#include "neopop_blip.h"

int sndCycles = 0;

int neopop_audio_accurate = 0; /* 0 = fast per-sample, 1 = band-limited Blip */

/* Re-derive the Blip synth parameters from the live chip register state, then
 * advance the band-limited synth by 'cycles' chip cycles. Keeping the Blip path
 * a pure observer of toneChip/noiseChip avoids duplicating the register decode. */
static void neopop_blip_sync_from_chips(void)
{
   int c, vol, div, ctrl, nfreq;

   for (c = 0; c < 3; c++)
   {
      div = toneChip.Register[c * 2] & 0x3FF;       /* 10-bit tone divisor */
      vol = toneChip.Register[c * 2 + 1] & 0x0F;    /* volume index */
      neopop_blip_sync_tone(c, div, vol);
   }

   ctrl = noiseChip.Register[6] & 0x07;             /* noise mode/rate */
   vol  = noiseChip.Register[7] & 0x0F;
   /* rate select: 0,1,2 => fixed shift rates; 3 => track tone 2. */
   switch (ctrl & 0x03)
   {
      case 0:  nfreq = 16;  break;
      case 1:  nfreq = 32;  break;
      case 2:  nfreq = 64;  break;
      default: nfreq = (toneChip.Register[4] & 0x3FF); break;
   }
   neopop_blip_sync_noise(nfreq, vol, (ctrl & 0x04) ? 1 : 0);
}

void soundStep(int cycles)
{
   sndCycles+= cycles;

   if (neopop_audio_accurate)
   {
      neopop_blip_sync_from_chips();
      neopop_blip_run(cycles);
   }
}

/*
 *
 * Neogeo Pocket Sound system
 *
 */

unsigned int	ngpRunning;

void ngpSoundStart(void)
{
   ngpRunning = 1;	/* ? */
#if defined(DRZ80) || defined(CZ80)
   Z80_Reset();
#else
   z80Init();
   z80SetRunning(1);
#endif
}

/* Execute all gained cycles (divided by 2) */
void ngpSoundExecute(void)
{
#if defined(DRZ80) || defined(CZ80)
   int toRun = sndCycles/2;
   if(ngpRunning)
      Z80_Execute(toRun);
   sndCycles -= toRun;
#else
   int		elapsed;
   while(sndCycles > 0)
   {
      elapsed = z80Step();
      sndCycles-= (2*elapsed);
   }
#endif
}

/* Switch sound system off */
void ngpSoundOff(void)
{
   ngpRunning = 0;
#if defined(DRZ80) || defined(CZ80)

#else
   z80SetRunning(0);
#endif
}

/* Generate interrupt to ngp sound system */
void ngpSoundInterrupt(void)
{
   if (ngpRunning)
   {
#if defined(DRZ80) || defined(CZ80)
      Z80_Cause_Interrupt(0x100); /* Z80_IRQ_INT??? */
#else
      z80Interrupt(0);
#endif
   }
}
