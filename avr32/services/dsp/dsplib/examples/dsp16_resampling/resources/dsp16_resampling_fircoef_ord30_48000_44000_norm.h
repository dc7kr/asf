/*****************************************************************************
 *
 * \file
 *
 * \brief FIR coefficients for the dsp16_resampling function from the DPSLib.
 *        These coefficients have the following caracterizations:
 *          - Input sampling frequency: 48000 Hz
 *          - Output sampling frequency: 44000 Hz
 *          - Re-sampling order: 30
 *          - Coefficients are normalized
 *          - Memory foot print: 660 bytes
 *         It matches the following parameters:
 *         (dsp16_resampling_options_t::dsp16_custom_filter)
 *          - Cut-off frequency (fc_hz): 22000
 *          - Sampling frequency (fs_hz): ******
 *          - Actual FIR filter order (order): 330
 *
 * Copyright (c) 2010 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 ******************************************************************************/

A_ALIGNED static const dsp16_t dsp16_resampling_fircoef_ord30_48000_44000_norm[] = {
  DSP16_Q(-0.00000000),
  DSP16_Q(0.00007411),
  DSP16_Q(-0.00019852),
  DSP16_Q(0.00016207),
  DSP16_Q(0.00030637),
  DSP16_Q(-0.00149341),
  DSP16_Q(0.00364879),
  DSP16_Q(-0.00693501),
  DSP16_Q(0.01138963),
  DSP16_Q(-0.01691092),
  DSP16_Q(0.02327802),
  DSP16_Q(-0.03022431),
  DSP16_Q(0.03762611),
  DSP16_Q(-0.04613134),
  DSP16_Q(0.06133375),
  DSP16_Q(0.44375182),
  DSP16_Q(0.01906227),
  DSP16_Q(-0.02759790),
  DSP16_Q(0.02783586),
  DSP16_Q(-0.02515542),
  DSP16_Q(0.02106639),
  DSP16_Q(-0.01642618),
  DSP16_Q(0.01185826),
  DSP16_Q(-0.00781631),
  DSP16_Q(0.00458455),
  DSP16_Q(-0.00227701),
  DSP16_Q(0.00085307),
  DSP16_Q(-0.00015155),
  DSP16_Q(-0.00006135),
  DSP16_Q(0.00003828),
  DSP16_Q(-0.00000088),
  DSP16_Q(0.00011561),
  DSP16_Q(-0.00034712),
  DSP16_Q(0.00050723),
  DSP16_Q(-0.00032225),
  DSP16_Q(-0.00053110),
  DSP16_Q(0.00237810),
  DSP16_Q(-0.00549671),
  DSP16_Q(0.01007939),
  DSP16_Q(-0.01622279),
  DSP16_Q(0.02397120),
  DSP16_Q(-0.03347502),
  DSP16_Q(0.04548070),
  DSP16_Q(-0.06329742),
  DSP16_Q(0.10803566),
  DSP16_Q(0.43359242),
  DSP16_Q(-0.01749878),
  DSP16_Q(-0.00897423),
  DSP16_Q(0.01688229),
  DSP16_Q(-0.01871370),
  DSP16_Q(0.01757006),
  DSP16_Q(-0.01486719),
  DSP16_Q(0.01150287),
  DSP16_Q(-0.00811590),
  DSP16_Q(0.00514497),
  DSP16_Q(-0.00284232),
  DSP16_Q(0.00128745),
  DSP16_Q(-0.00041466),
  DSP16_Q(0.00005533),
  DSP16_Q(0.00001051),
  DSP16_Q(-0.00000378),
  DSP16_Q(0.00015892),
  DSP16_Q(-0.00049537),
  DSP16_Q(0.00085983),
  DSP16_Q(-0.00099278),
  DSP16_Q(0.00055342),
  DSP16_Q(0.00084062),
  DSP16_Q(-0.00356825),
  DSP16_Q(0.00796937),
  DSP16_Q(-0.01434081),
  DSP16_Q(0.02300531),
  DSP16_Q(-0.03456039),
  DSP16_Q(0.05070148),
  DSP16_Q(-0.07777813),
  DSP16_Q(0.15761803),
  DSP16_Q(0.41369663),
  DSP16_Q(-0.04739048),
  DSP16_Q(0.00856967),
  DSP16_Q(0.00555904),
  DSP16_Q(-0.01140811),
  DSP16_Q(0.01309526),
  DSP16_Q(-0.01239929),
  DSP16_Q(0.01039586),
  DSP16_Q(-0.00785034),
  DSP16_Q(0.00531835),
  DSP16_Q(-0.00316818),
  DSP16_Q(0.00158999),
  DSP16_Q(-0.00061431),
  DSP16_Q(0.00014556),
  DSP16_Q(-0.00000826),
  DSP16_Q(-0.00000856),
  DSP16_Q(0.00019894),
  DSP16_Q(-0.00062949),
  DSP16_Q(0.00119213),
  DSP16_Q(-0.00165798),
  DSP16_Q(0.00168970),
  DSP16_Q(-0.00087085),
  DSP16_Q(-0.00125651),
  DSP16_Q(0.00516138),
  DSP16_Q(-0.01132769),
  DSP16_Q(0.02034817),
  DSP16_Q(-0.03326155),
  DSP16_Q(0.05271590),
  DSP16_Q(-0.08828667),
  DSP16_Q(0.20832773),
  DSP16_Q(0.38488927),
  DSP16_Q(-0.07000820),
  DSP16_Q(0.02402778),
  DSP16_Q(-0.00536990),
  DSP16_Q(-0.00377430),
  DSP16_Q(0.00799631),
  DSP16_Q(-0.00923976),
  DSP16_Q(0.00865641),
  DSP16_Q(-0.00707379),
  DSP16_Q(0.00512049),
  DSP16_Q(-0.00325188),
  DSP16_Q(0.00175256),
  DSP16_Q(-0.00074392),
  DSP16_Q(0.00020663),
  DSP16_Q(-0.00001843),
  DSP16_Q(-0.00001427),
  DSP16_Q(0.00022970),
  DSP16_Q(-0.00073477),
  DSP16_Q(0.00147466),
  DSP16_Q(-0.00226634),
  DSP16_Q(0.00279759),
  DSP16_Q(-0.00264401),
  DSP16_Q(0.00129691),
  DSP16_Q(0.00181350),
  DSP16_Q(-0.00733022),
  DSP16_Q(0.01608473),
  DSP16_Q(-0.02951096),
  DSP16_Q(0.05112378),
  DSP16_Q(-0.09364156),
  DSP16_Q(0.25828753),
  DSP16_Q(0.34835582),
  DSP16_Q(-0.08511548),
  DSP16_Q(0.03660267),
  DSP16_Q(-0.01521638),
  DSP16_Q(0.00366181),
  DSP16_Q(0.00264940),
  DSP16_Q(-0.00564026),
  DSP16_Q(0.00643959),
  DSP16_Q(-0.00587192),
  DSP16_Q(0.00459177),
  DSP16_Q(-0.00310799),
  DSP16_Q(0.00177793),
  DSP16_Q(-0.00080307),
  DSP16_Q(0.00023883),
  DSP16_Q(-0.00002141),
  DSP16_Q(-0.00001926),
  DSP16_Q(0.00024495),
  DSP16_Q(-0.00079686),
  DSP16_Q(0.00167854),
  DSP16_Q(-0.00276585),
  DSP16_Q(0.00379256),
  DSP16_Q(-0.00435403),
  DSP16_Q(0.00392373),
  DSP16_Q(-0.00186793),
  DSP16_Q(-0.00257370),
  DSP16_Q(0.01041798),
  DSP16_Q(-0.02340670),
  DSP16_Q(0.04573602),
  DSP16_Q(-0.09283844),
  DSP16_Q(0.30558344),
  DSP16_Q(0.30558344),
  DSP16_Q(-0.09283844),
  DSP16_Q(0.04573602),
  DSP16_Q(-0.02340670),
  DSP16_Q(0.01041798),
  DSP16_Q(-0.00257370),
  DSP16_Q(-0.00186793),
  DSP16_Q(0.00392373),
  DSP16_Q(-0.00435403),
  DSP16_Q(0.00379256),
  DSP16_Q(-0.00276585),
  DSP16_Q(0.00167854),
  DSP16_Q(-0.00079686),
  DSP16_Q(0.00024495),
  DSP16_Q(-0.00001926),
  DSP16_Q(-0.00002141),
  DSP16_Q(0.00023883),
  DSP16_Q(-0.00080307),
  DSP16_Q(0.00177793),
  DSP16_Q(-0.00310799),
  DSP16_Q(0.00459177),
  DSP16_Q(-0.00587192),
  DSP16_Q(0.00643959),
  DSP16_Q(-0.00564026),
  DSP16_Q(0.00264940),
  DSP16_Q(0.00366181),
  DSP16_Q(-0.01521638),
  DSP16_Q(0.03660267),
  DSP16_Q(-0.08511548),
  DSP16_Q(0.34835582),
  DSP16_Q(0.25828753),
  DSP16_Q(-0.09364156),
  DSP16_Q(0.05112378),
  DSP16_Q(-0.02951096),
  DSP16_Q(0.01608473),
  DSP16_Q(-0.00733022),
  DSP16_Q(0.00181350),
  DSP16_Q(0.00129691),
  DSP16_Q(-0.00264401),
  DSP16_Q(0.00279759),
  DSP16_Q(-0.00226634),
  DSP16_Q(0.00147466),
  DSP16_Q(-0.00073477),
  DSP16_Q(0.00022970),
  DSP16_Q(-0.00001427),
  DSP16_Q(-0.00001843),
  DSP16_Q(0.00020663),
  DSP16_Q(-0.00074392),
  DSP16_Q(0.00175256),
  DSP16_Q(-0.00325188),
  DSP16_Q(0.00512049),
  DSP16_Q(-0.00707379),
  DSP16_Q(0.00865641),
  DSP16_Q(-0.00923976),
  DSP16_Q(0.00799631),
  DSP16_Q(-0.00377430),
  DSP16_Q(-0.00536990),
  DSP16_Q(0.02402778),
  DSP16_Q(-0.07000820),
  DSP16_Q(0.38488927),
  DSP16_Q(0.20832773),
  DSP16_Q(-0.08828667),
  DSP16_Q(0.05271590),
  DSP16_Q(-0.03326155),
  DSP16_Q(0.02034817),
  DSP16_Q(-0.01132769),
  DSP16_Q(0.00516138),
  DSP16_Q(-0.00125651),
  DSP16_Q(-0.00087085),
  DSP16_Q(0.00168970),
  DSP16_Q(-0.00165798),
  DSP16_Q(0.00119213),
  DSP16_Q(-0.00062949),
  DSP16_Q(0.00019894),
  DSP16_Q(-0.00000856),
  DSP16_Q(-0.00000826),
  DSP16_Q(0.00014556),
  DSP16_Q(-0.00061431),
  DSP16_Q(0.00158999),
  DSP16_Q(-0.00316818),
  DSP16_Q(0.00531835),
  DSP16_Q(-0.00785034),
  DSP16_Q(0.01039586),
  DSP16_Q(-0.01239929),
  DSP16_Q(0.01309526),
  DSP16_Q(-0.01140811),
  DSP16_Q(0.00555904),
  DSP16_Q(0.00856967),
  DSP16_Q(-0.04739048),
  DSP16_Q(0.41369663),
  DSP16_Q(0.15761803),
  DSP16_Q(-0.07777813),
  DSP16_Q(0.05070148),
  DSP16_Q(-0.03456039),
  DSP16_Q(0.02300531),
  DSP16_Q(-0.01434081),
  DSP16_Q(0.00796937),
  DSP16_Q(-0.00356825),
  DSP16_Q(0.00084062),
  DSP16_Q(0.00055342),
  DSP16_Q(-0.00099278),
  DSP16_Q(0.00085983),
  DSP16_Q(-0.00049537),
  DSP16_Q(0.00015892),
  DSP16_Q(-0.00000378),
  DSP16_Q(0.00001051),
  DSP16_Q(0.00005533),
  DSP16_Q(-0.00041466),
  DSP16_Q(0.00128745),
  DSP16_Q(-0.00284232),
  DSP16_Q(0.00514497),
  DSP16_Q(-0.00811590),
  DSP16_Q(0.01150287),
  DSP16_Q(-0.01486719),
  DSP16_Q(0.01757006),
  DSP16_Q(-0.01871370),
  DSP16_Q(0.01688229),
  DSP16_Q(-0.00897423),
  DSP16_Q(-0.01749878),
  DSP16_Q(0.43359242),
  DSP16_Q(0.10803566),
  DSP16_Q(-0.06329742),
  DSP16_Q(0.04548070),
  DSP16_Q(-0.03347502),
  DSP16_Q(0.02397120),
  DSP16_Q(-0.01622279),
  DSP16_Q(0.01007939),
  DSP16_Q(-0.00549671),
  DSP16_Q(0.00237810),
  DSP16_Q(-0.00053110),
  DSP16_Q(-0.00032225),
  DSP16_Q(0.00050723),
  DSP16_Q(-0.00034712),
  DSP16_Q(0.00011561),
  DSP16_Q(-0.00000088),
  DSP16_Q(0.00003828),
  DSP16_Q(-0.00006135),
  DSP16_Q(-0.00015155),
  DSP16_Q(0.00085307),
  DSP16_Q(-0.00227701),
  DSP16_Q(0.00458455),
  DSP16_Q(-0.00781631),
  DSP16_Q(0.01185826),
  DSP16_Q(-0.01642618),
  DSP16_Q(0.02106639),
  DSP16_Q(-0.02515542),
  DSP16_Q(0.02783586),
  DSP16_Q(-0.02759790),
  DSP16_Q(0.01906227),
  DSP16_Q(0.44375182),
  DSP16_Q(0.06133375),
  DSP16_Q(-0.04613134),
  DSP16_Q(0.03762611),
  DSP16_Q(-0.03022431),
  DSP16_Q(0.02327802),
  DSP16_Q(-0.01691092),
  DSP16_Q(0.01138963),
  DSP16_Q(-0.00693501),
  DSP16_Q(0.00364879),
  DSP16_Q(-0.00149341),
  DSP16_Q(0.00030637),
  DSP16_Q(0.00016207),
  DSP16_Q(-0.00019852),
  DSP16_Q(0.00007411),
  DSP16_Q(-0.00000000)
};
