/*
 * fft.h
 *
 */

#ifndef INC_FFT_H_
#define INC_FFT_H_

#include <stdint.h>

/**
 * \brief Calculates the PDS of a given time domain signal
 * \param in input, time domain signal.
 * \param out output, out[0] to out[length / 2 - 1] contains the power density spectrum of in[] in dB
 * \param length of in[]. Must be a power of 2. The size of out[] must be at least length / 2 - 1.
 *
 * \note Be careful, only rudimentary error checking is performed.
 * \see http://en.wikipedia.org/wiki/Spectral_density
 */
void power_density_spectrum(float in[], float out[], int length);

/**
 * \brief Calculates the PDS of a given time domain signal
 * \param in input, time domain signal.
 * \param out output, contains the power density spectrum of in[] in dB.
 * \param length of in[]. Must be a power of 2. The size of out[] must be at least length/2 - 1.
 *
 * This is a convenience function that calls power_density_spectrum().
 *
 * \note Be careful, only rudimentary error checking is performed.
 * \see http://en.wikipedia.org/wiki/Spectral_density
 */
void easy_pds(uint32_t in[], float out[], int length);

/**
 * \brief This function calculates the weights for a blackman window
 */
float blackman_window(float alpha, int position, int length);


#endif /* INC_FFT_H_ */
