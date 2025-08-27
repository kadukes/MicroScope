/*
 * plot.h
 *
 */

#ifndef INC_PLOT_H_
#define INC_PLOT_H_

#include <stdint.h>
#include <stddef.h>

#define max(a, b) (a >= b ? a : b)

/**
 * \brief Plots a <b>time signal</b> of given length to framebuffer
 *
 * @param data  A pointer of data values
 * @param length The size of the data array
 * @param foreground The foreground color
 * @param background The background color
 *
 */
void plot(uint32_t data[], size_t length, uint32_t foreground,
		  uint32_t background);

/**
 * \brief plot an previously calculated <b>power density spectrum</b>
 *
 * @param spectrum The caculated pds data
 * @param length The length of the pds array
 * @param foreground The framebuffer foreground color
 * @param background The framebuffer background color
 *
 */
void plot_pds(float spectrum[], size_t length, uint32_t foreground,
		      uint32_t background);

#endif /* INC_PLOT_H_ */
