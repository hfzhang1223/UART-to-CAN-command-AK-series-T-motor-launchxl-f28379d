/*
 * hw_config.h
 *
 *  Created on: 2023. 1. 9.
 *      Author: RIMLAB
 */

#ifndef HW_CONFIG_H_
#define HW_CONFIG_H_


#define I_SCALE 0.02014160156f  // Amps per A/D Count  ///¹Ù²ã¾ßÇØ
#define V_SCALE 0.012890625f     // Bus volts per A/D Count   ///¹Ù²ã¾ßÇØ
#define DTC_MAX 0.94f          // Max phase duty cycle
#define DTC_MIN 0.0f          // Min phase duty cycle
#define DTC_COMP .000f          /// deadtime compensation (100 ns / 25 us)  ///¹Ù²ã¾ßÇØ





#endif /* HW_CONFIG_H_ */
