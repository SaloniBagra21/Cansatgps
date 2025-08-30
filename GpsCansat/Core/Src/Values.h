/*
 * Values.h
 *
 *  Created on: Jun 25, 2025
 *      Author: Saloni
 */

#ifndef SRC_VALUES_H_
#define SRC_VALUES_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern char latitude[8];
extern char longitude[9];
extern char altitude[3];
//extern char speed[6];
//extern char course[6];

extern char time[10];       // HHMMSS
extern char NOS[2];        // Number of satellites

extern char Checksum[3];
extern char NSIndicator[1];
extern char EWIndicator[1];
extern char FixQuality[1];
extern char AltUnit[1];
extern char HDOP[3];
extern char GeoidSeparation[4];
extern char DGPSAge[4];
extern char DGPSStationID[4];
extern char GeoidUnit[1];

// ---------------- Function Prototypes ----------------
// Now they just take the relevant field string(s)

void getTime(const char *field);
void getLocation(const char *latField, const char *lonField);
void getIndicator(const char *nsField, const char *ewField);
void getFixQuality(const char *field);
void getNoOfSatellites(const char *field);
void getHDOP(const char *field);
void getAltitude(const char *field);
void getAltitudeUnit(const char *field);
void getGeoidSeparation(const char *field);
void getGeoidUnit(const char *field);
void getDGPSAge(const char *field);
void getDGPSStationID(const char *field);


// Optional (if you later use RMC sentences etc.):
// void getSpeedandCourse(const char *speedField, const char *courseField);
// void getStatus(const char *field);
// void getDate(const char *field);
// void getMagneticVariation(const char *field);
// void getMagVarDirection(const char *field);

#endif /* SRC_VALUES_H_ */
