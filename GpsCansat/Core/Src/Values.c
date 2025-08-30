/*
 * Values.c
 *
 *  Created on: Jun 25, 2025
 *      Author: Saloni
 */
#include "Values.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

void getTime(const char *field) {
    // Example: "085601.000"
    if (field == NULL || strlen(field) < 6) {
        strcpy(time, "000000"); // fallback
        return;
    }
    strncpy(time, field, 6);   // HHMMSS
    time[6] = '\0';
}

void getLocation(const char *latField, const char *lonField) {
    if (latField) {
        strncpy(latitude, latField, sizeof(latitude));
        latitude[sizeof(latitude) - 1] = '\0';
    }
    if (lonField) {
        strncpy(longitude, lonField, sizeof(longitude));
        longitude[sizeof(longitude) - 1] = '\0';
    }
}

void getIndicator(const char *nsField, const char *ewField) {
    if (nsField) {
        strncpy(NSIndicator, nsField, sizeof(NSIndicator));
        NSIndicator[sizeof(NSIndicator) - 1] = '\0';
    }
    if (ewField) {
        strncpy(EWIndicator, ewField, sizeof(EWIndicator));
        EWIndicator[sizeof(EWIndicator) - 1] = '\0';
    }
}

void getFixQuality(const char *field) {
    if (field) {
        strncpy(FixQuality, field, sizeof(FixQuality));
        FixQuality[sizeof(FixQuality) - 1] = '\0';
    }
}

void getNoOfSatellites(const char *field) {
    if (field) {
        strncpy(NOS, field, sizeof(NOS));
        NOS[sizeof(NOS) - 1] = '\0';
    }
}

void getHDOP(const char *field) {
    if (field) {
        strncpy(HDOP, field, sizeof(HDOP));
        HDOP[sizeof(HDOP) - 1] = '\0';
    }
}

void getAltitude(const char *field) {
    if (field) {
        strncpy(altitude, field, sizeof(altitude));
        altitude[sizeof(altitude) - 1] = '\0';
    }
}

void getAltitudeUnit(const char *field) {
    if (field) {
        strncpy(AltUnit, field, sizeof(AltUnit));
        AltUnit[sizeof(AltUnit) - 1] = '\0';
    }
}

void getGeoidSeparation(const char *field) {
    if (field) {
        strncpy(GeoidSeparation, field, sizeof(GeoidSeparation));
        GeoidSeparation[sizeof(GeoidSeparation) - 1] = '\0';
    }
}

void getGeoidUnit(const char *field) {
    if (field) {
        strncpy(GeoidUnit, field, sizeof(GeoidUnit));
        GeoidUnit[sizeof(GeoidUnit) - 1] = '\0';
    }
}

void getDGPSAge(const char *field) {
    if (field) {
        strncpy(DGPSAge, field, sizeof(DGPSAge));
        DGPSAge[sizeof(DGPSAge) - 1] = '\0';
    }
}

void getDGPSStationID(const char *field) {
    if (field) {
        // Some NMEA strings here have "0000*67", so split on '*'
        char tmp[20];
        strncpy(tmp, field, sizeof(tmp));
        tmp[sizeof(tmp) - 1] = '\0';

        char *star = strchr(tmp, '*');
        if (star) *star = '\0';   // strip checksum

        strncpy(DGPSStationID, tmp, sizeof(DGPSStationID));
        DGPSStationID[sizeof(DGPSStationID) - 1] = '\0';

        if (star) {
            // Copy checksum separately
            strncpy(Checksum, star + 1, sizeof(Checksum));
            Checksum[sizeof(Checksum) - 1] = '\0';
        }
    }
}
