/*
 * Copyright (C) 2024 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#define LOG_TAG "GnssHalRpi5"

#include "NmeaParser.h"

#include <cmath>
#include <cstdlib>
#include <ctime>
#include <log/log.h>
#include <sstream>
#include <utils/Timers.h>

namespace aidl::android::hardware::gnss::implementation {

NmeaParser::NmeaParser() {}

std::vector<std::string> NmeaParser::split(const std::string& s, char delim) {
    std::vector<std::string> result;
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        result.push_back(item);
    }
    return result;
}

bool NmeaParser::verifyChecksum(const std::string& sentence) {
    if (sentence.empty() || sentence[0] != '$') return false;

    size_t starPos = sentence.find('*');
    if (starPos == std::string::npos || starPos + 2 >= sentence.length()) {
        return true;  // No checksum, assume valid
    }

    uint8_t checksum = 0;
    for (size_t i = 1; i < starPos; i++) {
        checksum ^= sentence[i];
    }

    char expected[3];
    snprintf(expected, sizeof(expected), "%02X", checksum);
    return sentence.substr(starPos + 1, 2) == expected;
}

double NmeaParser::parseLatLon(const std::string& value, const std::string& dir) {
    if (value.empty()) return 0.0;

    // NMEA format: DDDMM.MMMM or DDMM.MMMM
    double raw = std::stod(value);
    int degrees = static_cast<int>(raw / 100);
    double minutes = raw - (degrees * 100);
    double result = degrees + (minutes / 60.0);

    if (dir == "S" || dir == "W") {
        result = -result;
    }
    return result;
}

bool NmeaParser::parse(const std::string& sentence) {
    if (!verifyChecksum(sentence)) {
        return false;
    }

    // Remove checksum for parsing
    std::string data = sentence;
    size_t starPos = data.find('*');
    if (starPos != std::string::npos) {
        data = data.substr(0, starPos);
    }

    std::vector<std::string> fields = split(data, ',');
    if (fields.empty()) return false;

    const std::string& type = fields[0];

    if (type == "$GPGGA" || type == "$GNGGA") {
        return parseGGA(fields);
    } else if (type == "$GPRMC" || type == "$GNRMC") {
        return parseRMC(fields);
    } else if (type == "$GPGSV" || type == "$GLGSV" || type == "$GAGSV") {
        return parseGSV(fields);
    } else if (type == "$GPGSA" || type == "$GNGSA") {
        return parseGSA(fields);
    } else if (type == "$GPVTG" || type == "$GNVTG") {
        return parseVTG(fields);
    }

    return false;
}

bool NmeaParser::parseGGA(const std::vector<std::string>& fields) {
    // $GPGGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh
    // 0: $GPGGA
    // 1: UTC time (hhmmss.ss)
    // 2: Latitude
    // 3: N/S
    // 4: Longitude
    // 5: E/W
    // 6: Fix quality (0=invalid, 1=GPS, 2=DGPS)
    // 7: Number of satellites
    // 8: HDOP
    // 9: Altitude
    // 10: M (meters)
    // 11: Geoid separation
    // 12: M
    // 13: Age of differential GPS data
    // 14: Differential reference station ID

    if (fields.size() < 10) return false;

    int fixQuality = 0;
    if (!fields[6].empty()) {
        fixQuality = std::stoi(fields[6]);
    }

    mData.fixQuality = fixQuality;

    if (fixQuality > 0 && !fields[2].empty() && !fields[4].empty()) {
        mData.latitude = parseLatLon(fields[2], fields[3]);
        mData.longitude = parseLatLon(fields[4], fields[5]);
        mData.hasLocation = true;

        if (!fields[7].empty()) {
            mData.numSatellites = std::stoi(fields[7]);
        }

        if (!fields[8].empty()) {
            float hdop = std::stof(fields[8]);
            mData.accuracy = hdop * 5.0f;  // Rough estimate
        }

        if (!fields[9].empty()) {
            mData.altitude = std::stod(fields[9]);
        }

        mData.timestampMs = systemTime(SYSTEM_TIME_REALTIME) / 1000000LL;
        return true;
    }

    return false;
}

bool NmeaParser::parseRMC(const std::vector<std::string>& fields) {
    // $GPRMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,x.x,x.x,ddmmyy,x.x,a*hh
    // 0: $GPRMC
    // 1: UTC time
    // 2: Status (A=active, V=void)
    // 3: Latitude
    // 4: N/S
    // 5: Longitude
    // 6: E/W
    // 7: Speed over ground (knots)
    // 8: Track angle (degrees)
    // 9: Date (ddmmyy)

    if (fields.size() < 10) return false;

    if (fields[2] != "A") {
        // Void - no fix
        return false;
    }

    if (!fields[3].empty() && !fields[5].empty()) {
        mData.latitude = parseLatLon(fields[3], fields[4]);
        mData.longitude = parseLatLon(fields[5], fields[6]);
        mData.hasLocation = true;

        if (!fields[7].empty()) {
            // Convert knots to m/s
            mData.speed = std::stof(fields[7]) * 0.514444f;
        }

        if (!fields[8].empty()) {
            mData.bearing = std::stof(fields[8]);
        }

        mData.timestampMs = systemTime(SYSTEM_TIME_REALTIME) / 1000000LL;
        return true;
    }

    return false;
}

bool NmeaParser::parseGSV(const std::vector<std::string>& /*fields*/) {
    // $GPGSV,x,x,xx,xx,xx,xxx,xx,...*hh
    // Satellite info - we'll skip detailed parsing for now
    return false;
}

bool NmeaParser::parseGSA(const std::vector<std::string>& /*fields*/) {
    // $GPGSA,A,3,xx,xx,...,x.x,x.x,x.x*hh
    // DOP and active satellites
    return false;
}

bool NmeaParser::parseVTG(const std::vector<std::string>& fields) {
    // $GPVTG,x.x,T,x.x,M,x.x,N,x.x,K*hh
    // 0: $GPVTG
    // 1: Track degrees true
    // 3: Track degrees magnetic
    // 5: Speed knots
    // 7: Speed km/h

    if (fields.size() < 8) return false;

    if (!fields[7].empty() && fields[7] != "") {
        // Speed in km/h, convert to m/s
        mData.speed = std::stof(fields[7]) / 3.6f;
    }

    if (!fields[1].empty() && fields[1] != "") {
        mData.bearing = std::stof(fields[1]);
    }

    return false;  // VTG alone doesn't provide location
}

GnssLocation NmeaParser::getLocation() const {
    GnssLocation location;

    location.gnssLocationFlags = 0;

    if (mData.hasLocation) {
        location.gnssLocationFlags |= GnssLocation::HAS_LAT_LONG;
        location.latitudeDegrees = mData.latitude;
        location.longitudeDegrees = mData.longitude;
    }

    if (mData.altitude != 0.0) {
        location.gnssLocationFlags |= GnssLocation::HAS_ALTITUDE;
        location.altitudeMeters = mData.altitude;
    }

    if (mData.speed > 0.0f) {
        location.gnssLocationFlags |= GnssLocation::HAS_SPEED;
        location.speedMetersPerSec = mData.speed;
    }

    if (mData.bearing > 0.0f) {
        location.gnssLocationFlags |= GnssLocation::HAS_BEARING;
        location.bearingDegrees = mData.bearing;
    }

    location.gnssLocationFlags |= GnssLocation::HAS_HORIZONTAL_ACCURACY;
    location.horizontalAccuracyMeters = mData.accuracy;

    location.timestampMillis = mData.timestampMs;

    location.elapsedRealtime.flags = ElapsedRealtime::HAS_TIMESTAMP_NS;
    location.elapsedRealtime.timestampNs = systemTime(SYSTEM_TIME_MONOTONIC);

    return location;
}

}  // namespace aidl::android::hardware::gnss::implementation
