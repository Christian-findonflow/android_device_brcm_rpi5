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

#pragma once

#include <aidl/android/hardware/gnss/GnssLocation.h>
#include <aidl/android/hardware/gnss/IGnssCallback.h>
#include <string>
#include <vector>

namespace aidl::android::hardware::gnss::implementation {

struct NmeaData {
    bool hasLocation = false;
    double latitude = 0.0;
    double longitude = 0.0;
    double altitude = 0.0;
    float speed = 0.0;        // m/s
    float bearing = 0.0;
    float accuracy = 10.0;    // meters
    int64_t timestampMs = 0;
    int fixQuality = 0;       // 0=invalid, 1=GPS, 2=DGPS
    int numSatellites = 0;
    std::vector<IGnssCallback::GnssSvInfo> satellites;
};

class NmeaParser {
  public:
    NmeaParser();

    // Parse a single NMEA sentence, returns true if location was updated
    bool parse(const std::string& sentence);

    // Get current parsed data
    const NmeaData& getData() const { return mData; }

    // Check if we have a valid fix
    bool hasValidFix() const { return mData.hasLocation && mData.fixQuality > 0; }

    // Build GnssLocation from current data
    GnssLocation getLocation() const;

  private:
    bool parseGGA(const std::vector<std::string>& fields);
    bool parseRMC(const std::vector<std::string>& fields);
    bool parseGSV(const std::vector<std::string>& fields);
    bool parseGSA(const std::vector<std::string>& fields);
    bool parseVTG(const std::vector<std::string>& fields);

    std::vector<std::string> split(const std::string& s, char delim);
    double parseLatLon(const std::string& value, const std::string& dir);
    bool verifyChecksum(const std::string& sentence);

    NmeaData mData;
};

}  // namespace aidl::android::hardware::gnss::implementation
