/*
 Copyright (C) 2019 Jacek Tomasiak (gpl-3.0-or-later)
 Copyright (C) 2020-2025 Fredrik Öhrström (gpl-3.0-or-later)
 Copyright (C) 2021 Vincent Privat (gpl-3.0-or-later)

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

// NOTE: This is a hand-written port of upstream wmbusmeters' "izarv2" driver
// (see wmbusmeters/wmbusmeters drivers/src/izarv2.xmq), targeting this
// project's older, hand-coded MeterCommonImplementation driver framework.
//
// Upstream has migrated "izarv2" to a newer declarative driver engine
// (drivers/src/*.xmq + generated_database.cc) which is not present in this
// vendored, older codebase (based on wmbusmeters 1.19.0). This file
// reimplements the exact same field semantics and output format as
// upstream izarv2, reusing the already proven PRIOS decoding approach
// (LFSR decode + alarm bit extraction) already used by this project's
// "izar" driver (see driver_izar.cpp) - the underlying protocol decoding
// is identical, only the exposed field names/shapes differ:
//
//   total_m3            (same as izar)
//   target_m3            (izar: last_month_total_m3)
//   target_date           (izar: last_month_measure_date)
//   battery_y             (izar: remaining_battery_life_y)
//   manufacture_y         (izar: manufacture_year)
//   status                (izar: current_alarms + previous_alarms,
//                           combined here into a single space separated,
//                           UPPERCASE string, defaulting to "OK")
//
// This file MUST be named driver_izarv2.cpp (not merged into
// driver_izar.cpp): this project's Python build-time config validation
// (components/wmbus_common/__init__.py) auto-discovers valid `type:`
// values for wmbus_meter purely from driver_*.cpp filenames in this
// directory, not from what is actually registered in C++.

#include"meters_common_implementation.h"
#include"manufacturer_specificities.h"

namespace
{
    /** Contains all the booleans required to store the alarms of a PRIOS device. */
    struct IzarV2Alarms
    {
        bool leakage_currently;
        bool leakage_previously;
        bool meter_blocked;
        bool back_flow;
        bool underflow;
        bool overflow;
        bool submarine;
        bool sensor_fraud_currently;
        bool sensor_fraud_previously;
        bool mechanical_fraud_currently;
        bool mechanical_fraud_previously;
    };

    struct DriverV2 : public virtual MeterCommonImplementation
    {
        DriverV2(MeterInfo &mi, DriverInfo &di);

        void processContent(Telegram *t);

    private:

        std::string statusText(IzarV2Alarms &alarms);

        std::vector<uchar> decodePrios(const std::vector<uchar> &origin, const std::vector<uchar> &payload, uint32_t key);

        std::vector<uint32_t> keys;
    };

    static bool ok = registerDriver([](DriverInfo&di)
    {
        di.setName("izarv2");
        di.setDefaultFields("name,id,status,total_m3,target_m3,timestamp");
        di.setMeterType(MeterType::WaterMeter);
        di.addLinkMode(LinkMode::T1);
        di.addDetection(MANUFACTURER_HYD,  0x07,  0x85);
        di.addDetection(MANUFACTURER_SAP,  0x15,    -1);
        di.addDetection(MANUFACTURER_SAP,  0x04,    -1);
        di.addDetection(MANUFACTURER_SAP,  0x07,  0x00);
        di.addDetection(MANUFACTURER_DME,  0x07,  0x78);
        di.addDetection(MANUFACTURER_DME,  0x06,  0x78);
        di.addDetection(MANUFACTURER_HYD,  0x07,  0x86);
        di.usesProcessContent();

        di.setConstructor([](MeterInfo& mi, DriverInfo& di){ return std::shared_ptr<Meter>(new DriverV2(mi, di)); });
    });

    DriverV2::DriverV2(MeterInfo &mi, DriverInfo &di) : MeterCommonImplementation(mi, di)
    {
        initializeDiehlDefaultKeySupport(meterKeys()->confidentiality_key, keys);

        addStringField("prefix",
                       "The alphanumeric prefix printed before serial number on device.",
                       DEFAULT_PRINT_PROPERTIES);

        addStringField("serial_number",
                       "The meter serial number.",
                       DEFAULT_PRINT_PROPERTIES);

        addStringField("manufacture_y",
                       "The year during which the meter was manufactured.",
                       DEFAULT_PRINT_PROPERTIES);

        addNumericField("total",
                        Quantity::Volume,
                        DEFAULT_PRINT_PROPERTIES,
                        "The total water consumption recorded by this meter.");

        addNumericField("target",
                        Quantity::Volume,
                        DEFAULT_PRINT_PROPERTIES,
                        "The total water consumption recorded at the end of last month.");

        addStringField("target_date",
                       "The date when the meter recorded the most recent billing value.",
                       DEFAULT_PRINT_PROPERTIES);

        addNumericField("battery",
                        Quantity::Time,
                        DEFAULT_PRINT_PROPERTIES,
                        "How many more years the battery is expected to last",
                        Unit::Year);

        addNumericField("transmit_period", Quantity::Time,
                        DEFAULT_PRINT_PROPERTIES,
                        "The period at which the meter transmits its data.",
                        Unit::Second);

        addStringField("status",
                       "Current and previous alarms reported by the meter (space separated, OK if none).",
                       DEFAULT_PRINT_PROPERTIES);
    }

    // Mirrors upstream izarv2.xmq's "status" lookup: a single, space separated,
    // UPPERCASE list of currently-set alarm flags (declaration order matches
    // the upstream ALARMS lookup table), defaulting to "OK" when none are set.
    std::string DriverV2::statusText(IzarV2Alarms &alarms)
    {
        std::string s;
        if (alarms.leakage_currently)             s.append("LEAKAGE ");
        if (alarms.meter_blocked)                 s.append("METER_BLOCKED ");
        if (alarms.back_flow)                     s.append("BACK_FLOW ");
        if (alarms.underflow)                     s.append("UNDERFLOW ");
        if (alarms.overflow)                      s.append("OVERFLOW ");
        if (alarms.submarine)                     s.append("SUBMARINE ");
        if (alarms.sensor_fraud_currently)        s.append("SENSOR_FRAUD ");
        if (alarms.mechanical_fraud_currently)    s.append("MECHANICAL_FRAUD ");
        if (alarms.leakage_previously)            s.append("PREV_LEAKAGE ");
        if (alarms.sensor_fraud_previously)       s.append("PREV_SENSOR_FRAUD ");
        if (alarms.mechanical_fraud_previously)   s.append("PREV_MECHANICAL_FRAUD ");

        if (s.length() > 0) {
            s.pop_back(); // remove trailing space
            return s;
        }
        return "OK";
    }

    void DriverV2::processContent(Telegram *t)
    {
        std::vector<uchar> frame;
        t->extractFrame(&frame);
        std::vector<uchar> origin = t->original.empty() ? frame : t->original;

        std::vector<uchar> decoded_content;
        for (auto& key : keys) {
            decoded_content = decodePrios(origin, frame, key);
            if (!decoded_content.empty())
                break;
        }

        debug("(izarv2) Decoded PRIOS data: %s\n", bin2hex(decoded_content).c_str());

        if (decoded_content.empty())
        {
            if (t->beingAnalyzed() == false)
            {
                warning("(izarv2) Decoding PRIOS data failed. Ignoring telegram.\n");
            }
            return;
        }

        if (detectDiehlFrameInterpretation(frame) == DiehlFrameInterpretation::SAP_PRIOS)
        {
            std::string digits = std::to_string((origin[7] & 0x03) << 24 | origin[6] << 16 | origin[5] << 8 | origin[4]);
            digits = tostrprintf("%08d", atoi(digits.c_str())); // Make sure we are on 8 digits for 200x years
            // get the manufacture year
            uint8_t yy = atoi(digits.substr(0, 2).c_str());
            int manufacture_year = yy > 70 ? (1900 + yy) : (2000 + yy); // Maybe to adjust in 2070, if this code stills lives :D
            setStringValue("manufacture_y", tostrprintf("%d", manufacture_year), NULL);

            // get the serial number
            uint32_t serial_number = atoi(digits.substr(2, digits.size()).c_str());
            setStringValue("serial_number", tostrprintf("%06d", serial_number), NULL);

            // get letters
            uchar supplier_code = '@' + (((origin[9] & 0x0F) << 1) | (origin[8] >> 7));
            uchar meter_type = '@' + ((origin[8] & 0x7C) >> 2);
            uchar diameter = '@' + (((origin[8] & 0x03) << 3) | (origin[7] >> 5));
            // build the prefix
            std::string prefix = tostrprintf("%c%02d%c%c", supplier_code, yy, meter_type, diameter);
            setStringValue("prefix", prefix, NULL);
        }

        // get the remaining battery life (in year) and transmission period (in seconds)
        double remaining_battery_life = (frame[12] & 0x1F) / 2.0;
        setNumericValue("battery", Unit::Year, remaining_battery_life);

        int transmit_period_s = 1 << ((frame[11] & 0x0F) + 2);
        setNumericValue("transmit_period", Unit::Second, transmit_period_s);

        double total_water_consumption_l_ = uint32FromBytes(decoded_content, 1, true);
        setNumericValue("total", Unit::L, total_water_consumption_l_);

        if (decoded_content.size() > 8) {
            double last_month_total_water_consumption_l_ = uint32FromBytes(decoded_content, 5, true);
            setNumericValue("target", Unit::L, last_month_total_water_consumption_l_);
        }

        // get the date when the second measurement was taken
        if (decoded_content.size() > 10) {
            uint16_t h0_year = ((decoded_content[10] & 0xF0) >> 1) + ((decoded_content[9] & 0xE0) >> 5);
            if (h0_year > 80) {
                h0_year += 1900;
            } else {
                h0_year += 2000;
            }
            uint8_t h0_month = decoded_content[10] & 0xF;
            uint8_t h0_day = decoded_content[9] & 0x1F;

            setStringValue("target_date", tostrprintf("%d-%02d-%02d", h0_year, h0_month%99, h0_day%99), NULL);
        }

        // read the alarms:
        IzarV2Alarms alarms {};

        alarms.leakage_currently = frame[12] >> 7;
        alarms.leakage_previously = frame[12] >> 6 & 0x1;
        alarms.meter_blocked = frame[12] >> 5 & 0x1;
        alarms.back_flow = frame[13] >> 7;
        alarms.underflow = frame[13] >> 6 & 0x1;
        alarms.overflow = frame[13] >> 5 & 0x1;
        alarms.submarine = frame[13] >> 4 & 0x1;
        alarms.sensor_fraud_currently = frame[13] >> 3 & 0x1;
        alarms.sensor_fraud_previously = frame[13] >> 2 & 0x1;
        alarms.mechanical_fraud_currently = frame[13] >> 1 & 0x1;
        alarms.mechanical_fraud_previously = frame[13] & 0x1;

        setStringValue("status", statusText(alarms));
    }

    std::vector<uchar> DriverV2::decodePrios(const std::vector<uchar> &origin, const std::vector<uchar> &frame, uint32_t key)
    {
        return decodeDiehlLfsr(origin, frame, key, DiehlLfsrCheckMethod::HEADER_1_BYTE, 0x4B);
    }
}

// Test: IzarWater izarv2 21242472 NOKEY
// telegram=|1944304C72242421D401A2_013D4013DD8B46A4999C1293E582CC|
// {"_":"telegram","media":"water","driver":"izarv2","name":"IzarWater","id":"21242472","prefix":"C19UA","serial_number":"145842","manufacture_y":"2019","total_m3":3.488,"target_m3":3.486,"target_date":"2019-09-30","battery_y":14.5,"transmit_period_s":8,"status":"METER_BLOCKED UNDERFLOW","timestamp":"1111-11-11T11:11:11Z"}

// Test: IzarWater2 izarv2 66236629 NOKEY
// telegram=|2944A511780729662366A20118001378D3B3DB8CEDD77731F25832AAF3DA8CADF9774EA673172E8C61F2|
// {"_":"telegram","media":"water","driver":"izarv2","name":"IzarWater2","id":"66236629","total_m3":16.76,"target_m3":11.84,"target_date":"2019-11-30","battery_y":12,"transmit_period_s":8,"status":"OK","timestamp":"1111-11-11T11:11:11Z"}

// Test: IzarWater3 izarv2 20481979 NOKEY
// telegram=|1944A511780779194820A1_21170013355F8EDB2D03C6912B1E37
// {"_":"telegram","media":"water","driver":"izarv2","name":"IzarWater3","id":"20481979","total_m3":4.366,"target_m3":0,"target_date":"2020-12-31","battery_y":11.5,"transmit_period_s":8,"status":"OK","timestamp":"1111-11-11T11:11:11Z"}

// Test: IzarWater4 izarv2 2124589c NOKEY
// Comment: With mfct specific tpl ci field a3.
// telegram=|1944304c9c5824210c04a363140013716577ec59e8663ab0d31c|
// {"_":"telegram","media":"water","driver":"izarv2","name":"IzarWater4","id":"2124589c","prefix":"H19CA","serial_number":"159196","manufacture_y":"2019","total_m3":38.944,"target_m3":38.691,"target_date":"2021-02-01","battery_y":10,"transmit_period_s":32,"status":"OK","timestamp":"1111-11-11T11:11:11Z"}

// Test: IzarWater5 izarv2 20e4ffde NOKEY
// Comment: Ensure non-regression on manufacture year parsing
// telegram=|1944304CDEFFE420CC01A2_63120013258F907B0AFF12529AC33B|
// {"_":"telegram","media":"water","driver":"izarv2","name":"IzarWater5","id":"20e4ffde","prefix":"C15SA","serial_number":"007710","manufacture_y":"2015","total_m3":159.832,"target_m3":157.76,"target_date":"2021-02-01","battery_y":9,"transmit_period_s":32,"status":"OK","timestamp":"1111-11-11T11:11:11Z"}

// Test: IzarWater6 izarv2 48500375 NOKEY
// telegram=|19442423860775035048A251520015BEB6B2E1ED623A18FC74A5|
// {"_":"telegram","media":"water","driver":"izarv2","name":"IzarWater6","id":"48500375","total_m3":521.602,"target_m3":519.147,"target_date":"2021-11-15","battery_y":9,"transmit_period_s":8,"status":"PREV_LEAKAGE","timestamp":"1111-11-11T11:11:11Z"}

KEEP_DRIVER(izarv2);
