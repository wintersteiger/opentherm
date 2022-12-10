#ifndef _OPENTHERM_DATA_H_
#define _OPENTHERM_DATA_H_

#include <opentherm/transport.h>

// OpenTherm 2.2 application layer

namespace OpenTherm {

class Application {
public:
  Application(DeviceBase &device) : device(device) {}
  virtual ~Application() = default;

  enum RWSpec { RO, WO, RW };

  enum Type {
    flag8_flag8,
    F88,
    flag8_u8,
    u8_u8,
    s8_s8,
    u16,
    s16,
    special_u8,
    flag8_
  };

  struct ID {
    ID(uint8_t nr, RWSpec msg, std::string data_object, Type type,
       std::string description, ID *index[256])
        : nr(nr), msg(msg), data_object(data_object), type(type),
          description(description) {
      index[nr] = this;
    }

    virtual ~ID() = default;

    uint8_t nr;
    RWSpec msg;
    std::string data_object;
    Type type;
    std::string description;

    uint16_t value = 0;

    void set(uint16_t v) { value = v; }

    const char * to_string() {
      switch (type) {
        case flag8_flag8: break;
        case F88: break;
        case flag8_u8: break;
        case u8_u8: break;
        case s8_s8: break;
        case u16: break;
        case s16: break;
        case special_u8: break;
        case flag8_: break;
        default: break;
      }
      return "";
    }
  };

  DeviceBase &device;
  ID *index[256] = {0};

#define DID(NR, MSG, FNAME, OBJECT, TYPE, DESC)                                \
  ID FNAME = ID(NR, MSG, OBJECT, TYPE, DESC, index);

  // Ids 0 .. 127 are reserved for OpenTherm pre-defined information, while
  // id’s from 128 .. 255 can be used by manufacturers (members of the
  // association) for test & diagnostic purposes only.

  // clang-format off
  DID(  0, RO, status, "Status", flag8_flag8, "Master and Slave Status flags.");
  DID(  1, WO, tset, "TSet", F88, "Control setpoint ie CH water temperature setpoint (°C)");
  DID(  2, WO, mconfig_mmemberid, "M-Config / M-MemberIDcode", flag8_u8, "Master Configuration Flags / Master MemberID Code");
  DID(  3, RO, sconfig_smemberid, "S-Config / S-MemberIDcode", flag8_u8, "Slave Configuration Flags / Slave MemberID Code");
  DID(  4, WO, command, "Command", u8_u8, "Remote Command");
  DID(  5, RO, asf_flags, "ASF-flags / OEM-fault-code", flag8_u8, "Application-specific fault flags and OEM fault code");
  DID(  6, RO, rbp_flags, "RBP-flags", flag8_flag8, "Remote boiler parameter transfer-enable & read/write flags");
  DID(  7, WO, cooling_protocol, "Cooling-control", F88, "Cooling control signal (%)");
  DID(  8, WO, tsetch2, "TsetCH2", F88, "Control setpoint for 2^e CH circuit (°C)");
  DID(  9, RO, troverride, "TrOverride", F88, "Remote override room setpoint");
  DID( 10, RO, tsp, "TSP", u8_u8, "Number of Transparent-Slave-Parameters supported by slave");
  DID( 11, RW, tsp_index_value, "TSP-index / TSP-value", u8_u8, "Index number / Value of referred-to transparent slave parameter.");
  DID( 12, RO, fhb_size, "FHB-size", u8_u8, "Size of Fault-History-Buffer supported by slave");
  DID( 13, RO, fhb_index_value, "FHB-index / FHB-value", u8_u8, "Index number / Value of referred-to fault-history buffer entry.");
  DID( 14, WO, max_rel_mod_level_setting, "Max-rel-mod-level-setting", F88, "Maximum relative modulation level setting (%)");
  DID( 15, RO, max_capacity_min_mod_level, "Max-Capacity / Min-Mod-Level", u8_u8, "Maximum boiler capacity (kW) / Minimum boiler modulation level(%)");
  DID( 16, WO, trset, "TrSet", F88, "Room Setpoint (°C)");
  DID( 17, RO, rel_mod_level, "Rel.-mod-level", F88, "Relative Modulation Level (%)");
  DID( 18, RO, ch_pressure, "CH-pressure", F88, "Water pressure in CH circuit (bar)");
  DID( 19, RO, dhw_flow_rate, "DHW-flow-rate", F88, "Water flow rate in DHW circuit. (litres/minute)");
  DID( 20, RW, day_time, "Day-Time", special_u8, "Day of Week and Time of Day");
  DID( 21, RW, date, "Date", u8_u8, "Calendar date");
  DID( 22, RW, year, "Year", u16, "Calendar year");
  DID( 23, WO, trsetch2, "TrSetCH2", F88, "Room Setpoint for 2nd CH circuit (°C)");
  DID( 24, WO, tr, "Tr", F88, "Room temperature (°C)");
  DID( 25, RO, tboiler, "Tboiler", F88, "Boiler flow water temperature (°C)");
  DID( 26, RO, tdhw, "Tdhw", F88, "DHW temperature (°C)");
  DID( 27, RO, toutside, "Toutside", F88, "Outside temperature (°C)");
  DID( 28, RO, tret, "Tret", F88, "Return water temperature (°C)");
  DID( 29, RO, tstorage, "Tstorage", F88, "Solar storage temperature (°C)");
  DID( 30, RO, tcollector, "Tcollector", F88, "Solar collector temperature (°C)");
  DID( 31, RO, tflowch2, "TflowCH2", F88, "Flow water temperature CH2 circuit (°C)");
  DID( 32, RO, tdhw2, "Tdhw2", F88, "Domestic hot water temperature 2 (°C)");
  DID( 33, RO, texhaust, "Texhaust", s16, "Boiler exhaust temperature (°C)");
  DID( 48, RO, tdhwset_ub_lb, "TdhwSet-UB / TdhwSet-LB", s8_s8, "DHW setpoint upper & lower bounds for adjustment (°C)");
  DID( 49, RO, max_tset_ub_lb, "MaxTSet-UB / MaxTSet-LB", s8_s8, "Max CH water setpoint upper & lower bounds for adjustment (°C)");
  DID( 50, RO, hcratio_ub_lb, "Hcratio-UB / Hcratio-LB", s8_s8, "OTC heat curve ratio upper & lower bounds for adjustment");
  DID( 56, RW, tdhwset, "TdhwSet", F88, "DHW setpoint (°C) (Remote parameter 1)");
  DID( 57, RW, maxtset, "MaxTSet", F88, "Max CH water setpoint (°C) (Remote parameters 2)");
  DID( 58, RW, hcratio, "Hcratio", F88, "OTC heat curve ratio (°C) (Remote parameter 3)");
  DID(100, RO, remote_override, "Remote override function", flag8_, "Function of manual and program changes in master and remoteroom setpoint.");
  DID(115, RO, oem_diagnostic_code, "OEM diagnostic code", u16, "OEM-specific diagnostic/service code");
  DID(116, RW, burner_starts, "Burner starts", u16, "Number of starts burner");
  DID(117, RW, ch_pump_starts, "CH pump starts", u16, "Number of starts CH pump");
  DID(118, RW, dhw_pump_valve_starts, "DHW pump/valve starts", u16, "Number of starts DHW pump/valve");
  DID(119, RW, dhw_burner_starts, "DHW burner starts", u16, "Number of starts burner during DHW mode");
  DID(120, RW, burner_ophours, "Burner operation hours", u16, "Number of hours that burner is in operation (i.e. flame on)");
  DID(121, RW, ch_pump_ophours, "CH pump operation hours", u16, "Number of hours that CH pump has been running");
  DID(122, RW, dhw_pump_valve_ophours, "DHW pump/valve operation hours", u16, "Number of hours that DHW pump has been running or DHW valvehas been opened");
  DID(123, RW, dhw_burner_ophours, "DHW burner operation hours", u16, "Number of hours that burner is in operation during DHW mode");
  DID(124, WO, ot_version_master, "OpenTherm version Master", F88, "The implemented version of the OpenTherm Protocol Specificationin the master.");
  DID(125, RO, ot_version_slave, "OpenTherm version Slave", F88, "The implemented version of the OpenTherm Protocol Specificationin the slave.");
  DID(126, WO, master_version, "Master-version", u8_u8, "Master product version number and type");
  DID(127, RO, slave_version, "Slave-version", u8_u8, "Slave product version number and type");
  // clang-format on

  void process(const Frame& f) {}
};

} // namespace OpenTherm

#endif // _OPENTHERM_DATA_H_
