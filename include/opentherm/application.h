#ifndef _OPENTHERM_DATA_H_
#define _OPENTHERM_DATA_H_

#include <stdint.h>
#include <stdio.h>

#include "transport.h"

// OpenTherm 2.2 application layer

namespace OpenTherm {

class Application {
public:
  enum RWSpec : uint8_t { RO, WO, RW };

  enum Type : uint8_t {
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

  struct IDMeta {
    RWSpec msg;
    const char *data_object;
    Type type;
    const char *description;
  };

  Application(DeviceBase &device, IDMeta *idmeta = nullptr)
      : device(device), index_size(0), idmeta(idmeta) {
    // device.set_frame_callback(sprocess, this);
  }

  virtual ~Application() = default;

  struct ID;
  struct IDIndex {
    uint8_t id;
    ID *state;
  };

  struct ID {
    ID() = default;

    ID(uint8_t nr, RWSpec msg, const char *data_object, Type type,
       const char *description, IDIndex *index, size_t &index_size,
       IDMeta *meta = nullptr)
        : value(0) {
      if (meta)
        meta[nr] = IDMeta{msg, data_object, type, description};
      index[index_size] = {.id = nr, .state = this};
      index_size++;
    }

    virtual ~ID() = default;

    uint16_t value = 0;

    ID &operator=(uint16_t v) {
      value = v;
      return *this;
    }

    ID &operator=(float v) {
      value = v / 256.0;
      return *this;
    }

    static const char *to_string(Type type, uint16_t value) {
      static char buf[128];
      switch (type) {
      case flag8_flag8:
        snprintf(buf, sizeof(buf), "%02x/%02x", value >> 8, value & 0x00FF);
        break;
      case F88: {
        float f = ((signed)value) / 256.0f;
        snprintf(buf, sizeof(buf), "%.2f", f);
        break;
      }
      case flag8_u8:
        snprintf(buf, sizeof(buf), "%02x/%u", value >> 8, value & 0x00FF);
        break;
      case u8_u8:
        snprintf(buf, sizeof(buf), "%u/%u", value >> 8, value & 0x00FF);
        break;
      case s8_s8:
        snprintf(buf, sizeof(buf), "%d/%d", (int8_t)(value >> 8),
                 (int8_t)value);
        break;
      case u16:
        snprintf(buf, sizeof(buf), "%u", value);
        break;
      case s16:
        snprintf(buf, sizeof(buf), "%d", (int16_t)value);
        break;
      case special_u8:
        snprintf(buf, sizeof(buf), "XXX/%u", value & 0x00FF);
        break;
      case flag8_:
        snprintf(buf, sizeof(buf), "%02x", value >> 8);
        break;
      default:
        snprintf(buf, sizeof(buf), "unknown type");
        break;
      }
      return buf;
    }
  };

  virtual void run() = 0;

protected:
  DeviceBase &device;

  IDIndex index[59];
  size_t index_size;

  IDMeta *idmeta = nullptr;

#define DID(NR, MSG, FNAME, OBJECT, TYPE, DESC)                                \
  ID FNAME = ID(NR, MSG, OBJECT, TYPE, DESC, index, index_size, idmeta);

  // Ids 0 .. 127 are reserved for OpenTherm pre-defined information, while
  // id’s from 128 .. 255 can be used by manufacturers (members of the
  // association) for test & diagnostic purposes only.

  // clang-format off
  DID(  0, RO, status, "Status", flag8_flag8, "Master and Slave Status flags.");
  DID(  1, WO, tset, "TSet", F88, "Control setpoint, i.e. CH water temperature setpoint (&deg;C)");
  DID(  2, WO, mconfig_mmemberid, "M-Config / M-MemberIDcode", flag8_u8, "Master Configuration Flags / Master MemberID Code");
  DID(  3, RO, sconfig_smemberid, "S-Config / S-MemberIDcode", flag8_u8, "Slave Configuration Flags / Slave MemberID Code");
  DID(  4, WO, command, "Command", u8_u8, "Remote Command");
  DID(  5, RO, asf_flags, "ASF-flags / OEM-fault-code", flag8_u8, "Application-specific fault flags and OEM fault code");
  DID(  6, RO, rbp_flags, "RBP-flags", flag8_flag8, "Remote boiler parameter transfer-enable & read/write flags");
  DID(  7, WO, cooling_protocol, "Cooling-control", F88, "Cooling control signal (%)");
  DID(  8, WO, tsetch2, "TsetCH2", F88, "Control setpoint for 2nd CH circuit (&deg;C)");
  DID(  9, RO, troverride, "TrOverride", F88, "Remote override room setpoint");
  DID( 10, RO, tsp, "TSP", u8_u8, "Number of Transparent-Slave-Parameters supported by slave");
  DID( 11, RW, tsp_index_value, "TSP-index / TSP-value", u8_u8, "Index number / Value of referred-to transparent slave parameter.");
  DID( 12, RO, fhb_size, "FHB-size", u8_u8, "Size of Fault-History-Buffer supported by slave");
  DID( 13, RO, fhb_index_value, "FHB-index / FHB-value", u8_u8, "Index number / Value of referred-to fault-history buffer entry.");
  DID( 14, WO, max_rel_mod_level_setting, "Max-rel-mod-level-setting", F88, "Maximum relative modulation level setting (%)");
  DID( 15, RO, max_capacity_min_mod_level, "Max-Capacity / Min-Mod-Level", u8_u8, "Maximum boiler capacity (kW) / Minimum boiler modulation level (%)");
  DID( 16, WO, trset, "TrSet", F88, "Room Setpoint (&deg;C)");
  DID( 17, RO, rel_mod_level, "Rel.-mod-level", F88, "Relative Modulation Level (%)");
  DID( 18, RO, ch_pressure, "CH-pressure", F88, "Water pressure in CH circuit (bar)");
  DID( 19, RO, dhw_flow_rate, "DHW-flow-rate", F88, "Water flow rate in DHW circuit. (litres/minute)");
  DID( 20, RW, day_time, "Day-Time", special_u8, "Day of Week and Time of Day");
  DID( 21, RW, date, "Date", u8_u8, "Calendar date");
  DID( 22, RW, year, "Year", u16, "Calendar year");
  DID( 23, WO, trsetch2, "TrSetCH2", F88, "Room Setpoint for 2nd CH circuit (&deg;C)");
  DID( 24, WO, tr, "Tr", F88, "Room temperature (&deg;C)");
  DID( 25, RO, tboiler, "Tboiler", F88, "Boiler flow water temperature (&deg;C)");
  DID( 26, RO, tdhw, "Tdhw", F88, "DHW temperature (&deg;C)");
  DID( 27, RO, toutside, "Toutside", F88, "Outside temperature (&deg;C)");
  DID( 28, RO, tret, "Tret", F88, "Return water temperature (&deg;C)");
  DID( 29, RO, tstorage, "Tstorage", F88, "Solar storage temperature (&deg;C)");
  DID( 30, RO, tcollector, "Tcollector", F88, "Solar collector temperature (&deg;C)");
  DID( 31, RO, tflowch2, "TflowCH2", F88, "Flow water temperature CH2 circuit (&deg;C)");
  DID( 32, RO, tdhw2, "Tdhw2", F88, "Domestic hot water temperature 2 (&deg;C)");
  DID( 33, RO, texhaust, "Texhaust", s16, "Boiler exhaust temperature (&deg;C)");
  DID( 48, RO, tdhwset_ub_lb, "TdhwSet-UB / TdhwSet-LB", s8_s8, "DHW setpoint upper & lower bounds for adjustment (&deg;C)");
  DID( 49, RO, max_tset_ub_lb, "MaxTSet-UB / MaxTSet-LB", s8_s8, "Max CH water setpoint upper & lower bounds for adjustment (&deg;C)");
  DID( 50, RO, hcratio_ub_lb, "Hcratio-UB / Hcratio-LB", s8_s8, "OTC heat curve ratio upper & lower bounds for adjustment");
  DID( 56, RW, tdhwset, "TdhwSet", F88, "DHW setpoint (&deg;C) (Remote parameter 1)");
  DID( 57, RW, maxtset, "MaxTSet", F88, "Max CH water setpoint (&deg;C) (Remote parameters 2)");
  DID( 58, RW, hcratio, "Hcratio", F88, "OTC heat curve ratio (&deg;C) (Remote parameter 3)");
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
  DID(124, WO, ot_version_master, "OpenTherm version Master", F88, "The implemented version of the OpenTherm Protocol Specification in the master.");
  DID(125, RO, ot_version_slave, "OpenTherm version Slave", F88, "The implemented version of the OpenTherm Protocol Specification in the slave.");
  DID(126, WO, master_version, "Master-version", u8_u8, "Master product version number and type");
  DID(127, RO, slave_version, "Slave-version", u8_u8, "Slave product version number and type");

  DID( 34, RO, heat_ex_temp, "Theatex", F88, "Boiler heat exchanger temperature (&deg;C)");
  DID( 35, RW, boiler_fan_speed, "Boiler fan speed", u8_u8, "Boiler fan speed setpoint and actual value");
  DID( 36, RO, burner_flame_current, "Burner flame current", s16, "Electrical current through burner flame [µA]");
  DID( 37, WO, tr2, "Tr2", F88, "Room temperature for 2nd CH circuit (&deg;C)");
  DID( 38, RW, humidity, "Humidity", s16, "Relative Humidity");
  // clang-format on

  virtual ID *find(uint8_t id) {
    for (auto d : index)
      if (d.id == id)
        return d.state;
    return nullptr;
  }

  // Master to Slave
  virtual void on_read(uint8_t data_id, uint16_t data_value = 0x0000) {
    const ID *id = find(data_id);
    if (id != NULL) {
      // return DATA-INVALID(DATA-ID, DATA-VALUE) if the data ID is recognised
      // but the data requested is not available or invalid. DATA-VALUE can be
      // 0x0000 in this case.
      device.tx(Frame(ReadACK, data_id, id->value));
    } else
      device.tx(Frame(UnknownDataID, data_id, data_value));
  }

  virtual void on_write(uint8_t data_id, uint16_t data_value) {
    ID *id = find(data_id);
    if (id != NULL) {
      id->value = data_value;
      device.tx(Frame(WriteACK, data_id, data_value));
    } else
      device.tx(Frame(UnknownDataID, data_id, data_value));
  }

  virtual void on_invalid_data(uint8_t data_id, uint16_t data_value) {
    const ID *id = find(data_id);
    if (id != NULL)
      device.tx(Frame(DataInvalid, data_id, data_value));
    else
      device.tx(Frame(UnknownDataID, data_id, data_value));
  }

  // Slave to Master
  virtual void on_read_ack(uint8_t data_id, uint16_t data_value) {}
  virtual void on_write_ack(uint8_t data_id, uint16_t data_value) {}
  virtual void on_data_invalid(uint8_t data_id, uint16_t data_value) {}
  virtual void on_unknown_data_id(uint8_t data_id, uint16_t data_value) {}

protected:
  virtual bool process(const Frame &f) {
    switch (f.msg_type()) {
    case ReadData:
      on_read(f.id(), f.value());
      return true;
    case WriteData:
      on_write(f.id(), f.value());
      return true;
    case InvalidData:
      on_invalid_data(f.id(), f.value());
      return true;
    case ReadACK:
      on_read_ack(f.id(), f.value());
      return true;
    case WriteACK:
      on_write_ack(f.id(), f.value());
      return true;
    case DataInvalid:
      on_data_invalid(f.id(), f.value());
      return true;
    case UnknownDataID:
      on_unknown_data_id(f.id(), f.value());
      return true;
    }

    return false;
  }

  static bool sprocess(Application &app, const Frame &f) {
    return app.process(f);
  }
};

class RichApplication : public Application {
public:
  RichApplication(DeviceBase &device) : Application(device, idmeta) {}

  virtual ~RichApplication() = default;

  virtual ID *find(uint8_t id) override {
    return &index[id];
  }

protected:
  ID index[256];
  IDMeta idmeta[256];
};

} // namespace OpenTherm

#endif // _OPENTHERM_DATA_H_
