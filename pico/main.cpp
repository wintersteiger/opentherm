#include <stdio.h>

#include "lwipopts.h"

#include <boards/pico_w.h>

#include <hardware/adc.h>
#include <hardware/clocks.h>
#include <hardware/exception.h>
#include <hardware/flash.h>
#include <hardware/pll.h>
#include <pico/cyw43_arch.h>
#include <pico/multicore.h>
#include <pico/mutex.h>
#include <pico/platform.h>
#include <pico/stdlib.h>
#include <pico/unique_id.h>

#include <lwip/apps/fs.h>
#include <lwip/apps/httpd.h>
#include <lwip/apps/mqtt.h>
#include <lwip/stats.h>
#include <lwip/sys.h>
#include <lwip/tcpip.h>

#include <opentherm/application.h>
#include <opentherm/pid.h>
#include <opentherm/transport.h>

#include "hardware/timer.h"
#include "pico_opentherm_ds.h"

#if LWIP_TCPIP_CORE_LOCKING
extern "C" {
extern sys_mutex_t lock_tcpip_core;
void lwip_assert_core_locked(void) {
  LWIP_ASSERT("TCP/IP mutex is valid", sys_mutex_valid(lock_tcpip_core));
  // Check whether we are in an ISR?
  // LWIP_ASSERT("TCP/IP mutex is locked", ... );
}
}
#endif

using namespace OpenTherm;

alarm_pool_t *PicoTimer::alarm_pool = NULL;

auto_init_recursive_mutex(log_mtx);

void vllog(const char *fmt, va_list args) {
  recursive_mutex_enter_blocking(&log_mtx);
  printf("[%010llu] ", time_us_64());
  vprintf(fmt, args);
  printf("\n");
  fflush(stdout);
  recursive_mutex_exit(&log_mtx);
}

void llog(const char *fmt, ...) {
  va_list args;
  va_start(args, fmt);
  vllog(fmt, args);
  va_end(args);
}

float get_core_temperature() {
  static const float conversion_factor = 3.3f / (1 << 12);
  adc_select_input(4);
  uint16_t traw = adc_read();
  float v = traw * conversion_factor;
  return 27.0 - (v - 0.706) / 0.001721;
}

static void hard_fault_exception_handler(void) {
  ::llog("EXCEPTION: HARD FAULT ON CORE %d", get_core_num());
  while (true)
    ;
}

static uint64_t flash_size() {
  static constexpr const size_t flash_cmd_size = 4;
  uint8_t txbuf[flash_cmd_size] = {0x9f};
  uint8_t rxbuf[flash_cmd_size] = {0};
  flash_do_cmd(txbuf, rxbuf, flash_cmd_size);
  return 1 << rxbuf[3];
}

#pragma pack(push, 1)
struct Configuration {
  struct Wifi {
    const char ssid[33] = WIFI_SSID;
    const char password[64] = WIFI_PASSWORD;
    uint32_t auth = WIFI_AUTH;
  };

  Wifi wifi;
  float setpoint = 0.0f;
  uint8_t status = 0x00;
};
#pragma pack(pop)

class MyDevice
    : public Master<PicoTimer, PicoSemaphore, PicoTime, PicoQueue, PicoIO> {
public:
  enum State { INIT, SLAVE_INFO, NORMAL };
  State state = INIT;

  MyDevice(const Pins &pins, const Configuration &configuration)
      : Master(pins), state(INIT), configuration(configuration) {}

  virtual ~MyDevice() = default;

  Frame init_master_frames[2] = {Frame(ReadData, 0), Frame(ReadData, 3)};

  Frame slave_info_frames[6] = {Frame(ReadData, 3),  Frame(ReadData, 125),
                                Frame(ReadData, 15), Frame(ReadData, 127),
                                Frame(ReadData, 57), Frame(WriteData, 1, 0.0f)};

  Frame master_frames[10] = {Frame(ReadData, 0),  Frame(ReadData, 10),
                             Frame(ReadData, 17), Frame(ReadData, 18),
                             Frame(ReadData, 19), Frame(ReadData, 25),
                             Frame(ReadData, 27), Frame(ReadData, 28),
                             Frame(ReadData, 35), Frame(WriteData, 1, 0.0f)};

  void advance() {
    switch (state) {
    case INIT:
      master_index =
          (master_index + 1) % (sizeof(init_master_frames) / sizeof(Frame));
      if (master_index == 0)
        state = SLAVE_INFO;
      break;
    case SLAVE_INFO:
      master_index =
          (master_index + 1) % (sizeof(slave_info_frames) / sizeof(Frame));
      if (master_index == 0)
        state = NORMAL;
      break;
    case NORMAL:
      master_index =
          (master_index + 1) % (sizeof(master_frames) / sizeof(Frame));
      break;
    }
  }

  virtual void next_master_msg() override {
    Frame f;

    switch (state) {
    case INIT:
      f = init_master_frames[master_index];
      break;
    case SLAVE_INFO:
      f = slave_info_frames[master_index];
      break;
    case NORMAL:
      f = master_frames[master_index];
    }

    switch (f.id()) {
    case 0:
      f = Frame(ReadData, f.id(), configuration.status, 0x00);
      break;
    case 1:
      if (state == NORMAL)
        f = Frame(f.msg_type(), f.id(), configuration.setpoint);
      break;
    }

    tx(f, true);
  }

protected:
  size_t master_index = 0;
  const Configuration &configuration;
};

class MyApp : public RichApplication {
public:
  using RichApplication::idmeta;

  MyApp(const Pins &pins)
      : RichApplication(device), device(pins, configuration),
        mqtt_publish_queue(mqtt_max_concurrent_requests),
        wifi_link_timer(0, 1000000, on_wifi_link_timer_cb, nullptr, this),
        pid(10.0, 10.0 / 50.0, 10.0 * 1.0, 0.0, 80.0),
        pid_timer(0, 1000000, on_pid_timer_cb, nullptr, this) {
    device.set_frame_callback(Application::sprocess, this);
    pid.set_automatic(true, 0.0);
  }

  virtual ~MyApp() = default;

  virtual void run() override {
    adc_init();
    adc_set_temp_sensor_enabled(true);

    if (!wifi_connect())
      llog("Could not connect to wifi.");

    device.start();

    httpd_init();

    rx_forever();
  }

  virtual bool process_command(const CommandFrame &cmd_frame) override {
    llog("Command(%02" PRIx8 ", %04" PRIx16 ", %08" PRIx32 ")",
         cmd_frame.command_id, cmd_frame.user_data, cmd_frame.payload);

    switch (cmd_frame.command_id) {
    case CommandID::OPENTHERM_REQUEST: {
      RequestID rid = tx(
          cmd_frame.payload, false,
          [](Application *a, RequestStatus s, RequestID rid, const Frame &f) {
            MyApp *app = static_cast<MyApp *>(a);
            auto &req =
                app->mqtt_cmd_requests[rid % mqtt_max_concurrent_requests];
            if (req.rid != rid)
              llog("warning: outdated MQTT request ID");
            req.rid = NoRequestID;
            if (s == RequestStatus::OK)
              app->mqtt_publish_cmd_response(req.user_data, f);
            else
              app->mqtt_publish_cmd_response(req.user_data, s);
          });

      if (rid == NoRequestID) {
        llog("error: failure to submit MQTT request (the queue is likely "
             "full)");
        return false;
      } else {
        mqtt_cmd_requests[rid % mqtt_max_concurrent_requests] = {
            .rid = rid, .user_data = cmd_frame.user_data};
        return true;
      }
      break;
    }
    case CommandID::INVALID: {
      mqtt_publish_cmd_response(cmd_frame.user_data, RequestStatus::ERROR);
      return false;
    }
    case CommandID::SET_STATUS: {
      configuration.status = cmd_frame.payload & 0x000000FF;
      llog("* New status: %02" PRIx8, configuration.status);
      tx(Frame(ReadData, 0, configuration.status, 0x00));
      mqtt_publish_cmd_response(cmd_frame.user_data, RequestStatus::OK);
      return true;
    }
    case CommandID::SET_SETPOINT: {
      float sp = to_f88((uint16_t)(cmd_frame.payload & 0x0000FFFF));
      llog("* New setpoint: %.2f", sp);
      configuration.setpoint = sp;
      pid.set_setpoint(sp);
      mqtt_publish_cmd_response(cmd_frame.user_data, RequestStatus::OK);
      return true;
    }
    case CommandID::TEMPERATURE_REPORT: {
      float t = to_f88((uint16_t)(cmd_frame.payload & 0x0000FFFF));
      float o = pid.update(t);
      llog("Report: %f; PID output: %f", t, o);
      mqtt_publish_cmd_response(cmd_frame.user_data, RequestStatus::OK);
      return true;
    }
    default:
      return Application::process_command(cmd_frame);
    }
  }

  virtual void on_read_ack(uint8_t data_id, uint16_t data_value) override {
    ::llog("Read-Ack(%d, %04x)", data_id, data_value);
    RichApplication::on_read_ack(data_id, data_value);
    slave_id_infos[data_id].seen = true;
    device.advance();

    switch (data_id) {
    case 0:
      if ((data_value & 0x0001) != 0) {
        // Slave indicates a fault; try to get a fault code.
        tx(Frame(ReadData, 5));
      }
      break;
    }
  }

  virtual void on_write_ack(uint8_t data_id, uint16_t data_value) override {
    ::llog("Write-Ack(%d, %04x)", data_id, data_value);
    RichApplication::on_write_ack(data_id, data_value);
    slave_id_infos[data_id].seen = true;
    device.advance();
  }

  virtual void on_data_invalid(uint8_t data_id, uint16_t data_value) override {
    ::llog("Data-Invalid(%d, %04x)", data_id, data_value);
    RichApplication::on_data_invalid(data_id, data_value);
    device.advance();
  }

  virtual void on_unknown_data_id(uint8_t data_id,
                                  uint16_t data_value) override {
    ::llog("Unknown-DataID(%d, %04x)", data_id, data_value);
    RichApplication::on_unknown_data_id(data_id, data_value);
    slave_id_infos[data_id].unsupported = true;
    device.advance();
  }

  virtual RequestID tx(const Frame &f, bool skip_if_busy = false,
                       void (*callback)(Application *, RequestStatus,
                                        RequestID rid,
                                        const Frame &) = nullptr) {
    return device.tx(f, skip_if_busy, callback, this);
  }

  MyDevice &dev() { return device; }

  virtual bool process(const Frame &f) override {
    if (!RichApplication::process(f))
      return false;

    mqtt_publish_queue_add(mqtt_topic, f);
    mqtt_publish_all_queued();
    return true;
  }

  void rx_forever() {
    device.rx_forever([](bool on) {
      cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, on ? 1 : 0);
    });
  }

  void tx_forever() { device.tx_forever(); }

  struct SlaveIDInfo {
    bool seen = false;
    bool unsupported = false;
  };

  SlaveIDInfo slave_id_infos[256] = {{0}};

  size_t num_mqtt_reinit = 0;

  size_t tx_frame_count() const { return device.tx_frame_count; }
  size_t rx_frame_count() const { return device.rx_frame_count; }

protected:
  MyDevice device;
  Configuration configuration;

  mqtt_client_t *mqtt_client = nullptr;
  struct mqtt_connect_client_info_t mqtt_client_info;
  static constexpr const char *mqtt_topic = "OpenThermOstat";
  static constexpr const char *mqtt_cmd_in_topic = "OpenThermOstat/command/in";
  static constexpr const char *mqtt_cmd_out_topic =
      "OpenThermOstat/command/out";
  PicoSemaphore mqtt_sem;

  struct MQTTRequest {
    RequestID rid = NoRequestID;
    uint16_t user_data = 0;
  };

  static constexpr const size_t mqtt_max_concurrent_requests = 16;

  MQTTRequest mqtt_cmd_requests[mqtt_max_concurrent_requests];
  char mosq_cmd_hex[CommandFrame::serialized_size() *
                    mqtt_max_concurrent_requests];
  size_t mosq_data_arrived = 0;

  struct MQTTPublishRequest {
    const char *topic;
    char payload[32];
    size_t length;
  };

  PicoQueue<MQTTPublishRequest> mqtt_publish_queue;

  void mqtt_publish_queue_add(const MQTTPublishRequest &req) {
    if (!mqtt_publish_queue.try_add(req))
      llog("dropping MQTT publish request");
  }

  void mqtt_publish_queue_add(const char *topic, const Frame &f) {
    MQTTPublishRequest req;
    req.topic = topic;
    req.length =
        snprintf(req.payload, sizeof(req.payload), "%08" PRIx32, (uint32_t)f);
    mqtt_publish_queue_add(req);
  }

  void mqtt_publish_all_queued() {
    static volatile bool busy = false;

    if (!mqtt_client || busy)
      return;

    busy = true;

    MQTTPublishRequest req;
    while (mqtt_publish_queue.try_remove(req)) {
      if (!mqtt_client)
        mqtt_reinit();

      for (bool retry = true; retry;) {
        mqtt_sem.acquire_blocking();
#if LWIP_TCPIP_CORE_LOCKING
        sys_mutex_lock(&lock_tcpip_core)
#endif
        {
          int err = mqtt_publish(
              mqtt_client, req.topic, req.payload, req.length, 0, 0,
              [](void *arg, err_t err) {
                if (err != ERR_OK && err != ERR_TIMEOUT)
                  llog("mqtt_publish failed (callback): err=%d", err);
                if (err != ERR_TIMEOUT) {
#if LWIP_TCPIP_CORE_LOCKING
                  sys_mutex_unlock(&lock_tcpip_core)
#endif
                      static_cast<PicoSemaphore *>(arg)
                          ->release();
                }
              },
              &mqtt_sem);

          if (err != ERR_OK) {
            llog("mqtt_publish failed (call): err=%d", err);
            mqtt_reinit();
            retry = true;
          } else if (!mqtt_sem.acquire_timeout(MQTT_REQ_TIMEOUT * 1e6)) {
            llog("mqtt_publish failed (timeout)");
            retry = true;
          } else
            retry = false;
        }
#if LWIP_TCPIP_CORE_LOCKING
        sys_mutex_unlock(&lock_tcpip_core)
#endif
            mqtt_sem.release();
      }
    }

    busy = false;
  }

  static void on_mqtt_publish_cb(void *arg, const char *topic, u32_t tot_len) {
    MyApp *app = static_cast<MyApp *>(arg);
    app->on_mqtt_publish(topic, tot_len);
  }

  void on_mqtt_publish(const char *topic, u32_t tot_len) {
    // if (strcmp(topic, mqtt_cmd_in_topic) == 0) {}
    // llog("on_mqtt_publish: tot_len=%lu", tot_len);
  }

  static void on_mqtt_data_cb(void *arg, const u8_t *data, u16_t len,
                              u8_t flags) {
    MyApp *app = static_cast<MyApp *>(arg);
    app->on_mqtt_data(data, len, flags);
  }

  void on_mqtt_data(const u8_t *data, u16_t len, u8_t flags) {
    static constexpr const size_t fsz = CommandFrame::serialized_size();

    for (uint16_t i = 0; i < len && mosq_data_arrived < sizeof(mosq_cmd_hex);)
      mosq_cmd_hex[mosq_data_arrived++] = data[i++];

    if (flags & MQTT_DATA_FLAG_LAST) {
      if (mosq_data_arrived >= fsz) {
        CommandFrame cf(mosq_cmd_hex, mosq_data_arrived);
        if (!process_command(cf))
          llog("command processing failed for command frame '%s'",
               mosq_cmd_hex);
      } else
        llog("dropping command frame of size %lu: %s", mosq_data_arrived,
             mosq_cmd_hex);

      for (size_t i = 0; i < fsz && i + fsz < sizeof(mosq_cmd_hex); i++)
        mosq_cmd_hex[i] = mosq_cmd_hex[i + fsz];
      mosq_data_arrived -= fsz;
    }
  }

  void mqtt_publish_cmd_response(uint16_t user_id, const Frame &f) {
    CommandFrame cf(CommandID::OPENTHERM_REPLY, user_id, (uint32_t)f);

    MQTTPublishRequest req;
    req.topic = mqtt_cmd_out_topic;
    if (!cf.to_string(req.payload, sizeof(req.payload))) {
      llog("cf.to_string failed");
      snprintf(req.payload, sizeof(req.payload), "00%04x00000000", user_id);
    }
    req.length = cf.serialized_size();
    mqtt_publish_queue_add(req);
  }

  void mqtt_publish_cmd_response(uint16_t user_id, RequestStatus s) {
    CommandFrame cf(CommandID::CONFIRMATION, user_id, static_cast<uint32_t>(s));

    MQTTPublishRequest req;
    req.topic = mqtt_cmd_out_topic;
    if (!cf.to_string(req.payload, sizeof(req.payload))) {
      llog("cf.to_string failed");
      snprintf(req.payload, sizeof(req.payload), "00%04x00000000", user_id);
    }
    req.length = cf.serialized_size();
    mqtt_publish_queue_add(req);
  }

  static void on_mqtt_connection_cb(mqtt_client_t *client, void *arg,
                                    mqtt_connection_status_t status) {
    MyApp *app = static_cast<MyApp *>(arg);
    app->on_mqtt_connection(client, status);
  }

  void on_mqtt_connection(mqtt_client_t *client,
                          mqtt_connection_status_t status) {
    llog("MQTT connection status: %d", status);
  }

  void mqtt_reinit() {
    llog("MQTT reconnecting");
    num_mqtt_reinit++;
    if (mqtt_client) {
      mqtt_disconnect(mqtt_client);
      mqtt_client_free(mqtt_client);
      mqtt_client = NULL;
    }
    mqtt_init();
  }

  void mqtt_init() {
    mosq_data_arrived = 0;

    static const char *mqtt_host = "192.168.0.41";
    mqtt_client_info.client_id = "OpenThermOstat";
    mqtt_client_info.client_user = "monitor";
    mqtt_client_info.client_pass = "UirHdEAPDxiOCB5939VQ";

    ip_addr_t ip;
    if (ipaddr_aton(mqtt_host, &ip) != 1) {
      llog("ipaddr_aton failed");
      return;
    }

    u16_t port = 1883;

    mqtt_client = mqtt_client_new();

    if (!mqtt_client) {
      llog("failed to allocate new MQTT client");
      return;
    }

    err_t r = mqtt_client_connect(mqtt_client, &ip, port, on_mqtt_connection_cb,
                                  this, &mqtt_client_info);
    if (r != ERR_OK) {
      llog("mqtt_client_connect failed: %d", r);
      return;
    }

    r = mqtt_subscribe(
        mqtt_client, mqtt_cmd_in_topic, 0,
        [](void *arg, err_t err) {
          if (err != ERR_OK)
            llog("mqtt_subscribe timed out: %d", err);
        },
        NULL);
    if (r != ERR_OK) {
      llog("mqtt_subscribe failed: %d", r);
      return;
    }

    mqtt_set_inpub_callback(mqtt_client, on_mqtt_publish_cb, on_mqtt_data_cb,
                            this);
  }

  bool wifi_link_down = true;
  PicoTimer wifi_link_timer;

  static bool on_wifi_link_timer_cb(Timer *timer, void *obj) {
    MyApp *app = static_cast<MyApp *>(obj);
    return app->on_wifi_link_timer(timer);
  };

  bool on_wifi_link_timer(Timer *) {
    int ws = cyw43_wifi_link_status(&cyw43_state, 0);
    int ts = cyw43_tcpip_link_status(&cyw43_state, 0);
    if (wifi_link_down)
      llog("Wifi link status: wifi=%d tcpip=%d", ws, ts);
    if (ws <= 0) {
      ws = cyw43_arch_wifi_connect_async(configuration.wifi.ssid,
                                         configuration.wifi.password,
                                         configuration.wifi.auth);
      wifi_link_down = true;
    }
    if (wifi_link_down && ts == CYW43_LINK_UP) {
      llog("Wifi link up; ip=%s", ipaddr_ntoa(&cyw43_state.netif[0].ip_addr));
      wifi_link_down = false;
      mqtt_init();
    }
    return true;
  };

  bool wifi_connect() {
    if (cyw43_arch_init_with_country(CYW43_COUNTRY_UK)) {
      llog("wifi init failed.");
      return false;
    }

    llog("Connecting to '%s'", configuration.wifi.ssid);
    cyw43_wifi_pm(&cyw43_state, CYW43_AGGRESSIVE_PM);
    cyw43_arch_enable_sta_mode();
    wifi_link_timer.start();

    return true;
  }

  PIDController<PicoTime> pid;
  PicoTimer pid_timer;

  static bool on_pid_timer_cb(Timer *timer, void *obj) {
    MyApp *app = static_cast<MyApp *>(obj);
    return app->on_pid_timer(timer);
  };

  bool on_pid_timer(Timer *) {
    pid.update(0.0);
    return true;
  };
};

MyApp app({.rx = 27, .tx = 26, .owned = true});

extern "C" {
#define MAX_RESPONSE_SIZE 6 * 1024

struct FSCustomData {
  uint64_t request_id = -1;
  const char *status = NULL;
  char lbuf[MAX_RESPONSE_SIZE];
  size_t lbuf_sz = 0;
};

static int make_http_response(char *state, FSCustomData *data) {
  auto &sz = data->lbuf_sz;
  auto *status = data->status;

  if (!status && data->request_id != NoRequestID) {
    Frame response;
    switch (app.dev().get(data->request_id, response)) {
    case RequestStatus::OK:
      sz = snprintf(data->lbuf, sizeof(data->lbuf), "%08" PRIx32,
                    (uint32_t)response);
      status = "200";
      break;
    case RequestStatus::TIMED_OUT:
      sz = snprintf(data->lbuf, sizeof(data->lbuf), "timed out");
      status = "408";
      break;
    default:
      sz = snprintf(data->lbuf, sizeof(data->lbuf),
                    "request failed with status %d", (int)status);
      status = "500";
    }
  }

  return snprintf(state, MAX_RESPONSE_SIZE,
                  "HTTP/1.1 %s OK\ncontent-length: %d\n\n%s\n", status, sz + 1,
                  data->lbuf);
}

size_t make_status_page(char *buf, size_t sz) {
#define PSZ p, sz - (p - buf)

  char *p = buf;
  p += snprintf(PSZ, "<html>");
  p += snprintf(
      PSZ,
      "<head><style>td{text-align:right}#txt{text-align:left}</style></head>");

  p += snprintf(PSZ, "<h2>Controller statistics</h2>");
  p += snprintf(PSZ, "<table border=1>");
  p += snprintf(PSZ, "<tr><th>Item</th><th>Value</th></tr>");
  p += snprintf(PSZ, "<tr><td id=\"txt\">Uptime</td><td>%.2f sec</td></tr>",
                time_us_64() / 1e6);
  p += snprintf(
      PSZ, "<tr><td id=\"txt\">Core temperature</td><td>%.2f &deg;C</td></tr>",
      get_core_temperature());
  p += snprintf(PSZ, "<tr><td id=\"txt\">MQTT reconnects</td><td>%u</td></tr>",
                app.num_mqtt_reinit);
  p +=
      snprintf(PSZ, "<tr><td id=\"txt\"># frames rx/tx</td><td>%u/%u</td></tr>",
               app.tx_frame_count(), app.rx_frame_count());
  p += snprintf(PSZ, "</table>");

  p += snprintf(PSZ, "<h2>OpenTherm data IDs</h2>");
  p += snprintf(PSZ, "<table border=1>");
  p += snprintf(PSZ,
                "<tr><th>Data "
                "ID</th><th>Raw</th><th>Value</th><th>Description</th></tr>");
  for (size_t i = 0; i < 256; i++)
    if (app.slave_id_infos[i].seen) {
      const auto *id = app.find(i);
      if (id)
        p += snprintf(
            PSZ,
            "<tr><td>%u</td><td>%04x</td><td>%s</td><td id=\"txt\">%s</td</tr>",
            i, id->value,
            Application::ID::to_string(app.idmeta[i].type, id->value),
            app.idmeta[i].description);
    }
  p += snprintf(PSZ, "</table>");

  p += snprintf(PSZ, "<h2>lwIP memory statistics</h2>");
  p += snprintf(PSZ, "<table border=1>");
  p += snprintf(PSZ, "<tr><th>Pool</th><th>err</th><th>avail</th>"
                     "<th>used</th><th>max</th><th>illegal</th></tr>");
  p += snprintf(PSZ,
                "<tr><td id=\"txt\">Global</td><td>%u</td><td>%lu</td><td>%lu</"
                "td><td>%lu</td><td>%u</td></tr>",
                lwip_stats.mem.err, lwip_stats.mem.avail, lwip_stats.mem.used,
                lwip_stats.mem.max, lwip_stats.mem.illegal);
  for (size_t i = 0; i < MEMP_MAX; i++)
    if (lwip_stats.memp[i])
      p += snprintf(PSZ,
                    "<tr><td id=\"txt\">%s</td><td>%u</td><td>%lu</td><td>%lu</"
                    "td><td>%lu</td><td>%u</td></tr>",
                    lwip_stats.memp[i]->name, lwip_stats.memp[i]->err,
                    lwip_stats.memp[i]->avail, lwip_stats.memp[i]->used,
                    lwip_stats.memp[i]->max, lwip_stats.memp[i]->illegal);
  p += snprintf(PSZ, "</table>");
  p += snprintf(PSZ, "</html>");
  return p - buf;
#undef PSZ
}

int fs_open_custom(struct fs_file *file, const char *name) {
  if (strcmp(name, "/ot.ssi") == 0 || strcmp(name, "/otget.ssi") == 0 ||
      strcmp(name, "/temp.ssi") == 0 || strcmp(name, "/status.ssi") == 0) {

    memset(file, 0, sizeof(struct fs_file));
    file->flags = FS_FILE_FLAGS_CUSTOM | FS_FILE_FLAGS_SSI;
    file->state = fs_state_init(file, name);
    file->len = 0;
    file->index = 0;

    auto data = (FSCustomData *)mem_malloc(sizeof(FSCustomData));
    if (!data) {
      llog("file data allocation failure");
      return 0;
    } else {
      data->request_id = NoRequestID;
      data->status = NULL;
      data->lbuf_sz = 0;
    }

    file->pextension = data;

    return 1;
  }
  return 0;
}

void fs_close_custom(struct fs_file *file) {
  if (file && file->pextension) {
    mem_free(file->pextension);
    file->pextension = NULL;
  }
}

void *fs_state_init(struct fs_file *file, const char *name) {
  void *r = mem_calloc(MAX_RESPONSE_SIZE, 1);
  if (!r)
    llog("FS state allocation failed");
  return r;
}

void fs_state_free(struct fs_file *file, void *state) {
  if (state)
    mem_free(state);
  file->state = NULL;
}

u8_t fs_canread_custom(struct fs_file *file) {
  auto data = (FSCustomData *)file->pextension;
  if (data == NULL)
    return 1;
  bool ready = true;
  if (data->request_id != NoRequestID)
    ready = app.dev().is_finished(data->request_id);
  if (ready && file->len == 0)
    file->len = make_http_response((char *)file->state, data);
  return ready ? 1 : 0;
}

u8_t fs_wait_read_custom(struct fs_file *file, fs_wait_cb callback_fn,
                         void *callback_arg) {
  // llog("fs_wait_read_custom cb=%p", callback_fn);
  return 1;
}

int fs_read_async_custom(struct fs_file *file, char *buffer, int count,
                         fs_wait_cb callback_fn, void *callback_arg) {
  const char *from = (const char *)file->state + file->index;
  int rem = file->len - file->index;
  int len = count > rem ? rem : count;
  memcpy(buffer, from, len);
  file->index += count;
  return len;
}

void httpd_cgi_handler(struct fs_file *file, const char *uri, int num_params,
                       char **keys, char **values, void *state) {
  // llog("httpd_cgi_handler: %s", uri);
  if (state != NULL) {
    auto data = (FSCustomData *)file->pextension;
    if (data == NULL) {
      data->lbuf_sz =
          snprintf(data->lbuf, sizeof(data->lbuf), "missing extension data");
      data->status = "500";
    } else {
      if (strcmp(uri, "/ot.ssi") == 0) {
        bool found_msg = false;
        for (int i = 0; i < num_params; i++) {
          if (strcmp(keys[i], "msg") == 0) {
            uint32_t msg = 0;
            found_msg = true;
            if (sscanf(values[i], "%08" SCNx32, &msg) != 1) {
              data->lbuf_sz =
                  snprintf(data->lbuf, sizeof(data->lbuf),
                           "'%s' is not a hex-encoded uint32_t", values[i]);
              data->status = "400";
            } else {
              data->request_id = app.tx(Frame(msg));
              if (data->request_id == NoRequestID) {
                data->lbuf_sz = snprintf(data->lbuf, sizeof(data->lbuf),
                                         "too many requests (%u)",
                                         app.dev().num_outstanding_requests());
                data->status = "429";
              }
            }
          }
        }
        if (!found_msg) {
          data->lbuf_sz = snprintf(data->lbuf, sizeof(data->lbuf),
                                   "missing parameter 'msg'");
          data->status = "400";
        }
      } else if (strcmp(uri, "/temp.ssi") == 0) {
        data->lbuf_sz = snprintf(data->lbuf, sizeof(data->lbuf), "%2.3f",
                                 get_core_temperature());
        data->status = "200";
      } else if (strcmp(uri, "/status.ssi") == 0) {
        data->lbuf_sz = make_status_page(data->lbuf, sizeof(data->lbuf));
        data->status = "200";
      } else
        data->status = "404";
    }
  }
}
}

static void core1_main(void) {
  multicore_lockout_victim_init();
  exception_set_exclusive_handler(HARDFAULT_EXCEPTION,
                                  hard_fault_exception_handler);

  // pio_interrupt_clear(pio0, 0);
  // pio_set_irq0_source_enabled(pio0, pis_interrupt0, true);
  // irq_set_exclusive_handler(PIO0_IRQ_0, []() {
  //   if (pio_interrupt_get(pio0, 0)) {
  //     pio_interrupt_clear(pio0, 0);
  //     ::llog("irq: %08x; dropped frame", pio0->irq);
  //   }
  // });
  // irq_set_enabled(PIO0_IRQ_0, true);

  app.tx_forever();
}

int main() {
  multicore_lockout_victim_init();
  exception_set_exclusive_handler(HARDFAULT_EXCEPTION,
                                  hard_fault_exception_handler);

  stdio_init_all();
  sleep_ms(1000);

  llog("Controller starting...");

  llog("RP2040 chip version %u, rom version %u", rp2040_chip_version(),
       rp2040_rom_version());

  char board_id[2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1];
  pico_get_unique_board_id_string(board_id, sizeof(board_id));
  llog("Board ID: %s", board_id);

  llog("Flash size: %llu bytes", flash_size());
  llog("Running at %.2f MHz", clock_get_hz(clk_sys) / 1e6);

  stats_init();

  multicore_reset_core1();
  multicore_launch_core1(core1_main);

  app.run();

  return 0;
}
