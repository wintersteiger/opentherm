#include <stdio.h>

#include <iostream>
#include <unordered_set>
#include <memory>
#include <thread>
#include <mutex>

#include <mosquitto.h>

#include <pqxx/pqxx>
#include <pqxx/except.hxx>

#include <opentherm/transport.h>
#include <opentherm/application.h>

using namespace OpenTherm;
using namespace std::chrono_literals;

class MyDevice : public DeviceBase {
public:
  MyDevice() : DeviceBase() {}
  virtual ~MyDevice() = default;

  virtual RequestID tx(const Frame &f, bool skip_if_busy = false,
                       void (*callback)(Application *, RequestStatus, RequestID,
                                        const Frame &) = nullptr,
                       Application *app = nullptr) override
  {
    return NoRequestID;
  }
};


class MyApp : public Application {
public:
  MyApp() : Application(device) {
    device.set_frame_callback(Application::sprocess, this);

    pqc = std::make_shared<pqxx::connection>("postgresql://cwinter:UxTCFqbq4Bd3xT1Mq3vC@192.168.0.41/station_house");
    // pqxx::work txn{*pqc};
  }

  virtual ~MyApp() = default;

  virtual void run() override {}

  void update(uint8_t id, uint16_t value) {
    ID* idp = find(id);
    if (!idp) {
      idp = new ID();
      // index[id] = idp;
      idmeta[id] = IDMeta{RWSpec::RW, "unknown", Type::u16, ""};
    }
    idp->value = value;
  }

  void dev_process(const Frame &f) {
    device.process(f);

    if (pqc) {
      pqxx::nontransaction ntx(*pqc);
      static char msg_type_hex[3], value_hex[5];
      snprintf(msg_type_hex, sizeof(msg_type_hex), "%02x", f.msg_type());
      snprintf(value_hex, sizeof(value_hex), "%04x", f.value());
      ntx.exec0(std::string("INSERT INTO boiler VALUES(NOW(), '\\x") + msg_type_hex + "', " + std::to_string(f.id()) + ", '\\x" + value_hex + "');");
    }
  }

  bool outp = false;
  uint16_t master_state = 0;

  virtual void on_read(uint8_t id, uint16_t value = 0x0000) override {
    Application::on_read(id, value);

    if (outp && id == 0) {
      if (value != master_state) {
        Application::IDMeta &meta = MyApp::idmeta[id];
        printf("%s == %s", meta.data_object, ID::to_string(meta.type, value));
        printf("\t\tch: %d dhw: %d cool: %d otc: %d ch2: %d",
          (value & 0x0100) != 0,
          (value & 0x0200) != 0,
          (value & 0x0400) != 0,
          (value & 0x0800) != 0,
          (value & 0x1000) != 0);
      }
      master_state = value;
    }
  }

  virtual void on_read_ack(uint8_t id, uint16_t value = 0x0000) override {
    Application::on_read_ack(id, value);
    if (outp)
    {
      Application::ID *idp = find(id);
      if (!idp)
        printf("unknown data ID");
      else
      {
        Application::IDMeta &meta = MyApp::idmeta[id];
        if ((id != 0 && id != 3) || idp->value != value) {
            printf("%s == %s", meta.data_object, ID::to_string(meta.type, value));

          if (id == 0 && (idp->value & 0x00FF) != (value & 0x00FF)) {
            printf("\t\tfault: %d ch: %d dhw: %d flame: %d",
              (value & 0x01) != 0,
              (value & 0x02) != 0,
              (value & 0x04) != 0,
              (value & 0x08) != 0);
          }
        }

        idp->value = value;
      }
    }
  }

  virtual void on_write_ack(uint8_t id, uint16_t value = 0x0000) override {
    Application::on_write_ack(id, value);
    if (outp)
    {
      Application::ID *idp = find(id);
      if (!idp)
        printf("unknown data ID");
      else
      {
        Application::IDMeta &meta = MyApp::idmeta[id];
        printf("%s := %s", meta.data_object, ID::to_string(meta.type, value));
        idp->value = value;
      }
    }
  }

protected:
  MyDevice device;
  std::shared_ptr<pqxx::connection> pqc = nullptr;
};

static MyApp app;
static std::mutex log_mtx;

static struct mosquitto *mosq = NULL;
static std::thread *mosq_thread = nullptr;
static bool mosq_thread_running = false;
static const char *mosq_topic = "OpenThermOstat/#";

void mosquitto_msg_callback(mosquitto *mosq, void *arg,
                            const mosquitto_message *msg) {
  const std::lock_guard<std::mutex> lock(log_mtx);
  if (msg->payloadlen != 8) {
    std::cout << "unknown message of length " << msg->payloadlen << std::endl;
  } else {
    uint32_t otmsg = 0;
    if (sscanf((char *)msg->payload, "%08x", &otmsg) != 1)
      std::cout << "erroneous message: not an OpenTherm frame" << std::endl;
    else {
      Frame f(otmsg);
      std::cout << "\r> " << f.to_string() << std::endl;
      app.dev_process(f);
    }
  }
}

void mosquitto_init() {
  std::string host = "192.168.0.41";
  int port = 1883;
  int keepalive = 60;

  mosq = mosquitto_new(NULL, true, NULL);

  if (!mosq)
    throw std::bad_alloc();

  // mosquitto_log_callback_set(mosq, ::on_log);
  mosquitto_message_callback_set(mosq, mosquitto_msg_callback);
  mosquitto_connect_callback_set(mosq, [](mosquitto *, void *, int rc) {
    if (rc != MOSQ_ERR_SUCCESS) {
      const std::lock_guard<std::mutex> lock(log_mtx);
      std::cout << "MQTT connection failure: " << rc << std::endl;
    }
  });

  mosquitto_username_pw_set(mosq, "monitor", "UirHdEAPDxiOCB5939VQ");

  int r = mosquitto_connect(mosq, host.c_str(), port, keepalive);
  if (r != MOSQ_ERR_SUCCESS) {
    const std::lock_guard<std::mutex> lock(log_mtx);
    throw std::runtime_error("MQTT connection failed");
  }

  mosq_thread_running = true;

  mosq_thread = new std::thread([]() {
    while (mosq_thread_running) {
      int r = mosquitto_loop(mosq, 125, 1);
      if (r != MOSQ_ERR_SUCCESS) {
        const std::lock_guard<std::mutex> lock(log_mtx);
        std::cout << "MQTT reconnecting..." << std::endl;
        mosquitto_reconnect(mosq);
      }
    }
  });

  r = mosquitto_subscribe(mosq, NULL, mosq_topic, 0);
}

int main(int argc, const char **argv)
{
  if (argc != 1) {
    std::cout << "Usage: " << argv[0] << std::endl;
    return 1;
  }

  mosquitto_lib_init();
  mosquitto_init();

  while (true)
    std::this_thread::sleep_for(1s);

  return 0;
}
