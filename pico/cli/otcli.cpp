// CM Wintersteiger, 2022

#include <stdio.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <curl/curl.h>

#include <mosquitto.h>

#include <opentherm/application.h>
#include <opentherm/transport.h>

using namespace std::chrono_literals;
using namespace OpenTherm;

static std::mutex log_mtx;

static struct mosquitto *mosq = NULL;
static std::thread *mosq_thread = nullptr;
static bool mosq_thread_running = false;
static const char *mosq_cmd_in_topic = "OpenThermOstat/command/in";
static const char *mosq_cmd_out_topic = "OpenThermOstat/command/out";

static auto command_timeout = 2s;
static volatile uint16_t next_request_id = 0;
static volatile uint16_t request_id_rcvd = -1;

std::string to_string(const Frame &f) {
  using namespace OpenTherm;
  std::stringstream ss;
  switch (f.msg_type()) {
  case MsgType::ReadData:
    ss << "Read-Data";
    break;
  case MsgType::WriteData:
    ss << "Write-Data";
    break;
  case MsgType::InvalidData:
    ss << "Invalid-Data";
    break;
  case MsgType::ReadACK:
    ss << "Read-Ack";
    break;
  case MsgType::WriteACK:
    ss << "Write-Ack";
    break;
  case MsgType::DataInvalid:
    ss << "Data-Invalid";
    break;
  case MsgType::UnknownDataID:
    ss << "Unknown-DataId";
    break;
  default:
    ss << "Unknown-Command";
  }
  ss << "(id=" << (unsigned)f.id() << ", ";
  ss << std::hex << std::setw(2) << std::setfill('0')
     << (unsigned)f.data_byte_1() << ", ";
  ss << std::hex << std::setw(2) << std::setfill('0')
     << (unsigned)f.data_byte_2();
  ss << ")";
  return ss.str();
}

class CLIDevice : public DeviceBase {
public:
  CLIDevice() = default;
  virtual ~CLIDevice() = default;

  virtual RequestID tx(const Frame &f, bool skip_if_busy = false,
                       void (*callback)(Application *, RequestStatus, RequestID,
                                        const Frame &) = nullptr,
                       Application *app = nullptr) override {
    return NoRequestID;
  }
};

class CLIApp : public RichApplication {
public:
  CLIApp() : device(), RichApplication(device) {
    device.set_frame_callback(Application::sprocess, this);
  }
  virtual ~CLIApp() = default;

  virtual void run() override {}

  virtual bool process_command(const CommandFrame &cmd_frame) override {
    static constexpr const size_t fsz = CommandFrame::serialized_size();

    char buf[fsz];
    bool q = cmd_frame.to_string(buf, sizeof(buf));

    switch (cmd_frame.command_id) {
    case CommandID::INVALID: {
      {
        const std::lock_guard<std::mutex> lock(log_mtx);
        std::cout << "\rX invalid command frame: " << buf << " (" << q << ")"
                  << std::endl;
      }
      break;
    }
    case CommandID::OPENTHERM_REQUEST: {
      const std::lock_guard<std::mutex> lock(log_mtx);
      std::cout << "\r? unexpected request: " << Frame(cmd_frame.payload).to_string() << std::endl;
      break;
    }
    case CommandID::OPENTHERM_REPLY: {
      Frame f(cmd_frame.payload);
      {
        const std::lock_guard<std::mutex> lock(log_mtx);
        std::cout << "\r> " << to_string(f) << std::endl;
      }
      process(f);
      request_id_rcvd = cmd_frame.user_data;
      break;
    }
    case CommandID::SET_STATUS: {
      const std::lock_guard<std::mutex> lock(log_mtx);
      std::cout << "\r> status := " << cmd_frame.payload << std::endl;
      break;
    }
    case CommandID::SET_SETPOINT: {
      const std::lock_guard<std::mutex> lock(log_mtx);
      std::cout << "\r> setpoint := " << cmd_frame.payload << std::endl;
      break;
    }
    default: {
      const std::lock_guard<std::mutex> lock(log_mtx);
      std::cout << "\rX command frame with unknown command id "
                << (int)static_cast<uint8_t>(cmd_frame.command_id) << std::endl;
    }
    }

    return true;
  }

  virtual void on_read_ack(uint8_t id, uint16_t value = 0x0000) override {
    RichApplication::on_read_ack(id, value);
    Application::ID *idp = find(id);
    if (!idp)
      printf("X unknown data ID");
    else {
      Application::IDMeta &meta = CLIApp::idmeta[id];
      printf("  %s == %s", meta.data_object, ID::to_string(meta.type, value));

      if (id == 0) {
        printf("\t\tfault: %d ch: %d dhw: %d flame: %d", (value & 0x01) != 0,
               (value & 0x02) != 0, (value & 0x04) != 0, (value & 0x08) != 0);
      }

      idp->value = value;
    }
    printf("\n");
  }

  virtual void on_write_ack(uint8_t id, uint16_t value = 0x0000) override {
    RichApplication::on_write_ack(id, value);
    Application::ID *idp = find(id);
    if (!idp)
      printf("X unknown data ID");
    else {
      Application::IDMeta &meta = CLIApp::idmeta[id];
      printf("  %s := %s\n", meta.data_object, ID::to_string(meta.type, value));
      idp->value = value;
    }
  }

protected:
  CLIDevice device;
};

static CLIApp app;

static std::map<std::string,
                std::function<void(const std::vector<std::string> &)>>
    cmds;

Frame from_hex(const std::string &hex) {
  uint32_t msg;
  if (sscanf(hex.c_str(), "%08" SCNx32, &msg) != 1)
    throw std::runtime_error(std::string("erroneous message: ") + hex +
                             " is not a valid hex-encoded frame");
  return Frame(msg);
}

static size_t WriteCallback(void *contents, size_t size, size_t nmemb,
                            void *userp) {
  ((std::string *)userp)->append((char *)contents, size * nmemb);
  return size * nmemb;
}

std::string make_request(const std::string &url) {
  CURL *curl;
  std::string r;

  curl = curl_easy_init();
  if (!curl)
    throw std::runtime_error("curl init failure");

  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &r);
  CURLcode code = curl_easy_perform(curl);
  curl_easy_cleanup(curl);

  if (code == CURLE_OK)
    return r;
  else
    throw std::runtime_error(std::string("curl error: ") +
                             curl_easy_strerror(code));
}

void mosquitto_on_msg(mosquitto *mosq, void *arg,
                      const mosquitto_message *msg) {
  static constexpr const size_t fsz = CommandFrame::serialized_size();

  if (msg->payloadlen == fsz) {
    CommandFrame cmd_frame(static_cast<const char *>(msg->payload),
                           msg->payloadlen);
    app.process_command(cmd_frame);
  } else {
    const std::lock_guard<std::mutex> lock(log_mtx);
    std::cout << "\rX unknown message of length " << msg->payloadlen
              << std::endl;
  }
}

void mosquitto_on_log(struct mosquitto *mosq, void *obj, int level,
                      const char *str) {
  if (level < MOSQ_LOG_DEBUG) {
    const std::lock_guard<std::mutex> lock(log_mtx);
    std::cout << "- " << level << ": " << str << std::endl;
  }
}

void mosquitto_init() {
  std::string host = "192.168.0.41";
  int port = 1883;
  int keepalive = 60;

  mosq = mosquitto_new(NULL, true, NULL);

  if (!mosq)
    throw std::bad_alloc();

  mosquitto_log_callback_set(mosq, mosquitto_on_log);
  mosquitto_message_callback_set(mosq, mosquitto_on_msg);
  mosquitto_connect_callback_set(mosq, [](mosquitto *, void *, int rc) {
    if (rc != MOSQ_ERR_SUCCESS) {
      const std::lock_guard<std::mutex> lock(log_mtx);
      std::cout << "X MQTT connection failure: " << rc << std::endl;
    }
  });

  const char *user = std::getenv("MQTT_USER");
  const char *password = std::getenv("MQTT_PASS");

  if (!user || !password)
    throw std::runtime_error("missing MQTT auth settings");

  mosquitto_username_pw_set(mosq, user, password);

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
        std::cout << "! MQTT reconnecting..." << std::endl;
        mosquitto_reconnect(mosq);
      }
    }
  });

  r = mosquitto_subscribe(mosq, NULL, mosq_cmd_out_topic, 0);
}

void send_msg(const CommandFrame &cf) {
  static constexpr const size_t fsz = CommandFrame::serialized_size();

  CommandFrame tcf = cf;
  tcf.user_data = next_request_id++;

  if (tcf.command_id == CommandID::OPENTHERM_REQUEST)
    app.process(Frame(tcf.payload));

  char msg_hex[fsz];
  tcf.to_string(msg_hex, sizeof(msg_hex));

  {
    const std::lock_guard<std::mutex> lock(log_mtx);
    std::cout << "\r< ";
    if (cf.command_id == CommandID::OPENTHERM_REQUEST)
      std::cout << Frame(cf.payload).to_string() << " [" << msg_hex << "]"
                << std::endl;
    else
      std::cout << msg_hex << std::endl;
  }

  auto before = std::chrono::system_clock::now();
  auto err = mosquitto_publish(mosq, NULL, mosq_cmd_in_topic, sizeof(msg_hex),
                               msg_hex, 0, false);
  if (err != MOSQ_ERR_SUCCESS) {
    const std::lock_guard<std::mutex> lock(log_mtx);
    std::cout << "X error: " << (unsigned)err << std::endl;
  }
  static const char prog_chars[] = {'-', '\\', '|', '/'};
  int pci = 0;
  do {
    auto after = std::chrono::system_clock::now();
    if ((after - before) > command_timeout) {
      const std::lock_guard<std::mutex> lock(log_mtx);
      std::cout << "\rX timeout" << std::endl;
      return;
    }
    std::cout << '\r' << prog_chars[pci];
    pci = (pci + 1) % 4;
    std::cout.flush();
    std::this_thread::sleep_for(100ms);
  } while (tcf.user_data < request_id_rcvd);
}

void send_msg(CommandFrame &&cf) { send_msg(cf); }

static void trim(std::string &str) {
  size_t fs = str.find_first_not_of(" ");
  str.erase(0, fs);
  size_t ls = str.find_last_not_of(" ");
  str.erase(ls + 1);
}

static std::vector<std::string> split(const std::string &s) {
  std::vector<std::string> r;
  size_t last_pos = 0, next_pos = std::string::npos;
  do {
    next_pos = s.find(' ', last_pos);
    if (next_pos == std::string::npos)
      r.push_back(s.substr(last_pos));
    else
      r.push_back(s.substr(last_pos, next_pos - last_pos));
    last_pos = next_pos + 1;
  } while (next_pos != std::string::npos);
  return r;
}

static void handle_line(std::string &line, volatile bool &keep_running) {
  try {
    if (line.size() > 0) {
      trim(line);
      if (line == "exit" || line == "quit" || line == "x" || line == "q")
        keep_running = false;
      else {
        std::vector<std::string> args = split(line);
        if (args.empty())
          return;

        std::string fun_name = args.front();
        args.erase(args.begin());
        std::transform(fun_name.begin(), fun_name.end(), fun_name.begin(),
                       ::tolower);

        auto &cmd = cmds[fun_name];
        if (!cmd) {
          const std::lock_guard<std::mutex> lock(log_mtx);
          std::cout << "\rX unknown command '" << fun_name << "'" << std::endl;
        } else
          cmd(args);
      }
    }
  } catch (const std::exception &ex) {
    const std::lock_guard<std::mutex> lock(log_mtx);
    std::cout << "\rX exception: " << ex.what() << std::endl;
  }
}

static void register_cmds() {
  cmds["read"] = [](auto &args) {
    uint16_t num = 1;

    if (args.size() != 1 && args.size() != 2)
      throw std::runtime_error("invalid number of arguments");

    int id = 0;
    if (sscanf(args[0].c_str(), "%d", &id) != 1 | id > 255)
      throw std::runtime_error("invalid data id");

    if (args.size() == 2) {
      int to = 0;
      if (sscanf(args[1].c_str(), "%d", &to) != 1 | to > 255)
        throw std::runtime_error("invalid data id");
      if (id > to)
        throw std::runtime_error("from must be smaller than to");
      num = to - id + 1;
    }

    if (num == 1) {
      send_msg(CommandFrame(CommandID::OPENTHERM_REQUEST, 0,
                            (uint32_t)Frame(MsgType::ReadData, id)));
    } else {
      for (uint16_t i = 0; i < num; i++) {
        send_msg(CommandFrame(CommandID::OPENTHERM_REQUEST, 0,
                              Frame(MsgType::ReadData, id + i)));
        std::this_thread::sleep_for(250ms);
      }
    }
  };

  cmds["write"] = [](auto &args) {
    if (args.size() != 2)
      throw std::runtime_error("invalid number of arguments");
    int id = 0;
    if (sscanf(args[0].c_str(), "%d", &id) != 1 | id > 255)
      throw std::runtime_error("invalid data id");
    uint16_t value = 0;
    if (sscanf(args[1].c_str(), "%04" SCNx16, &value) != 1)
      throw std::runtime_error("invalid data value");
    send_msg(CommandFrame(CommandID::OPENTHERM_REQUEST, 0,
                          (uint32_t)Frame(MsgType::WriteData, id, value)));
  };
}

static void ask_for_input() {
  const std::lock_guard<std::mutex> lock(log_mtx);
  std::cout << "\r* ";
  std::cout.flush();
}

int main(int argc, const char **argv) {
  int r = 0;
  try {
    // auto rsp = make_request("http://192.168.0.224/ot.ssi?msg=00030000");
    // std::cout << rsp << std::endl;

    register_cmds();
    mosquitto_lib_init();
    mosquitto_init();

    if (argc <= 1)
      send_msg(
          CommandFrame(CommandID::OPENTHERM_REQUEST, 0, (uint32_t)Frame(0)));
    else {
      for (size_t i = 1; i < argc; i++) {
        try {
          send_msg(CommandFrame(CommandID::OPENTHERM_REQUEST, 0,
                                (uint32_t)from_hex(argv[i])));
        } catch (const std::exception &ex) {
          const std::lock_guard<std::mutex> lock(log_mtx);
          std::cout << "\rX Skipping invalid frame '" << argv[i]
                    << "': " << ex.what() << std::endl;
        }
      }
    }

    ask_for_input();

    volatile bool keep_running = true;
    while (std::cin.good() && keep_running) {
      std::string line;
      std::getline(std::cin, line);
      handle_line(line, keep_running);
      // std::this_thread::sleep_for(500ms);
      ask_for_input();
    }
  } catch (const std::exception &ex) {
    const std::lock_guard<std::mutex> lock(log_mtx);
    std::cout << "\rX Exception: " << ex.what() << std::endl;
    r = 1;
  } catch (...) {
    const std::lock_guard<std::mutex> lock(log_mtx);
    std::cout << "\rX Caught unknown exception" << std::endl;
    r = 2;
  }

  std::cout << "\r";
  mosq_thread_running = false;
  if (mosq_thread)
    mosq_thread->join();
  mosquitto_lib_cleanup();
  return r;
}
