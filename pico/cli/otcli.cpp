// CM Wintersteiger, 2022

#include <algorithm>
#include <chrono>
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

Application::IDMeta Application::idmeta[256] = {};

static struct mosquitto *mosq = NULL;
static std::thread *mosq_thread = nullptr;
static bool mosq_thread_running = false;
static const char *mosq_cmd_in_topic = "OpenThermOstat/command/in";
static const char *mosq_cmd_out_topic = "OpenThermOstat/command/out";

class CLIDevice : public DeviceBase {
public:
  CLIDevice() = default;
  virtual ~CLIDevice() = default;

  virtual RequestID tx(const Frame & f, bool skip_if_busy = false, void (*callback)(RequestStatus, const Frame &) = nullptr) override
  {
    return NoRequestID;
  }
};

class CLIApp : public Application {
public:
  CLIApp() : device(), Application(device) {
    device.set_frame_callback(Application::sprocess, this);
  }
  virtual ~CLIApp() = default;

  virtual void run() override {}

protected:
  CLIDevice device;
};

static CLIApp app;
static auto command_timeout = 2s;
static std::mutex log_mtx;
static bool waiting_for_reply = false;
static Frame last_frame;

static std::map<std::string,
                std::function<void(const std::vector<std::string> &)>>
    cmds;

Frame from_hex(const std::string &hex) {
  uint32_t msg;
  if (sscanf(hex.c_str(), "%08x", &msg) != 1)
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

void mosquitto_msg_callback(mosquitto *mosq, void *arg,
                            const mosquitto_message *msg) {
  const std::lock_guard<std::mutex> lock(log_mtx);
  if (msg->payloadlen != 8) {
    std::cout << "unknown message of length " << msg->payloadlen << std::endl;
  } else {
    uint32_t otmsg = 0;
    if (sscanf((char *)msg->payload, "%08x", &otmsg) != 1)
      std::cout << "erroneous message: %s is not a hex-encoded frame"
                << std::endl;
    else {
      last_frame = Frame(otmsg);
      std::cout << "\r> " << to_string(last_frame) << std::endl;
      waiting_for_reply = false;
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

  r = mosquitto_subscribe(mosq, NULL, mosq_cmd_out_topic, 0);
}

void send_msg(const Frame &f) {
  {
    const std::lock_guard<std::mutex> lock(log_mtx);
    std::cout << "\r< " << to_string(f) << std::endl;
  }
  char msg_hex[9];
  int len = snprintf(msg_hex, sizeof(msg_hex), "%08x", (uint32_t)f);
  waiting_for_reply = true;
  auto before = std::chrono::system_clock::now();
  auto err =
      mosquitto_publish(mosq, NULL, mosq_cmd_in_topic, len, msg_hex, 0, false);
  if (err != MOSQ_ERR_SUCCESS) {
    const std::lock_guard<std::mutex> lock(log_mtx);
    std::cout << "X error: " << (unsigned)err << std::endl;
  }
  static const char prog_chars[] = { '-', '\\', '|', '/' };
  int pci = 0;
  do {
    auto after = std::chrono::system_clock::now();
    if ((after - before) > command_timeout) {
      std::cout << "\rX timeout" << std::endl;
      return;
    }
    std::cout << '\r' << prog_chars[pci];
    pci = (pci + 1) % 4;
    std::cout.flush();
    std::this_thread::sleep_for(100ms);
  } while (waiting_for_reply);
  std::cout << "\r";
}

void send_msg(Frame &&f) { send_msg(f); }

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
        } else {
          cmd(args);
        }
      }
    }
  } catch (const std::exception &ex) {
    const std::lock_guard<std::mutex> lock(log_mtx);
    std::cout << "X exception: " << ex.what() << std::endl;
  }
}

void register_cmds() {
  cmds["echo"] = [](auto &args) {
    std::cout << "\r  - echo";
    for (const auto &arg : args)
      std::cout << " " << arg;
    std::cout << std::endl;
  };

  cmds["read"] = [](auto &args) {
    if (args.size() != 1)
      throw std::runtime_error("invalid number of arguments");
    int id = 0;
    if (sscanf(args[0].c_str(), "%d", &id) != 1 | id > 255)
      throw std::runtime_error("invalid data id");
    send_msg(Frame(MsgType::ReadData, id, 0));
  };

  cmds["write"] = [](auto &args) {
    if (args.size() != 2)
      throw std::runtime_error("invalid number of arguments");
    int id = 0;
    if (sscanf(args[0].c_str(), "%d", &id) != 1 | id > 255)
      throw std::runtime_error("invalid data id");
    uint16_t value = 0;
    if (sscanf(args[1].c_str(), "%04hx", &value) != 1)
      throw std::runtime_error("invalid data value");
    send_msg(Frame(MsgType::WriteData, id, value));
  };
}

void ask_for_input()
{
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
      send_msg(Frame(0));
    else {
      for (size_t i = 1; i < argc; i++) {
        try {
          send_msg(from_hex(argv[i]));
        } catch (const std::exception &ex) {
          const std::lock_guard<std::mutex> lock(log_mtx);
          std::cout << "X Skipping invalid frame '" << argv[i]
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
      ask_for_input();
    }
  } catch (const std::exception &ex) {
    std::cout << "Exception: " << ex.what() << std::endl;
    r = 1;
  } catch (...) {
    std::cout << "Caught unknown exception" << std::endl;
    r = 2;
  }

  std::cout << "\r";
  mosq_thread_running = false;
  if (mosq_thread)
    mosq_thread->join();
  mosquitto_lib_cleanup();
  return r;
}
