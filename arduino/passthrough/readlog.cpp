#include <stdio.h>
#include <iostream>

#include <opentherm/transport.h>
#include <opentherm/application.h>

using namespace OpenTherm;

Application::IDMeta Application::idmeta[256];

class MyDevice : public DeviceBase {
public:
  MyDevice() : DeviceBase() {}
  virtual ~MyDevice() = default;

  virtual RequestID tx(const Frame &f, bool skip_if_busy = false,
                       void (*callback)(Application *, RequestStatus,
                                        const Frame &) = nullptr,
                       Application *app = nullptr) override
  {
    return NoRequestID;
  }
};


class MyApp : public Application {
public:
  MyApp() : Application(device) {}
  virtual ~MyApp() = default;

  virtual void run() override {}

  void update(uint8_t id, uint16_t value) {
    ID* idp = index[id];
    if (!idp) {
      idp = new ID();
      index[id] = idp;
      idmeta[id] = IDMeta{RWSpec::RW, "unknown", Type::u16, ""};
    }
    idp->value = value;
  }

protected:
  MyDevice device;
};

static MyApp app;

int main(int argc, const char **argv)
{
  if (argc != 1 && argc != 2) {
    std::cout << "Usage: " << argv[0] << " [filename]" << std::endl;
    return 1;
  }

  FILE *f = stdin;

  if (argc == 2) {
    const char *filename = argv[1];
    f = fopen(filename, "r");
    if (!f) {
      std::cout << "failure to open " << filename << std::endl;
      return 2;
    }
  }

  uint64_t prev_time = 0, time = 0;
  char c = 'X';
  uint32_t msg = 0;

  while (!feof(f)) {
    int l = fscanf(f, "[%lu] %c: %08x\n", &time, &c, &msg);
    if (l != 3) {
      std::cout << "       fscanf failed: " << l << std::endl;
      do {
        fread(&c, 1, 1, f);
      } while (c != '\n' && !feof(f));
      continue;
    }
    else {
      double delta_t = (time-prev_time) / 1e6;
      const char *dev = c == 'S' ? "T" : "B";

      if (c == 'S')
        continue;

      Frame f(msg);
      printf("%6.3f %s %s", delta_t, dev, f.to_string());
      Application::ID *id = app.index[f.id()];
      Application::IDMeta &meta = MyApp::idmeta[f.id()];
      if (!id) {
        printf(" unknown data ID");
      }
      else {
        switch (f.msg_type()) {
          case ReadACK:
            printf(" %s == %s", meta.data_object, id->to_string(meta.type, f.value()));
            app.update(f.id(), f.value());
            break;
          case WriteACK:
            printf(" %s := %s", meta.data_object, id->to_string(meta.type, f.value()));
            app.update(f.id(), f.value());
            break;
        }
      }
      if (f.id() == 0) {
        printf("  fault: %d ch: %d dhw: %d flame: %d",
          (f.value() & 0x01) != 0,
          (f.value() & 0x02) != 0,
          (f.value() & 0x04) != 0,
          (f.value() & 0x08) != 0);
      }
      printf("\n");
    }

    prev_time = time;
  }

  fclose(f);

  return 0;
}
