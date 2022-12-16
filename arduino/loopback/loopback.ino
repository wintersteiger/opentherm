static FILE uartf = {0};
static const unsigned master_in = 2, master_out = 4;
static const unsigned slave_in = 3, slave_out = 5;

static int uart_putchar(char c, FILE *stream)
{
  Serial.write(c);
  return 0;
}

void setup()
{
  Serial.begin(115200);
  while (!Serial);

  fdev_setup_stream(&uartf, uart_putchar, NULL, _FDEV_SETUP_WRITE);
  stdout = &uartf;

  printf("Loopback test...\n");

  pinMode(master_in, INPUT_PULLUP);
  pinMode(master_out, OUTPUT);
  pinMode(slave_in, INPUT_PULLUP);
  pinMode(slave_out, OUTPUT);

  size_t i=0;
  while (true)
  {
    bool si0 = false, si1 = false;
    digitalWrite(master_out, LOW);
    delay(10);
    si0 = digitalRead(slave_in);
    digitalWrite(master_out, HIGH);
    delay(10);
    si1 = digitalRead(slave_in);

    bool mi0 = false, mi1 = false;
    digitalWrite(slave_out, LOW);
    delay(10);
    mi0 = digitalRead(master_in);
    digitalWrite(slave_out, HIGH);
    delay(10);
    mi1 = digitalRead(master_in);

    bool mosi_ok = si0 != si1;
    bool mosi_inv = mosi_ok && si0 && !si1;
    bool somi_ok = mi0 != mi1;
    bool somi_inv = somi_ok && mi0 && !mi1;

    const char* mosi_txt = "unresponsive";
    const char* somi_txt = mosi_txt;
    if (mosi_ok)
      mosi_txt = mosi_inv ? "inverted" : "not inverted";
    if (somi_ok)
      somi_txt = somi_inv ? "inverted" : "not inverted";
    printf("[%05d] mo/si %s (%d/%d)  so/mi %s (%d/%d)\n", i,
      mosi_txt, si0, si1,
      somi_txt, mi0, mi1);

    delay(1000);
    i++;
  }
}

void loop() {}
