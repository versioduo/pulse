#include <V2Buttons.h>
#include <V2Device.h>
#include <V2LED.h>
#include <V2Link.h>
#include <V2MIDI.h>
#include <V2PowerSupply.h>
#include <V2Solenoids.h>

V2DEVICE_METADATA("com.versioduo.pulse", 100, "versioduo:samd:pulse");

namespace {
  constexpr uint8_t nPorts{16};
  V2LED::WS2812     LED(nPorts + 2, PIN_LED_WS2812, &sercom2, SPI_PAD_0_SCK_1, PIO_SERCOM);
  V2Link::Port      Plug(&SerialPlug, PIN_SERIAL_PLUG_TX_ENABLE);
  V2Link::Port      Socket(&SerialSocket, PIN_SERIAL_SOCKET_TX_ENABLE);

  // Try to spread the power switching noise; run the timers with slightly
  // different periods, so they don't all start the rising edge of the PWM
  // period at the same time.
  std::array PWM{
    V2Base::Timer::PWM(0, 20000),
    V2Base::Timer::PWM(1, 20100),
    V2Base::Timer::PWM(2, 20200),
    V2Base::Timer::PWM(3, 20300),
    V2Base::Timer::PWM(4, 20400),
  };

  V2LED::Basic* DeviceLED{};

  std::array ADC{
    V2Base::Analog::ADC(0),
    V2Base::Analog::ADC(1),
  };

  class Power : public V2PowerSupply {
  public:
    constexpr Power() : V2PowerSupply({.min{6}, .max{26}}) {}

    void begin() {
      pinMode(PIN_POWER_ENABLE, OUTPUT);
      digitalWrite(PIN_POWER_ENABLE, LOW);
    }

  private:
    float handleMeasurement() override {
      const uint8_t id      = V2Base::Analog::ADC::getID(PIN_VOLTAGE_SENSE);
      const uint8_t channel = V2Base::Analog::ADC::getChannel(PIN_VOLTAGE_SENSE);
      return 36.f * ADC[id].readChannel(channel);
    }

    void handleOn() override {
      digitalWrite(PIN_POWER_ENABLE, HIGH);
      DeviceLED->setBrightness(0.1);
    }

    void handleOff() override {
      digitalWrite(PIN_POWER_ENABLE, LOW);
      DeviceLED->setBrightness(0);
    }

    void handleNotify(float voltage) override {
      // Loss of power or requests to switch-on without a power connection show yellow LEDs.
      if (voltage < config.min) {
        LED.splashHSV(0.5, V2Colour::Yellow, 1, 0.5);
        return;
      }

      // Over-voltage shows red LEDs.
      if (voltage > config.max) {
        LED.splashHSV(0.5, V2Colour::Red, 1, 1);
        return;
      }

      // A bar of green LEDs show the voltage.
      const float   fraction = voltage / (float)config.max;
      const uint8_t n        = ceil((float)nPorts * fraction);
      LED.splashHSV(0.5, 0, n, V2Colour::Green, 1, 0.5);
    }
  } Power;

  class Solenoids : public V2Solenoids<nPorts> {
  public:
    constexpr Solenoids() :
      V2Solenoids({
        .current{.max{3}, .alpha{0.001}},
        .resistance{.min{3}, .max{90}},
        .fade{.inSec{0.35}, .outSec{0.35}},
        .hold{.peakUsec{100 * 1000}, .fraction{0.5}},
      }) {}

    void begin(uint8_t revision) {
      _revision = revision;
    }

    bool setPower(PowerState state) override {
      switch (state) {
        case PowerState::On:
          return Power.on();

        case PowerState::Off:
          Power.off();
          return true;
      }

      return false;
    }

    float readVoltage() override {
      return Power.getVoltage();
    }

    float readCurrent() override {
      const uint8_t id      = V2Base::Analog::ADC::getID(PIN_CURRENT_SENSE);
      const uint8_t channel = V2Base::Analog::ADC::getChannel(PIN_CURRENT_SENSE);
      const float   voltage = 3.3f * ADC[id].readChannel(channel);

      // Current sense ratio 500, 330 Ω sense resistor, 5 A maximum range.
      if (_revision == 0)
        return voltage / 330.f * 500.f;

      // Current sense ratio 3900, 1350 Ω sense resistor, 9.5 A maximum range.
      return voltage / 1350.f * 3900.f;
    }

    // One single port at a time is connected to 3.3V. The load and a 100Ω resistor
    // form a voltage divider.
    float readResistanceVoltage() override {
      const uint8_t id      = V2Base::Analog::ADC::getID(PIN_RESISTANCE_SENSE);
      const uint8_t channel = V2Base::Analog::ADC::getChannel(PIN_RESISTANCE_SENSE);
      return 3.3f * ADC[id].readChannel(channel);
    }

    void setLED(LEDMode state, uint8_t port = 0, float value = -1) override {
      if (LED.isRainbow())
        return;

      switch (state) {
        case LEDMode::Off:
          LED.setBrightness(port, 0);
          break;

        case LEDMode::Initialize:
          LED.setHSV(nPorts + 0, V2Colour::Cyan, 1, 0.25);
          LED.setHSV(nPorts + 1, V2Colour::Cyan, 1, 0.25);
          break;

        case LEDMode::Ready:
          LED.setHSV(nPorts + 0, V2Colour::Orange, 1, 0.25);
          LED.setHSV(nPorts + 1, V2Colour::Orange, 1, 0.25);
          break;

        case LEDMode::Resistance:
          // Map the fraction of the configured resistance range from cyan to magenta.
          LED.setHSV(port, (float)V2Colour::Cyan + (120.f * value), 1, 0.15);
          break;

        case LEDMode::Power: {
          const float fraction = powf(value / 100.f, 8);
          LED.setHSV(port, V2Colour::Orange, 1, 0.3f + (0.3f * fraction));
        } break;

        case LEDMode::ShortCircuit:
          LED.setHSV(port, V2Colour::Red, 1, 1);
          break;

        case LEDMode::OverCurrent:
          LED.splashHSV(0.2, V2Colour::Magenta, 1, 1);
          break;
      }
    }

    void setPWMDuty(uint8_t port, float duty) override {
      const uint8_t id = V2Base::Timer::PWM::getID(PIN_PWM_CHANNEL + port);
      PWM[id].setDuty(PIN_PWM_CHANNEL + port, duty);
    }

  private:
    uint8_t _revision{};
  } Solenoids;

  // Config, written to EEPROM.
  constexpr struct Configuration {
    struct {
      uint8_t note;
      struct {
        float watts{0.5};
        float seconds{0.1};
      } min;
      struct {
        float watts{6};
        float seconds{0.05};
      } max;
      bool fadeIn{};
      bool fadeOut{};
    } outputs[nPorts]{
      {.note{V2MIDI::C(3)}},
      {.note{V2MIDI::Cs(3)}},
      {.note{V2MIDI::D(3)}},
      {.note{V2MIDI::Ds(3)}},
      {.note{V2MIDI::E(3)}},
      {.note{V2MIDI::F(3)}},
      {.note{V2MIDI::Fs(3)}},
      {.note{V2MIDI::G(3)}},
      {.note{V2MIDI::Gs(3)}},
      {.note{V2MIDI::A(3)}},
      {.note{V2MIDI::As(3)}},
      {.note{V2MIDI::B(3)}},
      {.note{V2MIDI::C(4)}},
      {.note{V2MIDI::Cs(4)}},
      {.note{V2MIDI::D(4)}},
      {.note{V2MIDI::Ds(4)}},
    };
  } ConfigurationDefault;

  class Device : public V2Device {
  public:
    constexpr Device() : V2Device() {
      metadata.description = "16 Channel Solenoid Controller";
      metadata.vendor      = "Versio Duo";
      metadata.product     = "V2 pulse";
      metadata.home        = "https://versioduo.com/#pulse";

      help.device = "Intelligent solenoid power controller, automatically adjusts to changes in supply voltage "
                    "or solenoid resistance.\n"
                    "It is driven by commands which specify the length in seconds and the power in watts of a solenoid "
                    "trigger impulse. It calculates the PWM duty cycle using the measured solenoid resistance and the "
                    "supplied voltage.\n"
                    "To protect itself, it monitors the overall current draw from the power supply, and isolates "
                    "channels which are short cut.";

      help.configuration = "# Playing Notes\n"
                           "The 16 channels are configured to listen to incoming notes. The watts and seconds "
                           "values of the note velocity 1 and 127 are configured. The velocity of the incoming note "
                           "is then used to calculate a pulse in the configured range.\n"
                           "A currently active pulse will end if a NoteOff is received. By configuring larger values "
                           "for seconds, this can be used to drive actors with the actual note length.";

      system.download  = "https://versioduo.com/download";
      system.configure = "https://versioduo.com/configure";

      configuration = {.version{1}, .size{sizeof(config)}, .data{&config}};
      DeviceLED     = &led;
    }

    enum class CC {
      Volume = V2MIDI::CC::ChannelVolume,
    };

    Configuration config{ConfigurationDefault};

  private:
    uint8_t _volume{100};

    void handleReset() override {
      Power.off();
      _volume = 100;
      LED.reset();
      Solenoids.reset();
    }

    float adjustVolume(float fraction) {
      if (_volume < 100) {
        const float range = (float)_volume / 100.f;
        return fraction * range;
      }

      const float range = (float)(_volume - 100) / 27.f;
      return powf(fraction, 1 - (0.5f * range));
    }

    void play(uint8_t note, uint8_t velocity) {
      for (uint8_t i = 0; i < nPorts; i++) {
        if (config.outputs[i].note != note)
          continue;

        if (velocity == 0) {
          Solenoids.triggerPort(i, 0, 0, false, config.outputs[i].fadeOut);
          continue;
        }

        if (_volume > 0) {
          float fraction = (float)velocity / 127.f;
          fraction       = powf(fraction, 3);
          fraction       = adjustVolume(fraction);

          float watts = config.outputs[i].min.watts;
          watts += (config.outputs[i].max.watts - config.outputs[i].min.watts) * fraction;

          float seconds = config.outputs[i].min.seconds;
          seconds += (config.outputs[i].max.seconds - config.outputs[i].min.seconds) * fraction;

          Solenoids.triggerPort(i, watts, seconds, config.outputs[i].fadeIn, config.outputs[i].fadeOut);
        }
      }
    }

    void allNotesOff() {
      _volume = 100;

      for (uint8_t i = 0; i < nPorts; i++)
        Solenoids.triggerPort(i, 0, 0, false, true);
    }

    void handleNote(uint8_t channel, uint8_t note, uint8_t velocity) override {
      play(note, velocity);
    }

    void handleNoteOff(uint8_t channel, uint8_t note, uint8_t velocity) override {
      play(note, 0);
    }

    void handleControlChange(uint8_t channel, uint8_t controller, uint8_t value) override {
      if (channel != 0)
        return;

      switch (controller) {
        case V2MIDI::CC::AllSoundOff:
        case V2MIDI::CC::AllNotesOff:
          allNotesOff();
          break;

        case (uint8_t)CC::Volume:
          _volume = value;
          break;
      }
    }

    void handleSystemExclusive(const uint8_t* buffer, uint32_t len) override {
      if (len < 10)
        return;

      // 0x7d == SysEx prototype/research/private ID
      if (buffer[1] != 0x7d)
        return;

      // Handle only JSON messages.
      if (buffer[2] != '{' || buffer[len - 2] != '}')
        return;

      // Read incoming message.
      JsonDocument json;
      if (deserializeJson(json, buffer + 2, len - 1))
        return;

      JsonObject pulse = json["pulse"];
      if (!pulse)
        return;

      if (pulse["index"].isNull())
        return;

      const uint8_t index = pulse["index"];
      if (index > 15)
        return;

      const float watts = pulse["watts"];
      if (watts <= 0.f)
        return;

      const float seconds = pulse["seconds"];
      if (seconds <= 0.f)
        return;

      Solenoids.triggerPort(index, watts, seconds);
    }

    void handleSystemReset() override {
      reset();
    }

    void exportSystem(JsonObject json) override {
      JsonObject jsonPower       = json["power"].to<JsonObject>();
      jsonPower["voltage"]       = serialized(String(Power.getVoltage(), 1));
      jsonPower["interruptions"] = Power.getInterruptions();

      if (Power.isOn())
        jsonPower["current"] = serialized(String(Solenoids.getCurrent(), 1));

      JsonArray jsonOutputs = json["outputs"].to<JsonArray>();
      for (uint8_t i = 0; i < nPorts; i++) {
        JsonObject jsonOutput    = jsonOutputs.add<JsonObject>();
        jsonOutput["resistance"] = serialized(String(Solenoids.getResistance(i), 1));
      }
    }

    void exportSettings(JsonArray json) override {
      for (uint8_t i = 0; i < nPorts; i++) {
        {
          JsonObject setting = json.add<JsonObject>();
          setting["type"]    = "title";

          char name[16];
          sprintf(name, "Output %d", i + 1);
          setting["title"] = name;
        }
        {
          JsonObject setting = json.add<JsonObject>();
          setting["type"]    = "note";
          setting["label"]   = "Note";

          setting["default"] = ConfigurationDefault.outputs[i].note;

          char path[64];
          sprintf(path, "outputs[%d]/note", i);
          setting["path"] = path;
        }
        {
          JsonObject setting = json.add<JsonObject>();
          setting["type"]    = "pulse";
          setting["ruler"]   = true;
          setting["label"]   = "Min";
          setting["index"]   = i;

          JsonObject defaults = setting["default"].to<JsonObject>();
          defaults["watts"]   = serialized(String(ConfigurationDefault.outputs[i].min.watts, 1));
          defaults["seconds"] = serialized(String(ConfigurationDefault.outputs[i].min.seconds, 3));

          char path[64];
          sprintf(path, "outputs[%d]/min", i);
          setting["path"] = path;
        }
        {
          JsonObject setting = json.add<JsonObject>();
          setting["type"]    = "pulse";
          setting["ruler"]   = true;
          setting["label"]   = "Max";
          setting["index"]   = i;

          JsonObject defaults = setting["default"].to<JsonObject>();
          defaults["watts"]   = serialized(String(ConfigurationDefault.outputs[i].max.watts, 1));
          defaults["seconds"] = serialized(String(ConfigurationDefault.outputs[i].max.seconds, 3));

          char path[64];
          sprintf(path, "outputs[%d]/max", i);
          setting["path"] = path;
        }
        {
          JsonObject setting = json.add<JsonObject>();
          setting["type"]    = "toggle";
          setting["ruler"]   = true;
          setting["label"]   = "Fade In";

          char path[64];
          sprintf(path, "outputs[%d]/fadeIn", i);
          setting["path"] = path;
        }
        {
          JsonObject setting = json.add<JsonObject>();
          setting["type"]    = "toggle";
          setting["label"]   = "Fade Out";

          char path[64];
          sprintf(path, "outputs[%d]/fadeOut", i);
          setting["path"] = path;
        }
      }
    }

    void exportConfiguration(JsonObject json) override {
      json["#outputs"]      = "The pulse parameters when playing notes";
      JsonArray jsonOutputs = json["outputs"].to<JsonArray>();

      for (uint8_t i = 0; i < nPorts; i++) {
        JsonObject jsonOutput = jsonOutputs.add<JsonObject>();

        if (i == 0)
          jsonOutput["#note"] = "Note number";
        jsonOutput["note"] = config.outputs[i].note;

        if (i == 0)
          jsonOutput["#min"] = "Minimum (velocity 1)";
        JsonObject jsonMin = jsonOutput["min"].to<JsonObject>();
        jsonMin["watts"]   = serialized(String(config.outputs[i].min.watts, 1));
        jsonMin["seconds"] = serialized(String(config.outputs[i].min.seconds, 3));

        if (i == 0)
          jsonOutput["#max"] = "Maximum (velocity 127)";
        JsonObject jsonMax = jsonOutput["max"].to<JsonObject>();
        jsonMax["watts"]   = serialized(String(config.outputs[i].max.watts, 1));
        jsonMax["seconds"] = serialized(String(config.outputs[i].max.seconds, 3));

        if (i == 0)
          jsonOutput["#fadeIn"] = "Soft start";
        jsonOutput["fadeIn"] = config.outputs[i].fadeIn;

        if (i == 0)
          jsonOutput["#fadeOut"] = "Soft stop";
        jsonOutput["fadeOut"] = config.outputs[i].fadeOut;
      }
    }

    void importConfiguration(JsonObject json) override {
      JsonArray jsonOutputs = json["outputs"];
      if (jsonOutputs) {
        for (uint8_t i = 0; i < nPorts; i++) {
          JsonObject jsonOutput = jsonOutputs[i];

          if (!jsonOutput["note"].isNull()) {
            uint8_t note = jsonOutput["note"];
            if (note > 127 - (nPorts - 1))
              note = 127 - (nPorts - 1);

            config.outputs[i].note = note;
          }

          JsonObject jsonMin = jsonOutput["min"];
          if (jsonMin) {
            if (!jsonMin["watts"].isNull()) {
              float watts = jsonMin["watts"];
              if (watts < 0.f) {
                watts = 0;
              }

              else if (watts > 100.f) {
                watts = 100;
              }
              config.outputs[i].min.watts = watts;
            }

            if (!jsonMin["seconds"].isNull()) {
              float seconds = jsonMin["seconds"];
              if (seconds < 0.f) {
                seconds = 0;
              }

              else if (seconds > 100.f) {
                seconds = 100;
              }
              config.outputs[i].min.seconds = seconds;
            }
          }

          JsonObject jsonMax = jsonOutput["max"];
          if (jsonMax) {
            if (!jsonMax["watts"].isNull()) {
              float watts = jsonMax["watts"];
              if (watts < 0.f) {
                watts = 0;
              }

              else if (watts > 100.f) {
                watts = 100;
              }
              config.outputs[i].max.watts = watts;
            }

            if (!jsonMax["seconds"].isNull()) {
              float seconds = jsonMax["seconds"];
              if (seconds < 0.f) {
                seconds = 0;
              }

              else if (seconds > 100.f) {
                seconds = 100;
              }
              config.outputs[i].max.seconds = seconds;
            }
          }

          if (!jsonOutput["fadeIn"].isNull())
            config.outputs[i].fadeIn = jsonOutput["fadeIn"];

          if (!jsonOutput["fadeOut"].isNull())
            config.outputs[i].fadeOut = jsonOutput["fadeOut"];
        }
      }

      reset();
    }

    void exportInput(JsonObject json) override {
      JsonArray jsonControllers = json["controllers"].to<JsonArray>();
      {
        JsonObject jsonController = jsonControllers.add<JsonObject>();
        jsonController["name"]    = "Volume";
        jsonController["number"]  = (uint8_t)CC::Volume;
        jsonController["value"]   = _volume;
      }

      JsonArray jsonNotes = json["notes"].to<JsonArray>();
      for (uint8_t i = 0; i < nPorts; i++) {
        JsonObject jsonNote = jsonNotes.add<JsonObject>();

        char name[16];
        sprintf(name, "Output %d", i + 1);
        jsonNote["name"]   = name;
        jsonNote["number"] = config.outputs[i].note;
      }
    }
  } Device;

  // Dispatch MIDI packets.
  class MIDI {
  public:
    void loop() {
      if (!Device.usb.midi.receive(&_midi))
        return;

      if (_midi.getPort() == 0) {
        Device.dispatch(&Device.usb.midi, &_midi);

      } else {
        _midi.setPort(_midi.getPort() - 1);
        Socket.send(&_midi);
      }
    }

  private:
    V2MIDI::Packet _midi{};
  } MIDI;

  // Dispatch Link packets.
  class Link : public V2Link {
  public:
    constexpr Link() : V2Link(&Plug, &Socket) {
      Device.link = this;
    }

  private:
    V2MIDI::Packet _midi{};

    // Receive a host event from our parent device.
    void receivePlug(V2Link::Packet* packet) override {
      switch (packet->getType()) {
        case V2Link::Packet::Type::MIDI:
          packet->receive(&_midi);
          Device.dispatch(&Plug, &_midi);
          break;

        case V2Link::Packet::Type::Pulse: {
          V2Link::Packet::Pulse pulse;
          packet->getPulse(&pulse);
          Solenoids.triggerPort(pulse.port, pulse.watts, pulse.seconds, pulse.fadeIn, pulse.fadeOut);
        } break;
      }
    }

    // Forward children device events to the host.
    void receiveSocket(V2Link::Packet* packet) override {
      if (packet->getType() == V2Link::Packet::Type::MIDI) {
        uint8_t address = packet->getAddress();
        if (address == 0x0f)
          return;

        if (Device.usb.midi.connected()) {
          packet->receive(&_midi);
          _midi.setPort(address + 1);
          Device.usb.midi.send(&_midi);
        }
      }
    }
  } Link;

  class Button : public V2Buttons::Button {
  public:
    constexpr Button() : V2Buttons::Button(&_config, PIN_BUTTON) {}

  private:
    const V2Buttons::Config _config{.clickUsec{200 * 1000}, .holdUsec{500 * 1000}};

    void handleHold(uint8_t count) override {
      LED.rainbow();
    }

    void handleRelease() override {
      Device.reset();
    }

    void handleClick(uint8_t count) override {
      Device.reset();
    }
  } Button;
}

auto setup() -> void {
  Serial.begin(9600);

  LED.begin();
  LED.setMaxBrightness(0.5);

  // Set the SERCOM interrupt priority, it requires a stable ~300 kHz interrupt
  // frequency. This needs to be after begin().
  Link.begin();
  setSerialPriority(&SerialPlug, 2);
  setSerialPriority(&SerialSocket, 2);

  Power.begin();

  for (auto& p : PWM)
    p.begin();

  for (uint8_t i{}; i < nPorts; i++)
    V2Base::Timer::PWM::setupPin(PIN_PWM_CHANNEL + i);

  for (auto& a : ADC)
    a.begin();

  for (uint8_t p : {PIN_RESISTANCE_SENSE, PIN_CURRENT_SENSE, PIN_VOLTAGE_SENSE})
    ADC[V2Base::Analog::ADC::getID(p)].addChannel(V2Base::Analog::ADC::getChannel(p));

  Button.begin();
  Device.begin();
  Solenoids.begin(Device.system.revision);
  Device.reset();
}

auto loop() -> void {
  LED.loop();
  MIDI.loop();
  Link.loop();
  Power.loop();
  V2Buttons::loop();
  Solenoids.loop();
  Device.loop();

  if (Link.idle() && Device.idle())
    Device.sleep();
}
