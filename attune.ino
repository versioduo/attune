// © Kay Sievers <kay@versioduo.com>, 2020-2022
// SPDX-License-Identifier: Apache-2.0

#include <V2Base.h>
#include <V2Color.h>
#include <V2Device.h>
#include <V2LED.h>
#include <V2Link.h>
#include <V2MIDI.h>
#include <V2Music.h>
#include <V2PowerSupply.h>
#include <V2Stepper.h>

V2DEVICE_METADATA("net.voltek-labs.attune", 24, "versioduo:samd:step");

static V2LED::WS2812 LED(4, PIN_LED_WS2812, &sercom2, SPI_PAD_0_SCK_1, PIO_SERCOM);
static V2Link::Port Plug(&SerialPlug);
static V2Link::Port Socket(&SerialSocket);
static V2Base::Timer::Periodic Timer(2, 200000);
static V2Base::Analog::ADC ADC(V2Base::Analog::ADC::getID(PIN_VOLTAGE_SENSE));

enum {
  DriverRail,
  DriverSolenoid,
  DriverLamp,
};

static class Lamp : public V2Stepper::Power {
public:
  Lamp(const Power::Config conf, uint8_t index) :
    Power(conf, &SPI, PIN_DRIVER_SELECT + index, PIN_DRIVER_STEP + index),
    _index(index) {}

  void trigger(uint8_t channel, float power, float seconds) {
    if (seconds < 0.01f) {
      _pulse[channel].duration == 0;
      scaleVoltage(channel, 0);
      return;
    }

    _pulse[channel].usec     = micros();
    _pulse[channel].duration = seconds * 1000000;
    scaleVoltage(channel, power);
  }

  void loop() {
    for (uint8_t i = 0; i < 2; i++) {
      if (_pulse[i].duration == 0)
        continue;

      if ((unsigned long)(micros() - _pulse[i].usec) < _pulse[i].duration)
        continue;

      _pulse[i].duration = 0;
      scaleVoltage(i, 0);
    }
  }

private:
  const uint8_t _index;
  struct {
    unsigned long usec;
    unsigned long duration;
  } _pulse[2]{};

  void handleScaleVoltage(uint8_t channel, float fraction) {
    LED.setHSV(_index, V2Color::Cyan, 1, fraction < 0.01f ? 0 : 0.5);
  }
} Lamp({.ampere{1}}, DriverLamp);

static class Solenoid : public V2Stepper::Power {
public:
  Solenoid(const Power::Config conf, uint8_t index) :
    Power(conf, &SPI, PIN_DRIVER_SELECT + index, PIN_DRIVER_STEP + index),
    _index(index) {}

  void trigger(uint8_t channel, float power, float seconds) {
    if (seconds < 0.01f) {
      _pulse[channel].duration = 0;
      scaleVoltage(channel, 0);
      return;
    }

    scaleVoltage(channel, power);
    _pulse[channel].usec     = micros();
    _pulse[channel].duration = seconds * 1000000;
    scaleVoltage(channel, power);
  }

  void loop() {
    for (uint8_t i = 0; i < 2; i++) {
      if (_pulse[i].duration == 0)
        continue;

      if ((unsigned long)(micros() - _pulse[i].usec) < _pulse[i].duration)
        continue;

      _pulse[i].duration = 0;
      scaleVoltage(i, 0);
    }
  }

private:
  const uint8_t _index;
  struct {
    unsigned long usec;
    unsigned long duration;
  } _pulse[2]{};

  void handleScaleVoltage(uint8_t channel, float fraction) {
    LED.setHSV(_index, V2Color::Magenta, 1, fraction < 0.01f ? 0 : 0.5);
  }
} Pulse({.ampere{1.5}}, DriverSolenoid);

static class Stepper : public V2Stepper::Motor {
public:
  Stepper(const Motor::Config conf, uint8_t index) :
    Motor(conf, &Timer, &SPI, PIN_DRIVER_SELECT + index, PIN_DRIVER_STEP + index),
    _index(index) {}

  void positionNote(uint8_t note, float speed, float octave_length) {
    // Do not move the head while the trigger is in-between the keys.
    if ((unsigned long)(micros() - _usec) < 350000)
      return;

    position(getNoteSteps(note, octave_length), speed);
  }

  void playNote(uint8_t note, float volume, float octave_length) {
    if (getNoteSteps(note, octave_length) != (uint32_t)getPosition())
      return;

    const float v = 0.5f + (0.5f * volume);
    Pulse.trigger(V2Music::Keyboard::isBlackKey(note) ? 0 : 1, v, 0.15);
    _usec = micros();
  }

private:
  const uint8_t _index;
  unsigned long _usec = 0;

  uint32_t getNoteSteps(uint8_t note, float octave_length) {
    // 40 teeth wheel == 80 mm.
    const float turns = V2Music::Keyboard::getKeyDistance(note, octave_length) / 0.08;
    return turns * 200;
  }

  void handleMovement(Move move) override {
    switch (move) {
      case Move::Forward:
        LED.setHSV(_index, V2Color::Blue, 1, 0.5);
        break;

      case Move::Reverse:
        LED.setHSV(_index, V2Color::Yellow, 1, 0.5);
        break;
      case Move::Stop:
        LED.setBrightness(_index, 0);
    }
  }
} Stepper(
  {
    .ampere{1.1},
    .microstepsShift{3},
    .home{.speed{200}, .stall{0.09}},
    .speed{.min{25}, .max{1200}, .accel{1500}},
  },
  DriverRail);

static class Power : public V2PowerSupply {
public:
  Power() : V2PowerSupply({.min{6}, .max{26}}) {}

  void begin() {
    pinMode(PIN_DRIVER_ENABLE, OUTPUT);
    digitalWrite(PIN_DRIVER_ENABLE, HIGH);
  }

private:
  float handleMeasurement() override {
    // A voltage 10/100k divider.
    return 36.f * ADC.readChannel(V2Base::Analog::ADC::getChannel(PIN_VOLTAGE_SENSE));
  }

  void handleOn() override {
    digitalWrite(PIN_DRIVER_ENABLE, LOW);
  }

  void handleOff() override {
    digitalWrite(PIN_DRIVER_ENABLE, HIGH);
  }

  void handleNotify(float voltage) override {
    // Power interruption, or commands without a power connection show yellow LEDs.
    if (voltage < config.min) {
      LED.splashHSV(0.5, V2Color::Yellow, 1, 0.5);
      return;
    }

    // Over-voltage shows red LEDs.
    if (voltage > config.max) {
      LED.splashHSV(0.5, V2Color::Red, 1, 1);
      return;
    }

    // The number of green LEDs shows the voltage.
    float fraction = voltage / (float)config.max;
    uint8_t n      = ceil((float)LED.getNumLEDs() * fraction);
    LED.splashHSV(0.5, n, V2Color::Green, 1, 0.5);
  }
} Power;

static class Device : public V2Device {
public:
  Device() : V2Device() {
    metadata.vendor      = "Voltek Labs";
    metadata.product     = "attune";
    metadata.description = "Piano One";
    metadata.home        = "https://voltek-labs.net/attune";

    system.download  = "https://versioduo.com/download";
    system.configure = "https://versioduo.com/configure";

    // https://github.com/versioduo/arduino-board-package/blob/master/boards.txt
    usb.pid = 0xef20;

    configuration = {.size{sizeof(config)}, .data{&config}};
  }

  enum class CC {
    Volume = V2MIDI::CC::ChannelVolume,
  };

  // Config, written to EEPROM.
  struct {
    struct {
      uint8_t start{V2MIDI::A(-1)};
      uint8_t count{88};
      float offset{0.05};
      float octave_width{0.166};
    } keys;
  } config;

private:
  float _volume    = 1;
  bool _home       = false;
  float _speed_max = 1;

  void handleReset() override {
    LED.reset();
    Lamp.reset();
    Pulse.reset();
    Stepper.reset();
    _volume    = 1;
    _home      = false;
    _speed_max = 1;
    Power.off();
  }

  void allNotesOff() {
    if (!power())
      return;

    // ~1.3m maximum distance, 40 teeth wheel == 80 mm.
    const float max_steps    = (1.3f / 0.08f) * 200.f;
    const float offset_steps = (config.keys.offset / 0.08f) * 200.f;
    Stepper.home(max_steps, offset_steps);
    _home = true;
  }

  bool power() {
    bool continuous;
    if (!Power.on(continuous))
      return false;

    if (!continuous) {
      Lamp.reset();
      Pulse.reset();
      Stepper.reset();
      _home = false;
    }

    return true;
  }

  void handleNote(uint8_t channel, uint8_t note, uint8_t velocity) override {
    if (note < config.keys.start)
      return;

    if (note >= config.keys.start + config.keys.count)
      return;

    if (!power())
      return;

    if (!_home)
      allNotesOff();

    if (velocity == 0) {
      Stepper.playNote(note, _volume, config.keys.octave_width);
      return;
    }

    const float fraction = (float)velocity / 127;
    Stepper.positionNote(note, fraction * _speed_max, config.keys.octave_width);
  }

  void handleNoteOff(uint8_t channel, uint8_t note, uint8_t velocity) override {
    handleNote(channel, note, 0);
  }

  void handleControlChange(uint8_t channel, uint8_t controller, uint8_t value) override {
    switch (controller) {
      case V2MIDI::CC::ModulationWheel:
        _speed_max = (float)value / 127;
        break;

      case V2MIDI::CC::BreathController: {
        const float fraction = (float)value / 127;
        Lamp.trigger(0, 0.5f * fraction, 10);
        break;
      }

      case (uint8_t)CC::Volume:
        _volume = (float)value / 127;
        break;

      case V2MIDI::CC::AllSoundOff:
      case V2MIDI::CC::AllNotesOff:
        allNotesOff();
        break;
    }
  }

  void handleSystemReset() override {
    reset();
  }

  void exportSystem(JsonObject json) override {
    JsonObject json_power       = json["power"].to<JsonObject>();
    json_power["voltage"]       = serialized(String(Power.getVoltage(), 1));
    json_power["interruptions"] = Power.getInterruptions();
  }

  void exportInput(JsonObject json) override {
    JsonArray json_controller = json["controllers"].to<JsonArray>();

    JsonObject json_speed = json_controller.add<JsonObject>();
    json_speed["name"]    = "Speed";
    json_speed["number"]  = V2MIDI::CC::ModulationWheel;
    json_speed["value"]   = _speed_max * 127;

    JsonObject json_light = json_controller.add<JsonObject>();
    json_light["name"]    = "Brightness";
    json_light["number"]  = V2MIDI::CC::BreathController;
    json_light["value"]   = 0;

    JsonObject json_volume = json_controller.add<JsonObject>();
    json_volume["name"]    = "Volume";
    json_volume["number"]  = (uint8_t)CC::Volume;
    json_volume["value"]   = _volume * 127;

    JsonObject json_chromatic = json["chromatic"].to<JsonObject>();
    json_chromatic["start"]   = config.keys.start;
    json_chromatic["count"]   = config.keys.count;
  }

  void importConfiguration(JsonObject json) override {
    JsonObject json_keys = json["keys"];
    if (json_keys) {
      uint8_t start = json_keys["start"];
      if (start > 0)
        config.keys.start = start;

      uint8_t count = json_keys["count"];
      if (count > 0)
        config.keys.count = count;

      if (!json_keys["offset"].isNull())
        config.keys.offset = json_keys["offset"];

      const float width = json_keys["width"];
      if (width > 0)
        config.keys.octave_width = width;
    }
  }

  void exportConfiguration(JsonObject json) override {
    JsonObject json_keys = json["keys"].to<JsonObject>();
    json_keys["#start"]  = "The MIDI number of the first note";
    json_keys["start"]   = config.keys.start;

    json_keys["#count"] = "The total number of keys";
    json_keys["count"]  = config.keys.count;

    json_keys["#offset"] = "The start of the first key (meters)";
    json_keys["offset"]  = config.keys.offset;

    json_keys["#width"] = "The width of one octave (meters)";
    json_keys["width"]  = config.keys.octave_width;
  }
} Device;

// Dispatch MIDI packets
static class MIDI {
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

// Dispatch Link packets
static class Link : public V2Link {
public:
  Link() : V2Link(&Plug, &Socket) {}

private:
  V2MIDI::Packet _midi{};

  // Receive a host event from our parent device
  void receivePlug(V2Link::Packet *packet) override {
    if (packet->getType() == V2Link::Packet::Type::MIDI) {
      packet->receive(&_midi);
      Device.dispatch(&Plug, &_midi);
    }
  }

  // Forward children device events to the host
  void receiveSocket(V2Link::Packet *packet) override {
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

void setup() {
  Serial.begin(9600);
  SPI.begin();

  for (uint8_t i = 0; i < 4; i++) {
    pinMode(PIN_DRIVER_SELECT + i, OUTPUT);
    digitalWrite(PIN_DRIVER_SELECT + i, HIGH);

    pinMode(PIN_DRIVER_STEP + i, OUTPUT);
    digitalWrite(PIN_DRIVER_STEP + i, LOW);
  }

  LED.begin();
  LED.setMaxBrightness(0.5);

  Plug.begin();
  Socket.begin();
  Device.link = &Link;

  // Set the SERCOM interrupt priority, it requires a stable ~300 kHz interrupt
  // frequency. This needs to be after begin().
  setSerialPriority(&SerialPlug, 2);
  setSerialPriority(&SerialSocket, 2);

  Power.begin();
  Lamp.begin();
  Pulse.begin();
  Stepper.begin();

  // The priority needs to be lower than the SERCOM priorities.
  Timer.begin([]() { Stepper.tick(); });
  Timer.setPriority(3);

  ADC.begin();
  ADC.addChannel(V2Base::Analog::ADC::getChannel(PIN_VOLTAGE_SENSE));

  Device.begin();
  Device.reset();
}

void loop() {
  Stepper.loop();
  Lamp.loop();
  Pulse.loop();
  LED.loop();
  MIDI.loop();
  Link.loop();
  Power.loop();
  Device.loop();

  if (Link.idle() && Device.idle())
    Device.sleep();
}
