#include "TimerOne.h"

#ifdef __AVR__
#include <avr/power.h>
#endif

// Neopixel stuff:
#include <Adafruit_NeoPixel.h>
constexpr unsigned neo_pin = 7;
constexpr unsigned buzzerPin1 = 5;
constexpr unsigned buzzerPin2 = 18;
constexpr unsigned samplerate = 1; // microsec;

struct Button {

  int pin;
  bool state = false;
  bool last_state = false;

  Button(int pin) : pin(pin) { pinMode(pin, INPUT_PULLUP); }

  void tick() {
    last_state = state;
    state = !(digitalRead(pin) == HIGH);
  }

  bool is_rising() { return !last_state && state; }

  bool is_falling() { return last_state && !state; }

  bool changed() { return last_state != state; }
};

Button btn_kick = Button{3};
Button btn_snare = Button{8};
Button btn_hat = Button{21};

Button btn_play = Button{14};
Button btn_write = Button{2};

using Notes = unsigned char;
using uchar = unsigned char;

enum Note {
  none = 0b0000,
  kick = 0b0001,
  snare = 0b0010,
  hat = 0b0100,
};

unsigned triangle(unsigned time, unsigned cycle, uchar amp = 255) {
  unsigned timemod = time % cycle;
  if (timemod < cycle / 2) {
    return timemod * amp / (cycle / 2);
  } else {
    return amp - (timemod - cycle / 2) * amp / (cycle / 2);
  }
}

unsigned square(unsigned time, unsigned cycle, uchar amp = 255) {
  if (time % cycle < cycle / 2)
    return amp;
  return 0;
}

unsigned noise(unsigned time, unsigned amp) { return rand(); }

struct Sound {

  long time = -1;

  void trigger() { time = 0; }

  virtual uchar sample(long timediff) = 0;
};

struct Kick final : Sound {
  uchar sample(long timediff) final {
    constexpr static auto length = 250000;
    if (time < 0)
      return 0;
    time += timediff;
    if (time < length / 2)
      return square(time / 100, 300 + (200 * (time) / length));
    return 0;
  }
};

struct Snare final : Sound {
  uchar sample(long timediff) final {
    constexpr static auto length = 250000;
    if (time < 0)
      return 0;
    time += timediff;
    if (time < length / 16)
      return square(time / 100, 30 * ((length / 2) - time) / (length / 2));
    return 0;
  }
};

struct Hat final : Sound {
  uchar sample(long timediff) final {
    constexpr static auto length = 250000;
    if (time < 0)
      return 0;
    time += timediff;
    if (time < length / 6)
      return square(time / 100, 300 * (time) / (length / 2));
    return 0;
  }
};

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(10, neo_pin, NEO_GRB + NEO_KHZ800);

struct Sequencer {

  static constexpr unsigned char length = 16;

  Notes notes[length] = {
      Note::kick | Note::hat, Note::hat, Note::snare | Note::hat, Note::hat,
      Note::kick | Note::hat, Note::hat, Note::snare | Note::hat, Note::hat,
      Note::kick | Note::hat, Note::hat, Note::snare | Note::hat, Note::hat,
      Note::kick | Note::hat, Note::hat, Note::snare | Note::hat, Note::kick | Note::hat
      };

  Notes current_notes = Note::none;
  unsigned char cur_step = 0;
  unsigned long beat_length = 250000;
  unsigned long last_time;
  unsigned long time = 0;
  unsigned long time_zero = micros();
  bool playing = false;
  bool writing = false;
  bool pixel_damage = false;

  Kick kick;
  Snare snare;
  Hat hat;

  Sequencer() {}

  uchar nearest() {
    if (!playing)
      return cur_step;
    // Quantize
    return (time <= (beat_length / 2)) ? cur_step : ((cur_step + 1) % length);
  }

  void record(Note note) { notes[nearest()] |= note; }

  void remove(Note note) { notes[nearest()] &= ~note; }

  void rec_or_rem(Note note) {
    if (!btn_write.state)
      record(note);
    else
      remove(note);
  }

  void clear() {
    for (char i = 0; i < length; i++) {
      notes[i] = Note::none;
    }
    cur_step = 0;
    playing = false;
    time_zero = micros();
    time = 0;
  }

  void update_pixels() {
    pixel_damage = true;
  }

  void do_update_pixels() {
    for (char i = 0; i < pixels.numPixels(); i++) {
      char stepmod = cur_step % 8;
      if (stepmod == i) {
        pixels.setPixelColor(i, pixels.Color(20, 20, 20));
      } else if (i < 8) {
        uchar idx = (cur_step < 8) ? i : i + 8;
        pixels.setPixelColor(i,
                             pixels.Color((notes[idx] & Note::kick) * 15,
                                          (notes[idx] & Note::snare) * 5, (notes[idx] & Note::hat) * 1));
      } else if ((i == 8 && cur_step < 8) || (i == 9 && cur_step >= 8)) {
        if (writing)
          pixels.setPixelColor(i, pixels.Color(50, 0, 0));
        else
          pixels.setPixelColor(i, pixels.Color(30, 20, 0));
      } else {
        pixels.setPixelColor(i, pixels.Color(0, 0, 0));
      }
    }
    pixels.show();
    pixel_damage = false;
  }

  void step() {
    if (playing) {
      cur_step = ++cur_step % length;
      current_notes = notes[cur_step];
      time = 0;
      time_zero = micros();
      if (current_notes & Note::kick)
        kick.trigger();
      if (current_notes & Note::snare)
        snare.trigger();
      if (current_notes & Note::hat)
        hat.trigger();
    }
    update_pixels();
  }

  void loop1() {
    if (time >= beat_length) {
      step();
    }
  }

  void loop2() {
    last_time = time;
    time = micros() - time_zero;
    long timediff = time - last_time;
    digitalWrite(buzzerPin1, kick.sample(timediff) + snare.sample(timediff));
    digitalWrite(buzzerPin2, hat.sample(timediff));
    do_update_pixels();
  }

} seq;

void setup() {

  // These pins are ground
  pinMode(6, OUTPUT);
  digitalWrite(6, LOW);
  pinMode(10, OUTPUT);
  digitalWrite(10, LOW);
  pinMode(9, OUTPUT);
  digitalWrite(9, LOW);
  pinMode(19, OUTPUT);
  digitalWrite(19, LOW);
  pinMode(4, OUTPUT);
  digitalWrite(4, LOW);

  pinMode(buzzerPin1, OUTPUT);
  pinMode(buzzerPin2, OUTPUT);

  pixels.begin();
  pixels.show();
}

bool write_click = false;
bool clear_click = false;

void loop() {
  btn_kick.tick();
  btn_snare.tick();
  btn_hat.tick();
  btn_play.tick();
  btn_write.tick();


  if (btn_kick.is_rising()) {
    seq.kick.trigger();
    if (seq.writing)
      seq.rec_or_rem(Note::kick);
    write_click = false;
    seq.update_pixels();
  }
  if (btn_snare.is_rising()) {
    seq.snare.trigger();
    if (seq.writing)
      seq.rec_or_rem(Note::snare);
    write_click = false;
    seq.update_pixels();
  }
  if (btn_hat.is_rising()) {
    seq.hat.trigger();
    if (seq.writing)
      seq.rec_or_rem(Note::hat);
    write_click = false;
    seq.update_pixels();
  }
  if (btn_play.is_rising()) {
    seq.playing = !seq.playing;
    write_click = false;
    seq.update_pixels();
    if (btn_write.state) clear_click = true;
  }
  if (btn_play.is_falling()) {
    if (clear_click) {
      seq.clear();
    }
    clear_click = false;
  }
  if (btn_write.is_rising()) {
    write_click = true;
    seq.update_pixels();
  }
  if (btn_write.is_falling() && write_click) {
    seq.writing = !seq.writing;
    write_click = false;
    clear_click = false;
    seq.update_pixels();
  }
  seq.loop1();
  seq.loop2();
}
