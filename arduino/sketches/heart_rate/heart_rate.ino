#include <QueueList.h>

#define DELAY 40
#define MAX_PULSE_VALUE 600
#define MIN_PULSE_VALUE 250

int sign(int value) {
  return value < 0? -1 : value > 0;
}

int derivative(size_t value, size_t &ovalue) {
  int diff = value - ovalue;
  ovalue = value;
  if(ovalue == -1 || value == -1)
    return 1;
  return diff;
}

bool sanity_check(int value, int low, int high) {
  return value <= high && value >= low;
}


// this is using moving average (well, it's a fake average)
template <typename T>
class MovingAverageValue {
  bool no_avg_ = { true };
  const size_t cnt_;
  T avg_;
  void fake_avg_(const T &value) {
    if (no_avg_ || cnt_ == 1) {
      avg_ = value;
      no_avg_ = false;
    } else {
      avg_ += (value - avg_) / (T)(cnt_-1);
    } 
  }
  public:
  MovingAverageValue(const T& avg_default, size_t avg_count = 3):avg_(avg_default), cnt_(avg_count == 0? 1 : avg_count) {
  }
  void push(const T &value) {
    fake_avg_(value);
  }
  void set(const T &value) {
    this->avg_ = value;
  }
  T value() const {
    return this->avg_;
  }
  size_t count() const {
    return this->cnt_;
  }
};



class FlickerDetector {
  const static int MAX_SIGNS = 32000;
  const size_t vote_thr_, thr_, sz_;
  
  bool is_flickering_ = { false };
  size_t cnt_ = { 0 }, in_thr_ = {0};
  int sign_total_ = { 0 };
  
  MovingAverageValue<int> avg_;
  void make_assumption_(int value) {
    if (cnt_ == 1 || in_thr_ == 0) {
      avg_.set(value);
      in_thr_ = 1;
      return;
    }
    int diff = value-avg_.value();
    Serial.print("DIFF: ");
    Serial.println(diff);
    if ( abs(diff) < thr_) {
      Serial.println("IN_THRESHOLD++");
      if (in_thr_ < sz_)
        ++in_thr_;
      avg_.push(value);
      if (abs(diff) > thr_ / 2)
        sign_total_ = min(sign_total_ + sign(diff), MAX_SIGNS);
      else if (sign_total_)
        sign_total_ += -sign(sign_total_);
    }
    else if (in_thr_ > 0) {
      --in_thr_;
      Serial.println("IN_THRESHOLD -- ");
    }
    Serial.print("IN THRESHOLD: ");
    Serial.print(in_thr_);
    Serial.print(" NEEDED: ");
    Serial.println(vote_thr_);
    Serial.println("SIGNS: ");
    Serial.print(abs(sign_total_));
    Serial.print(" NEEDED TO RESET: ");
    Serial.println((sz_-vote_thr_ +1));
    // if function is constantly growing or shrinking, for example.
    if (abs(sign_total_) > (sz_ - vote_thr_ + 1)) {
      is_flickering_ = false;
      if (in_thr_ > 0)
        --in_thr_;
    } 
    // if votes are lowered below threshold and we still think values are flickering - reset state.
    else if ( in_thr_ < vote_thr_ && is_flickering_) {
      this->reset_(value);
    } 
    // if votes are enough to tell that this is flickering
    else if (in_thr_ >= vote_thr_) {
      is_flickering_ = true;
    }
    
  }
  void reset_(int value) {
    cnt_ = 1;
    in_thr_ = 1;
    sign_total_ = 0;
    is_flickering_ = false;
    avg_.set(value);
  }
  
  public:
  FlickerDetector(size_t flicker_threshold, size_t window_size, size_t min_votes): 
    vote_thr_(min_votes), thr_(flicker_threshold), sz_(window_size), avg_(0, window_size)
  {
    
  }

  void push(int value) {
    ++cnt_;
    make_assumption_(value);
  }
  
  bool is_flickering() const {
    return is_flickering_;
  }
  
};




class Pulse {
  static const size_t MAX_BPM = 200, MIN_BPM = 0;
  
  const size_t delay_, rate_, pin_;
  size_t led_value_ = { 0 };
  MovingAverageValue<int> bpm_;
  FlickerDetector flicker_detector_;
  size_t minv, maxv;
  size_t periods_since_last_peak_ = { 0 }, old_bpm_value_ = { 0 };
  bool first_peak_detected_ = { false };
  int derivative_ = { 0 };
  
  public:
  Pulse(size_t read_delay, size_t sensor_pin, size_t min_sensor_value, size_t max_sensor_value, size_t max_error, size_t bpm_inertia = 10) : 
    delay_(read_delay), rate_(1000 / read_delay), pin_(sensor_pin), bpm_(0, bpm_inertia), 
    minv(min_sensor_value), maxv(max_sensor_value),
    flicker_detector_(max_error, 2000 / read_delay, 1750 / read_delay)
  {
    
  }
  void step() {
    size_t sensor_value = analogRead(pin_);
    if (!sanity_check(sensor_value, minv, maxv)) {
      Serial.println("BPM INSANE VALUE...");
      return;
    }
      
    flicker_detector_.push(sensor_value);
    if (flicker_detector_.is_flickering()) {
      if(old_bpm_value_ == 0)
        old_bpm_value_ = this->bpm_.value();
      this->bpm_.set(0);
      first_peak_detected_ = false;
      Serial.println("BPM FLICKERING...");
      return;
    } else if (old_bpm_value_ != 0) {
      this->bpm_.set(old_bpm_value_);
      old_bpm_value_ = 0;
    }
    Serial.println("BPM LISTENING...");
    int new_derivative = derivative(sensor_value, this->led_value_);
    if (sign(derivative_) >= 0 && sign(new_derivative) < 0) {
      Serial.println("BPM PEAK!");
      if (this->first_peak_detected_) {
        Serial.println("FIRST_PEAK");
        int new_bpm_value = 60 * this->rate_ / this->periods_since_last_peak_;
        if (sanity_check(new_bpm_value, Pulse::MIN_BPM, Pulse::MAX_BPM))
          this->bpm_.push(new_bpm_value);
        this->periods_since_last_peak_ = 0;
      }
      else {
        first_peak_detected_ = true;
        periods_since_last_peak_ = 0;
      }
    }
    
    derivative_ = new_derivative;
    ++periods_since_last_peak_;
  }
  
  size_t bpm() const {
    return bpm_.value();
  }
  size_t data() const {
    return led_value_;
  }

  bool is_flickering() const {
    return flicker_detector_.is_flickering();
  }
  
};

//Pulse pulse_sensor(DELAY, A0, MIN_PULSE_VALUE, MAX_PULSE_VALUE);
MovingAverageValue<int> avg_data(0, 4);
Pulse pulse(DELAY, A0, MIN_PULSE_VALUE, MAX_PULSE_VALUE, 35, 30);
void setup() {
  // put your setup code here, to run once:
  Serial.begin(38400);
  while(!Serial);
}

void loop() {
  // put your main code here, to run repeatedly:
  //pulse_sensor.step();
  size_t data = analogRead(A0);
  avg_data.push(data);
  pulse.step();
  
  
  // print in format for external view
  Serial.println("$");
  Serial.println(pulse.bpm());
  Serial.println(" ");
  Serial.println(avg_data.value());
  Serial.println(" ");
  Serial.println(pulse.is_flickering()*200);
  Serial.println(";");
  delay(DELAY);
}


