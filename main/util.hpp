// Typesafe sign function
template <typename T> int signum(T val) {
    if (val >= 0){
      return 1;
    } else {
      return -1;
    }
}

template <typename T,typename M> T deadzone(T val, M deadzone) {
    if (val > deadzone || val < -deadzone) return val;
    return (T)0;
}

template <typename T> T map_range(T value, T a_start, T a_end, T b_start, T b_end) {
  // Source - https://stackoverflow.com/a/5732390
  // Posted by Alok Singhal
  // Retrieved 2026-02-27, License - CC BY-SA 3.0

  double slope = 1.0 * (b_end - b_start) / (a_end - a_start);
  return (T) (b_start + slope * (value - a_start));
}