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
  return (value - a_start) * (b_end - b_start) / (a_end - a_start) + b_start;
}