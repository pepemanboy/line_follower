#ifndef LINE_FOLLOWER_LINE_SENSOR_H
#define LINE_FOLLOWER_LINE_SENSOR_H

namespace line_follower {

class LineSensor {

public:
  LineSensor();
  void Init();
  float ReadLine();
};


}  // namespace line_follower

#endif  // LINE_FOLLOWER_LINE_SENSOR_H
