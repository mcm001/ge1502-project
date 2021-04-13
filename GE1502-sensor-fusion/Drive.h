/**
 * GE1501 Group 5.
 */

class Drive {
public:
//  float getDistance();
//  void driveToDistance(float inches, int &max_photoresistor,
//                       volatile bool &startPressed);
  bool turnToAngle(const float reference);
  void drive(float left, float right);
  float normalizeAngle(float theta);
  float getGyro();
};
