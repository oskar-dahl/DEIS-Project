package robot_msgs;

public interface ctrlData extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "robot_msgs/ctrlData";
  static final java.lang.String _DEFINITION = "int32 SpeedLeft\nint32 SpeedRight\nint32 State";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  int getSpeedLeft();
  void setSpeedLeft(int value);
  int getSpeedRight();
  void setSpeedRight(int value);
  int getState();
  void setState(int value);
}
