package robot_msgs;

public interface odometryData extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "robot_msgs/odometryData";
  static final java.lang.String _DEFINITION = "int32 PosX\nint32 PosY\nint32 PosA";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  int getPosX();
  void setPosX(int value);
  int getPosY();
  void setPosY(int value);
  int getPosA();
  void setPosA(int value);
}
