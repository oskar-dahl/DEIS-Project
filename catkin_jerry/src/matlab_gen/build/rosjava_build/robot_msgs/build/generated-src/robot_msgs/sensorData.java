package robot_msgs;

public interface sensorData extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "robot_msgs/sensorData";
  static final java.lang.String _DEFINITION = "int32 IRLeft\nint32 IRCenter\nint32 IRRight\nint32 EncLeft\nint32 EncRight";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  int getIRLeft();
  void setIRLeft(int value);
  int getIRCenter();
  void setIRCenter(int value);
  int getIRRight();
  void setIRRight(int value);
  int getEncLeft();
  void setEncLeft(int value);
  int getEncRight();
  void setEncRight(int value);
}
