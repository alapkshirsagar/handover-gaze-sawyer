/*
 * Publish sensor readings received from range sensor and toggle sensor
 *
 */

#include <ros.h>
#include <std_msgs/Int32.h>

ros::NodeHandle  nh;

std_msgs::Int32 range_sensor_reading;
std_msgs::Int32 toggle_sensor_reading;

ros::Publisher range_sensor("range_sensor", &range_sensor_reading);
ros::Publisher toggle_sensor("toggle_sensor", &toggle_sensor_reading);
int toggleSensorPin = 5;

void setup()
{
  pinMode(toggleSensorPin, INPUT);
  nh.initNode();
  nh.advertise(range_sensor);
  nh.advertise(toggle_sensor);
}

void loop()
{
  range_sensor_reading.data = analogRead(A1);
  toggle_sensor_reading.data = digitalRead(toggleSensorPin);
  
  range_sensor.publish( &range_sensor_reading );
  toggle_sensor.publish( &toggle_sensor_reading );

  nh.spinOnce();
  delay(10);
}
