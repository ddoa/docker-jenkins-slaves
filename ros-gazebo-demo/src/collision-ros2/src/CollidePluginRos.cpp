#include "CollidePluginRos.hpp"

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(CollidePluginRos)

/////////////////////////////////////////////////
CollidePluginRos::CollidePluginRos() : SensorPlugin()
{
}

/////////////////////////////////////////////////
CollidePluginRos::~CollidePluginRos()
{
}

/////////////////////////////////////////////////
void CollidePluginRos::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/)
{
    printf("Hello Sensor!\n");
    std::cout << "hello Sensor !" <<std::endl;

  // Get the parent sensor.
  this->parentSensor =
    std::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);

  // Make sure the parent sensor is valid.
  if (!this->parentSensor)
  {
    gzerr << "ContactPlugin requires a ContactSensor.\n";
    return;
  }

  // Connect to the sensor update event.
  this->updateConnection = this->parentSensor->ConnectUpdated(
      std::bind(&CollidePluginRos::OnUpdate, this));

  // Make sure the parent sensor is active.
  this->parentSensor->SetActive(true);
}

/////////////////////////////////////////////////
void CollidePluginRos::OnUpdate()
{
  // Get all the contacts.
  msgs::Contacts contacts;
  contacts = this->parentSensor->Contacts();
  for (unsigned short i = 0; i < contacts.contact_size(); ++i)
  {
//      if(contacts.contact(i).collision2() != "ground_plane::link::collision") {
//          std::cout << "Collision between[" << contacts.contact(i).collision1()
//                    << "] and [" << contacts.contact(i).collision2() << "]\n";

  //        for (unsigned short j = 0; j < contacts.contact(i).position_size(); ++j) {
//              std::cout << j << "  Position:"
                //        << contacts.contact(i).position(j).x() << " "
              //          << contacts.contact(i).position(j).y() << " "
            //            << contacts.contact(i).position(j).z() << "\n";
          //    std::cout << "   Normal:"
        //                << contacts.contact(i).normal(j).x() << " "
      //                  << contacts.contact(i).normal(j).y() << " "
   //                     << contacts.contact(i).normal(j).z() << "\n";
   //           std::cout << "   Depth:" << contacts.contact(i).depth(j) << "\n";
 //         }
//      }
  }
}
