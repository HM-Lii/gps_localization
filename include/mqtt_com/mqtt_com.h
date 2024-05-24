#ifndef _MQTT_COM_H
#define _MQTT_COM_H
#include <mqtt/async_client.h>
#include <rapidjson/document.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>
#include <ros/ros.h>

#include <cstring>
#include <iostream>
#include <sstream>

class MqttCom {
 public:
  MqttCom(ros::NodeHandle& nh_)  {
    nh_.param("server_address", server_address_,
              std::string("tcp://120.77.28.253:1883"));
    nh_.param("client_id", client_id_, std::string("lhm"));
    nh_.param("id", id, std::string("1"));
    nh_.param("topic", topic, std::string("gecao/upload"));
    nh_.param("name", name, std::string("农用割草机AH-1"));
    nh_.param("code", code, 0);
    nh_.param("message", message, std::string("在线"));
    go_publisher = nh_.advertise<std_msgs::Int8>("/cmd_go", 10);
    work_publisher = nh_.advertise<std_msgs::Int8>("/cmd_work", 10);
    cb_.setMqttCom(this);
    client_ = std::make_shared<mqtt::async_client>(server_address_, client_id_);
    client_->set_callback(cb_);
    client_->connect(connOpts_)->wait();
    client_->subscribe("gecao/distribute", 1);  //  订阅一个或多个主题
  }

  void publish(const std::string& topic, double latitude, double longitude,
               double speed, double oil) {
    rapidjson::StringBuffer buffer;
    const char* payload = createJson(buffer, latitude, longitude, speed, oil);

    mqtt::message_ptr pubmsg = std::make_shared<mqtt::message>(topic, payload);
    pubmsg->set_qos(1);

    client_->publish(pubmsg);
    printJson(pubmsg);
  }
  
  void processJsonMessage(const rapidjson::Document& doc) {
    if (doc.HasMember("machineId") && doc.HasMember("routeFile") &&
        doc.HasMember("taskType") && doc.HasMember("taskId") &&
        doc.HasMember("topic")) {
      machine_id = doc["machineId"].GetInt();
      task_type = doc["taskType"].GetInt();
      ROS_INFO("taskType: %d",task_type);
      task_id = doc["taskId"].GetInt();
      if (task_type==01) { 
        std_msgs::Int8 msg;
        msg.data=1;     
        go_publisher.publish(msg);
        std::cout << "mower start going..." << std::endl;
      }
      else if (task_type==11) {
        std_msgs::Int8 msg;
        msg.data=1;     
        work_publisher.publish(msg);
        std::cout << "mower start working..." << std::endl;
      }
      else{
        std::cout << "task type error" << std::endl;
      }
    } 
    else {
      std::cerr << "Received  JSON  does  not  contain  the  expected  data."
                << std::endl;
    }
  }
  class mqtt_callback : public virtual mqtt::callback {
   public:
    void setMqttCom(MqttCom* mqttCom) { mqttCom_ = mqttCom; }
    void connection_lost(const std::string& cause) override {
      std::cout << "连接断开: " << cause << std::endl;
    }

    void delivery_complete(mqtt::delivery_token_ptr tok) override {
      std::cout << "消息发送完成" << std::endl;
    }
    void message_arrived(mqtt::const_message_ptr msg) override {
      // std::cout << "Message  arrived:  " << msg->to_string() << std::endl;
      rapidjson::Document document;
      document.Parse(msg->to_string().c_str());
      //  处理JSON数据
      if (!document.HasParseError()) {
        mqttCom_->processJsonMessage(document);
      } else {
        std::cerr << "Failed  to  parse  JSON  message." << std::endl;
      }
    }
    MqttCom* mqttCom_;
  };

 private:
  const char* createJson(rapidjson::StringBuffer& buffer, double latitude,
                         double longitude, double speed, double oil) {
    rapidjson::Document doc;
    doc.SetObject();
    rapidjson::Document::AllocatorType& allocator = doc.GetAllocator();




    rapidjson::Value id_value(id.c_str(), allocator);
    rapidjson::Value topic_value(topic.c_str(), allocator);
    rapidjson::Value name_value(name.c_str(), allocator);
    rapidjson::Value message_value(message.c_str(), allocator);
    doc.AddMember("id", id_value, allocator);
    doc.AddMember("topic", topic_value, allocator);
    doc.AddMember("name", name_value, allocator);
    doc.AddMember("latitude", latitude, allocator);
    doc.AddMember("longitude", longitude, allocator);

    rapidjson::Value status(rapidjson::kObjectType);
    status.AddMember("code", code, allocator);
    status.AddMember("message", message_value, allocator);
    status.AddMember("speed", speed, allocator);
    status.AddMember("oil", oil, allocator);

    doc.AddMember("status", status, allocator);

    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    doc.Accept(writer);

    return buffer.GetString();
  }

  void printJson(mqtt::message_ptr pubmsg) {
    const void* data = pubmsg->get_payload().data();
    std::size_t size = pubmsg->get_payload().size();
    std::string content(static_cast<const char*>(data), size);
    std::cout << "已发布JSON格式的消息:" << content;
    std::cout << "服务器地址：" << server_address_ << std::endl;
  }

  std::shared_ptr<mqtt::async_client> client_;
  mqtt_callback cb_;
  mqtt::connect_options connOpts_;
  std::string server_address_;
  std::string client_id_;
  std::string id, topic, name, message;
  int code;
  ros::Publisher go_publisher,work_publisher;
  int machine_id;
  int task_type;
  int task_id;
};
#endif
