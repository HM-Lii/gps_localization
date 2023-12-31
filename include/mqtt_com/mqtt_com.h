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
  MqttCom() : nh_("~") {
    nh_.param("server_address", server_address_, std::string("tcp://120.77.28.253:1883"));
    nh_.param("client_id", client_id_, std::string("lhm"));

    client_ = std::make_shared<mqtt::async_client>(server_address_, client_id_);
    client_->set_callback(cb_);
  }

  void publish(const std::string& topic,double latitude,double longitude,double speed,double oil) {
    rapidjson::StringBuffer buffer;
    const char* payload = createJson(buffer,latitude,longitude,speed,oil);

    mqtt::message_ptr pubmsg = std::make_shared<mqtt::message>(topic, payload);
    pubmsg->set_qos(0);

    client_->connect(connOpts_)->wait();
    client_->publish(pubmsg);
    printJson(pubmsg);

    client_->disconnect()->wait();
  }

 private:
  class mqtt_callback : public virtual mqtt::callback {
   public:
    void connection_lost(const std::string& cause) override {
      std::cout << "连接断开: " << cause << std::endl;
    }

    void delivery_complete(mqtt::delivery_token_ptr tok) override {
      std::cout << "消息发送完成" << std::endl;
    }
  };

  const char* createJson(rapidjson::StringBuffer& buffer,double latitude,double longitude,double speed,double oil) {
    rapidjson::Document doc;
    doc.SetObject();
    rapidjson::Document::AllocatorType& allocator = doc.GetAllocator();

    std::string id, topic,name, message;
    int code;

    nh_.param("id", id, std::string("1"));
    nh_.param("topic", topic, std::string("gecao"));
    nh_.param("name", name, std::string("农用割草机AH-1"));
    nh_.param("code", code, 0);
    nh_.param("message", message, std::string("在线"));
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
    std::cout << "已发布JSON格式的消息:" << content ;
    std::cout<<"服务器地址："<<server_address_<< std::endl;
  }

  std::shared_ptr<mqtt::async_client> client_;
  mqtt_callback cb_;
  mqtt::connect_options connOpts_;
  ros::NodeHandle nh_;
  std::string server_address_;
  std::string client_id_;
};
#endif
