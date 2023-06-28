#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include "sql_client.h"

#define	FILE_PATH	"/home/lee/catkin_ws/route_info/route.txt"
#define NODE_FILE_PATH  "/home/lee/catkin_ws/route_info/station_order.txt"

using namespace std;

// python으로부터 GPS text 파일 경로를 받는다
// route_node table flush
// 해당 경로의 파일 읽어서 mysql(route_node)로 전송하자
// 다시 올 때까지 무한 wait(ros::spin())

void listen_callback(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("[LISTEN_CALLBACK] ", msg->data.c_str());
	
	SQLClient sc("parkdoyun", "ehdbs1125"); // mysql connect

	// route_node flush
	sc.DeleteTable("route_node");
	sc.DeleteTable("temp_station");

	ros::Duration duration = ros::Duration(2); // 2 sec delay
	// file 읽기
	// ifstream in(msg->data.c_str());
	ifstream in(FILE_PATH);
	string idx, lon, lat, node;
	string dup_chk = "-1";
	while(in)
	{
		in >> idx;
		in >> lon;
		in >> lat;

		if(idx == dup_chk) break; // duplicate insert check
		dup_chk = idx;

		// DB insert
		try
		{
			sc.insertRoute(idx, lon, lat);
		}
		catch(sql::SQLException &err) // error
		{
			ROS_INFO("[ERROR] ", err.what());
		}
	}

	in.close();

	// 정류장 파일도 빼서 순서대로 넣자
	// 파일 읽어서 넣기
	ifstream fileIn(NODE_FILE_PATH);
	dup_chk = "-1";
	while(fileIn)
	{
	    fileIn >> idx;
	    fileIn >> node;

	    if(idx == dup_chk) break; // duplicate insert check
		dup_chk = idx;

		// DB insert
		try
		{
			sc.insertNode(idx, node);
		}
		catch(sql::SQLException &err) // error
		{
			ROS_INFO("[ERROR] ", err.what());
		}

	}

	fileIn.close();

	ROS_INFO("[SUCCESS] succeeded route_node & temp_station data insert\n");
	
}

void station_callback(const std_msgs::String::ConstPtr& msg)
{
    string msg_str = msg->data.c_str();
    ROS_INFO("[GET_STATION_CALLBACK] ", msg_str);
    stringstream ss(msg_str);
    string kindergarten_num, in_out;

    ss >> kindergarten_num >> in_out;

    SQLClient sc("parkdoyun", "ehdbs1125");
    sc.SelectStation(kindergarten_num, in_out);

    // 완료되었다고 보내기 (python으로 publish)

    ROS_INFO("[SUCCESS] succeeded get route station data\n");
    ros::NodeHandle n1;
    ros::Publisher cpp_finish_pub = n1.advertise<std_msgs::String>("cpp/station_fin", 1000);
    std_msgs::String msg1;
    msg1.data = "FIN";
    for(int i = 0; i < 20; i++)
    {
        cpp_finish_pub.publish(msg1);
        ros::Duration(2).sleep(); // 쉬는 시간 주면서 해야 알아듣는다
    }

}

void log_callback(const std_msgs::String::ConstPtr& msg) // 하차 시 정보 전송
{
    string msg_str = msg->data.c_str();
    stringstream ss(msg_str);
    string ID, in_out, station_ID;
    ss >> ID >> in_out >> station_ID;

    ROS_INFO("[LOG_CALLBACK]");
    SQLClient sc("parkdoyun", "ehdbs1125");

    // log 테이블 insert + temp_station update (yn false로)
    try
    {
        sc.insertLog(ID, in_out, station_ID); // log insert
        sc.updateTempStation(station_ID); // temp_station update
    }
    catch(sql::SQLException &err) // error
    {
        ROS_INFO("[ERROR] ", err.what());
    }
    ROS_INFO("[SUCCESS] LOG_CALLBACK SUCCEED");
}
void bus_callback(const std_msgs::String::ConstPtr& msg) // 버스 현위치 전송
{
  string msg_str = msg->data.c_str();
  stringstream ss(msg_str);
  string lon, lat;
  ss >> lon >> lat;

  ROS_INFO("[BUS_CALLBACK]");
  SQLClient sc("parkdoyun", "ehdbs1125");

 try
  {
      sc.insertBus(lon, lat);
  }
  catch(sql::SQLException &err) // error
  {
      ROS_INFO("[ERROR] ", err.what());
  }
  ROS_INFO("[SUCCESS] BUS_CALLBACK SUCCEED");
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "file_listener");
	ros::NodeHandle n, n1, n_log, n_bus;

	ROS_INFO("LISTEN START");

	// 정류장 정보
	ros::Subscriber station_sub = n1.subscribe("cpp/get_station_info", 1000, station_callback);
	ros::Subscriber sub = n.subscribe("file_listener", 1000, listen_callback);
	ros::Subscriber _log = n_log.subscribe("cpp/bus_log", 1000, log_callback); // 하차 시
	ros::Subscriber _bus = n_bus.subscribe("cpp/bus_position_log", 1000, bus_callback);

	ros::spin(); // loop


	return 0;
}
