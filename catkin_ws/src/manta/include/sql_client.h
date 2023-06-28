#pragma once

#include <stdlib.h>
#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include "mysql_connection.h"
#include <cppconn/driver.h>
#include <cppconn/exception.h>
#include <cppconn/resultset.h>
#include <cppconn/statement.h>
#include <string>

#define	DATABASE_NAME	"manta"
#define URI "j8a409.p.ssafy.io"

using namespace std;

class SQLClient{
	public:
		SQLClient(){}
		SQLClient(string user, string pwd);
		void ConnectToDatabase(string user, string pwd);
		void TruncateTable(string table_name);
		//void PrintTable(string table_name);
		void SelectTest();
		void DeleteTable(string table_name);
		void insertRoute(string idx, string lon, string lat);
		void insertNode(string idx, string node);
		void insertLog(string ID, string in_out, string station_ID);
		void insertBus(string lon, string lat);
		void updateTempStation(string station_ID);
		void SelectStation(string kindergarten_id, string in_out);
		~SQLClient(){delete stmt; delete _con;}
	private:
		sql::Driver* _driver;
		sql::Connection* _con;
		sql::Statement* stmt;
};

