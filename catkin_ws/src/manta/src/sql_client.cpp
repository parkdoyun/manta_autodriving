#include "sql_client.h"

SQLClient::SQLClient(string user, string pwd)
{
	ConnectToDatabase(user, pwd);
}

	                
void SQLClient::ConnectToDatabase(string user, string pwd)
{
	_driver = get_driver_instance();
	_con = _driver->connect(URI, user, pwd); // mysql connect
	if(_con->isValid())
	{
		ROS_INFO_STREAM("[SUCCESS] Connected to " << DATABASE_NAME);
		_con->setSchema(DATABASE_NAME);
	}
	else
	{
		ROS_INFO_STREAM("[FAILED] Connected to " << DATABASE_NAME);
	}
	stmt = _con->createStatement();
}

void SQLClient::TruncateTable(string table_name)
{
	stmt->execute("TRUNCATE " + table_name);
}

void SQLClient::SelectTest()
{
	sql::ResultSet* res;
	res = stmt->executeQuery("SELECT ID, name, team FROM kid_info;");
	if(res->rowsCount() == 0) cout << "NO INFORMATION\n"; // no select
	else
	{
		res->next();
		cout << res->getString("ID") << '\n';
		cout << res->getString("name") << '\n';
		cout << res->getInt("team") << '\n';
	}
}

void SQLClient::DeleteTable(string table_name)
{
	stmt->execute("DELETE FROM " + table_name);
}

void SQLClient::insertRoute(string idx, string lon, string lat)
{
	stmt->execute("INSERT INTO route_node VALUES(" + idx + ", " + lat + ", " + lon + ")");
}
void SQLClient::insertNode(string idx, string node)
{
    stmt->execute("INSERT INTO temp_station VALUES(" + idx + ", '" + node + "', false)");
}
void SQLClient::insertBus(string lon, string lat)
{
  stmt->execute("INSERT INTO bus (bus_time, bus_lon, bus_lat) VALUES(DATE_ADD(now(), INTERVAL 9 HOUR), " + lat + ", " + lon + ")");
}
void SQLClient::insertLog(string ID, string in_out, string station_ID) // 하차 시 정보 전송
{
    // 정류장 id 찾기
    sql::ResultSet* res;
	sql::ResultSet* res1;
    // node_idx 찾기
    res1 = stmt->executeQuery("SELECT node_idx FROM station_info WHERE ID = " + station_ID);
    if(res1->rowsCount() == 0)
	{
	    cout << "NO NODE IDX\n"; // no select
	    return;
	}
	res1->next();
	string node_idx = res1->getString("node_idx");

    // 만약 해당 날짜에 in_out 갖고 아이디 가지는 것 있다면 그냥 return
    res = stmt->executeQuery("select * from log where ID = '" + ID + "' and DATE_FORMAT(time, '%Y-%m-%d') = DATE_FORMAT(DATE_ADD(now(), INTERVAL 9 HOUR), '%Y-%m-%d') and in_out = " + in_out);
    if(res->rowsCount() != 0) // 만약 있다면
    {
      ROS_INFO("[LOG_CALLBACK] LOG IS EXISTED");
      return;
    }
    // 아니면 집어넣기
	res = stmt->executeQuery("SELECT ID FROM station_info WHERE node_idx = '" + node_idx + "'");
	ROS_INFO("[LOG_CALLBACK] ", node_idx);
	if(res->rowsCount() == 0)
	{
	    cout << "NO NODE IDX\n"; // no select
	    return;
	}
	else
	{
		res->next();
		string s_id = res->getString("ID");
		stmt->execute("INSERT INTO log VALUES('" + ID + "', DATE_ADD(now(), INTERVAL 9 HOUR), " + in_out + ", " + s_id + ")");
	}
}
void SQLClient::updateTempStation(string station_ID) // 하차 시 정류장 정보 변경
{
    sql::ResultSet* res;
    // node_idx 찾기
    res = stmt->executeQuery("SELECT node_idx FROM station_info WHERE ID = " + station_ID);
    if(res->rowsCount() == 0)
	{
	    cout << "NO NODE IDX\n"; // no select
	    return;
	}
	res->next();
	string node_idx = res->getString("node_idx");
    stmt->execute("UPDATE temp_station SET yn = false WHERE node_idx = '" + node_idx + "'");
}
//void SQLClient::PrintTable(string table_name)
void SQLClient::SelectStation(string kindergarten_id, string in_out) // 유치원 id와 flag로 오늘자 들러야 하는 정류장 리스트 찾기
{
    // 오늘이고 등원인 애들 수정해야 한다면 (유치원 같고) stationTF = 1, station_new_ID로 정류장 기록하기
    // stationTF = 0이면 station_origin_ID로 기록하기
    sql::ResultSet* res;
    sql::ResultSet* res_node_idx;
    res = stmt->executeQuery("select DISTINCT a.ID, a.name, a.station_in as station_origin_ID, b.station_ID as station_new_ID, IF(a.ID IN (select kid_ID from edit_station where date(time) = curdate() and in_out = " + in_out + "), true, false) as stationTF " +
    "from (select * from kid_info where team = " + kindergarten_id + ") as a LEFT JOIN (select * from edit_station where date(time) = curdate() and in_out = " + in_out + ") as b ON a.ID = b.kid_ID");

    res_node_idx = stmt->executeQuery("select node_idx, name from station_info where ID = " + kindergarten_id);


    // 정류장의 노드 번호 + 정류장 명 넣는 파일 "/home/parkdoyun/route_info/route_station.txt"
    ofstream of("/home/lee/catkin_ws/route_info/route_station.txt");

    // 정류장과 원생 ID + 원생 이름 같이 쓰기 "/home/parkdoyun/route_info/route_student.txt"
    ofstream of_id("/home/lee/catkin_ws/route_info/route_student.txt");

    // 파일에 기록
    if(of.is_open() && of_id.is_open())
    {
        // 정류장 노드 번호 쓰기
	res_node_idx->next();
        of << res_node_idx->getString("node_idx") << " " << res_node_idx->getString("name") << "\n";
        while(res->next())
        {
            if(res->getString("stationTF") == "1") // station_new_ID 사용
            {
                res_node_idx = stmt->executeQuery("select node_idx, name from station_info where ID = " + res->getString("station_new_ID"));
            }
            else // station_origin_ID 사용
            {
                res_node_idx = stmt->executeQuery("select node_idx, name from station_info where ID = " + res->getString("station_origin_ID"));
            }
	        res_node_idx->next();
            of << res_node_idx->getString("node_idx") << " " << res_node_idx->getString("name") << "\n";
            of_id << res->getString("ID") << " " << res_node_idx->getString("node_idx") << " " << res->getString("name") << "\n";
        }
    }

    of.close();
    of_id.close();
}
