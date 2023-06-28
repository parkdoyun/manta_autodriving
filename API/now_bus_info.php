<?php

// error report
error_reporting(E_ALL);
ini_set('display_errors', '1');

// 현 버스 위치 조회

include 'mysql_connect.php';

if($_SERVER['REQUEST_METHOD'] == 'GET')
{
	$sql = "SELECT * FROM bus ORDER BY bus_time DESC LIMIT 1";
	$res = mysqli_query($conn, $sql);

	if($row = mysqli_fetch_array($res))
	{
		$b_lon = $row[2];
		$b_lat = $row[3];
		$s_num = -1;

		$sql2 = "SELECT node_idx FROM temp_station WHERE yn = false ORDER BY idx ASC LIMIT 1";
		$res2 = mysqli_query($conn, $sql2);

		if($row2 = mysqli_fetch_array($res2))
		{
			// station_ID 구하기
			$sql3 = "SELECT ID FROM station_info WHERE node_idx = '".$row2[0]."'";
			$res3 = mysqli_query($conn, $sql3);

			if($row3 = mysqli_fetch_array($res3))
			{
				$s_num = $row3[0];
			}
		}
	}
	else
	{
		$b_lon = -1;
		$b_lat = -1;
		$s_num = -1;
	}

	header("Content-Type:application/json");
	echo json_encode(array('bus_lon'=>$b_lon, 'bus_lat'=>$b_lat, 'station_ID'=>$s_num), JSON_PRETTY_PRINT);
}
//else echo json_encode({"response":"NOT GET"}, JSON_PRETTY_PRINT);

mysqli_close($conn);

?>
