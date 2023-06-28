<?php

// error report
error_reporting(E_ALL);
ini_set('display_errors', '1');

// 경로 정류장 전부 조회(원생명과 함께)

include 'mysql_connect.php';

if($_SERVER['REQUEST_METHOD'] == 'GET')
{

	$sql = "SELECT * FROM temp_station ORDER BY idx";
	$res = mysqli_query($conn, $sql);


	# result
	$return_array = array();

	while($row = mysqli_fetch_array($res))
	{
		$tmp_node_idx = $row[1];
		$sql2 = "SELECT ID FROM station_info WHERE node_idx = '".$tmp_node_idx."'";
		$res2 = mysqli_query($conn, $sql2);
		$row2 = mysqli_fetch_array($res2);
		$tmp_s_id = $row2[0];

		array_push($return_array, ["idx"=>$row[0], "station_id"=>$tmp_s_id]);
	}

	header("Content-Type:application/json");
	echo json_encode($return_array, JSON_PRETTY_PRINT);
}
//else echo json_encode({"response":"NOT GET"}, JSON_PRETTY_PRINT);

mysqli_close($conn);

?>
