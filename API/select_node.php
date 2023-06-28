<?php

// error report
error_reporting(E_ALL);
ini_set('display_errors', '1');

// 경로 노드 GPS 전부 조회

include 'mysql_connect.php';

if($_SERVER['REQUEST_METHOD'] == 'GET')
{
	$sql = "SELECT * FROM route_node ORDER BY ID";
	$res = mysqli_query($conn, $sql);


	# result
	$return_array = array();

	while($row = mysqli_fetch_array($res))
	{
		array_push($return_array, ["lat"=>$row[1], "lon"=>$row[2]]);
	}

	header("Content-Type:application/json");
	echo json_encode($return_array, JSON_PRETTY_PRINT);
}
//else echo json_encode({"response":"NOT GET"}, JSON_PRETTY_PRINT);

mysqli_close($conn);

?>
