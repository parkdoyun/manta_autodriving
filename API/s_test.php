<?php

// error report
error_reporting(E_ALL);
ini_set('display_errors', '1');

include 'mysql_connect.php';

if($_SERVER['REQUEST_METHOD'] == 'GET')
{
	$post_station_id = $_GET['station_id'];

	$sql = "SELECT * FROM station_info WHERE ID = ".$post_station_id;
	$res = mysqli_query($conn, $sql);


	# result
	$return_array = array();

	while($row = mysqli_fetch_array($res))
	{
		array_push($return_array, ["NAME"=>$row[1], "lon"=>$row[2], "lat"=>$row[3]]);
	}

	header("Content-Type:application/json");
	echo json_encode($return_array, JSON_PRETTY_PRINT);
}
//else echo json_encode({"response":"NOT GET"}, JSON_PRETTY_PRINT);

mysqli_close($conn);

?>
