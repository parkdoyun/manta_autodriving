<?php

// error report
error_reporting(E_ALL);
ini_set('display_errors', '1');

// 등하원 기록 조회

include 'mysql_connect.php';

if($_SERVER['REQUEST_METHOD'] == 'POST')
{
	if(empty($_POST))
	{
		$_POST = (array) json_decode(file_get_contents('php://input'), true);
	}
	$post_id = $_POST['id'];

	$sql = "SELECT * FROM log WHERE ID = '".$post_id."'";
	$res = mysqli_query($conn, $sql);

	$return_array = array();

	while($row = mysqli_fetch_array($res))
	{
		$sql2 = "SELECT * FROM station_info WHERE ID = ".$row[3];
		$res2 = mysqli_query($conn, $sql2);

		if($res2 != false && $res2->num_rows != null)
		{
			$row2 = mysqli_fetch_array($res2);
			array_push($return_array, ['station_name'=>$row2[1], 'lon'=>$row2[2], 'lat'=>$row2[3], 'time'=>$row[1], 'in_out'=>$row[2]]);
		}

	}



	header("Content-Type:application/json");
	echo json_encode($return_array, JSON_PRETTY_PRINT);
}
//else echo json_encode({"response":"NOT GET"}, JSON_PRETTY_PRINT);

mysqli_close($conn);

?>
