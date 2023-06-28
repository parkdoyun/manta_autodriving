<?php

// error report
error_reporting(E_ALL);
ini_set('display_errors', '1');

// 프로필 조회
include 'mysql_connect.php';

if($_SERVER['REQUEST_METHOD'] == 'POST')
{
	$post_id = $_POST['id'];

	$sql = "SELECT * FROM kid_info WHERE ID = '".$post_id."'";
	$res = mysqli_query($conn, $sql);


	# result
	$return_pwd = '';
	$return_name = '';
	$return_kindergarten_id = -1;
	$return_tel = '';
	$return_img = '';
	$return_station_in = -1;
	$return_station_out = -1;


	if($res != false && $res->num_rows != null)
	{
		$row = mysqli_fetch_array($res);
		$return_pwd = $row[2];
		$return_name = $row[1];
		$return_kindergarten_id = $row[4];
		$return_tel = $row[3];
		$return_img = $row[7];
		$return_station_in = $row[5];
		$return_station_out = $row[6];
	}

	header("Content-Type:application/json");
	echo json_encode(array('pwd'=>$return_pwd, 'name'=>$return_name, 'kindergarten_id'=>$return_kindergarten_id, 'tel'=>$return_tel, 'img'=>$return_img, 'station_in'=>$return_station_in, 'station_out'=>$return_station_out), JSON_PRETTY_PRINT);
}
//else echo json_encode({"response":"NOT GET"}, JSON_PRETTY_PRINT);

mysqli_close($conn);

?>
