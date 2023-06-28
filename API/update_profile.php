<?php

// error report
error_reporting(E_ALL);
ini_set('display_errors', '1');

// 프로필 정보 수정

include 'mysql_connect.php';

if($_SERVER['REQUEST_METHOD'] == 'POST')
{
	$post_id = $_POST['id'];
	$post_img = $_POST['img'];
	$post_pwd = $_POST['pwd'];
	$post_kindergarten_id = $_POST['kindergarten_id'];
	$post_tel = $_POST['tel'];
	$post_name = $_POST['name'];
	$post_station_in = $_POST['station_in'];
	$post_station_out = $_POST['station_out'];

	$return_val = false;	
	$sql2 = "UPDATE kid_info SET img = '".$post_img."', pwd = '".$post_pwd."', team = ".$post_kindergarten_id.", tel = '".$post_tel."', name='".$post_name."', station_in=".$post_station_in.", station_out=".$post_station_out." WHERE ID = '".$post_id."';";
	$res2 = mysqli_query($conn, $sql2);
	if($res2) $return_val = true;
	

	header("Content-Type:application/json");
	echo json_encode(array('yn'=>$return_val, 'res'=>$res2), JSON_PRETTY_PRINT);
}
//else echo json_encode({"response":"NOT GET"}, JSON_PRETTY_PRINT);

mysqli_close($conn);

?>
