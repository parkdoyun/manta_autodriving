<?php

// error report
error_reporting(E_ALL);
ini_set('display_errors', '1');

// 회원가입 정보 조회

include 'mysql_connect.php';

if($_SERVER['REQUEST_METHOD'] == 'POST')
{
	$post_id = $_POST['id'];
	$post_pwd = $_POST['pwd'];
	$post_name = $_POST['name'];
	$post_kindergarten_id = $_POST['kindergarten_id'];
	$post_tel = $_POST['tel'];
	$post_img = $_POST['img'];
	$post_station_in = $_POST['station_in'];
	$post_station_out = $_POST['station_out'];

	$sql = "INSERT INTO kid_info VALUES('".$post_id."', '".$post_name."', '".$post_pwd."', '".$post_tel."', ".$post_kindergarten_id.", ".$post_station_in.", ".$post_station_out.",'".$post_img."')";
	$res = mysqli_query($conn, $sql);

	$return_val = false;
	if($res)
	{
		$return_val = true;
	}

	header("Content-Type:application/json");
	echo json_encode(array('yn'=>$return_val), JSON_PRETTY_PRINT);
}
//else echo json_encode({"response":"NOT GET"}, JSON_PRETTY_PRINT);

mysqli_close($conn);

?>
