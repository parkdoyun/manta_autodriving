<?php

// error report
error_reporting(E_ALL);
ini_set('display_errors', '1');

// 등하원 위치 수정

include 'mysql_connect.php';

if($_SERVER['REQUEST_METHOD'] == 'POST')
{
	$post_id = $_POST['id'];
	$post_in_out = $_POST['in_out'];
	$post_station_id = $_POST['station_id'];
	$post_time = $_POST['time'];

	// 정보 있는지 조회
	$sql = "SELECT * FROM edit_station WHERE kid_ID = '".$post_id."' AND in_out = ".$post_in_out." AND time = '".$post_time."';";
	$res = mysqli_query($conn, $sql);

	$return_val = false;
	// 정보 없음 -> 등록
	if($res == false || $res->num_rows == null)
	{
		// 마지막 번호 가져오기
		$sql_num = "SELECT ID FROM edit_station ORDER BY ID DESC LIMIT 1;";
		$res_num = mysqli_query($conn, $sql_num);

		$tmp_num = 1;
		if($row_num = mysqli_fetch_array($res_num))
		{
			$tmp_num = $row_num[0] + 1;
		}

		$sql2 = "INSERT INTO edit_station VALUES(".$tmp_num.", '".$post_id."', '".$post_time."', ".$post_in_out.", ".$post_station_id.");";
		$res2 = mysqli_query($conn, $sql2);
		if($res2) $return_val = true;	
	}
	else // 갱신
	{
		$sql2 = "UPDATE edit_station SET station_ID = ".$post_station_id." WHERE kid_ID = '".$post_id."' AND time = '".$post_time."' AND in_out = ".$post_in_out.";";
		$res2 = mysqli_query($conn, $sql2);
		if($res2) $return_val = true;
	}

	header("Content-Type:application/json");
	echo json_encode(array('yn'=>$return_val, 'res'=>$res2), JSON_PRETTY_PRINT);
}
//else echo json_encode({"response":"NOT GET"}, JSON_PRETTY_PRINT);

mysqli_close($conn);

?>
