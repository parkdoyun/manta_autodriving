<?php

error_reporting(E_ALL);
ini_set('displaly_errors', '1');

include 'mysql_connect.php';

if($_SERVER['REQUEST_METHOD'] == 'POST')
{
	define('API_ACCESS_KEY', 'AAAAiN7_cXw:APA91bGVzv_Jz9m2rePvbYsex_XCwCTPYwXikep5ZejLnNvt4K4A837Xp0a-uiHZvWIpv5mI4ZvQoSPnxQMACWtGBIjt7SgkjxSLL9w0P6aAue7JAlc_YFeRumKd3VdR-hektmeUJNXy');

	$fcmUrl = 'https://fcm.googleapis.com/fcm/send';

	// token 가져오기
	$post_id = $_POST['id'];
	$post_token = $_POST['token'];

	// database에서 가져오기
	$sql = "SELECT * FROM log WHERE ID = '".$post_id."' ORDER BY time DESC LIMIT 1";
	$res_sql = mysqli_query($conn, $sql);

	if($res_sql != false && $res_sql->num_rows != null)
	{
		$row = mysqli_fetch_array($res_sql);
		$return_time = $row[1];
		$return_in_out = $row[2];
		$return_station_ID = $row[3];

		$sql2 = "SELECT * FROM kid_info WHERE ID = '".$post_id."'";
		$res_sql2 = mysqli_query($conn, $sql2);
		if($res_sql2 != false && $res_sql2->num_rows != null)
		{
			$row2 = mysqli_fetch_array($res_sql2);
			$return_name = $row2[1];

			$sql3 = "SELECT * FROM station_info WHERE ID = ".$return_station_ID;
			$res_sql3 = mysqli_query($conn, $sql3);
			if($res_sql3 != false && $res_sql3->num_rows != null)
			{
				$row3 = mysqli_fetch_array($res_sql3);
				$return_station_name = $row3[1];

				$return_title = "승차";
				if($return_in_out == false) $return_title = "하차";

				$notification = [
					'title' => $return_title,
					'body' => $return_station_name." ".$return_time
				];
				$extraNotification = [
					'name' => $return_name
				];
				$fcmNotification = [
					'to' => $post_token,
					'notification' => $notification,
					'data' => $extraNotification
				];
				$header = [
					'Authorization: key='.API_ACCESS_KEY,
					'Content-Type: application/json'
				];

				$ch = curl_init();
				curl_setopt($ch, CURLOPT_URL, $fcmUrl);
				curl_setopt($ch, CURLOPT_POST, true);
				curl_setopt($ch, CURLOPT_HTTPHEADER, $header);
				curl_setopt($ch, CURLOPT_SSL_VERIFYPEER, false);
				curl_setopt($ch, CURLOPT_POSTFIELDS, json_encode($fcmNotification));
				$res = curl_exec($ch);
				curl_close($ch);

				echo $res;


			}
			else echo "FAILED";
		}
		else echo "FAILED";	

	}
	else echo "FAILED";

}

mysqli_close($conn);

?>
