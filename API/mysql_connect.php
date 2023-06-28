<?php
	$host = "j8a409.p.ssafy.io";
	$user = "parkdoyun";
	$pwd = "ehdbs1125";
	$dbName = "manta";

	$conn = mysqli_connect($host, $user, $pwd, $dbName);

	mysqli_query($conn, 'set session character_set_connection=utf8;');
	mysqli_query($conn, 'set session character_set_results=utf8;');
	mysqli_query($conn, 'set session character_set_client=utf8;');

	/*$res_data = mysqli_fetch_assoc($res);*/
	//echo "MYSQL CONNECT SUCCESS\n";
?>
