<?php

// error report
error_reporting(E_ALL);
ini_set('display_errors', '1');

include 'mysql_connect.php';

// login

if($_SERVER['REQUEST_METHOD'] == 'POST')
{
	$post_id = $_POST['id'];
	$post_pwd = $_POST['pwd'];

	$sql = "SELECT * FROM kid_info WHERE ID = '".$post_id."' AND pwd = '".$post_pwd."'";
	$res = mysqli_query($conn, $sql);

	$return_val = true;

	// select 없거나 실패한다면
	if($res == false || $res->num_rows == null) $return_val = false;

	header("Content-Type:application/json");
	echo json_encode(array('yn'=>$return_val, 'res'=>$res), JSON_PRETTY_PRINT);
}
//else echo json_encode({"response":"NOT GET"}, JSON_PRETTY_PRINT);

mysqli_close($conn);

?>
