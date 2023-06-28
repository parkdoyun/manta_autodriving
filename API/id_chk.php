<?php

// error report
error_reporting(E_ALL);
ini_set('display_errors', '1');

include 'mysql_connect.php';

// id duplicate check

if($_SERVER['REQUEST_METHOD'] == 'POST')
{
	$post_id = $_POST['id'];

	$sql = "SELECT * FROM kid_info WHERE ID = '".$post_id."'";
	$res = mysqli_query($conn, $sql);

	$return_val = false;
	if($res == false || $res->num_rows == null) $return_val = true;	

	header("Content-Type:application/json");
	echo json_encode(array('yn'=>$return_val), JSON_PRETTY_PRINT);
}
//else echo json_encode({"response":"NOT GET"}, JSON_PRETTY_PRINT);

mysqli_close($conn);

?>
