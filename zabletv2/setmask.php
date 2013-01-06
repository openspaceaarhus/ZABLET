#!/usr/bin/php
<?php
//$sm = 2*500;
$sm = 0;
while(true) {
	if(false === ($line = fgets(STDIN)))
		break;
	if(preg_match('/^\s*Pin\[(-?\d+)\s+(-?\d+)\s+(-?\d+)\s+(-?\d+)\s+(-?\d+)\s+(-?\d+)\s+"([^"]*)"\s+"([^"]*)"\s+"([^"]*)"]\s*$/', $line, $m)) {
		if($m[5] != 0)
			printf("\tPin[%d %d %d %d %d %d \"%s\" \"%s\" \"%s\"]\n", $m[1] , $m[2], $m[3], $m[4], $m[3]+$sm, $m[6], $m[7], $m[8], $m[9]);
		else
			printf("\tPin[%d %d %d %d %d %d \"%s\" \"%s\" \"%s\"]\n", $m[1] , $m[2], $m[3], $m[4], $m[5], $m[6], $m[7], $m[8], $m[9]);
	} else if(preg_match('/^\s*Pad\[(-?\d+)\s+(-?\d+)\s+(-?\d+)\s+(-?\d+)\s+(-?\d+)\s+(-?\d+)\s+(-?\d+)\s+"([^"]*)"\s+"([^"]*)"\s+"([^"]*)"]\s*$/', $line, $m)) {
		if($m[7] != 0)
			printf("\tPad[%d %d %d %d %d %d %d \"%s\" \"%s\" \"%s\"]\n", $m[1] , $m[2], $m[3], $m[4], $m[5], $m[6], $m[5]+$sm, $m[8], $m[9], $m[10]);
		else
			printf("\tPad[%d %d %d %d %d %d %d \"%s\" \"%s\" \"%s\"]\n", $m[1] , $m[2], $m[3], $m[4], $m[5], $m[6], $m[7], $m[8], $m[9], $m[10]);
	} else
		echo $line;
}
?>
