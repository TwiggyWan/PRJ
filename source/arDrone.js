var t = require('./build/Release/nodecpptest');
var ar = require('ar-drone');
var cli = ar.createClient();
var sleep = require('sleep');
fs = require('fs');


/*ar content;
fs.readFile('test.png', function read(err, data) {
    if (err) {
        throw err;
    }
    content = data;
    console.log(content);
    console.log(t.test(content,content));
});*/

var str = "test lol prout";
var res = str.split(' ');
console.log(res[1]);

cli.takeoff();

var pngstreamOld = cli.getPngStream();
pngstreamOld.on('data',console.log);

while(1 == 1)
{
	setInterval(move, 1000);
}

function move(){
	var pngstreamCurrent = cli.getPngStream();
	pngstreamCurrent.on('data',console.log);
  	var ret = t.test(pngstreamOld,pngstreamCurrent);
 	var retArray = ret.spolit(',');
	
	if(retArray[0] > 0)
	{cli.front(1);
	sleep.usleep(100)}
	
	if(retArray[0] < 0)
	{cli.back(1);
	sleep.usleep(100)}

	if(retArray[1] > 0)
	{cli.right(1);
	sleep.usleep(100)}

	if(retArray[1] < 0)
	{cli.left(1);
	sleep.usleep(100)}

	if(retArray[3] > 0)
	{cli.clockwise(1);
	sleep.usleep(100)}

	if(retArray[3] < 0)
	{cli.counterClockwise(1);
	sleep.usleep(100)}
}





