var t = require('./build/Release/nodecpptest');
var ar = require('ar-drone');
var cli = ar.createClient();
fs = require('fs');


var content;
fs.readFile('test.png', function read(err, data) {
    if (err) {
        throw err;
    }
    content = data;
    console.log(content);
    t.test(content,content);
});



/*

var pngstream = cli.getPngStream();
pngStream.on('data',console.log);*/
