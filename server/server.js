var express = require('express');
var com = require('./com');
var app = express();

// router (response for views(html))
var router = require('./router/main')(app);
// router_ajax (response for ajax)
var router_ajax = require('./router/ajax')(app);

app.set('views', __dirname + '/views');
app.set('view engine', 'ejs');
app.engine('html', require('ejs').renderFile);

app.listen(3000, () => {
    console.log("Express server has started on port 3000");
});

// for public files
app.use(express.static('public'));

com.init(__dirname + '/data', __dirname + '/data');
var attr = { interval: 500, maxTryCount: 10 };

com.receiveJSONData('map.json', attr, (err, json) => {
    if (err) {
        console.log("can't load file");
    }
    else {
        com.sendJSONData('map2.json', json, err => {
            if (err) console.log('error has been occured!');
        })
    }
});