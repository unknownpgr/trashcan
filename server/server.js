var express = require('express');
var app = express();

var htmlRouter = require('./router/main')(app);
var ajaxRouter = require('./router/ajax')(app, __dirname);

// for ejs
app.set('views', __dirname + '/views');
app.set('view engine', 'ejs');
app.engine('html', require('ejs').renderFile);

// open server
app.listen(3000, () => {
    console.log("Express server has started on port 3000");
});

// for public files
app.use(express.static('public'));