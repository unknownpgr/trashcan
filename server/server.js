var express = require('express');
var app = express();
var router = require('./router/main')(app);

app.set('views', __dirname + '/views');
app.set('view engine', 'ejs');
app.engine('html', require('ejs').renderFile);

app.listen(3000, () => {
    console.log("Express server has started on port 3000");
});

app.use(express.static('public'));