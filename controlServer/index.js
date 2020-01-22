var express = require('express')
var app = express()

app.use(express.static(__dirname + '/static'))

app.get('/', (req, res) => {
    res.sendFile(__dirname + '/static/index.html')
})

app.listen(80, () => {
    console.log('Control server started')
})