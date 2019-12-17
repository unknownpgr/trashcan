module.exports = (app) => {
    app.get('/', (req, res) => {
        res.render('map.html');
    });

    app.get('/jquerytest', (req, res) => {
        res.render('jquerytest.html');
    });
};