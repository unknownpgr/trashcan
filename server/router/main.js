module.exports = (app) => {
    app.get('/', (req, res) => {
        res.render('index.html');
    });

    app.get('/index', (req, res) => {
        res.render('index.html');
    });
};