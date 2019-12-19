var view = document.getElementById('map-view');
initView(view, {vertex: {backColor: 'rgb(58, 58, 60)', foreColor: 'white'}, edge: {color: 'rgb(58, 58, 58)'}});

// Draw Map Button Click Event Handler
function btnDrawMapOnClickHandler(id) {
    console.log("request map data file");

    // request map data(GET)
    $.ajax({
        url: '/map-data',
        type: 'GET',
        success: (data) => {
            console.log('success to get map data file and try to draw map');
            drawMap(view, data, mapClickHandler);
            
            var btn = document.getElementById(id);
            btn.setAttribute('onclick', '');        // unlink onclick handler
            btn.style.backgroundColor = 'rgb(142, 142, 147)';
        },
        error: (jqXHR, textStatus, err) => {
            console.log(`text status: ${textStatus}, error: ${err}`);
        }
    });
}

function btnExploreClickHandler(id) {
    // send data
    $.ajax({
        url: '/cmd/startExplore',
        type: 'POST',
        
        contentType: "text/plain",
        data: '',
        success: (data) => {
            console.log('success to post command');
        },
        error: (jqXHR, textStatus, err) => {
            console.log(`text status: ${textStatus}, error: ${err}`);
        }
    });
}

// this function is vertices & edges click event handler.
function mapClickHandler(evt) {
    // split element information
    var element = evt.target;
    var elementIdInfo = element.id.split('_');

    // when element is edge
    if (elementIdInfo[0] == 'edge') {
    
        /* make send data */
        // get element's information (x, y, w, h)
        var rect = element.getBoundingClientRect();
        var vertexA = document.getElementById('vertex_' + elementIdInfo[1]).getBoundingClientRect();
        var vertexB = document.getElementById('vertex_' + elementIdInfo[2]).getBoundingClientRect();

        // get element's relative position
        var x = evt.pageX - rect.left;
        var y = evt.pageY - rect.top;

        // for sendData
        var ratio = 0.0;
        var sendData = { firstNode: -1, secondNode: -1, ratio: 0.0 };

        if (x > y) {
            // for horizonal line
            if (vertexA.left < vertexB.left) {
                // when vertexA is near to left, firstNode is A
                sendData.firstNode = elementIdInfo[1];
                sendData.secondNode = elementIdInfo[2];
            }
            else {
                // but when vertexB is near to left, firstNode is B
                sendData.firstNode = elementIdInfo[2];
                sendData.secondNode = elementIdInfo[1];
            }

            sendData.ratio = x / rect.width;
        }
        else {
            // for vertical line
            if (vertexA.top < vertexB.top) {
                sendData.firstNode = elementIdInfo[1];
                sendData.secondNode = elementIdInfo[2];
            }
            else {
                sendData.firstNode = elementIdInfo[2];
                sendData.secondNode = elementIdInfo[1];
            }

            // get ratio
            sendData.ratio = y / rect.height;
        }

        console.log(sendData);
        console.log('request go command with json')

        // send data
        $.ajax({
            url: '/cmd/go',
            type: 'POST',
            
            contentType: "application/json",
            data: JSON.stringify(sendData),
            success: (data) => {
                console.log('success to post command');
            },
            error: (jqXHR, textStatus, err) => {
                console.log(`text status: ${textStatus}, error: ${err}`);
            }
        });
    }
    // when element is vertex
    else if (elementIdInfo[1] == 'vertex') {
        // current nothing to do
    }
}

// getStatusTimerHandler
function getStatusTimerHandler(id) {
    var element = document.getElementById(id);

    // request status
    $.ajax({
        url: '/status',
        type: 'GET',

        success: (data) => {
            console.log('success to get status');
            element.innerHTML = data;
        },
        error: (jqXHR, textStatus, err) => {
            console.log(`text status: ${textStatus}, error: ${err}`);
        }
    });
}
