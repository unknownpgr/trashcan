var vertexBackgroundColor = 'black';
var vertexForegroundColor = 'white';
var edgeColor = 'black';

function initView(view, attr) {
    view.style['position'] = 'relative';

    vertexBackgroundColor = attr.vertex.backColor;
    vertexForegroundColor = attr.vertex.foreColor;

    edgeColor = attr.edge.color;
}

// draw map
function drawMap(view, mapData, clickHandler) {
    var width = view.offsetWidth;
    var height = view.offsetHeight;
    var json = JSON.parse(mapData);

    // get max and min position to scale vertexes.
    var maxPosX = -Infinity;
    var minPosX = Infinity;
    var maxPosY = -Infinity;
    var minPosY = Infinity;
    json.forEach((value, index, array) => {
        if (value.position.x * 1 > maxPosX)
            maxPosX = value.position.x;
        
        if (value.position.x * 1 < minPosX)
            minPosX = value.position.x;

        if (value.position.y * 1 > maxPosY)
            maxPosY = value.position.y;

        if (value.position.y * 1 < minPosY)
            minPosY = value.position.y;

        console.log(`index: ${index}, maxPosX: ${maxPosX}, minPosX: ${minPosX}, maxPosY: ${maxPosY}, minPosY: ${minPosY}`);
    });

    // draw vertices in map-view
    json.forEach((value, index, array) => {
        var vertexPosX = (value.position.x - minPosX) / (maxPosX - minPosX) * width;
        var vertexPosY = (value.position.y - minPosY) / (maxPosY - minPosY) * height;
        createVertex(view, index, {w: 24, h: 24}, {x: vertexPosX, y: vertexPosY}, clickHandler);
    });

    // draw edges in map-view
    json.forEach((value, index, array) => {
        var vertexA = value.number * 1;
        var vertexB;

        value.connected.forEach((value, index, array) => {
            vertexB = value * 1;
            console.log([vertexA, vertexB]);
            createEdge(view, 10, [vertexA, vertexB], clickHandler);
        });
    });
}

// create vertex
function createVertex(view, num, size, pos, click) {
    var vertex = document.createElement('div');
    var id = 'vertex_' + num;
    
    vertex.id = id

    // size
    vertex.style.width = size.w + 'px';
    vertex.style.height = size.h + 'px';

    // position
    vertex.style.position = 'absolute';
    vertex.style.zIndex = 1;
    vertex.style.left = (pos.x - size.w / 2) + 'px';
    vertex.style.top = (pos.y - size.h / 2) + 'px';

    // color
    vertex.style.backgroundColor = vertexBackgroundColor;
    vertex.style.color = vertexForegroundColor;

    // border
    vertex.style.borderRadius = '50%';

    // text
    vertex.innerHTML = num;
    vertex.style.textAlign = 'center';

    // event handler
    vertex.onclick = click;

    view.appendChild(vertex);
}

// create edge
function createEdge(view, thick, vertices, click) {
    // preventing create existing edge again.
    var vertexMin = Math.min.apply(Math, vertices);
    var vertexMax = Math.max.apply(Math, vertices);
    var id = `edge_${vertexMin}_${vertexMax}`;
    var existingElement = document.getElementById(id);
    if (typeof(existingElement) != 'undefined' && existingElement != null) {
        console.log(id + ' element already exists.');
        return;
    }

    // create edge element
    var edge = document.createElement('div');
    edge.id = id;

    // get vertex information
    var vertexAElement = document.getElementById('vertex_' + vertexMin);
    var vertexBElement = document.getElementById('vertex_' + vertexMax);
    var vertexA = { x: parseFloat(vertexAElement.style.left),
                    y: parseFloat(vertexAElement.style.top),
                    w: parseFloat(vertexAElement.style.width),
                    h: parseFloat(vertexAElement.style.height)};
    var vertexB = { x: parseFloat(vertexBElement.style.left),
                    y: parseFloat(vertexBElement.style.top),
                    w: parseFloat(vertexBElement.style.width),
                    h: parseFloat(vertexBElement.style.height)};

    // size
    var lengthW = Math.abs(vertexA.x - vertexB.x);
    var lengthH = Math.abs(vertexA.y - vertexB.y);
    var lengthComp = lengthW > lengthH;

    if (lengthComp) {
        // horizontal line
        edge.style.width = lengthW + 'px';
        edge.style.height = thick + 'px';
    }
    else {
        // vertical line
        edge.style.width = thick + 'px';
        edge.style.height = lengthH + 'px';
    }

    // position
    var posX = (vertexA.x + vertexB.x - lengthW) / 2;
    var posY = (vertexA.y + vertexB.y - lengthH) / 2;

    if (lengthComp) {
        // horizonal line
        posY += (vertexA.w - thick) / 2;
        posX += vertexA.h / 2;
    }
    else {
        // vertical line
        posX += (vertexA.w - thick) / 2;
        posY += vertexA.h / 2;
    }

    edge.style.position = 'absolute';
    edge.style.left = posX + 'px';
    edge.style.top = posY + 'px';

    // color
    edge.style.backgroundColor = edgeColor;

    // event handler
    edge.onclick = click;

    view.appendChild(edge);
}
