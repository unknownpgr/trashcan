var vertexBackgroundColor = 'black';
var vertexForegroundColor = 'white';
var edgeColor = 'black';

function initView(view, attr) {
    view.style['position'] = 'relative';

    vertexBackgroundColor = attr.vertex.backColor;
    vertexForegroundColor = attr.vertex.foreColor;

    attr.edge.color = edgeColor;
}

// draw map
function drawMap(view, mapData) {
    var width = view.offsetWidth;
    var height = view.offsetHeight;
    var json = JSON.parse(mapData);

    // get max and min position to scale vertexes.
    var maxPosX = -Infinity;
    var minPosX = Infinity;
    var maxPosY = -Infinity;
    var minPosY = Infinity;
    json.forEach((value, index, array) => {
        console.log(`index: ${index}, maxPosX: ${maxPosX}, minPosX: ${minPosX}, maxPosY: ${maxPosY}, minPosY: ${minPosY}`);

        if (value.position.x * 1 > maxPosX)
            maxPosX = value.position.x;
        
        if (value.position.x * 1 < minPosX)
            minPosX = value.position.x;

        if (value.position.y * 1 > maxPosY)
            maxPosY = value.position.y;

        if (value.position.y * 1 < minPosY)
            minPosY = value.position.y;

    });

    // draw vertexes in map-view
    json.forEach((value, index, array) => {
        var vertexPosX = (value.position.x - minPosX) / (maxPosX - minPosX) * width;
        var vertexPosY = (value.position.y - minPosY) / (maxPosY - minPosY) * height;
        createVertex(view, index, {w: 24, h: 24}, {x: vertexPosX, y: vertexPosY});
    });
}

// create vertex
function createVertex(view, num, size, pos) {
    var vertex = document.createElement('div');
    vertex.id = 'vertex' + num;

    // size
    vertex.style.width = size.w + 'px';
    vertex.style.height = size.h + 'px';

    // position
    vertex.style.position = 'absolute';
    vertex.style.left = (pos.x - size.w / 2) + 'px';
    vertex.style.top = (pos.y*1 - size.h / 2) + 'px';

    // color
    vertex.style.backgroundColor = vertexBackgroundColor;
    vertex.style.color = vertexForegroundColor;

    // border
    vertex.style.borderRadius = '50%';

    // text
    vertex.innerHTML = num;
    vertex.style.textAlign = 'center';

    view.appendChild(vertex);
}
