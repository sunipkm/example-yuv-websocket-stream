let stream = true;
function initStream() {

    const loadingMessage = document.getElementById("loadingMessage");
    const c = document.getElementById("streamdata");

    const ctx = c.getContext("2d");

    if ("WebSocket" in window) {
        let proto = "wss";
        if (window.location.protocol === "http:") {
            proto = "ws";
        }

        let ws = new WebSocket(proto + "://" + window.location.hostname + ":" + window.location.port + "/v0/cam/stream/ws");
        ws.binaryType = "arraybuffer";

        ws.onopen = function () {
        };

        ws.onmessage = function (evt) {
            const received_msg = new Uint8Array(evt.data);
            console.info("Received: " + received_msg.length + " bytes -> " + received_msg.slice(0, 10))

            if (received_msg.length > 0 && stream === true) {
                loadingMessage.hidden = true;

                var b64encoded = '';
                var len = received_msg.length;
                for (var i = 0; i < len; i++) {
                    b64encoded += String.fromCharCode(received_msg[i]);
                }
                // console.info(b64encoded);
                var datajpg = "data:image/jpg;base64," + btoa(b64encoded);
                // console.info(datajpg);
                const img = new Image();
                img.src = datajpg;
                img.onload = () => {
                    wid = window.innerWidth;
                    hei = window.innerHeight;
                    iwid = img.width;
                    ihei = img.height;
                    scale = Math.min(wid / iwid, hei / ihei);
                    console.log("scale: " + scale + " wid: " + wid + " hei: " + hei + " iwid: " + iwid + " ihei: " + ihei)
                    ctx.canvas.width = iwid * scale;
                    ctx.canvas.height = ihei * scale;
                    ctx.drawImage(img, 0, 0, iwid * scale, ihei * scale);
                };
                ws.send("Received: " + received_msg.length + " bytes");
            }
        };

        ws.onclose = function () {

        };
    } else {
        loadingMessage.innerText = "Unable to access video stream (please make sure your browser supports canvas and websockets)";
    }
}