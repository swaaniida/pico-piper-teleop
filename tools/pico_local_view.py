#!/usr/bin/env python3
"""Serve the wrist camera as a low-latency full-screen MJPEG view for PICO."""

from __future__ import annotations

import argparse
import http.server
import ssl
import socketserver
import threading
import time

import cv2


HTML = b"""<!doctype html>
<html><head><meta name="viewport" content="width=device-width,initial-scale=1">
<title>PiPER local view</title><style>
html,body{margin:0;width:100%;height:100%;overflow:hidden;background:#000}
img,canvas{position:fixed;inset:0;width:100%;height:100%;object-fit:contain;background:#000}
#enter{position:fixed;left:50%;bottom:10%;transform:translateX(-50%);z-index:3;
font:700 28px sans-serif;padding:18px 34px;border:0;border-radius:12px;color:#fff;background:#1769e0}
#status{position:fixed;top:20px;left:50%;transform:translateX(-50%);z-index:3;color:#fff;
font:600 20px sans-serif;background:#000b;padding:8px 14px;border-radius:8px;white-space:nowrap}
</style></head><body><img id="video" src="/stream.mjpg"><canvas id="gl"></canvas>
<div id="status">LOCAL CAMERA</div><button id="enter">ENTER VR</button>
<script>
const status=document.getElementById('status'), button=document.getElementById('enter');
const image=document.getElementById('video'), canvas=document.getElementById('gl');
const gl=canvas.getContext('webgl',{antialias:false,alpha:false});
function shader(type,src){const s=gl.createShader(type);gl.shaderSource(s,src);gl.compileShader(s);return s}
const program=gl.createProgram();
gl.attachShader(program,shader(gl.VERTEX_SHADER,'attribute vec2 p;varying vec2 uv;void main(){uv=vec2((p.x+1.0)*.5,1.0-(p.y+1.0)*.5);gl_Position=vec4(p,0.,1.);}'));
gl.attachShader(program,shader(gl.FRAGMENT_SHADER,'precision mediump float;varying vec2 uv;uniform sampler2D tex;void main(){gl_FragColor=texture2D(tex,uv);}'));
gl.linkProgram(program);gl.useProgram(program);
const buf=gl.createBuffer();gl.bindBuffer(gl.ARRAY_BUFFER,buf);gl.bufferData(gl.ARRAY_BUFFER,new Float32Array([-1,-1,1,-1,-1,1,-1,1,1,-1,1,1]),gl.STATIC_DRAW);
const p=gl.getAttribLocation(program,'p');gl.enableVertexAttribArray(p);gl.vertexAttribPointer(p,2,gl.FLOAT,false,0,0);
const tex=gl.createTexture();gl.bindTexture(gl.TEXTURE_2D,tex);gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_MIN_FILTER,gl.LINEAR);gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_MAG_FILTER,gl.LINEAR);gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_WRAP_S,gl.CLAMP_TO_EDGE);gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_WRAP_T,gl.CLAMP_TO_EDGE);
function upload(){if(image.complete&&image.naturalWidth){gl.bindTexture(gl.TEXTURE_2D,tex);gl.texImage2D(gl.TEXTURE_2D,0,gl.RGB,gl.RGB,gl.UNSIGNED_BYTE,image)}}
async function enterVR(){
 try{
  if(!navigator.xr)throw Error('WebXR is unavailable in this browser');
  const session=await navigator.xr.requestSession('immersive-vr');
  gl.makeXRCompatible&&await gl.makeXRCompatible();
  session.updateRenderState({baseLayer:new XRWebGLLayer(session,gl)});
  button.style.display='none';status.style.display='none';image.style.display='none';
  const frame=(t,f)=>{const layer=session.renderState.baseLayer;gl.bindFramebuffer(gl.FRAMEBUFFER,layer.framebuffer);upload();for(const view of f.getViewerPose((window.xrRef))?.views||[]){const v=layer.getViewport(view);gl.viewport(v.x,v.y,v.width,v.height);gl.drawArrays(gl.TRIANGLES,0,6)}session.requestAnimationFrame(frame)};
  window.xrRef=await session.requestReferenceSpace('viewer');session.requestAnimationFrame(frame);
 }catch(e){status.textContent=e.message;}
}
button.onclick=enterVR;
navigator.xr?.isSessionSupported('immersive-vr').then(ok=>{if(!ok)status.textContent='Immersive VR not supported'});
</script>
</body></html>"""


class Camera:
    def __init__(self, device: str, width: int, height: int, fps: int):
        self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not self.cap.isOpened():
            raise RuntimeError(f"cannot open camera: {device}")
        self.frame: bytes | None = None
        self.condition = threading.Condition()
        threading.Thread(target=self._capture, daemon=True).start()

    def _capture(self) -> None:
        while True:
            ok, image = self.cap.read()
            if not ok:
                time.sleep(0.01)
                continue
            ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 82])
            if ok:
                with self.condition:
                    self.frame = encoded.tobytes()
                    self.condition.notify_all()


class Server(socketserver.ThreadingMixIn, http.server.HTTPServer):
    daemon_threads = True


def handler(camera: Camera):
    class Handler(http.server.BaseHTTPRequestHandler):
        def do_GET(self) -> None:
            if self.path in ("/", "/index.html"):
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(HTML)))
                self.end_headers()
                self.wfile.write(HTML)
                return
            if self.path != "/stream.mjpg":
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            try:
                while True:
                    with camera.condition:
                        camera.condition.wait(timeout=1)
                        frame = camera.frame
                    if frame is None:
                        continue
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(frame)}\r\n\r\n".encode())
                    self.wfile.write(frame + b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                pass

        def log_message(self, fmt: str, *args: object) -> None:
            print(f"{self.client_address[0]} - {fmt % args}", flush=True)

    return Handler


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", default="/dev/video4")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--cert")
    parser.add_argument("--key")
    args = parser.parse_args()
    camera = Camera(args.device, args.width, args.height, args.fps)
    with Server(("0.0.0.0", args.port), handler(camera)) as server:
        if args.cert and args.key:
            context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
            context.load_cert_chain(args.cert, args.key)
            server.socket = context.wrap_socket(server.socket, server_side=True)
        print(f"PICO local view listening on 0.0.0.0:{args.port}", flush=True)
        server.serve_forever()


if __name__ == "__main__":
    main()
