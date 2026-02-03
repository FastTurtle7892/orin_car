#!/usr/bin/env python3
import os
import json
import asyncio
import cv2
import requests
import websockets
import logging

from av import VideoFrame
from aiortc import (
    RTCPeerConnection,
    RTCSessionDescription,
    RTCConfiguration,
    RTCIceServer,
    VideoStreamTrack,
)

try:
    from aiortc.sdp import candidate_from_sdp
except Exception:
    from aiortc.rtcicetransport import candidate_from_sdp


# -------------------------------
# Config (ENV)
# -------------------------------
WEBRTC_HOST = os.environ.get("WEBRTC_HOST", "autowingcar.o-r.kr")
WEBRTC_PORT = str(os.environ.get("WEBRTC_PORT", "8443"))

LOGIN_EMAIL = os.environ.get("LOGIN_EMAIL", "pilot@atc.com")
LOGIN_PW    = os.environ.get("LOGIN_PW", "1234")

CAR_ID   = os.environ.get("CAR_ID", "TC01")
PILOT_ID = os.environ.get("PILOT_ID", "PILOT_001")

# TLS verify (https 인증서 검증) : 기본 True
# self-signed / 테스트면 WEBRTC_TLS_VERIFY=0 으로 끄기 가능
TLS_VERIFY = os.environ.get("WEBRTC_TLS_VERIFY", "1").strip().lower() not in ("0", "false", "no")

# START 없이도 바로 offer 만들고 싶으면(디버그용)
AUTO_START = os.environ.get("WEBRTC_AUTO_START", "0") == "1"

# ICE gather wait seconds (aiortc는 trickle도 가능하지만, 최소 대기)
ICE_GATHER_WAIT = float(os.environ.get("WEBRTC_ICE_GATHER_WAIT", "1.5"))

# 포트 기반 스킴 자동
SCHEME = os.environ.get("WEBRTC_SCHEME", "").strip().lower()
if not SCHEME:
    SCHEME = "https" if WEBRTC_PORT in ("443", "8443") else "http"
WS_SCHEME = "wss" if SCHEME == "https" else "ws"

LOGIN_URL = f"{SCHEME}://{WEBRTC_HOST}:{WEBRTC_PORT}/api/auth/login"
SERVER_WS_BASE = f"{WS_SCHEME}://{WEBRTC_HOST}:{WEBRTC_PORT}/ws-server/websocket"


# TURN/STUN (여긴 네가 “동작하던 값”으로 맞추는 게 중요)
ICE_SERVERS = [
    # ✅ 너가 예전에 쓰던 계정/암호로 되돌리는 걸 권장
    RTCIceServer(urls=["turn:i14a402.p.ssafy.io:8000"], username="pilot@atc.com", credential="1234"),
    RTCIceServer(urls=["stun:stun.l.google.com:19302"]),
]
RTC_CONFIG = RTCConfiguration(iceServers=ICE_SERVERS)


# -------------------------------
# Helpers
# -------------------------------
def login_and_get_token():
    print(f"[Login] Trying to login as {LOGIN_EMAIL}... ({LOGIN_URL})")
    try:
        resp = requests.post(
            LOGIN_URL,
            json={"email": LOGIN_EMAIL, "password": LOGIN_PW},
            timeout=5,
            verify=TLS_VERIFY,
        )
        if resp.status_code != 200:
            print(f"[Login] Failed: {resp.status_code} {resp.text}")
            return None
        token = resp.json().get("socketToken")
        if not token:
            print("[Login] No socketToken in response.")
            return None
        print("[Login] Success! Token acquired.")
        return token
    except Exception as e:
        print(f"[Login] Error: {e}")
        return None


def stomp_frame(cmd, headers, body=""):
    s = cmd + "\n"
    for k, v in headers.items():
        s += f"{k}:{v}\n"
    s += "\n" + (body or "") + "\0"
    return s


def parse_stomp_message(frame: str):
    if not frame or "MESSAGE" not in frame:
        return None
    parts = frame.split("\n\n", 1)
    if len(parts) != 2:
        return None
    body = parts[1].replace("\0", "").strip()
    try:
        return json.loads(body)
    except Exception:
        return None


# -------------------------------
# Shared Camera Track
# cam_manager는 get_latest_frame(copy=True) 제공한다고 가정
# -------------------------------
class CameraStreamTrack(VideoStreamTrack):
    def __init__(self, cam_manager):
        super().__init__()
        self.cam = cam_manager
        self._send_event = asyncio.Event()
        self._send_event.clear()  # 기본: STOP 상태 (START 받으면 resume)
        self._fps_cnt = 0
        self._fps_t0 = asyncio.get_event_loop().time()

    def pause(self):
        print("[WebRTC] PAUSE stream")
        self._send_event.clear()

    def resume(self):
        print("[WebRTC] RESUME stream")
        self._send_event.set()

    async def recv(self):
        # START 전까지 대기
        await self._send_event.wait()

        pts, time_base = await self.next_timestamp()

        frame = self.cam.get_latest_frame(copy=True)
        while frame is None:
            await asyncio.sleep(0.01)
            frame = self.cam.get_latest_frame(copy=True)

        # 디버그: 송출 fps 찍기(너무 시끄러우면 주석)
        self._fps_cnt += 1
        now = asyncio.get_event_loop().time()
        if now - self._fps_t0 > 1.0:
            print(f"[WebRTC] sending fps ~ {self._fps_cnt}")
            self._fps_cnt = 0
            self._fps_t0 = now

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        vf = VideoFrame.from_ndarray(rgb, format="rgb24")
        vf.pts = pts
        vf.time_base = time_base
        return vf


# -------------------------------
# Main entry for video_stack
# -------------------------------
async def webrtc_main(cam_manager):
    token = login_and_get_token()
    if not token:
        print("[WebRTC] Login failed. Continue without WebRTC.")
        return

    ws_url = f"{SERVER_WS_BASE}?socket_token={token}"
    # 토큰 마스킹
    tmask = token[:16] + "..." if len(token) > 16 else token
    print("==== WebRTC Sender (SharedCam) ====")
    print(f"Server: {SERVER_WS_BASE}?socket_token={tmask}")

    pc = RTCPeerConnection(RTC_CONFIG)
    track = CameraStreamTrack(cam_manager)
    pc.addTrack(track)

    send_q: asyncio.Queue[str] = asyncio.Queue()
    loop = asyncio.get_running_loop()

    # ---- 상태 로그 (진짜 연결 붙는지 확인) ----
    @pc.on("iceconnectionstatechange")
    async def _on_ice_state():
        print("[WebRTC] iceConnectionState =", pc.iceConnectionState)

    @pc.on("connectionstatechange")
    async def _on_conn_state():
        print("[WebRTC] connectionState =", pc.connectionState)

    @pc.on("signalingstatechange")
    async def _on_sig_state():
        print("[WebRTC] signalingState =", pc.signalingState)

    # ---- 차량 ICE 후보도 서버로 송신 (중요!) ----
    # aiortc 콜백은 sync라서 thread-safe enqueue 필요
    def _enqueue_send(msg: str):
        asyncio.run_coroutine_threadsafe(send_q.put(msg), loop)

    @pc.on("icecandidate")
    def _on_icecandidate(event):
        cand = event.candidate
        if cand is None:
            return

        payload = {
            "type": "ICE",
            "candidate": cand.candidate,
            "sdpMid": cand.sdpMid,
            "sdpMLineIndex": cand.sdpMLineIndex,
            "senderId": CAR_ID,
            "receiverId": PILOT_ID,
        }
        frame = stomp_frame(
            "SEND",
            {"destination": "/app/video/ice", "content-type": "application/json"},
            json.dumps(payload),
        )
        _enqueue_send(frame)

    async def ws_sender(ws):
        while True:
            msg = await send_q.get()
            if msg is None:
                return
            await ws.send(msg)

    offer_sent = False
    answered = False
    last_answer_sdp = None

    async def start_streaming():
        nonlocal offer_sent
        if offer_sent:
            print("[WebRTC] START received but offer already sent -> ignore")
            return

        print("[WebRTC] START received -> create offer")
        track.resume()

        offer = await pc.createOffer()
        await pc.setLocalDescription(offer)

        # (선택) 약간 기다려서 ICE가 SDP에 어느 정도 붙도록
        await asyncio.sleep(ICE_GATHER_WAIT)

        payload = {
            "type": "OFFER",
            "sdp": pc.localDescription.sdp,
            "senderId": CAR_ID,
            "receiverId": PILOT_ID,
        }
        await send_q.put(
            stomp_frame(
                "SEND",
                {"destination": "/app/video/offer", "content-type": "application/json"},
                json.dumps(payload),
            )
        )
        offer_sent = True
        print("[WebRTC] OFFER sent")

    # -------------------------------
    # STOMP session
    # -------------------------------
    async with websockets.connect(ws_url, subprotocols=["v12.stomp"]) as ws:
        sender_task = asyncio.create_task(ws_sender(ws))

        # CONNECT
        await send_q.put(stomp_frame("CONNECT", {"accept-version": "1.2", "host": "localhost"}))
        first = await ws.recv()
        if "CONNECTED" in first:
            print("[STOMP] CONNECTED")

        # SUBSCRIBE topics
        await send_q.put(stomp_frame("SUBSCRIBE", {"id": "sub-ctrl", "destination": f"/topic/video/control/{CAR_ID}"}))
        await send_q.put(stomp_frame("SUBSCRIBE", {"id": "sub-ans",  "destination": f"/topic/video/answer/{CAR_ID}"}))
        await send_q.put(stomp_frame("SUBSCRIBE", {"id": "sub-ice",  "destination": f"/topic/video/ice/{CAR_ID}"}))

        # 디버그: 시작 신호 없이도 바로 송출하고 싶으면
        if AUTO_START:
            await start_streaming()

        async for raw in ws:
            data = parse_stomp_message(raw)
            if not data:
                continue

            t = data.get("type")

            if t == "START":
                await start_streaming()

            elif t in ("PAUSE", "STOP"):
                track.pause()

            elif t == "RESUME":
                track.resume()

            elif t == "ANSWER":
                sdp = data.get("sdp")
                if not sdp:
                    continue

                # ✅ 중복 ANSWER 무시
                if answered:
                    if sdp == last_answer_sdp:
                        print("[WebRTC] ANSWER duplicate (same sdp) -> ignore")
                    else:
                        print("[WebRTC] ANSWER received again (different sdp) -> ignore")
                    continue

                # ✅ 지금 상태가 answer 받을 상태인지 가드
                if pc.signalingState != "have-local-offer":
                    print(f"[WebRTC] ANSWER in state={pc.signalingState} -> ignore")
                    continue

                print("[WebRTC] ANSWER received -> setRemoteDescription")
                await pc.setRemoteDescription(RTCSessionDescription(sdp=sdp, type="answer"))
                answered = True
                last_answer_sdp = sdp

            elif t == "ICE":
                # Pilot -> Car ICE
                try:
                    cand_sdp = data.get("candidate")
                    if not cand_sdp:
                        continue
                    cand = candidate_from_sdp(cand_sdp)
                    cand.sdpMid = data.get("sdpMid", "0")
                    cand.sdpMLineIndex = int(data.get("sdpMLineIndex", 0))
                    await pc.addIceCandidate(cand)
                except Exception as e:
                    print(f"[WebRTC] ICE error: {e}")

        # 종료 처리
        await send_q.put(None)
        await sender_task

    await pc.close()
    print("[WebRTC] closed")


# Standalone run (옵션)
if __name__ == "__main__":
    print("[webrtc_sharedcam] This module is intended to be called by video_stack with a shared cam_manager.")
    print("[webrtc_sharedcam] If you want standalone test, set WEBRTC_AUTO_START=1 and ensure your video_stack provides cam_manager.")
