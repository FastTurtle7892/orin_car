#!/usr/bin/env python3
import os
import json
import asyncio
import cv2
import requests
import websockets
import time
from typing import Optional

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
WEBRTC_PORT = str(os.environ.get("WEBRTC_PORT", "443"))

LOGIN_EMAIL = os.environ.get("LOGIN_EMAIL", "pilot@atc.com")
LOGIN_PW    = os.environ.get("LOGIN_PW", "1234")

CAR_ID   = os.environ.get("CAR_ID", "TC01")
PILOT_ID = os.environ.get("PILOT_ID", "PILOT_001")

TLS_VERIFY = os.environ.get("WEBRTC_TLS_VERIFY", "1").strip().lower() not in ("0", "false", "no")

ICE_GATHER_WAIT = float(os.environ.get("WEBRTC_ICE_GATHER_WAIT", "1.5"))

SCHEME = os.environ.get("WEBRTC_SCHEME", "").strip().lower()
if not SCHEME:
    SCHEME = "https" if WEBRTC_PORT in ("443", "8443") else "http"
WS_SCHEME = "wss" if SCHEME == "https" else "ws"

LOGIN_URL = f"{SCHEME}://{WEBRTC_HOST}:{WEBRTC_PORT}/api/auth/login"
SERVER_WS_BASE = f"{WS_SCHEME}://{WEBRTC_HOST}:{WEBRTC_PORT}/ws-server/websocket"

ICE_SERVERS = [
    RTCIceServer(urls=["turn:i14a402.p.ssafy.io:8000"], username="pilot@atc.com", credential="1234"),
    RTCIceServer(urls=["stun:stun.l.google.com:19302"]),
]
RTC_CONFIG = RTCConfiguration(iceServers=ICE_SERVERS)


# -------------------------------
# Logging
# -------------------------------
def log(*args):
    ts = time.strftime("%H:%M:%S")
    print(f"[{ts}]", *args, flush=True)


# -------------------------------
# Helpers
# -------------------------------
def login_and_get_token() -> Optional[str]:
    log(f"[Login] POST {LOGIN_URL} as {LOGIN_EMAIL}")
    try:
        resp = requests.post(
            LOGIN_URL,
            json={"email": LOGIN_EMAIL, "password": LOGIN_PW},
            timeout=5,
            verify=TLS_VERIFY,
        )
        log(f"[Login] status={resp.status_code}")
        if resp.status_code != 200:
            log(f"[Login] Failed body={resp.text[:200]}")
            return None
        token = resp.json().get("socketToken")
        if not token:
            log("[Login] No socketToken in response.")
            return None
        log("[Login] Success! socketToken acquired.")
        return token
    except Exception as e:
        log(f"[Login] Error: {e}")
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
# cam_manager.get_latest_frame(copy=True) 가 ndarray(BGR) 리턴한다고 가정
# -------------------------------
class CameraStreamTrack(VideoStreamTrack):
    def __init__(self, cam_manager):
        super().__init__()
        self.cam = cam_manager

    async def recv(self):
        # ✅ 여기서는 "멈춤" 로직을 track 안에 두지 않음
        # (우리는 PC 자체를 끊었다 켰다 할 거라서)
        pts, time_base = await self.next_timestamp()

        frame = self.cam.get_latest_frame(copy=True)
        while frame is None:
            await asyncio.sleep(0.01)
            frame = self.cam.get_latest_frame(copy=True)

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        vf = VideoFrame.from_ndarray(rgb, format="rgb24")
        vf.pts = pts
        vf.time_base = time_base
        return vf


# -------------------------------
# Main entry
# -------------------------------
async def webrtc_main(cam_manager):
    token = login_and_get_token()
    if not token:
        log("[WebRTC] Login failed -> abort webrtc_main")
        return

    ws_url = f"{SERVER_WS_BASE}?socket_token={token}"
    log("==== WebRTC Sender (Reconnect-on-START) ====")
    log(f"WS URL: {SERVER_WS_BASE}?socket_token=***")

    send_q: asyncio.Queue[str] = asyncio.Queue()
    loop = asyncio.get_running_loop()

    pc: Optional[RTCPeerConnection] = None
    track: Optional[CameraStreamTrack] = None
    offer_seq = 0

    def enqueue_send(msg: str):
        asyncio.run_coroutine_threadsafe(send_q.put(msg), loop)

    async def ws_sender(ws):
        while True:
            msg = await send_q.get()
            if msg is None:
                return
            await ws.send(msg)

    async def close_pc(reason: str):
        nonlocal pc, track
        if pc is None:
            log(f"[WebRTC] close_pc({reason}) but pc=None -> skip")
            return
        try:
            log(f"[WebRTC] closing PC... reason={reason} state={pc.connectionState} ice={pc.iceConnectionState} sig={pc.signalingState}")
            await pc.close()
        except Exception as e:
            log(f"[WebRTC] pc.close error: {e}")
        pc = None
        track = None
        log("[WebRTC] PC closed.")

    async def create_and_offer():
        """
        ✅ 무조건 새 PeerConnection 만들고 새 Offer 송신
        """
        nonlocal pc, track, offer_seq

        # 이미 있으면 먼저 닫고 새로
        if pc is not None:
            await close_pc("recreate_before_offer")

        offer_seq += 1
        my_seq = offer_seq

        pc = RTCPeerConnection(RTC_CONFIG)
        track = CameraStreamTrack(cam_manager)
        pc.addTrack(track)

        # 상태 로그
        @pc.on("iceconnectionstatechange")
        async def _ice_state():
            log(f"[WebRTC#{my_seq}] iceConnectionState={pc.iceConnectionState}")

        @pc.on("connectionstatechange")
        async def _conn_state():
            log(f"[WebRTC#{my_seq}] connectionState={pc.connectionState}")

        @pc.on("signalingstatechange")
        async def _sig_state():
            log(f"[WebRTC#{my_seq}] signalingState={pc.signalingState}")

        # ICE 후보 송신
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
            enqueue_send(frame)

        log(f"[WebRTC#{my_seq}] createOffer...")
        offer = await pc.createOffer()
        await pc.setLocalDescription(offer)

        log(f"[WebRTC#{my_seq}] localDescription set. wait ICE gather {ICE_GATHER_WAIT}s")
        await asyncio.sleep(ICE_GATHER_WAIT)

        payload = {
            "type": "OFFER",
            "sdp": pc.localDescription.sdp,
            "senderId": CAR_ID,
            "receiverId": PILOT_ID,
            "seq": my_seq,  # 디버그용(서버가 무시해도 OK)
        }
        await send_q.put(
            stomp_frame(
                "SEND",
                {"destination": "/app/video/offer", "content-type": "application/json"},
                json.dumps(payload),
            )
        )
        log(f"[WebRTC#{my_seq}] OFFER sent to {PILOT_ID}")

    async def handle_answer(data: dict):
        nonlocal pc
        if pc is None:
            log("[WebRTC] ANSWER received but pc=None -> ignore")
            return
        sdp = data.get("sdp")
        if not sdp:
            log("[WebRTC] ANSWER has no sdp -> ignore")
            return

        # 상태가 이상하면 로그만 남기고 무시(재연결 방식이라 굳이 억지로 안 맞춤)
        if pc.signalingState != "have-local-offer":
            log(f"[WebRTC] ANSWER in state={pc.signalingState} -> ignore (will reconnect on next START)")
            return

        log("[WebRTC] setRemoteDescription(ANSWER)")
        try:
            await pc.setRemoteDescription(RTCSessionDescription(sdp=sdp, type="answer"))
            log("[WebRTC] ANSWER applied OK")
        except Exception as e:
            log(f"[WebRTC] setRemoteDescription error: {e}")

    async def handle_ice(data: dict):
        nonlocal pc
        if pc is None:
            return
        try:
            cand_sdp = data.get("candidate")
            if not cand_sdp:
                return
            cand = candidate_from_sdp(cand_sdp)
            cand.sdpMid = data.get("sdpMid", "0")
            cand.sdpMLineIndex = int(data.get("sdpMLineIndex", 0))
            await pc.addIceCandidate(cand)
        except Exception as e:
            log(f"[WebRTC] addIceCandidate error: {e}")

    # -------------------------------
    # STOMP session
    # -------------------------------
    async with websockets.connect(ws_url, subprotocols=["v12.stomp"]) as ws:
        sender_task = asyncio.create_task(ws_sender(ws))

        # CONNECT
        await send_q.put(stomp_frame("CONNECT", {"accept-version": "1.2", "host": "localhost"}))
        first = await ws.recv()
        log("[STOMP] first=", first[:80].replace("\n", "\\n"))
        if "CONNECTED" in first:
            log("[STOMP] CONNECTED")

        # SUBSCRIBE
        await send_q.put(stomp_frame("SUBSCRIBE", {"id": "sub-ctrl", "destination": f"/topic/video/control/{CAR_ID}"}))
        await send_q.put(stomp_frame("SUBSCRIBE", {"id": "sub-ans",  "destination": f"/topic/video/answer/{CAR_ID}"}))
        await send_q.put(stomp_frame("SUBSCRIBE", {"id": "sub-ice",  "destination": f"/topic/video/ice/{CAR_ID}"}))
        log("[STOMP] SUBSCRIBED control/answer/ice")

        async for raw in ws:
            data = parse_stomp_message(raw)
            if not data:
                continue

            t = data.get("type")
            log(f"[STOMP] recv type={t} payload_keys={list(data.keys())}")

            # ✅ 너가 원하는 방식:
            # START(또는 RESUME) -> 무조건 새 PC + 새 Offer
            # PAUSE/STOP -> PC 닫기
            if t in ("START", "RESUME"):
                await create_and_offer()

            elif t in ("PAUSE", "STOP"):
                await close_pc(t)

            elif t == "ANSWER":
                await handle_answer(data)

            elif t == "ICE":
                await handle_ice(data)

        # 종료
        await send_q.put(None)
        await sender_task

    await close_pc("ws_end")
    log("[WebRTC] webrtc_main end")
