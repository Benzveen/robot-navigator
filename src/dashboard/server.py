"""FastAPI Dashboard Server — web UI for robot monitoring and control."""

from __future__ import annotations

import asyncio
import json
import os
from pathlib import Path
from typing import Optional

import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
from loguru import logger
from pydantic import BaseModel

# ── App Setup ───────────────────────────────────────────────────

app = FastAPI(title="Robot Navigator Dashboard", version="0.1.0")

STATIC_DIR = Path(__file__).parent / "static"
if STATIC_DIR.exists():
    app.mount("/static", StaticFiles(directory=str(STATIC_DIR)), name="static")

# Global references (set by main.py)
_agent = None
_bridge = None


def set_agent(agent) -> None:
    global _agent
    _agent = agent


def set_bridge(bridge) -> None:
    global _bridge
    _bridge = bridge


# ── WebSocket Manager ───────────────────────────────────────────

class ConnectionManager:
    """Manages WebSocket connections for real-time updates."""

    def __init__(self):
        self.active_connections: list[WebSocket] = []

    async def connect(self, websocket: WebSocket) -> None:
        await websocket.accept()
        self.active_connections.append(websocket)
        logger.info(f"Dashboard client connected. Total: {len(self.active_connections)}")

    def disconnect(self, websocket: WebSocket) -> None:
        self.active_connections.remove(websocket)
        logger.info(f"Dashboard client disconnected. Total: {len(self.active_connections)}")

    async def broadcast(self, message: dict) -> None:
        """Send a message to all connected clients."""
        disconnected = []
        for connection in self.active_connections:
            try:
                await connection.send_json(message)
            except Exception:
                disconnected.append(connection)

        for conn in disconnected:
            self.active_connections.remove(conn)


manager = ConnectionManager()


# ── REST Endpoints ──────────────────────────────────────────────

@app.get("/", response_class=HTMLResponse)
async def dashboard():
    """Serve the main dashboard page."""
    html_path = STATIC_DIR / "index.html"
    if html_path.exists():
        return HTMLResponse(content=html_path.read_text())
    return HTMLResponse(content="<h1>Robot Navigator Dashboard</h1><p>Static files not found.</p>")


@app.get("/api/state")
async def get_state():
    """Get current robot state."""
    if _bridge:
        return await _bridge.get_state()
    return {"error": "Bridge not initialized"}


@app.get("/api/map")
async def get_map_info():
    """Get map information."""
    if _bridge:
        return _bridge.map_manager.get_map_info()
    return {"error": "Bridge not initialized"}


class CommandRequest(BaseModel):
    message: str


@app.post("/api/command")
async def send_command(request: CommandRequest):
    """Send a natural language command to the agent."""
    if not _agent:
        return {"error": "Agent not initialized"}

    response = await _agent.send_command(request.message)
    return {"response": response}


@app.post("/api/stop")
async def emergency_stop():
    """Emergency stop the robot."""
    if _bridge:
        result = await _bridge.execute_tool("stop", {})
        return result
    return {"error": "Bridge not initialized"}


@app.post("/api/explore")
async def start_exploration():
    """Start autonomous exploration."""
    if _agent:
        response = await _agent.send_command("Start exploring the area systematically. Map all rooms.")
        return {"response": response}
    return {"error": "Agent not initialized"}


@app.post("/api/go-home")
async def go_home():
    """Navigate robot to home position."""
    if _bridge:
        result = await _bridge.execute_tool("go_home", {})
        return result
    return {"error": "Bridge not initialized"}


# ── WebSocket Endpoint ──────────────────────────────────────────

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    """WebSocket for real-time robot state streaming."""
    await manager.connect(websocket)

    try:
        while True:
            # Receive commands from client
            try:
                data = await asyncio.wait_for(websocket.receive_text(), timeout=0.1)
                message = json.loads(data)

                if message.get("type") == "command" and _agent:
                    response = await _agent.send_command(message.get("text", ""))
                    await websocket.send_json({"type": "response", "text": response})

                elif message.get("type") == "manual_control" and _bridge:
                    linear = message.get("linear", 0)
                    angular = message.get("angular", 0)
                    # Direct velocity command for joystick control
                    await _bridge.execute_tool("set_speed", {"speed": abs(linear)})

            except asyncio.TimeoutError:
                pass

            # Send state update
            if _bridge:
                state = await _bridge.get_state()
                await websocket.send_json({"type": "state", "data": state})

            await asyncio.sleep(0.1)  # 10 Hz update rate

    except WebSocketDisconnect:
        manager.disconnect(websocket)
    except Exception as e:
        logger.error(f"WebSocket error: {e}")
        manager.disconnect(websocket)


# ── State Broadcasting ──────────────────────────────────────────

async def broadcast_loop() -> None:
    """Continuously broadcast robot state to all WebSocket clients."""
    while True:
        if _bridge and manager.active_connections:
            try:
                state = await _bridge.get_state()
                await manager.broadcast({"type": "state", "data": state})
            except Exception as e:
                logger.error(f"Broadcast error: {e}")
        await asyncio.sleep(0.1)


# ── Server Entry Point ──────────────────────────────────────────

def main(host: str = "0.0.0.0", port: int = 8080) -> None:
    """Run the dashboard server."""
    logger.info(f"🌐 Dashboard starting at http://{host}:{port}")
    uvicorn.run(app, host=host, port=port, log_level="info")


if __name__ == "__main__":
    main()
