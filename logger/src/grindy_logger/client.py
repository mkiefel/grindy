"""WebSocket client for connecting to Grindy microcontroller."""

import asyncio
import signal
from typing import Callable, Optional

import websockets
from websockets.exceptions import WebSocketException

from .models import WsMessage, parse_message


class GrindyWebSocketClient:
    """Async WebSocket client with auto-reconnection."""

    def __init__(
        self,
        url: str,
        on_message: Callable[[WsMessage], None],
        reconnect_delay: float = 1.0,
    ):
        """
        Initialize WebSocket client.

        Args:
            url: WebSocket URL (e.g., ws://192.168.22.1/ws)
            on_message: Callback function for each received message.
            reconnect_delay: Seconds to wait before reconnecting after disconnect.
        """
        self.url = url
        self.on_message = on_message
        self.reconnect_delay = reconnect_delay
        self.running = False
        self._shutdown_event = asyncio.Event()

    async def connect(self) -> None:
        """
        Connect to WebSocket and process messages.

        This method handles reconnection automatically on disconnect.
        """
        self.running = True
        retry_count = 0

        while self.running:
            try:
                print(f"Connecting to {self.url}...")
                async with websockets.connect(
                    self.url, subprotocols=["grindy"]
                ) as websocket:
                    print(f"Connected to {self.url}")
                    retry_count = 0  # Reset on successful connection

                    while self.running:
                        try:
                            # Wait for message or shutdown
                            message = await asyncio.wait_for(
                                websocket.recv(), timeout=1.0
                            )

                            # Parse binary message
                            if isinstance(message, bytes):
                                try:
                                    parsed = parse_message(message)
                                    self.on_message(parsed)
                                except ValueError as e:
                                    print(f"Failed to parse message: {e}")
                                    print(
                                        f"Raw bytes ({len(message)}): {message[:100].hex()}..."
                                    )
                            else:
                                print(f"Unexpected text message: {message}")

                        except asyncio.TimeoutError:
                            # Timeout is normal - allows checking shutdown event
                            if self._shutdown_event.is_set():
                                self.running = False
                                break
                            continue

            except WebSocketException as e:
                if self.running:
                    retry_count += 1
                    print(f"WebSocket error (attempt {retry_count}): {e}")
                    print(f"Reconnecting in {self.reconnect_delay} seconds...")
                    await asyncio.sleep(self.reconnect_delay)

        print("WebSocket client stopped")

    def shutdown(self) -> None:
        """Signal the client to shutdown gracefully."""
        print("Shutting down WebSocket client...")
        self.running = False
        self._shutdown_event.set()


async def run_client(
    url: str,
    on_message: Callable[[WsMessage], None],
    shutdown_event: Optional[asyncio.Event] = None,
) -> None:
    """
    Run the WebSocket client until shutdown.

    Args:
        url: WebSocket URL
        on_message: Callback for messages
        shutdown_event: Optional event to signal shutdown
    """
    client = GrindyWebSocketClient(url, on_message)

    # Setup signal handlers for graceful shutdown
    def signal_handler():
        client.shutdown()

    if shutdown_event is None:
        shutdown_event = asyncio.Event()

    loop = asyncio.get_event_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, signal_handler)

    try:
        await client.connect()
    finally:
        # Cleanup signal handlers
        for sig in (signal.SIGINT, signal.SIGTERM):
            loop.remove_signal_handler(sig)
