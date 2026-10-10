#!/usr/bin/env python3
"""
Main Entry Point - Coordinates ROS and FastAPI threads
"""

import rclpy
import threading
import asyncio
import time
import sys
import logging

from .bridge_node import BridgeNode
from .websocket_handler import create_app, set_bridge_node, drain_outgoing_queue

try:
    import uvicorn
except ImportError:
    print("ERROR: uvicorn not installed. Install with: pip install uvicorn")
    sys.exit(1)

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger(__name__)


def start_fastapi_server(node, host: str, port: int) -> None:
    """
    Start FastAPI server in background thread with asyncio event loop

    Args:
        node: Bridge node instance
        host: WebSocket host
        port: WebSocket port
    """
    # Create new event loop for this thread
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    # Set bridge node reference
    set_bridge_node(node)

    # Store event loop reference in node
    node.fastapi_event_loop = loop

    logger.info(f"Starting FastAPI server on {host}:{port}")

    # Create queue drain task
    app = create_app()

    async def run_server():
        """Run server with queue drainer"""
        loop.create_task(drain_outgoing_queue(node))

        if node.config.get("mqtt_enabled"):
            from .mqtt_inventory import mqtt_inventory_listener
            loop.create_task(mqtt_inventory_listener(node, node.config))

        # Run uvicorn
        config = uvicorn.Config(
            app=app,
            host=host,
            port=port,
            loop="asyncio",
            log_level="info",
        )
        server = uvicorn.Server(config)
        await server.serve()

    try:
        loop.run_until_complete(run_server())
    except Exception as e:
        logger.error(f"FastAPI server error: {e}")
    finally:
        loop.close()


def main(args=None):
    """
    Main entry point

    Args:
        args: Command line arguments
    """
    # Initialize ROS
    rclpy.init(args=args)

    # Create bridge node
    node = BridgeNode()
    logger.info("Bridge node created")

    # Get configuration
    host = node.config["websocket_host"]
    port = node.config["websocket_port"]

    # Start FastAPI in background thread
    logger.info(f"Starting FastAPI server thread...")
    fastapi_thread = threading.Thread(
        target=start_fastapi_server,
        args=(node, host, port),
        daemon=True,
    )
    fastapi_thread.start()

    # Wait for event loop to be ready
    logger.info("Waiting for FastAPI event loop...")
    timeout = 10.0
    start_time = time.time()
    while not hasattr(node, "fastapi_event_loop") or node.fastapi_event_loop is None:
        if time.time() - start_time > timeout:
            logger.error("FastAPI event loop failed to initialize")
            node.destroy_node()
            rclpy.shutdown()
            return 1
        time.sleep(0.1)

    logger.info(f"FastAPI event loop ready")
    logger.info(f"Starting ROS spinner in main thread...")

    try:
        # Spin ROS in main thread
        rclpy.spin(node)
    except KeyboardInterrupt:
        logger.info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    sys.exit(main())
