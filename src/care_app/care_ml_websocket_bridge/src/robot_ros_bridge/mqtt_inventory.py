#!/usr/bin/env python3
import asyncio
import json
import logging
from typing import Any, Dict

logger = logging.getLogger(__name__)


async def mqtt_inventory_listener(node, config: Dict[str, Any]) -> None:
    """
    Subscribes to care/weight/<location_id>/<sku_id> MQTT topics and feeds
    weight readings directly into system_state, bypassing ROS entirely.
    Reconnects automatically on broker disconnect.

    Payload accepted: raw float (grams) or JSON {"grams": <float>}
    """
    try:
        import aiomqtt
    except ImportError:
        logger.error("aiomqtt not installed — MQTT inventory disabled. Run: pip install aiomqtt")
        return

    broker_host = config.get("mqtt_broker_host", "localhost")
    broker_port = int(config.get("mqtt_broker_port", 1883))
    topic_prefix = config.get("mqtt_topic_prefix", "care/weight")
    shelf_config = config.get("mqtt_shelf_config", {})

    logger.info(f"MQTT inventory listener connecting to {broker_host}:{broker_port}")

    while True:
        try:
            async with aiomqtt.Client(broker_host, port=broker_port) as client:
                await client.subscribe(f"{topic_prefix}/+/+")
                logger.info(f"MQTT subscribed to {topic_prefix}/+/+")

                async for message in client.messages:
                    await _handle_weight_message(
                        node=node,
                        topic=str(message.topic),
                        payload=message.payload,
                        topic_prefix=topic_prefix,
                        shelf_config=shelf_config,
                    )
        except Exception as e:
            logger.error(f"MQTT error: {e} — retrying in 5s")
            await asyncio.sleep(5)


async def _handle_weight_message(
    node,
    topic: str,
    payload: bytes,
    topic_prefix: str,
    shelf_config: Dict[str, Any],
) -> None:
    try:
        # Topic: <prefix>/<node_id>/<sku_id>
        suffix = topic.removeprefix(topic_prefix).strip("/")
        parts = suffix.split("/")
        if len(parts) != 2:
            logger.warning(f"Unexpected MQTT topic structure: {topic}")
            return

        node_id, sku_id = parts

        # Parse payload — raw float or JSON {"grams": <float>}
        raw = payload.decode("utf-8").strip()
        try:
            weight_g = float(raw)
        except ValueError:
            weight_g = float(json.loads(raw)["grams"])

        loc_cfg = shelf_config.get(node_id, {})
        sku_cfg = loc_cfg.get("skus", {}).get(sku_id, {})
        location_name = loc_cfg.get("name", node_id)

        weight_per_unit_g = float(sku_cfg.get("weight_per_unit_g", 0.0))
        par_level = sku_cfg.get("par_level", 0)
        reorder_point = sku_cfg.get("reorder_point", 0)
        max_capacity = sku_cfg.get("max_capacity", 0)
        category = sku_cfg.get("category", "unknown")

        if weight_per_unit_g > 0:
            stock_level = round(weight_g / weight_per_unit_g)
        else:
            logger.warning(f"No weight_per_unit_g for {node_id}/{sku_id} — using raw grams")
            stock_level = weight_g

        node.system_state.update_location_inventory(
            location_id=node_id,
            location_name=location_name,
            sku_inventory={
                sku_id: {
                    "stock_level": stock_level,
                    "max_level": max_capacity,
                    "par_level": par_level,
                    "reorder_point": reorder_point,
                    "category": category,
                    "weight_g": round(weight_g, 2),
                }
            },
            category_inventory={
                category: {
                    "total_stock": stock_level,
                    "max_stock": max_capacity,
                    "num_skus": 1,
                }
            },
        )

        from .message_converter import location_inventory_to_json
        node.outgoing_queue.put((
            "location_inventory",
            location_inventory_to_json(node.system_state.get_all_location_inventories()),
        ))

        logger.debug(
            f"Weight: {node_id}/{sku_id} = {weight_g}g → {stock_level} units "
            f"(par={par_level}, reorder={reorder_point})"
        )

    except Exception as e:
        logger.error(f"Failed to handle MQTT message on {topic}: {e}")
