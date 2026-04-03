import httpx
import logging


import json
import base64


logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)

ORION_URL = "http://localhost:1026"
ENTITY_TYPE = "BottleDetectionJob"
ENTITY_ID = "BottleDetectionJob:processor-01"  # single fixed entity
HEADERS = {
    "Content-Type": "application/json",
    "Fiware-Service": "openiot",
    "Fiware-Servicepath": "/",
}


def ensure_entity():
    raw_json = json.dumps({"status": "IDLE", "pick_pose": None})
    encoded_json = base64.b64encode(raw_json.encode()).decode().rstrip('=')
    entity = {
        "id":           ENTITY_ID,
        "type":         ENTITY_TYPE,
        "jobId":        {"type": "Text",    "value": ""},
        "status":       {"type": "Text",    "value": "IDLE"},
        "bottleCount":  {"type": "Integer", "value": 0},
        "pickX":        {"type": "Float",   "value": 0.0},
        "pickY":        {"type": "Float",   "value": 0.0},
        "pickRotation": {"type": "Float",   "value": 0.0},
        "error":        {"type": "Text",    "value": ""},
        "command":      {"type": "Text",    "value": ""},
        "json":         {"type": "Text",    "value": encoded_json}
    }
    try:
        r = httpx.post(
            f"{ORION_URL}/v2/entities?options=upsert",
            json=entity, headers=HEADERS, timeout=5
        )
        logger.info("FIWARE ensure_entity → %s", r.status_code)
    except Exception as e:
        logger.warning("FIWARE ensure_entity failed: %s", e)


def ensure_command_subscription(api_host: str = "http://localhost:22001"):
    """Clean up old subscriptions and register a fresh one with current host IP."""
    api_host = f"http://{api_host}:22001"
    headers = {
        "fiware-service": "openiot",
        "fiware-servicepath": "/",
    }
    sub_description = "BottleDetectionJob command trigger"

    try:
        # 1. Get all subscriptions
        r = httpx.get(f"{ORION_URL}/v2/subscriptions",
                      headers=headers, timeout=5)
        r.raise_for_status()
        subs = r.json()

        # 2. Find and remove the old one
        for sub in subs:
            if sub.get("description") == sub_description:
                sub_id = sub.get("id")
                logger.info(f"Cleaning up old subscription: {sub_id}")
                httpx.delete(
                    f"{ORION_URL}/v2/subscriptions/{sub_id}", headers=headers, timeout=5)

    except Exception as e:
        logger.warning("FIWARE cleanup failed or timed out: %s", e)
        # Continue to creation of the new subscription

    # 3. Create a new subscription with the correct HOST IP
    payload = {
        "description": sub_description,
        "subject": {
            "entities": [{"id": ENTITY_ID, "type": ENTITY_TYPE}],
            "condition": {"attrs": ["command"]}
        },
        "notification": {
            "http": {"url": f"{api_host}/api/v1/fiware/notify"},
            "attrs": ["command"]
        }
    }

    try:
        r = httpx.post(f"{ORION_URL}/v2/subscriptions", json=payload,
                       headers={**headers, "Content-Type": "application/json"}, timeout=5)
        logger.info("FIWARE fresh subscription created → %s", r.status_code)
    except Exception as e:
        logger.error("FIWARE ensure_command_subscription failed: %s", e)


def update_job_status(job_id: str, status: str, bottle_count: int = 0, error: str = "", pick_pose=None):
    raw_json = json.dumps({"status": status, "pick_pose": pick_pose})
    encoded_json = base64.b64encode(raw_json.encode()).decode().rstrip('=')
    attrs = {
        "jobId":        {"type": "Text",    "value": job_id},
        "status":       {"type": "Text",    "value": status},
        "bottleCount":  {"type": "Integer", "value": bottle_count},
        "pickX":        {"type": "Float",   "value": pick_pose["x"] if pick_pose else 0.0},
        "pickY":        {"type": "Float",   "value": pick_pose["y"] if pick_pose else 0.0},
        "pickRotation": {"type": "Float",   "value": pick_pose["rotation"] if pick_pose else 0.0},
        "error":        {"type": "Text",    "value": error},
        "json":         {"type": "Text",    "value": encoded_json}
    }
    try:
        url = f"{ORION_URL}/v2/entities/{ENTITY_ID}/attrs"
        r = httpx.patch(url, json=attrs, headers=HEADERS, timeout=5)
        logger.info("FIWARE response: %s — %s",
                    r.status_code, r.text)
        r.raise_for_status()
        logger.info("FIWARE [%s → %s] OK", job_id, status)
    except Exception as e:
        logger.warning(
            "FIWARE update_job_status failed (job=%s status=%s): %s", job_id, status, e)


def reset_command():
    attrs = {"status":  {"type": "Text", "value": "ACCEPTED"},
             "command": {"type": "Text", "value": ""}}
    try:
        url = f"{ORION_URL}/v2/entities/{ENTITY_ID}/attrs"
        r = httpx.patch(url, json=attrs, headers=HEADERS, timeout=5)
        r.raise_for_status()
    except Exception as e:
        logger.warning("FIWARE reset_command failed: %s", e)
