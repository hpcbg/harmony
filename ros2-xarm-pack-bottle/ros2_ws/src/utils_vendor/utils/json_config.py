import json


def load(path="config.json"):
    try:
        with open(path, "r") as f:
            CONFIG = json.load(f)
        print("[CONFIG] Loaded:", CONFIG)
    except Exception as e:
        print("[CONFIG] Error:", e)
        CONFIG = {}
    return CONFIG
